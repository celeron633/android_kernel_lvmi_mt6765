// SPDX-License-Identifier: GPL-2.0
/*
 * PSC5425E charger for S17Promini.
 * Register layout: Prisemi PSC5425E Rev.06.5 and the S17 stock callbacks.
 * Reference: flitsmeister/android-os-kernel hl7005.c, commit
 * ed306e55b1ff9a96980edde7998fdff792b4b04e (MediaTek, 2021).
 */

#include <linux/alarmtimer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pm_wakeup.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "charger_class.h"
#include "mtk_charger.h"

#define PSC_STATUS	0x00
#define PSC_CONTROL	0x01
#define PSC_VOLTAGE	0x02
#define PSC_ID		0x03
#define PSC_CURRENT	0x04
#define PSC_SPECIAL	0x05
#define PSC_DISABLE	BIT(2)
#define PSC_OTG		BIT(0)
#define PSC_HIZ		BIT(1)

/* The stock current thresholds correspond to the 50 milliohm table. */
static const u32 psc_charge_ua[] = {
	656000, 786000, 1048000, 1180000,
	1442000, 1574000, 1836000, 1966000,
};
static const u32 psc_input_ua[] = { 150000, 500000, 800000, U32_MAX };
static const u32 psc_cv_uv[] = { 4100000, 4200000, 4350000, 4400000 };
static const u8 psc_cv_code[] = { 0, 2, 36, 45 };

struct psc5425e {
	struct i2c_client *client;
	struct charger_device *chg;
	struct mutex lock;
	struct mutex otg_lock;
	struct alarm otg_alarm;
	struct work_struct otg_work;
	struct wakeup_source *otg_ws;
	bool otg_enabled;
	bool stopping;
};

static struct psc5425e *psc_info(struct charger_device *chg)
{
	/* charger_dev_set_drvdata is owned by the charging manager. */
	return dev_get_drvdata(&chg->dev);
}

static unsigned int psc_floor(const u32 *table, unsigned int count, u32 value)
{
	unsigned int i;

	for (i = count - 1; i; i--)
		if (value >= table[i])
			break;
	return i;
}

/* Caller holds lock. Never write after a failed read or replay RESET. */
static int psc_update(struct psc5425e *info, u8 reg, u8 mask, u8 value)
{
	int old = i2c_smbus_read_byte_data(info->client, reg);

	if (old < 0)
		return old;
	if (reg == PSC_CURRENT)
		old &= ~BIT(7);
	return i2c_smbus_write_byte_data(info->client, reg,
				      (old & ~mask) | (value & mask));
}

static int psc_enable(struct charger_device *chg, bool enable)
{
	struct psc5425e *info = psc_info(chg);
	int ret;

	mutex_lock(&info->lock);
	if (enable && info->otg_enabled)
		ret = -EBUSY;
	else
		ret = psc_update(info, PSC_CONTROL, PSC_DISABLE,
				 enable ? 0 : PSC_DISABLE);
	mutex_unlock(&info->lock);
	return ret;
}

static int psc_is_enabled(struct charger_device *chg, bool *enabled)
{
	struct psc5425e *info = psc_info(chg);
	int val;

	mutex_lock(&info->lock);
	val = i2c_smbus_read_byte_data(info->client, PSC_CONTROL);
	mutex_unlock(&info->lock);
	if (val < 0)
		return val;
	*enabled = !(val & (PSC_DISABLE | PSC_OTG));
	return 0;
}

static int psc_set_current(struct charger_device *chg, u32 ua)
{
	struct psc5425e *info = psc_info(chg);
	unsigned int code = psc_floor(psc_charge_ua, ARRAY_SIZE(psc_charge_ua), ua);
	int ret;

	if (!ua)
		return psc_enable(chg, false);
	mutex_lock(&info->lock);
	ret = psc_update(info, PSC_CURRENT, 0x70, code << 4);
	mutex_unlock(&info->lock);
	return ret;
}

static int psc_get_current(struct charger_device *chg, u32 *ua)
{
	struct psc5425e *info = psc_info(chg);
	int val;

	mutex_lock(&info->lock);
	val = i2c_smbus_read_byte_data(info->client, PSC_CURRENT);
	mutex_unlock(&info->lock);
	if (val < 0)
		return val;
	*ua = psc_charge_ua[(val >> 4) & 7];
	return 0;
}

static int psc_set_input(struct charger_device *chg, u32 ua)
{
	struct psc5425e *info = psc_info(chg);
	unsigned int code = psc_floor(psc_input_ua, ARRAY_SIZE(psc_input_ua), ua);
	int ret;

	if (!ua)
		return psc_enable(chg, false);
	/* 11 is unlimited, not a programmable 1/2/5 A current limit. */
	mutex_lock(&info->lock);
	ret = psc_update(info, PSC_CONTROL, 0xc0, code << 6);
	mutex_unlock(&info->lock);
	return ret;
}

static int psc_get_input(struct charger_device *chg, u32 *ua)
{
	struct psc5425e *info = psc_info(chg);
	int val;

	mutex_lock(&info->lock);
	val = i2c_smbus_read_byte_data(info->client, PSC_CONTROL);
	mutex_unlock(&info->lock);
	if (val < 0)
		return val;
	*ua = psc_input_ua[(val >> 6) & 3];
	return 0;
}

static int psc_set_cv(struct charger_device *chg, u32 uv)
{
	struct psc5425e *info = psc_info(chg);
	unsigned int index = psc_floor(psc_cv_uv, ARRAY_SIZE(psc_cv_uv), uv);
	int ret;

	if (uv < psc_cv_uv[0])
		return -ERANGE;
	mutex_lock(&info->lock);
	ret = psc_update(info, PSC_SPECIAL, BIT(7), 0);
	if (!ret)
		ret = psc_update(info, PSC_VOLTAGE, 0xfc, psc_cv_code[index] << 2);
	mutex_unlock(&info->lock);
	return ret;
}

static int psc_get_cv(struct charger_device *chg, u32 *uv)
{
	struct psc5425e *info = psc_info(chg);
	int val, special;

	mutex_lock(&info->lock);
	val = i2c_smbus_read_byte_data(info->client, PSC_VOLTAGE);
	special = i2c_smbus_read_byte_data(info->client, PSC_SPECIAL);
	mutex_unlock(&info->lock);
	if (val < 0 || special < 0)
		return val < 0 ? val : special;
	val >>= 2;
	*uv = val < 2 ? 4100000 : val < 36 ? 4200000 :
		val < 45 ? 4350000 : 4400000;
	if (special & BIT(7))
		*uv += 20000;
	return 0;
}

static int psc_done(struct charger_device *chg, bool *done)
{
	struct psc5425e *info = psc_info(chg);
	int val;

	mutex_lock(&info->lock);
	val = i2c_smbus_read_byte_data(info->client, PSC_STATUS);
	mutex_unlock(&info->lock);
	if (val < 0)
		return val;
	*done = ((val >> 4) & 3) == 2;
	return 0;
}

static int psc_kick(struct charger_device *chg)
{
	struct psc5425e *info = psc_info(chg);
	int ret;

	/* Any I2C access services the 32 second OTG watchdog. */
	mutex_lock(&info->lock);
	ret = i2c_smbus_read_byte_data(info->client, PSC_ID);
	mutex_unlock(&info->lock);
	return ret < 0 ? ret : 0;
}

static int psc_dump(struct charger_device *chg)
{
	struct psc5425e *info = psc_info(chg);
	u8 regs[6];
	int i, ret = 0;

	mutex_lock(&info->lock);
	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		ret = i2c_smbus_read_byte_data(info->client, i);
		if (ret < 0)
			break;
		regs[i] = ret;
	}
	mutex_unlock(&info->lock);
	if (ret < 0)
		return ret;
	dev_dbg(&info->client->dev, "registers: %*ph\n", (int)sizeof(regs), regs);
	return 0;
}

static int psc_event(struct charger_device *chg, u32 event, u32 args)
{
	if (event == EVENT_FULL)
		return charger_dev_notify(chg, CHARGER_DEV_NOTIFY_EOC);
	if (event == EVENT_RECHARGE)
		return charger_dev_notify(chg, CHARGER_DEV_NOTIFY_RECHG);
	return 0;
}

static void psc_otg_work(struct work_struct *work)
{
	struct psc5425e *info = container_of(work, struct psc5425e, otg_work);
	int ret;

	mutex_lock(&info->lock);
	if (info->otg_enabled && !info->stopping) {
		ret = i2c_smbus_read_byte_data(info->client, PSC_ID);
		if (ret < 0)
			dev_err_ratelimited(&info->client->dev, "OTG watchdog: %d\n", ret);
		alarm_start_relative(&info->otg_alarm, ktime_set(10, 0));
	}
	mutex_unlock(&info->lock);
	__pm_relax(info->otg_ws);
}

static enum alarmtimer_restart psc_otg_alarm(struct alarm *alarm, ktime_t now)
{
	struct psc5425e *info = container_of(alarm, struct psc5425e, otg_alarm);

	__pm_stay_awake(info->otg_ws);
	schedule_work(&info->otg_work);
	return ALARMTIMER_NORESTART;
}

static int psc_otg(struct charger_device *chg, bool enable)
{
	struct psc5425e *info = psc_info(chg);
	int ret;

	mutex_lock(&info->otg_lock);
	mutex_lock(&info->lock);
	if (info->stopping) {
		mutex_unlock(&info->lock);
		mutex_unlock(&info->otg_lock);
		return -ENODEV;
	}
	ret = psc_update(info, PSC_CONTROL, PSC_OTG | PSC_HIZ,
			 enable ? PSC_OTG : 0);
	if (!ret) {
		info->otg_enabled = enable;
		if (enable)
			alarm_start_relative(&info->otg_alarm, ktime_set(10, 0));
	}
	mutex_unlock(&info->lock);
	/* Serialize enable with cancellation; worker only takes lock. */
	if (!enable && !ret) {
		alarm_cancel(&info->otg_alarm);
		cancel_work_sync(&info->otg_work);
		__pm_relax(info->otg_ws);
	}
	mutex_unlock(&info->otg_lock);
	return ret;
}

static const struct charger_ops psc_ops = {
	.enable = psc_enable,
	.is_enabled = psc_is_enabled,
	.get_charging_current = psc_get_current,
	.set_charging_current = psc_set_current,
	.get_input_current = psc_get_input,
	.set_input_current = psc_set_input,
	.set_constant_voltage = psc_set_cv,
	.get_constant_voltage = psc_get_cv,
	.kick_wdt = psc_kick,
	.is_charging_done = psc_done,
	.dump_registers = psc_dump,
	.event = psc_event,
	.enable_otg = psc_otg,
};

static int psc_vbus_enable(struct regulator_dev *rdev)
{
	struct psc5425e *info = rdev_get_drvdata(rdev);

	return psc_otg(info->chg, true);
}

static int psc_vbus_disable(struct regulator_dev *rdev)
{
	struct psc5425e *info = rdev_get_drvdata(rdev);

	return psc_otg(info->chg, false);
}

static int psc_vbus_enabled(struct regulator_dev *rdev)
{
	struct psc5425e *info = rdev_get_drvdata(rdev);
	int val;

	mutex_lock(&info->lock);
	val = i2c_smbus_read_byte_data(info->client, PSC_CONTROL);
	mutex_unlock(&info->lock);
	return val < 0 ? val : !!(val & PSC_OTG);
}

static const struct regulator_ops psc_vbus_ops = {
	.enable = psc_vbus_enable,
	.disable = psc_vbus_disable,
	.is_enabled = psc_vbus_enabled,
};

static const struct regulator_desc psc_vbus_desc = {
	.name = "usb-otg-vbus",
	.of_match = "usb-otg-vbus",
	.ops = &psc_vbus_ops,
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
	.fixed_uV = 5000000,
	.n_voltages = 1,
};

static void psc_stop(struct psc5425e *info)
{
	mutex_lock(&info->otg_lock);
	mutex_lock(&info->lock);
	info->stopping = true;
	info->otg_enabled = false;
	psc_update(info, PSC_CONTROL, PSC_OTG, 0);
	mutex_unlock(&info->lock);
	alarm_cancel(&info->otg_alarm);
	cancel_work_sync(&info->otg_work);
	__pm_relax(info->otg_ws);
	mutex_unlock(&info->otg_lock);
}

static int psc_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct charger_properties props = { .alias_name = "psc5425e" };
	struct regulator_config config = { };
	struct regulator_dev *rdev;
	struct device_node *child;
	struct psc5425e *info;
	const char *name = "primary_chg";
	int ret, ident;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EOPNOTSUPP;
	/* Validate the real read-only ID before changing any charge setting. */
	ident = i2c_smbus_read_byte_data(client, PSC_ID);
	if (ident < 0)
		return ident;
	if (ident != 0xf0) {
		dev_err(&client->dev, "unexpected PSC5425E ID: 0x%02x\n", ident);
		return -ENODEV;
	}
	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->client = client;
	i2c_set_clientdata(client, info);
	mutex_init(&info->lock);
	mutex_init(&info->otg_lock);
	INIT_WORK(&info->otg_work, psc_otg_work);
	alarm_init(&info->otg_alarm, ALARM_BOOTTIME, psc_otg_alarm);
	info->otg_ws = wakeup_source_register(&client->dev, "psc5425e-otg");
	if (!info->otg_ws)
		return -ENOMEM;
	child = of_get_child_by_name(client->dev.of_node, "psc5425e");
	if (child) {
		of_property_read_string(child, "chg_name", &name);
		of_property_read_string(child, "chg_alias_name", &props.alias_name);
		of_node_put(child);
	} else {
		of_property_read_string(client->dev.of_node, "charger_name", &name);
		of_property_read_string(client->dev.of_node, "alias_name", &props.alias_name);
	}
	/* Stock setup, with charge disabled and 500 mA input until policy runs.
	 * REG4[7] must stay zero: setting it resets all charging parameters.
	 */
	ret = i2c_smbus_write_byte_data(client, PSC_CONTROL, 0x7c);
	if (!ret)
		ret = psc_update(info, 0x51, BIT(0), 0);
	if (!ret)
		ret = i2c_smbus_write_byte_data(client, PSC_VOLTAGE, 0x90);
	if (!ret)
		ret = i2c_smbus_write_byte_data(client, PSC_CURRENT, 0x16);
	if (!ret)
		ret = i2c_smbus_write_byte_data(client, PSC_SPECIAL, 0x24);
	if (ret < 0)
		goto fail_ws;
	info->chg = charger_device_register(name, &client->dev, info, &psc_ops, &props);
	if (IS_ERR(info->chg)) {
		ret = PTR_ERR(info->chg);
		goto fail_ws;
	}
	info->chg->is_polling_mode = true;
	config.dev = &client->dev;
	config.driver_data = info;
	config.of_node = client->dev.of_node;
	rdev = devm_regulator_register(&client->dev, &psc_vbus_desc, &config);
	if (IS_ERR(rdev)) {
		ret = PTR_ERR(rdev);
		psc_stop(info);
		charger_device_unregister(info->chg);
		goto fail_ws;
	}
	dev_info(&client->dev, "PSC5425E ID=0x%02x registered %s\n", ident, name);
	return 0;

fail_ws:
	wakeup_source_unregister(info->otg_ws);
	return ret;
}

static int psc_remove(struct i2c_client *client)
{
	struct psc5425e *info = i2c_get_clientdata(client);

	psc_stop(info);
	charger_device_unregister(info->chg);
	wakeup_source_unregister(info->otg_ws);
	return 0;
}

static void psc_shutdown(struct i2c_client *client)
{
	psc_stop(i2c_get_clientdata(client));
}

static const struct of_device_id psc_of_match[] = {
	{ .compatible = "mediatek,psc5425e" },
	{ }
};
MODULE_DEVICE_TABLE(of, psc_of_match);

static const struct i2c_device_id psc_ids[] = {
	{ "psc5425e", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, psc_ids);

static struct i2c_driver psc_driver = {
	.driver = {
		.name = "psc5425e",
		.of_match_table = psc_of_match,
	},
	.probe = psc_probe,
	.remove = psc_remove,
	.shutdown = psc_shutdown,
	.id_table = psc_ids,
};
module_i2c_driver(psc_driver);

MODULE_DESCRIPTION("S17Promini PSC5425E charger");
MODULE_LICENSE("GPL");
