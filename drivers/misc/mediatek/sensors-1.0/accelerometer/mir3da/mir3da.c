// SPDX-License-Identifier: GPL-2.0
/* S17 MIR3DA NSA_NTO and stock acceleration based proximity. */
#define pr_fmt(fmt) "[S17/MIR3DA] " fmt

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_wakeup.h>
#include <linux/workqueue.h>
#include <accel.h>
#include <accel_factory.h>
#include <alsps.h>
#include <hwmsen_helper.h>
#include "mir3da_math.h"

/* Built in, one board sensor; framework callbacks have kernel lifetime. */
static struct {
	struct i2c_client *client;
	struct i2c_client *alternate;
	struct device *dev;
	struct mutex lock;
	struct mutex control_lock;
	struct delayed_work ps_work;
	struct hwmsen_convert convert;
	struct mir3da_calibration calibration;
	int cali[3];
	unsigned int near_count;
	unsigned int acc_period_ms;
	unsigned int factory_period_ms;
	unsigned int ps_period_ms;
	int ps_value;
	int last_report;
	u8 chip_info;
	u8 package;
	u8 variant;
	bool ready;
	bool identified;
	bool powered;
	bool suspended;
	bool acc_enabled;
	bool factory_enabled;
	bool ps_enabled;
} sensor;

struct mir3da_reg {
	u8 reg;
	u8 mask;
	u8 value;
};

/* Stock Image offset 0x175b2f8, NSA_NTO init_sect. */
static const struct mir3da_reg mir3da_init_regs[] = {
	{ 0x0f, 0x03, 0x01 },
	{ 0x11, 0xff, 0x3e },
	{ 0x10, 0xff, 0x07 },
	{ 0x17, 0xff, 0x00 },
	{ 0x1a, 0xff, 0x00 },
	{ 0x7f, 0xff, 0x83 },
	{ 0x7f, 0xff, 0x69 },
	{ 0x7f, 0xff, 0xbd },
	{ 0x20, 0x0f, 0x05 },
};

static int mir3da_update(u8 reg, u8 mask, u8 value)
{
	int ret = i2c_smbus_read_byte_data(sensor.client, reg);

	if (ret < 0)
		return ret;
	return i2c_smbus_write_byte_data(sensor.client, reg,
				       (ret & ~mask) | (value & mask));
}

static unsigned int mir3da_period(void)
{
	unsigned int period = 1000;

	if (sensor.acc_enabled)
		period = min(period, sensor.acc_period_ms);
	if (sensor.factory_enabled)
		period = min(period, sensor.factory_period_ms);
	if (sensor.ps_enabled)
		period = min(period, sensor.ps_period_ms);
	return period;
}

static int mir3da_power(void)
{
	bool on = !sensor.suspended &&
		(sensor.acc_enabled || sensor.factory_enabled || sensor.ps_enabled);
	unsigned int period = mir3da_period();
	int ret;

	if (on) {
		ret = mir3da_update(0x10, 0x0f,
				    period <= 5 ? 8 : period <= 10 ? 7 : 6);
		if (ret)
			return ret;
	}
	if (on == sensor.powered)
		return 0;
	ret = mir3da_update(0x11, 0x80, on ? 0 : 0x80);
	if (!ret) {
		sensor.powered = on;
		if (on)
			msleep(20);
	}
	return ret;
}

static int mir3da_read(int mapped[3], bool calibrate)
{
	u8 data[6];
	int v[3], i, ret;

	if (!sensor.ready || !sensor.powered || sensor.suspended)
		return -EAGAIN;
	ret = i2c_smbus_read_i2c_block_data(sensor.client, 0x02, 6, data);
	if (ret != 6)
		return ret < 0 ? ret : -EIO;
	for (i = 0; i < 3; i++)
		v[i] = (s16)(data[i * 2] | data[i * 2 + 1] << 8) >> 3;
	/* Preserve the stock package specific Z handling and compensation. */
	if (sensor.chip_info == 0x4b || sensor.chip_info == 0x8c ||
	    sensor.chip_info == 0xca || sensor.variant == 5)
		v[2] = 0;
	else
		mir3da_temperature_compensate(&sensor.calibration, v);
	for (i = 0; i < 3; i++)
		mapped[sensor.convert.map[i]] = sensor.convert.sign[i] * v[i] +
			(calibrate ? sensor.cali[sensor.convert.map[i]] : 0);
	/* Stock special Z calibration correction, in sensor coordinates. */
	if (calibrate && abs(sensor.cali[sensor.convert.map[2]]) > 1300)
		mapped[sensor.convert.map[2]] -= 2048;
	return 0;
}

static void mir3da_ps_work(struct work_struct *work)
{
	int raw[3], value = -1, ret;

	mutex_lock(&sensor.lock);
	if (!sensor.ps_enabled || !sensor.ready || sensor.suspended)
		goto out;
	ret = mir3da_read(raw, true);
	if (!ret) {
		sensor.ps_value = mir3da_proximity_sample(raw, &sensor.near_count);
		if (sensor.ps_value != sensor.last_report) {
			value = sensor.ps_value;
			sensor.last_report = value;
		}
	} else {
		/* Failed reads cannot count toward four consecutive samples. */
		sensor.near_count = 0;
		dev_warn_ratelimited(sensor.dev, "PS read failed: %d\n", ret);
	}
	mod_delayed_work(system_wq, &sensor.ps_work,
			 msecs_to_jiffies(sensor.ps_period_ms));
out:
	mutex_unlock(&sensor.lock);
	if (value >= 0)
		ps_report_interrupt_data(value);
}

static int mir3da_open(int open)
{
	return sensor.ready ? 0 : -ENODEV;
}

static int mir3da_acc_enable(int enable)
{
	bool old;
	int ret;

	mutex_lock(&sensor.lock);
	if (!sensor.ready) {
		ret = -ENODEV;
		goto out;
	}
	old = sensor.acc_enabled;
	sensor.acc_enabled = !!enable;
	ret = mir3da_power();
	if (ret)
		sensor.acc_enabled = old;
out:
	mutex_unlock(&sensor.lock);
	return ret;
}

static int mir3da_acc_delay(u64 ns)
{
	unsigned int old;
	int ret;

	mutex_lock(&sensor.lock);
	old = sensor.acc_period_ms;
	sensor.acc_period_ms = clamp_t(u64, div_u64(ns, 1000000), 5, 1000);
	ret = sensor.ready ? mir3da_power() : -ENODEV;
	if (ret)
		sensor.acc_period_ms = old;
	mutex_unlock(&sensor.lock);
	return ret;
}

static int mir3da_acc_batch(int flag, s64 period, s64 latency)
{
	return period > 0 ? mir3da_acc_delay(period) : -EINVAL;
}

static int mir3da_acc_flush(void)
{
	return sensor.ready ? acc_flush_report() : -ENODEV;
}

static int mir3da_acc_data(int *x, int *y, int *z, int *status)
{
	int raw[3], ret;

	mutex_lock(&sensor.lock);
	ret = mir3da_read(raw, true);
	if (!ret) {
		*x = raw[0] * 9807 / 1024;
		*y = raw[1] * 9807 / 1024;
		*z = raw[2] * 9807 / 1024;
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}
	mutex_unlock(&sensor.lock);
	return ret;
}

static int mir3da_ps_enable(int enable)
{
	bool old;
	int ret;

	/* Serialize cancellation against enable across the two frameworks. */
	mutex_lock(&sensor.control_lock);
	mutex_lock(&sensor.lock);
	if (!sensor.ready) {
		ret = -ENODEV;
		goto unlock;
	}
	old = sensor.ps_enabled;
	sensor.ps_enabled = !!enable;
	ret = mir3da_power();
	if (ret) {
		sensor.ps_enabled = old;
		goto unlock;
	}
	if (enable) {
		if (!old) {
			sensor.near_count = 0;
			sensor.ps_value = 1;
			sensor.last_report = -1;
			pm_stay_awake(sensor.dev);
		}
		mod_delayed_work(system_wq, &sensor.ps_work, 0);
	}
unlock:
	mutex_unlock(&sensor.lock);
	if (!ret && !enable) {
		cancel_delayed_work_sync(&sensor.ps_work);
		pm_relax(sensor.dev);
	}
	mutex_unlock(&sensor.control_lock);
	return ret;
}

static int mir3da_ps_delay(u64 ns)
{
	unsigned int old;
	int ret;

	mutex_lock(&sensor.lock);
	old = sensor.ps_period_ms;
	/* Posture detection remains responsive even with slow HAL requests. */
	sensor.ps_period_ms = clamp_t(u64, div_u64(ns, 1000000), 5, 50);
	ret = sensor.ready ? mir3da_power() : -ENODEV;
	if (ret)
		sensor.ps_period_ms = old;
	mutex_unlock(&sensor.lock);
	return ret;
}

static int mir3da_ps_batch(int flag, s64 period, s64 latency)
{
	return period > 0 ? mir3da_ps_delay(period) : -EINVAL;
}

static int mir3da_ps_flush(void)
{
	return sensor.ready ? ps_flush_report() : -ENODEV;
}

static int mir3da_ps_data(int *value, int *status)
{
	int ret;

	mutex_lock(&sensor.lock);
	ret = sensor.ready ? 0 : -ENODEV;
	if (!ret) {
		*value = sensor.ps_value;
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}
	mutex_unlock(&sensor.lock);
	return ret;
}

static int mir3da_factory_enable(bool enable, s64 period)
{
	bool old;
	unsigned int old_period;
	int ret;

	if (period <= 0)
		return -EINVAL;
	mutex_lock(&sensor.lock);
	if (!sensor.ready) {
		ret = -ENODEV;
		goto out;
	}
	old = sensor.factory_enabled;
	old_period = sensor.factory_period_ms;
	sensor.factory_enabled = enable;
	sensor.factory_period_ms = clamp_t(s64, period, 5, 1000);
	ret = mir3da_power();
	if (ret) {
		sensor.factory_enabled = old;
		sensor.factory_period_ms = old_period;
	}
out:
	mutex_unlock(&sensor.lock);
	return ret;
}

static int mir3da_factory_data(s32 data[3], int *status)
{
	return mir3da_acc_data(&data[0], &data[1], &data[2], status);
}

static int mir3da_factory_raw(s32 data[3])
{
	int ret;

	mutex_lock(&sensor.lock);
	ret = mir3da_read(data, false);
	mutex_unlock(&sensor.lock);
	return ret;
}

static int mir3da_clear_cali(void)
{
	mutex_lock(&sensor.lock);
	memset(sensor.cali, 0, sizeof(sensor.cali));
	memset(&sensor.calibration, 0, sizeof(sensor.calibration));
	mutex_unlock(&sensor.lock);
	return 0;
}

static int mir3da_set_cali(s32 offset[3])
{
	int i, raw[3];

	for (i = 0; i < 3; i++) {
		if (offset[i] < -20000 || offset[i] > 20000)
			return -ERANGE;
		raw[i] = offset[i] * 1024 / 9807;
	}
	mutex_lock(&sensor.lock);
	for (i = 0; i < 3; i++)
		sensor.cali[i] = clamp(sensor.cali[i] + raw[i], -4096, 4096);
	mutex_unlock(&sensor.lock);
	return 0;
}

static int mir3da_get_cali(s32 offset[3])
{
	int i;

	mutex_lock(&sensor.lock);
	for (i = 0; i < 3; i++)
		offset[i] = sensor.cali[i] * 9807 / 1024;
	mutex_unlock(&sensor.lock);
	return 0;
}

static int mir3da_cali_buffer(u8 *data, u8 count)
{
	s32 offset[3];

	if (count != sizeof(offset))
		return -EINVAL;
	memcpy(offset, data, sizeof(offset));
	return mir3da_set_cali(offset);
}

static struct accel_factory_fops mir3da_factory_ops = {
	.enable_sensor = mir3da_factory_enable,
	.get_data = mir3da_factory_data,
	.get_raw_data = mir3da_factory_raw,
	.clear_cali = mir3da_clear_cali,
	.set_cali = mir3da_set_cali,
	.get_cali = mir3da_get_cali,
};

static struct accel_factory_public mir3da_factory = {
	.gain = 1,
	.sensitivity = 1,
	.fops = &mir3da_factory_ops,
};

static int mir3da_select_address(struct i2c_client *client)
{
	int id = -ENODEV, attempt;

	/* Stock probe retries REG01 six times before falling back to 0x27. */
	for (attempt = 0; attempt < 6; attempt++) {
		id = i2c_smbus_read_byte_data(client, 0x01);
		if (id == 0x13)
			return 0;
		if (attempt != 5)
			usleep_range(5000, 6000);
	}
	if (client->addr != 0x26)
		return id < 0 ? id : -ENODEV;
	dev_info(sensor.dev, "REG01 at 0x26 returned %d; trying stock fallback 0x27\n",
		 id);
	/* Reserve the alternate address instead of rewriting the DT client. */
	sensor.alternate = i2c_new_dummy(client->adapter, 0x27);
	if (!sensor.alternate) {
		dev_err(sensor.dev, "cannot reserve alternate I2C address 0x27\n");
		return -EBUSY;
	}
	id = i2c_smbus_read_byte_data(sensor.alternate, 0x01);
	if (id != 0x13) {
		dev_err(sensor.dev, "REG01 at 0x27 returned %d, expected 0x13\n", id);
		i2c_unregister_device(sensor.alternate);
		sensor.alternate = NULL;
		return id < 0 ? id : -ENODEV;
	}
	sensor.client = sensor.alternate;
	dev_info(sensor.dev, "MIR3DA found at alternate address 0x27\n");
	return 0;
}

static int mir3da_hardware_init(void)
{
	int id, info, c1, trim, package, revision, ret, i;

	id = i2c_smbus_read_byte_data(sensor.client, 0x01);
	dev_info(sensor.dev, "identity REG01=%d at addr=0x%02x\n",
		 id, sensor.client->addr);
	if (id < 0)
		return id;
	if (id != 0x13)
		return -ENODEV;
	/* The second NSA_NTO identification field is REG14 & 0xff == 0. */
	id = i2c_smbus_read_byte_data(sensor.client, 0x14);
	dev_info(sensor.dev, "identity REG14=%d\n", id);
	if (id < 0)
		return id;
	if (id)
		return -ENODEV;
	sensor.identified = true;
	ret = mir3da_update(0x00, 0x24, 0x24);
	if (ret) {
		dev_err(sensor.dev, "soft reset failed: %d\n", ret);
		return ret;
	}
	usleep_range(5000, 6000);
	for (i = 0; i < ARRAY_SIZE(mir3da_init_regs); i++) {
		ret = mir3da_update(mir3da_init_regs[i].reg,
			mir3da_init_regs[i].mask, mir3da_init_regs[i].value);
		if (ret) {
			dev_err(sensor.dev, "init REG%02x failed: %d\n",
				 mir3da_init_regs[i].reg, ret);
			return ret;
		}
	}
	usleep_range(10000, 11000);
	info = i2c_smbus_read_byte_data(sensor.client, 0xc0);
	dev_info(sensor.dev, "chip REGC0=%d\n", info);
	if (info < 0)
		return info;
	if (info < 0x40) {
		dev_err(sensor.dev, "invalid REGC0=0x%02x (stock requires >=0x40)\n",
			 info);
		return -ENODEV;
	}
	package = (info & 0x38) == 0x18 ? 4 :
		  (info & 0x38) == 0x10 ? 3 : 2;
	c1 = i2c_smbus_read_byte_data(sensor.client, 0xc1);
	dev_info(sensor.dev, "chip REGC1=%d\n", c1);
	if (c1 < 0)
		return c1;
	trim = i2c_smbus_read_byte_data(sensor.client, 0x8f);
	dev_info(sensor.dev, "chip REG8F=%d\n", trim);
	if (trim < 0)
		return trim;
	revision = package == 2 ? (c1 >> 6) | ((trim & 1) << 2) : c1 >> 5;
	sensor.chip_info = info;
	sensor.package = package;
	/* Stock initializes variant=2, then skips the switch for revision>4. */
	sensor.variant = 2;
	switch (revision) {
	case 0:
		sensor.variant = trim & 0x80 ? 3 : 2;
		break;
	case 1:
		sensor.variant = 4;
		break;
	case 2:
		sensor.variant = 2;
		break;
	case 3:
		sensor.variant = package == 4 && info == 0x5a ? 5 : 6;
		break;
	case 4:
		sensor.variant = 6;
		break;
	default:
		dev_info(sensor.dev, "revision=%d retains stock default variant=2\n",
			 revision);
		break;
	}
	if (package == 2 && sensor.client->addr == 0x26) {
		ret = mir3da_update(0x8c, 0xc0, 0);
		if (ret)
			return ret;
	}
	/* Probe leaves the chip off until a framework consumer enables it. */
	ret = mir3da_update(0x11, 0x80, 0x80);
	if (!ret)
		dev_info(sensor.dev,
			 "NSA_NTO addr=0x%02x ID=0x13 chip=0x%02x package=%d variant=%u direction=4\n",
			 sensor.client->addr, info, package, sensor.variant);
	return ret;
}

static int mir3da_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct acc_control_path acc_ctl = {
		.open_report_data = mir3da_open,
		.enable_nodata = mir3da_acc_enable,
		.set_delay = mir3da_acc_delay,
		.batch = mir3da_acc_batch,
		.flush = mir3da_acc_flush,
		.set_cali = mir3da_cali_buffer,
	};
	struct acc_data_path acc_data = {
		.get_data = mir3da_acc_data,
		.vender_div = 1000,
	};
	struct ps_control_path ps_ctl = {
		.open_report_data = mir3da_open,
		.enable_nodata = mir3da_ps_enable,
		.set_delay = mir3da_ps_delay,
		.batch = mir3da_ps_batch,
		.flush = mir3da_ps_flush,
		.is_report_input_direct = true,
	};
	struct ps_data_path ps_data = {
		.get_data = mir3da_ps_data,
		.vender_div = 100,
	};
	int ret;

	if (sensor.client)
		return -EBUSY;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA |
				     I2C_FUNC_SMBUS_READ_I2C_BLOCK))
		return -EOPNOTSUPP;
	sensor.client = client;
	sensor.dev = &client->dev;
	sensor.identified = false;
	sensor.acc_period_ms = 20;
	sensor.factory_period_ms = 20;
	sensor.ps_period_ms = 50;
	sensor.ps_value = 1;
	sensor.last_report = -1;
	mutex_init(&sensor.lock);
	mutex_init(&sensor.control_lock);
	INIT_DELAYED_WORK(&sensor.ps_work, mir3da_ps_work);
	ret = hwmsen_get_convert(4, &sensor.convert);
	if (ret)
		goto hardware_fail;
	ret = mir3da_select_address(client);
	if (ret)
		goto hardware_fail;
	ret = mir3da_hardware_init();
	if (ret)
		goto hardware_fail;
	ret = device_init_wakeup(&client->dev, true);
	if (ret)
		goto hardware_fail;
	ret = accel_factory_device_register(&mir3da_factory);
	if (ret)
		goto registration_fail;
	ret = acc_register_control_path(&acc_ctl);
	if (ret)
		goto factory_fail;
	ret = acc_register_data_path(&acc_data);
	if (ret)
		goto factory_fail;
	if (IS_ENABLED(CONFIG_GSENSOR_PROXIMITY)) {
		ret = ps_register_control_path(&ps_ctl);
		if (ret)
			goto factory_fail;
		ret = ps_register_data_path(&ps_data);
		if (ret)
			goto factory_fail;
	}
	sensor.ready = true;
	dev_info(&client->dev, "accelerometer and posture proximity registered\n");
	return 0;
factory_fail:
	accel_factory_device_deregister(&mir3da_factory);
registration_fail:
	device_init_wakeup(&client->dev, false);
	mir3da_update(0x11, 0x80, 0x80);
	/* Framework callbacks remain safe with ready=false on partial failure. */
	dev_err(&client->dev, "framework registration failed: %d\n", ret);
	goto release_address;
hardware_fail:
	dev_err(&client->dev, "hardware initialization failed: %d\n", ret);
	if (sensor.identified)
		mir3da_update(0x11, 0x80, 0x80);
release_address:
	if (sensor.alternate) {
		i2c_unregister_device(sensor.alternate);
		sensor.alternate = NULL;
	}
	sensor.client = NULL;
	sensor.dev = NULL;
	return ret;
}

static void mir3da_shutdown(struct i2c_client *client)
{
	if (!sensor.ready)
		return;
	mir3da_ps_enable(0);
	mutex_lock(&sensor.lock);
	sensor.ps_enabled = false;
	sensor.acc_enabled = false;
	sensor.factory_enabled = false;
	mir3da_power();
	sensor.ready = false;
	mutex_unlock(&sensor.lock);
	cancel_delayed_work_sync(&sensor.ps_work);
	pm_relax(&client->dev);
}

static int mir3da_suspend(struct device *dev)
{
	int ret;

	mutex_lock(&sensor.lock);
	/* The stock PS contract keeps the system awake while PS is enabled. */
	if (sensor.ps_enabled) {
		ret = -EBUSY;
	} else {
		sensor.suspended = true;
		ret = mir3da_power();
		if (ret)
			sensor.suspended = false;
	}
	mutex_unlock(&sensor.lock);
	return ret;
}

static int mir3da_resume(struct device *dev)
{
	int ret;

	mutex_lock(&sensor.lock);
	sensor.suspended = false;
	ret = mir3da_power();
	mutex_unlock(&sensor.lock);
	return ret;
}

static SIMPLE_DEV_PM_OPS(mir3da_pm, mir3da_suspend, mir3da_resume);
static const struct of_device_id mir3da_of_match[] = {
	{ .compatible = "mediatek,gsensor" },
	{ }
};

static const struct i2c_device_id mir3da_ids[] = {
	{ "mir3da", 0 },
	{ }
};

static struct i2c_driver mir3da_i2c_driver = {
	.driver = {
		.name = "s17-mir3da",
		.of_match_table = mir3da_of_match,
		.pm = &mir3da_pm,
		/* MTK callback APIs have no unregister; built in for this board. */
		.suppress_bind_attrs = true,
	},
	.probe = mir3da_probe,
	.shutdown = mir3da_shutdown,
	.id_table = mir3da_ids,
};

static int mir3da_local_init(void)
{
	int ret = i2c_add_driver(&mir3da_i2c_driver);

	return ret ? ret : sensor.ready ? 0 : -ENODEV;
}

static struct acc_init_info mir3da_init_info = {
	.name = "mir3da",
	.init = mir3da_local_init,
};

static int __init mir3da_init(void)
{
	return acc_driver_add(&mir3da_init_info);
}
module_init(mir3da_init);
MODULE_DESCRIPTION("S17 MIR3DA accelerometer and posture proximity");
MODULE_LICENSE("GPL");
