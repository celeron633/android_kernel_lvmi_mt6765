// SPDX-License-Identifier: GPL-2.0-only
/*
 * S17Promini AXS15260 basic reports reconstructed from its stock kernel.
 * The LCM owns the shared TDDI reset; this driver does not request GPIO 174.
 * Firmware upgrade and the vendor debugging interfaces are separate work.
 */
#include <linux/device.h>
#include <linux/fb.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include "axs15260_s17.h"

struct axs_touch {
	struct i2c_client *client;
	struct input_dev *input;
	struct touchscreen_properties props;
	struct notifier_block fb_notifier;
	struct mutex state_lock;
	bool fb_suspended;
	bool system_suspended;
	bool irq_disabled;
	bool pressed;
};

static void axs_release(struct axs_touch *touch)
{
	if (!touch->pressed)
		return;
	input_report_key(touch->input, BTN_TOUCH, 0);
	input_mt_sync(touch->input);
	input_sync(touch->input);
	touch->pressed = false;
}

static irqreturn_t axs_irq_thread(int irq, void *data)
{
	struct axs_touch *touch = data;
	struct axs_frame frame;
	u8 buf[AXS_REPORT_SIZE];
	struct i2c_msg msg = {
		.addr = touch->client->addr,
		.flags = I2C_M_RD,
		.len = sizeof(buf),
		.buf = buf,
	};
	unsigned int i, active = 0;
	int ret;

	/* Stock reads one message directly, without a register-address write. */
	ret = i2c_transfer(touch->client->adapter, &msg, 1);
	if (ret != 1) {
		dev_err_ratelimited(&touch->client->dev,
				    "report read failed: %d\n", ret);
		axs_release(touch);
		return IRQ_HANDLED;
	}
	ret = axs_decode_report(buf, sizeof(buf), &frame);
	if (ret)
		return IRQ_HANDLED;

	for (i = 0; i < frame.count; i++) {
		struct axs_point *point = &frame.points[i];

		/* Vendor type-A reports consider events 0/2 down/contact. */
		if (point->event != 0 && point->event != 2)
			continue;
		input_report_abs(touch->input, ABS_MT_TRACKING_ID, point->id);
		input_report_abs(touch->input, ABS_MT_PRESSURE, point->pressure);
		input_report_abs(touch->input, ABS_MT_TOUCH_MAJOR, point->area);
		touchscreen_report_pos(touch->input, &touch->props,
				       point->x, point->y, true);
		input_mt_sync(touch->input);
		active++;
	}
	if (!active) {
		axs_release(touch);
		return IRQ_HANDLED;
	}
	input_report_key(touch->input, BTN_TOUCH, 1);
	input_sync(touch->input);
	touch->pressed = true;
	return IRQ_HANDLED;
}

/* Callers serialize with state_lock; IRQ never takes it. */
static void axs_update_irq(struct axs_touch *touch)
{
	bool disable = touch->fb_suspended || touch->system_suspended;

	if (disable == touch->irq_disabled)
		return;
	if (disable) {
		disable_irq(touch->client->irq);
		axs_release(touch);
	} else {
		enable_irq(touch->client->irq);
	}
	touch->irq_disabled = disable;
}

static int axs_fb_event(struct notifier_block *nb, unsigned long event, void *data)
{
	struct axs_touch *touch = container_of(nb, struct axs_touch, fb_notifier);
	struct fb_event *fb_event = data;
	int blank;

	if (!fb_event || !fb_event->data)
		return NOTIFY_DONE;
	blank = *(int *)fb_event->data;
	if (event == FB_EARLY_EVENT_BLANK && blank == FB_BLANK_POWERDOWN) {
		mutex_lock(&touch->state_lock);
		touch->fb_suspended = true;
		axs_update_irq(touch);
		mutex_unlock(&touch->state_lock);
	} else if (event == FB_EVENT_BLANK && blank == FB_BLANK_UNBLANK) {
		mutex_lock(&touch->state_lock);
		touch->fb_suspended = false;
		axs_update_irq(touch);
		mutex_unlock(&touch->state_lock);
	}
	return NOTIFY_OK;
}

static int axs_suspend(struct device *dev)
{
	struct axs_touch *touch = i2c_get_clientdata(to_i2c_client(dev));

	mutex_lock(&touch->state_lock);
	touch->system_suspended = true;
	axs_update_irq(touch);
	mutex_unlock(&touch->state_lock);
	return 0;
}

static int axs_resume(struct device *dev)
{
	struct axs_touch *touch = i2c_get_clientdata(to_i2c_client(dev));

	mutex_lock(&touch->state_lock);
	touch->system_suspended = false;
	axs_update_irq(touch);
	mutex_unlock(&touch->state_lock);
	return 0;
}

static SIMPLE_DEV_PM_OPS(axs_pm_ops, axs_suspend, axs_resume);

static void axs_disable_supply(void *data)
{
	regulator_disable(data);
}

static void axs_remove_irq(void *data)
{
	struct axs_touch *touch = data;

	devm_free_irq(&touch->client->dev, touch->client->irq, touch);
}

static int axs_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct axs_touch *touch;
	struct regulator *supply;
	u8 cmd = 0x0c, version[2];
	struct i2c_msg version_msgs[] = {
		{ .addr = client->addr, .len = 1, .buf = &cmd },
		{ .addr = client->addr, .flags = I2C_M_RD,
		  .len = 2, .buf = version },
	};
	int ret;

	if (client->addr != 0x3b || client->irq <= 0)
		return -EINVAL;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EOPNOTSUPP;

	touch = devm_kzalloc(&client->dev, sizeof(*touch), GFP_KERNEL);
	if (!touch)
		return -ENOMEM;
	touch->client = client;
	mutex_init(&touch->state_lock);
	i2c_set_clientdata(client, touch);
	supply = devm_regulator_get(&client->dev, "vtouch");
	if (IS_ERR(supply))
		return PTR_ERR(supply);
	ret = regulator_enable(supply);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(&client->dev, axs_disable_supply, supply);
	if (ret)
		return ret;

	/* Read-only startup: log existing firmware; leave flash writes for later. */
	ret = i2c_transfer(client->adapter, version_msgs, ARRAY_SIZE(version_msgs));
	if (ret == ARRAY_SIZE(version_msgs))
		dev_info(&client->dev, "existing firmware version %02x%02x\n",
			 version[0], version[1]);
	else
		dev_info(&client->dev, "firmware version not ready: %d\n", ret);

	touch->input = devm_input_allocate_device(&client->dev);
	if (!touch->input)
		return -ENOMEM;
	touch->input->name = "axs_ts";
	touch->input->id.bustype = BUS_I2C;
	input_set_capability(touch->input, EV_KEY, BTN_TOUCH);
	__set_bit(INPUT_PROP_DIRECT, touch->input->propbit);
	input_set_abs_params(touch->input, ABS_MT_TRACKING_ID, 0, 15, 0, 0);
	input_set_abs_params(touch->input, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(touch->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(touch->input, ABS_MT_POSITION_X, 0, 384, 0, 0);
	input_set_abs_params(touch->input, ABS_MT_POSITION_Y, 0, 854, 0, 0);
	touchscreen_parse_properties(touch->input, true, &touch->props);
	ret = input_register_device(touch->input);
	if (ret)
		return ret;
	ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
					axs_irq_thread, IRQF_ONESHOT, "axs_ts", touch);
	if (ret)
		return ret;
	/* Release IRQ before the managed input device on probe failure/removal. */
	ret = devm_add_action_or_reset(&client->dev, axs_remove_irq, touch);
	if (ret)
		return ret;
	touch->fb_notifier.notifier_call = axs_fb_event;
	ret = fb_register_client(&touch->fb_notifier);
	if (ret)
		return ret;
	dev_info(&client->dev, "basic reports ready: addr=0x3b irq=%d\n", client->irq);
	return 0;
}

static int axs_remove(struct i2c_client *client)
{
	struct axs_touch *touch = i2c_get_clientdata(client);

	fb_unregister_client(&touch->fb_notifier);
	mutex_lock(&touch->state_lock);
	touch->system_suspended = true;
	axs_update_irq(touch);
	mutex_unlock(&touch->state_lock);
	return 0;
}

static const struct of_device_id axs_of_match[] = {
	{ .compatible = "lvmi,s17promini-axs15260" },
	{ }
};
MODULE_DEVICE_TABLE(of, axs_of_match);

static const struct i2c_device_id axs_ids[] = {
	{ "axs15260-s17", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, axs_ids);

static struct i2c_driver axs_driver = {
	.driver = {
		.name = "axs15260-s17",
		.of_match_table = axs_of_match,
		.pm = &axs_pm_ops,
	},
	.probe = axs_probe,
	.remove = axs_remove,
	.id_table = axs_ids,
};
module_i2c_driver(axs_driver);

MODULE_DESCRIPTION("S17Promini AXS15260 basic touch reports");
MODULE_LICENSE("GPL");
