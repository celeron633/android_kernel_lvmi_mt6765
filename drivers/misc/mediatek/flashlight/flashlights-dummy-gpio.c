// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/pinctrl/consumer.h>

#include "flashlight-core.h"
#include "flashlight-dt.h"

/* define device tree */
#ifndef DUMMY_GPIO_DTNAME
#define DUMMY_GPIO_DTNAME "mediatek,flashlights_dummy_gpio"
#endif

#define DUMMY_NAME "flashlights-dummy-gpio"

/* define mutex and work queue */
static DEFINE_MUTEX(dummy_mutex);
static DEFINE_MUTEX(dummy_op_mutex);
static struct work_struct dummy_work;

/* define pinctrl */
#define DUMMY_PINCTRL_PIN_GPIO 0
#define DUMMY_PINCTRL_PINSTATE_LOW 0
#define DUMMY_PINCTRL_PINSTATE_HIGH 1
#define DUMMY_PINCTRL_STATE_GPIO_HIGH "dummy_gpio_high"
#define DUMMY_PINCTRL_STATE_GPIO_LOW  "dummy_gpio_low"
static struct pinctrl *dummy_pinctrl;
static struct pinctrl_state *dummy_gpio_high;
static struct pinctrl_state *dummy_gpio_low;

/* define usage count */
static int use_count;

/* platform data */
struct dummy_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};


/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int dummy_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

	/* get pinctrl */
	dummy_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(dummy_pinctrl)) {
		pr_info("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(dummy_pinctrl);
		return ret;
	}

	/* Stock GPIO state names also match the S17 device tree. */
	dummy_gpio_high = pinctrl_lookup_state(
			dummy_pinctrl, DUMMY_PINCTRL_STATE_GPIO_HIGH);
	if (IS_ERR(dummy_gpio_high)) {
		pr_info("Failed to init (%s)\n", DUMMY_PINCTRL_STATE_GPIO_HIGH);
		ret = PTR_ERR(dummy_gpio_high);
	}
	dummy_gpio_low = pinctrl_lookup_state(
			dummy_pinctrl, DUMMY_PINCTRL_STATE_GPIO_LOW);
	if (IS_ERR(dummy_gpio_low)) {
		pr_info("Failed to init (%s)\n", DUMMY_PINCTRL_STATE_GPIO_LOW);
		ret = PTR_ERR(dummy_gpio_low);
	}

	return ret;
}

static int dummy_pinctrl_set(int pin, int state)
{
	struct pinctrl_state *pins;

	if (pin != DUMMY_PINCTRL_PIN_GPIO)
		return -EINVAL;
	if (IS_ERR_OR_NULL(dummy_pinctrl))
		return -ENODEV;
	if (state == DUMMY_PINCTRL_PINSTATE_LOW)
		pins = dummy_gpio_low;
	else if (state == DUMMY_PINCTRL_PINSTATE_HIGH)
		pins = dummy_gpio_high;
	else
		return -EINVAL;
	if (IS_ERR_OR_NULL(pins))
		return -ENODEV;

	return pinctrl_select_state(dummy_pinctrl, pins);
}

/* S17 stock hardware has one binary GPIO, with high enabling the LED. */
static int dummy_enable(void)
{
	return dummy_pinctrl_set(DUMMY_PINCTRL_PIN_GPIO,
			DUMMY_PINCTRL_PINSTATE_HIGH);
}

static int dummy_disable(void)
{
	return dummy_pinctrl_set(DUMMY_PINCTRL_PIN_GPIO,
			DUMMY_PINCTRL_PINSTATE_LOW);
}

/* Stock SET_DUTY resets the GPIO low; this hardware has no PWM levels. */
static int dummy_set_level(int level)
{
	return dummy_disable();
}

static int dummy_init(void)
{
	return dummy_disable();
}

static int dummy_uninit(void)
{
	return dummy_disable();
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer dummy_timer;
static unsigned int dummy_timeout_ms;

static void dummy_work_disable(struct work_struct *data)
{
	pr_debug("work queue callback\n");
	mutex_lock(&dummy_mutex);
	dummy_disable();
	mutex_unlock(&dummy_mutex);
}

static enum hrtimer_restart dummy_timer_func(struct hrtimer *timer)
{
	schedule_work(&dummy_work);
	return HRTIMER_NORESTART;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
/* Called with dummy_op_mutex held, before taking dummy_mutex. */
static void dummy_cancel_timeout(void)
{
	hrtimer_cancel(&dummy_timer);
	cancel_work_sync(&dummy_work);
}

static int dummy_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg = (struct flashlight_dev_arg *)arg;
	int ret = 0;

	mutex_lock(&dummy_op_mutex);
	if (cmd == FLASH_IOC_SET_ONOFF)
		dummy_cancel_timeout();
	mutex_lock(&dummy_mutex);
	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		if (fl_arg->arg < 0)
			ret = -EINVAL;
		else
			dummy_timeout_ms = fl_arg->arg;
		break;
	case FLASH_IOC_SET_DUTY:
		ret = dummy_set_level(fl_arg->arg);
		break;
	case FLASH_IOC_SET_ONOFF:
		if (fl_arg->arg == 1) {
			ret = dummy_enable();
			if (!ret && dummy_timeout_ms)
				hrtimer_start(&dummy_timer,
					ms_to_ktime(dummy_timeout_ms),
					HRTIMER_MODE_REL);
		} else {
			ret = dummy_disable();
		}
		pr_info("channel=%d on=%d timeout=%u ret=%d\n",
			fl_arg->channel, fl_arg->arg == 1,
			dummy_timeout_ms, ret);
		break;
	default:
		ret = -ENOTTY;
		break;
	}
	mutex_unlock(&dummy_mutex);
	mutex_unlock(&dummy_op_mutex);
	return ret;
}

static int dummy_open(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int dummy_release(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int dummy_set_driver(int set)
{
	int ret = 0;

	/* Serialize timeout cleanup against a new on request. */
	mutex_lock(&dummy_op_mutex);
	if (!set && use_count <= 1)
		dummy_cancel_timeout();
	mutex_lock(&dummy_mutex);
	if (set) {
		if (!use_count)
			ret = dummy_init();
		if (!ret)
			use_count++;
		pr_debug("Set driver: %d\n", use_count);
	} else {
		if (use_count > 0)
			use_count--;
		if (!use_count)
			ret = dummy_uninit();
		pr_debug("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&dummy_mutex);
	mutex_unlock(&dummy_op_mutex);

	return ret;
}

static ssize_t dummy_strobe_store(struct flashlight_arg arg)
{
	int ret;

	ret = dummy_set_driver(1);
	if (ret)
		return ret;
	mutex_lock(&dummy_op_mutex);
	dummy_cancel_timeout();
	mutex_lock(&dummy_mutex);
	ret = dummy_set_level(arg.level);
	if (!ret)
		ret = dummy_enable();
	if (!ret)
		msleep(arg.dur);
	if (!ret)
		ret = dummy_disable();
	mutex_unlock(&dummy_mutex);
	mutex_unlock(&dummy_op_mutex);
	dummy_set_driver(0);
	return ret;
}

static struct flashlight_operations dummy_ops = {
	dummy_open,
	dummy_release,
	dummy_ioctl,
	dummy_strobe_store,
	dummy_set_driver
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int dummy_chip_init(void)
{
	/* NOTE: Chip initialication move to "set driver" for power saving.
	 * dummy_init();
	 */

	return 0;
}

static int dummy_parse_dt(struct device *dev,
		struct dummy_platform_data *pdata)
{
	struct device_node *np, *cnp;
	u32 decouple = 0;
	int i = 0;

	if (!dev || !dev->of_node || !pdata)
		return -ENODEV;

	np = dev->of_node;

	pdata->channel_num = of_get_child_count(np);
	if (!pdata->channel_num) {
		pr_info("Parse no dt, node.\n");
		return 0;
	}
	pr_info("Channel number(%d).\n", pdata->channel_num);

	if (of_property_read_u32(np, "decouple", &decouple))
		pr_info("Parse no dt, decouple.\n");

	pdata->dev_id = devm_kzalloc(dev,
			pdata->channel_num *
			sizeof(struct flashlight_device_id),
			GFP_KERNEL);
	if (!pdata->dev_id)
		return -ENOMEM;

	for_each_child_of_node(np, cnp) {
		if (of_property_read_u32(cnp, "type", &pdata->dev_id[i].type))
			goto err_node_put;
		if (of_property_read_u32(cnp, "ct", &pdata->dev_id[i].ct))
			goto err_node_put;
		if (of_property_read_u32(cnp, "part", &pdata->dev_id[i].part))
			goto err_node_put;
		snprintf(pdata->dev_id[i].name, FLASHLIGHT_NAME_SIZE,
				DUMMY_NAME);
		pdata->dev_id[i].channel = i;
		pdata->dev_id[i].decouple = decouple;

		pr_info("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
				pdata->dev_id[i].type, pdata->dev_id[i].ct,
				pdata->dev_id[i].part, pdata->dev_id[i].name,
				pdata->dev_id[i].channel,
				pdata->dev_id[i].decouple);
		i++;
	}

	return 0;

err_node_put:
	of_node_put(cnp);
	return -EINVAL;
}

static int dummy_probe(struct platform_device *pdev)
{
	struct dummy_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int err;
	int i;

	pr_debug("Probe start.\n");

	/* init pinctrl */
	err = dummy_pinctrl_init(pdev);
	if (err) {
		pr_debug("Failed to init pinctrl.\n");
		goto err;
	}
	err = dummy_disable();
	if (err)
		return err;

	/* init platform data */
	if (!pdata) {
		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err;
		}
		pdev->dev.platform_data = pdata;
		err = dummy_parse_dt(&pdev->dev, pdata);
		if (err)
			goto err;
	}

	/* init work queue */
	INIT_WORK(&dummy_work, dummy_work_disable);

	/* init timer */
	hrtimer_init(&dummy_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	dummy_timer.function = dummy_timer_func;
	dummy_timeout_ms = 100;

	/* init chip hw */
	dummy_chip_init();

	/* clear usage count */
	use_count = 0;

	/* register flashlight device */
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++)
			if (flashlight_dev_register_by_device_id(
						&pdata->dev_id[i],
						&dummy_ops)) {
				err = -EFAULT;
				goto err;
			}
	} else {
		if (flashlight_dev_register(DUMMY_NAME, &dummy_ops)) {
			err = -EFAULT;
			goto err;
		}
	}

	pr_info("Registered GPIO flashlight, channels=%d\n",
		pdata->channel_num);

	return 0;
err:
	return err;
}

static int dummy_remove(struct platform_device *pdev)
{
	struct dummy_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int i;

	pr_debug("Remove start.\n");

	pdev->dev.platform_data = NULL;

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(
					&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(DUMMY_NAME);

	mutex_lock(&dummy_op_mutex);
	dummy_cancel_timeout();
	mutex_lock(&dummy_mutex);
	dummy_disable();
	mutex_unlock(&dummy_mutex);
	mutex_unlock(&dummy_op_mutex);

	pr_debug("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id dummy_gpio_of_match[] = {
	{.compatible = DUMMY_GPIO_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, dummy_gpio_of_match);
#else
static struct platform_device dummy_gpio_platform_device[] = {
	{
		.name = DUMMY_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, dummy_gpio_platform_device);
#endif

static struct platform_driver dummy_platform_driver = {
	.probe = dummy_probe,
	.remove = dummy_remove,
	.driver = {
		.name = DUMMY_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = dummy_gpio_of_match,
#endif
	},
};

static int __init flashlight_dummy_init(void)
{
	int ret;

	pr_debug("Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&dummy_gpio_platform_device);
	if (ret) {
		pr_info("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&dummy_platform_driver);
	if (ret) {
		pr_info("Failed to register platform driver\n");
		return ret;
	}

	pr_debug("Init done.\n");

	return 0;
}

static void __exit flashlight_dummy_exit(void)
{
	pr_debug("Exit start.\n");

	platform_driver_unregister(&dummy_platform_driver);

	pr_debug("Exit done.\n");
}

module_init(flashlight_dummy_init);
module_exit(flashlight_dummy_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Wang <Simon-TCH.Wang@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight DUMMY GPIO Driver");

