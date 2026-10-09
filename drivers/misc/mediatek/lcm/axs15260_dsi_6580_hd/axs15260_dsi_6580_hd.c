// SPDX-License-Identifier: GPL-2.0-only
/*
 * S17Promini AXS15260 LCM, reconstructed from its stock 4.19.191 kernel.
 * See Documentation/S17Promini-axs15260.md for addresses and validation.
 * Kernel implementation only; the original LK remains unchanged.
 */
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/string.h>

#include "lcm_drv.h"
#include "disp_dts_gpio.h"

#define REGFLAG_DELAY 0x0ffe
#define REGFLAG_END_OF_TABLE 0x0fff

static struct LCM_UTIL_FUNCS lcm_util;
static struct i2c_client *bias_client;
static struct gpio_desc *touch_reset_gpio;
static DEFINE_MUTEX(bias_lock);

struct lcm_setting {
	u32 cmd;
	u8 count;
	u8 params[64];
};

/* Stock tables at Image offsets 0x176e9f8 and 0x176ec38 (stride 0x48). */
static struct lcm_setting init_setting[] = {
	{ 0xbb, 8, { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5a, 0xa5 } },
	{ 0xb2, 11, { 0x01, 0x00, 0x00, 0x06, 0x00, 0x0a,
		      0x04, 0x21, 0x9c, 0x00, 0x1e } },
	{ 0xb2, 1, { 0x41 } },
	{ 0x11, 1, { 0x00 } },
	{ REGFLAG_DELAY, 120, { 0 } },
	{ 0x29, 1, { 0x00 } },
	{ REGFLAG_DELAY, 10, { 0 } },
	{ REGFLAG_END_OF_TABLE, 0, { 0 } },
};

static struct lcm_setting suspend_setting[] = {
	{ 0x28, 1, { 0x00 } },
	{ REGFLAG_DELAY, 120, { 0 } },
	{ 0x10, 1, { 0x00 } },
	{ REGFLAG_DELAY, 30, { 0 } },
	{ REGFLAG_END_OF_TABLE, 0, { 0 } },
};

static void push_table(struct lcm_setting *table, unsigned int count)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		if (table[i].cmd == REGFLAG_END_OF_TABLE)
			return;
		if (table[i].cmd == REGFLAG_DELAY)
			(lcm_util.mdelay)(table[i].count);
		else
			lcm_util.dsi_set_cmdq_V2(table[i].cmd, table[i].count,
					       table[i].params, 1);
	}
}

static int bias_write(u8 reg)
{
	u8 data[] = { reg, 0x12 };
	int ret;

	mutex_lock(&bias_lock);
	if (bias_client)
		ret = i2c_master_send(bias_client, data, sizeof(data));
	else
		ret = -ENODEV;
	mutex_unlock(&bias_lock);

	if (ret == sizeof(data))
		return 0;
	if (ret >= 0)
		ret = -EIO;
	pr_err("[LCM/AXS15260] bias register %u write failed: %d\n", reg, ret);
	return ret;
}

static int bias_enable(void)
{
	int ret;

	/* Stock GPIO state 10 = lcd_bias_enp1_gpio (GPIO 150), then 20 ms. */
	ret = disp_dts_gpio_select_state(DTS_GPIO_STATE_LCD_BIAS_ENP1);
	if (ret)
		return ret;
	(lcm_util.mdelay)(20);
	ret = bias_write(0);
	if (!ret)
		ret = bias_write(1);
	if (ret)
		disp_dts_gpio_select_state(DTS_GPIO_STATE_LCD_BIAS_ENP0);
	return ret;
}

static void reset_panel(void)
{
	lcm_util.set_reset_pin(1);
	(lcm_util.mdelay)(20);
	lcm_util.set_reset_pin(0);
	(lcm_util.mdelay)(20);
	lcm_util.set_reset_pin(1);
	(lcm_util.mdelay)(100);
}

/* Stock axs_reset_proc(20), confirmed by LK GPIO 174 sequencing too. */
static int reset_touch(void)
{
	mutex_lock(&bias_lock);
	if (!touch_reset_gpio) {
		mutex_unlock(&bias_lock);
		return -ENODEV;
	}
	gpiod_set_value_cansleep(touch_reset_gpio, 1);
	usleep_range(1000, 1100);
	gpiod_set_value_cansleep(touch_reset_gpio, 0);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(touch_reset_gpio, 1);
	msleep(20);
	mutex_unlock(&bias_lock);
	return 0;
}

static void set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(lcm_util));
}

static void get_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(*params));
	params->type = LCM_TYPE_DSI;
	params->width = 384;
	params->height = 854;
	params->dsi.mode = SYNC_EVENT_VDO_MODE;
	params->dsi.LANE_NUM = LCM_THREE_LANE;
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;
	params->dsi.intermediat_buffer_num = 2;
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.packet_size = 256;
	params->dsi.vertical_sync_active = 2;
	params->dsi.vertical_backporch = 20;
	params->dsi.vertical_frontporch = 170;
	params->dsi.vertical_active_line = 854;
	params->dsi.horizontal_sync_active = 20;
	params->dsi.horizontal_backporch = 180;
	params->dsi.horizontal_frontporch = 180;
	params->dsi.horizontal_active_pixel = 384;
	params->dsi.PLL_CLOCK = 210;
}

static void lcm_init(void)
{
	pr_info("[LCM/AXS15260] init\n");
	if (bias_enable())
		return;
	/* Display and touch share this TDDI; this is a reset, not just a delay. */
	if (reset_touch())
		return;
	reset_panel();
	push_table(init_setting, ARRAY_SIZE(init_setting));
}

static void lcm_suspend(void)
{
	pr_info("[LCM/AXS15260] suspend\n");
	push_table(suspend_setting, ARRAY_SIZE(suspend_setting));
	(lcm_util.mdelay)(120);
	disp_dts_gpio_select_state(DTS_GPIO_STATE_LCD_BIAS_ENP0);
	(lcm_util.mdelay)(20);
}

static unsigned int compare_id(void)
{
	u8 id[3] = { 0 };
	unsigned int request = 0x00013700;

	if (bias_enable())
		return 0;
	reset_panel();
	lcm_util.dsi_set_cmdq(&request, 1, 1);
	(lcm_util.mdelay)(10);
	lcm_util.dsi_dcs_read_lcm_reg_v2(0xda, &id[0], 1);
	lcm_util.dsi_dcs_read_lcm_reg_v2(0xdb, &id[1], 1);
	lcm_util.dsi_dcs_read_lcm_reg_v2(0xdc, &id[2], 1);
	return ((id[0] << 16) | (id[1] << 8) | id[2]) == 0x005260;
}

struct LCM_DRIVER axs15260_dsi_6580_hd_lcm_drv = {
	.name = "axs15260_dsi_6580_hd",
	.set_util_funcs = set_util_funcs,
	.get_params = get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_init,
	.compare_id = compare_id,
};

static int bias_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct gpio_desc *reset;

	/* The panel temporarily owns reset until the AXS touch driver is ported. */
	reset = devm_gpiod_get(&client->dev, "touch-reset", GPIOD_OUT_HIGH);
	if (IS_ERR(reset))
		return PTR_ERR(reset);
	mutex_lock(&bias_lock);
	if (bias_client) {
		mutex_unlock(&bias_lock);
		return -EBUSY;
	}
	bias_client = client;
	touch_reset_gpio = reset;
	mutex_unlock(&bias_lock);
	return 0;
}

static int bias_remove(struct i2c_client *client)
{
	mutex_lock(&bias_lock);
	if (bias_client == client) {
		bias_client = NULL;
		touch_reset_gpio = NULL;
	}
	mutex_unlock(&bias_lock);
	return 0;
}

static const struct of_device_id bias_of_match[] = {
	{ .compatible = "mediatek,I2C_LCD_BIAS" },
	{ }
};
MODULE_DEVICE_TABLE(of, bias_of_match);

static const struct i2c_device_id bias_ids[] = {
	{ "s17-lcd-bias", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bias_ids);

static struct i2c_driver bias_driver = {
	.driver = {
		.name = "s17-lcd-bias",
		.of_match_table = bias_of_match,
	},
	.probe = bias_probe,
	.remove = bias_remove,
	.id_table = bias_ids,
};
module_i2c_driver(bias_driver);

MODULE_DESCRIPTION("S17Promini AXS15260 panel and stock I2C bias control");
MODULE_LICENSE("GPL");
