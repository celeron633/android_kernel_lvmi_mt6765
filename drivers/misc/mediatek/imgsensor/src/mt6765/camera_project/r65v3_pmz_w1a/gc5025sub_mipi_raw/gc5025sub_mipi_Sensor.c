// SPDX-License-Identifier: GPL-2.0
/*
 * S17-specific GC5025sub port of the ZTE GC5025 driver.
 * Upstream commit b0c0f1ec4cb6d616090e7879a7df0785a1323c5d.
 * Sensor init, timing, crop, shutter and gain follow S17 stock Ghidra analysis.
 * See Documentation/S17Promini-gc5025sub.md.
 */

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "gc5025sub_mipi_Sensor.h"

#define PFX "GC5025_camera_sensor"
#define LOG_1 LOG_INF("S17 GC5025SUB,MIPI 2LANE\n")

#define LOG_INF(format, args...) pr_debug(PFX "[%s] " format, __func__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = GC5025SUB_SENSOR_ID,

	.checksum_value = 0xf7375923,

	.pre = {
		.pclk = 72000000,
		.linelength = 2400,
		.framelength = 988,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,

		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 72000000,

		.max_framerate = 300,
	},
	.cap = {
		.pclk = 72000000,
		.linelength = 2400,
		.framelength = 988,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 72000000,
		.max_framerate = 300,
	},
	.normal_video = {
		.pclk = 72000000,
		.linelength = 2400,
		.framelength = 988,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 72000000,
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 72000000,
		.linelength = 2400,
		.framelength = 988,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 72000000,
		.max_framerate = 300,
	},
	.slim_video = {
		.pclk = 72000000,
		.linelength = 2400,
		.framelength = 988,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 72000000,
		.max_framerate = 300,
	},
	.margin = 0,
	.min_shutter = 1,
	.max_frame_length = 0x3fff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,
	.ihdr_le_firstline = 0,
	.sensor_mode_num = 3,

	.cap_delay_frame = 2,
	.pre_delay_frame = 2,
	.video_delay_frame = 2,
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,

	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_R,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_2_LANE,
	.i2c_addr_table = {0x6e, 0xff},
};

static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x3ED,
	.gain = 0x40,
	.dummy_pixel = 0,
	.dummy_line = 0,
	.current_fps = 300,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_en = 0,
	.i2c_write_id = 0x6e,
};

static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] = {
	{ 2592, 1944, 0, 0, 2592, 1944, 2592, 1944, 0, 0, 2592, 1944, 0, 0,
	  2592, 1944 },
	{ 2592, 1944, 0, 0, 2592, 1944, 2592, 1944, 0, 0, 2592, 1944, 0, 0,
	  2592, 1944 },
	{ 2592, 1944, 0, 0, 2592, 1944, 2592, 1944, 0, 0, 2592, 1944, 0, 0,
	  2592, 1944 },
	{ 2592, 1944, 0, 0, 2592, 1944, 2592, 1944, 0, 0, 2592, 1944, 0, 0,
	  2592, 1944 },
	{ 2592, 1944, 0, 0, 2592, 1944, 2592, 1944, 0, 0, 2592, 1944, 0, 0,
	  2592, 1944 },
};

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[1] = { (char)(addr & 0xFF) };

	iReadRegI2C(pu_send_cmd, 1, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);
	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[2] = { (char)(addr & 0xFF), (char)(para & 0xFF) };

	iWriteRegI2C(pu_send_cmd, 2, imgsensor.i2c_write_id);
}

#define DD_PARAM_QTY 200
#define WINDOW_WIDTH 0x0a30
#define WINDOW_HEIGHT 0x079c
#define REG_ROM_START 0x62

struct otp_gc5025 {
	kal_uint16 dd_param_x[DD_PARAM_QTY];
	kal_uint16 dd_param_y[DD_PARAM_QTY];
	kal_uint16 dd_param_type[DD_PARAM_QTY];
	kal_uint16 dd_cnt;
	kal_uint16 dd_flag;
	kal_uint16 reg_addr[10];
	kal_uint16 reg_value[10];
	kal_uint16 reg_num;
};

static struct otp_gc5025 gc5025_otp_info;

enum otp_page {
	otp_page0 = 0,
	otp_page1,
};

enum otp_state {
	otp_close = 0,
	otp_open,
};

static kal_uint8 gc5025_read_otp(kal_uint8 addr)
{
	kal_uint8 value;
	kal_uint8 regd4;
	kal_uint16 realaddr = addr * 8;

	regd4 = read_cmos_sensor(0xd4);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xd4, (regd4 & 0xfc) + ((realaddr >> 8) & 0x03));
	write_cmos_sensor(0xd5, realaddr & 0xff);
	write_cmos_sensor(0xf3, 0x20);
	value = read_cmos_sensor(0xd7);

	return value;
}

static void gc5025_read_otp_group(kal_uint8 addr, kal_uint8 *buff, int size)
{
	kal_uint8 i;
	kal_uint8 regd4;
	kal_uint16 realaddr = addr * 8;

	regd4 = read_cmos_sensor(0xd4);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xd4, (regd4 & 0xfc) + ((realaddr >> 8) & 0x03));
	write_cmos_sensor(0xd5, realaddr);
	write_cmos_sensor(0xf3, 0x20);
	write_cmos_sensor(0xf3, 0x88);

	for (i = 0; i < size; i++)
		buff[i] = read_cmos_sensor(0xd7);
}

static void gc5025_select_page_otp(enum otp_page otp_select_page)
{
	kal_uint8 page;

	write_cmos_sensor(0xfe, 0x00);
	page = read_cmos_sensor(0xd4);

	switch (otp_select_page) {
	case otp_page0:
		page = page & 0xfb;
		break;
	case otp_page1:
		page = page | 0x04;
		break;
	default:
		break;
	}

	mdelay(5);
	write_cmos_sensor(0xd4, page);
}

static void gc5025_gcore_read_otp_info(void)
{
	kal_uint8 flagdd, flag_chipversion;
	kal_uint8 i, j, cnt = 0;
	kal_uint16 check;
	kal_uint8 total_number = 0;
	kal_uint8 ddtempbuff[50 * 4];
	kal_uint8 ddchecksum;

	memset(&gc5025_otp_info, 0, sizeof(struct otp_gc5025));

	gc5025_select_page_otp(otp_page0);
	flagdd = gc5025_read_otp(0x00);
	LOG_INF("GC5025_OTP_DD : flag_dd = 0x%x\n", flagdd);
	flag_chipversion = gc5025_read_otp(0x7f);

	switch (flagdd & 0x03) {
	case 0x00:
		LOG_INF("GC5025_OTP_DD is Empty !!\n");
		gc5025_otp_info.dd_flag = 0x00;
		break;
	case 0x01:
		LOG_INF("GC5025_OTP_DD is Valid!!\n");
		total_number = gc5025_read_otp(0x01) + gc5025_read_otp(0x02);
		LOG_INF("GC5025_OTP : total_number = %d\n", total_number);
		if (total_number > ARRAY_SIZE(ddtempbuff) / 4) {
			pr_warn(PFX "invalid OTP defect count %u\n",
				total_number);
			gc5025_otp_info.dd_flag = 2;
			break;
		}
		if (total_number <= 31) {
			gc5025_read_otp_group(0x03, &ddtempbuff[0],
					      total_number * 4);
		} else {
			gc5025_read_otp_group(0x03, &ddtempbuff[0], 31 * 4);
			gc5025_select_page_otp(otp_page1);
			gc5025_read_otp_group(0x29, &ddtempbuff[31 * 4],
					      (total_number - 31) * 4);
		}

		gc5025_select_page_otp(otp_page1);
		ddchecksum = gc5025_read_otp(0x75);
		check = total_number;
		for (i = 0; i < 4 * total_number; i++)
			check += ddtempbuff[i];
		if ((check % 255 + 1) == ddchecksum) {
			LOG_INF("GC5025_OTP_DD:DD checksum correct!checksum = 0x%x\n",
				ddchecksum);
		} else {
			LOG_INF("GC5025_OTP_DD:DD check sum error!otpchecksum = 0x%x, sum = 0x%x\n",
				ddchecksum, (check % 255 + 1));
		}

		for (i = 0; i < total_number; i++) {
			LOG_INF("GC5025_OTP_DD:index = %d,data[0] = %x,data[1] = %x,data[2]=%x,data[3]=%x\n",
				i, ddtempbuff[4 * i], ddtempbuff[4 * i + 1],
				ddtempbuff[4 * i + 2], ddtempbuff[4 * i + 3]);

			if (ddtempbuff[4 * i + 3] & 0x10) {
				switch (ddtempbuff[4 * i + 3] & 0x0f) {
				case 3:
					for (j = 0; j < 4; j++) {
						gc5025_otp_info.dd_param_x[cnt] =
							(((kal_uint16)ddtempbuff
								  [4 * i + 1] &
							  0x0f)
							 << 8) +
							ddtempbuff[4 * i];
						gc5025_otp_info.dd_param_y[cnt] =
							((kal_uint16)ddtempbuff
								 [4 * i + 2]
							 << 4) +
							((ddtempbuff[4 * i + 1] &
							  0xf0) >>
							 4) +
							j;
						gc5025_otp_info
							.dd_param_type[cnt++] =
							2;
					}
					break;
				case 4:
					for (j = 0; j < 2; j++) {
						gc5025_otp_info.dd_param_x[cnt] =
							(((kal_uint16)ddtempbuff
								  [4 * i + 1] &
							  0x0f)
							 << 8) +
							ddtempbuff[4 * i];
						gc5025_otp_info.dd_param_y[cnt] =
							((kal_uint16)ddtempbuff
								 [4 * i + 2]
							 << 4) +
							((ddtempbuff[4 * i + 1] &
							  0xf0) >>
							 4) +
							j;
						gc5025_otp_info
							.dd_param_type[cnt++] =
							2;
					}
					break;
				default:
					gc5025_otp_info.dd_param_x[cnt] =
						(((kal_uint16)
							  ddtempbuff[4 * i + 1] &
						  0x0f)
						 << 8) +
						ddtempbuff[4 * i];
					gc5025_otp_info.dd_param_y[cnt] =
						((kal_uint16)
							 ddtempbuff[4 * i + 2]
						 << 4) +
						((ddtempbuff[4 * i + 1] &
						  0xf0) >>
						 4);
					gc5025_otp_info.dd_param_type[cnt++] =
						ddtempbuff[4 * i + 3] & 0x0f;
					break;
				}
			} else {
				LOG_INF("GC5025_OTP_DD:check_id[%d] = %x,checkid error!!\n",
					i, ddtempbuff[4 * i + 3] & 0xf0);
			}
		}

		gc5025_otp_info.dd_cnt = cnt;
		gc5025_otp_info.dd_flag = 0x01;
		break;
	case 0x02:
	case 0x03:
		LOG_INF("GC5025_OTP_DD is Invalid !!\n");
		gc5025_otp_info.dd_flag = 0x02;
		break;
	default:
		break;
	}

	LOG_INF("GC5025_OTP_CHIPVESION:flag_chipversion = 0x%x\n",
		flag_chipversion);

	switch ((flag_chipversion >> 4) & 0x03) {
	case 0x00:
		LOG_INF("GC5025_OTP_CHIPVERSION is Empty !!\n");
		break;
	case 0x01:
		LOG_INF("GC5025_OTP_CHIPVERSION is Valid !!\n");
		gc5025_select_page_otp(otp_page1);
		i = 0;
		do {
			gc5025_otp_info.reg_addr[i] =
				gc5025_read_otp(REG_ROM_START + i * 2);
			gc5025_otp_info.reg_value[i] =
				gc5025_read_otp(REG_ROM_START + i * 2 + 1);
			LOG_INF("GC5025_OTP_CHIPVERSION reg_addr[%d] = 0x%x,reg_value[%d] = 0x%x\n",
				i, gc5025_otp_info.reg_addr[i], i,
				gc5025_otp_info.reg_value[i]);
			i++;
		} while ((gc5025_otp_info.reg_addr[i - 1] != 0) && (i < 10));
		gc5025_otp_info.reg_num = i - 1;
		break;
	case 0x02:
		LOG_INF("GC5025_OTP_CHIPVERSION is Invalid !!\n");
		break;
	default:
		break;
	}
}

static void gc5025_gcore_update_dd(void)
{
	unsigned int i, j;
	kal_uint16 x, y, type;

	if (gc5025_otp_info.dd_flag != 1)
		return;
	/* S17's stock H/V-flipped defect coordinates. */
	for (i = 0; i < gc5025_otp_info.dd_cnt; i++) {
		type = gc5025_otp_info.dd_param_type[i];
		gc5025_otp_info.dd_param_x[i] = WINDOW_WIDTH -
						gc5025_otp_info.dd_param_x[i] +
						(type == 0 ? 1 :
						 type == 1 ? -1 :
								   0);
		gc5025_otp_info.dd_param_y[i] =
			WINDOW_HEIGHT - gc5025_otp_info.dd_param_y[i] + 1;
	}
	/* Sort by row, then column, without reading the slot after the array. */
	for (i = 1; i < gc5025_otp_info.dd_cnt; i++) {
		x = gc5025_otp_info.dd_param_x[i];
		y = gc5025_otp_info.dd_param_y[i];
		type = gc5025_otp_info.dd_param_type[i];
		j = i;
		while (j && (gc5025_otp_info.dd_param_y[j - 1] > y ||
			     (gc5025_otp_info.dd_param_y[j - 1] == y &&
			      gc5025_otp_info.dd_param_x[j - 1] > x))) {
			gc5025_otp_info.dd_param_x[j] =
				gc5025_otp_info.dd_param_x[j - 1];
			gc5025_otp_info.dd_param_y[j] =
				gc5025_otp_info.dd_param_y[j - 1];
			gc5025_otp_info.dd_param_type[j] =
				gc5025_otp_info.dd_param_type[j - 1];
			j--;
		}
		gc5025_otp_info.dd_param_x[j] = x;
		gc5025_otp_info.dd_param_y[j] = y;
		gc5025_otp_info.dd_param_type[j] = type;
	}
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0xa8, 0x00);
	write_cmos_sensor(0x9d, 0x04);
	write_cmos_sensor(0xbe, 0x00);
	write_cmos_sensor(0xa9, 0x01);
	for (i = 0; i < gc5025_otp_info.dd_cnt; i++) {
		x = gc5025_otp_info.dd_param_x[i];
		y = gc5025_otp_info.dd_param_y[i];
		write_cmos_sensor(0xaa, i);
		write_cmos_sensor(0xac, x & 0xff);
		write_cmos_sensor(0xac, ((y & 0xf) << 4) | ((x >> 8) & 0xf));
		write_cmos_sensor(0xac, (y >> 4) & 0xff);
		write_cmos_sensor(0xac, gc5025_otp_info.dd_param_type[i]);
	}
	write_cmos_sensor(0xbe, 0x01);
	write_cmos_sensor(0xfe, 0x00);
}

static void gc5025_gcore_update_chipversion(void)
{
	kal_uint8 i;

	LOG_INF("GC5025_OTP_UPDATE_CHIPVERSION:reg_num = %d\n",
		gc5025_otp_info.reg_num);

	write_cmos_sensor(0xfe, 0x00);

	for (i = 0; i < gc5025_otp_info.reg_num; i++) {
		write_cmos_sensor(gc5025_otp_info.reg_addr[i],
				  gc5025_otp_info.reg_value[i]);
		LOG_INF("GC5025_OTP_UPDATE_CHIP_VERSION:{0x%x,0x%x}!!\n",
			gc5025_otp_info.reg_addr[i],
			gc5025_otp_info.reg_value[i]);
	}

	write_cmos_sensor(0xfe, 0x00);
}

static void gc5025_gcore_update_otp(void)
{
	gc5025_gcore_update_dd();
	gc5025_gcore_update_chipversion();
}

static void gc5025_gcore_enable_otp(enum otp_state state)
{
	kal_uint8 otp_clk, otp_en;

	otp_clk = read_cmos_sensor(0xfa);
	otp_en = read_cmos_sensor(0xd4);
	if (state) {
		otp_clk = otp_clk | 0x10;
		otp_en = otp_en | 0x80;
		mdelay(5);
		write_cmos_sensor(0xfa, otp_clk);
		write_cmos_sensor(0xd4, otp_en);

		LOG_INF("GC5025_OTP: Enable OTP!\n");
	} else {
		otp_en = otp_en & 0x7f;
		otp_clk = otp_clk & 0xef;
		mdelay(5);
		write_cmos_sensor(0xd4, otp_en);
		write_cmos_sensor(0xfa, otp_clk);
		LOG_INF("GC5025_OTP: Disable OTP!\n");
	}
}

static void gc5025_gcore_initial_otp(void)
{
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xf7, 0x01);
	write_cmos_sensor(0xf8, 0x11);
	write_cmos_sensor(0xf9, 0x00);
	write_cmos_sensor(0xfa, 0xa0);
	write_cmos_sensor(0xfc, 0x2e);
}

static void gc5025_gcore_identify_otp(void)
{
	gc5025_gcore_initial_otp();
	gc5025_gcore_enable_otp(otp_open);
	gc5025_gcore_read_otp_info();
	gc5025_gcore_enable_otp(otp_close);
}

static void set_dummy(void)
{
	/* Stock tracks frame length in software; it does not write VB here. */
}

static kal_uint32 return_sensor_id(void)
{
	return ((read_cmos_sensor(0xf0) << 8) | read_cmos_sensor(0xf1)) + 1;
}

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;

	if (!framerate)
		return;

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;

	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ?
					       frame_length :
					       imgsensor.min_frame_length;
	imgsensor.dummy_line =
		imgsensor.frame_length - imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line =
			imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}

static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	kal_uint32 raw_shutter;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	imgsensor.frame_length =
		min_t(kal_uint32,
		      max_t(kal_uint32, shutter, imgsensor.min_frame_length),
		      0x3fff);
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	raw_shutter =
		min_t(kal_uint32, max_t(kal_uint32, shutter, 1) * 2, 0x3ffe);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x03, (raw_shutter >> 8) & 0x3f);
	write_cmos_sensor(0x04, raw_shutter & 0xff);
}

#define ANALOG_GAIN_1 64
#define ANALOG_GAIN_2 92

static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint32 reg_gain = max_t(kal_uint32, gain, 64);

	write_cmos_sensor(0xfe, 0x00);
	if (reg_gain < 92) {
		write_cmos_sensor(0xb6, 0x00);
	} else {
		write_cmos_sensor(0xb6, 0x01);
		reg_gain = 64 * reg_gain / 92;
	}
	write_cmos_sensor(0xb1, reg_gain >> 6);
	write_cmos_sensor(0xb2, (reg_gain << 2) & 0xfc);
	return gain;
}

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se,
				    kal_uint16 gain)
{
	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n", le, se, gain);
}

static void night_mode(kal_bool enable)
{
}

static void sensor_init(void)
{
	/* S17 stock init; mode changes do not reprogram the sensor. */
	write_cmos_sensor(0xfe, 0x80);
	write_cmos_sensor(0xfe, 0x80);
	write_cmos_sensor(0xfe, 0x80);
	write_cmos_sensor(0xf7, 0x01);
	write_cmos_sensor(0xf8, 0x11);
	write_cmos_sensor(0xf9, 0x00);
	write_cmos_sensor(0xfa, 0xa0);
	write_cmos_sensor(0xfc, 0x2a);
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x01, 0x07);
	write_cmos_sensor(0xfc, 0x2e);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x88, 0x03);
	write_cmos_sensor(0xe7, 0xcc);
	write_cmos_sensor(0x03, 0x07);
	write_cmos_sensor(0x04, 0x08);
	write_cmos_sensor(0x05, 0x02);
	write_cmos_sensor(0x06, 0x58);
	write_cmos_sensor(0x08, 0x08);
	write_cmos_sensor(0x0a, 0x1c);
	write_cmos_sensor(0x0c, 0x04);
	write_cmos_sensor(0x0d, 0x07);
	write_cmos_sensor(0x0e, 0x9c);
	write_cmos_sensor(0x0f, 0x0a);
	write_cmos_sensor(0x10, 0x30);
	write_cmos_sensor(0x17, 0xc3);
	write_cmos_sensor(0x18, 0x02);
	write_cmos_sensor(0x19, 0x17);
	write_cmos_sensor(0x1a, 0x1a);
	write_cmos_sensor(0x1c, 0x1c);
	write_cmos_sensor(0x1e, 0x90);
	write_cmos_sensor(0x1f, 0xb0);
	write_cmos_sensor(0x20, 0x2b);
	write_cmos_sensor(0x21, 0x2b);
	write_cmos_sensor(0x26, 0x2b);
	write_cmos_sensor(0x25, 0xc1);
	write_cmos_sensor(0x27, 0x64);
	write_cmos_sensor(0x28, 0x00);
	write_cmos_sensor(0x29, 0x3f);
	write_cmos_sensor(0x2b, 0x80);
	write_cmos_sensor(0x2f, 0x4a);
	write_cmos_sensor(0x30, 0x11);
	write_cmos_sensor(0x31, 0x20);
	write_cmos_sensor(0x32, 0xa0);
	write_cmos_sensor(0x33, 0x00);
	write_cmos_sensor(0x34, 0x55);
	write_cmos_sensor(0x38, 0x02);
	write_cmos_sensor(0x39, 0x00);
	write_cmos_sensor(0x3a, 0x00);
	write_cmos_sensor(0x3b, 0x00);
	write_cmos_sensor(0x3c, 0x02);
	write_cmos_sensor(0x3d, 0x02);
	write_cmos_sensor(0x81, 0x60);
	write_cmos_sensor(0xcb, 0x02);
	write_cmos_sensor(0xcd, 0xad);
	write_cmos_sensor(0xcf, 0x50);
	write_cmos_sensor(0xd0, 0xb3);
	write_cmos_sensor(0xd1, 0x18);
	write_cmos_sensor(0xd3, 0xcc);
	write_cmos_sensor(0xd9, 0xaa);
	write_cmos_sensor(0xdc, 0x03);
	write_cmos_sensor(0xdd, 0xaa);
	write_cmos_sensor(0xe0, 0x00);
	write_cmos_sensor(0xe1, 0x0a);
	write_cmos_sensor(0xe3, 0x2a);
	write_cmos_sensor(0xe4, 0xa0);
	write_cmos_sensor(0xe5, 0x06);
	write_cmos_sensor(0xe6, 0x10);
	write_cmos_sensor(0xe7, 0xc2);
	write_cmos_sensor(0x1d, 0x13);
	write_cmos_sensor(0x80, 0x10);
	write_cmos_sensor(0x89, 0x03);
	write_cmos_sensor(0xfe, 0x01);
	write_cmos_sensor(0x88, 0xf7);
	write_cmos_sensor(0x8a, 0x03);
	write_cmos_sensor(0x8e, 0xc7);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x40, 0x22);
	write_cmos_sensor(0x43, 0x03);
	write_cmos_sensor(0xae, 0x40);
	write_cmos_sensor(0x60, 0x00);
	write_cmos_sensor(0x61, 0x80);
	write_cmos_sensor(0xb0, 0x58);
	write_cmos_sensor(0xb1, 0x01);
	write_cmos_sensor(0xb2, 0x00);
	write_cmos_sensor(0xb6, 0x00);
	write_cmos_sensor(0x91, 0x00);
	write_cmos_sensor(0x92, 0x01);
	write_cmos_sensor(0x94, 0x02);
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x02, 0x01);
	write_cmos_sensor(0x03, 0x8e);
	write_cmos_sensor(0x06, 0x80);
	write_cmos_sensor(0x15, 0x00);
	write_cmos_sensor(0x16, 0x09);
	write_cmos_sensor(0x18, 0x0a);
	write_cmos_sensor(0x21, 0x10);
	write_cmos_sensor(0x22, 0x05);
	write_cmos_sensor(0x23, 0x20);
	write_cmos_sensor(0x24, 0x02);
	write_cmos_sensor(0x25, 0x20);
	write_cmos_sensor(0x26, 0x08);
	write_cmos_sensor(0x29, 0x06);
	write_cmos_sensor(0x2a, 0x0a);
	write_cmos_sensor(0x2b, 0x08);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x3f, 0x91);
}

static void preview_setting(void)
{
	LOG_INF("E!\n");
}

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n", currefps);
}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n", currefps);
}

static void hs_video_setting(void)
{
	LOG_INF("E\n");
}

static void slim_video_setting(void)
{
	LOG_INF("E\n");
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {
		write_cmos_sensor(0xfe, 0x00);
		write_cmos_sensor(0x8c, 0x11);
	} else {
		write_cmos_sensor(0xfe, 0x00);
		write_cmos_sensor(0x8c, 0x10);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			if (*sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, *sensor_id);
				return ERROR_NONE;
			}
			LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n",
				imgsensor.i2c_write_id, *sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}

static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint32 sensor_id = 0;

	LOG_1;

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}
			LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n",
				imgsensor.i2c_write_id, sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	gc5025_gcore_identify_otp();

	sensor_init();

	gc5025_gcore_update_otp();

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}

static kal_uint32 close(void)
{
	LOG_INF("E\n");

	return ERROR_NONE;
}

static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;

	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();

	return ERROR_NONE;
}

static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;

	if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
		pr_err("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
		       imgsensor.current_fps,
		       imgsensor_info.cap.max_framerate / 10);
	imgsensor.pclk = imgsensor_info.cap.pclk;
	imgsensor.line_length = imgsensor_info.cap.linelength;
	imgsensor.frame_length = imgsensor_info.cap.framelength;
	imgsensor.min_frame_length = imgsensor_info.cap.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;

	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps);
	return ERROR_NONE;
}

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			       MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("GC5025 normal video Entre\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;

	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);

	return ERROR_NONE;
}

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;

	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	return ERROR_NONE;
}

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			     MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	return ERROR_NONE;
}

static kal_uint32
get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth =
		imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight =
		imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth =
		imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight =
		imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth =
		imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight =
		imgsensor_info.normal_video.grabwindow_height;

	sensor_resolution->SensorHighSpeedVideoWidth =
		imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight =
		imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth =
		imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight =
		imgsensor_info.slim_video.grabwindow_height;
	return ERROR_NONE;
}

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4;
	sensor_info->SensorResetActiveHigh = FALSE;
	sensor_info->SensorResetDelayCount = 5;

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat =
		imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame =
		imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame =
		imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0;
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;

	sensor_info->AESensorGainDelayFrame =
		imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame =
		imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3;
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2;
	sensor_info->SensorPixelClockCount = 3;
	sensor_info->SensorDataLatchCount = 2;

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;
	sensor_info->SensorHightSampling = 0;
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.cap.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		sensor_info->SensorGrabStartX =
			imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY =
			imgsensor_info.normal_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX =
			imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY =
			imgsensor_info.slim_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;
		break;
	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			  MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s:scenario_id = %d\n", __func__, scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		preview(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		capture(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		normal_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		hs_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		slim_video(image_window, sensor_config_data);
		break;
	default:
		LOG_INF("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}

static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);

	if (framerate == 0)

		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable)
		imgsensor.autoflicker_en = KAL_TRUE;
	else
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32
set_max_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			      MUINT32 framerate)
{
	kal_uint32 frame_length;

	if (!framerate)
		return ERROR_NONE;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length = imgsensor_info.pre.pclk / framerate * 10 /
			       imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.pre.framelength) ?
				      (frame_length -
				 imgsensor_info.pre.framelength) :
				      0;
		imgsensor.frame_length =
			imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.normal_video.pclk / framerate *
			       10 / imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length >
			 imgsensor_info.normal_video.framelength) ?
				      (frame_length -
				 imgsensor_info.normal_video.framelength) :
				      0;
		imgsensor.frame_length =
			imgsensor_info.normal_video.framelength +
			imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:

		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			LOG_INF("current_fps %d fps is not support, so use cap's setting: %d fps!\n",
				framerate,
				imgsensor_info.cap.max_framerate / 10);
		frame_length = imgsensor_info.cap.pclk / framerate * 10 /
			       imgsensor_info.cap.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.cap.framelength) ?
				      (frame_length -
				 imgsensor_info.cap.framelength) :
				      0;
		imgsensor.frame_length =
			imgsensor_info.cap.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);

		set_dummy();
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10 /
			       imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.hs_video.framelength) ?
				      (frame_length -
				 imgsensor_info.hs_video.framelength) :
				      0;
		imgsensor.frame_length = imgsensor_info.hs_video.framelength +
					 imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10 /
			       imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.slim_video.framelength) ?
				      (frame_length -
				 imgsensor_info.slim_video.framelength) :
				      0;
		imgsensor.frame_length = imgsensor_info.slim_video.framelength +
					 imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	default:
		frame_length = imgsensor_info.pre.pclk / framerate * 10 /
			       imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.pre.framelength) ?
				      (frame_length -
				 imgsensor_info.pre.framelength) :
				      0;
		imgsensor.frame_length =
			imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		LOG_INF("error scenario_id = %d, we use preview scenario\n",
			scenario_id);
		break;
	}
	return ERROR_NONE;
}

static kal_uint32
get_default_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id,
				  MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*framerate = imgsensor_info.pre.max_framerate;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*framerate = imgsensor_info.normal_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*framerate = imgsensor_info.cap.max_framerate;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		*framerate = imgsensor_info.slim_video.max_framerate;
		break;
	default:
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
				  UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *)feature_para;
	UINT16 *feature_data_16 = (UINT16 *)feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *)feature_para;
	UINT32 *feature_data_32 = (UINT32 *)feature_para;
	unsigned long long *feature_data = (unsigned long long *)feature_para;

	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
		(MSDK_SENSOR_REG_INFO_STRUCT *)feature_para;

	LOG_INF("feature_id = %d\n", feature_id);
	switch (feature_id) {
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		night_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16)*feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor(sensor_reg_data->RegAddr,
				  sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData =
			read_cmos_sensor(sensor_reg_data->RegAddr);
		LOG_INF("adb_i2c_read 0x%x = 0x%x\n", sensor_reg_data->RegAddr,
			sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:

		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL)*feature_data_16,
				      *(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM) *
						      feature_data,
					      *(feature_data + 1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario(
			(enum MSDK_SCENARIO_ID_ENUM)*(feature_data),
			(MUINT32 *)(uintptr_t)(*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_INF("current fps :%d\n", (UINT32)*feature_data);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = *feature_data;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR:
		LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.ihdr_en = (BOOL)*feature_data;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
			(UINT32)*feature_data);
		wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(
			feature_data + 1));
		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo,
			       (void *)&imgsensor_winsize_info[1],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo,
			       (void *)&imgsensor_winsize_info[2],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo,
			       (void *)&imgsensor_winsize_info[3],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo,
			       (void *)&imgsensor_winsize_info[4],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo,
			       (void *)&imgsensor_winsize_info[0],
			       sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data + 1),
			(UINT16)*(feature_data + 2));
		ihdr_write_shutter_gain((UINT16)*feature_data,
					(UINT16)*(feature_data + 1),
					(UINT16)*(feature_data + 2));
		break;
	default:
		break;
	}

	return ERROR_NONE;
}

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open, get_info, get_resolution, feature_control, control, close
};

UINT32 GC5025SUB_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}
