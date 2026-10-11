// SPDX-License-Identifier: GPL-2.0
/*
 * IMX278 MediaTek driver ported from h0sch180/android_kernel_cubot_max
 * commit 7450eaa6b7662499002982526e81973ca0e5f433 (MT6753).
 * S17 board settings recovered from its verified stock kernel.
 * See Documentation/S17Promini-imx278.md for provenance and validation.
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

#include "imx278mipiraw_Sensor.h"

#define PFX "IMX278_camera_sensor"

#define LOG_1 LOG_INF("IMX278,MIPI 4LANE\n")
#define LOG_2 LOG_INF("S17 preview/capture/video 2100x1558 at 30fps\n")

#define LOG_INF(format, args...) pr_debug(PFX "[%s] " format, __func__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = IMX278_SENSOR_ID,

	.checksum_value = 0x6b3a163d,
	.pre = {
		.pclk = 480000000,
		.linelength = 4976,
		.framelength = 3200,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2100,
		.grabwindow_height = 1558,

		.mipi_data_lp2hs_settle_dc = 85,

		.max_framerate = 300,
	},
	.cap = {
		.pclk = 480000000,
		.linelength = 4976,
		.framelength = 3200,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2100,
		.grabwindow_height = 1558,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.cap1 = {
		.pclk = 388000000,
		.linelength = 4976,
		.framelength = 3220,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2100,
		.grabwindow_height = 1558,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 240,
	},
	.normal_video = {
		.pclk = 480000000,
		.linelength = 4976,
		.framelength = 3200,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2100,
		.grabwindow_height = 1558,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 480000000,
		.linelength = 4976,
		.framelength = 800,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1316,
		.grabwindow_height = 480,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 1200,
	},
	.slim_video = {
		.pclk = 184000000,
		.linelength = 4976,
		.framelength = 1228,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1400,
		.grabwindow_height = 782,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.margin = 10,
	.min_shutter = 1,
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,
	.ihdr_le_firstline = 0,
	.sensor_mode_num = 10,

	.cap_delay_frame = 2,
	.pre_delay_frame = 2,
	.video_delay_frame = 3,
	.hs_video_delay_frame = 3,
	.slim_video_delay_frame = 3,

	.isp_driving_current = ISP_DRIVING_8MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x34, 0x20, 0x40, 0xff},
	.i2c_speed = 300,
};

static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_HV_MIRROR,
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x3D0,
	.gain = 0x100,
	.dummy_pixel = 0,
	.dummy_line = 0,
	.current_fps = 300,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_en = 0,
	.i2c_write_id = 0x34,

};

static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] = {
	{ 4208, 3120, 0, 0, 4208, 3120, 2104, 1560, 2, 0, 2100, 1558, 0, 0,
	  2100, 1558 },
	{ 4208, 3120, 0, 0, 4208, 3120, 2104, 1560, 2, 0, 2100, 1558, 0, 0,
	  2100, 1558 },
	{ 4208, 3120, 0, 0, 4208, 3120, 2104, 1560, 2, 0, 2100, 1558, 0, 0,
	  2100, 1558 },
	{ 4208, 3120, 0, 792, 4208, 2328, 1320, 732, 2, 4, 1316, 480, 0, 0,
	  1316, 480 },
	{ 4208, 3120, 0, 384, 4208, 2736, 1402, 782, 2, 0, 1400, 782, 0, 0,
	  1400, 782 },
};

#define MaxGainIndex (104)
static kal_uint16 sensorGainMapping[MaxGainIndex][2] = {
	{ 71, 53 },   { 72, 57 },   { 73, 65 },	  { 74, 69 },	{ 75, 73 },
	{ 76, 80 },   { 78, 91 },   { 79, 98 },	  { 80, 101 },	{ 81, 108 },
	{ 82, 111 },  { 83, 117 },  { 84, 120 },  { 85, 126 },	{ 86, 132 },
	{ 89, 143 },  { 90, 148 },  { 94, 163 },  { 96, 170 },	{ 99, 181 },
	{ 102, 191 }, { 108, 207 }, { 113, 222 }, { 116, 230 }, { 117, 232 },
	{ 119, 236 }, { 120, 239 }, { 122, 244 }, { 123, 245 }, { 124, 248 },
	{ 125, 249 }, { 129, 258 }, { 131, 262 }, { 133, 266 }, { 134, 268 },
	{ 135, 269 }, { 136, 272 }, { 142, 281 }, { 143, 282 }, { 144, 284 },
	{ 145, 286 }, { 146, 288 }, { 147, 288 }, { 148, 290 }, { 149, 292 },
	{ 150, 294 }, { 151, 295 }, { 152, 297 }, { 153, 299 }, { 154, 300 },
	{ 155, 301 }, { 156, 302 }, { 157, 304 }, { 160, 307 }, { 169, 318 },
	{ 175, 325 }, { 180, 330 }, { 185, 335 }, { 193, 342 }, { 197, 346 },
	{ 198, 347 }, { 206, 353 }, { 208, 355 }, { 216, 360 }, { 221, 364 },
	{ 223, 365 }, { 228, 368 }, { 231, 370 }, { 235, 373 }, { 241, 376 },
	{ 243, 377 }, { 250, 381 }, { 254, 383 }, { 264, 388 }, { 266, 389 },
	{ 270, 391 }, { 275, 393 }, { 280, 395 }, { 284, 397 }, { 290, 399 },
	{ 298, 402 }, { 300, 403 }, { 306, 405 }, { 312, 407 }, { 318, 409 },
	{ 324, 411 }, { 330, 413 }, { 337, 415 }, { 345, 417 }, { 352, 419 },
	{ 360, 421 }, { 368, 423 }, { 377, 425 }, { 385, 427 }, { 394, 429 },
	{ 404, 431 }, { 414, 433 }, { 425, 435 }, { 437, 437 }, { 448, 439 },
	{ 468, 442 }, { 475, 443 }, { 489, 445 }, { 504, 447 }
};

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

	kdSetI2CSpeed(imgsensor_info.i2c_speed);
	iReadRegI2C(pu_send_cmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = { (char)(addr >> 8), (char)(addr & 0xFF),
				(char)(para & 0xFF) };

	kdSetI2CSpeed(imgsensor_info.i2c_speed);
	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d\n", imgsensor.dummy_line,
		imgsensor.dummy_pixel);

	write_cmos_sensor(0x0104, 0x01);
	write_cmos_sensor(0x0340, (imgsensor.frame_length >> 8) & 0xFF);
	write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
	write_cmos_sensor(0x0342, (imgsensor.line_length >> 8) & 0xFF);
	write_cmos_sensor(0x0343, imgsensor.line_length & 0xFF);
	write_cmos_sensor(0x0104, 0x00);
}

static kal_uint32 return_sensor_id(void)
{
	write_cmos_sensor(0x0A02, 0x13);
	write_cmos_sensor(0x0A00, 0x01);
	write_cmos_sensor(0x0A01, 0x01);
	return (((read_cmos_sensor(0x0A28) << 8) |
		 (read_cmos_sensor(0x0A29) & 0xF0)) >>
		4);
}

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;

	if (!framerate)
		return;

	LOG_INF("framerate = %d, min framelength should enable?\n", framerate,
		min_framelength_en);

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

static void write_shutter(kal_uint16 shutter)
{
	kal_uint16 realtime_fps = 0;
	kal_uint32 frame_length = 0;

	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ?
				imgsensor_info.min_shutter :
				shutter;
	shutter = (shutter >
		   (imgsensor_info.max_frame_length - imgsensor_info.margin)) ?
				(imgsensor_info.max_frame_length -
			   imgsensor_info.margin) :
				shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 /
			       imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			write_cmos_sensor(0x0104, 0x01);
			write_cmos_sensor(0x0340,
					  (imgsensor.frame_length >> 8) & 0xFF);
			write_cmos_sensor(0x0341,
					  imgsensor.frame_length & 0xFF);
			write_cmos_sensor(0x0104, 0x00);
		}
	} else {
		write_cmos_sensor(0x0104, 0x01);
		write_cmos_sensor(0x0340, (imgsensor.frame_length >> 8) & 0xFF);
		write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor(0x0104, 0x00);
	}

	write_cmos_sensor(0x0104, 0x01);
	write_cmos_sensor(0x0202, (shutter >> 8) & 0xFF);
	write_cmos_sensor(0x0203, shutter & 0xFF);
	write_cmos_sensor(0x0104, 0x00);
	LOG_INF("shutter =%d, framelength =%d\n", shutter,
		imgsensor.frame_length);
}

static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint8 iI;

	for (iI = 0; iI < (MaxGainIndex - 1); iI++) {
		if (gain <= sensorGainMapping[iI][0])
			break;
	}

	return sensorGainMapping[iI][1];
}

static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

	if (gain < BASEGAIN || gain > 32 * BASEGAIN) {
		LOG_INF("Error gain setting");

		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 32 * BASEGAIN)
			gain = 32 * BASEGAIN;
	}

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor(0x0104, 0x01);
	write_cmos_sensor(0x0204, (reg_gain >> 8) & 0xFF);
	write_cmos_sensor(0x0205, reg_gain & 0xFF);
	write_cmos_sensor(0x0104, 0x00);

	return gain;
}

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se,
				    kal_uint16 gain)
{
	/* S17 stock does not implement interleaved HDR. */
}

static void night_mode(kal_bool enable)
{
}

static void sensor_init(void)
{
	write_cmos_sensor(0x0136, 0x18);
	write_cmos_sensor(0x0137, 0x00);
	write_cmos_sensor(0x3042, 0x01);
	write_cmos_sensor(0x5ce8, 0x00);
	write_cmos_sensor(0x5ce9, 0x91);
	write_cmos_sensor(0x5cea, 0x00);
	write_cmos_sensor(0x5ceb, 0x2a);
	write_cmos_sensor(0x5f1b, 0x01);
	write_cmos_sensor(0x5c2c, 0x01);
	write_cmos_sensor(0x5c2d, 0xff);
	write_cmos_sensor(0x5c2e, 0x00);
	write_cmos_sensor(0x5c2f, 0x00);
	write_cmos_sensor(0x5f0d, 0x6e);
	write_cmos_sensor(0x5f0e, 0x7c);
	write_cmos_sensor(0x5f0f, 0x14);
	write_cmos_sensor(0x6100, 0x30);
	write_cmos_sensor(0x6101, 0x12);
	write_cmos_sensor(0x6102, 0x14);
	write_cmos_sensor(0x6104, 0x91);
	write_cmos_sensor(0x6105, 0x30);
	write_cmos_sensor(0x6106, 0x11);
	write_cmos_sensor(0x6107, 0x12);
	write_cmos_sensor(0x6505, 0xc4);
	write_cmos_sensor(0x6507, 0x25);
	write_cmos_sensor(0x6508, 0xe1);
	write_cmos_sensor(0x6509, 0xd7);
	write_cmos_sensor(0x650a, 0x20);
	write_cmos_sensor(0x7382, 0x01);
	write_cmos_sensor(0x7383, 0x13);
	write_cmos_sensor(0x7788, 0x04);
	write_cmos_sensor(0x9006, 0x0c);
	write_cmos_sensor(0xb000, 0x78);
	write_cmos_sensor(0xb001, 0xb6);
	write_cmos_sensor(0xb002, 0x78);
	write_cmos_sensor(0xb003, 0xb7);
	write_cmos_sensor(0xb004, 0x98);
	write_cmos_sensor(0xb005, 0x00);
	write_cmos_sensor(0xb006, 0x98);
	write_cmos_sensor(0xb007, 0x01);
	write_cmos_sensor(0xb008, 0x98);
	write_cmos_sensor(0xb009, 0x02);
	write_cmos_sensor(0xb00a, 0xa9);
	write_cmos_sensor(0xb00b, 0x0f);
	write_cmos_sensor(0xb00c, 0xa9);
	write_cmos_sensor(0xb00d, 0x12);
	write_cmos_sensor(0xb00e, 0xa9);
	write_cmos_sensor(0xb00f, 0x15);
	write_cmos_sensor(0xb010, 0xa9);
	write_cmos_sensor(0xb011, 0x16);
	write_cmos_sensor(0xb012, 0xa9);
	write_cmos_sensor(0xb013, 0x17);
	write_cmos_sensor(0xb014, 0xa9);
	write_cmos_sensor(0xb015, 0x18);
	write_cmos_sensor(0xb016, 0xa9);
	write_cmos_sensor(0xb017, 0x19);
	write_cmos_sensor(0x3020, 0x02);
	write_cmos_sensor(0x73a0, 0x21);
	write_cmos_sensor(0x73a1, 0x03);
	write_cmos_sensor(0x7709, 0x00);
	write_cmos_sensor(0x7786, 0x09);
	write_cmos_sensor(0x778a, 0x00);
	write_cmos_sensor(0x778b, 0x60);
	write_cmos_sensor(0x951e, 0x3d);
	write_cmos_sensor(0x951f, 0x05);
	write_cmos_sensor(0x9521, 0x12);
	write_cmos_sensor(0x9522, 0x02);
	write_cmos_sensor(0x9524, 0x00);
	write_cmos_sensor(0x9525, 0x31);
	write_cmos_sensor(0x9526, 0x00);
	write_cmos_sensor(0x9527, 0x04);
	write_cmos_sensor(0x952a, 0x00);
	write_cmos_sensor(0x952b, 0x7a);
	write_cmos_sensor(0x952c, 0x00);
	write_cmos_sensor(0x952d, 0x0b);
	write_cmos_sensor(0x9606, 0x00);
	write_cmos_sensor(0x9607, 0x90);
	write_cmos_sensor(0x9618, 0x00);
	write_cmos_sensor(0x9619, 0x52);
	write_cmos_sensor(0x961e, 0x00);
	write_cmos_sensor(0x961f, 0x1e);
	write_cmos_sensor(0x9620, 0x00);
	write_cmos_sensor(0x9621, 0x18);
	write_cmos_sensor(0x9622, 0x00);
	write_cmos_sensor(0x9623, 0x18);
	write_cmos_sensor(0x9706, 0x00);
	write_cmos_sensor(0x9707, 0x00);
	write_cmos_sensor(0x9718, 0x00);
	write_cmos_sensor(0x9719, 0x32);
	write_cmos_sensor(0x971a, 0x00);
	write_cmos_sensor(0x971b, 0x32);
	write_cmos_sensor(0x971c, 0x00);
	write_cmos_sensor(0x971d, 0x32);
	write_cmos_sensor(0x971e, 0x00);
	write_cmos_sensor(0x971f, 0x1e);
	write_cmos_sensor(0x9720, 0x00);
	write_cmos_sensor(0x9721, 0x20);
	write_cmos_sensor(0x9722, 0x00);
	write_cmos_sensor(0x9723, 0x20);
	write_cmos_sensor(0x7837, 0x00);
	write_cmos_sensor(0x7965, 0x0f);
	write_cmos_sensor(0x7966, 0x0a);
	write_cmos_sensor(0x7967, 0x07);
	write_cmos_sensor(0x7968, 0x0f);
	write_cmos_sensor(0x7969, 0x60);
	write_cmos_sensor(0x796b, 0x32);
	write_cmos_sensor(0xa85b, 0x10);
	write_cmos_sensor(0x924e, 0x42);
	write_cmos_sensor(0x9250, 0x78);
	write_cmos_sensor(0x9251, 0x3c);
	write_cmos_sensor(0x9252, 0x14);
	write_cmos_sensor(0x9332, 0x02);
	write_cmos_sensor(0x9333, 0x02);
	write_cmos_sensor(0x9335, 0x02);
	write_cmos_sensor(0x9336, 0x02);
	write_cmos_sensor(0x9357, 0x04);
	write_cmos_sensor(0x9359, 0x04);
	write_cmos_sensor(0x935a, 0x04);
	write_cmos_sensor(0x9809, 0x02);
	write_cmos_sensor(0x980a, 0x02);
	write_cmos_sensor(0x980b, 0x02);
	write_cmos_sensor(0x980d, 0x00);
	write_cmos_sensor(0x980e, 0x00);
	write_cmos_sensor(0x980f, 0x06);
	write_cmos_sensor(0x9812, 0x00);
	write_cmos_sensor(0x9813, 0x00);
	write_cmos_sensor(0x9814, 0x00);
	write_cmos_sensor(0x981b, 0x1e);
	write_cmos_sensor(0x981c, 0x23);
	write_cmos_sensor(0x981d, 0x23);
	write_cmos_sensor(0x981e, 0x28);
	write_cmos_sensor(0x981f, 0x55);
	write_cmos_sensor(0x9820, 0x55);
	write_cmos_sensor(0x9822, 0x1b);
	write_cmos_sensor(0x9823, 0x1b);
	write_cmos_sensor(0x9824, 0x0a);
	write_cmos_sensor(0x9825, 0x00);
	write_cmos_sensor(0x9826, 0x00);
	write_cmos_sensor(0x9827, 0x69);
	write_cmos_sensor(0x9828, 0xa0);
	write_cmos_sensor(0x9829, 0xa0);
	write_cmos_sensor(0x982a, 0x00);
	write_cmos_sensor(0x982b, 0x80);
	write_cmos_sensor(0x982c, 0x00);
	write_cmos_sensor(0x982d, 0x8c);
	write_cmos_sensor(0x982e, 0x00);
	write_cmos_sensor(0x982f, 0x8c);
	write_cmos_sensor(0x9830, 0x04);
	write_cmos_sensor(0x9831, 0x80);
	write_cmos_sensor(0x9832, 0x05);
	write_cmos_sensor(0x9833, 0x00);
	write_cmos_sensor(0x9834, 0x05);
	write_cmos_sensor(0x9835, 0x00);
	write_cmos_sensor(0x9836, 0x00);
	write_cmos_sensor(0x9837, 0x80);
	write_cmos_sensor(0x9838, 0x00);
	write_cmos_sensor(0x9839, 0x80);
	write_cmos_sensor(0x983a, 0x00);
	write_cmos_sensor(0x983b, 0x80);
	write_cmos_sensor(0x983c, 0x0e);
	write_cmos_sensor(0x983d, 0x01);
	write_cmos_sensor(0x983e, 0x01);
	write_cmos_sensor(0x983f, 0x0e);
	write_cmos_sensor(0x9840, 0x06);
	write_cmos_sensor(0x9845, 0x0e);
	write_cmos_sensor(0x9846, 0x00);
	write_cmos_sensor(0x9848, 0x0e);
	write_cmos_sensor(0x9849, 0x06);
	write_cmos_sensor(0x984a, 0x06);
	write_cmos_sensor(0x9871, 0x14);
	write_cmos_sensor(0x9872, 0x0e);
	write_cmos_sensor(0x9877, 0x7f);
	write_cmos_sensor(0x9878, 0x1e);
	write_cmos_sensor(0x9879, 0x09);
	write_cmos_sensor(0x987b, 0x0e);
	write_cmos_sensor(0x987c, 0x0e);
	write_cmos_sensor(0x988a, 0x13);
	write_cmos_sensor(0x988b, 0x13);
	write_cmos_sensor(0x9893, 0x13);
	write_cmos_sensor(0x9894, 0x13);
	write_cmos_sensor(0x9898, 0x75);
	write_cmos_sensor(0x9899, 0x2d);
	write_cmos_sensor(0x989a, 0x26);
	write_cmos_sensor(0x989e, 0x96);
	write_cmos_sensor(0x989f, 0x1e);
	write_cmos_sensor(0x98a0, 0x0d);
	write_cmos_sensor(0x98a1, 0x43);
	write_cmos_sensor(0x98a2, 0x0e);
	write_cmos_sensor(0x98a3, 0x03);
	write_cmos_sensor(0x98ab, 0x66);
	write_cmos_sensor(0x98ac, 0x66);
	write_cmos_sensor(0x98b1, 0x4d);
	write_cmos_sensor(0x98b2, 0x4d);
	write_cmos_sensor(0x98b4, 0x0d);
	write_cmos_sensor(0x98b5, 0x0d);
	write_cmos_sensor(0x98bc, 0x7a);
	write_cmos_sensor(0x98bd, 0x66);
	write_cmos_sensor(0x98be, 0x78);
	write_cmos_sensor(0x98c2, 0x66);
	write_cmos_sensor(0x98c3, 0x66);
	write_cmos_sensor(0x98c4, 0x62);
	write_cmos_sensor(0x98c6, 0x14);
	write_cmos_sensor(0x98ce, 0x7a);
	write_cmos_sensor(0x98cf, 0x78);
	write_cmos_sensor(0x98d0, 0x78);
	write_cmos_sensor(0x98d4, 0x66);
	write_cmos_sensor(0x98d5, 0x62);
	write_cmos_sensor(0x98d6, 0x62);
	write_cmos_sensor(0x9921, 0x0a);
	write_cmos_sensor(0x9922, 0x01);
	write_cmos_sensor(0x9923, 0x01);
	write_cmos_sensor(0x9928, 0xa0);
	write_cmos_sensor(0x9929, 0xa0);
	write_cmos_sensor(0x9949, 0x06);
	write_cmos_sensor(0x994a, 0x06);
	write_cmos_sensor(0x9999, 0x26);
	write_cmos_sensor(0x999a, 0x26);
	write_cmos_sensor(0x999f, 0x0d);
	write_cmos_sensor(0x99a0, 0x0d);
	write_cmos_sensor(0x99a2, 0x03);
	write_cmos_sensor(0x99a3, 0x03);
	write_cmos_sensor(0x99bd, 0x78);
	write_cmos_sensor(0x99be, 0x78);
	write_cmos_sensor(0x99c3, 0x62);
	write_cmos_sensor(0x99c4, 0x62);
	write_cmos_sensor(0x99cf, 0x78);
	write_cmos_sensor(0x99d0, 0x78);
	write_cmos_sensor(0x99d5, 0x62);
	write_cmos_sensor(0x99d6, 0x62);
	write_cmos_sensor(0xa900, 0x00);
	write_cmos_sensor(0xa901, 0x00);
	write_cmos_sensor(0xa90b, 0x00);
	write_cmos_sensor(0x9342, 0x04);
	write_cmos_sensor(0x934d, 0x04);
	write_cmos_sensor(0x934f, 0x04);
	write_cmos_sensor(0x9350, 0x04);
	write_cmos_sensor(0x3011, 0xff);
	write_cmos_sensor(0xaf00, 0x01);
	write_cmos_sensor(0xaf01, 0x00);
	write_cmos_sensor(0xaf02, 0x00);
	write_cmos_sensor(0xaf03, 0xdb);
	write_cmos_sensor(0xaf04, 0x01);
	write_cmos_sensor(0xaf05, 0x00);
	write_cmos_sensor(0xaf06, 0x01);
	write_cmos_sensor(0xaf07, 0xd2);
	write_cmos_sensor(0xaf08, 0x02);
	write_cmos_sensor(0xaf09, 0x3d);
	write_cmos_sensor(0xaf0a, 0x02);
	write_cmos_sensor(0xaf0b, 0x83);
	write_cmos_sensor(0x0101, 0x03);
}

static void preview_setting(void)
{
	write_cmos_sensor(0x0100, 0x00);
	write_cmos_sensor(0x0112, 0x0a);
	write_cmos_sensor(0x0113, 0x0a);
	write_cmos_sensor(0x0114, 0x03);
	write_cmos_sensor(0x0301, 0x05);
	write_cmos_sensor(0x0303, 0x02);
	write_cmos_sensor(0x0305, 0x0c);
	write_cmos_sensor(0x0306, 0x02);
	write_cmos_sensor(0x0307, 0x58);
	write_cmos_sensor(0x0309, 0x0a);
	write_cmos_sensor(0x030b, 0x01);
	write_cmos_sensor(0x030d, 0x0c);
	write_cmos_sensor(0x030e, 0x05);
	write_cmos_sensor(0x030f, 0x14);
	write_cmos_sensor(0x0310, 0x00);
	write_cmos_sensor(0x3041, 0x01);
	write_cmos_sensor(0x0342, 0x13);
	write_cmos_sensor(0x0343, 0x70);
	write_cmos_sensor(0x0340, 0x0c);
	write_cmos_sensor(0x0341, 0x80);
	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x00);
	write_cmos_sensor(0x0347, 0x00);
	write_cmos_sensor(0x0348, 0x10);
	write_cmos_sensor(0x0349, 0x6f);
	write_cmos_sensor(0x034a, 0x0c);
	write_cmos_sensor(0x034b, 0x2f);
	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x01);
	write_cmos_sensor(0x0387, 0x01);
	write_cmos_sensor(0x0900, 0x00);
	write_cmos_sensor(0x0901, 0x11);
	write_cmos_sensor(0x0401, 0x02);
	write_cmos_sensor(0x0404, 0x00);
	write_cmos_sensor(0x0405, 0x20);
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0x02);
	write_cmos_sensor(0x040a, 0x00);
	write_cmos_sensor(0x040b, 0x00);
	write_cmos_sensor(0x040c, 0x10);
	write_cmos_sensor(0x040d, 0x6a);
	write_cmos_sensor(0x040e, 0x0c);
	write_cmos_sensor(0x040f, 0x2e);
	write_cmos_sensor(0x3038, 0x00);
	write_cmos_sensor(0x303a, 0x00);
	write_cmos_sensor(0x303b, 0x10);
	write_cmos_sensor(0x034c, 0x08);
	write_cmos_sensor(0x034d, 0x34);
	write_cmos_sensor(0x034e, 0x06);
	write_cmos_sensor(0x034f, 0x16);
	write_cmos_sensor(0x3029, 0x00);
	write_cmos_sensor(0x3a00, 0x00);
	write_cmos_sensor(0x3a01, 0x00);
	write_cmos_sensor(0x3a02, 0x05);
	write_cmos_sensor(0x3a03, 0x05);
	write_cmos_sensor(0x3a04, 0x05);
	write_cmos_sensor(0x3a05, 0xf8);
	write_cmos_sensor(0x3a06, 0x40);
	write_cmos_sensor(0x3a07, 0xfe);
	write_cmos_sensor(0x3a08, 0x10);
	write_cmos_sensor(0x3a09, 0x14);
	write_cmos_sensor(0x3a0a, 0xfe);
	write_cmos_sensor(0x3a0b, 0x44);
	write_cmos_sensor(0x0202, 0x0c);
	write_cmos_sensor(0x0203, 0x76);
	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x020e, 0x01);
	write_cmos_sensor(0x020f, 0x00);
	write_cmos_sensor(0x0100, 0x01);
}

static void capture_setting(kal_uint16 currefps)
{
	write_cmos_sensor(0x0100, 0x00);
	write_cmos_sensor(0x0112, 0x0a);
	write_cmos_sensor(0x0113, 0x0a);
	write_cmos_sensor(0x0114, 0x03);
	write_cmos_sensor(0x0301, 0x05);
	write_cmos_sensor(0x0303, 0x02);
	write_cmos_sensor(0x0305, 0x0c);
	write_cmos_sensor(0x0306, 0x02);
	write_cmos_sensor(0x0307, 0x58);
	write_cmos_sensor(0x0309, 0x0a);
	write_cmos_sensor(0x030b, 0x01);
	write_cmos_sensor(0x030d, 0x0c);
	write_cmos_sensor(0x030e, 0x05);
	write_cmos_sensor(0x030f, 0x14);
	write_cmos_sensor(0x0310, 0x00);
	write_cmos_sensor(0x3041, 0x01);
	write_cmos_sensor(0x0342, 0x13);
	write_cmos_sensor(0x0343, 0x70);
	write_cmos_sensor(0x0340, 0x0c);
	write_cmos_sensor(0x0341, 0x80);
	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x00);
	write_cmos_sensor(0x0347, 0x00);
	write_cmos_sensor(0x0348, 0x10);
	write_cmos_sensor(0x0349, 0x6f);
	write_cmos_sensor(0x034a, 0x0c);
	write_cmos_sensor(0x034b, 0x2f);
	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x01);
	write_cmos_sensor(0x0387, 0x01);
	write_cmos_sensor(0x0900, 0x00);
	write_cmos_sensor(0x0901, 0x11);
	write_cmos_sensor(0x0401, 0x02);
	write_cmos_sensor(0x0404, 0x00);
	write_cmos_sensor(0x0405, 0x20);
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0x02);
	write_cmos_sensor(0x040a, 0x00);
	write_cmos_sensor(0x040b, 0x00);
	write_cmos_sensor(0x040c, 0x10);
	write_cmos_sensor(0x040d, 0x6a);
	write_cmos_sensor(0x040e, 0x0c);
	write_cmos_sensor(0x040f, 0x2e);
	write_cmos_sensor(0x3038, 0x00);
	write_cmos_sensor(0x303a, 0x00);
	write_cmos_sensor(0x303b, 0x10);
	write_cmos_sensor(0x034c, 0x08);
	write_cmos_sensor(0x034d, 0x34);
	write_cmos_sensor(0x034e, 0x06);
	write_cmos_sensor(0x034f, 0x16);
	write_cmos_sensor(0x3029, 0x00);
	write_cmos_sensor(0x3a00, 0x00);
	write_cmos_sensor(0x3a01, 0x00);
	write_cmos_sensor(0x3a02, 0x05);
	write_cmos_sensor(0x3a03, 0x05);
	write_cmos_sensor(0x3a04, 0x05);
	write_cmos_sensor(0x3a05, 0xf8);
	write_cmos_sensor(0x3a06, 0x40);
	write_cmos_sensor(0x3a07, 0xfe);
	write_cmos_sensor(0x3a08, 0x10);
	write_cmos_sensor(0x3a09, 0x14);
	write_cmos_sensor(0x3a0a, 0xfe);
	write_cmos_sensor(0x3a0b, 0x44);
	write_cmos_sensor(0x0202, 0x0c);
	write_cmos_sensor(0x0203, 0x76);
	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x020e, 0x01);
	write_cmos_sensor(0x020f, 0x00);
	write_cmos_sensor(0x0100, 0x01);
}

static void normal_video_setting(kal_uint16 currefps)
{
	preview_setting();
}
static void hs_video_setting(void)
{
	LOG_INF("E\n  hs_Video  120fps ");

	write_cmos_sensor(0x0100, 0x00);

	write_cmos_sensor(0x0112, 0x0A);
	write_cmos_sensor(0x0113, 0x0A);
	write_cmos_sensor(0x0114, 0x03);

	write_cmos_sensor(0x0301, 0x05);
	write_cmos_sensor(0x0303, 0x02);
	write_cmos_sensor(0x0305, 0x0C);
	write_cmos_sensor(0x0306, 0x02);
	write_cmos_sensor(0x0307, 0x58);
	write_cmos_sensor(0x0309, 0x0A);
	write_cmos_sensor(0x030B, 0x01);
	write_cmos_sensor(0x030D, 0x0C);
	write_cmos_sensor(0x030E, 0x05);
	write_cmos_sensor(0x030F, 0x14);
	write_cmos_sensor(0x0310, 0x00);
	write_cmos_sensor(0x3041, 0x01);

	write_cmos_sensor(0x0342, 0x13);
	write_cmos_sensor(0x0343, 0x70);
	write_cmos_sensor(0x0340, 0x03);
	write_cmos_sensor(0x0341, 0x20);

	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x03);
	write_cmos_sensor(0x0347, 0x18);
	write_cmos_sensor(0x0348, 0x10);
	write_cmos_sensor(0x0349, 0x6F);
	write_cmos_sensor(0x034A, 0x09);
	write_cmos_sensor(0x034B, 0x17);

	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x01);
	write_cmos_sensor(0x0387, 0x01);
	write_cmos_sensor(0x0900, 0x01);
	write_cmos_sensor(0x0901, 0x12);

	write_cmos_sensor(0x0401, 0x02);
	write_cmos_sensor(0x0404, 0x00);
	write_cmos_sensor(0x0405, 0x19);
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0x04);
	write_cmos_sensor(0x040A, 0x00);
	write_cmos_sensor(0x040B, 0x08);
	write_cmos_sensor(0x040C, 0x10);
	write_cmos_sensor(0x040D, 0x66);
	write_cmos_sensor(0x040E, 0x02);
	write_cmos_sensor(0x040F, 0xF0);
	write_cmos_sensor(0x3038, 0x01);
	write_cmos_sensor(0x303A, 0x00);
	write_cmos_sensor(0x303B, 0x33);

	write_cmos_sensor(0x034C, 0x05);
	write_cmos_sensor(0x034D, 0x24);
	write_cmos_sensor(0x034E, 0x01);
	write_cmos_sensor(0x034F, 0xE0);

	write_cmos_sensor(0x3029, 0x01);
	write_cmos_sensor(0x3A00, 0x01);
	write_cmos_sensor(0x3A01, 0x01);
	write_cmos_sensor(0x3A02, 0x05);
	write_cmos_sensor(0x3A03, 0x05);
	write_cmos_sensor(0x3A04, 0x05);
	write_cmos_sensor(0x3A05, 0xF8);
	write_cmos_sensor(0x3A06, 0x40);
	write_cmos_sensor(0x3A07, 0xFE);
	write_cmos_sensor(0x3A08, 0x10);
	write_cmos_sensor(0x3A09, 0x14);
	write_cmos_sensor(0x3A0A, 0xFE);
	write_cmos_sensor(0x3A0B, 0x44);

	write_cmos_sensor(0x0202, 0x03);
	write_cmos_sensor(0x0203, 0x16);

	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x020E, 0x01);
	write_cmos_sensor(0x020F, 0x00);
	write_cmos_sensor(0x0100, 0x01);
}

static void slim_video_setting(void)
{
	LOG_INF("E\n  Slim_Video  30fps ");

	write_cmos_sensor(0x0100, 0x00);

	write_cmos_sensor(0x0112, 0x0A);
	write_cmos_sensor(0x0113, 0x0A);
	write_cmos_sensor(0x0114, 0x03);

	write_cmos_sensor(0x0301, 0x05);
	write_cmos_sensor(0x0303, 0x02);
	write_cmos_sensor(0x0305, 0x0C);
	write_cmos_sensor(0x0306, 0x01);
	write_cmos_sensor(0x0307, 0xCC);
	write_cmos_sensor(0x0309, 0x0A);
	write_cmos_sensor(0x030B, 0x01);
	write_cmos_sensor(0x030D, 0x0C);
	write_cmos_sensor(0x030E, 0x05);
	write_cmos_sensor(0x030F, 0x14);
	write_cmos_sensor(0x0310, 0x00);
	write_cmos_sensor(0x3041, 0x01);

	write_cmos_sensor(0x0342, 0x13);
	write_cmos_sensor(0x0343, 0x70);
	write_cmos_sensor(0x0340, 0x04);
	write_cmos_sensor(0x0341, 0xCC);

	write_cmos_sensor(0x0344, 0x00);
	write_cmos_sensor(0x0345, 0x00);
	write_cmos_sensor(0x0346, 0x01);
	write_cmos_sensor(0x0347, 0x80);
	write_cmos_sensor(0x0348, 0x10);
	write_cmos_sensor(0x0349, 0x6F);
	write_cmos_sensor(0x034A, 0x0A);
	write_cmos_sensor(0x034B, 0xAF);

	write_cmos_sensor(0x0381, 0x01);
	write_cmos_sensor(0x0383, 0x01);
	write_cmos_sensor(0x0385, 0x01);
	write_cmos_sensor(0x0387, 0x01);
	write_cmos_sensor(0x0900, 0x01);
	write_cmos_sensor(0x0901, 0x12);

	write_cmos_sensor(0x0401, 0x02);
	write_cmos_sensor(0x0404, 0x00);
	write_cmos_sensor(0x0405, 0x18);
	write_cmos_sensor(0x0408, 0x00);
	write_cmos_sensor(0x0409, 0x02);
	write_cmos_sensor(0x040A, 0x00);
	write_cmos_sensor(0x040B, 0x00);
	write_cmos_sensor(0x040C, 0x10);
	write_cmos_sensor(0x040D, 0x6A);
	write_cmos_sensor(0x040E, 0x04);
	write_cmos_sensor(0x040F, 0x96);
	write_cmos_sensor(0x3038, 0x01);
	write_cmos_sensor(0x303A, 0x00);
	write_cmos_sensor(0x303B, 0x30);

	write_cmos_sensor(0x034C, 0x05);
	write_cmos_sensor(0x034D, 0x78);
	write_cmos_sensor(0x034E, 0x03);
	write_cmos_sensor(0x034F, 0x0E);

	write_cmos_sensor(0x3029, 0x01);
	write_cmos_sensor(0x3A00, 0x01);
	write_cmos_sensor(0x3A01, 0x01);
	write_cmos_sensor(0x3A02, 0x05);
	write_cmos_sensor(0x3A03, 0x05);
	write_cmos_sensor(0x3A04, 0x05);
	write_cmos_sensor(0x3A05, 0xF8);
	write_cmos_sensor(0x3A06, 0x40);
	write_cmos_sensor(0x3A07, 0xFE);
	write_cmos_sensor(0x3A08, 0x10);
	write_cmos_sensor(0x3A09, 0x14);
	write_cmos_sensor(0x3A0A, 0xFE);
	write_cmos_sensor(0x3A0B, 0x44);

	write_cmos_sensor(0x0202, 0x04);
	write_cmos_sensor(0x0203, 0xC2);

	write_cmos_sensor(0x0204, 0x00);
	write_cmos_sensor(0x0205, 0x00);
	write_cmos_sensor(0x020E, 0x01);
	write_cmos_sensor(0x020F, 0x00);
	write_cmos_sensor(0x0100, 0x01);
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable)
		write_cmos_sensor(0x0601, 0x04);
	else
		write_cmos_sensor(0x0601, 0x00);
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
			LOG_INF("Read sensor id fail, id: 0x%x, sensor id: 0x%x\n",
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
	LOG_2;

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("test\n");
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}
			LOG_INF("Read sensor id fail,id: 0x%x, sensor id: 0x%x\n",
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

	sensor_init();

	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = KAL_FALSE;
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
	LOG_INF("Exit\n");
	return ERROR_NONE;
}

static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == 300) {
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else {
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("Caputre fps:%d\n", imgsensor.current_fps);
	capture_setting(imgsensor.current_fps);

	return ERROR_NONE;
}
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			       MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

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
	sensor_info->SensorInterruptDelayLines = 1;
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
	sensor_info->SensorClockDividCount = 5;
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
	LOG_INF("scenario_id = %d\n", scenario_id);
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

		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor.current_fps == 300) {
			frame_length = imgsensor_info.cap.pclk / framerate *
				       10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line =
				(frame_length >
				 imgsensor_info.cap.framelength) ?
					      (frame_length -
					 imgsensor_info.cap.framelength) :
					      0;
			imgsensor.frame_length =
				imgsensor_info.cap.framelength +
				imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		} else {
			frame_length = imgsensor_info.cap1.pclk / framerate *
				       10 / imgsensor_info.cap1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line =
				(frame_length >
				 imgsensor_info.cap1.framelength) ?
					      (frame_length -
					 imgsensor_info.cap1.framelength) :
					      0;
			imgsensor.frame_length =
				imgsensor_info.cap1.framelength +
				imgsensor.dummy_line;
			spin_unlock(&imgsensor_drv_lock);
		}

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
	unsigned long long *feature_return_para =
		(unsigned long long *)feature_para;

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
		LOG_INF("feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n",
			imgsensor.pclk, imgsensor.current_fps);
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

UINT32 IMX278_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}
