/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _AXS15260_S17_H
#define _AXS15260_S17_H

#include <linux/errno.h>
#include <linux/types.h>

#define AXS_MAX_POINTS 5
#define AXS_REPORT_SIZE (2 + 6 * AXS_MAX_POINTS)

struct axs_point {
	u16 x;
	u16 y;
	u8 event;
	u8 id;
	u8 pressure;
	u8 area;
};

struct axs_frame {
	u8 count;
	struct axs_point points[AXS_MAX_POINTS];
};

/* Recovered from stock Image 0x9f93e0..0x9f94ec, not the ESP32 sample. */
static inline int axs_decode_report(const u8 *buf, unsigned int len,
				    struct axs_frame *frame)
{
	unsigned int i, count;

	if (len != AXS_REPORT_SIZE)
		return -EINVAL;
	/* Gesture/ESD frames are not ordinary contact reports. */
	if (buf[0] || (buf[1] & 0xf0))
		return -EAGAIN;
	count = buf[1] & 0x0f;
	if (count > AXS_MAX_POINTS)
		return -EINVAL;

	frame->count = 0;
	for (i = 0; i < count; i++) {
		const u8 *p = buf + 2 + 6 * i;
		struct axs_point *point = &frame->points[frame->count];

		/* Stock stops at the first ID outside its five-contact range. */
		if ((p[2] >> 4) >= AXS_MAX_POINTS)
			break;
		point->x = ((p[0] & 0x0f) << 8) | p[1];
		point->y = ((p[2] & 0x0f) << 8) | p[3];
		point->event = p[0] >> 6;
		point->id = p[2] >> 4;
		point->pressure = p[4] ? p[4] : 0x3f;
		point->area = (p[5] >> 4) ? (p[5] >> 4) : 9;
		frame->count++;
	}
	return 0;
}

#endif
