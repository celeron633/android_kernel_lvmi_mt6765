/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_INPUT_AXS15260_S17_H
#define _LINUX_INPUT_AXS15260_S17_H

#include <linux/types.h>

/* GPIO174 remains owned by the AXS LCM bias device. */
int s17_axs15260_reset_touch(unsigned int settle_ms);
int s17_axs15260_pram_session(
	int (*write_mode)(void *data, const u8 *buf, unsigned int len),
	int (*download)(void *data), void *data);
int s17_axs15260_flash_read_session(
	int (*write_mode)(void *data, const u8 *buf, unsigned int len),
	int (*read_flash)(void *data), void *data);
int s17_axs15260_flash_session(
	int (*write_mode)(void *data, const u8 *buf, unsigned int len),
	int (*run)(void *data), void *data);
int s17_axs15260_flash_repair_session(
	int (*write_mode)(void *data, const u8 *buf, unsigned int len),
	int (*run)(void *data), int (*check_version)(void *data), void *data);

#endif
