/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __GED_GE_H__
#define __GED_GE_H__

#include <linux/types.h>
#include <ged_type.h>

/* S17 vendor gralloc_extra/ge_config.h allocates 18 metadata regions.
 * Keep the GE_ALLOC input copy and entry region count in sync with it.
 */
#define GE_ALLOC_STRUCT_NUM 18

GED_ERROR ged_ge_init(void);
int ged_ge_exit(void);
int ged_ge_alloc(int region_num, uint32_t *region_sizes);
int ged_ge_get(int ge_fd, int region_id, int u32_offset,
	int u32_size, uint32_t *output_data);
int ged_ge_set(int ge_fd, int region_id, int u32_offset,
	int u32_size, uint32_t *input_data);

#endif
