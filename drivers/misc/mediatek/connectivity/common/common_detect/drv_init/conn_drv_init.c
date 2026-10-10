/*
* Copyright (C) 2011-2014 MediaTek Inc.
*
* This program is free software: you can redistribute it and/or modify it under the terms of the
* GNU General Public License version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See the GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef DFT_TAG
#undef DFT_TAG
#endif
#define DFT_TAG         "[WCN-MOD-INIT]"

#include <linux/mutex.h>
#include "wmt_detect.h"
#include "conn_drv_init.h"
#include "common_drv_init.h"
#include "wlan_drv_init.h"
#include "bluetooth_drv_init.h"

#if (MTK_WCN_REMOVE_KO)
/* Cache the actual result: repeated loader ioctls must not fake success or
 * register partially initialized drivers twice.
 */
static DEFINE_MUTEX(connectivity_init_lock);
static bool connectivity_init_attempted;
static int connectivity_init_result;

int do_connectivity_driver_init(int chip_id)
{
	int ret;

	mutex_lock(&connectivity_init_lock);
	if (connectivity_init_attempted) {
		ret = connectivity_init_result;
		goto out;
	}
	connectivity_init_attempted = true;

	ret = do_common_drv_init(chip_id);
	if (ret)
		goto done;

	ret = do_bluetooth_drv_init(chip_id);
	if (ret)
		goto done;

	ret = do_wlan_drv_init(chip_id);
done:
	connectivity_init_result = ret;
	WMT_DETECT_PR_INFO("S17 WMT/BT/Wi-Fi init finished, ret:%d\n", ret);
out:
	mutex_unlock(&connectivity_init_lock);
	return ret;
}
#endif
