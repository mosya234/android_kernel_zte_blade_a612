/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __MT65XX_LCM_LIST_H__
#define __MT65XX_LCM_LIST_H__

#include <lcm_drv.h>
extern LCM_DRIVER ili9881c_hd720_dsi_vdo_cmi_lcm_drv;//for A612
extern LCM_DRIVER ili9881c_hd720_dsi_vdo_hsd_lcm_drv;//for A602

#ifdef BUILD_LK
extern void mdelay(unsigned long msec);
#endif

#endif
