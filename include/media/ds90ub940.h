/*
 * ds90ub940.h 
 * TI FPD-Link III Deserializer
 * 
 * Based on Intel crlmodule.h
 *
 * Copyright (c) 2014--2016 Intel Corporation.
 * Copyright (c) 2017 Mentor Graphics Corp.
 *
 * Author: Vinod Govindapillai <vinod.govindapillai@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __DS90UB940_H
#define __DS90UB940_H

#include <media/v4l2-subdev.h>

#define DS90UB940_MODULE_NAME		"ds90ub940"

struct ds90ub940_platform_data {
	unsigned short i2c_addr;
	unsigned short i2c_adapter;

	unsigned int ext_clk;		/* sensor external clk */

	unsigned int csi2_lanes;	/* Number of CSI-2 lanes */
	unsigned int csi2_port;		/* CSI-2 port */
	const s64 *op_sys_clock;

	char module_name[16]; /* module name from ACPI */
	int powerdown_pin;
	int pass_pin;
};

#endif /* __DS90UB940_H  */
