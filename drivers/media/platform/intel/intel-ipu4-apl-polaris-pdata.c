/*
 * Copyright (c) 2015--2016 Intel Corporation.
 * Copyright (c) 2018 Mentor - a Siemens Business 
 * 
 * Based on intel-ipu4-bxt-p-pdata.c
 *
 * Author: Samu Onkalo <samu.onkalo@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/pci.h>

#include <media/intel-ipu4-isys.h>
#include <media/ds90ub940.h>

#define GPIO_BASE             434

#if defined(CONFIG_VIDEO_ADV728X) || defined(CONFIG_VIDEO_ADV728X_MODULE)
static struct intel_ipu4_isys_csi2_config adv7282_csi2_cfg = {
	.nlanes = 1,
	.port = 5,
};

static struct intel_ipu4_isys_subdev_info adv7282_sd = {
	.csi2 = &adv7282_csi2_cfg,
	.i2c = {
		.board_info = {
			 .type = "adv728x",
			 .flags = 0,
			 .addr = 0x20,
			 .platform_data = NULL,
			 .irq = GPIO_BASE + 22, /* GPIO_22 */
		},
		.i2c_adapter_id = 0,
	}
};
#endif

#if defined(CONFIG_VIDEO_DS90UB940) || defined(CONFIG_VIDEO_DS90UB940_MODULE)

struct ds90ub940_platform_data ds90ub940_pdata = {
	.i2c_addr = 0x2C,
	.i2c_adapter = 0,
	.ext_clk = 0xFF,
	.csi2_lanes = 2,
	.csi2_port = 4,
	.module_name = "ds90ub940",
	.powerdown_pin = 434 + 2,
	.pass_pin = -1,
};

static struct intel_ipu4_isys_csi2_config ds90ub940_csi2_cfg = {
	.nlanes = 2,
	.port = 4,
};

static struct intel_ipu4_isys_subdev_info ds90ub940_sd = {
	.csi2 = &ds90ub940_csi2_cfg,
	.i2c = {
		.board_info = {
			 .type = "ds90ub940",
			 .flags = 0,
			 .addr = 0x2C,
			 .platform_data = &ds90ub940_pdata,
		},
		.i2c_adapter_id = 0,
	},
};
#endif

/*
 * Map buttress output sensor clocks to sensors -
 */
static struct intel_ipu4_isys_clk_mapping clk_mapping[] = {
	{ CLKDEV_INIT(NULL, NULL, NULL), NULL }
};

static struct intel_ipu4_isys_subdev_pdata pdata = {
	.subdevs = (struct intel_ipu4_isys_subdev_info *[]) {
#if defined(CONFIG_VIDEO_ADV728X) || defined(CONFIG_VIDEO_ADV728X_MODULE)
		&adv7282_sd,
#endif
#if defined(CONFIG_VIDEO_DS90UB940) || defined(CONFIG_VIDEO_DS90UB940_MODULE)
		&ds90ub940_sd,
#endif
		NULL,
	},
	.clk_map = clk_mapping,
};

static void ipu4_quirk(struct pci_dev *pci_dev)
{
	pci_dev->dev.platform_data = &pdata;
}

DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, 0x5a88,
			ipu4_quirk);
