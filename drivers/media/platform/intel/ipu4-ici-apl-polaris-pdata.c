// SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
/*
 * Copyright (C) 2018 Mentor Graphic Corporation
 */

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <media/ipu-isys.h>
#include <media/crlmodule-lite.h>
#include "ipu.h"

#define GPIO_BASE		434

#if defined(CONFIG_INTEL_IPU4_ICI_ADV7282M)

#define ADV7282_CSI2_LANES	1
#define ADV7282_I2C_ADDRESS	0x20

static struct crlmodule_lite_platform_data adv7282_crl_lite_pdata = {
#if (!IS_ENABLED(CONFIG_VIDEO_INTEL_UOS))
	/* xshutdown GPIO pin unavailable on ACRN UOS */
	.xshutdown = GPIO_BASE + 0,
#endif
	.lanes = ADV7282_CSI2_LANES,
	.ext_clk = 28636300,
	.op_sys_clock = (uint64_t []){216000000},
	.module_name = "ADV7282M"
};

static struct ipu_isys_csi2_config adv7282_csi2_cfg = {
	.nlanes = ADV7282_CSI2_LANES,
	.port = 5,
};

static struct ipu_isys_subdev_info adv7282_sd = {
	.csi2 = &adv7282_csi2_cfg,
	.i2c = {
		.board_info = {
			 .type = CRLMODULE_LITE_NAME,
			 .flags = 0,
			 .addr = ADV7282_I2C_ADDRESS,
			 .platform_data = &adv7282_crl_lite_pdata,
			 .irq = GPIO_BASE + 22,
		},

		.i2c_adapter_id = 0,
	}
};
#endif

/*
 * Map buttress output sensor clocks to sensors -
 */
static struct ipu_isys_clk_mapping clk_mapping[] = {
	{ CLKDEV_INIT(NULL, NULL, NULL), NULL }
};

static struct ipu_isys_subdev_pdata pdata = {
	.subdevs = (struct ipu_isys_subdev_info *[]) {
#if defined(CONFIG_INTEL_IPU4_ICI_ADV7282M)
		&adv7282_sd,
#endif
		NULL,
	},
	.clk_map = clk_mapping,
};

static void ipu4_quirk(struct pci_dev *pci_dev)
{
	pci_dev->dev.platform_data = &pdata;
}

DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, IPU_PCI_ID, ipu4_quirk);
