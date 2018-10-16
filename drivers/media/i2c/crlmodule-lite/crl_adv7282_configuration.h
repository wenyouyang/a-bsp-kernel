/* SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0) */
/*
 * Copyright (C) 2018 Mentor Graphics Corporation
 */

#ifndef __CRLMODULE_ADV7282_CONFIGURATION_H_
#define __CRLMODULE_ADV7282_CONFIGURATION_H_

#include "crlmodule-sensor-ds.h"

static struct crl_register_write_rep adv7282_powerup_regset[] = {
#if 0 // Free-Run mode
	{0x0f, CRL_REG_LEN_08BIT, 0x80, 0x40, 0x0e00},// Start reset sequence
	{0x00, CRL_REG_LEN_DELAY, 0x0a, 0x00, 0x0000},
	{0x0f, CRL_REG_LEN_08BIT, 0x00, 0x40, 0x0e00},// Exit Power Down Mode
	{0x00, CRL_REG_LEN_DELAY, 0x0a, 0x00, 0x0000},
	{0x00, CRL_REG_LEN_08BIT, 0x04, 0x40, 0x0e00},// ADI Required Write
	{0x0c, CRL_REG_LEN_08BIT, 0x37, 0x40, 0x0e00},// Force Free-run Mode
	{0x02, CRL_REG_LEN_08BIT, 0x84, 0x40, 0x0e00},// Force standard to PAL
	{0x14, CRL_REG_LEN_08BIT, 0x15, 0x40, 0x0e00},// Set Free-run pattern to color bars
	{0x03, CRL_REG_LEN_08BIT, 0x4e, 0x40, 0x0e00},// ADI Required Write
	{0x04, CRL_REG_LEN_08BIT, 0x57, 0x40, 0x0e00},// Power-up INTRQ pin
	{0x13, CRL_REG_LEN_08BIT, 0x00, 0x40, 0x0e00},// Enable INTRQ output driver
	{0x17, CRL_REG_LEN_08BIT, 0x41, 0x40, 0x0e00},// select SH1
	{0x1d, CRL_REG_LEN_08BIT, 0xc0, 0x40, 0x0e00},// Tri-State LLC output driver
	{0x52, CRL_REG_LEN_08BIT, 0xcd, 0x40, 0x0e00},// ADI Required Write
	{0x80, CRL_REG_LEN_08BIT, 0x51, 0x40, 0x0e00},// ADI Required Write
	{0x81, CRL_REG_LEN_08BIT, 0x51, 0x40, 0x0e00},// ADI Required Write
	{0x82, CRL_REG_LEN_08BIT, 0x68, 0x40, 0x0e00},// ADI Required Write
	{0x5d, CRL_REG_LEN_08BIT, 0x1c, 0x40, 0x0e00},// Enable Diagnostic pin 1 - Level=1.125V
	{0x5e, CRL_REG_LEN_08BIT, 0x1c, 0x40, 0x0e00},// Enable Diagnostic pin 2 - Level=1.125V
	{0xfe, CRL_REG_LEN_08BIT, 0x88, 0x40, 0x0e00},// Set CSI Map Address
	{0xde, CRL_REG_LEN_08BIT, 0x02, 0x88, 0x0000},// Power up MIPI D-PHY
	{0xd2, CRL_REG_LEN_08BIT, 0xf7, 0x88, 0x0000},// ADI Required Write
	{0xd8, CRL_REG_LEN_08BIT, 0x65, 0x88, 0x0000},// ADI Required Write
	{0xe0, CRL_REG_LEN_08BIT, 0x09, 0x88, 0x0000},// ADI Required Write
	{0x2c, CRL_REG_LEN_08BIT, 0x00, 0x88, 0x0000},// ADI Required Write
#else
	{0x0f, CRL_REG_LEN_08BIT, 0x80, 0x40, 0x0e00},// Start reset sequence
	{0x00, CRL_REG_LEN_DELAY, 0x0a, 0x00, 0x0000},
	{0x0f, CRL_REG_LEN_08BIT, 0x00, 0x40, 0x0e00},// Exit Power Down Mode
	{0x00, CRL_REG_LEN_DELAY, 0x0a, 0x00, 0x0000},
	{0x00, CRL_REG_LEN_08BIT, 0x0e, 0x40, 0x0e00},// INSEL: CVBS_P in on AIN1, CVBS_N in on AIN2
	{0x02, CRL_REG_LEN_08BIT, 0x04, 0x40, 0x0e00},// Force standard to PAL
	{0x0f, CRL_REG_LEN_08BIT, 0x00, 0x40, 0x0e00},// ADI Required Write
	{0x0e, CRL_REG_LEN_08BIT, 0x80, 0x40, 0x0e00},// ADI Required Write
	{0x9c, CRL_REG_LEN_08BIT, 0x00, 0x40, 0x0e80},// ADI Required Write
	{0x9c, CRL_REG_LEN_08BIT, 0xff, 0x40, 0x0e80},// ADI Required Write
	{0x0e, CRL_REG_LEN_08BIT, 0x00, 0x40, 0x0e00},// Enter User Sub Map
	{0x03, CRL_REG_LEN_08BIT, 0x4e, 0x40, 0x0e00},// ADI Required Write
	{0x04, CRL_REG_LEN_08BIT, 0xd7, 0x40, 0x0e00},// Enable Intrq pin
	{0x13, CRL_REG_LEN_08BIT, 0x00, 0x40, 0x0e00},// Enable INTRQ output driver
	{0x17, CRL_REG_LEN_08BIT, 0x41, 0x40, 0x0e00},// select SH1
	{0x1d, CRL_REG_LEN_08BIT, 0xc0, 0x40, 0x0e00},// Tri-State LLC output driver
	{0x52, CRL_REG_LEN_08BIT, 0xc0, 0x40, 0x0e00},// ADI Required Write
	{0x80, CRL_REG_LEN_08BIT, 0x51, 0x40, 0x0e00},// ADI Required Write
	{0x81, CRL_REG_LEN_08BIT, 0x51, 0x40, 0x0e00},// ADI Required Write
	{0x82, CRL_REG_LEN_08BIT, 0x68, 0x40, 0x0e00},// ADI Required Write
	{0x5d, CRL_REG_LEN_08BIT, 0x1c, 0x40, 0x0e00},// Enable Diagnostic pin 1 - Level=1.125V
	{0x5e, CRL_REG_LEN_08BIT, 0x1c, 0x40, 0x0e00},// Enable Diagnostic pin 2 - Level=1.125V
	{0x5f, CRL_REG_LEN_08BIT, 0xa8, 0x40, 0x0e00},// SHA gain for Div4
	{0x5a, CRL_REG_LEN_08BIT, 0x90, 0x40, 0x0e00},// ADI Required Write
	{0x60, CRL_REG_LEN_08BIT, 0xb0, 0x40, 0x0e00},// ADI Required Write
	{0x0e, CRL_REG_LEN_08BIT, 0x80, 0x40, 0x0e00},// ADI Required Write
	{0xb6, CRL_REG_LEN_08BIT, 0x08, 0x40, 0x0e80},// ADI Required Write
	{0xc0, CRL_REG_LEN_08BIT, 0xa0, 0x40, 0x0e80},// ADI Required Write
	{0x0e, CRL_REG_LEN_08BIT, 0x00, 0x40, 0x0e00},// ADI Required Write
	{0xfe, CRL_REG_LEN_08BIT, 0x88, 0x40, 0x0e00},// Set CSI Map Address
	{0xde, CRL_REG_LEN_08BIT, 0x02, 0x88, 0x0000},// Power up MIPI D-PHY
	{0xd2, CRL_REG_LEN_08BIT, 0xf7, 0x88, 0x0000},// ADI Required Write
	{0xd8, CRL_REG_LEN_08BIT, 0x65, 0x88, 0x0000},// ADI Required Write
	{0xe0, CRL_REG_LEN_08BIT, 0x09, 0x88, 0x0000},// ADI Required Write
	{0x2c, CRL_REG_LEN_08BIT, 0x00, 0x88, 0x0000},// ADI Required Write
#endif
};


static struct crl_register_write_rep adv7282_streamon_regs[] = {
	{0x00, CRL_REG_LEN_08BIT, 0x00, 0x88, 0x00}, // CSI Tx on
};

static struct crl_register_write_rep adv7282_streamoff_regs[] = {
	{0x00, CRL_REG_LEN_08BIT, 0x80, 0x88, 0x00}, // CSI Tx off
};

static struct crl_sensor_detect_config adv7282_sensor_detect_regset[] = {
	{
		.reg = {0x11, CRL_REG_LEN_08BIT, 0xff, 0x40, 0x0e00},
		.width = 1,
	},
};

static const s64 adv7282_op_sys_clock[] =  {13500000};

static struct crl_pll_configuration adv7282_pll_configurations[] = {
	{
		.input_clk = 28636300,
		.op_sys_clk = 13500000,
		.bitsperpixel = 16,
		.pixel_rate_csi = 13500000,
		.pixel_rate_pa = 13500000,
		.comp_items = 0,
		.ctrl_data = 0,
		.pll_regs_items = 0,
		.pll_regs = NULL,
	 },
};

static struct crl_subdev_rect_rep adv7282_ntsc_rects[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.in_rect.left = 0,
		.in_rect.top = 0,
		.in_rect.width = 720,
		.in_rect.height = 288,
		.out_rect.left = 0,
		.out_rect.top = 0,
		.out_rect.width = 720,
		.out_rect.height = 288,
	},
};

static struct crl_mode_rep adv7282_modes[] = {
	{
		.sd_rects_items = ARRAY_SIZE(adv7282_ntsc_rects),
		.sd_rects = adv7282_ntsc_rects,
		.binn_hor = 1,
		.binn_vert = 1,
		.scale_m = 1,
		.width = 720,
		.height = 288,
		.comp_items = 0,
		.ctrl_data = 0,
		.mode_regs_items = 0,
		.mode_regs = NULL,
	},
};

static struct crl_sensor_subdev_config adv7282_sensor_subdevs[] = {
	{
		.subdev_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.name = "adv7282m",
	},
};

static struct crl_sensor_limits adv7282_sensor_limits = {
	.x_addr_min = 0,
	.y_addr_min = 0,
	.x_addr_max = 720,
	.y_addr_max = 288,
	.min_frame_length_lines = 160,
	.max_frame_length_lines = 65535,
	.min_line_length_pixels = 6024,
	.max_line_length_pixels = 32752,
	.scaler_m_min = 1,
	.scaler_m_max = 1,
	.scaler_n_min = 1,
	.scaler_n_max = 1,
	.min_even_inc = 1,
	.max_even_inc = 1,
	.min_odd_inc = 1,
	.max_odd_inc = 1,
};

static struct crl_csi_data_fmt adv7282_crl_csi_data_fmt[] = {
	{
		.code = ICI_FORMAT_UYVY,
		.pixel_order = CRL_PIXEL_ORDER_GRBG,
		.bits_per_pixel = 16,
		.regs_items = 0,
		.regs = NULL,
	},
};

static struct crl_ctrl_data adv7282_ctrls[] = {
	{
		.sd_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.op_type = CRL_CTRL_SET_OP,
		.context = SENSOR_IDLE,
		.ctrl_id = ICI_EXT_SD_PARAM_ID_LINK_FREQ,
		.name = "CTRL_ID_LINK_FREQ",
		.type = CRL_CTRL_TYPE_MENU_INT,
		.data.int_menu.def = 0,
		.data.int_menu.max = ARRAY_SIZE(adv7282_pll_configurations) - 1,
		.data.int_menu.menu = adv7282_op_sys_clock,
		.flags = 0,
		.impact = CRL_IMPACTS_NO_IMPACT,
		.regs_items = 0,
		.regs = 0,
		.dep_items = 0,
		.dep_ctrls = 0,
	},
	{
		.sd_type = CRL_SUBDEV_TYPE_PIXEL_ARRAY,
		.op_type = CRL_CTRL_GET_OP,
		.context = SENSOR_POWERED_ON,
		.ctrl_id = ICI_EXT_SD_PARAM_ID_PIXEL_RATE,
		.name = "CTRL_ID_PIXEL_RATE_PA",
		.type = CRL_CTRL_TYPE_INTEGER,
		.data.std_data.min = 0,
		.data.std_data.max = 0,
		.data.std_data.step = 1,
		.data.std_data.def = 0,
		.flags = 0,
		.impact = CRL_IMPACTS_NO_IMPACT,
		.regs_items = 0,
		.regs = 0,
		.dep_items = 0,
		.dep_ctrls = 0,
	},
};

static struct crl_sensor_configuration adv7282_crl_configuration = {
	.sensor_init = NULL,
	.sensor_cleanup = NULL,

	.onetime_init_regs_items = 0,
	.onetime_init_regs = NULL,

	.powerup_regs_items = ARRAY_SIZE(adv7282_powerup_regset),
	.powerup_regs = adv7282_powerup_regset,

	.poweroff_regs_items = ARRAY_SIZE(adv7282_streamoff_regs),
	.poweroff_regs = adv7282_streamoff_regs,

	.id_reg_items = ARRAY_SIZE(adv7282_sensor_detect_regset),
	.id_regs = adv7282_sensor_detect_regset,

	.subdev_items = ARRAY_SIZE(adv7282_sensor_subdevs),
	.subdevs = adv7282_sensor_subdevs,

	.sensor_limits = &adv7282_sensor_limits,

	.pll_config_items = ARRAY_SIZE(adv7282_pll_configurations),
	.pll_configs = adv7282_pll_configurations,
	.op_sys_clk = adv7282_op_sys_clock,

	.modes_items = ARRAY_SIZE(adv7282_modes),
	.modes = adv7282_modes,

	.streamon_regs_items = ARRAY_SIZE(adv7282_streamon_regs),
	.streamon_regs = adv7282_streamon_regs,

	.streamoff_regs_items = ARRAY_SIZE(adv7282_streamoff_regs),
	.streamoff_regs = adv7282_streamoff_regs,

	.ctrl_items = ARRAY_SIZE(adv7282_ctrls),
	.ctrl_bank = adv7282_ctrls,

	.csi_fmts_items = ARRAY_SIZE(adv7282_crl_csi_data_fmt),
	.csi_fmts = adv7282_crl_csi_data_fmt,

	.addr_len = CRL_ADDR_7BIT,
};

#endif  /* __CRLMODULE_ADV7282_CONFIGURATION_H_ */
