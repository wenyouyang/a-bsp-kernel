/*
 * Device driver for TI SN65DSI85
 *
 * Copyright 2017 Mentor Graphics Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/property.h>
#include <linux/string.h>
#include <linux/acpi.h>
#include <linux/delay.h>

#define SN65DSI85_MODULE_NAME "sn65dsi85"
#define SN65DSI85_MODULE_VERSION "0.0.1"

struct sn65dsi85_state {
	int irq;
	int enable_pin;
	struct i2c_adapter *i2c_bus;
};

#define I2C_BUS  "i2c_designware.5"
#define MAX_I2C_TRIAL 5

static struct sn65dsi85_state state = {
	.irq = 458,
	.enable_pin = 457,
	.i2c_bus = NULL,
};

static int sn65dsi85_write(u8 addr, u8 reg, u8 val)
{
	int ret;
	int i;
	u8 buf[2];
	struct i2c_msg msg;
	buf[0] = reg;
	buf[1] = val;

	msg.addr = addr;
	msg.buf = buf;
	msg.len = 2;
	msg.flags = 0;

	for (i = 0; i < MAX_I2C_TRIAL; i++) {
		ret = i2c_transfer(state.i2c_bus, &msg, 1);
		if (1 == ret) {
			pr_debug
			    ("sn65dsi85_write success: %02X [%02X]:%02X r:%d\n",
			     addr, reg, val, ret);
			return 0;
		}
	}

	pr_err("sn65dsi85_write fail: %02X [%02X]:%02X r:%d\n", addr, reg, val,
		ret);

	return ret;
}

static int __sn65dsi85_init(void)
{
	int ret = 0;
	if (state.enable_pin != -1)
		gpio_direction_output(state.enable_pin, 1);

	/* 

	   settings are hard coded for 
	   preliminary version

	 */

	ret |= sn65dsi85_write(0x2D, 0x09, 0x01);
	msleep(500);
	/* PLL_EN = 0 */
	ret |= sn65dsi85_write(0x2D, 0x0d, 0x00);
	/* LVDS_CLK_RANGE 69 MHz / DSI Clock as source */
	ret |= sn65dsi85_write(0x2D, 0x0A, 0x05);
	/* 200MHz / 3 = 66.6 MHz ~ 69 MHz LVDS Clock  */
	ret |= sn65dsi85_write(0x2D, 0x0B, 0x10);
	/* dual DSI 4 Lanes, 2 single DSI + 0x18.4=0 */
	/* 0: LEFT/RIGHT MODE */
	/* 10: Two single DSI recivers */
	/* 00: CHA: 4 DSI lanes */
	/* 00: CHB: 4 DSI lanes  */
	/* 1: Single bit errors tolerated  */
	ret |= sn65dsi85_write(0x2D, 0x10, 0x41);
	/* 200 MHz CHA DSI Clock */
	ret |= sn65dsi85_write(0x2D, 0x12, 0x28);
	/* 200 MHz CHB DSI Clock */
	ret |= sn65dsi85_write(0x2D, 0x13, 0x28);
	/* 0x0C: 24 bpp */
	/* 0x03: _Y3 pair transmits LSB  */
	ret |= sn65dsi85_write(0x2D, 0x18, 0x0F);
	ret |= sn65dsi85_write(0x2D, 0x19, 0x00);

	/* serializer expects 100 ohm termination */
	ret |= sn65dsi85_write(0x2D, 0x1A, 0x00);
	/* CHA active length 1280  */
	ret |= sn65dsi85_write(0x2D, 0x20, 0x00);
	ret |= sn65dsi85_write(0x2D, 0x21, 0x05);
	/* CHB active length 1280  */
	ret |= sn65dsi85_write(0x2D, 0x22, 0x00);
	ret |= sn65dsi85_write(0x2D, 0x23, 0x05);
	/* CHA vertical 720 */
	ret |= sn65dsi85_write(0x2D, 0x24, 0xD0);
	ret |= sn65dsi85_write(0x2D, 0x25, 0x02);
	/* CHB vertical 720 */
	ret |= sn65dsi85_write(0x2D, 0x26, 0xD0);
	ret |= sn65dsi85_write(0x2D, 0x27, 0x02);
	/* CHA sync delay 32 */
	ret |= sn65dsi85_write(0x2D, 0x28, 0x20);
	ret |= sn65dsi85_write(0x2D, 0x29, 0x00);
	/* CHB sync delay 32 */
	ret |= sn65dsi85_write(0x2D, 0x2A, 0x20);
	ret |= sn65dsi85_write(0x2D, 0x2B, 0x00);
	/* CHA hsync width 20 */
	ret |= sn65dsi85_write(0x2D, 0x2C, 0x14);
	ret |= sn65dsi85_write(0x2D, 0x2D, 0x00);
	/* CHB hsync width 20 */
	ret |= sn65dsi85_write(0x2D, 0x2E, 0x14);
	ret |= sn65dsi85_write(0x2D, 0x2F, 0x00);
	/* CHA vsync width 05 */
	ret |= sn65dsi85_write(0x2D, 0x30, 0x05);
	ret |= sn65dsi85_write(0x2D, 0x31, 0x00);
	/* CHB vsync width 05 */
	ret |= sn65dsi85_write(0x2D, 0x32, 0x05);
	ret |= sn65dsi85_write(0x2D, 0x33, 0x00);
	/* CHA horizontal back porch 90 */
	ret |= sn65dsi85_write(0x2D, 0x34, 0x5A);
	/* CHB horizontal back porch 90 */
	ret |= sn65dsi85_write(0x2D, 0x35, 0x5A);
	/* CHA vertical back porch 24 */
	ret |= sn65dsi85_write(0x2D, 0x36, 0x18);
	/* CHB vertical back porch 24 */
	ret |= sn65dsi85_write(0x2D, 0x37, 0x18);
	/* CHA horizontal front porch 90 */
	ret |= sn65dsi85_write(0x2D, 0x38, 0x5A);
	/* CHB horizontal front porch 90 */
	ret |= sn65dsi85_write(0x2D, 0x39, 0x5A);
	/* CHA vertical front porch 24 */
	ret |= sn65dsi85_write(0x2D, 0x3A, 0x18);
	/* CHB vertical front porch 24 */
	ret |= sn65dsi85_write(0x2D, 0x3B, 0x18);
	/* test pattern CHA=ON CHB=ON */
	/* ret |= sn65dsi85_write(0x2D, 0x3C, 0x11); */
	/* PLL_EN = 1 */
	ret |= sn65dsi85_write(0x2D, 0x0D, 0x01);
	msleep(1);
	ret |= sn65dsi85_write(0x2D, 0x09, 0x00);

	if (ret)
		pr_debug("sn65dsi85 init failed\n");
	else
		pr_debug("sn65dsi85 init success\n");

	return ret;
}

static int find_i2c_bus(struct device *dev, void *data)
{
	pr_debug("sn65dsi85 search: %s\n", dev_name(dev->parent));
	if (strcmp(data, dev_name(dev->parent)))
		return 0;

	if (state.i2c_bus == NULL) {
		pr_debug("sn65dsi85 assigning %s to state.i2c_bus\n",
			dev_name(dev->parent));
		state.i2c_bus = to_i2c_adapter(dev);
	}

	return 1;
}

static int __init sn65dsi85_init(void)
{
	pr_info("sn65dsi85 init\n");

	i2c_for_each_dev(I2C_BUS, find_i2c_bus);

	if (state.i2c_bus == NULL) {
		pr_err("sn65dsi85: no i2c bus named %s found", I2C_BUS);
		return -ENODEV;
	}

	return __sn65dsi85_init();
}

static void __exit sn65dsi85_remove(void)
{
	if (state.enable_pin != -1)
		gpio_set_value(state.enable_pin, 1);

	if (state.i2c_bus != NULL) {
		i2c_put_adapter(state.i2c_bus);
		state.i2c_bus = NULL;
	}
	pr_info("sn65dsi85_remove\n");
}

module_init(sn65dsi85_init);
module_exit(sn65dsi85_remove);

MODULE_DESCRIPTION
    ("Device driver for TI SN65DSI85 Dual-Channel MIPI DSI to Dual-Link LVDS Bridge");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Mentor Graphics");
MODULE_VERSION(SN65DSI85_MODULE_VERSION);
