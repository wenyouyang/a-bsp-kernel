/*
 * Device driver for TI FPD-Link III Serializers
 *
 * Copyright (c) 2017-2018, Mentor - a Siemens Business
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
#include <linux/delay.h>

#define FPD3SER_MODULE_NAME "fpdlink3"
#define FPD3SER_MODULE_VERSION "0.0.1"

struct fpd3ser_state {
	int gpio_pdb;
	int gpio_int;
	struct i2c_adapter *i2c_bus;
};

/* until ACPI is supported, device will searched in the bus named as 
	I2C_BUS
*/
#define I2C_BUS  "i2c_designware.3"
#define MAX_I2C_TRIAL 5

static struct fpd3ser_state state = {
	.gpio_pdb = 456,
	.gpio_int = 457,
	.i2c_bus = NULL,
};

static int fpd3ser_write(u8 addr, u8 reg, u8 val)
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
			    ("fpd3ser_write success: %02X [%02X]:%02X r:%d\n",
			     addr, reg, val, ret);
			return 0;
		}
	}

	pr_err("fpd3ser_write fail: %02X [%02X]:%02X r:%d\n", addr, reg, val,
		ret);

	return ret;
}

static int fpd3ser_read(u8 addr, u8 reg, u8 * buf, int len)
{
	int ret;
	int i;

	struct i2c_msg msg[2] = {
		[0] = {
		       .addr = addr,
		       .flags = 0,
		       .len = 1,
		       .buf = &reg,
		       },
		[1] = {
		       .addr = addr,
		       .flags = I2C_M_RD,
		       .len = len,
		       .buf = buf,
		       },

	};

	for (i = 0; i < MAX_I2C_TRIAL; i++) {
		ret = i2c_transfer(state.i2c_bus, msg, 2);
		if (ret >= 0) {
			pr_debug("fpd3ser_read  success: %02X [%02X]:%02X\n",
				addr, reg, buf[0]);
			return 0;
		}
	}

	pr_err("fpd3ser_read  fail: %02X [%02X]:? r:%d \n", addr, reg, ret);
	return ret;
}

static int __fpd3ser_init(void)
{
	int ret = 0;
	u8 val;
	int try = 0;
	int reset_resp = 0;

	if (state.gpio_pdb != -1)
		gpio_direction_output(state.gpio_pdb, 1);

	if (state.gpio_int != -1)
		gpio_direction_input(state.gpio_int);
	msleep(10);

	/* enable I2C pass-through mode */
	ret |= fpd3ser_write(0x1A, 0x03, 0xDA);

	/* reset deserializer */
	ret |= fpd3ser_write(0x2C, 0x01, 0x02);

	/* wait reset bit to clear reg 0x01 bit 1 */
	do {
		reset_resp = fpd3ser_read(0x2C, 0x01, &val, 1);
		try++;
		msleep(1);
		
		pr_debug("fpd3ser_init: reset_resp:%d %02X\n",reset_resp, val);
	} while ((reset_resp != 0) && (try < 100) && (val & 0xFD));

	/* use 24bit + sync */
	ret |= fpd3ser_write(0x1A, 0x1A, 0x00);
	ret |= fpd3ser_write(0x1A, 0x54, 0x02);
	/* failsafe to high / clear counters */
	ret |= fpd3ser_write(0x1A, 0x04, 0xA0);

	/* tristate serializer GPIOs  */
	ret |= fpd3ser_write(0x1A, 0x0D, 0x00);
	ret |= fpd3ser_write(0x1A, 0x0E, 0x20);
	ret |= fpd3ser_write(0x1A, 0x0F, 0x02);


	/* PORT1_SEL = 1 */
	/* tristate D_GPIO1 and 2 */ 
	ret |= fpd3ser_write(0x1A, 0x0E, 0x20);
	ret |= fpd3ser_write(0x1A, 0x1E, 0x02);
	/* PORT0_SEL = 1 */
	ret |= fpd3ser_write(0x1A, 0x0E, 0x01);

	/* set SCL high time */
	ret |= fpd3ser_write(0x2C, 0x26, 0x16);
	/* set SCL high time */
	ret |= fpd3ser_write(0x2C, 0x27, 0x16);

	/* set video disabled and override FC */
	ret |= fpd3ser_write(0x2C, 0x28, 0x80);
	ret |= fpd3ser_write(0x2C, 0x34, 0x89);
	ret |= fpd3ser_write(0x2C, 0x49, 0x60);

	/* set TPC_RST high */
	ret |= fpd3ser_write(0x2C, 0x1D, 0x09);
	/* set LAMP_ON and LAMP_PWM high */
	ret |= fpd3ser_write(0x2C, 0x1E, 0x90);
	ret |= fpd3ser_write(0x2C, 0x1F, 0x09);
	return ret;
}

static int find_i2c_bus(struct device *dev, void *data)
{
	pr_debug("fpdlink3 search: %s\n", dev_name(dev->parent));
	if (strcmp(data, dev_name(dev->parent)))
		return 0;

	if (state.i2c_bus == NULL) {
		pr_debug("fpdlink3 assigning %s to state.i2c_bus\n",
			dev_name(dev->parent));
		state.i2c_bus = to_i2c_adapter(dev);
	}

	return 1;
}

static int __init fpdlink3_init(void)
{
	pr_info("fpdlink3 init\n");

	i2c_for_each_dev(I2C_BUS, find_i2c_bus);

	if (state.i2c_bus == NULL) {
		pr_err("fpdlink3: no i2c bus named %s found", I2C_BUS);
		return -ENODEV;
	}

	return __fpd3ser_init();
}

static void __exit fpdlink3_remove(void)
{
	if (state.gpio_pdb != -1)
		gpio_set_value(state.gpio_pdb, 0);

	if (state.i2c_bus != NULL) {
		i2c_put_adapter(state.i2c_bus);
		state.i2c_bus = NULL;
	}
	pr_info("fpdlink3_remove\n");
}

module_init(fpdlink3_init);
module_exit(fpdlink3_remove);

MODULE_DESCRIPTION("Device driver for TI FPD-Link III Serializers");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Mentor");
MODULE_VERSION(FPD3SER_MODULE_VERSION);
