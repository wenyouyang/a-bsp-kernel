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
#define FPD3SER_MODULE_VERSION "0.1.1"

/* until ACPI is supported, devices will searched on i2b busses */
#define MAX_I2C_TRIAL 5
#define ID_STRING_LENGTH 7
#define FPD3SER_HW_VERSION_A1 1
#define FPD3SER_HW_VERSION_B1 0

#if (FPD3SER_HW_VERSION_A1 == 1)
#define FPD3SER_NR_OF_DEVICES 3
#elif (FPD3SER_HW_VERSION_B1 == 1)
#define FPD3SER_NR_OF_DEVICES 4
#else
#error undefined FPD3SER_HW_VERSION_xx
#endif

struct fpd3ser_state {
	char i2c_bus_name[32];
	int gpio_pdb;
	int gpio_int;
	struct i2c_adapter *i2c_bus;
	char id_string[ID_STRING_LENGTH];
	unsigned char i2c_ser_addr;
	unsigned char i2c_deser_addr;
};

static struct fpd3ser_state links[FPD3SER_NR_OF_DEVICES] = {
#if (FPD3SER_HW_VERSION_A1 == 1)	
	[0].i2c_bus_name = "i2c_designware.2",
	[0].id_string = "_UB949",
	[0].gpio_pdb = 454,
	[0].gpio_int = 455,
	[0].i2c_bus = NULL,
	[0].i2c_ser_addr = 0x0C,
	[0].i2c_deser_addr = 0x2C,

	[1].i2c_bus_name = "i2c_designware.6",
	[1].id_string = "_UB947",
	[1].gpio_pdb = 459,
	[1].gpio_int = 460,
	[1].i2c_bus = NULL,
	[1].i2c_ser_addr = 0x0C,
	[1].i2c_deser_addr = 0x0F,

	[2].i2c_bus_name = "i2c_designware.6",
	[2].id_string = "_UB947",
	[2].gpio_pdb = 461,
	[2].gpio_int = 462,
	[2].i2c_bus = NULL,
	[2].i2c_ser_addr = 0x12,
	[2].i2c_deser_addr = 0x15,
#elif (FPD3SER_HW_VERSION_B1 == 1)
	[0].i2c_bus_name = "i2c_designware.2",
	[0].id_string = "_UB949",
	[0].gpio_pdb = 454,
	[0].gpio_int = 455,
	[0].i2c_bus = NULL,
	[0].i2c_ser_addr = 0x0C,
	[0].i2c_deser_addr = 0x2C,

	[1].i2c_bus_name = "i2c_designware.6",
	[1].id_string = "_UB947",
	[1].gpio_pdb = 459,
	[1].gpio_int = 460,
	[1].i2c_bus = NULL,
	[1].i2c_ser_addr = 0x0C,
	[1].i2c_deser_addr = 0x0F,

	[2].i2c_bus_name = "i2c_designware.6",
	[2].id_string = "_UB947",
	[2].gpio_pdb = 461,
	[2].gpio_int = 462,
	[2].i2c_bus = NULL,
	[2].i2c_ser_addr = 0x12,
	[2].i2c_deser_addr = 0x15,

	[3].i2c_bus_name = "i2c_designware.3",
	[3].id_string = "_UB949",
	[3].gpio_pdb = 447,
	[3].gpio_int = 448,
	[3].i2c_bus = NULL,
	[3].i2c_ser_addr = 0x12,
	[3].i2c_deser_addr = 0x2C,
#else
#error undefined FPD3SER_HW_VERSION_xx
#endif	
};

static int fpd3ser_write(unsigned int index, u8 addr, u8 reg, u8 val)
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
		ret = i2c_transfer(links[index].i2c_bus, &msg, 1);
		if (1 == ret) {
			pr_debug
			    ("[%d] fpd3ser_write success: %02X [%02X]:%02X r:%d\n",
			     index, addr, reg, val, ret);
			return 0;
		}
	}

	pr_err("[%d] fpd3ser_write fail: %02X [%02X]:%02X r:%d\n", index, addr,
	       reg, val, ret);

	return ret;
}

static int fpd3ser_read(unsigned int index, u8 addr, u8 reg, u8 * buf, int len)
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
		ret = i2c_transfer(links[index].i2c_bus, msg, 2);
		if (ret >= 0) {
			pr_debug
			    ("[%d] fpd3ser_read  success: %02X [%02X]:%02X\n",
			     index, addr, reg, buf[0]);
			return 0;
		}
	}

	pr_err("[%d] fpd3ser_read  fail: %02X [%02X]:? r:%d \n", index, addr,
	       reg, ret);
	return ret;
}

static int __fpd3ser_init(void)
{
	int ret = 0;
	u8 val;
	int try = 0;
	int resp = 0;
	unsigned int i = 0;

	u8 id[ID_STRING_LENGTH] = { 0, 0, 0, 0, 0, 0, 0 };

	for (i = 0; i < ARRAY_SIZE(links); i++) {
		if (links[i].gpio_pdb != -1)
			gpio_direction_output(links[i].gpio_pdb, 1);

		msleep(10);

		memset(id, 0, ID_STRING_LENGTH);

		/* cross check */
		ret |= fpd3ser_read(i, links[0].i2c_ser_addr, 0xf0, id, 6);
		if (strncmp(id, links[i].id_string, ID_STRING_LENGTH)) {
			pr_err
			    ("[%d] fpd3ser_init: ID string mismatch (%s != %s)",
			     i, links[i].id_string, id);
			continue;
		}

		/* DS90UB949 */
		if (links[i].id_string[5] == '9') {
			/* enable I2C pass-through mode */
			ret |=
			    fpd3ser_write(i, links[i].i2c_ser_addr, 0x03, 0xDA);

			/* reset deserializer */
			ret |=
			    fpd3ser_write(i, links[i].i2c_deser_addr, 0x01,
					  0x02);

			/* wait reset bit to clear reg 0x01 bit 1 */
			do {
				resp =
				    fpd3ser_read(i, links[i].i2c_deser_addr,
						 0x01, &val, 1);
				try++;
				msleep(1);

				pr_debug("fpd3ser_init: resp:%d %02X\n", resp,
					 val);
			} while ((resp != 0) && (try < 100) && (val & 0xFD));

			/* use 24bit + sync */
			ret |=
			    fpd3ser_write(i, links[i].i2c_ser_addr, 0x1A, 0x00);
			ret |=
			    fpd3ser_write(i, links[i].i2c_ser_addr, 0x54, 0x02);
			/* failsafe to high / clear counters */
			ret |=
			    fpd3ser_write(i, links[i].i2c_ser_addr, 0x04, 0xA0);

			/* tristate serializer GPIOs  */
			ret |=
			    fpd3ser_write(i, links[i].i2c_ser_addr, 0x0D, 0x00);
			ret |=
			    fpd3ser_write(i, links[i].i2c_ser_addr, 0x0E, 0x20);
			ret |=
			    fpd3ser_write(i, links[i].i2c_ser_addr, 0x0F, 0x02);

			/* PORT1_SEL = 1 */
			/* tristate D_GPIO1 and 2 */
			ret |=
			    fpd3ser_write(i, links[i].i2c_ser_addr, 0x0E, 0x20);
			ret |=
			    fpd3ser_write(i, links[i].i2c_ser_addr, 0x1E, 0x02);
			/* PORT0_SEL = 1 */
			ret |=
			    fpd3ser_write(i, links[i].i2c_ser_addr, 0x0E, 0x01);

		} else if (links[i].id_string[5] == '7') {
			/* DS90UB947 */
			ret |=
			    fpd3ser_write(i, links[i].i2c_ser_addr, 0x03, 0xCA);
			/* deserializer address 7 bit 
			   deviate from hardware coded 0x2c */
			ret |=
			    fpd3ser_write(i, 0x2C, 0x00,
					  (links[i].i2c_deser_addr << 1) |
					  0x01);
			ret |=
			    fpd3ser_write(i, links[i].i2c_ser_addr, 0x06,
					  (links[i].i2c_deser_addr << 1) |
					  0x01);

			/* GPIO0: tristate */
			ret |=
			    fpd3ser_write(i, links[i].i2c_ser_addr, 0x0D, 0x20);
			/* GPIO2: tristate  
			   GPIO1: tristate */
			ret |=
			    fpd3ser_write(i, links[i].i2c_ser_addr, 0x0E, 0x22);
			/* GPIO3: tristate */
			ret |=
			    fpd3ser_write(i, links[i].i2c_ser_addr, 0x0F, 0x02);
			/* enable pass through all i2c addresses */
			ret |=
			    fpd3ser_write(i, links[i].i2c_ser_addr, 0x17, 0x9E);

			pr_debug("ser init %s", ret ? "failed" : "done");

		}
		/* set SCL high time */
		ret |= fpd3ser_write(i, links[i].i2c_deser_addr, 0x26, 0x16);
		/* set SCL high time */
		ret |= fpd3ser_write(i, links[i].i2c_deser_addr, 0x27, 0x16);

		/* set video disabled and override FC */
		ret |= fpd3ser_write(i, links[i].i2c_deser_addr, 0x28, 0x80);
		ret |= fpd3ser_write(i, links[i].i2c_deser_addr, 0x34, 0x89);
		ret |= fpd3ser_write(i, links[i].i2c_deser_addr, 0x49, 0x60);

		/* set TPC_RST high */
		ret |= fpd3ser_write(i, links[i].i2c_deser_addr, 0x1D, 0x09);
		/* set LAMP_ON and LAMP_PWM high */
		ret |= fpd3ser_write(i, links[i].i2c_deser_addr, 0x1E, 0x90);
		ret |= fpd3ser_write(i, links[i].i2c_deser_addr, 0x1F, 0x09);
		pr_debug("deser init %s", ret ? "failed" : "done");

	}
	return ret;
}

static int find_i2c_bus(struct device *dev, void *data)
{
	int i = 0;
	pr_debug("fpdlink3 search: %s\n", dev_name(dev->parent));

	for (i = 0; i < ARRAY_SIZE(links); i++) {
		if (strcmp(links[i].i2c_bus_name, dev_name(dev->parent)))
			continue;

		if (links[i].i2c_bus == NULL) {
			pr_info("[%d] fpdlink3 assigning %s to i2c_bus\n",
				i, dev_name(dev->parent));
			links[i].i2c_bus = to_i2c_adapter(dev);
		}
	}

	return 0;
}

static int __init fpdlink3_init(void)
{
	unsigned int i = 0;
	unsigned int err = 0;
	pr_debug("fpdlink3 init\n");

	i2c_for_each_dev(NULL, find_i2c_bus);

	for (i = 0; i < ARRAY_SIZE(links); i++) {
		if (links[i].i2c_bus == NULL) {
			pr_err("fpdlink3: no i2c bus named %s found\n",
			       links[i].i2c_bus_name);
			err++;
		}
	}

	if (err == FPD3SER_NR_OF_DEVICES) {
		pr_err("fpdlink3: none of expected devices found\n");
		return -ENODEV;
	}

	return __fpd3ser_init();
}

static void __exit fpdlink3_remove(void)
{
	unsigned int i = 0;
	for (i = 0; i < ARRAY_SIZE(links); i++) {
		if (links[i].gpio_pdb != -1)
			gpio_set_value(links[i].gpio_pdb, 0);

		if (links[i].i2c_bus != NULL) {
			i2c_put_adapter(links[i].i2c_bus);
			links[i].i2c_bus = NULL;
		}
	}
	pr_info("fpdlink3_remove\n");
}

module_init(fpdlink3_init);
module_exit(fpdlink3_remove);

MODULE_DESCRIPTION("Device driver for TI FPD-Link III Serializers");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Mentor");
MODULE_VERSION(FPD3SER_MODULE_VERSION);
