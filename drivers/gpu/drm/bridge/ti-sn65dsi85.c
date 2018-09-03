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
	struct i2c_client *client;
	int irq;
	int enable_pin;
};

static int sn65dsi85_write(struct sn65dsi85_state *state, unsigned int reg,
			   unsigned int value)
{
	dev_dbg(&state->client->dev, "%02X %02X %02X",
		state->client->addr << 1, reg & 0xff, value);
	return i2c_smbus_write_byte_data(state->client, reg & 0xff, value);
}

/* for future use, avoid warning for now 
static int sn65dsi85_read(struct sn65dsi85_state *state, unsigned int reg)
{
	int ret = 0;
	ret = i2c_smbus_read_byte_data(state->client, reg & 0xff);
	dev_dbg(&state->client->dev, "%02X %02X %02X READ",
		state->client->addr << 1, reg & 0xff, ret);
	return ret;
}
*/

static int __sn65dsi85_init(struct sn65dsi85_state *state)
{
	int ret = 0;
	if (state->enable_pin != -1)
		gpio_direction_output(state->enable_pin, 1);

	/* 

	   settings are hard coded for 
	   preliminary version

	 */

	ret |= sn65dsi85_write(state, 0x09, 0x01);
	msleep(500);
	/* PLL_EN = 0 */
	ret |= sn65dsi85_write(state, 0x0d, 0x00);
	/* LVDS_CLK_RANGE 69 MHz / DSI Clock as source */
	ret |= sn65dsi85_write(state, 0x0A, 0x05);
	/* 200MHz / 3 = 66.6 MHz ~ 69 MHz LVDS Clock  */
	ret |= sn65dsi85_write(state, 0x0B, 0x10);
	/* dual DSI 4 Lanes, 2 single DSI + 0x18.4=0 */
	/* 0: LEFT/RIGHT MODE */
	/* 10: Two single DSI recivers */
	/* 00: CHA: 4 DSI lanes */
	/* 00: CHB: 4 DSI lanes  */
	/* 1: Single bit errors tolerated  */
	ret |= sn65dsi85_write(state, 0x10, 0x41);
	/* 200 MHz CHA DSI Clock */
	ret |= sn65dsi85_write(state, 0x12, 0x28);
	/* 200 MHz CHB DSI Clock */
	ret |= sn65dsi85_write(state, 0x13, 0x28);
	/* 0x0C: 24 bpp */
	/* 0x03: _Y3 pair transmits LSB  */
	ret |= sn65dsi85_write(state, 0x18, 0x0F);
	ret |= sn65dsi85_write(state, 0x19, 0x00);

	/* serializer expects 100 ohm termination */
	ret |= sn65dsi85_write(state, 0x1A, 0x00);
	/* CHA active length 1280  */
	ret |= sn65dsi85_write(state, 0x20, 0x00);
	ret |= sn65dsi85_write(state, 0x21, 0x05);
	/* CHB active length 1280  */
	ret |= sn65dsi85_write(state, 0x22, 0x00);
	ret |= sn65dsi85_write(state, 0x23, 0x05);
	/* CHA vertical 720 */
	ret |= sn65dsi85_write(state, 0x24, 0xD0);
	ret |= sn65dsi85_write(state, 0x25, 0x02);
	/* CHB vertical 720 */
	ret |= sn65dsi85_write(state, 0x26, 0xD0);
	ret |= sn65dsi85_write(state, 0x27, 0x02);
	/* CHA sync delay 32 */
	ret |= sn65dsi85_write(state, 0x28, 0x20);
	ret |= sn65dsi85_write(state, 0x29, 0x00);
	/* CHB sync delay 32 */
	ret |= sn65dsi85_write(state, 0x2A, 0x20);
	ret |= sn65dsi85_write(state, 0x2B, 0x00);
	/* CHA hsync width 20 */
	ret |= sn65dsi85_write(state, 0x2C, 0x14);
	ret |= sn65dsi85_write(state, 0x2D, 0x00);
	/* CHB hsync width 20 */
	ret |= sn65dsi85_write(state, 0x2E, 0x14);
	ret |= sn65dsi85_write(state, 0x2F, 0x00);
	/* CHA vsync width 05 */
	ret |= sn65dsi85_write(state, 0x30, 0x05);
	ret |= sn65dsi85_write(state, 0x31, 0x00);
	/* CHB vsync width 05 */
	ret |= sn65dsi85_write(state, 0x32, 0x05);
	ret |= sn65dsi85_write(state, 0x33, 0x00);
	/* CHA horizontal back porch 90 */
	ret |= sn65dsi85_write(state, 0x34, 0x5A);
	/* CHB horizontal back porch 90 */
	ret |= sn65dsi85_write(state, 0x35, 0x5A);
	/* CHA vertical back porch 24 */
	ret |= sn65dsi85_write(state, 0x36, 0x18);
	/* CHB vertical back porch 24 */
	ret |= sn65dsi85_write(state, 0x37, 0x18);
	/* CHA horizontal front porch 90 */
	ret |= sn65dsi85_write(state, 0x38, 0x5A);
	/* CHB horizontal front porch 90 */
	ret |= sn65dsi85_write(state, 0x39, 0x5A);
	/* CHA vertical front porch 24 */
	ret |= sn65dsi85_write(state, 0x3A, 0x18);
	/* CHB vertical front porch 24 */
	ret |= sn65dsi85_write(state, 0x3B, 0x18);
	/* test pattern CHA=ON CHB=ON */
	/* ret |= sn65dsi85_write(state, 0x3C, 0x11); */
	/* PLL_EN = 1 */
	ret |= sn65dsi85_write(state, 0x0D, 0x01);
	msleep(1);
	ret |= sn65dsi85_write(state, 0x09, 0x00);

	return ret;
}

static int sn65dsi85_get_gpio(struct sn65dsi85_state *state,
			      char *def, int *gpio_nr)
{
	struct gpio_desc *gpiodescriptor;

	gpiodescriptor = devm_gpiod_get(&state->client->dev, def);
	if (IS_ERR(gpiodescriptor)) {
		*gpio_nr = -1;
		dev_err(&state->client->dev, "%s is not defined", def);
		return PTR_ERR(gpiodescriptor);
	} else {
		*gpio_nr = desc_to_gpio(gpiodescriptor);
		dev_dbg(&state->client->dev, "%s: %d", def, *gpio_nr);
	}
	return 0;
}

static int sn65dsi85_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct sn65dsi85_state *state;
	int ret = 0;

	dev_dbg(&client->dev, "%s", __func__);

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "i2c_check_functionality failed");
		return -EIO;
	}

	state = devm_kzalloc(&client->dev, sizeof(*state), GFP_KERNEL);
	if (unlikely(state == NULL)) {
		dev_err(&client->dev, "devm_kzalloc failed");
		return -ENOMEM;
	}
	state->client = client;

	/* get gpios from acpi tables */
	ret = sn65dsi85_get_gpio(state, "irq", &state->irq);
	/* interrupt is optional 
	   enable below if it is requested non-optional
	   if (ret)
	   return ret; */

	ret = sn65dsi85_get_gpio(state, "enable", &state->enable_pin);
	if (ret)
		return ret;

	ret = __sn65dsi85_init(state);
	if (ret)
		dev_err(&client->dev, "initialization failed");

	return ret;
}

static int sn65dsi85_remove(struct i2c_client *client)
{
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int sn65dsi85_suspend(struct device *dev)
{
	return 0;
}

static int sn65dsi85_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(sn65dsi85_pm_ops, sn65dsi85_suspend, sn65dsi85_resume);
#define SN65DSI85_PM_OPS (&sn65dsi85_pm_ops)

#else
#define SN65DSI85_PM_OPS NULL
#endif

static const struct i2c_device_id sn65dsi85_id[] = {
	{SN65DSI85_MODULE_NAME, (kernel_ulong_t) NULL},
	{},
};

static const struct acpi_device_id sn65dsi85_acpi_match[] = {
	{"DSI6585", (kernel_ulong_t) NULL},
	{},
};

static struct i2c_driver sn65dsi85_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = KBUILD_MODNAME,
		   .pm = SN65DSI85_PM_OPS,
		   .acpi_match_table = ACPI_PTR(sn65dsi85_acpi_match),
		   },
	.probe = sn65dsi85_probe,
	.remove = sn65dsi85_remove,
	.id_table = sn65dsi85_id,
};

module_i2c_driver(sn65dsi85_driver);

#ifndef CONFIG_ACPI
MODULE_DEVICE_TABLE(i2c, sn65dsi85_id);
#else
MODULE_DEVICE_TABLE(acpi, sn65dsi85_acpi_match);
#endif

MODULE_DESCRIPTION
    ("Device driver for TI SN65DSI85 Dual-Channel MIPI DSI to Dual-Link LVDS Bridge");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Mentor Graphics");
MODULE_VERSION(SN65DSI85_MODULE_VERSION);
