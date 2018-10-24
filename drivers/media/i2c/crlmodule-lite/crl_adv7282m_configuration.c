// SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
/*
 * Copyright (C) 2018 Mentor Graphics Corporation
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include "crlmodule.h"
#include "crlmodule-nvm.h"
#include "crlmodule-regs.h"

int adv7282m_sensor_init(struct i2c_client *client)
{
	struct ici_ext_subdev *subdev = i2c_get_clientdata(client);
	struct crl_sensor *sensor = to_crlmodule_sensor(subdev);
	int timeout;
	int ret;

	ret = crlmodule_write_reg(sensor, 0x20, 0x0f,
				  CRL_REG_LEN_08BIT, 0xff, 0x80, 0x00);
	if (ret)
		dev_warn(&client->dev, "power down failed!");

	msleep(100);

	timeout = 100;
	do {
		ret = crlmodule_write_reg(sensor, 0x20, 0x0f,
					  CRL_REG_LEN_08BIT, 0xff, 0x00, 0x00);
		msleep(1);
	} while (ret && (timeout--));

	if (timeout <= 0)
		return -ENODEV;

	dev_info(&client->dev, "chip found @ 0x%02x (%s)\n",
		 client->addr, client->adapter->name);

	return 0;
}

int adv7282m_sensor_cleanup(struct i2c_client *client)
{
	return 0;
}
