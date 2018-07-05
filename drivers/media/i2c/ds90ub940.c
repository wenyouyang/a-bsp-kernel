/*
 * ds90ub940.c 
 * TI FPD-Link III Deserializer
 * 
 * Copyright (C) 2017 Mentor Graphics Corp.
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <media/v4l2-ioctl.h>
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/ds90ub940.h>

/* test pattern generator for debug */
static int tpg_en = 1;
module_param(tpg_en, int, 0664);
#define DS90UB940_TPG_CUSTOM_TIMING 1

#define V4L2_CID_ADV_LOCK_STATUS	(V4L2_CID_USER_ADV7180_BASE + 0x02)

struct ds90ub940_state {
	struct v4l2_ctrl_handler ctrl_hdl;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct mutex mutex;	/* mutual excl. when accessing chip */
	int irq;

	bool powered;
	u8 input;

	struct i2c_client *client;
	u32 width;
	u32 height;
	unsigned int pass_pin;
	unsigned int powerdown_pin;
};

#define to_ds90ub940_sd(_ctrl) (&container_of(_ctrl->handler,		\
					    struct ds90ub940_state,	\
					    ctrl_hdl)->sd)

static int ds90ub940_select_input(struct ds90ub940_state *state,
				  unsigned int input);

static int ds90ub940_get_input(struct ds90ub940_state *state,
			       unsigned int *ptr);

static int ds90ub940_write(struct ds90ub940_state *state, unsigned int reg,
			   unsigned int value)
{
	lockdep_assert_held(&state->mutex);
	dev_dbg(&state->client->dev, "%02X %02X %02X",
		state->client->addr << 1, reg & 0xff, value);
	return i2c_smbus_write_byte_data(state->client, reg & 0xff, value);
}

static int ds90ub940_read(struct ds90ub940_state *state, unsigned int reg)
{
	lockdep_assert_held(&state->mutex);
	return i2c_smbus_read_byte_data(state->client, reg & 0xff);
}

static int ds90ub940_csi_write(struct ds90ub940_state *state,
					 unsigned int reg, unsigned int value)
{
	ds90ub940_write(state, 0x6C, reg);
	return ds90ub940_write(state, 0x6D, value);
}

static int ds90ub940_csi_read(struct ds90ub940_state *state,
					 unsigned int reg)
{
	ds90ub940_write(state, 0x6C, reg);
	return ds90ub940_read(state, 0x6D);
}

static int ds90ub940_tpg_write(struct ds90ub940_state *state,
					unsigned int reg, unsigned int value)
{
	ds90ub940_write(state, 0x66, reg);
	return ds90ub940_write(state, 0x67, value);
}

static int ds90ub940_tpg_read(struct ds90ub940_state *state,
					 unsigned int reg)
{
	ds90ub940_write(state, 0x66, reg);
	return ds90ub940_read(state, 0x67);
}

static u32 ds90ub940_status_to_v4l2(u8 status1)
{
	u32 ret = 0;

	return ret;
}

static int __ds90ub940_get_status(struct ds90ub940_state *state,
				  u32 * ret_status)
{
	if (!state)
		return -1;

	dev_dbg(&state->client->dev, "%s", __func__);

	return 0;
}

static inline struct ds90ub940_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ds90ub940_state, sd);
}

static int ds90ub940_s_routing(struct v4l2_subdev *sd, u32 input,
			       u32 output, u32 config)
{
	struct ds90ub940_state *state = to_state(sd);
	int ret = mutex_lock_interruptible(&state->mutex);

	if (ret)
		return ret;

	ret = ds90ub940_select_input(state, input);

	mutex_unlock(&state->mutex);
	return ret;
}

static int ds90ub940_g_input_status(struct v4l2_subdev *sd, u32 * status)
{
	struct ds90ub940_state *state = to_state(sd);
	int ret;
	dev_dbg(&state->client->dev, "%s", __func__);

	ret = mutex_lock_interruptible(&state->mutex);
	if (ret)
		return ret;

	ret = __ds90ub940_get_status(state, status);

	mutex_unlock(&state->mutex);

	return ret;
}

static int ds90ub940_set_power(struct ds90ub940_state *state, bool on)
{
	int ret = 0;

	/* write to power management reg */
	if (on) {
		// ret = ds90ub940_write(state, 0x0F, 0x00);
		state->powered = true;
	} else {
		// ret = ds90ub940_write(state, 0x0F, 0x20);
		state->powered = false;
	}

	if (ret) {
		dev_err(&state->client->dev, "set power failed!");
		return ret;
	}

	return 0;
}

static int ds90ub940_s_power(struct v4l2_subdev *sd, int on)
{
	struct ds90ub940_state *state = to_state(sd);
	int ret;

	ret = mutex_lock_interruptible(&state->mutex);
	if (ret)
		return ret;

	ret = ds90ub940_set_power(state, on);

	mutex_unlock(&state->mutex);
	return ret;
}

static int ds90ub940_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_ds90ub940_sd(ctrl);
	struct ds90ub940_state *state = to_state(sd);
	int ret = mutex_lock_interruptible(&state->mutex);
	int val;

	dev_dbg(&state->client->dev, "%s", __func__);

	if (ret)
		return ret;
	val = ctrl->val;
	switch (ctrl->id) {
	case V4L2_CID_LINK_FREQ:
		dev_dbg(&state->client->dev, "%s: Link freq is constant %d",
			__func__, ctrl->val);
		break;
	default:
		dev_info(&state->client->dev,
			 "%s: case 0x%X observed and not handled", __func__,
			 ctrl->id);

		ret = -EINVAL;
	}

	mutex_unlock(&state->mutex);
	return ret;
}

static int ds90ub940_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_ds90ub940_sd(ctrl);
	struct ds90ub940_state *state = to_state(sd);
	int ret = mutex_lock_interruptible(&state->mutex);

	dev_dbg(&state->client->dev, "%s", __func__);

	if (ret)
		return ret;

	switch (ctrl->id) {
	case V4L2_CID_ADV_LOCK_STATUS:
		ret = __ds90ub940_get_status(state, &ctrl->val);
		break;

	default:
		dev_info(&state->client->dev,
			 "%s: case 0x%X observed and not handled", __func__,
			 ctrl->id);
		ret = -EINVAL;
	}

	mutex_unlock(&state->mutex);
	return ret;
}

static const struct v4l2_ctrl_ops ds90ub940_ctrl_ops = {
	.s_ctrl = ds90ub940_s_ctrl,
	.g_volatile_ctrl = ds90ub940_g_volatile_ctrl,
};

static const struct v4l2_ctrl_config ds90ub940_ctrl_lock_status = {
	.ops = &ds90ub940_ctrl_ops,
	.id = V4L2_CID_ADV_LOCK_STATUS,
	.name = "Lock Status",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.step = 1,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
};

static const s64 ds90ub940_op_sys_clock[] = { 136400000UL };

static int ds90ub940_init_controls(struct ds90ub940_state *state)
{
	v4l2_ctrl_handler_init(&state->ctrl_hdl, 6);

	v4l2_ctrl_new_int_menu(&state->ctrl_hdl, &ds90ub940_ctrl_ops,
			       V4L2_CID_LINK_FREQ, 0, 0,
			       ds90ub940_op_sys_clock);

	v4l2_ctrl_new_custom(&state->ctrl_hdl, &ds90ub940_ctrl_lock_status,
			     NULL);

	state->sd.ctrl_handler = &state->ctrl_hdl;
	if (state->ctrl_hdl.error) {
		int err = state->ctrl_hdl.error;

		v4l2_ctrl_handler_free(&state->ctrl_hdl);
		return err;
	}
	v4l2_ctrl_handler_setup(&state->ctrl_hdl);

	return 0;
}

static void ds90ub940_exit_controls(struct ds90ub940_state *state)
{
	v4l2_ctrl_handler_free(&state->ctrl_hdl);
}

static int ds90ub940_enum_mbus_code(struct v4l2_subdev *sd,
				    struct v4l2_subdev_pad_config *cfg,
				    struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_UYVY8_1X16;

	return 0;
}

static int ds90ub940_get_pad_format(struct v4l2_subdev *sd,
				    struct v4l2_subdev_pad_config *cfg,
				    struct v4l2_subdev_format *format)
{
	struct ds90ub940_state *state = to_state(sd);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		format->format = *v4l2_subdev_get_try_format(sd, cfg, 0);
	} else {
		format->format.code = MEDIA_BUS_FMT_UYVY8_1X16;
		format->format.colorspace = V4L2_COLORSPACE_SMPTE170M;
		format->format.width = state->width;
		format->format.height = state->height;
	}

	return 0;
}

static int ds90ub940_set_pad_format(struct v4l2_subdev *sd,
				    struct v4l2_subdev_pad_config *cfg,
				    struct v4l2_subdev_format *format)
{
	struct ds90ub940_state *state = to_state(sd);
	struct v4l2_mbus_framefmt *framefmt;

	dev_info(&state->client->dev, "%s", __func__);

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		framefmt = &format->format;
	} else {
		framefmt = v4l2_subdev_get_try_format(sd, cfg, 0);
		*framefmt = format->format;
	}

	state->width = format->format.width;
	state->height = format->format.height;
	framefmt->code = MEDIA_BUS_FMT_UYVY8_1X16;
	framefmt->colorspace = V4L2_COLORSPACE_SMPTE170M;

	return 0;
}

static int ds90ub940_g_mbus_config(struct v4l2_subdev *sd,
				   struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_CSI2;
	cfg->flags = V4L2_MBUS_CSI2_1_LANE |
	    V4L2_MBUS_CSI2_CHANNEL_0 | V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	return 0;
}

static int ds90ub940_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct ds90ub940_state *state = to_state(subdev);

	dev_info(&state->client->dev, "ds90ub940_s_stream %s\n",
		 (enable ? "on" : "off"));
/*
	if (state && (state->powered != true)) {
		dev_err(&state->client->dev, "can't start stream");
		return -EIO;
	}
*/
	if (enable) {
		return ds90ub940_write(state, 0x64, 0xC5);
	} else {
		return ds90ub940_write(state, 0x64, 0xC4);
	}
	return 0;
}

static long ds90ub940_ioctl(struct v4l2_subdev *subdev, unsigned int cmd,
			    void *arg)
{
	struct ds90ub940_state *state = to_state(subdev);
	int ret = 0;
	unsigned int input = 0;

	switch (cmd) {
	case VIDIOC_S_INPUT:
		/* check range */
		ret = ds90ub940_s_routing(subdev, *(unsigned int *)arg, 0, 0);
		dev_dbg(&state->client->dev, "set input %X ret:%d",
			*(unsigned int *)arg, ret);
		break;
	case VIDIOC_G_INPUT:
		ret = ds90ub940_get_input(state, &input);
		*((unsigned int *)arg) = input;
		dev_dbg(&state->client->dev, "get input %X ret:%d",
			*(unsigned int *)arg, ret);
		break;
	case VIDIOC_QUERYCAP:
		{
			struct v4l2_capability r;
			memset(&r, 0, sizeof(r));
			strcpy(r.driver, "ds90ub940");
			sprintf(r.bus_info, "I2C: bus %d 7-bit Addr:0x%X",
				state->client->adapter->nr,
				state->client->addr);
			r.version = KERNEL_VERSION(1, 0, 0);
			r.device_caps = V4L2_CAP_STREAMING;
			r.capabilities = r.device_caps | V4L2_CAP_DEVICE_CAPS;
			memcpy(arg, &r, sizeof(struct v4l2_capability));
			dev_dbg(&state->client->dev, "VIDIOC_QUERYCAP");
			break;
		}
	default:
		dev_err(&state->client->dev, "can't handle ioctl 0x%X", cmd);
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_subdev_video_ops ds90ub940_video_ops = {
	.g_input_status = ds90ub940_g_input_status,
	.s_routing = ds90ub940_s_routing,
	.g_mbus_config = ds90ub940_g_mbus_config,
	.s_stream = ds90ub940_s_stream,
};

static const struct v4l2_subdev_core_ops ds90ub940_core_ops = {
	.s_power = ds90ub940_s_power,
	.ioctl = ds90ub940_ioctl,
};

static const struct v4l2_subdev_pad_ops ds90ub940_pad_ops = {
	.enum_mbus_code = ds90ub940_enum_mbus_code,
	.set_fmt = ds90ub940_set_pad_format,
	.get_fmt = ds90ub940_get_pad_format,
};

static const struct v4l2_subdev_ops ds90ub940_ops = {
	.core = &ds90ub940_core_ops,
	.video = &ds90ub940_video_ops,
	.pad = &ds90ub940_pad_ops,
};

static int ds90ub940_get_input(struct ds90ub940_state *state, unsigned int *ptr)
{
	int ret = mutex_lock_interruptible(&state->mutex);
	if (ret)
		return ret;

	/* TODO */

	mutex_unlock(&state->mutex);

	return 0;
}

static int ds90ub940_select_input(struct ds90ub940_state *state,
				  unsigned int input)
{

	/* TODO */

	return 0;
}

static int __ds90ub940_init_tpg(struct ds90ub940_state *state)
{
#if DS90UB940_TPG_CUSTOM_TIMING
	unsigned int total_h = 0;
	unsigned int total_v = 0;
	unsigned int active_h = 0;
	unsigned int active_v = 0;
#endif	
	/* 2 Data lines, continuous clock */    
	ds90ub940_write(state, 0x6A, 0x22);	   
#if DS90UB940_TPG_CUSTOM_TIMING
	/* Custom timing */ 
	/* PGCDC = 0x06, set clock divider to 6 */
	ds90ub940_tpg_write(state, 0x03, 0x06);	 
	/* PGAFS1 = 0x20, set active horizontal width */
	ds90ub940_tpg_write(state, 0x07, 0x20);
	/* PGAFS2 = 0x03, set active vertical and horizontal width */
	ds90ub940_tpg_write(state, 0x08, 0x03);
	/* PGAFS3 = 0x1E, set active vertical  */
	ds90ub940_tpg_write(state, 0x09, 0x1E);	
	/* PGTFS1 = 0x20, total horizontal width */
	ds90ub940_tpg_write(state, 0x04, 0x98);	
	/* PGTFS2 = 0x20, total horizontal and vertical width */
	ds90ub940_tpg_write(state, 0x05, 0xD4);	
	/* PGTFS3 = 0x20, total vertical width */
	ds90ub940_tpg_write(state, 0x06, 0x20);	
	/* PGHBP = 0xD8, horizontal back porch */
	ds90ub940_tpg_write(state, 0x0C, 0xD8);	
	/* PGVBP = 0x23, vertical back porch */
	ds90ub940_tpg_write(state, 0x0D, 0x23);

	total_h = ((ds90ub940_tpg_read(state, 0x05) & 0xf ) << 8) | 
		ds90ub940_tpg_read(state,0x04);

	total_v = (ds90ub940_tpg_read(state, 0x06) << 4) | 
		((ds90ub940_tpg_read(state,0x05) & 0xf0) >> 4);

	active_h = ((ds90ub940_tpg_read(state, 0x08) & 0xf ) << 8) | 
		ds90ub940_tpg_read(state,0x07);
	
	active_v = (ds90ub940_tpg_read(state, 0x09) << 4) | 
		((ds90ub940_tpg_read(state,0x08) & 0xf0) >> 4);

	dev_info(&state->client->dev, "TPG: Total:%dx%d Active:%dx%d", 
		total_h, total_v, active_h, active_v );
#endif
	/* PGCFG Pattern generator creates its own timing */
	ds90ub940_write(state, 0x65, 0x05);	

	return 0;
}

static int __ds90ub940_init_device(struct ds90ub940_state *state)
{
	int ret = 0;
	int timeout;

	mutex_lock(&state->mutex);

	/* chip reset */
	ds90ub940_write(state, 0x01, 0x03);

	/* wait until self-clearing reset bits are clear */
	timeout = 100;
	while (( 0x03 & ds90ub940_read(state, 0x01) && timeout--)){
		usleep_range(10, 50);
		dev_dbg(&state->client->dev, "waiting reset %d", (100-timeout));
	}

	ds90ub940_set_power(state, true);
	if (tpg_en)
	{
		__ds90ub940_init_tpg(state);
	}

	/* set output format YUV422 */ 
	ds90ub940_write(state, 0x6B, 0x50);
	
	mutex_unlock(&state->mutex);

	return ret;
}

static int ds90ub940_check_device_exists(struct i2c_client *client)
{
	u8 id[7] = {0,0,0,0,0,0,0};
	int ret = i2c_smbus_read_i2c_block_data(client, 0xF0, 6, id);
	if (ret == 6) {
		if ( memcmp(id, "_UB940", 6) == 0 || 
			memcmp(id, "_UH940",6) == 0) {
			return 0;
		}
	}
	dev_info(&client->dev, "unexpected id string:%s",id);
	return -1;
}

static int ds90ub940_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct ds90ub940_state *state;
	struct v4l2_subdev *sd;
	int ret;
	struct ds90ub940_platform_data *pdata;

	dev_dbg(&client->dev, "%s", __func__);

	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "%s can't probe, No platform data", __func__);
		return -ENODEV;
	}

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "i2c_check_functionality failed");
		return -EIO;
	}

	state = devm_kzalloc(&client->dev, sizeof(*state), GFP_KERNEL);
	if (state == NULL) {
		dev_err(&client->dev, "devm_kzalloc failed");
		return -ENOMEM;
	}


	pdata = client->dev.platform_data;
	
	dev_dbg(&client->dev, "gpios: %d %d ", pdata->powerdown_pin, 
		pdata->pass_pin);
#if 0
	if (!pdata->pass_pin || !pdata->powerdown_pin) {
		dev_warn(&client->dev, "powerdown and/or pass pin not defined.");
		ret = -EINVAL;
		goto err_free_state;
	}
#else
	state->powerdown_pin = pdata->powerdown_pin;
	state->pass_pin = pdata->pass_pin;

	gpio_set_value(state->powerdown_pin, 1);
#endif	
/*
	if (devm_gpio_request_one(&client->dev, state->powerdown_pin,
				  GPIOF_OUT_INIT_HIGH,
				  "ds90ub940 powerdown")) {
		dev_err(&client->dev, "failed to request GPIO %d (PDN)\n",
			state->powerdown_pin);
		ret = -EBUSY;
		goto err_free_state;
	}
*/
	if (ds90ub940_check_device_exists(client)) {
		dev_info(&client->dev, "no ds90ub940 found.");
		return -ENODEV;
	}

	v4l_info(client, "chip found @ 0x%02x (%s)\n",
		 client->addr, client->adapter->name);

	state->client = client;

	state->irq = client->irq;
	mutex_init(&state->mutex);

	sd = &state->sd;
	v4l2_i2c_subdev_init(sd, client, &ds90ub940_ops);
	sd->flags = V4L2_SUBDEV_FL_HAS_DEVNODE;

	ret = ds90ub940_init_controls(state);
	if (ret) {
		dev_err(&client->dev, "can't init controls");
		goto err_unregister_csi_client;
	}

	state->pad.flags = MEDIA_PAD_FL_SOURCE;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	sd->entity.type |= MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
#else
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
#endif	
	sd->entity.flags = MEDIA_ENT_FL_DEFAULT;
	/* drop the i2c bus and addr from media entity name */
	memset(&sd->name, 0, sizeof(sd->name));
	strncpy((char *)&sd->name,
		"ds90ub940",
		strlen("ds90ub940"));

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	ret = media_entity_init(&sd->entity, 1, &state->pad, 0);
#else
	ret = media_entity_pads_init(&sd->entity, 1, &state->pad);
#endif
	if (ret) {
		dev_err(&client->dev, "can't initialize media entity");
		goto err_free_ctrl;
	}

	ret = __ds90ub940_init_device(state);
	if (ret) {
		dev_err(&client->dev, "can't initialize device");
		goto err_media_entity_cleanup;
	}

	ret = v4l2_async_register_subdev(sd);
	if (ret) {
		dev_err(&client->dev, "can't register register subdev");
		goto err_media_entity_cleanup;
	}

	return 0;

 err_media_entity_cleanup:
	media_entity_cleanup(&sd->entity);
 err_free_ctrl:
	ds90ub940_exit_controls(state);
 err_unregister_csi_client:
	mutex_destroy(&state->mutex);
 err_free_state:
	devm_kfree(&client->dev, state);
	return ret;
}

static int ds90ub940_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ds90ub940_state *state = to_state(sd);

	v4l2_async_unregister_subdev(sd);

	media_entity_cleanup(&sd->entity);
	ds90ub940_exit_controls(state);

	mutex_destroy(&state->mutex);

	return 0;
}

static void ds90ub940_shutdown(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ds90ub940_state *state = to_state(sd);
	
	gpio_set_value(state->powerdown_pin, 0);
	return; 
}

static const struct i2c_device_id ds90ub940_id[] = {
	{"ds90ub940", (kernel_ulong_t) NULL},
	{},
};

MODULE_DEVICE_TABLE(i2c, ds90ub940_id);

#ifdef CONFIG_PM_SLEEP
static int ds90ub940_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ds90ub940_state *state = to_state(sd);

	return ds90ub940_set_power(state, false);
}

static int ds90ub940_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ds90ub940_state *state = to_state(sd);
	int ret;

	ret = __ds90ub940_init_device(state);
	if (ret < 0)
		return ret;

	ret = ds90ub940_set_power(state, state->powered);
	if (ret)
		return ret;

	return 0;
}

static SIMPLE_DEV_PM_OPS(ds90ub940_pm_ops, ds90ub940_suspend, ds90ub940_resume);
#define DS90UB940_PM_OPS (&ds90ub940_pm_ops)

#else
#define DS90UB940_PM_OPS NULL
#endif

static struct i2c_driver ds90ub940_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = KBUILD_MODNAME,
		   .pm = DS90UB940_PM_OPS,
		   },
	.probe = ds90ub940_probe,
	.remove = ds90ub940_remove,
	.shutdown = ds90ub940_shutdown,
	.id_table = ds90ub940_id,
};

module_i2c_driver(ds90ub940_driver);

MODULE_DESCRIPTION("TI DS90UB940 FPD-Link III Deserializer");
MODULE_AUTHOR("Mentor Graphics Corp.");
MODULE_LICENSE("GPL v2");
