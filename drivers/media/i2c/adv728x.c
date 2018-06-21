/*
 * adv728x.c Analog Devices ADV728x video decoder driver
 * 
 * Based on ADV7180 Driver 
 *
 * Copyright (c) 2009 Intel Corporation
 * Copyright (C) 2013 Cogent Embedded, Inc.
 * Copyright (C) 2013 Renesas Solutions Corp.
 * Copyright (c) 2017 Mentor Graphics Corp.
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
#include <media/v4l2-ioctl.h>
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/property.h>
#include <linux/string.h>
#include <linux/media.h>

#define ADV728X_MODULE_VERSION "3.0.0"
/* without ACPI */
#define ADV728X_WO_ACPI 1
/* following have no effect when ACPI is defined */ 
#define ADV728X_DEFAULT_MODEL "ADV7282-M"
#define ADV728X_GPIO_BASE	434
#define ADV728X_GPIO_RST	1
#define ADV728X_GPIO_PDN	0
#define ADV728X_GPIO_INT	22

#define ADV728X_STD_AD_PAL_BG_NTSC_J_SECAM		0x0
#define ADV728X_STD_AD_PAL_BG_NTSC_J_SECAM_PED		0x1
#define ADV728X_STD_AD_PAL_N_NTSC_J_SECAM		0x2
#define ADV728X_STD_AD_PAL_N_NTSC_M_SECAM		0x3
#define ADV728X_STD_NTSC_J				0x4
#define ADV728X_STD_NTSC_M				0x5
#define ADV728X_STD_PAL60				0x6
#define ADV728X_STD_NTSC_443				0x7
#define ADV728X_STD_PAL_BG				0x8
#define ADV728X_STD_PAL_N				0x9
#define ADV728X_STD_PAL_M				0xa
#define ADV728X_STD_PAL_M_PED				0xb
#define ADV728X_STD_PAL_COMB_N				0xc
#define ADV728X_STD_PAL_COMB_N_PED			0xd
#define ADV728X_STD_PAL_SECAM				0xe
#define ADV728X_STD_PAL_SECAM_PED			0xf

#define ADV728X_REG_INPUT_CONTROL			0x0000
#define ADV728X_INPUT_CONTROL_INSEL_MASK		0x1f

#define ADV728X_REG_INPUT_VIDSEL			0x0002

/* Contrast */
#define ADV728X_REG_CON		0x0008	/*Unsigned */
#define ADV728X_CON_MIN		0
#define ADV728X_CON_DEF		128
#define ADV728X_CON_MAX		255
/* Brightness*/
#define ADV728X_REG_BRI		0x000a	/*Signed */
#define ADV728X_BRI_MIN		-128
#define ADV728X_BRI_DEF		0
#define ADV728X_BRI_MAX		127
/* Hue */
#define ADV728X_REG_HUE		0x000b	/*Signed, inverted */
#define ADV728X_HUE_MIN		-127
#define ADV728X_HUE_DEF		0
#define ADV728X_HUE_MAX		128

#define ADV728X_REG_CTRL		0x000e

#define ADV728X_REG_STATUS1		0x0010
#define ADV728X_STATUS1_IN_LOCK		0x01
#define ADV728X_STATUS1_FSC_LOCK	0x04
#define ADV728X_STATUS1_COL_KILL	0x80
#define ADV728X_STATUS1_AUTOD_MASK	0x70
#define ADV728X_STATUS1_AUTOD_NTSM_M_J	0x00
#define ADV728X_STATUS1_AUTOD_NTSC_4_43 0x10
#define ADV728X_STATUS1_AUTOD_PAL_M	0x20
#define ADV728X_STATUS1_AUTOD_PAL_60	0x30
#define ADV728X_STATUS1_AUTOD_PAL_B_G	0x40
#define ADV728X_STATUS1_AUTOD_SECAM	0x50
#define ADV728X_STATUS1_AUTOD_PAL_COMB	0x60
#define ADV728X_STATUS1_AUTOD_SECAM_525	0x70

/* Saturation */
#define ADV728X_REG_SD_SAT_CB	0x00e3	/*Unsigned */
#define ADV728X_REG_SD_SAT_CR	0x00e4	/*Unsigned */
#define ADV728X_SAT_MIN		0
#define ADV728X_SAT_DEF		128
#define ADV728X_SAT_MAX		255

#define ADV728X_INT_DRIVE_LOW	0x01
#define ADV728X_INT_UNTIL_CLEARED	0xC0

#define ADV728X_INPUT_CVBS_AIN1 0x00
#define ADV728X_INPUT_CVBS_AIN2 0x01
#define ADV728X_INPUT_CVBS_AIN3 0x02
#define ADV728X_INPUT_CVBS_AIN4 0x03
#define ADV728X_INPUT_CVBS_AIN5 0x04
#define ADV728X_INPUT_CVBS_AIN6 0x05
#define ADV728X_INPUT_CVBS_AIN7 0x06
#define ADV728X_INPUT_CVBS_AIN8 0x07
#define ADV728X_INPUT_SVIDEO_AIN1_AIN2 0x08
#define ADV728X_INPUT_SVIDEO_AIN3_AIN4 0x09
#define ADV728X_INPUT_SVIDEO_AIN5_AIN6 0x0a
#define ADV728X_INPUT_SVIDEO_AIN7_AIN8 0x0b
#define ADV728X_INPUT_YPRPB_AIN1_AIN2_AIN3 0x0c
#define ADV728X_INPUT_YPRPB_AIN4_AIN5_AIN6 0x0d
#define ADV728X_INPUT_DIFF_CVBS_AIN1_AIN2 0x0e
#define ADV728X_INPUT_DIFF_CVBS_AIN3_AIN4 0x0f
#define ADV728X_INPUT_DIFF_CVBS_AIN5_AIN6 0x10
#define ADV728X_INPUT_DIFF_CVBS_AIN7_AIN8 0x11
#define ADV728X_INPUT_MAX ADV728X_INPUT_DIFF_CVBS_AIN7_AIN8
/* to warn other side about an reserved value in register */
#define ADV728X_INPUT_NONE 0xFF

#define ADV728X_DEFAULT_CSI_I2C_ADDR 0x44	/* 7-Bit */
#define ADV728X_DEFAULT_VPP_I2C_ADDR 0x42	/* 7-Bit */

#define V4L2_CID_ADV_FAST_SWITCH	(V4L2_CID_USER_ADV7180_BASE + 0x01)
#define V4L2_CID_ADV_LOCK_STATUS	(V4L2_CID_USER_ADV7180_BASE + 0x02)

/* power-up sequence */
#define ADV728X_POWERUPSEQ_OPTIMAL 0x00	/* use PWRDWN_N, RESET_N pins */
#define ADV728X_POWERUPSEQ_SIMPLIFIED 0x01	/* use soft reset + 10ms delay */

enum variant_index {
	ADV728X_VARIANT_7280_M,
	ADV728X_VARIANT_7281_M,
	ADV728X_VARIANT_7281_MA,
	ADV728X_VARIANT_7282_M,
	ADV728X_VARIANT_COUNT,
};

struct adv728x_variant_info {
	/* used for ACPI device matching
	   as seen on user guide all capitals with dash */
	char model_str[16];
	/* existance of on-chip deinterlacer */
	bool have_i2p;
	/* available video input */
	unsigned int valid_input_mask;
	/* diagnostic pins */
	bool have_diag_pins;
	/* entity name for media control library */
	char media_entity_name[V4L2_SUBDEV_NAME_SIZE];
};

const struct adv728x_variant_info adv728x_variants[] = {
	[ADV728X_VARIANT_7280_M] = {
				    .model_str = "ADV7280-M",
				    .have_i2p = true,
				    .valid_input_mask =
				    BIT(ADV728X_INPUT_CVBS_AIN1) |
				    BIT(ADV728X_INPUT_CVBS_AIN2) |
				    BIT(ADV728X_INPUT_CVBS_AIN3) |
				    BIT(ADV728X_INPUT_CVBS_AIN4) |
				    BIT(ADV728X_INPUT_CVBS_AIN5) |
				    BIT(ADV728X_INPUT_CVBS_AIN6) |
				    BIT(ADV728X_INPUT_CVBS_AIN7) |
				    BIT(ADV728X_INPUT_CVBS_AIN8) |
				    BIT(ADV728X_INPUT_SVIDEO_AIN1_AIN2) |
				    BIT(ADV728X_INPUT_SVIDEO_AIN3_AIN4) |
				    BIT(ADV728X_INPUT_SVIDEO_AIN5_AIN6) |
				    BIT(ADV728X_INPUT_SVIDEO_AIN7_AIN8) |
				    BIT(ADV728X_INPUT_YPRPB_AIN1_AIN2_AIN3) |
				    BIT(ADV728X_INPUT_YPRPB_AIN4_AIN5_AIN6),
				    .have_diag_pins = false,
				    .media_entity_name = "adv7280m"},
	[ADV728X_VARIANT_7281_M] = {
				    .model_str = "ADV7281-M",
				    .have_i2p = false,
				    .valid_input_mask =
				    BIT(ADV728X_INPUT_CVBS_AIN1) |
				    BIT(ADV728X_INPUT_CVBS_AIN2) |
				    BIT(ADV728X_INPUT_CVBS_AIN3) |
				    BIT(ADV728X_INPUT_CVBS_AIN4) |
				    BIT(ADV728X_INPUT_CVBS_AIN7) |
				    BIT(ADV728X_INPUT_CVBS_AIN8) |
				    BIT(ADV728X_INPUT_SVIDEO_AIN1_AIN2) |
				    BIT(ADV728X_INPUT_SVIDEO_AIN3_AIN4) |
				    BIT(ADV728X_INPUT_SVIDEO_AIN7_AIN8) |
				    BIT(ADV728X_INPUT_YPRPB_AIN1_AIN2_AIN3) |
				    BIT(ADV728X_INPUT_DIFF_CVBS_AIN1_AIN2) |
				    BIT(ADV728X_INPUT_DIFF_CVBS_AIN3_AIN4) |
				    BIT(ADV728X_INPUT_DIFF_CVBS_AIN7_AIN8),
				    .have_diag_pins = true,
				    .media_entity_name = "adv7281m"},
	[ADV728X_VARIANT_7281_MA] = {
				     .model_str = "ADV7281-MA",
				     .have_i2p = false,
				     .valid_input_mask =
				     BIT(ADV728X_INPUT_CVBS_AIN1) |
				     BIT(ADV728X_INPUT_CVBS_AIN2) |
				     BIT(ADV728X_INPUT_CVBS_AIN3) |
				     BIT(ADV728X_INPUT_CVBS_AIN4) |
				     BIT(ADV728X_INPUT_CVBS_AIN5) |
				     BIT(ADV728X_INPUT_CVBS_AIN6) |
				     BIT(ADV728X_INPUT_CVBS_AIN7) |
				     BIT(ADV728X_INPUT_CVBS_AIN8) |
				     BIT(ADV728X_INPUT_SVIDEO_AIN1_AIN2) |
				     BIT(ADV728X_INPUT_SVIDEO_AIN3_AIN4) |
				     BIT(ADV728X_INPUT_SVIDEO_AIN5_AIN6) |
				     BIT(ADV728X_INPUT_SVIDEO_AIN7_AIN8) |
				     BIT(ADV728X_INPUT_YPRPB_AIN1_AIN2_AIN3) |
				     BIT(ADV728X_INPUT_YPRPB_AIN4_AIN5_AIN6) |
				     BIT(ADV728X_INPUT_DIFF_CVBS_AIN1_AIN2) |
				     BIT(ADV728X_INPUT_DIFF_CVBS_AIN3_AIN4) |
				     BIT(ADV728X_INPUT_DIFF_CVBS_AIN5_AIN6) |
				     BIT(ADV728X_INPUT_DIFF_CVBS_AIN7_AIN8),
				     .have_diag_pins = false,
				     .media_entity_name = "adv7281ma"},
	[ADV728X_VARIANT_7282_M] = {
				    .model_str = "ADV7282-M",
				    .have_i2p = true,
				    .valid_input_mask =
				    BIT(ADV728X_INPUT_CVBS_AIN1) |
				    BIT(ADV728X_INPUT_CVBS_AIN2) |
				    BIT(ADV728X_INPUT_CVBS_AIN3) |
				    BIT(ADV728X_INPUT_CVBS_AIN4) |
				    BIT(ADV728X_INPUT_CVBS_AIN5) |
				    BIT(ADV728X_INPUT_CVBS_AIN6) |
				    BIT(ADV728X_INPUT_CVBS_AIN7) |
				    BIT(ADV728X_INPUT_CVBS_AIN8) |
				    BIT(ADV728X_INPUT_SVIDEO_AIN1_AIN2) |
				    BIT(ADV728X_INPUT_SVIDEO_AIN3_AIN4) |
				    BIT(ADV728X_INPUT_SVIDEO_AIN5_AIN6) |
				    BIT(ADV728X_INPUT_SVIDEO_AIN7_AIN8) |
				    BIT(ADV728X_INPUT_YPRPB_AIN1_AIN2_AIN3) |
				    BIT(ADV728X_INPUT_YPRPB_AIN4_AIN5_AIN6) |
				    BIT(ADV728X_INPUT_DIFF_CVBS_AIN1_AIN2) |
				    BIT(ADV728X_INPUT_DIFF_CVBS_AIN3_AIN4) |
				    BIT(ADV728X_INPUT_DIFF_CVBS_AIN5_AIN6) |
				    BIT(ADV728X_INPUT_DIFF_CVBS_AIN7_AIN8),
				    .have_diag_pins = true,
				    .media_entity_name = "adv7282m"},
};

struct adv728x_state {
	struct v4l2_ctrl_handler ctrl_hdl;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct mutex mutex;	/* mutual excl. when accessing chip */
	int irq;
	v4l2_std_id expected_tv_std;
	bool autodetect;
	bool powered;
	u8 input;

	struct i2c_client *client;
	unsigned int register_page;
	struct i2c_client *csi_client;
	struct i2c_client *vpp_client;
	enum v4l2_field field;
	/* return this value when g_input_status is asked and interrupt is not set */
	u32 cached_status;
	u32 cached_sync_status;
	bool bad_sync;
	int reset_pin;
	int powerdown_pin;
	enum variant_index variant;
	unsigned int power_seq;
	/* range: 0-63 / -1 : uninitialized or failed to read from ACPI */
	int syncdepth_threshold_bad;
	/* range: 0-63 / -1 : uninitialized or failed to read from ACPI */
	int syncdepth_threshold_good;
};

#define to_adv728x_sd(_ctrl) (&container_of(_ctrl->handler,		\
					    struct adv728x_state,	\
					    ctrl_hdl)->sd)

static int adv728x_select_input(struct adv728x_state *state,
				unsigned int input);

static int adv728x_get_input(struct adv728x_state *state, unsigned int *ptr);

static int adv728x_select_page(struct adv728x_state *state, unsigned int page)
{
	if (state->register_page != page) {
		i2c_smbus_write_byte_data(state->client, ADV728X_REG_CTRL,
					  page);
		dev_dbg(&state->client->dev, "%02X %02X %02X",
			state->client->addr << 1, ADV728X_REG_CTRL, page);
		state->register_page = page;
	}

	return 0;
}

static int adv728x_write(struct adv728x_state *state, unsigned int reg,
			 unsigned int value)
{
	lockdep_assert_held(&state->mutex);
	adv728x_select_page(state, reg >> 8);
	dev_dbg(&state->client->dev, "%02X %02X %02X",
		state->client->addr << 1, reg & 0xff, value);
	return i2c_smbus_write_byte_data(state->client, reg & 0xff, value);
}

static int adv728x_read(struct adv728x_state *state, unsigned int reg)
{
	lockdep_assert_held(&state->mutex);
	adv728x_select_page(state, reg >> 8);
	return i2c_smbus_read_byte_data(state->client, reg & 0xff);
}

static int adv728x_csi_write(struct adv728x_state *state, unsigned int reg,
			     unsigned int value)
{
	dev_dbg(&state->client->dev, "%02X %02X %02X",
		state->csi_client->addr << 1, reg, value);
	return i2c_smbus_write_byte_data(state->csi_client, reg, value);
}

static int adv728x_vpp_write(struct adv728x_state *state, unsigned int reg,
			     unsigned int value)
{
	dev_dbg(&state->client->dev, "%02X %02X %02X",
		state->vpp_client->addr << 1, reg, value);
	return i2c_smbus_write_byte_data(state->vpp_client, reg, value);
}

static int adv728x_set_video_standard(struct adv728x_state *state,
				      unsigned int std)
{
	/* register 0x02 - user sub map
	   bitwise or 0x04: not to disturb undocumented/default register values */
	return adv728x_write(state, ADV728X_REG_INPUT_VIDSEL, std << 4 | 0x04);
}

static v4l2_std_id adv728x_std_to_v4l2(u8 status1)
{
	/* in case V4L2_IN_ST_NO_SIGNAL */
	if (!(status1 & ADV728X_STATUS1_IN_LOCK) || status1 == 0xFF)
		return V4L2_STD_UNKNOWN;

	switch (status1 & ADV728X_STATUS1_AUTOD_MASK) {
	case ADV728X_STATUS1_AUTOD_NTSM_M_J:
		return V4L2_STD_NTSC;
	case ADV728X_STATUS1_AUTOD_NTSC_4_43:
		return V4L2_STD_NTSC_443;
	case ADV728X_STATUS1_AUTOD_PAL_M:
		return V4L2_STD_PAL_M;
	case ADV728X_STATUS1_AUTOD_PAL_60:
		return V4L2_STD_PAL_60;
	case ADV728X_STATUS1_AUTOD_PAL_B_G:
		return V4L2_STD_PAL;
	case ADV728X_STATUS1_AUTOD_SECAM:
		return V4L2_STD_SECAM;
	case ADV728X_STATUS1_AUTOD_PAL_COMB:
		return V4L2_STD_PAL_Nc | V4L2_STD_PAL_N;
	case ADV728X_STATUS1_AUTOD_SECAM_525:
		return V4L2_STD_SECAM;
	default:
		return V4L2_STD_UNKNOWN;
	}
}

static int v4l2_std_to_adv728x(v4l2_std_id std)
{
	if (std == V4L2_STD_PAL_60)
		return ADV728X_STD_PAL60;
	if (std == V4L2_STD_NTSC_443)
		return ADV728X_STD_NTSC_443;
	if (std == V4L2_STD_PAL_N)
		return ADV728X_STD_PAL_N;
	if (std == V4L2_STD_PAL_M)
		return ADV728X_STD_PAL_M;
	if (std == V4L2_STD_PAL_Nc)
		return ADV728X_STD_PAL_COMB_N;

	if (std & V4L2_STD_PAL)
		return ADV728X_STD_PAL_BG;
	if (std & V4L2_STD_NTSC)
		return ADV728X_STD_NTSC_M;
	if (std & V4L2_STD_SECAM)
		return ADV728X_STD_PAL_SECAM;

	return -EINVAL;
}

static u32 adv728x_status_to_v4l2(u8 status1)
{
	u32 ret = 0;

	/* cached_status can be marked with -1 */
	if (status1 == -1)
		return V4L2_IN_ST_NO_SIGNAL;

	if (!(status1 & ADV728X_STATUS1_IN_LOCK))
		ret = V4L2_IN_ST_NO_SIGNAL;

	if (!(status1 & ADV728X_STATUS1_FSC_LOCK))
		ret |= V4L2_IN_ST_NO_COLOR;

	if (status1 & ADV728X_STATUS1_COL_KILL)
		ret |= V4L2_IN_ST_COLOR_KILL;

	return ret;
}

static int __adv728x_update_cached_status(struct adv728x_state *state)
{
	if (!state)
		return -1;

	state->cached_status = adv728x_read(state, ADV728X_REG_STATUS1);
	/* raw status 3 */
	state->cached_sync_status = adv728x_read(state, 0x2049);
	dev_dbg(&state->client->dev, "reading status again");

	if ((state->cached_status & 0x03) == 0x03) {
		/*
		 * The video stream locked after a lost-lock condition:
		 *
		 * We need to update the lost-lock bit in the status cache.
		 * This bit is automatically reset by the ADV after a register
		 * read command. The ADV does not raise an interrupt for this.
		 * So we have to invalidate the status cache to re-read the 
		 * actual value.
		 */

		/* invalidate status cache */
		dev_dbg(&state->client->dev, "### invalidating cached status");
		state->cached_status = -1;
	}

	return 0;
}

/* returns status from register 0x10 ADV728X_REG_STATUS1 */
static u32 __adv728x_get_status(struct adv728x_state *state)
{
	u32 ret_status = V4L2_IN_ST_NO_SIGNAL;
	v4l2_std_id std = V4L2_STD_UNKNOWN;

	if (!state)
		return ret_status;

	dev_dbg(&state->client->dev, "%s", __func__);

	if (gpio_is_valid(state->irq)) {
		/* platform supports ADV728X interrupts */
		int irq_status;
		/* get interrupt status */
		irq_status = gpio_get_value(state->irq);

		if (irq_status == 0 || state->cached_status == -1) {
			/* clear interrupts */
			adv728x_write(state, 0x2043, 0xFF);
			adv728x_write(state, 0x204B, 0xFF);

			/* status has changed, read new value from register */
			__adv728x_update_cached_status(state);
		} else {
			/* status not changed, use cached value */
			dev_dbg(&state->client->dev,
				"no interrupts, cached statuses are up-to-date");
		}
	} else {
		/* platform does not support ADV728X interrupts; poll status register */
		__adv728x_update_cached_status(state);

		dev_dbg(&state->client->dev,
			"platform does not support interrupts");
	}
	dev_dbg(&state->client->dev, "[%d] cached_status:%#X",
		state->irq, state->cached_status);

	/* translate raw register value into V4L2 */
	ret_status = adv728x_status_to_v4l2(state->cached_status);
	if ((state->cached_sync_status & 0x06) != 0x06) {
		ret_status |= V4L2_IN_ST_NO_H_LOCK;
		dev_info(&state->client->dev, "sync(s) missing");
	} else if (state->syncdepth_threshold_bad != -1
		   && state->syncdepth_threshold_good != -1) {
		int sync_depth;

		/* read sync-depth register */
		sync_depth = adv728x_read(state, 0x8079);

		/*
		 * If sync depth is below threshold value, we assume a bad video signal.
		 * In order to avoid jittering we also do a hysteresis test.
		 */
		if (!state->bad_sync
		    && sync_depth <= state->syncdepth_threshold_bad) {
			/* sync levels are bad */
			state->bad_sync = true;
			dev_info(&state->client->dev,
				 "bad video signal (sync depth=%d)\n",
				 sync_depth);
		}

		if (state->bad_sync) {
			if (sync_depth >= state->syncdepth_threshold_good) {
				/* sync levels are good again */
				state->bad_sync = false;
				dev_info(&state->client->dev,
					 "video signal is good again (sync depth=%d)\n",
					 sync_depth);
			} else {
				/* signal is bad */
				ret_status |= V4L2_IN_ST_NO_H_LOCK;
			}
		}
	}

	std = adv728x_std_to_v4l2(state->cached_status);
	/* log unexpected tv std */
	if (((ret_status & V4L2_IN_ST_NO_H_LOCK) == 0)
	    && (std != V4L2_STD_UNKNOWN)
	    && (!(std & state->expected_tv_std)))
		dev_warn(&state->client->dev,
			 "found a tv standard other than the expected");

	return ret_status;
}

/* returns to detected tv standard */
static v4l2_std_id adv728x_get_tv_std(struct adv728x_state *state)
{
	v4l2_std_id std = V4L2_STD_UNKNOWN;
	u32 status = 0;

	if (!state)
		goto exit_get_tv_std;

	status = __adv728x_get_status(state);

	/* read the detected tv std from status register if signal present */
	if (!(status & (V4L2_IN_ST_NO_SIGNAL | V4L2_IN_ST_NO_H_LOCK))) {
		/* used cached_status if it is read at least once not invalid */
		if (state->cached_status == -1) {
			__adv728x_update_cached_status(state);
		} else {
			dev_dbg(&state->client->dev, "using cached");
		}
		std = adv728x_std_to_v4l2(state->cached_status);
	}

 exit_get_tv_std:
	return std;
}

static inline struct adv728x_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct adv728x_state, sd);
}

static int adv728x_querystd(struct v4l2_subdev *sd, v4l2_std_id * std)
{
	struct adv728x_state *state = to_state(sd);
	int err = mutex_lock_interruptible(&state->mutex);
	if (err)
		return err;

	/* get detected std directly from the chip */
	*std = adv728x_get_tv_std(state);

	mutex_unlock(&state->mutex);
	return err;
}

static int adv728x_s_routing(struct v4l2_subdev *sd, u32 input,
			     u32 output, u32 config)
{
	struct adv728x_state *state = to_state(sd);
	int ret = mutex_lock_interruptible(&state->mutex);

	if (ret)
		return ret;

	ret = adv728x_select_input(state, input);

	mutex_unlock(&state->mutex);
	return ret;
}

static int adv728x_g_input_status(struct v4l2_subdev *sd, u32 * status)
{
	struct adv728x_state *state = to_state(sd);
	int ret;
	dev_dbg(&state->client->dev, "%s", __func__);

	ret = mutex_lock_interruptible(&state->mutex);
	if (ret)
		return ret;

	*status = __adv728x_get_status(state);

	mutex_unlock(&state->mutex);

	return ret;
}

static int adv728x_program_std(struct adv728x_state *state)
{
	int ret;

	if (state->autodetect) {
		ret = adv728x_set_video_standard(state,
						 ADV728X_STD_AD_PAL_BG_NTSC_J_SECAM);
		if (ret < 0)
			return ret;
	} else {
		ret = v4l2_std_to_adv728x(state->expected_tv_std);
		if (ret < 0)
			return ret;

		ret = adv728x_set_video_standard(state, ret);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int adv728x_s_std(struct v4l2_subdev *sd, v4l2_std_id std)
{
	struct adv728x_state *state = to_state(sd);
	int ret = mutex_lock_interruptible(&state->mutex);

	if (ret)
		return ret;

	/* all standards -> autodetect */
	if (std == V4L2_STD_ALL) {
		state->autodetect = true;
		state->expected_tv_std = V4L2_STD_ALL;
	} else {
		/* Make sure we can support this std */
		ret = v4l2_std_to_adv728x(std);
		if (ret < 0) {
			dev_err(&state->client->dev, "standard not supported");
			goto out;
		}

		state->expected_tv_std = std;
		state->autodetect = false;
	}

	ret = adv728x_program_std(state);
 out:
	mutex_unlock(&state->mutex);
	return ret;
}

static int adv728x_g_std(struct v4l2_subdev *sd, v4l2_std_id * std)
{
	struct adv728x_state *state = to_state(sd);
	int ret = mutex_lock_interruptible(&state->mutex);

	if (std)
		*std = state->expected_tv_std;
	else
		ret = -EINVAL;

	mutex_unlock(&state->mutex);
	return ret;
}

static int adv728x_set_power(struct adv728x_state *state, bool on)
{
	return 0;
}

static int adv728x_s_power(struct v4l2_subdev *sd, int on)
{
	struct adv728x_state *state = to_state(sd);
	int ret;

	ret = mutex_lock_interruptible(&state->mutex);
	if (ret)
		return ret;

	ret = adv728x_set_power(state, on);

	mutex_unlock(&state->mutex);
	return ret;
}

static int adv728x_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_adv728x_sd(ctrl);
	struct adv728x_state *state = to_state(sd);
	int ret = mutex_lock_interruptible(&state->mutex);
	int val;

	dev_dbg(&state->client->dev, "%s", __func__);

	if (ret)
		return ret;
	val = ctrl->val;
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		ret = adv728x_write(state, ADV728X_REG_BRI, val);
		break;
	case V4L2_CID_HUE:
		/*Hue is inverted according to HSL chart */
		ret = adv728x_write(state, ADV728X_REG_HUE, -val);
		break;
	case V4L2_CID_CONTRAST:
		ret = adv728x_write(state, ADV728X_REG_CON, val);
		break;
	case V4L2_CID_SATURATION:
		/*
		 *This could be V4L2_CID_BLUE_BALANCE/V4L2_CID_RED_BALANCE
		 *Let's not confuse the user, everybody understands saturation
		 */
		ret = adv728x_write(state, ADV728X_REG_SD_SAT_CB, val);
		if (ret < 0)
			break;
		ret = adv728x_write(state, ADV728X_REG_SD_SAT_CR, val);
		break;
	case V4L2_CID_ADV_FAST_SWITCH:
		if (ctrl->val) {
			/* ADI required write */
			adv728x_write(state, 0x80d9, 0x44);
			/* enable fast locking */
			adv728x_write(state, 0x40e0, 0x01);
		} else {
			/* ADI required write */
			adv728x_write(state, 0x80d9, 0xc4);
			/* disable fast locking */
			adv728x_write(state, 0x40e0, 0x00);
		}
		break;
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

static int adv728x_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_adv728x_sd(ctrl);
	struct adv728x_state *state = to_state(sd);
	int ret = mutex_lock_interruptible(&state->mutex);

	dev_dbg(&state->client->dev, "%s", __func__);

	if (ret)
		return ret;

	switch (ctrl->id) {
	case V4L2_CID_ADV_LOCK_STATUS:
		ctrl->val = __adv728x_get_status(state);
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

static const struct v4l2_ctrl_ops adv728x_ctrl_ops = {
	.s_ctrl = adv728x_s_ctrl,
	.g_volatile_ctrl = adv728x_g_volatile_ctrl,
};

static const struct v4l2_ctrl_config adv728x_ctrl_fast_switch = {
	.ops = &adv728x_ctrl_ops,
	.id = V4L2_CID_ADV_FAST_SWITCH,
	.name = "Fast Switching",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.step = 1,
};

static const struct v4l2_ctrl_config adv728x_ctrl_lock_status = {
	.ops = &adv728x_ctrl_ops,
	.id = V4L2_CID_ADV_LOCK_STATUS,
	.name = "Lock Status",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.step = 1,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
};

static const s64 adv728x_op_sys_clock[] = { 135000000UL };

static int adv728x_init_controls(struct adv728x_state *state)
{
	v4l2_ctrl_handler_init(&state->ctrl_hdl, 6);

	v4l2_ctrl_new_std(&state->ctrl_hdl, &adv728x_ctrl_ops,
			  V4L2_CID_BRIGHTNESS, ADV728X_BRI_MIN,
			  ADV728X_BRI_MAX, 1, ADV728X_BRI_DEF);
	v4l2_ctrl_new_std(&state->ctrl_hdl, &adv728x_ctrl_ops,
			  V4L2_CID_CONTRAST, ADV728X_CON_MIN,
			  ADV728X_CON_MAX, 1, ADV728X_CON_DEF);
	v4l2_ctrl_new_std(&state->ctrl_hdl, &adv728x_ctrl_ops,
			  V4L2_CID_SATURATION, ADV728X_SAT_MIN,
			  ADV728X_SAT_MAX, 1, ADV728X_SAT_DEF);
	v4l2_ctrl_new_std(&state->ctrl_hdl, &adv728x_ctrl_ops,
			  V4L2_CID_HUE, ADV728X_HUE_MIN,
			  ADV728X_HUE_MAX, 1, ADV728X_HUE_DEF);

	v4l2_ctrl_new_int_menu(&state->ctrl_hdl, &adv728x_ctrl_ops,
			       V4L2_CID_LINK_FREQ, 0, 0, adv728x_op_sys_clock);

	v4l2_ctrl_new_custom(&state->ctrl_hdl, &adv728x_ctrl_fast_switch, NULL);
	v4l2_ctrl_new_custom(&state->ctrl_hdl, &adv728x_ctrl_lock_status, NULL);

	state->sd.ctrl_handler = &state->ctrl_hdl;
	if (state->ctrl_hdl.error) {
		int err = state->ctrl_hdl.error;

		v4l2_ctrl_handler_free(&state->ctrl_hdl);
		return err;
	}
	v4l2_ctrl_handler_setup(&state->ctrl_hdl);

	return 0;
}

static void adv728x_exit_controls(struct adv728x_state *state)
{
	v4l2_ctrl_handler_free(&state->ctrl_hdl);
}

static int adv728x_mbus_fmt(struct v4l2_subdev *sd,
			    struct v4l2_mbus_framefmt *fmt)
{
	struct adv728x_state *state = to_state(sd);

	fmt->code = MEDIA_BUS_FMT_UYVY8_1X16;
	fmt->colorspace = V4L2_COLORSPACE_SMPTE170M;
	fmt->width = 720;
	if (fmt->field == V4L2_FIELD_NONE) {
		fmt->height =
		    adv728x_get_tv_std(state) & V4L2_STD_625_50 ? 576 : 487;
	} else {
		fmt->height =
		    adv728x_get_tv_std(state) & V4L2_STD_625_50 ?
		    (576 / 2) : 244;
	}
	dev_dbg(&state->client->dev, "format:%dx%d field:%d",
		fmt->width, fmt->height, fmt->field);
	return 0;
}

static int adv728x_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_UYVY8_1X16;

	return 0;
}

static int adv728x_get_pad_format(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_format *format)
{
	struct adv728x_state *state = to_state(sd);
	mutex_lock(&state->mutex);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *fmt;	
		fmt = v4l2_subdev_get_try_format(sd, cfg, 0);
		if (fmt->field == V4L2_FIELD_NONE)
			format->format.field = V4L2_FIELD_NONE;
		else
			format->format.field = V4L2_FIELD_ALTERNATE;
	} else {
		format->format.field = state->field;
	}
	adv728x_mbus_fmt(sd, &format->format);

	mutex_unlock(&state->mutex);
	return 0;
}

static int adv728x_set_pad_format(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_format *fmt)
{
	struct adv728x_state *state = to_state(sd);
	int r = mutex_lock_interruptible(&state->mutex);
	if (r)
		return r;

	dev_dbg(&state->client->dev, "%s", __func__);

	switch (fmt->format.field) {
	case V4L2_FIELD_NONE:
		/* correct if non-existing I2P is tried to be activated */
		dev_dbg(&state->client->dev, "V4L2_FIELD_NONE");
		if (!adv728x_variants[state->variant].have_i2p) {
			fmt->format.field = V4L2_FIELD_ALTERNATE;
			dev_dbg(&state->client->dev, "no deinterlacer");
		}
		break;
	default:
		/* alternate is default */
		dev_dbg(&state->client->dev,
			 "V4L2_FIELD_ALTERNATE is default. tried:%d",
			 fmt->format.field);
		fmt->format.field = V4L2_FIELD_ALTERNATE;
		break;
	}

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		dev_dbg(&state->client->dev, "V4L2_SUBDEV_FORMAT_ACTIVE");
		if (state->field != fmt->format.field) {
			state->field = fmt->format.field;
			dev_dbg(&state->client->dev, "field has changed");
			/* enable deinterlacer I2P 
			   should not be performed during streaming */
			if (adv728x_variants[state->variant].have_i2p) {
				dev_dbg(&state->client->dev, "there is i2p");
				if (state->field == V4L2_FIELD_NONE) {
					/* switch on I2P */
					adv728x_vpp_write(state, 0xA3, 0x00);
					adv728x_vpp_write(state, 0x5B, 0x00);
					adv728x_vpp_write(state, 0x55, 0x80);
					adv728x_csi_write(state, 0x01, 0x20);
					adv728x_csi_write(state, 0x02, 0x28);
					adv728x_csi_write(state, 0x03, 0x38);
					adv728x_csi_write(state, 0x04, 0x30);
					adv728x_csi_write(state, 0x05, 0x30);
					adv728x_csi_write(state, 0x06, 0x80);
					adv728x_csi_write(state, 0x07, 0x70);
					adv728x_csi_write(state, 0x08, 0x50);
					adv728x_csi_write(state, 0x1D, 0x80);
					dev_dbg(&state->client->dev,
						"switch I2P ON");
				} else {
					/* switch off I2P 
					   values are power-up defaults
					   unnecessary to load them if I2P is not existing
					 */
					adv728x_vpp_write(state, 0xA3, 0x70);
					adv728x_vpp_write(state, 0x5B, 0x80);
					adv728x_vpp_write(state, 0x55, 0x00);
					adv728x_csi_write(state, 0x01, 0x18);
					adv728x_csi_write(state, 0x02, 0x18);
					adv728x_csi_write(state, 0x03, 0x30);
					adv728x_csi_write(state, 0x04, 0x20);
					adv728x_csi_write(state, 0x05, 0x28);
					adv728x_csi_write(state, 0x06, 0x40);
					adv728x_csi_write(state, 0x07, 0x58);
					adv728x_csi_write(state, 0x08, 0x30);
					adv728x_csi_write(state, 0x1D, 0x00);
					dev_dbg(&state->client->dev,
						"switch I2P OFF");
				}
			} else {
				dev_dbg(&state->client->dev, "NO i2p");
			}
		}
	} else {
		struct v4l2_mbus_framefmt *framefmt;
		framefmt = v4l2_subdev_get_try_format(sd, cfg, 0);
		framefmt->field = fmt->format.field;
		dev_dbg(&state->client->dev, "V4L2_SUBDEV_FORMAT_TRY %d",
			fmt->format.field);
	}
	mutex_unlock(&state->mutex);
	return adv728x_mbus_fmt(sd, &fmt->format);
}

static int adv728x_g_mbus_config(struct v4l2_subdev *sd,
				 struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_CSI2;
	cfg->flags = V4L2_MBUS_CSI2_1_LANE |
	    V4L2_MBUS_CSI2_CHANNEL_0 | V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	return 0;
}

static int adv728x_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct adv728x_state *state = to_state(subdev);

	dev_dbg(&state->client->dev, "adv728x_s_stream %s\n",
		(enable ? "on" : "off"));

	if (state && (state->powered != true)) {
		dev_err(&state->client->dev, "can't start stream");
		return -EIO;
	}

	if (enable) {
		/* Power up MIPI CSI-2 Tx */
		adv728x_csi_write(state, 0x00, 0x00);
	} else {
		/* Power down MIPI CSI-2 Tx */
		adv728x_csi_write(state, 0x00, 0x80);
	}
	return 0;
}

static long adv728x_ioctl(struct v4l2_subdev *subdev, unsigned int cmd,
			  void *arg)
{
	struct adv728x_state *state = to_state(subdev);
	int ret = 0;
	v4l2_std_id std = V4L2_STD_UNKNOWN;
	unsigned int input = ADV728X_INPUT_NONE;

	switch (cmd) {
	case VIDIOC_QUERYSTD:
		ret = adv728x_querystd(subdev, &std);
		*((v4l2_std_id *) arg) = std;
		dev_dbg(&state->client->dev, "detected %X ret:%d",
			*(unsigned int *)arg, ret);
		break;
	case VIDIOC_S_STD:
		ret = adv728x_s_std(subdev, *(v4l2_std_id *) arg);
		dev_dbg(&state->client->dev, "set tv std %X ret:%d",
			*(unsigned int *)arg, ret);
		break;
	case VIDIOC_G_STD:
		ret = adv728x_g_std(subdev, &std);
		*((v4l2_std_id *) arg) = std;
		dev_dbg(&state->client->dev, "get tv std %X ret:%d",
			*(unsigned int *)arg, ret);
		break;
	case VIDIOC_S_INPUT:
		/* check range */
		if (*(unsigned int *)arg > ADV728X_INPUT_MAX) {
			dev_err(&state->client->dev,
				"VIDIOC_S_INPUT: not a valid input value %d",
				*(unsigned int *)arg);
			return -1;
		}
		ret = adv728x_s_routing(subdev, *(unsigned int *)arg, 0, 0);
		dev_dbg(&state->client->dev, "set input %X ret:%d",
			*(unsigned int *)arg, ret);
		break;
	case VIDIOC_G_INPUT:
		ret = adv728x_get_input(state, &input);
		*((unsigned int *)arg) = input;
		dev_dbg(&state->client->dev, "get input %X ret:%d",
			*(unsigned int *)arg, ret);
		break;
	case VIDIOC_QUERYCAP:
		{
			struct v4l2_capability r;
			memset(&r, 0, sizeof(r));
			strcpy(r.driver, "adv728x");
			sprintf(r.bus_info, "I2C: bus %d 7-bit Addr:0x%X",
				state->client->adapter->nr,
				state->client->addr);
			r.version = KERNEL_VERSION(3, 0, 0);
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

static const struct v4l2_subdev_video_ops adv728x_video_ops = {
	.g_input_status = adv728x_g_input_status,
	.s_routing = adv728x_s_routing,
	.g_mbus_config = adv728x_g_mbus_config,
	.s_stream = adv728x_s_stream,
};

static const struct v4l2_subdev_core_ops adv728x_core_ops = {
	.s_power = adv728x_s_power,
	.ioctl = adv728x_ioctl,
};

static const struct v4l2_subdev_pad_ops adv728x_pad_ops = {
	.enum_mbus_code = adv728x_enum_mbus_code,
	.set_fmt = adv728x_set_pad_format,
	.get_fmt = adv728x_get_pad_format,
};

static const struct v4l2_subdev_ops adv728x_ops = {
	.core = &adv728x_core_ops,
	.video = &adv728x_video_ops,
	.pad = &adv728x_pad_ops,
};

enum adv728x_input_type {
	ADV728X_INPUT_TYPE_CVBS,
	ADV728X_INPUT_TYPE_DIFF_CVBS,
	ADV728X_INPUT_TYPE_SVIDEO,
	ADV728X_INPUT_TYPE_YPBPR,
};

static enum adv728x_input_type adv728x_get_input_type(unsigned int input)
{
	switch (input) {
	case ADV728X_INPUT_CVBS_AIN1:
	case ADV728X_INPUT_CVBS_AIN2:
	case ADV728X_INPUT_CVBS_AIN3:
	case ADV728X_INPUT_CVBS_AIN4:
	case ADV728X_INPUT_CVBS_AIN5:
	case ADV728X_INPUT_CVBS_AIN6:
	case ADV728X_INPUT_CVBS_AIN7:
	case ADV728X_INPUT_CVBS_AIN8:
		return ADV728X_INPUT_TYPE_CVBS;
	case ADV728X_INPUT_SVIDEO_AIN1_AIN2:
	case ADV728X_INPUT_SVIDEO_AIN3_AIN4:
	case ADV728X_INPUT_SVIDEO_AIN5_AIN6:
	case ADV728X_INPUT_SVIDEO_AIN7_AIN8:
		return ADV728X_INPUT_TYPE_SVIDEO;
	case ADV728X_INPUT_YPRPB_AIN1_AIN2_AIN3:
	case ADV728X_INPUT_YPRPB_AIN4_AIN5_AIN6:
		return ADV728X_INPUT_TYPE_YPBPR;
	case ADV728X_INPUT_DIFF_CVBS_AIN1_AIN2:
	case ADV728X_INPUT_DIFF_CVBS_AIN3_AIN4:
	case ADV728X_INPUT_DIFF_CVBS_AIN5_AIN6:
	case ADV728X_INPUT_DIFF_CVBS_AIN7_AIN8:
		return ADV728X_INPUT_TYPE_DIFF_CVBS;
	default:		/* Will never happen */
		return 0;
	}
}

/* ADI recommended writes to registers 0x52, 0x53, 0x54 */
static unsigned int adv728x_lbias_settings[][3] = {
	[ADV728X_INPUT_TYPE_CVBS] = {0xCD, 0x4E, 0x80},
	[ADV728X_INPUT_TYPE_DIFF_CVBS] = {0xC0, 0x4E, 0x80},
	[ADV728X_INPUT_TYPE_SVIDEO] = {0x0B, 0xCE, 0x80},
	[ADV728X_INPUT_TYPE_YPBPR] = {0x0B, 0x4E, 0xC0},
};

static int adv728x_get_input(struct adv728x_state *state, unsigned int *ptr)
{
	unsigned int input;
	int ret = mutex_lock_interruptible(&state->mutex);
	if (ret)
		return ret;

	input = adv728x_read(state, ADV728X_REG_INPUT_CONTROL);
	if ((input & 0x1F) > ADV728X_INPUT_MAX) {
		/* should never happen */
		*ptr = ADV728X_INPUT_NONE;
		dev_err(&state->client->dev,
			"reserved value in input selection register");
		return -EFAULT;
	} else {
		*ptr = input;
	}

	mutex_unlock(&state->mutex);

	return 0;
}

static int adv728x_select_input(struct adv728x_state *state, unsigned int input)
{
	enum adv728x_input_type input_type;
	unsigned int *lbias;
	unsigned int i;
	int ret;

	if (input > ADV728X_INPUT_MAX)
		return -EINVAL;

	if (!(BIT(input) & adv728x_variants[state->variant].valid_input_mask)) {
		dev_err(&state->client->dev, "invalid input");
		return -EINVAL;
	}

	/* register 0x00 user sub map */
	ret = adv728x_write(state, ADV728X_REG_INPUT_CONTROL, input);
	if (ret)
		return ret;

	/* Reset clamp circuitry - ADI recommended writes */
	adv728x_write(state, 0x809c, 0x00);
	adv728x_write(state, 0x809c, 0xff);

	input_type = adv728x_get_input_type(input);

	switch (input_type) {
	case ADV728X_INPUT_TYPE_CVBS:
	case ADV728X_INPUT_TYPE_DIFF_CVBS:
		/* ADI recommends to use the SH1 filter */
		adv728x_write(state, 0x0017, 0x41);
		break;
	default:
		adv728x_write(state, 0x0017, 0x01);
		break;
	}

	lbias = adv728x_lbias_settings[input_type];

	for (i = 0; i < ARRAY_SIZE(adv728x_lbias_settings[0]); i++)
		adv728x_write(state, 0x0052 + i, lbias[i]);

	if (input_type == ADV728X_INPUT_TYPE_DIFF_CVBS) {
		/* ADI required writes to make differential CVBS work */
		adv728x_write(state, 0x005f, 0xa8);
		adv728x_write(state, 0x005a, 0x90);
		adv728x_write(state, 0x0060, 0xb0);
		adv728x_write(state, 0x80b6, 0x08);
		adv728x_write(state, 0x80c0, 0xa0);
		adv728x_write(state, 0x80d9, 0x44);
		/* enable fast lock */
		adv728x_write(state, 0x40e0, 0x01);
	} else {
		adv728x_write(state, 0x005f, 0xf0);
		adv728x_write(state, 0x005a, 0xd0);
		adv728x_write(state, 0x0060, 0x10);
		adv728x_write(state, 0x80b6, 0x9c);
		adv728x_write(state, 0x80c0, 0x00);
	}

	/* update state */
	state->input = input;
	return 0;
}

static int __adv728x_init_device(struct adv728x_state *state)
{
	int ret;
	int timeout;
	int video_input;

	/* don't reply user space before init */
	mutex_lock(&state->mutex);

	/* power-up sequence */
	if (state->power_seq == ADV728X_POWERUPSEQ_OPTIMAL) {
		gpio_direction_output(state->powerdown_pin, 1);
		msleep(5);
		gpio_direction_output(state->reset_pin,1);
		msleep(5);
	}

	/* chip reset */
	adv728x_write(state, 0x0F, 0x80);
	/* sleep 10 ms after software reset 
	   also means: fulfill ADV728X_POWERUPSEQ_SIMPLIFIED */
	if (state->power_seq == ADV728X_POWERUPSEQ_SIMPLIFIED) {
		msleep(10);
	} else {
		msleep(5);
	}

	/* power up chip 
	   in case it is not ready after the reset try several times */
	timeout = 5;
	while ((0 > adv728x_write(state, 0x0F, 0x00) && timeout--)) {
		dev_info(&state->client->dev, "power up failed!");
		msleep(1);
	}

	if (timeout <= 0) {
		ret = -ENODEV;
		goto out_unlock;
	}
	state->powered = true;

	v4l_info(state->client, "chip found @ 0x%02x (%s)\n",
		 state->client->addr, state->client->adapter->name);

	/* initialize register page 0x000E */
	adv728x_write(state, ADV728X_REG_CTRL, 0);
	state->register_page = 0;

	/* set I2C address for CSI map */
	adv728x_write(state, 0xFE, ADV728X_DEFAULT_CSI_I2C_ADDR << 1);

	/* ADI recommended writes for improved video quality */
	adv728x_write(state, 0x0080, 0x51);
	adv728x_write(state, 0x0081, 0x51);
	adv728x_write(state, 0x0082, 0x68);

	/* ADI required writes */
	adv728x_write(state, 0x0003, 0x4e);
	/* enable BT656.4 */
	adv728x_write(state, 0x0004, 0xd7);
	adv728x_write(state, 0x001d, 0xc0);

	adv728x_write(state, 0x0013, 0x00);

	ret =
	    device_property_read_u32(&state->client->dev, "default-video-input",
				     &video_input);
	if (ret == 0) {
		ret = adv728x_select_input(state, video_input);
		dev_dbg(&state->client->dev, "video input: %d", video_input);
	} else {
		ret =
		    adv728x_select_input(state,
					 ADV728X_INPUT_DIFF_CVBS_AIN1_AIN2);
		dev_warn(&state->client->dev, "can't find default video input");
	}

	if (ret) {
		ret = -EINVAL;
		goto out_unlock;
	}

	/* tv standard is initialised with expected_tv_std and autodetect */
	ret = adv728x_program_std(state);
	if (ret)
		goto out_unlock;

	adv728x_set_power(state, true);

	if (gpio_is_valid(state->irq)) {
		/* configure interrupt pin / register 0x40 interrupt submap */
		/* interrupt configuration 1 */
		adv728x_write(state, 0x2040,
			      ADV728X_INT_DRIVE_LOW |
			      ADV728X_INT_UNTIL_CLEARED);
		/* enable SD_LOCK SD_UNLOCK interrupts interrupt mask 1 */
		adv728x_write(state, 0x2044, 0x03);
		/* clear interrupts interrupt clear 1 */
		adv728x_write(state, 0x2043, 0xFF);
		/* no interrupts from interrupt mask 2 */
		adv728x_write(state, 0x2048, 0x00);
		/* clear interrupts interrupt clear 2 */
		adv728x_write(state, 0x2047, 0xFF);
		/* enable SD_AD_CHNG SD_OP_CHNG SD_V_LOCK_CHNG SD_H_LOCK_CHNG interrupts 
		   interrupt mask 3 */
		adv728x_write(state, 0x204C, 0x0F);
		/* clear interrupts interrupt clear 3 */
		adv728x_write(state, 0x204B, 0xFF);
		/* no interrupts from interrupt mask 4 */
		adv728x_write(state, 0x2050, 0x00);
		/* clear interrupts interrupt clear 4 */
		adv728x_write(state, 0x204F, 0xFF);
		/* invalidate status cache */
		state->cached_status = -1;
	}

	if (adv728x_variants[state->variant].have_i2p) {
		/* set I2C address for VPP map */
		ret =
		    adv728x_write(state, 0xFD,
				  ADV728X_DEFAULT_VPP_I2C_ADDR << 1);
		dev_dbg(&state->client->dev, "VPP addr %s written",
			ret ? "NOT" : " ");
	}

	ret = 0;
	/* Power up MIPI D-PHY */
	ret |= adv728x_csi_write(state, 0xDE, 0x02);
	/* 5 ms is decided to let IPU detect LP->HS transition 09.12.2016 */
	msleep(5);
	/* 4* ADI Required Write */
	ret |= adv728x_csi_write(state, 0xD2, 0xF7);
	ret |= adv728x_csi_write(state, 0xD8, 0x65);
	ret |= adv728x_csi_write(state, 0xE0, 0x09);
	ret |= adv728x_csi_write(state, 0x2C, 0x00);
	/* if there is an I2P and it is enabled */
	if (adv728x_variants[state->variant].have_i2p
	    && state->field == V4L2_FIELD_NONE) {
		ret |= adv728x_csi_write(state, 0x1D, 0x80);
	} else {
		ret |= adv728x_csi_write(state, 0x1D, 0x00);
	}

	if (ret)
		dev_err(&state->client->dev, "can't initialize csi map");

 out_unlock:
	mutex_unlock(&state->mutex);
	return ret;
}

static int adv728x_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct adv728x_state *state;
	struct v4l2_subdev *sd;
	int ret;
	const char *model = NULL;
	int i;
	struct gpio_desc *gpiodescriptor;

	dev_dbg(&client->dev, "%s", __func__);
	pr_info("%s", __func__);

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

	/* read the model string from _DSD */
	if (device_property_read_string(&client->dev, "model", &model)) {
#if (ADV728X_WO_ACPI == 1)
		model = ADV728X_DEFAULT_MODEL;
#else
		dev_err(&client->dev, "can't find model in _DSD (ACPI)");
		return -ENODEV;
#endif
	}

	/* find index in variants table */
	for (i = 0; i < ADV728X_VARIANT_COUNT; i++) {
		if (0 == strcmp(model, adv728x_variants[i].model_str)) {
			state->variant = i;
			dev_dbg(&client->dev, "%s found in _DSD",
				adv728x_variants[i].model_str);
			break;
		}
	}

	if (i == ADV728X_VARIANT_COUNT) {
		dev_err(&client->dev, "model string in _DSD is not recognized");
		return -EINVAL;
	}

	if (device_property_read_u32
	    (&client->dev, "power-up-sequence", &state->power_seq)) {
#if (ADV728X_WO_ACPI == 1)
		state->power_seq = ADV728X_POWERUPSEQ_OPTIMAL;
#else
		dev_err(&client->dev,
			"can't find power-up-sequence in _DSD (ACPI)");
		return -ENODEV;
#endif
	}

	/* get intrq_pin from acpi tables */
	gpiodescriptor = devm_gpiod_get(&client->dev, "intrq", GPIOD_ASIS);
	if (IS_ERR(gpiodescriptor)) {
#if (ADV728X_WO_ACPI == 1)
		state->irq = ADV728X_GPIO_BASE + ADV728X_GPIO_INT;
#else
		state->irq = -1;
		dev_err(&client->dev, "intrq-gpio is not defined");
		return PTR_ERR(gpiodescriptor);
#endif
	} else {
		state->irq = desc_to_gpio(gpiodescriptor);
		dev_dbg(&client->dev, "irq:%d", state->irq);
	}

	if (state->power_seq == ADV728X_POWERUPSEQ_OPTIMAL) {
		/* get powerdown_pin from acpi tables */
		gpiodescriptor = devm_gpiod_get(&client->dev, "pdn", GPIOD_ASIS);
		if (IS_ERR(gpiodescriptor)) {
#if (ADV728X_WO_ACPI == 1)
			state->powerdown_pin = ADV728X_GPIO_BASE + ADV728X_GPIO_PDN;
#else
			state->powerdown_pin = -1;
			dev_err(&client->dev, "pdn-gpio is not defined");
			return PTR_ERR(gpiodescriptor);
#endif
		} else {
			state->powerdown_pin = desc_to_gpio(gpiodescriptor);
			dev_dbg(&client->dev, "pdn:%d", state->powerdown_pin);
		}
		/* get reset_pin from acpi tables */
		gpiodescriptor = devm_gpiod_get(&client->dev, "reset", GPIOD_ASIS);
		if (IS_ERR(gpiodescriptor)) {
#if (ADV728X_WO_ACPI == 1)
			state->reset_pin = ADV728X_GPIO_BASE + ADV728X_GPIO_RST;
#else
			state->reset_pin = -1;
			dev_err(&client->dev, "reset-gpio is not defined");
			return PTR_ERR(gpiodescriptor);
#endif
		} else {
			state->reset_pin = desc_to_gpio(gpiodescriptor);
			dev_dbg(&client->dev, "reset:%d", state->reset_pin);
		}
	} else if (state->power_seq != ADV728X_POWERUPSEQ_SIMPLIFIED) {
		/* ADV728X_POWERUPSEQ_SIMPLIFIED is performed after chip 
		   reset. anything other is not recognized */
		return -EINVAL;
	} else {
		state->powerdown_pin = -1;
		state->reset_pin = -1;
	}

	state->client = client;
	state->field = V4L2_FIELD_ALTERNATE;

	state->csi_client = i2c_new_dummy(client->adapter,
					  ADV728X_DEFAULT_CSI_I2C_ADDR);
	if (!state->csi_client) {
		dev_err(&client->dev, "can't register new CSI2 MAP");
		return -ENOMEM;
	}

	if (adv728x_variants[state->variant].have_i2p) {
		state->vpp_client = i2c_new_dummy(client->adapter,
						  ADV728X_DEFAULT_VPP_I2C_ADDR);
		if (!state->vpp_client) {
			dev_err(&client->dev, "can't register new VPP MAP");
			ret = -ENOMEM;
			goto err_unregister_csi_client;
		}
	}

	mutex_init(&state->mutex);

	/* initialize default inputs NTSC_M differential on AIN1 and AIN2 */
	state->autodetect = false;
	state->expected_tv_std = V4L2_STD_NTSC;
	state->powered = false;

	sd = &state->sd;
	v4l2_i2c_subdev_init(sd, client, &adv728x_ops);
	sd->flags = V4L2_SUBDEV_FL_HAS_DEVNODE;

	ret = adv728x_init_controls(state);
	if (ret) {
		dev_err(&client->dev, "can't init controls");
		goto err_mutex_free;
	}

	state->pad.flags = MEDIA_PAD_FL_SOURCE;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	sd->entity.type |= MEDIA_ENT_T_V4L2_SUBDEV_DECODER;
#else
	sd->entity.function = MEDIA_ENT_F_ATV_DECODER;
#endif	
	sd->entity.flags = MEDIA_ENT_FL_DEFAULT;
	memset(&sd->name, 0, sizeof(sd->name));
	strncpy((char *)&sd->name,
		adv728x_variants[state->variant].media_entity_name,
		strlen(adv728x_variants[state->variant].media_entity_name));
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	ret = media_entity_init(&sd->entity, 1, &state->pad, 0);
#else
	ret = media_entity_pads_init(&sd->entity, 1, &state->pad);
#endif	
	if (ret) {
		dev_err(&client->dev, "can't initialize media entity");
		goto err_free_ctrl;
	}

	ret = __adv728x_init_device(state);
	if (ret) {
		dev_err(&client->dev, "can't initialize device");
		goto err_media_entity_cleanup;
	}

	ret = v4l2_async_register_subdev(sd);
	if (ret) {
		dev_err(&client->dev, "can't register register subdev");
		goto err_media_entity_cleanup;
	}
	/* get sync depth thresholds from _DSD */
	if (device_property_read_u32(&client->dev,
				     "sync-depth-threshold-bad",
				     &state->syncdepth_threshold_bad)) {
		dev_info(&client->dev,
			 "failed to get sync-depth-threshold-bad");
		state->syncdepth_threshold_bad = -1;
	}

	if (device_property_read_u32(&client->dev,
				     "sync-depth-threshold-good",
				     &state->syncdepth_threshold_good)) {
		dev_info(&client->dev,
			 "failed to get sync-depth-threshold-good");
		state->syncdepth_threshold_good = -1;
	}

	/* range check sync_depths */
	if ((state->syncdepth_threshold_bad < 0)
	    || (state->syncdepth_threshold_bad > 0x3F)
	    || (state->syncdepth_threshold_good < 0)
	    || (state->syncdepth_threshold_good > 0x3F)) {
		/* uninitialized */
		state->syncdepth_threshold_bad = -1;
		state->syncdepth_threshold_good = -1;
		dev_info(&client->dev, "sync depth thresholds out of range");
		dev_info(&client->dev, "sync depth check is disabled");
	} else {
		dev_dbg(&client->dev, "sync-depth-threshold-bad:%d",
			state->syncdepth_threshold_bad);
		dev_dbg(&client->dev, "sync-depth-threshold-good:%d",
			state->syncdepth_threshold_good);
	}

	/* initialize state */
	state->cached_status = -1;

	return 0;

 err_media_entity_cleanup:
	media_entity_cleanup(&sd->entity);
 err_free_ctrl:
	adv728x_exit_controls(state);
 err_mutex_free:
	mutex_destroy(&state->mutex);
	if (adv728x_variants[state->variant].have_i2p)
		i2c_unregister_device(state->vpp_client);
 err_unregister_csi_client:
	i2c_unregister_device(state->csi_client);
	return ret;
}

static int adv728x_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct adv728x_state *state = to_state(sd);

	v4l2_async_unregister_subdev(sd);

	media_entity_cleanup(&sd->entity);
	adv728x_exit_controls(state);

	i2c_unregister_device(state->csi_client);

	mutex_destroy(&state->mutex);
	return 0;
}

static void adv728x_shutdown(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct adv728x_state *state = to_state(sd);

	if (state->power_seq == ADV728X_POWERUPSEQ_OPTIMAL) {
		gpio_set_value(state->reset_pin, 0);
		gpio_set_value(state->powerdown_pin, 0);
	}
	return;
}

static const struct i2c_device_id adv728x_id[] = {
	{"adv728x", (kernel_ulong_t) NULL},
	{},
};

MODULE_DEVICE_TABLE(i2c, adv728x_id);

#ifdef CONFIG_PM_SLEEP
static int adv728x_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct adv728x_state *state = to_state(sd);

	/* set GPIOs to 0 */
	if (state->power_seq == ADV728X_POWERUPSEQ_OPTIMAL) {
		gpio_set_value(state->reset_pin, 0);
		gpio_set_value(state->powerdown_pin, 0);
	}

	return adv728x_set_power(state, false);
}

static int adv728x_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct adv728x_state *state = to_state(sd);
	int ret;

	ret = __adv728x_init_device(state);
	if (ret < 0)
		return ret;

	ret = adv728x_set_power(state, state->powered);
	if (ret)
		return ret;

	return 0;
}

static SIMPLE_DEV_PM_OPS(adv728x_pm_ops, adv728x_suspend, adv728x_resume);
#define ADV728X_PM_OPS (&adv728x_pm_ops)

#else
#define ADV728X_PM_OPS NULL
#endif

static struct i2c_driver adv728x_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = KBUILD_MODNAME,
		   .pm = ADV728X_PM_OPS,
		   },
	.probe = adv728x_probe,
	.remove = adv728x_remove,
	.shutdown = adv728x_shutdown,
	.id_table = adv728x_id,
};

module_i2c_driver(adv728x_driver);

MODULE_DESCRIPTION("Analog Devices ADV728x video decoder driver");
MODULE_AUTHOR("Mentor Graphics Corp.");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(ADV728X_MODULE_VERSION);
