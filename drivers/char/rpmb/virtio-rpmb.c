/*
 * Virtio RPMB FrontEnd Driver
 * Virtio RPMB frontend driver for trusty usage
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright (c) 2018 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Contact Information: weideng <wei.a.deng@intel.com>
 *
 * BSD LICENSE
 *
 * Copyright (c) 2017 Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of Intel Corporation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * weideng <wei.a.deng@intel.com>
 *
 * Create virtio rpmb frontend driver. This driver will work with RPMB
 * VBS-U together to provide one communication channel between UOS and
 * SOS. The message from RPMB daemon in Android will be transferred over
 * the channel and finally arrived RPMB physical driver on SOS kernel.
 *
 */

#include <linux/err.h>
#include <linux/scatterlist.h>
#include <linux/spinlock.h>
#include <linux/virtio.h>
#include <linux/module.h>
#include <linux/virtio_ids.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/virtio_config.h>
#include <linux/uaccess.h>
#include <linux/rpmb.h>

#ifndef VIRTIO_ID_RPMB
#define	VIRTIO_ID_RPMB		0xFFFF
#endif
#define INFO(fmt, ...) pr_info("[vrpmb-FE] %s(): "fmt, __func__, ## __VA_ARGS__)
#define ERR(fmt, ...) pr_err("[vrpmb-FE] %s(): "fmt, __func__, ## __VA_ARGS__)
#define DBG(fmt, ...) pr_debug("[vrpmb-FE] %s(): "fmt, __func__, ## __VA_ARGS__)

#define SEQ_CMD_MAX	3	/*support up to 3 cmds*/

struct virtio_rpmb_info {
	struct virtqueue *vq;
	struct completion have_data;
	bool busy;
	unsigned int data_avail;
	struct miscdevice misc_dev;
};

struct virtio_rpmb_ioctl_cmd {
	unsigned int cmd;	/* ioctl cmd*/
	int result;		/* result for ioctl cmd*/
};

struct virtio_rpmb_ioc_seq_cmd {
	struct virtio_rpmb_ioctl_cmd ioc;
	u64 num_of_cmds;	/* num of  seq cmds*/
};

struct virtio_rpmb_ioc_seq_data {
	struct virtio_rpmb_ioc_seq_cmd seq_cmd;
	struct rpmb_cmd cmds[SEQ_CMD_MAX + 1];
};

static void virtio_rpmb_recv_done(struct virtqueue *vq)
{
	struct virtio_rpmb_info *vi = NULL;

	if (!vq) {
		ERR("error, found invalid args.\n");
		return;
	}

	vi = vq->vdev->priv;
	if (!vq) {
		ERR("error, no found vi data.\n");
		return;
	}

	if (!virtqueue_get_buf(vi->vq, &vi->data_avail))
		INFO("Not found buf in vq.\n");

	complete(&vi->have_data);
}

static void virtio_rpmb_register_buffer(struct virtio_rpmb_info *vi,
					void *buf, size_t size)
{
	struct scatterlist sg;

	sg_init_one(&sg, buf, size);

	/* There should always be room for one buffer. */
	virtqueue_add_inbuf(vi->vq, &sg, 1, buf, GFP_KERNEL);

	virtqueue_kick(vi->vq);
}

/*
 * get address of seq cmds.
 */
static struct rpmb_cmd *
virtio_rpmb_get_seq_cmds(void *pdata)
{
	struct virtio_rpmb_ioc_seq_data *seq_data;

	if (!pdata) {
		ERR("error, invalid args NULL\n");
		return NULL;
	}

	seq_data = (struct virtio_rpmb_ioc_seq_data *)pdata;

	return (struct rpmb_cmd *)&seq_data->cmds[0];
}

/*
 * get address of seq specail frames.
 * index:	the index of cmds.
 */
static struct rpmb_frame *
virtio_rpmb_get_seq_frames(void *pdata, u32 index)
{
	struct virtio_rpmb_ioc_seq_data *seq_data;
	struct rpmb_frame *frames;
	struct rpmb_cmd *cmds, *cmd;
	u64 ncmds;
	u32 offset = 0;
	u32 num = 0;

	if (!pdata || index > SEQ_CMD_MAX) {
		ERR("error, invalid args\n");
		return NULL;
	}

	seq_data = (struct virtio_rpmb_ioc_seq_data *)pdata;

	/* get number of cmds*/
	ncmds = seq_data->seq_cmd.num_of_cmds;
	if (ncmds > SEQ_CMD_MAX) {
		ERR("error, ncmds(%llu) > max(%u)\n", ncmds, SEQ_CMD_MAX);
		return NULL;
	}

	/* check index.*/
	if (index > ncmds) {
		ERR("error, index(%u) is larger than ncmds(%u)\n", index, ncmds);
		return NULL;
	}

	/* get start address of cmds.*/
	cmds = virtio_rpmb_get_seq_cmds(pdata);
	if (!cmds) {
		ERR("fail to get seq cmds.\n");
		return NULL;
	}

	/* get start address of frames.*/
	frames = (struct rpmb_frame *)&seq_data->cmds[ncmds + 1];
	if (!frames) {
		ERR("error, found invalid frames ptr!\n");
		return NULL;
	}

	/* get offset of frames.*/
	for (num = 0; num < index; num++) {
		cmd = &cmds[num];
		if (!cmd) {
			ERR("error, found invalid cmd ptr!\n");
			return NULL;
		}
		offset += cmd->nframes;
	}

	return (struct rpmb_frame *)&frames[offset];
}

/*
 * Address space is different between
 * SOS VBS-U and UOS kernel.
 * Update frames_ptr to current space.
 */
static int virtio_rpmb_map_seq_frames(void *pdata)
{
	struct virtio_rpmb_ioc_seq_cmd *seq_cmd = NULL;
	struct rpmb_cmd *cmds, *cmd;
	u64 index;

	if (!pdata) {
		ERR("error, invalid args NULL\n");
		return -EINVAL;
	}

	seq_cmd = (struct virtio_rpmb_ioc_seq_cmd *)pdata;
	if (seq_cmd->num_of_cmds > SEQ_CMD_MAX) {
		ERR("invalid num of cmds.\n");
		return -EINVAL;
	}

	/* get start address of cmds.*/
	cmds = virtio_rpmb_get_seq_cmds(pdata);
	if (!cmds) {
		ERR("fail to get seq cmds\n");
		return -EINVAL;
	}

	/*set address of frames to address space.*/
	for (index = 0; index < seq_cmd->num_of_cmds; index++) {
		cmd = &cmds[index];
		if (!cmd) {
			ERR("error, found invalid cmd ptr!\n");
			return -EINVAL;
		}
		/* set frames address.*/
		cmd->frames = virtio_rpmb_get_seq_frames(pdata, index);
		if (!cmd->frames) {
			ERR("fail to get frames[%llu] ptr!\n", index);
			return -EINVAL;
		}
	}

	return 0;
}

static void *virtio_rpmb_get_seq_data(void __user *args, size_t *size)
{
	struct rpmb_ioc_seq_cmd __user *ptr = (struct rpmb_ioc_seq_cmd *)args;
	struct virtio_rpmb_ioc_seq_cmd *seq_cmd = NULL;
	struct rpmb_ioc_cmd __user *ucmds, *ucmd;
	struct rpmb_cmd *cmds, *cmd;
	u64 uframes_ptr;
	void *pdata = NULL;
	u32 total_nframes = 0;
	u64 ncmds;
	u64 index_ucmds = 0;
	u32 nframes = 0;

	if (!size || !args) {
		ERR("error, invalid args!\n");
		return NULL;
	}

	/* get number of cmds.*/
	if (copy_from_user(&ncmds, &ptr->num_of_cmds, sizeof(ncmds))) {
		DBG("get num of cmds from user space error.\n");
		return NULL;
	}

	if (ncmds > SEQ_CMD_MAX) {
		ERR("supporting up to 3 packets (%llu)\n", ncmds);
		return NULL;
	}

	/* get ucmds list*/
	ucmds = (struct rpmb_ioc_cmd __user *)&ptr->cmds[0];
	if (!ucmds) {
		ERR("get ucmds failed!\n");
		return NULL;
	}

	/* get total of frames*/
	for (index_ucmds = 0; index_ucmds < ncmds; index_ucmds++) {
		ucmd = &ucmds[index_ucmds];
		if (!ucmd) {
			ERR("error, get ucmd[%llu] is NULL!\n", index_ucmds);
			return NULL;
		}

		if (get_user(nframes, &ucmd->nframes)) {
			DBG("fail to get total_frames from user space.\n");
			return NULL;
		}
		total_nframes += nframes;
	}

	/* total size = seq_cmd + m * ucmds + n * frames */
	*size = sizeof(struct virtio_rpmb_ioc_seq_cmd) +
			ncmds * sizeof(struct rpmb_cmd) +
			total_nframes * sizeof(struct rpmb_frame);

	/* alloc all of buffer.*/
	pdata = kzalloc(*size, GFP_KERNEL);
	if (!pdata) {
		ERR("fail to malloc buffer.\n");
		return NULL;
	}

	/* set num_of_cmds*/
	seq_cmd = (struct virtio_rpmb_ioc_seq_cmd *)pdata;
	seq_cmd->num_of_cmds = ncmds;

	/* get cmds list*/
	cmds = virtio_rpmb_get_seq_cmds(pdata);
	if (!cmds) {
		ERR("fail to get seq cmds\n");
		goto out;
	}

	/* set all of rpmb cmds.*/
	for (index_ucmds = 0; index_ucmds < ncmds; index_ucmds++) {
		/* get ucmd*/
		ucmd = &ucmds[index_ucmds];
		if (!ucmd) {
			ERR("error, get ucmd[%llu] is NULL!\n", index_ucmds);
			return NULL;
		}
		/* get cmd*/
		cmd = &cmds[index_ucmds];
		if (!cmd) {
			ERR("error, get cmd[%llu] is NULL!\n", index_ucmds);
			return NULL;
		}
		/* get flags*/
		if (get_user(cmd->flags, &ucmd->flags)) {
			DBG("fail to get total_frames from user space.\n");
			goto out;
		}

		/* get nframes.*/
		if (get_user(cmd->nframes, &ucmd->nframes)) {
			DBG("fail to get total_frames from user space.\n");
			goto out;
		}

		/* get frames_ptr.*/
		cmd->frames = virtio_rpmb_get_seq_frames(pdata, index_ucmds);
		if (!cmd->frames) {
			ERR("fail to get seq frames[%llu]\n", index_ucmds);
			goto out;
		}

		/* get frames_ptr of ucmd*/
		if (get_user(uframes_ptr, &ucmd->frames_ptr)) {
			DBG("fail to get frames_ptr from user space.\n");
			goto out;
		}

		if (!u64_to_user_ptr(uframes_ptr)) {
			ERR("found uframes_ptr is NULL.\n");
			goto out;
		}

		/* get data from frames_ptr*/
		if (copy_from_user((void *)cmd->frames,
				u64_to_user_ptr(uframes_ptr),
				cmd->nframes * sizeof(struct rpmb_frame))) {
			DBG("error, fail to get frame data.\n");
			goto out;
		}
	}

	return pdata;

out:
	kfree(pdata);

	return NULL;
}

static int virtio_rpmb_put_seq_data(void __user *args, void *pdata)
{
	struct rpmb_ioc_seq_cmd __user *uargs = (struct rpmb_ioc_seq_cmd *)args;
	struct virtio_rpmb_ioc_seq_cmd *seq_cmd = NULL;
	struct rpmb_ioc_cmd __user *ucmds, *ucmd;
	struct rpmb_cmd *cmds, *cmd;
	u64 uframes_ptr;
	u64 index;

	if (!args || !pdata) {
		ERR("error, invalid args.\n");
		return -EFAULT;
	}

	seq_cmd = (struct virtio_rpmb_ioc_seq_cmd *)pdata;

	/* get cmd list both of cmds and ucmds.*/
	ucmds = (struct rpmb_ioc_cmd __user *)&uargs->cmds[0];
	if (!ucmds) {
		ERR("fail to get ucmds!\n");
		return -EFAULT;
	}
	cmds = virtio_rpmb_get_seq_cmds(pdata);
	if (!cmds) {
		ERR("fail to get seq cmds\n");
		return -EFAULT;
	}

	/* copy all of data to user space.*/
	for (index = 0; index < seq_cmd->num_of_cmds; index++) {
		cmd = &cmds[index];
		if (!cmd) {
			ERR("invaild cmd!\n");
			return -EFAULT;
		}
		ucmd = &ucmds[index];
		if (!ucmd) {
			ERR("invaild ucmd!\n");
			return -EFAULT;
		}
		/*get frames_ptr of user space*/
		if (get_user(uframes_ptr, &ucmd->frames_ptr)) {
			DBG("fail to get frames_ptr from user space.\n");
			return -EFAULT;
		}

		if (!u64_to_user_ptr(uframes_ptr)) {
			ERR("found uframes_ptr is NULL.\n");
			return -EFAULT;
		}

		if (!cmd->frames) {
			ERR("found cmd frames_ptr is NULL.\n");
			return -EFAULT;
		}

		if (copy_to_user(u64_to_user_ptr(uframes_ptr),
				cmd->frames,
				cmd->nframes * sizeof(struct rpmb_frame))) {
			DBG("copy data to user error!!!");
			return -EFAULT;
		}
	}
	return 0;
}

static int virtio_rpmb_seq_cmd_handler(struct virtio_rpmb_info *vi,
					unsigned int cmd, void __user *args)
{
	struct virtio_rpmb_ioctl_cmd *ioc = NULL;
	void *pdata = NULL;
	size_t size = 0;
	int ret;

	if (!vi || !args) {
		ERR("error, invalid args.\n");
		return -EINVAL;
	}


	if (vi->busy) {
		ERR("Busy , do nothing!!!");
		return -EBUSY;
	}

	/*get data from user sapce*/
	pdata = virtio_rpmb_get_seq_data(args, &size);
	if (!pdata || !size) {
		ERR("get data from user space error!\n");
		ret = -EFAULT;
		goto out;
	}

	/*set cmd for ioctl.*/
	ioc = (struct virtio_rpmb_ioctl_cmd *)pdata;
	ioc->cmd = cmd;

	vi->busy = true;

	/* sent data to vq.*/
	virtio_rpmb_register_buffer(vi, pdata, size);

	/* wait for completion from vq.*/
	ret = wait_for_completion_killable(&vi->have_data);
	if (ret < 0) {
		ERR("err=%d, after completion done.\n", ret);
		goto out;
	}

	/*check result from vq*/
	if (ioc->result < 0) {
		ERR("error, process ioctl cmd(0x%X) failed!\n", cmd);
		ret = -EIO;
		goto out;
	}

	/*update frames_ptr to curent space*/
	ret = virtio_rpmb_map_seq_frames(pdata);
	if (ret) {
		ERR("map seq frames failed!!!\n");
		goto out;
	}

	/*put all of data to user space.*/
	ret = virtio_rpmb_put_seq_data(args, pdata);
	if (ret)
		ERR("put data to user space error.\n");

out:
	kfree(pdata);

	vi->busy = false;
	return ret;
}

static long virtio_rpmb_ioctl(struct file *fp,
				unsigned int cmd, unsigned long args)
{
	long ret;
	struct virtio_rpmb_info *vi;

	if (!fp) {
		ERR("error, invalid fp!\n");
		return -EINVAL;
	}

	vi = container_of(fp->private_data, struct virtio_rpmb_info, misc_dev);
	if (!vi) {
		ERR("fail to get vi data.\n");
		return -EFAULT;
	}


	switch (cmd) {
	case RPMB_IOC_SEQ_CMD:
		ret = virtio_rpmb_seq_cmd_handler(vi, cmd, (void __user *)args);
		break;

	default:
		ERR("unsupported ioctl 0x%x.\n", cmd);
		return -ENOIOCTLCMD;
	}

	return ret;
}

static int vrpmb_dev_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int vrpmb_dev_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations vrpmb_dev_fops = {
	.owner			= THIS_MODULE,
	.open			= vrpmb_dev_open,
	.release		= vrpmb_dev_release,
	.unlocked_ioctl		= virtio_rpmb_ioctl,
};

static int vrpmb_dev_init(struct virtio_rpmb_info *vi)
{
	if (!vi) {
		ERR("error, no found vi data.\n");
		return -1;
	}

	vi->misc_dev.name = "vrpmb";
	vi->misc_dev.minor = MISC_DYNAMIC_MINOR;
	vi->misc_dev.fops = &vrpmb_dev_fops;

	if (misc_register(&vi->misc_dev)) {
		ERR("fail to register misc dev!\n");
		return -1;
	}

	return 0;
}

static int virtio_rpmb_init(struct virtio_device *vdev)
{
	int rc;
	struct virtio_rpmb_info *vi = NULL;

	if (!vdev) {
		ERR("error, found invalid vdev device.\n");
		return -1;
	}

	vi = kzalloc(sizeof(struct virtio_rpmb_info), GFP_KERNEL);
	if (!vi) {
		ERR("kzalloc vi error\n");
		return -ENOMEM;
	}

	init_completion(&vi->have_data);

	vi->busy = false;

	vdev->priv = vi;

	/* We expect a single virtqueue. */
	vi->vq = virtio_find_single_vq(vdev, virtio_rpmb_recv_done, "request");
	if (IS_ERR(vi->vq)) {
		ERR("get single vq fail!\n");
		rc = PTR_ERR(vi->vq);
		goto out;
	}

	/* create vrpmb device. */
	rc = vrpmb_dev_init(vi);
	if (rc) {
		ERR("create vrpmb device error.\n");
		goto out;
	}

	INFO("done!\n");

	return 0;

out:
	kfree(vi);
	return rc;
}

static void virtio_rpmb_remove(struct virtio_device *vdev)
{
	struct virtio_rpmb_info *vi = NULL;

	if (!vdev) {
		ERR("error, found invalid args!");
		return;
	}

	vi = vdev->priv;
	if (!vi) {
		ERR("no found vi data.\n");
		return;
	}

	misc_deregister(&vi->misc_dev);
	complete(&vi->have_data);
	vi->data_avail = 0;
	vi->busy = false;

	if (vdev->config->reset)
		vdev->config->reset(vdev);

	if (vdev->config->del_vqs)
		vdev->config->del_vqs(vdev);

	kfree(vi);
}

static int virtio_rpmb_probe(struct virtio_device *vdev)
{
	return virtio_rpmb_init(vdev);
}

#ifdef CONFIG_PM_SLEEP
static int virtio_rpmb_freeze(struct virtio_device *vdev)
{
	virtio_rpmb_remove(vdev);
	return 0;
}

static int virtio_rpmb_restore(struct virtio_device *vdev)
{
	return virtio_rpmb_init(vdev);
}
#endif

static struct virtio_device_id id_table[] = {
	{ VIRTIO_ID_RPMB, VIRTIO_DEV_ANY_ID },
	{ 0 },
};

static struct virtio_driver virtio_rpmb_driver = {
	.driver.name =	KBUILD_MODNAME,
	.driver.owner =	THIS_MODULE,
	.id_table =	id_table,
	.probe =	virtio_rpmb_probe,
	.remove =	virtio_rpmb_remove,
#ifdef CONFIG_PM_SLEEP
	.freeze =	virtio_rpmb_freeze,
	.restore =	virtio_rpmb_restore,
#endif
};

module_virtio_driver(virtio_rpmb_driver);
MODULE_DEVICE_TABLE(virtio, id_table);
MODULE_DESCRIPTION("Virtio rpmb frontend driver");
MODULE_LICENSE("GPL");
