/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include "msm_sd.h"
#include "msm_ois.h"
#include "msm_cci.h"

#define OIS_NAME "OIS"

//#define CONFIG_MSMB_CAMERA_DEBUG
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

static long msm_ois_subdev_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, void *arg);
static int32_t msm_ois_get_subdev_id(struct msm_ois_ctrl_t *fctrl, void *arg);
static int32_t msm_ois_config(struct msm_ois_ctrl_t *fctrl, void *data);

static struct v4l2_subdev_core_ops msm_ois_subdev_core_ops = {
	.ioctl = msm_ois_subdev_ioctl,
};

static struct v4l2_subdev_ops msm_ois_subdev_ops = {
	.core = &msm_ois_subdev_core_ops,
};

static const struct v4l2_subdev_internal_ops msm_ois_internal_ops;

static long msm_ois_subdev_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, void *arg)
{
	struct msm_ois_ctrl_t *fctrl = NULL;
	void __user *argp = (void __user *)arg;
	if (!sd) {
		pr_err("OIS sd NULL\n");
		return -EINVAL;
	}
	fctrl = v4l2_get_subdevdata(sd);
	if (!fctrl) {
		pr_err("OIS fctrl NULL\n");
		return -EINVAL;
	}
	switch (cmd) {
	case VIDIOC_MSM_SENSOR_GET_SUBDEV_ID:
		return msm_ois_get_subdev_id(fctrl, argp);
	case VIDIOC_MSM_OIS_DATA_CFG:
		return msm_ois_config(fctrl, argp);
	default:
		pr_err("OIS invalid cmd %d\n", cmd);
		return -ENOIOCTLCMD;
	}
}

static int32_t msm_ois_get_subdev_id(struct msm_ois_ctrl_t *fctrl,
	void *arg)
{
	uint32_t *subdev_id = (uint32_t *)arg;
	if (!subdev_id) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
		return -EINVAL;
	}
	*subdev_id = fctrl->pdev->id;
	CDBG("%s:%d subdev_id %d\n", __func__, __LINE__, *subdev_id);
	return 0;
}

static int32_t msm_ois_config(struct msm_ois_ctrl_t *fctrl,
	void *data)
{
	int rc = 0;
	struct msm_camera_ois_cfg_t *cfg = (struct msm_camera_ois_cfg_t *)data;
	mutex_lock(fctrl->msm_ois_mutex);

	switch (cfg->cfgtype) {
		case MSM_CAMERA_OIS_POWER_UP:
			CDBG("OIS power up\n");
			fctrl->func_tbl->ois_power_up(fctrl);
			break;
		case MSM_CAMERA_OIS_POWER_DOWN:
			CDBG("OIS power down\n");
			fctrl->func_tbl->ois_power_down(fctrl);
			break;
		case MSM_CAMERA_OIS_ENABLE:
			CDBG("OIS enable\n");
			fctrl->func_tbl->ois_enable(fctrl, 1);
			break;
		case MSM_CAMERA_OIS_DISABLE:
			fctrl->func_tbl->ois_enable(fctrl, 0);
			CDBG("OIS disable\n");
			break;
		default:
			rc = -EFAULT;
			break;
	}
	mutex_unlock(fctrl->msm_ois_mutex);

	CDBG("msm_ois_config: return %d\n", rc);
	return rc;
}

static int32_t msm_ois_create_v4lsubdev(struct platform_device *pdev, void *data)
{
	struct msm_ois_ctrl_t *fctrl =
		(struct msm_ois_ctrl_t *)data;

	if (!fctrl) {
		pr_err("fctrl NULL\n");
		return -EINVAL;
	}

	/* Initialize sub device */
	v4l2_subdev_init(&fctrl->msm_sd.sd, &msm_ois_subdev_ops);
	v4l2_set_subdevdata(&fctrl->msm_sd.sd, fctrl);

	fctrl->pdev = pdev;
	fctrl->msm_sd.sd.internal_ops = &msm_ois_internal_ops;
	fctrl->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(fctrl->msm_sd.sd.name, ARRAY_SIZE(fctrl->msm_sd.sd.name),
		"msm_ois");
	media_entity_init(&fctrl->msm_sd.sd.entity, 0, NULL, 0);
	fctrl->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	fctrl->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_OIS;
	msm_sd_register(&fctrl->msm_sd);

	return 0;
}

int32_t msm_ois_probe(struct platform_device *pdev, void *data)
{
	int32_t rc = 0;
	int32_t sid;
	struct device_node *of_node = pdev->dev.of_node;
	struct msm_ois_ctrl_t *fctrl = (struct msm_ois_ctrl_t *)data;
	struct msm_camera_cci_client *cci_client = NULL;

	cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!cci_client) {
		pr_err("failed no memory\n");
		return -ENOMEM;
	}

	if (!of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}

	fctrl->pdev = pdev;

	rc = of_property_read_u32(of_node, "cell-index", &pdev->id);
	if (rc < 0) {
		pr_err("Failed to get OIS cell index\n");
		return -EINVAL;
	}
	CDBG("pdev id %d\n", pdev->id);

	rc = of_property_read_u32(of_node, "qcom,cci-master",
		&cci_client->cci_i2c_master);
	if (rc < 0) {
		/* Set default master 0 */
		cci_client->cci_i2c_master = MASTER_0;
		rc = 0;
	}
	CDBG("%s qcom,cci-master %d, rc %d\n", __func__, cci_client->cci_i2c_master, rc);

	rc = of_property_read_u32(of_node, "qcom,slave-addr",
		&sid);
	if (rc < 0) {
		/* Set default master 0 */
		pr_err("Failed to get OIS slave addr\n");
		return -EINVAL;
	}
	CDBG("%s qcom,slave-addr %x, rc %d\n", __func__, sid, rc);

	fctrl->i2c_client.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	fctrl->i2c_client.cci_client = cci_client;
	cci_client->sid = sid >> 1;
	cci_client->retries = 3;
	cci_client->id_map = 0;
	cci_client->cci_subdev = msm_cci_get_subdev();

	// Do some sort of probing here
	rc = msm_ois_create_v4lsubdev(pdev, fctrl);
	return rc;
}
