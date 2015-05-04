/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/debugfs.h>
#include <linux/module.h>
#include "msm_sd.h"
#include "msm_actuator.h"
#include "msm_cci.h"
#include "msm_actuator_debugfs.h"

/* This is really only for the BU63164 */
#define _OP_FX_Cmnd_0	0xF0

static struct msm_actuator_ctrl_t *a_ctrl;
static u16 vcm_value = 0;
static u32 cmd = 0;

static int msm_actuator_debugfs_cmd_set(void *data, u64 val)
{
	int rc;
	uint8_t vcm_write_array[4] = {0x90,0x00,0x00,0x00};

	vcm_write_array[2] = vcm_value>>8 & 0xff;
	vcm_write_array[3] = vcm_value & 0x00ff;

	a_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	a_ctrl->i2c_client.cci_client->sid = 0x0E;

	//rc = msm_sensor_cci_i2c_util(&a_ctrl->i2c_client, MSM_CCI_INIT);
	rc = msm_camera_cci_i2c_write_seq(&a_ctrl->i2c_client, _OP_FX_Cmnd_0, vcm_write_array, 4);
	if(rc < 0) {
		pr_err("VCM write failed\n");
		return 0;
	}
	//rc = msm_sensor_cci_i2c_util(&a_ctrl->i2c_client, MSM_CCI_RELEASE);

	return 0;
}

static int msm_actuator_debugfs_cmd_get(void *data, u64 * val)
{
	printk("%s \n", __func__);
	*val = *(u32 *) data;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(msm_actuator_debugfs_cmd_fops, msm_actuator_debugfs_cmd_get,
                        msm_actuator_debugfs_cmd_set, "%llu\n");

struct dentry *mydebugfs_create_cmd32(const char *name, mode_t mode,
                                      struct dentry *parent, u32 * value)
{
	/* if there are no write bits set, make read only */
	if (!(mode & S_IWUGO))
		return debugfs_create_file(name, mode, parent, value,
								  &msm_actuator_debugfs_cmd_fops);
	/* if there are no read bits set, make write only */
	if (!(mode & S_IRUGO))
		return debugfs_create_file(name, mode, parent, value,
								   &msm_actuator_debugfs_cmd_fops);

	return debugfs_create_file(name, mode, parent, value,
							   &msm_actuator_debugfs_cmd_fops);
}


void msm_actuator_debugfs_init(struct msm_actuator_ctrl_t *ctrl)
{
	struct dentry *ois_dir;
	static uint8_t init = 0;

	a_ctrl = ctrl;
	if(!init) {
		ois_dir = debugfs_create_dir("ois_ctrl", NULL);

		if (!ois_dir) {
			printk("Unable to create debugfs directory \n");
			return;
		} else {
			mydebugfs_create_cmd32("cmd", 0666, ois_dir, &cmd);
			debugfs_create_x16("vcm_value", 0666, ois_dir, &vcm_value);
		}
		init = 1;
	}
}
