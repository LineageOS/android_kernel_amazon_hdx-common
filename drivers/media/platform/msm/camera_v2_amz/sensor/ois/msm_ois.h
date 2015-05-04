/* Copyright (c) 2009-2013, The Linux Foundation. All rights reserved.
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
#ifndef MSM_OIS_H
#define MSM_OIS_H

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <mach/camera2.h>
#include <media/v4l2-subdev.h>
#include <media/msmb_camera.h>
#include "msm_camera_i2c.h"

#define DEFINE_MSM_MUTEX(mutexname) \
	static struct mutex mutexname = __MUTEX_INITIALIZER(mutexname)

struct msm_ois_ctrl_t;

struct msm_ois_fn_t {
	int32_t (*ois_power_down) (struct msm_ois_ctrl_t *);
	int32_t (*ois_power_up) (struct msm_ois_ctrl_t *);
	int32_t (*ois_enable) (struct msm_ois_ctrl_t *, uint8_t enable);
};

struct msm_ois_ctrl_t {
	struct msm_camera_i2c_client i2c_client;
	struct msm_sd_subdev msm_sd;
	struct platform_device *pdev;
	struct mutex *msm_ois_mutex;
	struct msm_ois_fn_t *func_tbl;
};

int32_t msm_ois_probe(struct platform_device *pdev, void *data);

#endif
