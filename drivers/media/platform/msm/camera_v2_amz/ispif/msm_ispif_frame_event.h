/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
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

#ifndef MSM_ISPIF_FRAME_EVENT_H
#define MSM_ISPIF_FRAME_EVENT_H

#include <linux/clk.h>
#include <linux/io.h>
#include <media/v4l2-subdev.h>
#include <media/msmb_ispif.h>
#include <media/msmb_camera.h>
#include "msm_sd.h"

struct ispif_device;

enum msm_ispif_frame_evt {
	FRAME_EVENT_SOF,
	FRAME_EVENT_EOF,
};

struct ispif_frame_event {
	enum msm_ispif_vfe_intf vfe_idx;
	enum msm_ispif_intftype interface;
	enum msm_ispif_frame_evt event;
	struct timespec timestamp;
};

#define MAX_QUEUE_SIZE 6

struct frame_event_mgr{
	struct work_struct async_work;
	struct mutex sync_mutex; /* mutex for worker function serialization */
	struct ispif_frame_event queue[MAX_QUEUE_SIZE];
	uint8_t fe_writer;
	uint8_t fe_reader;
	int qf_count;
	atomic_t outstanding;
};


#define VIDIOC_SENSOR_FRAME_EVENT_HOOK\
        _IOW('V', ISPIF_FRAME_EVENT_BASE, struct ispif_frame_event)

void frame_event_workqueue_create(void);

void frame_event_workqueue_destroy(void);

void frame_event_manager_reset(struct frame_event_mgr *fem);

void dispatch_async_frame_event(struct ispif_device *ispif, enum msm_ispif_vfe_intf vfe_idx,
	enum msm_ispif_intftype interface, enum msm_ispif_frame_evt event);


void async_frame_event_do_work(struct work_struct *data);

#endif
