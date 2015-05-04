/*
 * ov680_flash.h
 *
 * Copyright (c) 2012-2013 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * PROPRIETARY/CONFIDENTIAL
 *
 * Use is subject to license terms.
 */

#include "msm_sensor.h"

#ifndef OV680_FLASH_H
#define OV680_FLASH_H

// TODO: Use DTS
#define QUAD_MODE_GPIO 56

#define NUM_FLASH_DEVICES 2

/**
 * Describes the physical device, device channel, and flash current used by each sensor.
 */
struct ov680_flash_led_desc_t {
    uint8_t device_number;
    uint8_t device_channel;
    uint32_t flash_current;
};

struct ov680_flash_ctrl_t;

struct ov680_flash_func_tbl_t {
    int (*flash_init)(struct ov680_flash_ctrl_t *);
    int (*flash_release)(struct ov680_flash_ctrl_t *);
    int (*flash_enable)(struct ov680_flash_ctrl_t *, uint32_t);
    int (*flash_get_max_current)(struct ov680_irled_max_current_req *);
};

struct ov680_flash_ctrl_t {
    struct msm_camera_i2c_client sensor_i2c_client[NUM_FLASH_DEVICES];
    struct ov680_flash_led_desc_t led_desc[4];
    struct ov680_flash_func_tbl_t func_tbl;
};

int ov680_flash_probe(struct ov680_flash_ctrl_t *fctrl, struct i2c_client *);
int ov680_flash_init(struct ov680_flash_ctrl_t *);
int ov680_flash_release(struct ov680_flash_ctrl_t *);
int ov680_flash_enable(struct ov680_flash_ctrl_t *, uint32_t);
int ov680_flash_get_max_current(struct ov680_flash_ctrl_t *, struct ov680_irled_max_current_req *);
#endif //OV680_FLASH_H
