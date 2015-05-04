/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
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
#ifndef OV680_PROF_LUT_H
#define OV680_PROF_LUT_H

#define OV680_PROF_ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))

struct ov680_tplms_lut {
    const uint32_t * lut;
    const uint32_t   length;
    const uint32_t   default_tplms; // when no lookup exists...
};

//TODO: Add bounds checking to these arrays - don't want to get in to dirty business! :)
static const uint32_t ov680_tplms_lut_fw [] = {
    TID_CAM_OV680_FWI_FIRMWARE_LOAD_START,
    TID_CAM_OV680_FWI_FIRMWARE_STANDBY_START,
    TID_CAM_OV680_FWI_FIRMWARE_RESUME_START,
    TID_CAM_OV680_FWI_FIRMWARE_RESTORE_START,
};

static const uint32_t ov680_tplms_lut_fps [] = {
    TID_CAM_OV680_FWI_FRAME_RATE_15_START,
    TID_CAM_OV680_FWI_FRAME_RATE_30_START,
    TID_CAM_OV680_FWI_FRAME_RATE_60_START,
    TID_CAM_OV680_FWI_FRAME_RATE_120_START,
    TID_CAM_OV680_FWI_FRAME_RATE_240_START,
    TID_CAM_OV680_FWI_FRAME_RATE_8_START,
    TID_CAM_OV680_FWI_FRAME_RATE_10_START,
    TID_CAM_OV680_FWI_FRAME_RATE_12_START,
};

static const uint32_t ov680_tplms_lut_autoexp [] = {
    TID_CAM_OV680_FWI_AUTO_EXPOSURE_OFF_START,
    TID_CAM_OV680_FWI_AUTO_EXPOSURE_ON_START,
};

static const uint32_t ov680_tplms_lut_flash []= {
    TID_CAM_OV680_FWI_FLASH_OFF_START,
    TID_CAM_OV680_FWI_FLASH_ON_START,
};

static const uint32_t ov680_tplms_lut_mode [] = {
    TID_CAM_OV680_FWI_FRAME_MODE_FULL_START,
    TID_CAM_OV680_FWI_FRAME_MODE_BINNED_2X2_START,
    TID_CAM_OV680_FWI_FRAME_MODE_BINNED_4X4_START,
};

static const uint32_t ov680_tplms_lut_exposure_conf [] = {
    TID_CAM_OV680_FWI_EXPOSURE_NONE_START,
    TID_CAM_OV680_FWI_EXPOSURE_SET_START,
};

static const uint32_t ov680_tplms_lut_gain_conf [] = {
    TID_CAM_OV680_FWI_GAIN_NONE_START,
    TID_CAM_OV680_FWI_GAIN_SET_START,
};

static const uint32_t ov680_tplms_lut_isp [] = {
    TID_CAM_OV680_FWI_ISP_OFF_START,
    TID_CAM_OV680_FWI_ISP_ON_START,
};

static const struct ov680_tplms_lut ov680_tplms_lut [] = {
    /*"FIRMWARE",*/
        {
            ov680_tplms_lut_fw,
            OV680_PROF_ARRAY_SIZE(ov680_tplms_lut_fw),
            0
        },
    /*"FRAME_RATE",*/
        {
            ov680_tplms_lut_fps,
            OV680_PROF_ARRAY_SIZE(ov680_tplms_lut_fps),
            0
        },
    /*"ISP_2A",*/
        {
            ov680_tplms_lut_autoexp,
            OV680_PROF_ARRAY_SIZE(ov680_tplms_lut_autoexp),
            0
        },
    /*"ISP_2A_TARGET",*/
        {
            NULL,
            0,
            TID_CAM_OV680_FWI_AUTO_EXPOSURE_TARGET_START
        },
    /*"ISP_2A_ROI_CONFIG",*/
        {
            NULL,
            0,
            TID_CAM_OV680_FWI_AUTO_EXPOSURE_ROI_START
        },
    /*"SENSOR_SEL",*/
        {
            NULL,
            0,
            TID_CAM_OV680_FWI_SENSOR_SW_SEL_START
        },
    /*"IDLE",*/
        {
            NULL,
            0,
            TID_CAM_OV680_FWI_IDLE_START
        },
    /*"FLASH",*/
        {
            NULL,
            0,
            0
        },
    /*"FLASH_CONFIG",*/
        {
            ov680_tplms_lut_flash,
            OV680_PROF_ARRAY_SIZE(ov680_tplms_lut_flash),
            0
        },
    /*"FRAME_MODE",*/
        {
            ov680_tplms_lut_mode,
            OV680_PROF_ARRAY_SIZE(ov680_tplms_lut_mode),
            0
        },
    /*"ISP_CONFIG",*/
        {
            ov680_tplms_lut_isp,
            ARRAY_SIZE(ov680_tplms_lut_isp),
            0
        },
    /*"LENC_CONFIG"*/
        {
            NULL,
            0,
            0
        },
    /*"DPC_CONFIG",*/
        {
            NULL,
            0,
            0
        },
    /*"EXPOSURE_CONFIG",*/
        {
            ov680_tplms_lut_exposure_conf,
            ARRAY_SIZE(ov680_tplms_lut_exposure_conf),
            0
        },
    /*"GAIN_CONFIG",*/
        {
            ov680_tplms_lut_gain_conf,
            ARRAY_SIZE(ov680_tplms_lut_gain_conf),
            0
        },
};

#endif
