/* Copyright (c) 2011, Lab126. All rights reserved.
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

// YES, these may very well be best suited to the buckshot config file,
// however I'd like to keep them here for now.

#ifndef OV680_CONFIG_H

/**
 * Disables ISP config. Should improve performance.
 */
//#define OV680_NO_ISP_CONFIG

/**
 * Turning off AE during frame switch improve stability.
 * Symptom is split frame.
 */
//#define OV680_FRAME_SW_WORKAROUND

/**
 * Skip setting selections when setting is already written.
 */
//#define OV680_AVOID_WRITING_SAME_SETTING

/**
 * Improve sensor switching time by adjusting frame rate to
 * 120fps before a sensor switch. After completion, restore
 * frame rate.
 */
//#define OV680_USE_FAST_SWITCH

/**
 * GPIO used for interrupts/command complete message.
 */
#define OV680_CMD_RDY_GPIO 50

/**
 * When enabled, the standby mode of the sensor/isp will be enabled.
 */
#define OV680_SUPPORT_STANDBY

/**
 * GPIO that engages s/w standby.
 */

/*TODO: this must be determined from dts */
#define OV680_PWDN_GPIO 25

/**
 * GPIO for MCLK.
 */

/*TODO: this must be determined from dts */
#define OV680_MCLK_GPIO 16

#endif
