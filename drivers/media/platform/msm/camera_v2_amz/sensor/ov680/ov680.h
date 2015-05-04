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
#ifndef OV680_H
#define OV680_H

#include <mach/board.h>
#include <ov680_config.h>
#include <linux/firmware.h>
#include "ov680_firmware.h"
#include "ov680_flash.h"

//#include <media/msm_camera2.h>
//#include "linux/tplms_camera.h"

extern struct platform_driver ov680_driver;
extern struct ov680_driver_priv_t ov680_props;
extern struct msm_sensor_ctrl_t ov680_s_ctrl;
extern int ov680_debug;

struct ov680_sensor_v4l2_ctrl_info_t {
	uint32_t ctrl_id;
	int16_t min;
	int16_t max;
	int16_t step;
	struct msm_camera_i2c_enum_conf_array *enum_cfg_settings;
	int (*s_v4l2_ctrl) (struct ov680_sensor_v4l2_ctrl_info_t *, int);
};

int ov680_sensor_v4l2_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl);

enum {
	OV680_DEBUG_LVL0=0,
	OV680_DEBUG_LVL1,
	OV680_DEBUG_LVL2,
	OV680_DEBUG_LVL3,
	OV680_DEBUG_LVL4,
};

#define ov680_err(x...)    pr_err ("OV680 " x)
// !!!! printing to pr_info has performance implications !!!!
#define ov680_perf(x...)   do{if(ov680_debug >= OV680_DEBUG_LVL1)pr_err("OV680 " x);}while(0);
#define ov680_info(x...)   do{if(ov680_debug >= OV680_DEBUG_LVL2)pr_err("OV680 " x);}while(0);
#define ov680_dbg(x...)    do{if(ov680_debug >= OV680_DEBUG_LVL3)pr_err("OV680 " x);}while(0);
#define ov680_verb(x...)   do{if(ov680_debug >= OV680_DEBUG_LVL4)pr_err("OV680 " x);}while(0);
#define ov680_rc(rc)       ov680_rc_i((rc), __func__)

static inline int ov680_rc_i(int rc, const char *fn)
{
	ov680_dbg("%s: RC = %d\n", fn, rc);
	return rc;
}

int tdiff (const struct timespec *t1, const struct timespec *t0);

#define OV680FW_NAME "ov680.fw"

/**
 * Sensor gain limitations.
 */
#define OV680_MAX_GAIN 248

/**
 * User Params
 */
enum {
	OV680_UPARM_FLASH_LED_SPAN_LOW,	//0x8000
	OV680_UPARM_FLASH_LED_SPAN_HIGH,	//0x8001
	//ISP 1
	OV680_UPARM_AEC1_WEIGHT_ZONE_0100,	//0x8002
	OV680_UPARM_AEC1_WEIGHT_ZONE_0302,	//0x8003
	OV680_UPARM_AEC1_WEIGHT_ZONE_1110,	//0x8004
	OV680_UPARM_AEC1_WEIGHT_ZONE_1312,	//0x8005
	OV680_UPARM_AEC1_WEIGHT_ZONE_2120,	//0x8006
	OV680_UPARM_AEC1_WEIGHT_ZONE_2322,	//0x8007
	OV680_UPARM_AEC1_WEIGHT_ZONE_3130,	//0x8008
	OV680_UPARM_AEC1_WEIGHT_ZONE_3332,	//0x8009
	//ISP 2
	OV680_UPARM_AEC2_WEIGHT_ZONE_0100,	//0x800A
	OV680_UPARM_AEC2_WEIGHT_ZONE_0302,	//0x800B
	OV680_UPARM_AEC2_WEIGHT_ZONE_1110,	//0x800C
	OV680_UPARM_AEC2_WEIGHT_ZONE_1312,	//0x800D
	OV680_UPARM_AEC2_WEIGHT_ZONE_2120,	//0x800E
	OV680_UPARM_AEC2_WEIGHT_ZONE_2322,	//0x800F
	OV680_UPARM_AEC2_WEIGHT_ZONE_3130,	//0x8010
	OV680_UPARM_AEC2_WEIGHT_ZONE_3332,	//0x8011
	//Manual exposure
	OV680_UPARM_SENSOR_0_EXPOSURE_LOW,	//0x8012
	OV680_UPARM_SENSOR_0_EXPOSURE_HIGH,	//0x8013
	OV680_UPARM_SENSOR_1_EXPOSURE_LOW,	//0x8014
	OV680_UPARM_SENSOR_1_EXPOSURE_HIGH,	//0x8015
	OV680_UPARM_SENSOR_2_EXPOSURE_LOW,	//0x8016
	OV680_UPARM_SENSOR_2_EXPOSURE_HIGH,	//0x8017
	OV680_UPARM_SENSOR_3_EXPOSURE_LOW,	//0x8018
	OV680_UPARM_SENSOR_3_EXPOSURE_HIGH,	//0x8019
	//Manual gain
	OV680_UPARM_SENSOR_0_GAIN,	//0x801A
	OV680_UPARM_SENSOR_1_GAIN,	//0x801B
	OV680_UPARM_SENSOR_2_GAIN,	//0x801C
	OV680_UPARM_SENSOR_3_GAIN,	//0x801D
	//Sensor Sel
	OV680_UPARM_SENSOR_SEL,	//0x801e
	OV680_UPARM_SENSOR_FRAME_MODE,	//0x801f
	//AEC Target
	OV680_UPARM_SENSOR_AEC_TARGET,	//0x8020

	OV680_UPARM_MAX,
};

enum{
	//READ Params
	OV680_UPARM_READ_SENSOR0_ID,	//0x9000
	OV680_UPARM_READ_SENSOR1_ID,	//0x9001
	OV680_UPARM_READ_SENSOR2_ID,	//0x9002
	OV680_UPARM_READ_SENSOR3_ID,	//0x9003
	OV680_UPARM_READ_SCRATCH,	//0x9004
	OV680_UPARM_READ_MAX,
};

/**
 * I/O Ops
 */
enum {
	OV680_IO_OPS_IRQ = 1,	// 0 - setup, 1 -
	OV680_IO_OPS_POWER = 2,
	OV680_IO_OPS_SLEEP = 3,
	OV680_IO_OPS_UPARM_WR = 0x8000,
	OV680_IO_OPS_UPARM_RD = 0x9000,
	OV680_IO_OPS_UPARM_TOP = OV680_IO_OPS_UPARM_RD + OV680_UPARM_MAX,
	OV680_IO_OPS_NOP = 0xFFFF,
};

/**
 * Frame rates ordered by index in the fw image.
 */
enum ov680_fps{
	OV680_FRAME_RATE_15 = 0,
	OV680_FRAME_RATE_30,
	OV680_FRAME_RATE_60,
	OV680_FRAME_RATE_120,
	OV680_FRAME_RATE_240,
	OV680_FRAME_RATE_8,
	OV680_FRAME_RATE_10,
	OV680_FRAME_RATE_21,
	OV680_FRAME_RATE_24,
	OV680_FRAME_RATE_1,
	OV680_FRAME_RATE_2,
	OV680_FRAME_RATE_3,
	OV680_FRAME_RATE_4,
	OV680_FRAME_RATE_5,
	OV680_FRAME_RATE_6,
	OV680_FRAME_RATE_18,
	OV680_FRAME_RATE_27,
	OV680_FRAME_RATE_40,
	OV680_FRAME_RATE_MAX,
};

/**
 * Auto exposure target by index in the fw image.
 */
enum {
	OV680_AUTO_EXPOSURE_TARGET_MIN = 1,
	OV680_AUTO_EXPOSURE_TARGET_DEFAULT = 10,
	OV680_AUTO_EXPOSURE_TARGET_MAX = 21,
};

/**
 * ISP enable disable by index in the fw image.
 */
enum ov680_isp_enale_disable {
	OV680_ISP_DISABLE,
	OV680_ISP_ENABLE,
	OV680_ISP_MAX
};

/**
 * LENC enable disable by index in the fw image.
 */
enum ov680_lenc_enale_disable {
	OV680_LENC_DISABLE,
	OV680_LENC_ENABLE,
	OV680_LENC_MAX
};

/**
 * DPC enable disable by index in the fw image.
 */
enum ov680_dpc_enale_disable {
	OV680_DPC_DISABLE,
	OV680_DPC_ENABLE,
	OV680_DPC_MAX
};

/**
 * Auto exposure by index in the fw image.
 */
enum ov680_auto_exposure {
	OV680_AUTO_EXPOSURE_DISABLE,
	OV680_AUTO_EXPOSURE_ENABLE,
	OV680_AUTO_EXPOSURE_MAX
};

/**
 * Flash enable/disable by index in the fw image.
 */
enum ov680_flash_mode {
	OV680_FLASH_DISABLE,
	OV680_FLASH_ENABLE,
	OV680_FLASH_MAX
};

/**
 * Flash config by index in the fw image.
 */
enum ov680_flash_config_mode {
	OV680_FLASH_CONFIG_NONE,
	OV680_FLASH_CONFIG_RUN,
	OV680_FLASH_CONFIG_MAX
};
/**
 * Manual exposure config by index in the fw image.
 */
enum ov680_exposure_config_mode {
	OV680_EXPOSURE_CONFIG_NONE,
	OV680_EXPOSURE_CONFIG_RUN,
	OV680_EXPOSURE_CONFIG_MAX
};

/**
 * Manual gain config by index in the fw image.
 */
enum ov680_gain_config_mode {
	OV680_GAIN_CONFIG_NONE,
	OV680_GAIN_CONFIG_RUN,
	OV680_GAIN_CONFIG_MAX
};

/**
 * Initializes the firmware image
 */
enum {
	OV680_FIRMWARE_INIT,
	OV680_FIRMWARE_STANDBY,
	OV680_FIRMWARE_RESUME,
	OV680_FIRMWARE_RESTORE,
};

/**
 * OV680 state machine values.
 */
enum ov680_state_t {
	OV680_STATE_OFF,
	//OV680_STATE_SETUP,
	OV680_STATE_STANDBY,
	OV680_STATE_IDLE,
	OV680_STATE_STREAMING,
	OV680_STATE_UNWIND,
};


enum ov680_switching_state_t {
	OV680_SWITCHING_IDLE,
	OV680_SWITCHING_ACTIVE,
};

/**
 * Number of sensors, implied height of image.
 * 1x1 = 400x400, 2x1 = 400x800.
 */
enum ov680_mode_t {
	OV680_MODE_1X1 = 1,
	OV680_MODE_2X1 = 2,
	OV680_MODE_4X1 = 4,
};

/**
 * Interrupt state machine. Mostly used for debugging.
 */
enum ov680_isr_state_t {
	OV680_ISR_INVALID,
	OV680_ISR_UNKNOWN,	// bad completion
	OV680_ISR_COMMAND_PENDING = 0x80,
	OV680_ISR_COMMAND_COMPLETE = 0x81,
};

/**
 * Frame mode
 */
enum ov680_frame_mode {
	OV680_FULL_RES = 0,
	OV680_BINNING_2x2,
	OV680_BINNING_2x2_SCALING_2x2,
	OV680_FRAME_MODE_MAX
};

enum ov680_fw_state_t {
    OV680_FW_MISSING,
    OV680_FW_LOADED
};

/**
 * Firmware sections
 */
enum ov680_fwi_section {
        OV680_FWI_FIRMWARE,
        OV680_FWI_FRAME_RATE,
        OV680_FWI_ISP_2A,
        OV680_FWI_ISP_2A_TARGET,
        OV680_FWI_ISP_2A_ROI_CONFIG,
        OV680_FWI_SENSOR_SEL,
        OV680_FWI_IDLE,
        OV680_FWI_FLASH,
        OV680_FWI_FLASH_CONFIG,
        OV680_FWI_FRAME_MODE,
        OV680_FWI_ISP_CONFIG,
        OV680_FWI_LENC_CONFIG,
        OV680_FWI_DPC_CONFIG,
        OV680_FWI_EXPOSURE_CONFIG,
        OV680_FWI_GAIN_CONFIG,
        OV680_FWI_MAX
};


#define EXP_META_UPDATE   (1 << 0)
#define GAIN_META_UPDATE  (1 << 1)
#define FPS_META_UPDATE   (1 << 2)
#define FLASH_SPAN_META_UPDATE   (1 << 3)

struct ov680_metadata_update {
	uint16_t exposure1;
	uint8_t  gain1;
	uint16_t exposure2;
	uint8_t  gain2;
	uint8_t  fps;
	uint16_t flash_duration;
        /* frame number to whcih this metadata needs to be applied */
	uint64_t frame_num;
	uint32_t flags;
	atomic_t updated;
};

#define OV680_MAX_METADATA_UPDATES 10

struct metadata_update_mgr {
	struct ov680_metadata_update list[OV680_MAX_METADATA_UPDATES];
	spinlock_t lock;
};

#define OV680_MAX_CURRENT_BUFFER 3
struct ov680_irled_current {
	int flash_current[4];
	uint64_t frame_num;
	uint32_t updated;
};

struct irled_current_update_mgr {
	struct ov680_irled_current list[OV680_MAX_CURRENT_BUFFER];
	spinlock_t lock;
};
/**
 * OV680 related settings storage.
 */
struct ov680_driver_priv_t {
	enum ov680_state_t state;
	enum ov680_mode_t streaming_mode;
	enum ov680_auto_exposure aec_state;
	enum ov680_fw_state_t fw_state;
	struct mutex fw_mutex;
	enum ov680_flash_config_mode flash_state;
	uint32_t current_sensors;
	uint32_t frame_rate;
	struct {
		wait_queue_head_t wq;
		volatile enum ov680_isr_state_t state;
		volatile int count_since_last;
		uint32_t count;	// for commands (resume) that issue two interrupts.
	} irq;
	struct ov680_sensor_v4l2_ctrl_info_t *ctrls;
	uint32_t ctrls_size;
	uint8_t params[OV680_UPARM_MAX];
	uint8_t read_params[OV680_UPARM_READ_MAX];
	struct ov680_fw_section ov680_fw_image[OV680_FWI_MAX];
	struct ov680_flash_ctrl_t flash_ctrl;
	struct msm_sensor_ctrl_t s_ctrl;
	struct msm_sensor_metadata metadata;
	uint64_t frame_cntr;
	struct metadata_update_mgr mdm;
	struct irled_current_update_mgr icm;
	atomic_t exp_set;
	atomic_t gain_set;
	atomic_t strobe_span_set;
	enum ov680_switching_state_t switching_state;
};

#define STRINGIFY(x) #x

/**
 * When standby is enabled, override MSM power up with a standby-capable version.
 */
int ov680_sensor_standby_power_up(struct msm_sensor_ctrl_t *s_ctrl);

/**
 * When standby is enabled, override MSM power down with a standby-capable version.
 */
int ov680_sensor_standby_power_down(struct msm_sensor_ctrl_t *s_ctrl);

/**
 * When standby is enabled, moddify the probe process to enable power using the
 * standard msm power up sequence first. On completion, ov680 will be in standby
 * mode (or on failure, the device will have fully powered off ov680).
 */
int ov680_sensor_standby_i2c_probe(struct i2c_client *client,
				   const struct i2c_device_id *id);

/**
 * Initializes the firmware image. Called during module startup to establish sane defaults.
 */
int ov680_fwi_init(void);

/**
 * Resets settings to defaults.
 */
int ov680_fwi_reset(void);

/**
 * Select a lut item to be included on the start up script.
 */
int ov680_fwi_select(enum ov680_fwi_section section, uint32_t index);

/**
 * Select a lut item to be immediately executed (if streaming).
 */
int ov680_fwi_run(enum ov680_fwi_section section, uint32_t index);

/**
 * Select a lut item to be immediately executed (regardless of state).
 */
int ov680_fwi_run_force(enum ov680_fwi_section section, uint32_t index);

/**
 * Run the startup sequence.
 */
int ov680_fwi_run_startup(void);

/**
 * Select a lut item to be immediately executed (regardless of state).
 */
int ov680_fwi_uparam_updated(enum ov680_fwi_section section);

/**
 * FW loading callback, gets invoked when userspace loads firware to the sysfs firmware clas
 */
void ov680_fw_loader_callback(const struct firmware *fw_img, void *context);

#endif
