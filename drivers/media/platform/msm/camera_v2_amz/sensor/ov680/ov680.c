/*
 * ov680.c
 *
 * Copyright (c) 2012-2013 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * PROPRIETARY/CONFIDENTIAL
 *
 * Use is subject to license terms.
 */

#include "msm_sensor.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include "msm_ispif_frame_event.h"
#include "ov680.h"
#include "ov680_config.h"
#include <mach/socinfo.h>
#include <mach/board-detect.h>
#include <linux/havok.h>  /* ACOS_MOD_ONELINE */


#define OV680_SENSOR_NAME "ov680"
#define PLATFORM_DRIVER_NAME "msm_camera_ov680"
DEFINE_MSM_MUTEX(ov680_mut);

//struct msm_sensor_ctrl_t ov680_s_ctrl;
#define OV680_XPWDN   SENSOR_GPIO_RESET
#define OV680_RESET_N SENSOR_GPIO_STANDBY

#undef CDBG
#define CDBG pr_err

#define OV6710_US_TO_LINES(exp_us) (((exp_us)*1000)/19025)
#define OV6710_LINES_TO_EXPOSURE_US(lines) (((lines)*19025)/1000)


#define OV680_DEFAULT_FPS    30

static unsigned int board_rev;
int ov680_debug = OV680_DEBUG_LVL0;
module_param(ov680_debug, int, 0644);
void metadata_update_list_init(struct metadata_update_mgr* mdm);
int dispatch_metadata_update(struct metadata_update_mgr* mdm, struct ov680_metadata_update* update);
void irled_current_update_list_init(struct irled_current_update_mgr* icm);
int dispatch_irled_current_update(struct irled_current_update_mgr* icm, struct ov680_irled_current* update);
void ov680_static_exp_gain_update(struct msm_sensor_ctrl_t *sctrl);
static int ov680_set_irled_config(struct ov680_irled_config * irled_config);
void get_delayed_metadata_updates(struct msm_sensor_ctrl_t *sctrl,struct msm_sensor_metadata *sensor_metadata);


// In microseconds
int tdiff(const struct timespec *t1, const struct timespec *t0)
{
        return
                ((t1->tv_sec - t0->tv_sec) * 1000000) +
                ((t1->tv_nsec - t0->tv_nsec) / 1000);
}

int irq_num = 0;

/* Atomic Operations */

static inline void ov680_atomic_inc(atomic_t* var)
{
	int ret;
	smp_mb__before_atomic_inc();
	ret = atomic_inc_return(var);
	smp_mb__after_atomic_inc();
	ov680_dbg("%s [%d]", __func__, ret);
}

static inline void ov680_atomic_dec(atomic_t *var)
{
	int ret;
	smp_mb__before_atomic_dec();
	ret = atomic_dec_return(var);
	smp_mb__after_atomic_dec();
	if(unlikely(ret <0))
	{
		ov680_err("%s -ve[%d]", __func__, ret);
		smp_mb();
		ret = atomic_set(var, 0);
		smp_mb();
	}
	ov680_dbg("%s [%d]", __func__, ret);
}

static inline int ov680_atomic_test(atomic_t *var)
{
	int val;
	val = atomic_read(var);
	ov680_dbg("%s [%d]", __func__, val);
	return (val > 0)?1:0;
}

int validate_sensor_indexes(int x, ... )
{
	int idx,id,i, base;
	int ret=0;
	int sensor_idreg_map[] = {0xc8, 0xc4, 0xcc, 0xca};
	va_list a_list;
	base = OV680_UPARM_READ_SENSOR0_ID;

	va_start(a_list,x);
	for(i=0;i<x;i++)
	{
		idx = va_arg(a_list, int);
		id = idx -1;
		ov680_dbg("%s idx=%d",__func__, idx);
		if(ov680_props.read_params[base+id] != sensor_idreg_map[id])
		{
			ov680_err("sensor %d was not communicable during POST", id);
			ret = -1;
			break;
		}
	}
	va_end(a_list);

    return ret;
}

static int ov680_sensor_idx_set(struct msm_sensor_ctrl_t *s_ctrl,
				unsigned int id)
{
	int s_a, s_b, s_c, s_d, rc;
	int idx = 0;
	int section = 0;

	s_a = id & 0xF;
	s_b = (id >> 4) & 0xF;
	s_c = (id >> 8) & 0xF;
	s_d = (id >> 12) & 0xF;
	rc = -EINVAL;

	ov680_dbg("Sensor select req=%02X s_a=%d sa_b=%d s_c=%d s_d=%d\n", id,
		  s_a, s_b, s_c, s_d);

	if (s_a == 0) {
		ov680_err("A sensor's index is out of range: %d,%d,%d,%d\n",
			  s_a, s_b, s_c, s_d);
		return -EINVAL;
	}

	if (s_a > 4 || s_b > 4 || s_c > 4 || s_d > 4) {
		ov680_err("Selected sensors are out of range: %d,%d,%d,%d\n",
			  s_a, s_b, s_c, s_d);
		return -EINVAL;
	}

	if(board_rev == URSA_REVISION_P1){
	// P1 fix - sensors 0 and 1 are swapped
		if (s_a == 1) {
		    s_a = 2;
		} else if (s_a == 2) {
		    s_a = 1;
		}

		if (s_b == 1) {
		    s_b = 2;
		} else if (s_b == 2) {
		    s_b = 1;
		}
	}
	if (s_a == 1 && s_b == 2 && s_c == 3 && s_d == 4) {
		if(validate_sensor_indexes(4, s_a, s_b, s_c, s_d))
		{
			ov680_err("Cannot select 4 sensor combination");
			return -ENODEV;
		}
		idx = 4 + (4 * 4);
		ov680_props.streaming_mode = OV680_MODE_4X1;
		section = 2;
	} else if (s_a > 0 && s_b > 0 && s_c > 0 && s_d > 0) {
		ov680_err("4 sensor mode only support config 1,2,3,4.\n");
		return -EINVAL;
	} else if (s_a > 0 && s_b > 0 && s_c > 0) {
		ov680_err("3 sensor not supported.\n");
		return -EINVAL;
	} else if (s_a > 0 && s_b > 0) {
		if(validate_sensor_indexes(2, s_a, s_b))
		{
			ov680_err("Cannot select this 2 sensor combination");
			return -ENODEV;
		}
		ov680_props.streaming_mode = OV680_MODE_2X1;
		idx = (s_b -1) + ((s_a -1) * 4);
		section = 1;
	} else if (s_a > 0) {
		if(validate_sensor_indexes(1, s_a))
		{
			ov680_err("Cannot select this sensor combination");
			return -ENODEV;
		}
		ov680_props.streaming_mode = OV680_MODE_1X1;
		idx = s_a - 1;
		section = 0;
	} else {
		ov680_err
		    ("Selected sensors are an invalid configuration: %d,%d,%d,%d\n",
		     s_a, s_b, s_c, s_d);
		return -EINVAL;
	}
	ov680_dbg("sensor select index=%d section=%d\n", idx, section);

	/* Update uparam */
	ov680_props.params[OV680_UPARM_SENSOR_SEL] = idx;
	ov680_props.switching_state = OV680_SWITCHING_ACTIVE;
	ov680_fwi_run(OV680_FWI_SENSOR_SEL, section);
	rc = ov680_fwi_select(OV680_FWI_SENSOR_SEL, section);
	ov680_props.current_sensors = id;
	ov680_props.metadata.input = id;
	ov680_static_exp_gain_update(s_ctrl);
	ov680_props.switching_state = OV680_SWITCHING_IDLE;

	return ov680_rc(rc);
}

static int ov680_sensor_set_frame_mode(struct ov680_sensor_v4l2_ctrl_info_t
				       *ctrl_info, int value)
{
	int rc = -ERANGE;

	ov680_dbg("Sensor frame mode change: %d \n", value);

	ov680_props.params[OV680_UPARM_SENSOR_FRAME_MODE]= value;
	rc = ov680_fwi_select(OV680_FWI_FRAME_MODE, 0);

	return ov680_rc(rc);
}

static int ov680_sensor_set_aec_agc_target(struct ov680_sensor_v4l2_ctrl_info_t
					   *ctrl_info, int value)
{
	int rc = 0;

	ov680_dbg("Setting auto exposure target: %d\n", value);

	ov680_props.params[OV680_UPARM_SENSOR_AEC_TARGET]= value - OV680_AUTO_EXPOSURE_TARGET_MIN;

	if(ov680_props.aec_state == OV680_AUTO_EXPOSURE_ENABLE)
		ov680_fwi_run(OV680_FWI_ISP_2A_TARGET,0);
	rc = ov680_fwi_select(OV680_FWI_ISP_2A_TARGET,0);

	return ov680_rc(rc);
}

static int ov680_sensor_set_aec_agc_mode(struct ov680_sensor_v4l2_ctrl_info_t
					 *ctrl_info, int value)
{
	int rc = 0;

	ov680_dbg("Setting auto exposure: %d\n", value);

	ov680_props.aec_state = value;

	ov680_fwi_run(OV680_FWI_ISP_2A, value);
	rc = ov680_fwi_select(OV680_FWI_ISP_2A, value);

	if(value == OV680_AUTO_EXPOSURE_DISABLE)
	{
		ov680_verb("OV680_AUTO_EXPOSURE_DISABLE update");
		ov680_props.metadata.AEC_on = 0b0;
		ov680_props.metadata.AGC_on = 0b0;

	}
	else if(value == OV680_AUTO_EXPOSURE_ENABLE)
	{
		ov680_verb("OV680_AUTO_EXPOSURE_ENABLE update");
		ov680_props.metadata.AEC_on = 0b1;
		ov680_props.metadata.AGC_on = 0b1;
	}

	// Apply exposure/gain settings if AEC was turned off
	if( value == OV680_AUTO_EXPOSURE_DISABLE ) {
	    /* actual exposure and gain would get set after SOF frame event from ispif */
	    ov680_atomic_inc(&ov680_props.exp_set);
	    ov680_atomic_inc(&ov680_props.gain_set);

	    ov680_fwi_run(OV680_FWI_GAIN_CONFIG, OV680_GAIN_CONFIG_RUN);
	    rc = ov680_fwi_select(OV680_FWI_GAIN_CONFIG, OV680_GAIN_CONFIG_RUN);
	    if(rc) return ov680_rc(rc);
	}

	return ov680_rc(rc);
}

static int ov680_sensor_isp_config(struct ov680_sensor_v4l2_ctrl_info_t
				   *ctrl_info, int value)
{
	int rc = 0;
	ov680_dbg("ISP config : %d\n", value);

	ov680_fwi_run(OV680_FWI_ISP_CONFIG, value);
	rc = ov680_fwi_select(OV680_FWI_ISP_CONFIG, value);

	return ov680_rc(rc);
}

static int ov680_sensor_lenc_config(struct ov680_sensor_v4l2_ctrl_info_t
				    *ctrl_info, int value)
{
	int rc = 0;
	ov680_dbg("LENC config : %d\n", value);

	ov680_fwi_run(OV680_FWI_LENC_CONFIG, value);
	rc = ov680_fwi_select(OV680_FWI_LENC_CONFIG, value);

	return ov680_rc(rc);
}

int ov680_sensor_dpc_config(struct ov680_sensor_v4l2_ctrl_info_t *ctrl_info,
			    int value)
{
	int rc = 0;
	ov680_dbg("DPC config : %d\n", value);

	ov680_fwi_run(OV680_FWI_DPC_CONFIG, value);
	rc = ov680_fwi_select(OV680_FWI_DPC_CONFIG, value);

	return ov680_rc(rc);
}

static int ov680_sensor_set_fps(struct ov680_sensor_v4l2_ctrl_info_t *ctrl_info,
				int value)
{
	int32_t rc = 0;
	int16_t fps_value = 0;
	struct ov680_driver_priv_t* priv = &ov680_props;
	struct ov680_metadata_update meta_update;

	ov680_dbg("Set FPS to %d \n", value);

	switch (value) {
	case 1:
		fps_value = OV680_FRAME_RATE_1;
		break;
	case 2:
		fps_value = OV680_FRAME_RATE_2;
		break;
	case 3:
		fps_value = OV680_FRAME_RATE_3;
		break;
	case 4:
		fps_value = OV680_FRAME_RATE_4;
		break;
	case 5:
		fps_value = OV680_FRAME_RATE_5;
		break;
	case 6:
		fps_value = OV680_FRAME_RATE_6;
		break;
	case 8:
		fps_value = OV680_FRAME_RATE_8;
		break;
	case 10:
		fps_value = OV680_FRAME_RATE_10;
		break;
	case 15:
		fps_value = OV680_FRAME_RATE_15;
		break;
	case 18:
		fps_value = OV680_FRAME_RATE_18;
		break;
	case 21:
		fps_value = OV680_FRAME_RATE_21;
		break;
	case 24:
		fps_value = OV680_FRAME_RATE_24;
		break;
	case 27:
		fps_value = OV680_FRAME_RATE_27;
		break;
	case 30:
		fps_value = OV680_FRAME_RATE_30;
		break;
	case 40:
		fps_value = OV680_FRAME_RATE_40;
		break;
	case 60:
		fps_value = OV680_FRAME_RATE_60;
		break;
	case 120:
		fps_value = OV680_FRAME_RATE_120;
		break;
	case 240:
		fps_value = OV680_FRAME_RATE_240;
		break;
	default:
		ov680_err("Invalid frame rate - %d\n", value);
		return -ERANGE;
		break;
	}

	priv->frame_rate = value;

	// Reduce IRLED current if we're moving above the maximum
	if(priv->flash_state == OV680_FLASH_CONFIG_RUN) {
		uint8_t i = 0;
		uint8_t updated = 0;
		uint32_t irled_duration;
		struct ov680_irled_max_current_req max_current_req;
		struct ov680_irled_config irled_config;

		irled_duration = OV6710_LINES_TO_EXPOSURE_US(
						(priv->params[OV680_UPARM_FLASH_LED_SPAN_HIGH] << 8) |
						priv->params[OV680_UPARM_FLASH_LED_SPAN_LOW]
					  );

		max_current_req.frame_rate = value;
		max_current_req.duration = irled_duration;
		ov680_flash_get_max_current(&priv->flash_ctrl, &max_current_req);
		for(i = 0; i <= 3; i++) {
			if( priv->flash_ctrl.led_desc[i].flash_current > max_current_req.max_current ) {
				irled_config.flash_current[i] = max_current_req.max_current;
				updated = 1;
			} else {
				irled_config.flash_current[i] = priv->flash_ctrl.led_desc[i].flash_current;
			}
		}
		if (updated) {
			irled_config.duration = irled_duration;
			rc = ov680_set_irled_config(&irled_config);
		}
		if (rc) {
			return ov680_rc(rc);
		}
	}

	if (!rc) {
		ov680_fwi_run(OV680_FWI_FRAME_RATE, fps_value);
		ov680_fwi_select(OV680_FWI_FRAME_RATE, fps_value);
	}

	if(priv->state == OV680_STATE_STREAMING)
	{
		memset(&meta_update, 0, sizeof(meta_update));
		meta_update.fps = value;
		meta_update.frame_num = priv->frame_cntr + 2;
		meta_update.flags = FPS_META_UPDATE;
		dispatch_metadata_update(&priv->mdm, &meta_update);
	}
	else
		priv->metadata.frame_rate = value;

	return ov680_rc(rc);

}

int ov680_set_exposure(struct ov680_exposure * exposure)
{
	int rc = 0;
	int exposure_lines;
	int i = 0;
	uint32_t period_us = (1000 * 1000 / ov680_props.frame_rate);

	if (exposure->sensor_idx == 0) {
		rc = -EINVAL;
		ov680_err("Sensor mask cannot be 0");
		return ov680_rc(rc);
	}

	if (exposure->exposure > period_us) {
		rc = -EINVAL;
		ov680_err("Exposure value (%d) is greater than frame period (%d)", exposure->exposure, period_us);
		return ov680_rc(rc);
	}

	// Convert from us to lines
	exposure_lines = OV6710_US_TO_LINES(exposure->exposure);


	ov680_dbg("Exposure value: %d us (%d lines)\n", exposure->exposure,
		  exposure_lines);
	for (i = 0; i <= 3; i++) {
		if (exposure->sensor_idx & (1 << i)) {
			ov680_props.params[OV680_UPARM_SENSOR_0_EXPOSURE_LOW +
					   2 * (i)] =
			    (exposure_lines << 4) & 0xFF;
			ov680_props.params[OV680_UPARM_SENSOR_0_EXPOSURE_HIGH +
					   2 * (i)] =
			    (exposure_lines >> 4) & 0xFF;
		}
	}

	/* actual exposure would get set after SOF frame event from ispif */
	if (ov680_props.aec_state == OV680_AUTO_EXPOSURE_DISABLE)
		ov680_atomic_inc(&ov680_props.exp_set);

	return ov680_rc(rc);
}

uint32_t ov680_set_exposure_on_sof(struct ov680_driver_priv_t*  priv)
{
	int rc = 0;
	if (priv->aec_state == OV680_AUTO_EXPOSURE_DISABLE && ov680_atomic_test(&priv->exp_set)) {
		rc = ov680_fwi_run(OV680_FWI_EXPOSURE_CONFIG,
			      OV680_EXPOSURE_CONFIG_RUN);

		if(rc)
		{
			ov680_err("%s failed %d", __func__, rc);
			priv->metadata.exposure_failed = 0b1;
		}
		else
		{
			ov680_atomic_dec(&priv->exp_set);
			priv->metadata.exposure_failed = 0b0;
			ov680_fwi_select(OV680_FWI_EXPOSURE_CONFIG,
				      OV680_EXPOSURE_CONFIG_RUN);
		}
	}
	return ov680_rc(rc);
}

uint32_t ov680_set_gain_on_sof(struct ov680_driver_priv_t*  priv)
{
	int rc = 0;
	if (priv->aec_state == OV680_AUTO_EXPOSURE_DISABLE && ov680_atomic_test(&priv->gain_set)) {
		rc = ov680_fwi_run(OV680_FWI_GAIN_CONFIG, OV680_GAIN_CONFIG_RUN);
		if(rc)
		{
			ov680_err("%s failed %d", __func__, rc);
			priv->metadata.gain_failed = 0b1;
		}
		else
		{
			ov680_atomic_dec(&priv->gain_set);
			priv->metadata.gain_failed = 0b0;
			ov680_fwi_select(OV680_FWI_GAIN_CONFIG, OV680_GAIN_CONFIG_RUN);
		}
	}
	return ov680_rc(rc);
}

int ov680_set_gain(struct ov680_gain * gain)
{
	int rc = 0;
	int i = 0;

	if (gain->sensor_idx == 0) {
		rc = -EINVAL;
		ov680_err("Sensor mask cannot be 0");
		return ov680_rc(rc);
	}

	if (gain->gain > OV680_MAX_GAIN) {
		rc = -EINVAL;
		ov680_err("Gain value (%d) is greater than maximum gain (%d)", gain->gain, OV680_MAX_GAIN);
		return ov680_rc(rc);
	}

	ov680_dbg(" Gain value: %d\n ", gain->gain);
	for (i = 0; i <= 3; i++) {
		if (gain->sensor_idx & (1 << i)) {
			ov680_props.params[OV680_UPARM_SENSOR_0_GAIN + (i)] =
			    (gain->gain) & 0xFF;
		}
	}

	/* actual gain would get set after SOF frame event from ispif */
	if (ov680_props.aec_state == OV680_AUTO_EXPOSURE_DISABLE)
		ov680_atomic_inc(&ov680_props.gain_set);


	return ov680_rc(rc);
}

static int ov680_set_irled_config(struct ov680_irled_config * irled_config)
{
	int rc = 0;
	int i = 0;
	int span_lines = OV6710_US_TO_LINES(irled_config->duration);
	struct ov680_irled_max_current_req max_current_req;

	// Check to see if the current is allowed!
	max_current_req.frame_rate = ov680_props.frame_rate;
	max_current_req.duration = irled_config->duration;
	ov680_flash_get_max_current(&ov680_props.flash_ctrl, &max_current_req);

	for (i = 0; i <= 3; i++) {
	    if (irled_config->flash_current[i] > max_current_req.max_current) {
		rc = -ERANGE;
		return ov680_rc(rc);
	    }
	}

	ov680_props.flash_state = (span_lines > 0) ? OV680_FLASH_CONFIG_RUN: OV680_FLASH_CONFIG_NONE;

	// Update span
	ov680_dbg("Flash span: %d us, %d lines\n", irled_config->duration, span_lines);
	ov680_props.params[OV680_UPARM_FLASH_LED_SPAN_LOW] = span_lines & 0xff;
	ov680_props.params[OV680_UPARM_FLASH_LED_SPAN_HIGH] =
	    (span_lines >> 8) & 0xff;

	// If not streaming, just statically select everything - otherwise, update dynamically
	if (ov680_props.state == OV680_STATE_STREAMING) {
		struct ov680_irled_current update;
		for (i = 0; i <= 3; i++) {
		    update.flash_current[i] = irled_config->flash_current[i];
		}

		// Apply the current update 2 EOFs from the next SOF
		update.frame_num = ov680_props.frame_cntr + 2;
		ov680_atomic_inc(&ov680_props.strobe_span_set);

		rc = dispatch_irled_current_update(&ov680_props.icm, &update);

	} else {
		for (i = 0; i <= 3; i++) {
		    ov680_props.flash_ctrl.led_desc[i].flash_current = irled_config->flash_current[i];
		    ov680_props.metadata.flash_current[i] = irled_config->flash_current[i];
		}

		rc = ov680_fwi_select(OV680_FWI_FLASH_CONFIG,
				    ov680_props.flash_state );
		ov680_props.metadata.flash_duration = OV6710_LINES_TO_EXPOSURE_US(span_lines);
	}
	return ov680_rc(rc);
}

uint32_t ov680_set_strobe_span_on_sof(struct ov680_driver_priv_t*  priv)
{
	int rc = 0;
	struct ov680_metadata_update meta_update;
	memset(&meta_update, 0, sizeof(meta_update));

	if (ov680_atomic_test(&priv->strobe_span_set)) {
		rc = ov680_fwi_run(OV680_FWI_FLASH_CONFIG, ov680_props.flash_state );
		if(rc)
		{
			ov680_err("%s failed %d", __func__, rc);
			priv->metadata.flash_duration_failed = 0b1;
		}
		else
		{
			priv->metadata.flash_duration_failed = 0b0;
			ov680_fwi_select(OV680_FWI_FLASH_CONFIG,
				    ov680_props.flash_state);
			ov680_dbg("Updating span on frame %llu\n", priv->frame_cntr);

			meta_update.flash_duration = OV6710_LINES_TO_EXPOSURE_US(
					(ov680_props.params[OV680_UPARM_FLASH_LED_SPAN_HIGH] << 8) |
					ov680_props.params[OV680_UPARM_FLASH_LED_SPAN_LOW]
				      );
			meta_update.frame_num = priv->frame_cntr + 2;
			meta_update.flags = FLASH_SPAN_META_UPDATE;

			dispatch_metadata_update(&priv->mdm, &meta_update);
			ov680_atomic_dec(&ov680_props.strobe_span_set);
		}
	}

	return ov680_rc(rc);
}

uint32_t ov680_set_flash_current_on_eof(struct ov680_driver_priv_t*  priv)
{
	int rc = 0;
	int i;
	int flash_update = 0;
	uint64_t prev_frame_num = 0;

	uint32_t flash_current[4];

	// Go through the list and look for a frame that requires a current update
	for(i=0;i<OV680_MAX_CURRENT_BUFFER;i++)
	{
		if(!priv->icm.list[i].updated)
		{
			/* check if this applies to current frame, or if we're running behind */
			if(priv->icm.list[i].frame_num <= priv->frame_cntr)
			{
				// If processing is delayed, we might have multiple flash updates for the current frame.  Take the latest one.
				if(priv->icm.list[i].frame_num > prev_frame_num) {
					int j = 0;
					for (j = 0; j <= 3; j++) {
						flash_current[j] = priv->icm.list[i].flash_current[j];
					}
					prev_frame_num = priv->icm.list[i].frame_num;
					flash_update = 1;
				}
				priv->icm.list[i].updated = 1;
			}
		}
	}
	if(flash_update) {
		for (i = 0; i <= 3; i++) {
			priv->flash_ctrl.led_desc[i].flash_current = flash_current[i];
		}
		rc = ov680_flash_enable(&priv->flash_ctrl, 1);

		if(rc){
			ov680_err("%s failed %d", __func__, rc);
			priv->metadata.flash_current_failed = 0b1;
		} else {
			// Update flash current metadata
			priv->metadata.flash_current_failed = 0b0;
			for (i = 0; i <= 3; i++) {
				priv->metadata.flash_current[i] = flash_current[i];
			}
		}
	}

	return ov680_rc(rc);
}

static int get_aec_index(int index)
{
	int s_0, s_1, id;
	ov680_dbg("%s index[%d]\n", __func__, index);

	if (ov680_props.streaming_mode == OV680_MODE_2X1) {
		id = ov680_props.current_sensors;
		ov680_dbg("%s OV680_MODE_2X1 id[%x]\n", __func__, id);

		s_0 = id & 0xF;
		s_1 = (id >> 4) & 0xF;
		if (s_0 == index)
			return 0;
		else if (s_1 == index)
			return 1;
		else
			return -1;

	} else if (ov680_props.streaming_mode == OV680_MODE_1X1) {
		ov680_dbg("%s OV680_MODE_1X1\n", __func__);
		return 0;
	}

	return -1;
}

static uint32_t ov680_set_roi(struct ov680_roi *roi)
{
	uint32_t found_change = FALSE;
	int i, j, aec_index;
	uint16_t param_base = 0;
	uint8_t value;
	int rc = -EINVAL;

	ov680_dbg("%s", __func__);

	if (ov680_props.streaming_mode == OV680_MODE_4X1)
		return 0;
		/* not supported in 4 sensor streaming mode */

	/* find out which AEC unit is being used by sensor at index roi->sensor_idx */
	aec_index = get_aec_index(roi->sensor_idx);	/* supplied as a 1 based index */
	if (aec_index == 0) {
		param_base = OV680_UPARM_AEC1_WEIGHT_ZONE_0100;
		rc = 0;
	} else if (aec_index == 1) {
		param_base = OV680_UPARM_AEC2_WEIGHT_ZONE_0100;
		rc = 0;
	} else
		ov680_err("Invalid sensor_idx provided [%d]\n",
			  roi->sensor_idx);

	ov680_dbg("Setting idx[%d] to AEC[%d]\n", roi->sensor_idx, aec_index);

	if (rc)
		return rc;

	for (i = 0; i < 4; i++) {
		for (j = 0; j < 2; j++) {
			value = roi->zone_weight[i][j*2] |
				(roi->zone_weight[i][j*2 +1] << 4);
			found_change |= ov680_props.params[param_base] != value;
			ov680_props.params[param_base++] = value;
		}
	}

	if (found_change)
		ov680_fwi_uparam_updated(OV680_FWI_ISP_2A_ROI_CONFIG);

	if(ov680_props.aec_state == OV680_AUTO_EXPOSURE_ENABLE)
		ov680_fwi_run(OV680_FWI_ISP_2A_ROI_CONFIG, 0);

	return 0;
}

struct ov680_sensor_v4l2_ctrl_info_t ov680_v4l2_ctrl_info[] = {
	{
	 .ctrl_id = MSM_V4L2_PID_OV680_SENSOR_FRAME_MODE,
	 .min = OV680_FULL_RES,
	 .max = OV680_FRAME_MODE_MAX -1,
	 .step = 1,
	 .s_v4l2_ctrl = ov680_sensor_set_frame_mode,
	 },
	{
	 .ctrl_id = MSM_V4L2_PID_OV680_AEC_AGC_MODE,
	 .min = OV680_AUTO_EXPOSURE_DISABLE,
	 .max = OV680_AUTO_EXPOSURE_ENABLE,
	 .step = 1,
	 .s_v4l2_ctrl = ov680_sensor_set_aec_agc_mode,
	 },
	{
	 .ctrl_id = MSM_V4L2_PID_OV680_AEC_AGC_TARGET,
	 .min = OV680_AUTO_EXPOSURE_TARGET_MIN,
	 .max = OV680_AUTO_EXPOSURE_TARGET_MAX - 1,
	 .step = 1,
	 .s_v4l2_ctrl = ov680_sensor_set_aec_agc_target,
	 },
	{
	 .ctrl_id = MSM_V4L2_PID_OV680_FPS,
	 .min = 1,
	 .max = 120,
	 .step = 1,
	 .s_v4l2_ctrl = ov680_sensor_set_fps,
	 },
	{
	 .ctrl_id = MSM_V4L2_PID_OV680_SENSOR_ISP_CONFIG,
	 .min = OV680_ISP_DISABLE,
	 .max = OV680_ISP_MAX -1,
	 .step = 1,
	 .s_v4l2_ctrl = ov680_sensor_isp_config,
	 },
	{
	 .ctrl_id = MSM_V4L2_PID_OV680_SENSOR_LENC_CONFIG,
	 .min = OV680_LENC_DISABLE,
	 .max = OV680_LENC_MAX -1,
	 .step = 1,
	 .s_v4l2_ctrl = ov680_sensor_lenc_config,
	 },
	{
	 .ctrl_id = MSM_V4L2_PID_OV680_SENSOR_DPC_CONFIG,
	 .min = OV680_DPC_DISABLE,
	 .max = OV680_DPC_MAX -1,
	 .step = 1,
	 .s_v4l2_ctrl = ov680_sensor_dpc_config,
	 },
};

int ov680_sensor_v4l2_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int rc = -1, i = 0;

	struct ov680_sensor_v4l2_ctrl_info_t *v4l2_ctrl = ov680_props.ctrls;
	int size = ov680_props.ctrls_size;

	ov680_dbg("%s\n", __func__);
	ov680_dbg("%d\n", ctrl->id);

	if (v4l2_ctrl == NULL)
		return rc;
	for (i = 0; i < size; i++) {
		if (v4l2_ctrl[i].ctrl_id == ctrl->id) {
			if (v4l2_ctrl[i].s_v4l2_ctrl != NULL) {
				ov680_dbg
				    ("\n calling msm_sensor_s_ctrl_by_enum\n");
				if(ctrl->value <= v4l2_ctrl[i].max && ctrl->value >= v4l2_ctrl[i].min)
				{

					rc = v4l2_ctrl[i].s_v4l2_ctrl(&v4l2_ctrl[i],
							      ctrl->value);
				}
				else
				{
					rc  = -EINVAL;
				}
			}
			break;
		}
	}

	return rc;
}

static struct msm_sensor_power_setting ov680_power_setting[] = {
	{
	 .seq_type = SENSOR_VREG,
	 .seq_val = CAM_VANA,
	 .config_val = 0,
	 .delay = 2,
	 },
	{
	 .seq_type = SENSOR_VREG,
	 .seq_val = CAM_VDIG,
	 .config_val = 0,
	 .delay = 0,
	 },
	{
	 .seq_type = SENSOR_CLK,
	 .seq_val = SENSOR_CAM_MCLK,
	 .config_val = 19200000,
	 .delay = 1,
	 },
	{
	 .seq_type = SENSOR_GPIO,
	 .seq_val = OV680_XPWDN,
	 .config_val = GPIO_OUT_HIGH,
	 .delay = 10,
	 },
	{
	 .seq_type = SENSOR_GPIO,
	 .seq_val = OV680_RESET_N,
	 .config_val = GPIO_OUT_LOW,
	 .delay = 1,
	 },
	{
	 .seq_type = SENSOR_GPIO,
	 .seq_val = OV680_RESET_N,
	 .config_val = GPIO_OUT_HIGH,
	 .delay = 40,
	 },
	{
	 .seq_type = SENSOR_I2C_MUX,
	 .seq_val = 0,
	 .config_val = 0,
	 .delay = 0,
	 },
};

static struct msm_sensor_power_setting ov680_power_setting_p0_5[] = {
	{
	 .seq_type = SENSOR_VREG,
	 .seq_val = CAM_VIO,
	 .config_val = 0,
	 .delay = 0,
	 },
	{
	 .seq_type = SENSOR_VREG,
	 .seq_val = CAM_VANA,
	 .config_val = 0,
	 .delay = 2,
	 },
	{
	 .seq_type = SENSOR_CLK,
	 .seq_val = SENSOR_CAM_MCLK,
	 .config_val = 19200000,
	 .delay = 1,
	 },
	{
	 .seq_type = SENSOR_VREG,
	 .seq_val = CAM_VDIG,
	 .config_val = 0,
	 .delay = 0,
	 },
	{
	 .seq_type = SENSOR_GPIO,
	 .seq_val = OV680_XPWDN,
	 .config_val = GPIO_OUT_LOW,
	 .delay = 1,
	 },
	{
	 .seq_type = SENSOR_GPIO,
	 .seq_val = OV680_RESET_N,
	 .config_val = GPIO_OUT_LOW,
	 .delay = 1,
	 },
	{
	 .seq_type = SENSOR_GPIO,
	 .seq_val = OV680_RESET_N,
	 .config_val = GPIO_OUT_HIGH,
	 .delay = 40,
	 },
	{
	 .seq_type = SENSOR_I2C_MUX,
	 .seq_val = 0,
	 .config_val = 0,
	 .delay = 0,
	 },
};

static int ov680_i2c_cb(struct msm_camera_i2c_client *client,
			struct msm_camera_i2c_reg_conf *reg_conf_tbl)
{
	int rc = -1;

	ov680_verb("I2C OP cmd=%d, data=%d\n", reg_conf_tbl->reg_addr,
		   reg_conf_tbl->reg_data);

	if (reg_conf_tbl->reg_addr == OV680_IO_OPS_IRQ) {
		uint32_t cmd = reg_conf_tbl->reg_data & 0xF;
		uint32_t count = reg_conf_tbl->reg_data >> 4;
		if (count == 0)
			count = 1;	// for older firmware images
		/*
		 * Manage interrupt command complete
		 *
		 * States:
		 *  - OV680_ISR_COMMAND_PENDING  -> Pending IRQ rx.
		 *  - OV680_ISR_COMMAND_COMPLETE -> IRQ successfully completed.
		 *  - OV680_ISR_UNKNOWN          -> Not waiting for an IRQ.
		 */
		if (cmd == 0x0) {
			// Prepares the IRQ state for command complete processing
			// The IRQ keeps some status information to make debugging
			// a little easier. I.e. you can detect too many IRQs, too
			// few, or when they are unexpected.
			ov680_verb("Command IRQ: Setup. (Count=%d)\n", count);
#ifdef TPLMS_INST
			TPLMS_CAM(TID_CAM_OV680_COMMAND_IRQ_START);
#endif
			ov680_props.irq.count = count;
			ov680_props.irq.state = OV680_ISR_COMMAND_PENDING;
			rc = 0;

			ov680_verb("Command IRQ: %d interrupts.\n",
				   ov680_props.irq.count_since_last);
		} else if (cmd == 0x1) {
			// Waits for the IRQ state to be updated to indicate completion.
			// Times out after 2500ms. (Consider reducing to detect failures
			// sooner.) Returns failure, if IRQ did not exactly match the
			// specification. (See note above.)
			const uint32_t timeout = msecs_to_jiffies(2500);
			uint32_t time;
			ov680_verb("Command IRQ: Waiting.\n");
#ifdef TPLMS_INST
			TPLMS_CAM(TID_CAM_OV680_COMMAND_IRQ_WAIT);
#endif
			time = wait_event_timeout(ov680_props.irq.wq,
						  ov680_props.irq.state ==
						  OV680_ISR_COMMAND_COMPLETE,
						  timeout);
			if (time == 0) {
				ov680_err("Command IRQ: Timeout.\n");
				rc = 1;
			} else {
				ov680_dbg("Command IRQ: Success.\n");
				rc = 0;
			}
#ifdef TPLMS_INST
			TPLMS_CAM(TID_CAM_OV680_COMMAND_IRQ_END);
#endif
			ov680_props.irq.state = OV680_ISR_UNKNOWN;
			ov680_dbg("Command IRQ: %d interrupts.\n",
				  ov680_props.irq.count_since_last);
		} else {
			ov680_err("Invalid command irq op: %d\n",
				  reg_conf_tbl->reg_data);
			rc = -1;
		}

		ov680_props.irq.count_since_last = 0;
	} else if (reg_conf_tbl->reg_addr == OV680_IO_OPS_POWER) {
		// Manipulates GPIO for asserting/deasserting HW standby
		// on the ov680 part. This is needed because the fwi
		// sequences involve toggling midway through the I2C
		// writes.
		if (reg_conf_tbl->reg_data == 0x0) {
			gpio_set_value(OV680_PWDN_GPIO, 1);	// assert
			rc = 0;
			ov680_dbg("PWDN assert\n");
		} else if (reg_conf_tbl->reg_data == 0x1) {
			gpio_set_value(OV680_PWDN_GPIO, 0);	// deassert
			rc = 0;
			ov680_dbg("PWDN deassert\n");
		} else {
			ov680_err("Invalid power down op: %d\n",
				  reg_conf_tbl->reg_data);
			rc = -1;
		}
	} else if (reg_conf_tbl->reg_addr == OV680_IO_OPS_NOP) {
		// Used for empty commands in the FWI reader file.
		ov680_verb("NOP\n");
		rc = 0;
	} else if (reg_conf_tbl->reg_addr == OV680_IO_OPS_SLEEP) {
		ov680_verb("Sleep %dms\n",reg_conf_tbl->reg_data);
		msleep(reg_conf_tbl->reg_data);
		rc = 0;
	} else if (reg_conf_tbl->reg_addr >= OV680_IO_OPS_UPARM_WR
		   && reg_conf_tbl->reg_addr < OV680_IO_OPS_UPARM_RD) {
		// For sections that are highly configurable (i.e. ROI, flash, etc)
		// grab configuration params from the uparam array.
		uint32_t index = reg_conf_tbl->reg_addr - OV680_IO_OPS_UPARM_WR;

		ov680_dbg("Write parameter %d -> %x (addr).\n", index,
			  reg_conf_tbl->reg_data);

		if (index >= OV680_UPARM_MAX) {
			ov680_err("Write Parameter %d does not exist.\n", index);
			rc = -1;
		} else {
			ov680_verb("I2C Write: %04x -> %02X.\n",
				   reg_conf_tbl->reg_data,
				   ov680_props.params[index]);
			rc = ov680_props.s_ctrl.sensor_i2c_client->
			    i2c_func_tbl->i2c_write(client,
						    reg_conf_tbl->reg_data,
						    ov680_props.params[index],
						    MSM_CAMERA_I2C_BYTE_DATA);

			rc = 0;
		}
	} else if (reg_conf_tbl->reg_addr >= OV680_IO_OPS_UPARM_RD
		   && reg_conf_tbl->reg_addr < OV680_IO_OPS_UPARM_TOP) {

		uint32_t index = reg_conf_tbl->reg_addr - OV680_IO_OPS_UPARM_RD;
		uint16_t data;
		ov680_dbg("Read parameter %d -> %x (addr).\n", index,
			  reg_conf_tbl->reg_data);

		if (index >= OV680_UPARM_READ_MAX) {
			ov680_err("Read Parameter %d does not exist.\n", index);
			rc = -1;
		} else {
			ov680_verb("I2C Read: %04x \n",
				   reg_conf_tbl->reg_data);
			rc = ov680_props.s_ctrl.sensor_i2c_client->
			    i2c_func_tbl->i2c_read(client,
						    reg_conf_tbl->reg_data,
						    &data,
						    MSM_CAMERA_I2C_BYTE_DATA);

			ov680_props.read_params[index] = data& 0xff;

			ov680_err("Read Register %04x=%02x", reg_conf_tbl->reg_data, ov680_props.read_params[index]);

			rc = 0;
		}

	} else {
		ov680_err("Invalid i2c op command: %d\n",
			  reg_conf_tbl->reg_addr);
		rc = -1;
	}

	return rc;

}

static struct v4l2_subdev_info ov680_subdev_info[] = {
	{
	 .code = V4L2_MBUS_FMT_SBGGR10_1X10,
//              .code   = V4L2_MBUS_FMT_YUYV8_2X8,
	 .colorspace = V4L2_COLORSPACE_JPEG,
	 .fmt = 1,
	 .order = 0,
	 },
};

static const struct i2c_device_id ov680_i2c_id[] = {
	{OV680_SENSOR_NAME, (kernel_ulong_t) & ov680_props.s_ctrl},
	{}
};

#if 0
static struct i2c_driver ov680_i2c_driver = {
	.id_table = ov680_i2c_id,
	.probe = ov680_sensor_standby_i2c_probe,
	.driver = {
		   .name = OV680_SENSOR_NAME,
		   },
};
#endif

static struct msm_camera_i2c_client ov680_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
	.write_op_cb = ov680_i2c_cb,
};

static const struct of_device_id ov680_dt_match[] = {
	{.compatible = "lab126,ov680",.data = &ov680_props.s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, ov680_dt_match);

static struct platform_driver ov680_platform_driver = {
	.driver = {
		   .name = "qcom,ov680",
		   .owner = THIS_MODULE,
		   .of_match_table = ov680_dt_match,
		   },
};

static irqreturn_t ov680_cmd_rdy_isr(int dummy, void *priv)
{
	ov680_props.irq.count_since_last++;
	if (ov680_props.irq.state == OV680_ISR_COMMAND_PENDING) {

		if (ov680_props.irq.count <= 1) {
#ifdef TPLMS_INST
			TPLMS_CAM(TID_CAM_OV680_COMMAND_IRQ_SCHEDULE);
#endif
			ov680_verb("Command IRQ: Trigger completed.\n");
			ov680_props.irq.state = OV680_ISR_COMMAND_COMPLETE;
			ov680_props.irq.count = 0;
		} else {
			ov680_verb
			    ("Command IRQ: Triggered. Waiting for %d more.\n",
			     ov680_props.irq.count);
			ov680_props.irq.count--;
		}
	} else {
		ov680_err("Command IRQ: Invalid state.\n");
		ov680_props.irq.state = OV680_ISR_UNKNOWN;
	}
	wake_up(&ov680_props.irq.wq);
	return IRQ_HANDLED;
}

static int ov680_setup_isr(struct platform_device *pdev)
{
	struct device_node *of_node = pdev->dev.of_node;
	int rc = 0;
	uint32_t count = 0, irq_gpio_index, irq_gpio = OV680_CMD_RDY_GPIO;

	of_get_property(of_node, "lab126,gpio-cmd-rdy-irq-num", &count);

	count /= sizeof(uint32_t);

	pr_err("Count is =%d", count);

	if (count != 1) {
		ov680_err
		    ("%s could not read property lab126,gpio-cmd-rdy-irq-num\n",
		     __func__);
		/* fallback, use hardcoded GPIO number from ov680_config.h file */
		goto skip_dts;

	}

	rc = of_property_read_u32_array(of_node, "lab126,gpio-cmd-rdy-irq-num",
					&irq_gpio_index, count);
	if (rc) {
		ov680_err
		    ("%s could not read array lab126,gpio-cmd-rdy-irq-num\n",
		     __func__);
		/* fallback, use hardcoded GPIO number from ov680_config.h file */
		goto skip_dts;
	}

	ov680_dbg("irq gpio index[%u]\n", irq_gpio_index);

	irq_gpio = of_get_gpio(of_node, irq_gpio_index);

	ov680_dbg("irq at gpio[%u]\n", irq_gpio);

 skip_dts:
	ov680_dbg("Enabling command complete IRQ.\n");
	irq_num = gpio_to_irq(irq_gpio);
	ov680_dbg("irq_num[%d]", irq_num);

	return rc;
}

static int32_t ov680_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(ov680_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	if (rc) {
		ov680_err("%s : msm_sensor_platform_probe failed rc=%d",
			  __func__, rc);
		goto end;
	}
	rc = ov680_setup_isr(pdev);
	if (rc) {
		ov680_err("%s : ov680_setup_isr failed rc=%d", __func__, rc);
		goto end;
	}

 end:
	return rc;
}

/* Flash related structures */
int ov680_flash_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {
    return ov680_flash_probe(&ov680_props.flash_ctrl, client);
}

static struct of_device_id ov680_flash_match_table[] = {
	{ .compatible = "maxim,max8834"},
	{ },
};

static const struct i2c_device_id ov680_flash_id[] = {
    { "max8834", 0 },
    {},
};

static struct i2c_driver ov680_flash_i2c_driver = {
    .driver = {
        .owner  = THIS_MODULE,
        .name   = "max8834",
        .of_match_table = ov680_flash_match_table,
     },
    .probe         = ov680_flash_i2c_probe,
    .id_table      = ov680_flash_id,
};

static int __init ov680_init_module(void)
{
	int32_t rc = 0;
	pr_err("%s:%d\n", __func__, __LINE__);

	board_rev = ursa_board_revision();
	init_waitqueue_head(&ov680_props.irq.wq);
	spin_lock_init(&ov680_props.mdm.lock);
	spin_lock_init(&ov680_props.icm.lock);

	if(board_rev == URSA_REVISION_P0_5) {
		ov680_props.s_ctrl.power_setting_array.power_setting = ov680_power_setting_p0_5;
		ov680_props.s_ctrl.power_setting_array.size = ARRAY_SIZE(ov680_power_setting_p0_5);
	}
	//ov680_fwi_init();
	rc = platform_driver_probe(&ov680_platform_driver, ov680_platform_probe);
	if (!rc)
	{
		rc = i2c_add_driver(&ov680_flash_i2c_driver);
		if (rc != 0)
			pr_err("Failed to register ov680_flash driver: %d\n", rc);
    }
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return rc;
//	return i2c_add_driver(&ov680_i2c_driver);
}

static void __exit ov680_exit_module(void)
{
	pr_err("%s:%d\n", __func__, __LINE__);
	if (ov680_props.s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&ov680_props.s_ctrl);
		platform_driver_unregister(&ov680_platform_driver);
	}
	//else
	//	i2c_del_driver(&ov680_i2c_driver);
	return;
}

void print_reg(struct msm_camera_i2c_reg_conf *reg, int size)
{
	int i;
	for (i = 0; i < size; i++) {
		pr_err("addr=%2x data=%2x", reg[i].reg_addr, reg[i].reg_data);

	}

}

int32_t ov680_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
			    void __user * argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	long rc = 0;
	int32_t i = 0;
	struct timespec t1,t0;
	ov680_dbg("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
	     s_ctrl->sensordata->sensor_name, cdata->cfgtype);
	switch (cdata->cfgtype) {
	case CFG_GET_SENSOR_INFO:
		CDBG("OV680 GET SENSOR INFO\n");
		memcpy(cdata->cfg.sensor_info.sensor_name,
		       s_ctrl->sensordata->sensor_name,
		       sizeof(cdata->cfg.sensor_info.sensor_name));
		cdata->cfg.sensor_info.session_id =
		    s_ctrl->sensordata->sensor_info->session_id;
		for (i = 0; i < SUB_MODULE_MAX; i++)
			cdata->cfg.sensor_info.subdev_id[i] =
			    s_ctrl->sensordata->sensor_info->subdev_id[i];
		CDBG("%s:%d sensor name %s\n", __func__, __LINE__,
		     cdata->cfg.sensor_info.sensor_name);
		CDBG("%s:%d session id %d\n", __func__, __LINE__,
		     cdata->cfg.sensor_info.session_id);
		for (i = 0; i < SUB_MODULE_MAX; i++)
			CDBG("%s:%d subdev_id[%d] %d\n", __func__, __LINE__, i,
			     cdata->cfg.sensor_info.subdev_id[i]);

		break;
	case CFG_SET_INIT_SETTING:
		break;
	case CFG_SET_RESOLUTION:
		ov680_dbg("OV680 SENSOR SET RESO\n");
#if 0
		rc = s_ctrl->sensor_i2c_client->
		    i2c_func_tbl->i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
						     ov680_720p_settings,
						     ARRAY_SIZE
						     (ov680_720p_settings),
						     MSM_CAMERA_I2C_BYTE_DATA);
#endif
		break;
	case CFG_SET_STOP_STREAM:
		HV_PWR(1, 6, 0, 0);   /* ACOS_MOD_ONELINE */
		ov680_dbg("SENSOR STOP STREAM\n");
		ov680_fwi_run(OV680_FWI_IDLE, 0);
		ov680_flash_enable(&ov680_props.flash_ctrl, 0);
		ov680_dbg("SENSOR STOP STREAM-end\n");

		/* Update Streaming State */
		ov680_props.state = OV680_STATE_IDLE;
		ov680_props.frame_cntr =0;
		break;

	case CFG_SET_START_STREAM:
		HV_PWR(1, 6, 0, 1);   /* ACOS_MOD_ONELINE */
		ov680_dbg("SENSOR START STREAM\n");
		ov680_fwi_run_startup();
		if (ov680_props.flash_state == OV680_FLASH_CONFIG_RUN) {
		    ov680_flash_enable(&ov680_props.flash_ctrl, 1);
		}
		/* Update Streaming State */
		ov680_props.state = OV680_STATE_STREAMING;
		metadata_update_list_init(&ov680_props.mdm);
		irled_current_update_list_init(&ov680_props.icm);
		ov680_props.frame_cntr =0;
		ov680_dbg("SENSOR START STREAM-end\n");
		break;

	case CFG_GET_SENSOR_INIT_PARAMS:
		CDBG("OV680 SENSOR INIT PARAMS\n");
		cdata->cfg.sensor_init_params =
		    *s_ctrl->sensordata->sensor_init_params;
		CDBG("%s:%d init params mode %d pos %d mount %d\n", __func__,
		     __LINE__,
		     cdata->cfg.sensor_init_params.modes_supported,
		     cdata->cfg.sensor_init_params.position,
		     cdata->cfg.sensor_init_params.sensor_mount_angle);
		break;

	case CFG_SET_SLAVE_INFO:{
			struct msm_camera_sensor_slave_info sensor_slave_info;
			struct msm_sensor_power_setting_array
			*power_setting_array;
			int slave_index = 0;
			CDBG("OV680 SET SLAVE INFO\n");
			if (copy_from_user(&sensor_slave_info,
					   (void *)cdata->cfg.setting,
					   sizeof(struct
						  msm_camera_sensor_slave_info)))
			{
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			/* Update sensor slave address */
			if (sensor_slave_info.slave_addr) {
				s_ctrl->sensor_i2c_client->cci_client->sid =
				    sensor_slave_info.slave_addr >> 1;
			}

			/* Update sensor address type */
			s_ctrl->sensor_i2c_client->addr_type =
			    sensor_slave_info.addr_type;

			/* Update power up / down sequence */
			s_ctrl->power_setting_array =
			    sensor_slave_info.power_setting_array;
			power_setting_array = &s_ctrl->power_setting_array;
			power_setting_array->power_setting =
			    kzalloc(power_setting_array->size *
				    sizeof(struct msm_sensor_power_setting),
				    GFP_KERNEL);
			if (!power_setting_array->power_setting) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -ENOMEM;
				break;
			}
			if (copy_from_user(power_setting_array->power_setting,
					   (void *)
					   sensor_slave_info.
					   power_setting_array.power_setting,
					   power_setting_array->size *
					   sizeof(struct
						  msm_sensor_power_setting))) {
				kfree(power_setting_array->power_setting);
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			s_ctrl->free_power_setting = true;
			CDBG("%s sensor id %x\n", __func__,
			     sensor_slave_info.slave_addr);
			CDBG("%s sensor addr type %d\n", __func__,
			     sensor_slave_info.addr_type);
			CDBG("%s sensor reg %x\n", __func__,
			     sensor_slave_info.sensor_id_info.
			     sensor_id_reg_addr);
			CDBG("%s sensor id %x\n", __func__,
			     sensor_slave_info.sensor_id_info.sensor_id);
			for (slave_index = 0;
			     slave_index < power_setting_array->size;
			     slave_index++) {
				CDBG("%s i %d power setting %d %d %ld %d\n",
				     __func__, slave_index,
				     power_setting_array->
				     power_setting[slave_index].seq_type,
				     power_setting_array->
				     power_setting[slave_index].seq_val,
				     power_setting_array->
				     power_setting[slave_index].config_val,
				     power_setting_array->
				     power_setting[slave_index].delay);
			}
			kfree(power_setting_array->power_setting);
			break;
		}
	case CFG_WRITE_I2C_ARRAY:{
			break;
		}
	case CFG_WRITE_I2C_SEQ_ARRAY:{
			break;
		}

	case CFG_POWER_UP:
		ov680_dbg("power_up\n");
		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_DOWN) {
			ov680_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}

		if ((rc = request_irq(irq_num, ov680_cmd_rdy_isr,
				      IRQF_TRIGGER_FALLING, "OV680-CMD-RDY-ISR",
				      NULL)) != 0) {
			ov680_err("IRQ err - no IRQ (reason %ld)\n", rc);
		} else {
			ov680_dbg("IRQ OK.\n");
			if (s_ctrl->func_tbl->sensor_power_up){
				getnstimeofday(&t0);
				rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
				if (rc < 0) {
					ov680_err("%s:%d failed rc %ld\n", __func__,
					__LINE__, rc);
					break;
				}
				rc = ov680_flash_init(&ov680_props.flash_ctrl);
				if (rc < 0) {
					ov680_err("%s:%d failed rc %ld\n", __func__,
					__LINE__, rc);
					break;
				}
				getnstimeofday(&t1);
				ov680_perf("POWER_UP took - %d us", tdiff(&t1, &t0));
				s_ctrl->sensor_state = MSM_SENSOR_POWER_UP;
				ov680_err("%s:%d sensor state %d\n", __func__, __LINE__,
				s_ctrl->sensor_state);
			} else {
				rc = -EFAULT;
			}
		}
		break;

	case CFG_POWER_DOWN:
		ov680_dbg("power_down\n");
		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_UP) {
			ov680_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}
		if (s_ctrl->func_tbl->sensor_power_down){
			rc = s_ctrl->func_tbl->sensor_power_down(s_ctrl);
			if (rc < 0) {
				ov680_err("%s:%d failed rc %ld\n", __func__,
					__LINE__, rc);
				break;
			}
			rc = ov680_flash_release(&ov680_props.flash_ctrl);
			if (rc < 0) {
				ov680_err("%s:%d failed rc %ld\n", __func__,
					__LINE__, rc);
				break;
			}
			free_irq(irq_num, NULL);
			s_ctrl->sensor_state = MSM_SENSOR_POWER_DOWN;
			ov680_err("%s:%d sensor state %d\n", __func__, __LINE__,
				s_ctrl->sensor_state);
		} else {
			rc = -EFAULT;
		}
		break;

	case CFG_SET_STOP_STREAM_SETTING:{
			break;
		}
	default:
		rc = -EFAULT;
		break;
	}


	return rc;
}

static int ov680_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
        int rc = 0;
	uint16_t chipid = 0xff;
	ov680_info("%s line %d", __func__, __LINE__);

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			s_ctrl->sensordata->slave_info->sensor_id_reg_addr,
			&chipid, MSM_CAMERA_I2C_WORD_DATA);

	ov680_dbg("%s chipid=%x\n", __func__, chipid);

	if (chipid == 0) {
                ov680_err("%s: probe i2c read returned [%d]\n", __func__, rc);
                return -EINVAL;
        }

        return rc;
}

void metadata_update_list_init(struct metadata_update_mgr* mdm)
{
	int i;

	for(i=0;i<OV680_MAX_METADATA_UPDATES;i++)
	{
		memset(&mdm->list[i],0,sizeof(struct ov680_metadata_update));
		atomic_set(&mdm->list[i].updated, 1);
	}
	atomic_set(&ov680_props.exp_set, 0);
	atomic_set(&ov680_props.gain_set, 0);

}

int dispatch_metadata_update(struct metadata_update_mgr* mdm, struct ov680_metadata_update* update)
{
	int i, rc;
	rc = 0;

	ov680_dbg("%s", __func__);
	spin_lock(&mdm->lock);

	for(i=0;i<OV680_MAX_METADATA_UPDATES;i++)
	{
		if(atomic_read(&mdm->list[i].updated))
		{
			memcpy(&mdm->list[i], update, sizeof(struct ov680_metadata_update));
			atomic_set(&mdm->list[i].updated, 0);
			break;
		}

	}


	if (i == OV680_MAX_METADATA_UPDATES) {
		ov680_err("Metadata list eviction attempt fc[%llu]", ov680_props.frame_cntr);

		/* lets get our local metadata object updated with whatever is
		 * latest so that we free up stale entries to make room for newer entries. The
                 * local metadata object gets copied to the frame metadata callback in the
                 * ov680_sensor_index_metadata_hook call hence we do not lose this metadata */
		get_delayed_metadata_updates(&ov680_props.s_ctrl, &ov680_props.metadata);

		/* now try adding the new update */
		for(i=0;i<OV680_MAX_METADATA_UPDATES;i++)
		{
			if(atomic_read(&mdm->list[i].updated))
			{
				memcpy(&mdm->list[i], update, sizeof(struct ov680_metadata_update));
				atomic_set(&mdm->list[i].updated, 0);
				break;
			}

		}
		/* should never get here */
		if (unlikely(i == OV680_MAX_METADATA_UPDATES)) {
			ov680_err("Metadata list full fc[%llu]", ov680_props.frame_cntr);
		}


		rc = -1;
	}
	spin_unlock(&mdm->lock);

	return rc;
}

void irled_current_update_list_init(struct irled_current_update_mgr* icm)
{
	int i;

	for(i=0;i<OV680_MAX_CURRENT_BUFFER;i++)
	{
		memset(&icm->list[i],0,sizeof(struct ov680_metadata_update));
		icm->list[i].updated = 1;
	}
	atomic_set(&ov680_props.strobe_span_set, 0);
}

int dispatch_irled_current_update(struct irled_current_update_mgr* icm, struct ov680_irled_current *update)
{
	int i, rc = 0;
	int old_index = 0;

	spin_lock(&icm->lock);

	for(i=0;i<OV680_MAX_CURRENT_BUFFER;i++)
	{
		// Use the newest request in case someone tries to update current multiple times per frame
		if(icm->list[i].updated || update->frame_num == icm->list[i].frame_num)
		{
			memcpy(&icm->list[i], update, sizeof(struct ov680_irled_current));
			icm->list[i].updated = 0;
			break;
		} else {
			if(icm->list[old_index].frame_num > icm->list[i].frame_num) {
				old_index = i;
			}
		}
	}

	if (i == OV680_MAX_CURRENT_BUFFER) {
		// Buffer was full, let's overwrite the oldest entry.  By design, this entry would be too outdated to apply anyway.
		memcpy(&icm->list[old_index], update, sizeof(struct ov680_irled_current));
		icm->list[old_index].updated = 0;
	}

	spin_unlock(&icm->lock);

	return rc;
}

void ov680_static_exp_gain_update(struct msm_sensor_ctrl_t *sctrl)
{
	struct ov680_driver_priv_t*  priv = container_of(sctrl, struct ov680_driver_priv_t, s_ctrl);
        uint32_t id, param_exp_base, param_gain_base;
        uint8_t sl, sr;
	uint16_t temp_hi, temp_lo;
	id = ov680_props.current_sensors;
	sl = id & 0xF;
	sr = (id>>4) & 0xF;
        param_exp_base = OV680_UPARM_SENSOR_0_EXPOSURE_LOW;
        param_gain_base = OV680_UPARM_SENSOR_0_GAIN;

	if(priv->aec_state == OV680_AUTO_EXPOSURE_DISABLE)
	{

		ov680_dbg("%s @ fc=%llu", __func__, priv->frame_cntr);
		if(priv->streaming_mode == OV680_MODE_2X1)
		{
			temp_lo = priv->params[param_exp_base + (sl -1)*2];
			temp_hi = priv->params[param_exp_base + (sl -1)*2 + 1];
			priv->metadata.exposure1 =  OV6710_LINES_TO_EXPOSURE_US((temp_lo>>4 | temp_hi<<4));
			priv->metadata.gain1 = priv->params[param_gain_base + sl -1];

			temp_lo = priv->params[param_exp_base + (sr -1)*2];
			temp_hi = priv->params[param_exp_base + (sr -1)*2 + 1];
			priv->metadata.exposure2 =  OV6710_LINES_TO_EXPOSURE_US((temp_lo>>4 | temp_hi<<4));
			priv->metadata.gain2 = priv->params[param_gain_base + sr -1];
		}
		if(priv->streaming_mode == OV680_MODE_1X1)
		{
			temp_lo = priv->params[param_exp_base + (sl -1)*2];
			temp_hi = priv->params[param_exp_base + (sl)*2];
			priv->metadata.exposure1 =  OV6710_LINES_TO_EXPOSURE_US((temp_lo>>4 | temp_hi<<4));
			priv->metadata.gain1 = priv->params[param_gain_base + sl -1];

		}
	}
}

static int ov680_read_exp_gain_registers(struct msm_sensor_ctrl_t *sctrl)
{
	struct ov680_driver_priv_t*  priv = container_of(sctrl, struct ov680_driver_priv_t, s_ctrl);
	uint16_t temp_hi, temp_lo;
        uint16_t aec1_exp, aec2_exp, aec1_gain, aec2_gain, aec_unit;
        uint8_t sl, sr;
        uint32_t id, param_exp_base, param_gain_base;
	struct msm_camera_i2c_fn_t *i2c_func_tbl;
	struct ov680_metadata_update meta_update;
	memset(&meta_update, 0, sizeof(meta_update));

	i2c_func_tbl = sctrl->sensor_i2c_client->i2c_func_tbl;
	id = ov680_props.current_sensors;
        sl = id & 0xF;
        sr = (id>>4) & 0xF;
        param_exp_base = OV680_UPARM_SENSOR_0_EXPOSURE_LOW;
        param_gain_base = OV680_UPARM_SENSOR_0_GAIN;


	if(priv->aec_state == OV680_AUTO_EXPOSURE_ENABLE){

		i2c_func_tbl->i2c_read(sctrl->sensor_i2c_client, 0x6e01, &temp_hi,
                        MSM_CAMERA_I2C_BYTE_DATA);
                i2c_func_tbl->i2c_read(sctrl->sensor_i2c_client, 0x6e02, &temp_lo,
                        MSM_CAMERA_I2C_BYTE_DATA);
                aec1_exp = OV6710_LINES_TO_EXPOSURE_US((temp_lo>>4 | temp_hi<<4));

                i2c_func_tbl->i2c_read(sctrl->sensor_i2c_client, 0x6e81, &temp_hi,
                        MSM_CAMERA_I2C_BYTE_DATA);
                i2c_func_tbl->i2c_read(sctrl->sensor_i2c_client, 0x6e82, &temp_lo,
                        MSM_CAMERA_I2C_BYTE_DATA);
                aec2_exp = OV6710_LINES_TO_EXPOSURE_US((temp_lo>>4 | temp_hi<<4));

                i2c_func_tbl->i2c_read(sctrl->sensor_i2c_client, 0x6e0A, &temp_hi,
                        MSM_CAMERA_I2C_BYTE_DATA);
                i2c_func_tbl->i2c_read(sctrl->sensor_i2c_client, 0x6e0B, &temp_lo,
                        MSM_CAMERA_I2C_BYTE_DATA);
                aec1_gain = temp_lo | temp_hi<<8;

                i2c_func_tbl->i2c_read(sctrl->sensor_i2c_client, 0x6e8A, &temp_hi,
                        MSM_CAMERA_I2C_BYTE_DATA);
                i2c_func_tbl->i2c_read(sctrl->sensor_i2c_client, 0x6e8B, &temp_lo,
                        MSM_CAMERA_I2C_BYTE_DATA);
                aec2_gain = temp_lo | temp_hi<<8;

		if(priv->streaming_mode == OV680_MODE_2X1)
                {
                        aec_unit = get_aec_index(sl);
                        if(aec_unit == 0){
                                meta_update.exposure1 = aec1_exp;
                                meta_update.gain1 = aec1_gain;
                        }
                        else if(aec_unit == 1){
                                meta_update.exposure1 = aec2_exp;
                                meta_update.gain1 = aec2_gain;
                        }
                        aec_unit = get_aec_index(sr);
                        if(aec_unit == 0){
                                meta_update.exposure2 = aec1_exp;
                                meta_update.gain2 = aec1_gain;
                        }
                        else if(aec_unit == 1){
                                meta_update.exposure2 = aec2_exp;
                                meta_update.gain2 = aec2_gain;
                        }
                }
                else if(priv->streaming_mode == OV680_MODE_1X1)
                {
                        aec_unit = get_aec_index(sl);
                        if(aec_unit == 0){
                                meta_update.exposure1 = aec1_exp;
                                meta_update.gain1 = aec1_gain;
                        }
                        else if(aec_unit == 1){
                                meta_update.exposure1 = aec2_exp;
                                meta_update.gain1 = aec2_gain;
                        }
                }


	} else if(priv->aec_state == OV680_AUTO_EXPOSURE_DISABLE){

		if(ov680_atomic_test(&priv->exp_set) || ov680_atomic_test(&priv->gain_set))
		{
			ov680_dbg("%s exp/gain update fc=%llu", __func__, priv->frame_cntr);
			if(priv->streaming_mode == OV680_MODE_2X1)
			{
				temp_lo = priv->params[param_exp_base + (sl -1)*2];
				temp_hi = priv->params[param_exp_base + (sl -1)*2 + 1];
				meta_update.exposure1 =  OV6710_LINES_TO_EXPOSURE_US((temp_lo>>4 | temp_hi<<4));
				meta_update.gain1 = priv->params[param_gain_base + sl -1];
				temp_lo = priv->params[param_exp_base + (sr -1)*2];
				temp_hi = priv->params[param_exp_base + (sr -1)*2 + 1];
				meta_update.exposure2 =  OV6710_LINES_TO_EXPOSURE_US((temp_lo>>4 | temp_hi<<4));
				meta_update.gain2 = priv->params[param_gain_base + sr -1];
			}
			if(priv->streaming_mode == OV680_MODE_1X1)
			{
				temp_lo = priv->params[param_exp_base + (sl -1)*2];
				temp_hi = priv->params[param_exp_base + (sl)*2];
				meta_update.exposure1 =  OV6710_LINES_TO_EXPOSURE_US((temp_lo>>4 | temp_hi<<4));
				meta_update.gain1 = priv->params[param_gain_base + sl -1];
			}
		}
		else
			ov680_dbg("%s exp_gain no update fc=%llu", __func__, priv->frame_cntr);

        }
	if (ov680_props.switching_state == OV680_SWITCHING_IDLE)
	{
		if(priv->aec_state == OV680_AUTO_EXPOSURE_ENABLE || ov680_atomic_test(&priv->exp_set) || ov680_atomic_test(&priv->gain_set))
		{
			/* dispatch metadata update event */
			ov680_dbg("%s dispatch delayed exp/gain update", __func__);
			meta_update.frame_num = priv->frame_cntr + 2; /* exp gain metadata to be updated 2 frames later */
			meta_update.flags = EXP_META_UPDATE | GAIN_META_UPDATE;
			dispatch_metadata_update(&priv->mdm, &meta_update);
		}
	}

	ov680_dbg("AEC/AGC %s", (priv->aec_state == OV680_AUTO_EXPOSURE_DISABLE)?"disable":"enable");
        ov680_dbg("exposure1[%d]", meta_update.exposure1);
        ov680_dbg("exposure2[%d]", meta_update.exposure2);
        ov680_dbg("gain1[%d]", meta_update.gain1);
        ov680_dbg("gain2[%d]", meta_update.gain2);

return 0;
}

static int ov680_sensor_frame_event_hook(struct msm_sensor_ctrl_t *s_ctrl, struct ispif_frame_event* fe)
{
	struct ov680_driver_priv_t*  priv = container_of(s_ctrl, struct ov680_driver_priv_t, s_ctrl);
	int rc_exp=0, rc_gain=0, rc_flash=0;
	static int rc_strobe=0;
	ov680_dbg("FRAME EVENT [%s]", fe->event?"EOF":"SOF");
	if(fe->vfe_idx == VFE1 && fe->interface == RDI0) {
		if(fe->event == FRAME_EVENT_SOF) {
			priv->frame_cntr++;/* update frame cntr */
			/* read exp and gain registers*/
			ov680_read_exp_gain_registers(s_ctrl);
			if(ov680_props.switching_state == OV680_SWITCHING_IDLE)
			{
				rc_exp = ov680_set_exposure_on_sof(priv);
				rc_gain = ov680_set_gain_on_sof(priv);
				if(rc_exp | rc_gain)
				{
					ov680_err("Failed to set exp or gain on SOF rc_exp=%d rc_gain=%d", rc_exp, rc_gain);
				}

				rc_strobe = ov680_set_strobe_span_on_sof(priv);
				if(rc_strobe)
				{
					ov680_err("Failed to set Flash Strobe  rc=%d", rc_strobe);
				}
			}
		} else if(fe->event == FRAME_EVENT_EOF) {
			if(!rc_strobe) { /* we do not want to set flash current if strobe setting failed */
				if(ov680_props.switching_state == OV680_SWITCHING_IDLE)
				{
					rc_flash = ov680_set_flash_current_on_eof(priv);
					if(rc_flash)
					{
						ov680_err("Failed to set Flash Current  rc=%d", rc_flash);
					}
				}
			}
		}
	}
	return 0;
}

void get_delayed_metadata_updates(struct msm_sensor_ctrl_t *sctrl,struct msm_sensor_metadata *sensor_metadata)
{
	struct ov680_driver_priv_t*  priv = container_of(sctrl, struct ov680_driver_priv_t, s_ctrl);
	int i;
	uint32_t flags;
	uint64_t last_frame_num[3];

	ov680_dbg("%s", __func__);

	memset(last_frame_num, 0, sizeof(last_frame_num));

	for(i=0;i<OV680_MAX_METADATA_UPDATES;i++)
	{
		if(!atomic_read(&priv->mdm.list[i].updated))
		{
			/* check if this applies to current or previous frame */
			if( (priv->mdm.list[i].frame_num <= priv->frame_cntr) )
			{
				flags = priv->mdm.list[i].flags;
				if((flags & (EXP_META_UPDATE|GAIN_META_UPDATE)) && (priv->mdm.list[i].frame_num > last_frame_num[0]))
				{
					sensor_metadata->exposure1 = priv->mdm.list[i].exposure1;
					sensor_metadata->exposure2 = priv->mdm.list[i].exposure2;
					sensor_metadata->gain1 = priv->mdm.list[i].gain1;
					sensor_metadata->gain2 = priv->mdm.list[i].gain2;
					/* also update priv->metadata */
					priv->metadata.exposure1 = priv->mdm.list[i].exposure1;
					priv->metadata.exposure2 = priv->mdm.list[i].exposure2;
					priv->metadata.gain1 = priv->mdm.list[i].gain1;
					priv->metadata.gain2 = priv->mdm.list[i].gain2;
					last_frame_num[0] = priv->mdm.list[i].frame_num;
				}
				else if((flags & FPS_META_UPDATE) && (priv->mdm.list[i].frame_num > last_frame_num[1]))
				{
					sensor_metadata->frame_rate = priv->mdm.list[i].fps;
					/* also update priv->metadata */
					priv->metadata.frame_rate = priv->mdm.list[i].fps;
					last_frame_num[1] = priv->mdm.list[i].frame_num;

				}
				else if((flags & FLASH_SPAN_META_UPDATE) && (priv->mdm.list[i].frame_num > last_frame_num[2]))
				{
					sensor_metadata->flash_duration = priv->mdm.list[i].flash_duration;
					/* also update priv->metadata */
					priv->metadata.flash_duration = priv->mdm.list[i].flash_duration;
					last_frame_num[2] = priv->mdm.list[i].frame_num;
				}
				atomic_set(&priv->mdm.list[i].updated, 1);

				 /* clear flags */
				 priv->mdm.list[i].flags = 0;

			}


		}
	}

}

static int ov680_sensor_index_metadata_hook(struct msm_sensor_ctrl_t *s_ctrl, struct msm_sensor_metadata *sensor_metadata)
{
	ov680_dbg("%s", __func__);
	if(!sensor_metadata){
		ov680_err("%s - Null Arg", __func__);
		return -EINVAL;
	}

        if ( ov680_props.state == OV680_STATE_STREAMING) {
		memcpy(sensor_metadata, &ov680_props.metadata, sizeof(struct msm_sensor_metadata));

		get_delayed_metadata_updates(s_ctrl, sensor_metadata);
        	ov680_dbg("exposure1[%d]", sensor_metadata->exposure1);
		ov680_dbg("exposure2[%d]", sensor_metadata->exposure2);
		ov680_dbg("gain1[%d]", sensor_metadata->gain1);
		ov680_dbg("gain2[%d]", sensor_metadata->gain2);

	}
        else
		memset(sensor_metadata,0xf, sizeof(struct msm_sensor_metadata));

	return 0;
}

int ov680_request_firmware(struct msm_sensor_ctrl_t *s_ctrl)
{
		return request_firmware_nowait(THIS_MODULE, FW_ACTION_NOHOTPLUG, OV680FW_NAME, &ov680_props.s_ctrl.pdev->dev,
		GFP_KERNEL, (void*)&ov680_props, ov680_fw_loader_callback);

}

int ov680_load_firmware(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc=0;
	struct sensorb_cfg_data cdata;

	ov680_info("%s", __func__);

	if(ov680_props.fw_state == OV680_FW_LOADED){
		ov680_info("FW already loaded, returning");
		return 0;
	}

	cdata.cfgtype = CFG_POWER_UP;
	if (s_ctrl->func_tbl->sensor_config)
		rc = s_ctrl->func_tbl->sensor_config(s_ctrl, (void __user *)&cdata);
	if(rc){
		ov680_err("%s - power up failed%d !!", __func__, __LINE__);
		return -EINVAL;
	}

	rc = ov680_fwi_run_force(OV680_FWI_FIRMWARE,
				 OV680_FIRMWARE_INIT);

	if (rc) {
		ov680_err("%s FW loading failed rc=%d %d", __func__, rc, __LINE__);
		ov680_props.fw_state = OV680_FW_MISSING;
		ov680_props.state = OV680_STATE_OFF;
		rc = 0;
		//return rc;
	} else {
		ov680_info("%s Change state to OV680_FW_LOADED %d", __func__, __LINE__);
		ov680_props.fw_state = OV680_FW_LOADED;
		ov680_props.state = OV680_STATE_IDLE;
	}

	rc = ov680_fwi_run_force(OV680_FWI_IDLE, 0);
	if(rc)
	{
		ov680_err("%s Changing to Idle mode Failed %d", __func__, __LINE__);
	}

	cdata.cfgtype = CFG_POWER_DOWN;
	if (s_ctrl->func_tbl->sensor_config)
		rc = s_ctrl->func_tbl->sensor_config(s_ctrl, (void __user *)&cdata);
	if(rc){
		ov680_err("%s - power down failed %d!!", __func__, __LINE__);
		return -EINVAL;
	}

	return rc;
}

long ov680_sensor_subdev_ioctl(struct v4l2_subdev *sd,
			       unsigned int cmd, void *arg)
{
	struct msm_sensor_ctrl_t *s_ctrl;
	void __user *argp = (void __user *)arg;
	struct v4l2_input *input;
	int rc=-ENOIOCTLCMD;

	s_ctrl = container_of(container_of(sd, struct msm_sd_subdev, sd),
				struct msm_sensor_ctrl_t, msm_sd);

	ov680_verb("%s", __func__);

	if (!s_ctrl) {
		ov680_err("%s s_ctrl NULL\n", __func__);
		return -EBADF;
	}

	/* called within tasklet so cannot acquire mutex */
	if (cmd == VIDIOC_SENSOR_GET_METADATA)
		return ov680_sensor_index_metadata_hook(s_ctrl,(struct msm_sensor_metadata*)arg);

	/* since all ioctl's modify the driver state variables, lets call them
	 * with mutex acquired */
	mutex_lock(s_ctrl->msm_sensor_mutex);
	switch (cmd) {
	case VIDIOC_OV680_SENSOR_SET_CTRL:
		rc = ov680_sensor_v4l2_s_ctrl(sd, argp);
		break;
	case VIDIOC_MSM_SENSOR_CFG:
		rc = s_ctrl->func_tbl->sensor_config(s_ctrl, argp);
		break;
	case VIDIOC_S_INPUT:
		rc = ov680_sensor_idx_set(s_ctrl, *(unsigned int *)argp);
		break;
	case VIDIOC_G_INPUT:
		input = (struct v4l2_input *)argp;
		input->index = ov680_props.current_sensors;
		rc = 0;
		break;
	case VIDIOC_OV680_SENSOR_SET_ROI:
		rc = ov680_set_roi((struct ov680_roi *)arg);
		break;
	case VIDIOC_OV680_SENSOR_SET_EXPOSURE:
		rc = ov680_set_exposure((struct ov680_exposure *)arg);
		break;
	case VIDIOC_OV680_SENSOR_SET_GAIN:
		rc = ov680_set_gain((struct ov680_gain *)arg);
		break;
	case VIDIOC_OV680_SENSOR_SET_IRLED_CONFIG:
		rc = ov680_set_irled_config((struct ov680_irled_config *)arg);
		break;
	case VIDIOC_OV680_SENSOR_GET_IRLED_MAX_CURRENT:
		rc = ov680_flash_get_max_current(&ov680_props.flash_ctrl, (struct ov680_irled_max_current_req *)arg);
		break;
	case VIDIOC_OV680_SENSOR_LOAD_FIRMWARE:
		rc = ov680_load_firmware(s_ctrl);
		break;
	case VIDIOC_OV680_SENSOR_REQUEST_FIRMWARE:
		rc = ov680_request_firmware(s_ctrl);
		break;
	case VIDIOC_SENSOR_FRAME_EVENT_HOOK:
		ov680_sensor_frame_event_hook(s_ctrl, (struct ispif_frame_event*)arg);
		rc = 0;
		break;
	default:
		rc = msm_sensor_subdev_ioctl(sd, cmd, arg);
	}
	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return ov680_rc(rc);
}

static struct v4l2_subdev_core_ops ov680_subdev_core_ops = {
	.s_ctrl = ov680_sensor_v4l2_s_ctrl,
	.ioctl = ov680_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_ops ov680_subdev_ops = {
	.core = &ov680_subdev_core_ops,
};

static struct msm_sensor_fn_t ov680_sensor_func_tbl = {
	.sensor_config = ov680_sensor_config,
	.sensor_power_up = ov680_sensor_standby_power_up,
	.sensor_power_down = ov680_sensor_standby_power_down,
	.sensor_match_id = ov680_sensor_match_id,
};

struct ov680_driver_priv_t ov680_props = {
	.state = OV680_STATE_OFF,
	.streaming_mode = OV680_MODE_1X1,
	.aec_state = OV680_AUTO_EXPOSURE_ENABLE,
	.switching_state = OV680_SWITCHING_IDLE,
	.frame_rate = OV680_DEFAULT_FPS,
	.exp_set.counter = 0,
	.gain_set.counter = 0,
	.strobe_span_set.counter = 0,
	.metadata = {
		.AEC_on = 0b1,
		.AGC_on = 0b1,
		.frame_rate = OV680_DEFAULT_FPS,
	},
	.fw_state = OV680_FW_MISSING,
	.fw_mutex = __MUTEX_INITIALIZER(ov680_props.fw_mutex),
	.flash_state = OV680_FLASH_CONFIG_NONE,
	.current_sensors = 4,
	.ctrls = ov680_v4l2_ctrl_info,
	.ctrls_size = ARRAY_SIZE(ov680_v4l2_ctrl_info),
	.params = {
		   0x64, 0,	/* Flash Span defaults */
		   0xf, 0xf, 0xf, 0xf,	/* AEC/AGC ROI Defaults */
		   0xf, 0xf, 0xf, 0xf,
		   0xf, 0xf, 0xf, 0xf,
		   0xf, 0xf, 0xf, 0xf,
		   0x20, 0x1B,	/* Manual exposure defaults */
		   0x20, 0x1B,
		   0x20, 0x1B,
		   0x20, 0x1B,
		   16, 16, 16, 16,	/* Manual gain defaults */
		   0x00, /* Sensor Sel */
		   0x00, /* Frame Mode Full */
		   0x0a, /* AEC Target */
		   },
	.read_params = {
			0xff, 0xff,0xff,0xff,/* sensor id registers */
			0xff, /* scratch register */
		  },
	.flash_ctrl = {
		      .led_desc = {
				      {1, 1, 200},
				      {1, 0, 200},
				      {0, 0, 200},
				      {0, 1, 200},
				  },
		   },
	.s_ctrl = {
		.sensor_i2c_client = &ov680_sensor_i2c_client,
		.power_setting_array.power_setting = ov680_power_setting,
		.power_setting_array.size = ARRAY_SIZE(ov680_power_setting),
		.msm_sensor_mutex = &ov680_mut,
		.sensor_v4l2_subdev_info = ov680_subdev_info,
		.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov680_subdev_info),
		.func_tbl = &ov680_sensor_func_tbl,
		.sensor_v4l2_subdev_ops = &ov680_subdev_ops,
		},
};

late_initcall_sync(ov680_init_module);
module_exit(ov680_exit_module);
MODULE_DESCRIPTION("ov680");
MODULE_LICENSE("GPL v2");
