/*
 * ov680_fw_mgr.c
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
#include "msm_sensor.h"
#include "msm_sensor.h"
#include "ov680.h"
#include "ov680_config.h"
#include "msm.h"
#include "msm_camera_i2c.h"
#include "ov680_firmware.h"
#ifdef TPLMS_INST
#include "ov680_prof.h"
#endif

#define OV680_SETTING_INVALID 0xFFFF

/**
 * Start up script (compiled).
 */
static struct {
	uint32_t startup_index;
	uint32_t enabled;
	uint32_t current_setting;
} ov680_fwi_ops[OV680_FWI_MAX] = {
	{0, 1, OV680_SETTING_INVALID},	//OV680_FWI_FIRMWARE,
	{0, 1, OV680_SETTING_INVALID},	//OV680_FWI_FRAME_RATE,
	{0, 1, OV680_SETTING_INVALID},	//OV680_FWI_ISP_2A,
	{0, 1, OV680_SETTING_INVALID},	//OV680_FWI_ISP_2A_TARGET,
	{0, 1, OV680_SETTING_INVALID},	//OV680_FWI_ISP_2A_ROI_CONFIG,
	{0, 1, OV680_SETTING_INVALID},	//OV680_FWI_SENSOR_SEL,
	{0, 1, OV680_SETTING_INVALID},	//OV680_FWI_IDLE,
	{0, 1, OV680_SETTING_INVALID},	//OV680_FWI_FLASH,
	{0, 1, OV680_SETTING_INVALID},	//OV680_FWI_FLASH_CONFIG,
	{0, 1, OV680_SETTING_INVALID},	//OV680_FWI_FRAME_MODE
#ifdef OV680_NO_ISP_CONFIG
	{1, 0, OV680_SETTING_INVALID},	//OV680_FWI_ISP_CONFIG - don't support ISP config
	{1, 0, OV680_SETTING_INVALID},	//OV680_FWI_LENC_CONFIG
	{1, 0, OV680_SETTING_INVALID},	//OV680_FWI_DPC_CONFIG
#else
	{1, 1, OV680_SETTING_INVALID},	//OV680_FWI_ISP_CONFIG
	{1, 1, OV680_SETTING_INVALID},	//OV680_FWI_LENC_CONFIG
	{1, 1, OV680_SETTING_INVALID},	//OV680_FWI_DPC_CONFIG
#endif
	{0, 1, OV680_SETTING_INVALID},	//OV680_FWI_EXPOSURE_CONFIG
	{0, 1, OV680_SETTING_INVALID},	//OV680_FWI_GAIN_CONFIG
};

uint32_t ov680_fwi_startup_seq[] = {
	OV680_FWI_FRAME_MODE,	//Determines whether to stream in Full res or Binning/Skipping modes
	OV680_FWI_FLASH_CONFIG,
#ifndef OV680_USE_FAST_SWITCH
	OV680_FWI_FRAME_RATE,
#endif
	OV680_FWI_ISP_2A_TARGET,
#ifndef OV680_FRAME_SW_WORKAROUND
	OV680_FWI_ISP_2A,
#endif
	OV680_FWI_ISP_2A_ROI_CONFIG,
	OV680_FWI_ISP_CONFIG,
	OV680_FWI_EXPOSURE_CONFIG,
	OV680_FWI_GAIN_CONFIG,
	OV680_FWI_SENSOR_SEL,	// swapped in the case of selecting 2x1
};

void* memcpy_update(void *dest, uint8_t **src, size_t n)
{
	void* rc;
	rc = memcpy(dest,(void*)*src,n);
	*src = *src + n;
	return rc;
}

/**
 * callback invoked when firmware is loaded from userspace
 */
void ov680_fw_loader_callback(const struct firmware *fw_img, void *context)
{
	struct ov680_fw_header header;
	struct ov680_section_header section;
	struct ov680_subsection_header sub;
	struct ov680_subsection *sub_section = NULL;
	uint8_t *data = (uint8_t*)fw_img->data;
	uint16_t size = fw_img->size;
	int i,j,k;
	struct ov680_driver_priv_t* priv = (struct ov680_driver_priv_t*)context;

	ov680_err("ov680 fw loader callback");

	/* Sanity check */
	if(!fw_img || !data || !size)
	{
		ov680_err("%s fw_img issue fw_img[%p] data[%p] size[%d], abort FW loading", __func__, fw_img, data, size);
		return;
	}

	/* Zero Initialize the FW sections */
	for(i=0; i < OV680_FWI_MAX; i++){
		memset(&priv->ov680_fw_image[i], 0, sizeof(struct ov680_fw_section));

	}
	/* Copy header*/
	memcpy_update(&header, &data, sizeof(header));

	ov680_info("Found FW img addr[%p] size[%d]", data, size);
	ov680_info("Found FW [%s released on %s] with %d number of sections", header.name, header.date, header.num_of_sections);


	/* Sections */
	for(i=0;i<header.num_of_sections; i++){
		memcpy_update(&section, &data, sizeof(section));
		ov680_info("Found Section id-%d[%s] with %d number of subsections", section.id, section.name, section.num_of_sub_sections);
		/* Copy name String */
		memcpy(priv->ov680_fw_image[section.id].name, section.name, FW_STRLEN);
		/* Allocate Subsections */
		if(priv->ov680_fw_image[section.id].subsections)
			kfree(priv->ov680_fw_image[section.id].subsections);
		priv->ov680_fw_image[section.id].subsections = kzalloc(sizeof(struct ov680_fw_subsection) * section.num_of_sub_sections, GFP_KERNEL);
		if(!priv->ov680_fw_image[section.id].subsections)
			ov680_err("Could not allocate %d bytes for ov680_fw_subsection", sizeof(struct ov680_fw_subsection));
		else
			priv->ov680_fw_image[section.id].enabled = 1;

		/* Subsections */
		for(j=0;j<section.num_of_sub_sections;j++){
			/* Copy Subsection Header */
			memcpy_update(&sub, &data, sizeof(sub));
			ov680_info("Found SubSection [%s] id[%d] of size %d", sub.name, j,sub.size);
			/* Copy name String */
			memcpy(priv->ov680_fw_image[section.id].subsections[j].name, sub.name, FW_STRLEN);
			priv->ov680_fw_image[section.id].subsections[j].size =  sub.size;

			kfree(sub_section);
			sub_section = kzalloc(sizeof(struct ov680_subsection)*sub.size, GFP_KERNEL);
			/* copy Subsection Data */
			memcpy_update(sub_section,&data,sizeof(struct ov680_subsection)*sub.size);

			if(priv->ov680_fw_image[section.id].subsections[j].conf)
				kfree(priv->ov680_fw_image[section.id].subsections[j].conf);
			priv->ov680_fw_image[section.id].subsections[j].conf = kzalloc(sizeof(struct msm_camera_i2c_reg_conf)*sub.size, GFP_KERNEL);
			if(!priv->ov680_fw_image[section.id].subsections[j].conf)
				ov680_err("Could not allocate %d bytes for msm_camera_i2c_reg_conf", sizeof(struct msm_camera_i2c_reg_conf)*sub.size);
			for(k=0;k<sub.size;k++){
				priv->ov680_fw_image[section.id].subsections[j].conf[k].reg_addr = sub_section[k].data[0];
				priv->ov680_fw_image[section.id].subsections[j].conf[k].reg_data = sub_section[k].data[1];
				priv->ov680_fw_image[section.id].subsections[j].conf[k].dt       = MSM_CAMERA_I2C_BYTE_DATA;
				priv->ov680_fw_image[section.id].subsections[j].conf[k].cmd_type = sub_section[k].data[2];
				priv->ov680_fw_image[section.id].subsections[j].conf[k].mask     = 0;
			}
			priv->ov680_fw_image[section.id].subsections[j].enabled = 1;
			//data = data + (sub.size * sizeof(struct ov680_subsection));
		}

	}
	ov680_fwi_init();

}
/**
 * Initializes the firmware image. Called during module startup to establish sane defaults.
 */
int ov680_fwi_init(void)
{

	ov680_fwi_select(OV680_FWI_FRAME_MODE, OV680_FULL_RES);
	ov680_fwi_select(OV680_FWI_SENSOR_SEL, 0);
	ov680_fwi_select(OV680_FWI_FRAME_RATE, OV680_FRAME_RATE_15);
	ov680_fwi_select(OV680_FWI_ISP_2A_TARGET, 0);
	ov680_fwi_select(OV680_FWI_ISP_2A, OV680_AUTO_EXPOSURE_ENABLE);
	ov680_fwi_select(OV680_FWI_FLASH, OV680_FLASH_DISABLE);
	ov680_fwi_select(OV680_FWI_FLASH_CONFIG, OV680_FLASH_CONFIG_NONE);
	ov680_fwi_select(OV680_FWI_EXPOSURE_CONFIG, OV680_EXPOSURE_CONFIG_NONE);
	ov680_fwi_select(OV680_FWI_GAIN_CONFIG, OV680_EXPOSURE_CONFIG_NONE);

	return 0;
}

/**
 * Resets settings to defaults.
 */
int ov680_fwi_reset(void)
{
	// todo - use fw defaults where possible
	int i;
	for (i = 0; i < ARRAY_SIZE(ov680_fwi_startup_seq); i++) {
		ov680_fwi_ops[i].current_setting = OV680_SETTING_INVALID;
	}
	return 0;
}

/**
 * Select a lut item to be included on the start up script.
 */
int ov680_fwi_select(enum ov680_fwi_section section, uint32_t index)
{
	int rc = 0;

	ov680_fwi_ops[section].startup_index = index;
	return rc;
}

/**
 * Select a lut item to be immediately executed.
 */
int ov680_fwi_run(enum ov680_fwi_section section, uint32_t index)
{

	if (ov680_props.state != OV680_STATE_STREAMING) {
		ov680_dbg("Ignoring run request: %d, %d (not streaming)",
			  section, index);
		return -1;
	}

	return ov680_fwi_run_force(section, index);
}

/**
 * Select a lut item to be immediately executed.
 */
int ov680_fwi_run_force(enum ov680_fwi_section section, uint32_t index)
{
	struct msm_camera_i2c_reg_conf *arr = NULL;
	uint16_t size = 0;
	int rc = 0;
	struct timespec t1,t0;

	mutex_lock(&ov680_props.fw_mutex);
	ov680_verb("Run section - section=%d, index=%d\n", section, index);

	if (!ov680_fwi_ops[section].enabled) {
		ov680_info("Section (%d) is disabled.\n",
			   section);
		goto end;
	}

	if(ov680_props.ov680_fw_image[section].enabled && ov680_props.ov680_fw_image[section].subsections[index].enabled)
	{
		arr = ov680_props.ov680_fw_image[section].subsections[index].conf;
		size = ov680_props.ov680_fw_image[section].subsections[index].size;
	}
	else
	{
		ov680_err("Section %d not enabled", section);
		rc = -EBUSY;
		goto end;
	}

	ov680_dbg("Begin %s(%d)[index=%d].\n",ov680_props.ov680_fw_image[section].name,section, index);

	getnstimeofday(&t0);
	rc = ov680_props.s_ctrl.sensor_i2c_client->
	    i2c_func_tbl->i2c_write_conf_tbl(ov680_props.s_ctrl.sensor_i2c_client,
					     arr, size,
					     MSM_CAMERA_I2C_BYTE_DATA);
	getnstimeofday(&t1);
	ov680_perf("Section[%s] Sub[%s] took - %d us", ov680_props.ov680_fw_image[section].name, ov680_props.ov680_fw_image[section].subsections[index].name, tdiff(&t1, &t0));

	if(rc){
		ov680_err("!!! cci write failed [%d], retry once again !!!", rc);
		rc = ov680_props.s_ctrl.sensor_i2c_client->
		    i2c_func_tbl->i2c_write_conf_tbl(ov680_props.s_ctrl.sensor_i2c_client,
						     arr, size,
						     MSM_CAMERA_I2C_BYTE_DATA);
	}
	ov680_dbg("End %s(%d)[index=%d].\n",ov680_props.ov680_fw_image[section].name,section, index);
end:
	mutex_unlock(&ov680_props.fw_mutex);
	return rc;
}

/**
 * Run the startup sequence.
 */
int ov680_fwi_run_startup(void)
{
	int i;
	ov680_info("Begin startup sequence.\n");
#ifdef TPLMS_INST
	TPLMS_CAM(TID_CAM_OV680_FWI_STARTUP_START);
#endif
	for (i = 0; i < ARRAY_SIZE(ov680_fwi_startup_seq); i++) {
		ov680_fwi_run_force(ov680_fwi_startup_seq[i],
				    ov680_fwi_ops[ov680_fwi_startup_seq
						  [i]].startup_index);
	}
#ifdef TPLMS_INST
	TPLMS_CAM(TID_CAM_OV680_FWI_STARTUP_END);
#endif
	ov680_info("Finished startup sequence.\n");
	return 0;		// caller really can't report failure anyways.
}

/**
 * Select a lut item to be immediately executed (regardless of state).
 */
int ov680_fwi_uparam_updated(enum ov680_fwi_section section)
{
	if (section >= 0 && section <= ARRAY_SIZE(ov680_fwi_ops))
		ov680_fwi_ops[section].current_setting = OV680_SETTING_INVALID;
	return 0;
}
