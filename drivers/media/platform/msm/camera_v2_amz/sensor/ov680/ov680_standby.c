/*
 * ov680_standby.c
 *
 * Copyright (c) 2012-2013 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * PROPRIETARY/CONFIDENTIAL
 *
 * Use is subject to license terms.
 */

#include "msm_sensor.h"
#include "ov680.h"
//#include "ov680_debugfs.h"
#include "ov680_config.h"
#include "msm.h"
#include "msm_camera_io_util.h"
#include "msm_camera_i2c.h"
#include <linux/gpio.h>
#include "msm_cci.h"

#ifndef OV680_SUPPORT_STANDBY
static int ov680_skip_fw_load = 0;	// Used for probe
#else
struct msm_sensor_power_setting ov680_standby_power_setting[] = {
	{
	 .seq_type = SENSOR_VREG,
	 .seq_val = CAM_VANA,
	 .config_val = 0,
	 .delay = 10,
	 },
	{
	 .seq_type = SENSOR_CLK,
	 .seq_val = SENSOR_CAM_MCLK,
	 .config_val = 19200000,
	 .delay = 5,
	 },
};

struct gpio standby_gpios[] = {
	{
	 .gpio = OV680_PWDN_GPIO,
	 .label = "OV680_PWDN",
	 .flags = GPIOF_OUT_INIT_HIGH,
	 },
	{
	 .gpio = OV680_MCLK_GPIO,
	 .label = "CAM_4CC_MCLK1",
	 .flags = GPIOF_DIR_IN,
	 },
};

int ov680_power_up(struct msm_sensor_ctrl_t *s_ctrl, struct msm_sensor_power_setting* power_setting_table, int size)
{
	int index, rc = 0;
	struct msm_sensor_power_setting* power_setting;
	struct msm_camera_sensor_board_info *data = s_ctrl->sensordata;
	ov680_info("%s line %d", __func__, __LINE__);

	/* request GPIO table */
	rc = msm_camera_request_gpio_table(standby_gpios, sizeof(standby_gpios)/sizeof(standby_gpios[0]), 1);
	if(rc){
		ov680_err("%s msm_camera_request_gpio_table failed rc=%d %d", __func__, rc, __LINE__);
		return rc;
	}

	for (index = 0; index < size; index++) {
		ov680_dbg("%s index %d\n", __func__, index);
		power_setting = &power_setting_table[index];
		ov680_dbg("%s type %d\n", __func__, power_setting->seq_type);
		switch (power_setting->seq_type) {
		case SENSOR_CLK:
			if (power_setting->seq_val >= s_ctrl->clk_info_size) {
				pr_err("%s clk index %d >= max %d\n", __func__,
					power_setting->seq_val,
					s_ctrl->clk_info_size);
				goto ov680_power_up_failed;
			}
			if (power_setting->config_val)
				s_ctrl->clk_info[power_setting->seq_val].
					clk_rate = power_setting->config_val;

			ov680_dbg("%s Enable MCLK %d", __func__, __LINE__);
			rc = msm_cam_clk_enable(s_ctrl->dev,
				&s_ctrl->clk_info[0],
				(struct clk **)&power_setting->data[0],
				s_ctrl->clk_info_size,
				1);
			if (rc < 0) {
				pr_err("%s: clk enable failed\n",
					__func__);
				goto ov680_power_up_failed;
			}
			break;
		case SENSOR_VREG:
			ov680_dbg("%s Enable VREG %d", __func__, __LINE__);
			rc = msm_camera_config_single_vreg(s_ctrl->dev,
				&data->cam_vreg[power_setting->seq_val],
				(struct regulator **)&power_setting->data[0],
				1);
			if (rc < 0) {
				pr_err("%s: vreg enable failed\n",
					__func__);
				goto ov680_power_up_failed;
			}

			break;
		case SENSOR_GPIO:
		case SENSOR_I2C_MUX:
		default:
			ov680_info("Not handled");
			break;
		}
		if (power_setting->delay > 20) {
			msleep(power_setting->delay);
		} else if (power_setting->delay) {
			usleep_range(power_setting->delay * 1000,
				(power_setting->delay * 1000) + 1000);
		}
	}
	/* call CCI INIT */
	ov680_dbg("call CCI INIT");
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util(
		s_ctrl->sensor_i2c_client, MSM_CCI_INIT);
	if (rc < 0) {
		pr_err("%s cci_init failed\n", __func__);
		goto ov680_power_up_failed;
	}
	return 0;

ov680_power_up_failed:
	ov680_err("%s:%d failed\n", __func__, __LINE__);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util(
                        s_ctrl->sensor_i2c_client, MSM_CCI_RELEASE);

	for (index--; index >= 0; index--) {
                ov680_err("%s index %d\n", __func__, index);
		power_setting = &power_setting_table[index];
                ov680_err("%s type %d\n", __func__, power_setting->seq_type);
                switch (power_setting->seq_type) {
                case SENSOR_CLK:
                        msm_cam_clk_enable(s_ctrl->dev,
                                &s_ctrl->clk_info[0],
                                (struct clk **)&power_setting->data[0],
                                s_ctrl->clk_info_size,
                                0);
                        break;
		case SENSOR_VREG:
                        msm_camera_config_single_vreg(s_ctrl->dev,
                                &data->cam_vreg[power_setting->seq_val],
                                (struct regulator **)&power_setting->data[0],
                                0);
			break;
		case SENSOR_GPIO:
		case SENSOR_I2C_MUX:
		default:
			ov680_info("Not handled");
			break;

		}
		if (power_setting->delay > 20) {
			msleep(power_setting->delay);
		} else if (power_setting->delay) {
			usleep_range(power_setting->delay * 1000,
				(power_setting->delay * 1000) + 1000);
		}
	}
	msm_camera_request_gpio_table(standby_gpios, sizeof(standby_gpios)/sizeof(standby_gpios[0]), 0);

return rc;
}

int ov680_power_down(struct msm_sensor_ctrl_t *s_ctrl, struct msm_sensor_power_setting* power_setting_table, int size)
{
	int index;
	struct msm_sensor_power_setting* power_setting;
	struct msm_camera_sensor_board_info *data = s_ctrl->sensordata;
	ov680_info("%s line %d", __func__, __LINE__);

	/* call CCI RELEASE */
	ov680_dbg("call CCI REL");
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util(
		s_ctrl->sensor_i2c_client, MSM_CCI_RELEASE);

	for (index = size-1; index >= 0; index--) {
		ov680_dbg("%s index %d\n", __func__, index);
		power_setting = &power_setting_table[index];
		ov680_dbg("%s type %d\n", __func__, power_setting->seq_type);
		switch (power_setting->seq_type) {
		case SENSOR_CLK:
			ov680_dbg("%s Disable MCLK %d", __func__, __LINE__);
			msm_cam_clk_enable(s_ctrl->dev,
				&s_ctrl->clk_info[0],
				(struct clk **)&power_setting->data[0],
				s_ctrl->clk_info_size,
				0);
			break;
		case SENSOR_VREG:
			ov680_dbg("%s Disable VREG %d", __func__, __LINE__);
			msm_camera_config_single_vreg(s_ctrl->dev,
				&data->cam_vreg[power_setting->seq_val],
				(struct regulator **)&power_setting->data[0],
				0);
			break;
		case SENSOR_GPIO:
		case SENSOR_I2C_MUX:
		default:
			ov680_info("Not handled");
			break;
		}
		if (power_setting->delay > 20) {
			msleep(power_setting->delay);
		} else if (power_setting->delay) {
			usleep_range(power_setting->delay * 1000,
				(power_setting->delay * 1000) + 1000);
		}
	}
	/* de-request GPIO table */
	msm_camera_request_gpio_table(standby_gpios, sizeof(standby_gpios)/sizeof(standby_gpios[0]), 0);

return 0;
}


#endif

int ov680_sensor_standby_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
#ifndef OV680_SUPPORT_STANDBY
	int rc = 0;
	ov680_info("%s line %d", __func__, __LINE__);
	msm_sensor_power_up(s_ctrl);
	if (rc < 0) {
		ov680_err("%s line %d power up failed", __func__, __LINE__);
		return rc;
	}
	if (!rc) {
		ov680_dbg("LOAD FW ++");
		rc = ov680_fwi_run_force(OV680_FWI_FIRMWARE,
					 OV680_FIRMWARE_INIT);
		ov680_dbg("LOAD FW --");
		if (rc < 0) {
			pr_err("FW load failed try next time rc=%d", rc);
			rc = 0;
		}
	}
	return rc;
#else
	int rc = 0;
	int  i, j;
	ov680_info("%s line %d", __func__, __LINE__);
	if (ov680_props.state == OV680_STATE_OFF) {
		ov680_dbg
		    ("%s Standby mode availability. Powering ON for the first time %d", __func__, __LINE__);
		/* Acquire PWDN gpio, it's not part of the DTS GPIO list */
		rc = msm_camera_request_gpio_table(standby_gpios, 1, 1);
		if(rc){
			ov680_err("%s msm_camera_request_gpio_table failed rc=%d %d", __func__, rc, __LINE__);
			msm_sensor_power_down(s_ctrl);
			return rc;
		}
		gpio_set_value(OV680_PWDN_GPIO, 0); // deassert
		rc = msm_sensor_power_up(s_ctrl);

		if (rc) {
			ov680_err("%s line %d stadby power up failed",
				  __func__, __LINE__);
			return rc;
		}
		/* copy clk and vreg information */
		for (i = 0; i < s_ctrl->power_setting_array.size; i++) {
			if (s_ctrl->power_setting_array.
			    power_setting[i].seq_type == SENSOR_VREG &&
			    s_ctrl->power_setting_array.
			    power_setting[i].seq_val == CAM_VANA){
				ov680_dbg("%s Found VANA %d", __func__, __LINE__);
				for (j = 0; j < 10; j++)
					ov680_standby_power_setting[0].data[j] =
					    s_ctrl->
					    power_setting_array.power_setting
					    [i].data[j];
			}
		}
		for (i = 0; i < s_ctrl->power_setting_array.size; i++) {
			if (s_ctrl->power_setting_array.
			    power_setting[i].seq_type == SENSOR_CLK){
				ov680_dbg("%s Found MCLK %d", __func__, __LINE__);
				for (j = 0; j < 10; j++)
					ov680_standby_power_setting[1].data[j] =
					    s_ctrl->
					    power_setting_array.power_setting
					    [i].data[j];
			}

		}
		/* swap to standby power script to be used in next power down */
		#if 0
		s_ctrl->power_setting_array.power_setting =
		    ov680_standby_power_setting;
		s_ctrl->power_setting_array.size =
		    ARRAY_SIZE(ov680_standby_power_setting);
		#endif

		#if 0
		/* Load the Firmware duirng first power on */
		ov680_dbg("%s Load the Firmware duirng first power on %d", __func__, __LINE__);
		rc = ov680_fwi_run_force(OV680_FWI_FIRMWARE,
					 OV680_FIRMWARE_INIT);
		if (rc) {
			ov680_err("%s FW loading failed rc=%d %d", __func__, rc, __LINE__);
			ov680_props.fw_state = OV680_FW_MISSING;
			ov680_props.state = OV680_STATE_OFF;
			rc = 0;
			//return rc;
		} else {
			ov680_dbg("%s Change state to OV680_FW_LOADED %d", __func__, __LINE__);
			ov680_props.fw_state = OV680_FW_LOADED;
			ov680_props.state = OV680_STATE_IDLE;
		}
		#endif

	} else if (ov680_props.state == OV680_STATE_STANDBY) {
		ov680_dbg("%s Resuming from standby %d", __func__, __LINE__);

		/* this tunrs on Rails and Clk using standby power up sequnce */
		rc = ov680_power_up(s_ctrl, ov680_standby_power_setting, ARRAY_SIZE(ov680_standby_power_setting));

		if (rc < 0) {
			ov680_err("%s Unable to power up rc=%d %d", __func__, rc, __LINE__);
			return rc;
		}
		if (ov680_props.fw_state == OV680_FW_MISSING) {
			ov680_dbg("%s Loading firmware %d", __func__, __LINE__);
			rc = ov680_fwi_run_force
			    (OV680_FWI_FIRMWARE, OV680_FIRMWARE_INIT);
			ov680_props.fw_state = OV680_FW_LOADED;

			if (rc) {
				ov680_err("%s FW loading failed rc=%d %d", __func__, rc, __LINE__);
				ov680_props.fw_state = OV680_FW_MISSING;
				ov680_props.state = OV680_STATE_OFF;
				return rc;
			} else {
				ov680_info("%s Loading firmware SUCCESS %d", __func__, __LINE__);
				ov680_props.fw_state = OV680_FW_LOADED;
				ov680_props.state = OV680_STATE_IDLE;
			}

		} else { /* FW Loaded */
			ov680_dbg("%s FW already loaded Begin resume %d", __func__, __LINE__);
			rc = ov680_fwi_run_force(OV680_FWI_FIRMWARE,
						 OV680_FIRMWARE_RESUME);

			if (rc) {
				ov680_info("%s FW resume failed rc=%d %d", __func__, rc, __LINE__);
			}

			ov680_dbg("%s Restoring old firmware %d", __func__, __LINE__);
			rc = ov680_fwi_run_force(OV680_FWI_FIRMWARE,
						 OV680_FIRMWARE_RESTORE);
			if (rc) {
				ov680_info
				    ("Failed to resume. Loading firmware, resetting.\n");
				{
					rc = ov680_fwi_run_force
					    (OV680_FWI_FIRMWARE,
					     OV680_FIRMWARE_INIT);
					ov680_props.fw_state = OV680_FW_LOADED;
					ov680_props.state = OV680_STATE_IDLE;
				}
			}
		}

	}
return rc;
#endif
}

int ov680_sensor_standby_power_down(struct
				    msm_sensor_ctrl_t
				    *s_ctrl)
{
#ifndef OV680_SUPPORT_STANDBY
	ov680_info("%s line %d", __func__, __LINE__);
	return msm_sensor_power_down(s_ctrl);
#else
	int rc = 0;
	ov680_info("%s line %d", __func__, __LINE__);
	if (ov680_props.state != OV680_STATE_IDLE) {
		ov680_err("%s Standby has bad state: %d %d, forcing to idle", __func__, ov680_props.state, __LINE__);
		ov680_fwi_run(OV680_FWI_IDLE, 0);
		// do it anyways? typically want shutdown procedures to always work.
	}

	if (ov680_props.fw_state == OV680_FW_MISSING) {
		ov680_dbg("%s firmware not loaded regular power_down %d", __func__, __LINE__);
		/* Release PWDN gpio, it's not part of the DTS GPIO list */
		msm_camera_request_gpio_table(standby_gpios, 1, 0);
		rc = msm_sensor_power_down(s_ctrl);
		goto end;
	}

	ov680_dbg("%s Entering standby %d", __func__, __LINE__);
	rc = ov680_fwi_run_force(OV680_FWI_FIRMWARE, OV680_FIRMWARE_STANDBY);

	if (rc) {
		ov680_err("%s standby enter failed %d", __func__, __LINE__);
	}

	rc = ov680_power_down(s_ctrl, ov680_standby_power_setting, ARRAY_SIZE(ov680_standby_power_setting));

	if (rc) {
		ov680_err("%s power down failed %d", __func__, __LINE__);
		goto end;

	}


	ov680_props.state = OV680_STATE_STANDBY;

end:
	return rc;
#endif
}

#if 0
/* this never gets called as we have a platfrom driver and not an i2c driver */
int ov680_sensor_standby_i2c_probe(struct i2c_client
				   *client, const struct
				   i2c_device_id *id)
{
#ifndef OV680_SUPPORT_STANDBY
	int rc;
	ov680_info("%s line %d", __func__, __LINE__);
	ov680_skip_fw_load = 1;
	rc = msm_sensor_i2c_probe(client, id);
	ov680_skip_fw_load = 0;
	return rc;
#else
	int rc;
	ov680_info("%s line %d", __func__, __LINE__);
	// Essentially use the original msm power commands to power on
	// On failure, power down MSM style too.
	ov680_props.fw_state = OV680_FW_MISSING;
	ov680_props.state = OV680_STATE_OFF;
	rc = msm_sensor_i2c_probe(client, id);
	if (rc) {
		// In the case of a camera subsystem failure during boot up (probe failed)
		// it is necessary to turn everything on just so it can all be turned off again!
		ov680_props.state = OV680_STATE_UNWIND;
		ov680_sensor_standby_power_up(&ov680_s_ctrl);
		msm_sensor_power_down(&ov680_s_ctrl);
		ov680_props.state = OV680_STATE_OFF;
		return rc;
	}

	rc = ov680_sensor_standby_power_up(&ov680_s_ctrl);

	if (rc) {
		ov680_err("Unable to load firmware - power on failed.\n");
		msm_sensor_power_down(&ov680_s_ctrl);
		ov680_props.state = OV680_STATE_OFF;
		return rc;
	}
	rc = ov680_fwi_run_force(OV680_FWI_FIRMWARE, OV680_FIRMWARE_INIT);
	ov680_props.fw_state = OV680_FW_LOADED;

	ov680_sensor_standby_power_down(&ov680_s_ctrl);

	return rc;
#endif
}
#endif
