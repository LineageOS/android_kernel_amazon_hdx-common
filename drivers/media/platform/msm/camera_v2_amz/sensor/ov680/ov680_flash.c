/*
 * ov680_flash.c
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
#include "ov680.h"
#include "ov680_config.h"
#include "ov680_flash.h"
#include "ov680_flash_max8834.h"
#include "ov680_flash_ozl003.h"
#include <mach/socinfo.h>
#include <mach/board-detect.h>


static struct msm_camera_i2c_fn_t msm_sensor_cci_func_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = NULL,
	.i2c_write = msm_camera_cci_i2c_write,
	.i2c_write_table = NULL,
	.i2c_write_seq_table = NULL,
	.i2c_write_table_w_microdelay =
		NULL,
	.i2c_util = msm_sensor_cci_i2c_util,
	.i2c_write_conf_tbl = NULL,
};

static struct msm_camera_i2c_fn_t msm_sensor_qup_func_tbl = {
	.i2c_read = msm_camera_qup_i2c_read,
	.i2c_read_seq = NULL,
	.i2c_write = msm_camera_qup_i2c_write,
	.i2c_write_table = NULL,
	.i2c_write_seq_table = NULL,
	.i2c_write_table_w_microdelay = NULL,
};

static int ov680_flash_cci_init(struct msm_camera_i2c_client *client, int index) {
	int rc = 0;
	ov680_dbg("%s", __func__);
	client->cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!client->cci_client) {
		ov680_err("%s failed line %d\n", __func__, __LINE__);
		return -ENOMEM;
	}
	client->i2c_func_tbl = &msm_sensor_cci_func_tbl;

	client->addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	client->cci_client->retries = 3;
	client->cci_client->id_map = 0;
	client->cci_client->cci_i2c_master = index;
	client->cci_client->cci_subdev = msm_cci_get_subdev();

	rc = client->i2c_func_tbl->i2c_util( client, MSM_CCI_INIT );
	if (rc < 0) {
		ov680_err("%s cci_init failed\n", __func__);
		if (client->cci_client)
			kfree(client->cci_client);
	}
	return ov680_rc(rc);
}

static int ov680_flash_cci_release(struct msm_camera_i2c_client *client) {
	int rc = 0;
	ov680_dbg("%s", __func__);
	if(client->cci_client) {
		rc = client->i2c_func_tbl->i2c_util(client, MSM_CCI_RELEASE);
	}
	else
		ov680_err("%s cci_client is NULL", __func__);

	kfree(client->cci_client);
	client->cci_client = NULL;
	return ov680_rc(rc);
}

static inline int ov680_flash_write(struct msm_camera_i2c_client *client, uint8_t reg, uint8_t val) {
	int rc;
	rc = client->i2c_func_tbl->i2c_write(client, reg, val, MSM_CAMERA_I2C_BYTE_DATA);
	return rc;
}

static inline int ov680_flash_read(struct msm_camera_i2c_client *client, uint8_t reg, uint16_t *val) {
	int rc;
	rc = client->i2c_func_tbl->i2c_read(client, reg, val, MSM_CAMERA_I2C_BYTE_DATA);
	return rc;
}

/* MAX8834 specific functions */
static int max8834_flash_enable(struct ov680_flash_ctrl_t *fctrl, uint32_t enable) {

	uint8_t led_ctrl[NUM_FLASH_DEVICES] = {0,0};
	uint8_t boost_ctrl[NUM_FLASH_DEVICES] = {0,0};
	uint16_t i;
	int rc=0;
	ov680_dbg("%s", __func__);

	// Always run in quad mode
	if( enable )
	    gpio_set_value(QUAD_MODE_GPIO, 1);
	else
	    gpio_set_value(QUAD_MODE_GPIO, 0);

	for( i = 0; i < 4; i++ ) {
	    if( fctrl->led_desc[i].flash_current && enable) {
		uint8_t reg, flash_cur;
		ov680_dbg("Flash current: %d", fctrl->led_desc[i].flash_current );

		reg = (fctrl->led_desc[i].device_channel == 0) ? MAX8834_FLASH1_CUR : MAX8834_FLASH2_CUR;
		flash_cur = FLASH_CUR_UA_TO_REG(fctrl->led_desc[i].flash_current*1000) << FLASHx;
		rc = ov680_flash_write(&fctrl->sensor_i2c_client[fctrl->led_desc[i].device_number], reg, flash_cur);
		if(rc){
			ov680_err("%s Flash Current write failed line %d rc %d", __func__, __LINE__, rc);
			goto end;
		}

		led_ctrl[fctrl->led_desc[i].device_number] |= ((reg == MAX8834_FLASH1_CUR) ?
								   (FLED1_EN << FLASH_EN) :
								   (FLED2_EN << FLASH_EN));
		boost_ctrl[fctrl->led_desc[i].device_number] = (BOOST_CTRL_MV_TO_REG(MAX8834_BOOST_VOLTAGE) << BOOST_CTRL) |
								(BOOST_MODE_PROG << BOOST_MODE) |
								(0x1 << BOOST_EN);
	    }
	}

	// Turn on/off the appropriate channels
	for( i = 0; i < NUM_FLASH_DEVICES; i++ ) {
		led_ctrl[i] |= (LED_EN_CTRL << FLASH_EN);
		rc = ov680_flash_write(&fctrl->sensor_i2c_client[i], MAX8834_BOOST_CTRL, boost_ctrl[i]);
		if(rc){
			ov680_err("%s Flash Current write failed line %d rc %d", __func__, __LINE__, rc);
			goto end;
		}
		rc = ov680_flash_write(&fctrl->sensor_i2c_client[i], MAX8834_LED_CTRL, led_ctrl[i]);
		if(rc){
			ov680_err("%s Flash Current write failed line %d rc %d", __func__, __LINE__, rc);
			goto end;
		}
	}

end:
	return rc;
}

/* Initializes MAX8834 to sane defaults, outputs off */
static int max8834_flash_init(struct ov680_flash_ctrl_t *fctrl) {
	int rc = 0;
	int i;

	ov680_dbg("%s", __func__);

	// CCI0
	rc = ov680_flash_cci_init(&fctrl->sensor_i2c_client[0], 0);
	fctrl->sensor_i2c_client[0].cci_client->sid = MAX8834_DEVICE_ADDR;
	if (rc < 0) {
		ov680_err("Flash CCI0 init failed\n");
		return rc;
	}

	// P2 and above uses CC1, older revs use i2c-0
	if (ursa_board_revision() >= URSA_REVISION_P2) {
		// CCI1
		rc = ov680_flash_cci_init(&fctrl->sensor_i2c_client[1], 1);
		fctrl->sensor_i2c_client[1].cci_client->sid = MAX8834_DEVICE_ADDR;
		if (rc < 0) {
			ov680_err("Flash CCI1 init failed\n");
			ov680_flash_cci_release(&fctrl->sensor_i2c_client[0]);
			return rc;
		}
	}
	// Set sane defaults
	for( i = 0; i < NUM_FLASH_DEVICES; i++ ) {

		ov680_flash_write(&fctrl->sensor_i2c_client[i], MAX8834_TMR_DUR, (0x1 << TMR_MODE) | TMR_DUR_MS_TO_REG (800) << TMR_DUR);
		ov680_flash_write(&fctrl->sensor_i2c_client[i], MAX8834_NTC_CTRL, (0x1 << FLASH_TMR_CNTL));
	}

	return rc;
}

/* Cleans up the flash */
static int max8834_flash_release(struct ov680_flash_ctrl_t *fctrl) {
	int rc = 0;
	ov680_dbg("%s", __func__);

	rc = ov680_flash_cci_release(&fctrl->sensor_i2c_client[0]);
	if (ursa_board_revision() >= URSA_REVISION_P2) {
		rc = ov680_flash_cci_release(&fctrl->sensor_i2c_client[1]);
	}
	return rc;
}

static int max8834_flash_get_max_current(struct ov680_irled_max_current_req *max_current_req) {

	uint32_t period_us = (1000 * 1000 / max_current_req->frame_rate);
	uint32_t duty_cycle = (max_current_req->duration * 100) / period_us;
	ov680_dbg("%s", __func__);

	if(duty_cycle >= 50) {
		max_current_req->max_current = 50;
	} else {
		max_current_req->max_current = 100;
	}

	return 0;
}


/* OZL003 specific functions */
static int ozl003_flash_enable(struct ov680_flash_ctrl_t *fctrl, uint32_t enable) {

	uint8_t led_ctrl[NUM_FLASH_DEVICES] = {0,0};
	uint8_t isen1[NUM_FLASH_DEVICES] = {0,0};
	uint8_t isen2[NUM_FLASH_DEVICES] = {0,0};

	uint16_t i;
	int rc=0;

	ov680_dbg("%s", __func__);

	// Always run in quad mode
	if( enable )
	    gpio_set_value(QUAD_MODE_GPIO, 1);
	else
	    gpio_set_value(QUAD_MODE_GPIO, 0);

	for( i = 0; i < 4; i++ ) {
	    if( fctrl->led_desc[i].flash_current && enable) {
		uint8_t flash_cur;
		ov680_dbg("Flash current: %d", fctrl->led_desc[i].flash_current );

		flash_cur = OZ_FLASH_CUR_UA_TO_REG(fctrl->led_desc[i].flash_current*1000) << OZ_ISENx;
		if( fctrl->led_desc[i].device_channel == 0 ) {
		    isen1[fctrl->led_desc[i].device_number] = flash_cur;
		} else {
		    isen2[fctrl->led_desc[i].device_number] = flash_cur;
		}
		led_ctrl[fctrl->led_desc[i].device_number] = (OZ_BOOST_CTRL_UV_TO_REG(OZLOO3_BOOST_VOLTAGE) << OZ_VLED) |
							     (OZ_MODE_FIXED << OZ_MODE) |
							     (1 << OZ_IC_ENA);
	    }
	}

	// Turn on/off the appropriate channels
	for( i = 0; i < NUM_FLASH_DEVICES; i++ ) {
		fctrl->sensor_i2c_client[0].cci_client->sid = (i == 0) ? OZL003_DEVICE_ADDR0 : OZL003_DEVICE_ADDR1;
		rc = ov680_flash_write(&fctrl->sensor_i2c_client[0], OZL003_LED_CTRL, led_ctrl[i]);
		if(rc){
			ov680_err("%s Flash Current write failed line %d rc %d", __func__, __LINE__, rc);
			goto end;
		}
		rc = ov680_flash_write(&fctrl->sensor_i2c_client[0], OZL003_ISEN1, isen1[i]);
		if(rc){
			ov680_err("%s Flash Current write failed line %d rc %d", __func__, __LINE__, rc);
			goto end;
		}
		rc = ov680_flash_write(&fctrl->sensor_i2c_client[0], OZL003_ISEN2, isen2[i]);
		if(rc){
			ov680_err("%s Flash Current write failed line %d rc %d", __func__, __LINE__, rc);
			goto end;
		}
	}

end:
	return rc;
}

static int ozl003_flash_init(struct ov680_flash_ctrl_t *fctrl) {
	int rc = 0;
	int i;

	ov680_dbg("%s", __func__);

	// Only need CCI0 for OZL003
	rc = ov680_flash_cci_init(&fctrl->sensor_i2c_client[0], 0);

	if (rc < 0) {
		ov680_err("Flash CCI0 init failed\n");
		return rc;
	}

	// Set sane defaults
	for( i = 0; i < NUM_FLASH_DEVICES; i++ ) {
		fctrl->sensor_i2c_client[0].cci_client->sid = (i == 0) ? OZL003_DEVICE_ADDR0 : OZL003_DEVICE_ADDR1;
		ov680_flash_write(&fctrl->sensor_i2c_client[0], OZL003_PROT, OZL003_PROT_SETTINGS);
	}

	return rc;
}

static int ozl003_flash_release(struct ov680_flash_ctrl_t *fctrl) {
	int rc = 0;
	ov680_dbg("%s", __func__);

	rc = ov680_flash_cci_release(&fctrl->sensor_i2c_client[0]);
	return rc;
}

static int oz1003_flash_get_max_current(struct ov680_irled_max_current_req *max_current_req) {
	uint32_t period_us = (1000 * 1000 / max_current_req->frame_rate);
	uint32_t duty_cycle = (max_current_req->duration * 100) / period_us;
	ov680_dbg("%s", __func__);

	if(duty_cycle >= 50) {
		max_current_req->max_current = 100;
	} else if(duty_cycle >= 25) {
		max_current_req->max_current = 200;
	} else {
		max_current_req->max_current = 244;
	}

	return 0;
}


int ov680_flash_probe(struct ov680_flash_ctrl_t *fctrl, struct i2c_client *client)
{
	ov680_dbg("%s", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
	    pr_err("%s :need I2C_FUNC_I2C\n", __func__);
	    return  -ENODEV;
	}
	if (ursa_board_revision() < URSA_REVISION_P2) {
	    fctrl->sensor_i2c_client[1].client = client;
	    fctrl->sensor_i2c_client[1].client->addr = MAX8834_DEVICE_ADDR << 1;
	    fctrl->sensor_i2c_client[1].i2c_func_tbl = &msm_sensor_qup_func_tbl;
	    fctrl->sensor_i2c_client[1].addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	}
	return 0;
}

int ov680_flash_enable(struct ov680_flash_ctrl_t *fctrl, uint32_t enable) {
	ov680_dbg("%s", __func__);
    return fctrl->func_tbl.flash_enable(fctrl, enable);
}

int ov680_flash_init(struct ov680_flash_ctrl_t *fctrl) {
	ov680_dbg("%s", __func__);

	// Check for O2micro chip
	if( !fctrl->func_tbl.flash_init ) {
	    int rc = 0;
	    uint16_t val = 0;
	    ov680_flash_cci_init(&fctrl->sensor_i2c_client[0], 0);
	    fctrl->sensor_i2c_client[0].cci_client->sid = OZL003_DEVICE_ADDR0;
	    rc = ov680_flash_read(&fctrl->sensor_i2c_client[0], OZL003_STATUS, &val);
	    if( !rc && val == OZL003_DEVICE_ID) {
		fctrl->func_tbl.flash_init = ozl003_flash_init;
		fctrl->func_tbl.flash_release = ozl003_flash_release;
		fctrl->func_tbl.flash_enable = ozl003_flash_enable;
		fctrl->func_tbl.flash_get_max_current = oz1003_flash_get_max_current;
	    } else {
		// Assume the MAX8834 is present if 02micro isn't.
		fctrl->func_tbl.flash_init = max8834_flash_init;
		fctrl->func_tbl.flash_release = max8834_flash_release;
		fctrl->func_tbl.flash_enable = max8834_flash_enable;
		fctrl->func_tbl.flash_get_max_current = max8834_flash_get_max_current;
	    }
	    ov680_flash_cci_release(&fctrl->sensor_i2c_client[0]);
	}

	return fctrl->func_tbl.flash_init(fctrl);
}

int ov680_flash_release(struct ov680_flash_ctrl_t *fctrl) {
    ov680_dbg("%s", __func__);
    return fctrl->func_tbl.flash_release(fctrl);
}

int ov680_flash_get_max_current(struct ov680_flash_ctrl_t *fctrl, struct ov680_irled_max_current_req *max_current_req) {
    ov680_dbg("%s", __func__);
    return fctrl->func_tbl.flash_get_max_current(max_current_req);
}
