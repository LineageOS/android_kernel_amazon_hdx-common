/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
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

#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__


#include <linux/module.h>
#include "msm_sd.h"
#include "msm_actuator.h"
#include "msm_actuator_debugfs.h"
#include "msm_cci.h"
#include <linux/firmware.h>

DEFINE_MSM_MUTEX(msm_actuator_mutex);

//#define MSM_ACUTUATOR_DEBUG
#undef CDBG
#ifdef MSM_ACUTUATOR_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) pr_debug(fmt, ##args)
#endif

static struct msm_actuator msm_vcm_actuator_table;
static struct msm_actuator msm_piezo_actuator_table;
static struct msm_actuator msm_ois_actuator_table;

static struct i2c_driver msm_actuator_i2c_driver;
static struct msm_actuator *actuators[] = {
	&msm_vcm_actuator_table,
	&msm_piezo_actuator_table,
	&msm_ois_actuator_table,
};

#define EEPROM_SID 0x50
#define EEPROM_FACT_ADJ_START 0x0F20
#define EEPROM_MODULE_ID 0x0021

#define SCENE_REG_MAX 3
#define SENSOR_MODULE_MAX 3

#define RETURN_STATUS_CHECK(x) if ((rc = (x)) < 0) goto Error_Exit;

enum OpCode {
	OpCode_Prog = 0x80,
	OpCode_Coeff = 0x88,
	OpCode_Pheripheral_RW = 0x82,
	OpCode_Memory_RW = 0x84
};

enum sensor_module_build_rev {
	SENSOR_MODULE_REV_P1  = 0x10,
	SENSOR_MODULE_REV_P2  = 0x20,
	SENSOR_MODULE_REV_EVT = 0x30,
};

struct bu63164_module_fw_map_t {
	enum sensor_module_build_rev rev;
	const char *fw_name;
	const char *coeff_name;
};

enum bu63164_fact_adj_item_type_t {
	FactAdjItemType_Peripheral,
	FactAdjItemType_Memory
};

struct bu63164_fact_adj_item_t{
	uint8_t en; // Indicates whether or not this field should be used
	uint16_t data;
	uint16_t eerom_addr;
	uint8_t ctrl_addr;
	enum bu63164_fact_adj_item_type_t item_type;
};


enum bu63164_scene_param_t{

	SCENE_PARAM_NIGHT1,
	SCENE_PARAM_NIGHT2,
	SCENE_PARAM_NIGHT3,

	SCENE_PARAM_DAY1,
	SCENE_PARAM_DAY2,
	SCENE_PARAM_DAY3,

	SCENE_PARAM_SPORT1,
	SCENE_PARAM_SPORT2,
	SCENE_PARAM_SPORT3,
	SCENE_PARAM_TEST,

	SCENE_PARAM_MAX
};

struct bu63164_scene_parameter_info_t{
	uint8_t addr[SCENE_REG_MAX];
	uint16_t data[SCENE_PARAM_MAX][SCENE_REG_MAX];
};

enum bu63164_fact_adj_item_name_t{
	FactAdjName_CURDAT,
	FactAdjName_HALOFS_X,
	FactAdjName_HALOFS_Y,
	FactAdjName_HX_OFS,
	FactAdjName_HY_OFS,
	FactAdjName_PSTXOF,
	FactAdjName_PSTYOF,
	FactAdjName_GX_OFS,
	FactAdjName_GY_OFS,
	FactAdjName_KgxHG,
	FactAdjName_KgyHG,
	FactAdjName_KGXG,
	FactAdjName_KGYG,
	FactAdjName_SFTHAL_X,
	FactAdjName_SFTHAL_Y,
	FactAdjName_TMP_X,
	FactAdjName_TMP_Y,
	FactAdjName_KgxH0,
	FactAdjName_KgyH0,
	FactAdjName_MAX
};

enum memory_address_t {
	MemoryAddr_Kgx00 = 0x06,
	MemoryAddr_Kgx09 = 0x10,
	MemoryAddr_Kgx0A,
	MemoryAddr_Kgx0B,
	MemoryAddr_Kgx0C,
	MemoryAddr_wDgx02,
	MemoryAddr_Kgx0D = 0x18,
	MemoryAddr_Kgx0E,
	MemoryAddr_wDgx03 = 0x1C,
	MemoryAddr_Kgxdr = 0x36,
	MemoryAddr_Kgx13 = 0x3A,
	MemoryAddr_Kgx14,
	MemoryAddr_Kgx15,
	MemoryAddr_wDgx06 = 0x3E,
	MemoryAddr_X_LMT = 0x40,
	MemoryAddr_X_TGT = 0x43,
	MemoryAddr_DigGx = 0x55,
	MemoryAddr_DigGy,
	MemoryAddr_TMP_X = 0x6A,
	MemoryAddr_TMP_Y,
	MemoryAddr_EQCTL = 0x7F,
	MemoryAddr_Kgy00 = 0x86,
	MemoryAddr_wDgy02 = 0x94,
	MemoryAddr_wDgy03 = 0x9C,
	MemoryAddr_Kgy13 = 0xBA,
	MemoryAddr_Kgy14,
	MemoryAddr_Kgy15,
	MemoryAddr_wDgy06 = 0xBE,
	MemoryAddr_OIS_STS = 0xF7,
	MemoryAddr_X_MAX
};

static struct bu63164_module_fw_map_t ModuleFwMap[SENSOR_MODULE_MAX] = {
	{SENSOR_MODULE_REV_P1, "bu63164.prog", "bu63164.coeff"},
	{SENSOR_MODULE_REV_P2, "bu63164.prog", "bu63164.coeff"},
	{SENSOR_MODULE_REV_EVT, "bu63164_evt.prog", "bu63164_evt.coeff"},
};

static struct bu63164_fact_adj_item_t FactoryAdjData[FactAdjName_MAX] = {
	{1, 0, 0xF20, 0x30, FactAdjItemType_Peripheral},
	{1, 0, 0xF22, 0x31, FactAdjItemType_Peripheral},
	{1, 0, 0xF24, 0x32, FactAdjItemType_Peripheral},
	{1, 0, 0xF26, 0x1E, FactAdjItemType_Memory},
	{1, 0, 0xF28, 0x9E, FactAdjItemType_Memory},
	{1, 0, 0xF2A, 0x39, FactAdjItemType_Peripheral},
	{1, 0, 0xF2C, 0x3B, FactAdjItemType_Peripheral},
	{1, 0, 0xF2E, 0x06, FactAdjItemType_Memory},
	{1, 0, 0xF30, 0x86, FactAdjItemType_Memory},
	{1, 0, 0xF32, 0x46, FactAdjItemType_Memory},
	{1, 0, 0xF34, 0xC6, FactAdjItemType_Memory},
	{1, 0, 0xF36, 0x0F, FactAdjItemType_Memory},
	{1, 0, 0xF38, 0x8F, FactAdjItemType_Memory},
	{0, 0, 0xF3A, 0x31, FactAdjItemType_Peripheral},
	{0, 0, 0xF3C, 0x32, FactAdjItemType_Peripheral},
	{0, 0, 0xF3E, 0x6A, FactAdjItemType_Memory},
	{0, 0, 0xF40, 0x6B, FactAdjItemType_Memory},
	{1, 0, 0xF42, 0x70, FactAdjItemType_Memory},
	{1, 0, 0xF44, 0x72, FactAdjItemType_Memory},
};

static const struct bu63164_scene_parameter_info_t SceneParamInfo = {
	{MemoryAddr_Kgxdr, MemoryAddr_X_LMT, MemoryAddr_X_TGT,},
	{
		{0x7FFE, 0x2406, 0x0830},
		{0x7FFC, 0x2406, 0x0830},
		{0x7FFA, 0x2406, 0x0830},
		{0x7FFE, 0x2406, 0x1478},
		{0x7FFA, 0x2406, 0x1478},
		{0x7FF0, 0x2406, 0x1478},
		{0x7FFE, 0x2406, 0x1EB4},
		{0x7FF0, 0x2406, 0x1EB4},
		{0x7FE0, 0x2406, 0x1EB4},
		{0x7FF0, 0x7FFF, 0x7FFF},
	},
};

static int32_t msm_actuator_ois_init(struct msm_actuator_ctrl_t *a_ctrl)
{
	uint16_t BytesInTx = 0;
	uint16_t TotalBytes = 0;
	uint8_t * ptr = NULL;
	int32_t rc = 0;
	const struct firmware *fw = NULL;
	const char *fw_name_prog = NULL;
	const char *fw_name_coeff = NULL;
	struct device *dev = &(a_ctrl->pdev->dev);
	uint8_t i = 0;

	// Detect module revision - load different FWs for different modules
	uint16_t sensor_sid = a_ctrl->i2c_client.cci_client->sid;
	uint16_t build_rev;
	enum msm_camera_i2c_reg_addr_type sensor_addr_type = a_ctrl->i2c_client.addr_type;


	a_ctrl->i2c_client.cci_client->sid = EEPROM_SID;
	a_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;

	rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_read(
		&a_ctrl->i2c_client,
		EEPROM_MODULE_ID,
		&build_rev,
		MSM_CAMERA_I2C_BYTE_DATA);

	a_ctrl->i2c_client.cci_client->sid = sensor_sid;
	a_ctrl->i2c_client.addr_type = sensor_addr_type;

	if (rc) {
		pr_err("Failed to read EEPROM ID\n");
		goto msm_actuator_ois_init_Exit;
	}

	/* setup defaults */
	fw_name_prog = ModuleFwMap[0].fw_name;
	fw_name_coeff = ModuleFwMap[0].coeff_name;

	for( i = 0; i < SENSOR_MODULE_MAX; i++ ) {
		if( ModuleFwMap[i].rev == build_rev ) {
			fw_name_prog = ModuleFwMap[i].fw_name;
			fw_name_coeff = ModuleFwMap[i].coeff_name;
		}
	}

	rc = request_firmware(&fw, fw_name_prog, dev);
	if (rc) {
			dev_err(dev, "Failed to locate blob %s\n", fw_name_prog);
			goto msm_actuator_ois_init_Exit;
	}

	TotalBytes = fw->size;

	for (ptr = (uint8_t *)fw->data; TotalBytes; TotalBytes -= BytesInTx, ptr += BytesInTx)
	{
		BytesInTx = (TotalBytes > 10) ? 10 : TotalBytes;
		rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write_seq(
			&a_ctrl->i2c_client,
			OpCode_Prog, ptr,
			BytesInTx);
		if (rc < 0){
			goto msm_actuator_ois_releaseFW;
		}
	}

	release_firmware(fw);
	rc = request_firmware(&fw, fw_name_coeff, dev);
	if (rc) {
			dev_err(dev, "Failed to locate blob %s\n", fw_name_prog);
			goto msm_actuator_ois_init_Exit;
		}

	TotalBytes = fw->size;

	for (ptr = (uint8_t *)fw->data; TotalBytes; TotalBytes -= BytesInTx, ptr += BytesInTx)
	{
		BytesInTx = (TotalBytes >= 10) ? 10 : TotalBytes;
		rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write_seq(
			&a_ctrl->i2c_client,
			OpCode_Coeff, ptr,
			BytesInTx);
		if (rc < 0){
			goto msm_actuator_ois_releaseFW;
		}
	}

	msm_actuator_debugfs_init(a_ctrl);
	release_firmware(fw);
	return 0;

msm_actuator_ois_releaseFW:
	release_firmware(fw);
msm_actuator_ois_init_Exit:
	pr_err("Error in %s at line %d\n", __func__, __LINE__);
	return rc;

}

static int32_t msm_actuator_ois_memory_write(
	struct msm_camera_i2c_client *i2c_client,
	uint8_t mem_addr,
	uint16_t data)
{
	int32_t rc = 0;
	enum msm_camera_i2c_reg_addr_type sensor_addr_type = i2c_client->addr_type;

	i2c_client->addr_type = MSM_CAMERA_I2C_WORD_ADDR;

	data = ((data & 0xFF) << 8) | (data >> 8);
	rc = i2c_client->i2c_func_tbl->i2c_write(
	i2c_client, (OpCode_Memory_RW << 8) | mem_addr, data,
	MSM_CAMERA_I2C_WORD_DATA);

	i2c_client->addr_type = sensor_addr_type;

	return rc;
}

static int32_t msm_actuator_ois_memory_read(
	struct msm_camera_i2c_client *i2c_client,
	uint8_t mem_addr,
	uint16_t * data)
{
	int32_t rc = 0;
	enum msm_camera_i2c_reg_addr_type sensor_addr_type = i2c_client->addr_type;

	i2c_client->addr_type = MSM_CAMERA_I2C_WORD_ADDR;

	rc = i2c_client->i2c_func_tbl->i2c_read(
	i2c_client, (OpCode_Memory_RW << 8) | mem_addr, data,
	MSM_CAMERA_I2C_WORD_DATA);

	i2c_client->addr_type = sensor_addr_type;

	return rc;
}

static int32_t msm_actuator_ois_enable(
	struct msm_actuator_ctrl_t *a_ctrl, 
	uint8_t enable_ois)
{
	uint16_t ctrl_reg_val;
	uint16_t ctrl_reg_val1;
	int32_t rc = 0;
	enum msm_camera_i2c_reg_addr_type sensor_addr_type = a_ctrl->i2c_client.addr_type;


	rc = msm_actuator_ois_memory_read(&a_ctrl->i2c_client, MemoryAddr_EQCTL, &ctrl_reg_val);
	if (rc < 0)
		goto msm_actuator_ois_enable_Exit;

	if (enable_ois)
		ctrl_reg_val |= 0x0101;
	else
		ctrl_reg_val &= ~0x0101;

	rc = msm_actuator_ois_memory_write(&a_ctrl->i2c_client, MemoryAddr_EQCTL, ctrl_reg_val);
	if (rc < 0){
		goto msm_actuator_ois_enable_Exit;
	}

	a_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;

	rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_read(
		&a_ctrl->i2c_client, (OpCode_Memory_RW << 8) | 0x7F, &ctrl_reg_val1, MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0){
		goto msm_actuator_ois_enable_Exit;
	}
	a_ctrl->i2c_client.addr_type = sensor_addr_type;

msm_actuator_ois_enable_Exit:
	return rc;
}

static int32_t msm_actuator_ois_peripheral_write(
	struct msm_camera_i2c_client *i2c_client,
	uint8_t mem_addr,
	uint16_t data)
{
	int32_t rc = 0;
	enum msm_camera_i2c_reg_addr_type sensor_addr_type = i2c_client->addr_type;

	i2c_client->addr_type = MSM_CAMERA_I2C_WORD_ADDR;

	data = ((data & 0xFF) << 8) | (data >> 8);
	rc = i2c_client->i2c_func_tbl->i2c_write(
	i2c_client, (OpCode_Pheripheral_RW << 8) | mem_addr, data,
	MSM_CAMERA_I2C_WORD_DATA);

	i2c_client->addr_type = sensor_addr_type;

	return rc;
}

static int32_t msm_actuator_ois_fact_adj_apply(
	struct msm_actuator_ctrl_t *a_ctrl,
	struct bu63164_fact_adj_item_t *pFactoryAdjData,
	uint16_t total_entries)
{
	uint16_t i = 0;
	int32_t rc = 0;
	uint16_t sensor_sid = a_ctrl->i2c_client.cci_client->sid;
	enum msm_camera_i2c_reg_addr_type sensor_addr_type = a_ctrl->i2c_client.addr_type;

	for (i = 0; i < total_entries; i++)
	{

		a_ctrl->i2c_client.cci_client->sid = EEPROM_SID;
		a_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;

		RETURN_STATUS_CHECK(a_ctrl->i2c_client.i2c_func_tbl->i2c_read(
			&a_ctrl->i2c_client,
			pFactoryAdjData[i].eerom_addr,
			&pFactoryAdjData[i].data,
			MSM_CAMERA_I2C_WORD_DATA));

		a_ctrl->i2c_client.cci_client->sid = sensor_sid;
		a_ctrl->i2c_client.addr_type = sensor_addr_type;

		if( !pFactoryAdjData[i].en )
			continue;

		pFactoryAdjData[i].data = ((pFactoryAdjData[i].data & 0xFF) << 8) | (pFactoryAdjData[i].data >> 8);
		if (pFactoryAdjData[i].item_type == FactAdjItemType_Peripheral)
		{
			RETURN_STATUS_CHECK(
				msm_actuator_ois_peripheral_write(
				&a_ctrl->i2c_client, pFactoryAdjData[i].ctrl_addr, pFactoryAdjData[i].data));
		}
		else if (pFactoryAdjData[i].item_type == FactAdjItemType_Memory)
		{
			RETURN_STATUS_CHECK(
				msm_actuator_ois_memory_write(
				&a_ctrl->i2c_client, pFactoryAdjData[i].ctrl_addr, pFactoryAdjData[i].data));
		}
	}

	a_ctrl->i2c_client.cci_client->sid = sensor_sid;
	a_ctrl->i2c_client.addr_type = sensor_addr_type;

	// Special case for TMP_X, TMP_Y
	RETURN_STATUS_CHECK(
				msm_actuator_ois_memory_write(
				&a_ctrl->i2c_client, MemoryAddr_TMP_X, 0));
	RETURN_STATUS_CHECK(
				msm_actuator_ois_memory_write(
				&a_ctrl->i2c_client, MemoryAddr_TMP_Y, 0));

	return rc;

Error_Exit:
	pr_err("Error in %s at line %d rc = %d\n", __func__, __LINE__, rc);
	return rc;

}

static int32_t msm_actuator_piezo_set_default_focus(
	struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_move_params_t *move_params)
{
	int32_t rc = 0;
	CDBG("Enter\n");

	if (a_ctrl->curr_step_pos != 0) {
		a_ctrl->i2c_tbl_index = 0;
		a_ctrl->func_tbl->actuator_parse_i2c_params(a_ctrl,
			a_ctrl->initial_code, 0, 0);
		a_ctrl->func_tbl->actuator_parse_i2c_params(a_ctrl,
			a_ctrl->initial_code, 0, 0);
		rc = a_ctrl->i2c_client.i2c_func_tbl->
			i2c_write_table_w_microdelay(
			&a_ctrl->i2c_client, a_ctrl->i2c_reg_tbl,
			a_ctrl->i2c_tbl_index, a_ctrl->i2c_data_type);
		if (rc < 0) {
			pr_err("%s: i2c write error:%d\n",
				__func__, rc);
			return rc;
		}
		a_ctrl->i2c_tbl_index = 0;
		a_ctrl->curr_step_pos = 0;
	}
	CDBG("Exit\n");
	return rc;
}

static void msm_actuator_parse_i2c_params(struct msm_actuator_ctrl_t *a_ctrl,
	int16_t next_lens_position, uint32_t hw_params, uint16_t delay)
{
	struct msm_actuator_reg_params_t *write_arr = a_ctrl->reg_tbl;
	uint32_t hw_dword = hw_params;
	uint16_t i2c_byte1 = 0, i2c_byte2 = 0;
	uint16_t value = 0;
	uint32_t size = a_ctrl->reg_tbl_size, i = 0;
	struct msm_camera_i2c_reg_tbl *i2c_tbl = a_ctrl->i2c_reg_tbl;
	CDBG("Enter\n");
	for (i = 0; i < size; i++) {
		/* check that the index into i2c_tbl cannot grow larger that
		the allocated size of i2c_tbl */
		if ((a_ctrl->total_steps + 1) < (a_ctrl->i2c_tbl_index)) {
			break;
		}
		if (write_arr[i].reg_write_type == MSM_ACTUATOR_WRITE_DAC) {
			value = (next_lens_position <<
				write_arr[i].data_shift) |
				((hw_dword & write_arr[i].hw_mask) >>
				write_arr[i].hw_shift);

			if (write_arr[i].reg_addr != 0xFFFF) {
				i2c_byte1 = write_arr[i].reg_addr;
				i2c_byte2 = value;
				if (size != (i+1)) {
					i2c_byte2 = value & 0xFF;
					CDBG("byte1:0x%x, byte2:0x%x\n",
						i2c_byte1, i2c_byte2);
					i2c_tbl[a_ctrl->i2c_tbl_index].
						reg_addr = i2c_byte1;
					i2c_tbl[a_ctrl->i2c_tbl_index].
						reg_data = i2c_byte2;
					i2c_tbl[a_ctrl->i2c_tbl_index].
						delay = 0;
					a_ctrl->i2c_tbl_index++;
					i++;
					i2c_byte1 = write_arr[i].reg_addr;
					i2c_byte2 = (value & 0xFF00) >> 8;
				}
			} else {
				i2c_byte1 = (value & 0xFF00) >> 8;
				i2c_byte2 = value & 0xFF;
			}
		} else {
			i2c_byte1 = write_arr[i].reg_addr;
			i2c_byte2 = (hw_dword & write_arr[i].hw_mask) >>
				write_arr[i].hw_shift;
		}
		CDBG("i2c_byte1:0x%x, i2c_byte2:0x%x\n", i2c_byte1, i2c_byte2);
		i2c_tbl[a_ctrl->i2c_tbl_index].reg_addr = i2c_byte1;
		i2c_tbl[a_ctrl->i2c_tbl_index].reg_data = i2c_byte2;
		i2c_tbl[a_ctrl->i2c_tbl_index].delay = delay;
		a_ctrl->i2c_tbl_index++;
	}
	CDBG("Exit\n");
}

static int32_t msm_actuator_init_focus(struct msm_actuator_ctrl_t *a_ctrl,
	uint16_t size, enum msm_actuator_data_type type,
	struct reg_settings_t *settings)
{
	int32_t rc = -EFAULT;
	int32_t i = 0;

	CDBG("Enter\n");
	for (i = 0; i < size; i++) {
		switch (type) {
		case MSM_ACTUATOR_BYTE_DATA:
			rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write(
				&a_ctrl->i2c_client,
				settings[i].reg_addr,
				settings[i].reg_data, 
				MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_ACTUATOR_WORD_DATA:
			rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write(
				&a_ctrl->i2c_client,
				settings[i].reg_addr,
				settings[i].reg_data, MSM_CAMERA_I2C_WORD_DATA);
			break;
		default:
			pr_err("Unsupport data type: %d\n", type);
			break;
		}
		if (rc < 0)
			break;
	}

	a_ctrl->curr_step_pos = 0;

	CDBG("Exit\n");
	return rc;
}

static void msm_actuator_ois_parse_i2c_params(struct msm_actuator_ctrl_t *a_ctrl,
	int16_t next_lens_position, uint32_t hw_params, uint16_t delay)
{

	char buff[5];
	buff[0] = 0xF0;
	buff[1] = 0xB0;
	buff[2] = 0x00;
	buff[3] = (next_lens_position & 0xFF00) >> 8;
	buff[4] = (next_lens_position & 0xFF);

	pr_err("writing via i2c_write\n");
	a_ctrl->i2c_client.i2c_func_tbl->i2c_write_seq(&a_ctrl->i2c_client,
		buff[0], &buff[1], 4);
	usleep_range(delay, delay + 1000);
	pr_err("completed writing via i2c_write\n");
	CDBG("Exit\n");
}

int32_t msm_actuator_ois_set_scene_param(
	struct msm_actuator_ctrl_t *a_ctrl,
	enum bu63164_scene_param_t scene,
	struct bu63164_fact_adj_item_t *pFactoryAdjData,
	uint8_t enable_ois,
	uint8_t filter,
	uint8_t range)
{
	int32_t rc = 0;
	uint16_t i = 0;
	uint16_t data = 0;

	// Disable OIS
	RETURN_STATUS_CHECK(msm_actuator_ois_enable(a_ctrl, 0));

	for (i = 0; i < SCENE_REG_MAX; i++)
	{
		RETURN_STATUS_CHECK(msm_actuator_ois_memory_write(&(a_ctrl->i2c_client), SceneParamInfo.addr[i], SceneParamInfo.data[scene][i]));
	}
	for (i = 0; i < SCENE_REG_MAX; i++)
	{
		RETURN_STATUS_CHECK(msm_actuator_ois_memory_write(&(a_ctrl->i2c_client), SceneParamInfo.addr[i] + 0x80, SceneParamInfo.data[scene][i]));
	}

	if (filter == 1)
	{
		RETURN_STATUS_CHECK(msm_actuator_ois_memory_read(&(a_ctrl->i2c_client), MemoryAddr_EQCTL, &data));
		data |= 0x4000;
		RETURN_STATUS_CHECK(msm_actuator_ois_memory_write(&(a_ctrl->i2c_client), MemoryAddr_EQCTL, data));

		RETURN_STATUS_CHECK(msm_actuator_ois_memory_write(&(a_ctrl->i2c_client), MemoryAddr_Kgx13, 0x5900));
		RETURN_STATUS_CHECK(msm_actuator_ois_memory_write(&(a_ctrl->i2c_client), MemoryAddr_Kgx14, 0xC700));
		RETURN_STATUS_CHECK(msm_actuator_ois_memory_write(&(a_ctrl->i2c_client), MemoryAddr_Kgy13, 0x5900));
		RETURN_STATUS_CHECK(msm_actuator_ois_memory_write(&(a_ctrl->i2c_client), MemoryAddr_Kgy14, 0xC700));
	}
	else
	{
		RETURN_STATUS_CHECK(msm_actuator_ois_memory_read(&(a_ctrl->i2c_client), MemoryAddr_EQCTL, &data));
		data &= ~0x4000;
		RETURN_STATUS_CHECK(msm_actuator_ois_memory_write(&(a_ctrl->i2c_client), MemoryAddr_EQCTL, data));

		RETURN_STATUS_CHECK(msm_actuator_ois_memory_write(&(a_ctrl->i2c_client), MemoryAddr_Kgx13, 0x7470));
		RETURN_STATUS_CHECK(msm_actuator_ois_memory_write(&(a_ctrl->i2c_client), MemoryAddr_Kgx14, 0xCB90));
		RETURN_STATUS_CHECK(msm_actuator_ois_memory_write(&(a_ctrl->i2c_client), MemoryAddr_Kgy13, 0x7470));
		RETURN_STATUS_CHECK(msm_actuator_ois_memory_write(&(a_ctrl->i2c_client), MemoryAddr_Kgy14, 0xCB90));
	}

	RETURN_STATUS_CHECK(msm_actuator_ois_memory_write(&(a_ctrl->i2c_client), MemoryAddr_wDgx02, 0x0000));
	RETURN_STATUS_CHECK(msm_actuator_ois_memory_write(&(a_ctrl->i2c_client), MemoryAddr_wDgx03, 0x0000));
	RETURN_STATUS_CHECK(msm_actuator_ois_memory_write(&(a_ctrl->i2c_client), MemoryAddr_wDgx06, 0x7FFF));
	RETURN_STATUS_CHECK(msm_actuator_ois_memory_write(&(a_ctrl->i2c_client), MemoryAddr_Kgx15,  0x0000));
	
	RETURN_STATUS_CHECK(msm_actuator_ois_memory_write(&(a_ctrl->i2c_client), MemoryAddr_wDgy02, 0x0000));
	RETURN_STATUS_CHECK(msm_actuator_ois_memory_write(&(a_ctrl->i2c_client), MemoryAddr_wDgy03, 0x0000));
	RETURN_STATUS_CHECK(msm_actuator_ois_memory_write(&(a_ctrl->i2c_client), MemoryAddr_wDgy06, 0x7FFF));
	RETURN_STATUS_CHECK(msm_actuator_ois_memory_write(&(a_ctrl->i2c_client), MemoryAddr_Kgy15,  0x0000));

	if (range == 1)
	{
		RETURN_STATUS_CHECK(
			msm_actuator_ois_peripheral_write(
				&(a_ctrl->i2c_client), FactoryAdjData[FactAdjName_HALOFS_X].ctrl_addr, FactoryAdjData[FactAdjName_SFTHAL_X].data));
		RETURN_STATUS_CHECK(
			msm_actuator_ois_peripheral_write(
				&(a_ctrl->i2c_client), FactoryAdjData[FactAdjName_HALOFS_Y].ctrl_addr, FactoryAdjData[FactAdjName_SFTHAL_Y].data));
	}
	else
	{
		RETURN_STATUS_CHECK(
			msm_actuator_ois_peripheral_write(
				&(a_ctrl->i2c_client), FactoryAdjData[FactAdjName_HALOFS_X].ctrl_addr, FactoryAdjData[FactAdjName_HALOFS_X].data));
		RETURN_STATUS_CHECK(
			msm_actuator_ois_peripheral_write(
				&(a_ctrl->i2c_client), FactoryAdjData[FactAdjName_HALOFS_Y].ctrl_addr, FactoryAdjData[FactAdjName_HALOFS_Y].data));
	}
	// Enable / Disable OIS
	RETURN_STATUS_CHECK(msm_actuator_ois_enable(a_ctrl, enable_ois));

	return rc;

Error_Exit:
	pr_err("Error in %s at line %d rc = %d\n", __func__, __LINE__, rc);
	return rc;
}

static int32_t msm_actuator_ois_vcoset(struct msm_actuator_ctrl_t *a_ctrl)
{
	int32_t rc = 0;
	uint16_t CLK_PS = 12000;     // Input Frequency [kHz] of CLK/PS terminal (Depend on your system)     RHM_HT 2013.03.19       Change 6M -> 12M
	uint16_t FVCO_1 = 27000;     // Target Frequency [kHz]
	uint16_t FREF = 25;          // Reference Clock Frequency [kHz]

	uint16_t DIV_N = CLK_PS / FREF - 1;  // calc DIV_N
	uint16_t DIV_M = FVCO_1 / FREF - 1;  // calc DIV_M

	RETURN_STATUS_CHECK(msm_actuator_ois_peripheral_write(&(a_ctrl->i2c_client), 0x62, DIV_N));
	RETURN_STATUS_CHECK(msm_actuator_ois_peripheral_write(&(a_ctrl->i2c_client), 0x63, DIV_M));
	RETURN_STATUS_CHECK(msm_actuator_ois_peripheral_write(&(a_ctrl->i2c_client), 0x64, 0x4060));

	RETURN_STATUS_CHECK(msm_actuator_ois_peripheral_write(&(a_ctrl->i2c_client), 0x60, 0x3011));
	RETURN_STATUS_CHECK(msm_actuator_ois_peripheral_write(&(a_ctrl->i2c_client), 0x65, 0x0080));
	RETURN_STATUS_CHECK(msm_actuator_ois_peripheral_write(&(a_ctrl->i2c_client), 0x61, 0x8002));
	RETURN_STATUS_CHECK(msm_actuator_ois_peripheral_write(&(a_ctrl->i2c_client), 0x61, 0x8003));
	RETURN_STATUS_CHECK(msm_actuator_ois_peripheral_write(&(a_ctrl->i2c_client), 0x61, 0x8809));

	msleep(30); 				   // Wait for PLL lock

	RETURN_STATUS_CHECK(msm_actuator_ois_peripheral_write(&(a_ctrl->i2c_client), 0x05, 0x000C));
	RETURN_STATUS_CHECK(msm_actuator_ois_peripheral_write(&(a_ctrl->i2c_client), 0x05, 0x000D));

	return rc;
Error_Exit:
	pr_err("Error in %s at line %d rc = %d\n", __func__, __LINE__, rc);
	return rc;
}

static int32_t msm_actuator_ois_init_focus(struct msm_actuator_ctrl_t *a_ctrl,
	uint16_t size, enum msm_actuator_data_type type,
	struct reg_settings_t *settings)
{
	int32_t rc = -EFAULT;
	int32_t i = 0;
	uint16_t status = 0;

	CDBG("Enter\n");

	RETURN_STATUS_CHECK(msm_actuator_ois_init(a_ctrl));

	RETURN_STATUS_CHECK(
		msm_actuator_ois_fact_adj_apply(
		a_ctrl, FactoryAdjData, sizeof(FactoryAdjData) / sizeof(FactoryAdjData[0])));

	RETURN_STATUS_CHECK(msm_actuator_ois_vcoset(a_ctrl));

	for (i = 0; i < size; i++) {
		switch (type) {
		case MSM_ACTUATOR_BYTE_DATA:
			rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write(
				&a_ctrl->i2c_client,
				settings[i].reg_addr,
				settings[i].reg_data, 
				MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_ACTUATOR_WORD_DATA:
			rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write(
				&a_ctrl->i2c_client,
				settings[i].reg_addr,
				settings[i].reg_data, MSM_CAMERA_I2C_WORD_DATA);
			break;
		default:
			pr_err("Unsupport data type: %d\n", type);
			break;
		}
		if (rc < 0)
			break;
	}

	a_ctrl->curr_step_pos = 0;

	RETURN_STATUS_CHECK(msm_actuator_ois_memory_read(&a_ctrl->i2c_client, MemoryAddr_OIS_STS, &status));

	if (status != 0x0104)
	{
		pr_err("Error in %s at line %d Actuator in bad state\n", __func__, __LINE__);
		rc = -1;
		goto Error_Exit;
	}
	
	// Position Servo ON (OIS OFF)
	RETURN_STATUS_CHECK(msm_actuator_ois_memory_write(&a_ctrl->i2c_client, MemoryAddr_EQCTL, 0x0C0C));

	RETURN_STATUS_CHECK(msm_actuator_ois_set_scene_param(a_ctrl, SCENE_PARAM_TEST, FactoryAdjData, 1, 0, 0));

	return rc;
Error_Exit:
	pr_err("Error in %s at line %d rc = %d\n", __func__, __LINE__, rc);
	return rc;
}

static void msm_actuator_write_focus(
	struct msm_actuator_ctrl_t *a_ctrl,
	uint16_t curr_lens_pos,
	struct damping_params_t *damping_params,
	int8_t sign_direction,
	int16_t code_boundary)
{
	int16_t next_lens_pos = 0;
	uint16_t damping_code_step = 0;
	uint16_t wait_time = 0;
	CDBG("Enter\n");

	damping_code_step = damping_params->damping_step;
	wait_time = damping_params->damping_delay;

	/* Write code based on damping_code_step in a loop */
	for (next_lens_pos =
		curr_lens_pos + (sign_direction * damping_code_step);
		(sign_direction * next_lens_pos) <=
			(sign_direction * code_boundary);
		next_lens_pos =
			(next_lens_pos +
				(sign_direction * damping_code_step))) {
		a_ctrl->func_tbl->actuator_parse_i2c_params(a_ctrl,
			next_lens_pos, damping_params->hw_params, wait_time);
		curr_lens_pos = next_lens_pos;
	}

	if (curr_lens_pos != code_boundary) {
		a_ctrl->func_tbl->actuator_parse_i2c_params(a_ctrl,
			code_boundary, damping_params->hw_params, wait_time);
	}
	CDBG("Exit\n");
}

static int32_t msm_actuator_piezo_move_focus(
	struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_move_params_t *move_params)
{
	int32_t dest_step_position = move_params->dest_step_pos;
	struct damping_params_t ringing_params_kernel;
	int32_t rc = 0;
	int32_t num_steps = move_params->num_steps;
	CDBG("Enter\n");

	if (copy_from_user(&ringing_params_kernel,
		&(move_params->ringing_params[0]),
		sizeof(struct damping_params_t))) {
		pr_err("copy_from_user failed\n");
		return -EFAULT;
	}

	if (num_steps == 0)
		return rc;

	a_ctrl->i2c_tbl_index = 0;
	a_ctrl->func_tbl->actuator_parse_i2c_params(a_ctrl,
		(num_steps *
		a_ctrl->region_params[0].code_per_step),
		ringing_params_kernel.hw_params, 0);

	rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write_table_w_microdelay(
		&a_ctrl->i2c_client,
		a_ctrl->i2c_reg_tbl, a_ctrl->i2c_tbl_index,
		a_ctrl->i2c_data_type);
	if (rc < 0) {
		pr_err("i2c write error:%d\n", rc);
		return rc;
	}
	a_ctrl->i2c_tbl_index = 0;
	a_ctrl->curr_step_pos = dest_step_position;
	CDBG("Exit\n");
	return rc;
}

static int32_t msm_actuator_move_focus(
	struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_move_params_t *move_params)
{
	int32_t rc = 0;
	struct damping_params_t ringing_params_kernel;
	int8_t sign_dir = move_params->sign_dir;
	uint16_t step_boundary = 0;
	uint16_t target_step_pos = 0;
	uint16_t target_lens_pos = 0;
	int16_t dest_step_pos = move_params->dest_step_pos;
	uint16_t curr_lens_pos = 0;
	int dir = move_params->dir;
	int32_t num_steps = move_params->num_steps;

	if (copy_from_user(&ringing_params_kernel,
		&(move_params->ringing_params[a_ctrl->curr_region_index]),
		sizeof(struct damping_params_t))) {
		pr_err("copy_from_user failed\n");
		return -EFAULT;
	}


	CDBG("called, dir %d, num_steps %d\n", dir, num_steps);

	if (dest_step_pos == a_ctrl->curr_step_pos)
		return rc;

	if ((sign_dir > MSM_ACTUATOR_MOVE_SIGNED_NEAR) ||
		(sign_dir < MSM_ACTUATOR_MOVE_SIGNED_FAR)) {
		pr_err("%s:%d Invalid sign_dir = %d\n",
		__func__, __LINE__, sign_dir);
		return -EFAULT;
	}
	if ((dir > MOVE_FAR) || (dir < MOVE_NEAR)) {
		pr_err("%s:%d Invalid direction = %d\n",
		__func__, __LINE__, dir);
		return -EFAULT;
	}
	if (dest_step_pos > a_ctrl->total_steps) {
		pr_err("Step pos greater than total steps = %d\n",
		dest_step_pos);
		return -EFAULT;
	}
	curr_lens_pos = a_ctrl->step_position_table[a_ctrl->curr_step_pos];
	a_ctrl->i2c_tbl_index = 0;
	CDBG("curr_step_pos =%d dest_step_pos =%d curr_lens_pos=%d\n",
		a_ctrl->curr_step_pos, dest_step_pos, curr_lens_pos);

	while (a_ctrl->curr_step_pos != dest_step_pos) {
		step_boundary =
			a_ctrl->region_params[a_ctrl->curr_region_index].
			step_bound[dir];
		if ((dest_step_pos * sign_dir) <=
			(step_boundary * sign_dir)) {

			target_step_pos = dest_step_pos;
			target_lens_pos =
				a_ctrl->step_position_table[target_step_pos];
			a_ctrl->func_tbl->actuator_write_focus(a_ctrl,
					curr_lens_pos,
					&ringing_params_kernel,
					sign_dir,
					target_lens_pos);
			curr_lens_pos = target_lens_pos;

		} else {
			target_step_pos = step_boundary;
			target_lens_pos =
				a_ctrl->step_position_table[target_step_pos];
			a_ctrl->func_tbl->actuator_write_focus(a_ctrl,
					curr_lens_pos,
					&ringing_params_kernel,
					sign_dir,
					target_lens_pos);
			curr_lens_pos = target_lens_pos;

			a_ctrl->curr_region_index += sign_dir;
		}
		a_ctrl->curr_step_pos = target_step_pos;
#if defined(CONFIG_ARCH_MSM8974_APOLLO)
        a_ctrl->current_lens_pos = a_ctrl->step_position_table[a_ctrl->curr_step_pos];
#endif
	}

#if defined(CONFIG_ARCH_MSM8974_APOLLO)
	rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write_table_w_microdelay(
		&a_ctrl->i2c_client,
		a_ctrl->i2c_reg_tbl, a_ctrl->i2c_tbl_index,
		a_ctrl->i2c_data_type);
	if (rc < 0) {
		pr_err("i2c write error:%d\n", rc);
		return rc;
	}
	a_ctrl->i2c_tbl_index = 0;
#endif
	CDBG("Exit\n");

	return rc;
}

static int32_t msm_actuator_init_step_table(struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_set_info_t *set_info)
{
	int16_t code_per_step = 0;
	int16_t cur_code = 0;
	int16_t step_index = 0;
	int32_t region_index = 0;
	uint16_t step_boundary = 0;
	uint32_t max_code_size = 1;
	uint16_t data_size = set_info->actuator_params.data_size;
	CDBG("Enter\n");

	for (; data_size > 0; data_size--)
		max_code_size *= 2;

	max_code_size -= 1;
	kfree(a_ctrl->step_position_table);
	a_ctrl->step_position_table = NULL;

	if (set_info->af_tuning_params.total_steps
		>  MAX_ACTUATOR_AF_TOTAL_STEPS) {
		pr_err("%s: Max actuator totalsteps exceeded = %d\n",
		__func__, set_info->af_tuning_params.total_steps);
		return -EFAULT;
	}
	/* Fill step position table */
	a_ctrl->step_position_table =
		kmalloc(sizeof(uint16_t) *
		(set_info->af_tuning_params.total_steps + 1), GFP_KERNEL);

	if (a_ctrl->step_position_table == NULL)
		return -ENOMEM;

	cur_code = set_info->af_tuning_params.initial_code;
	a_ctrl->step_position_table[step_index++] = cur_code;
	for (region_index = 0;
		region_index < a_ctrl->region_size;
		region_index++) {
		code_per_step =
			a_ctrl->region_params[region_index].code_per_step;
		step_boundary =
			a_ctrl->region_params[region_index].
			step_bound[MOVE_NEAR];
		if (step_boundary > set_info->af_tuning_params.total_steps - 1) {
		     pr_err("%s: Error af steps mismatch!", __func__);
			 return -EFAULT;
		}
		for (; step_index <= step_boundary;
			step_index++) {
			cur_code = set_info->af_tuning_params.initial_code + ((step_index*code_per_step)>>8);
			if (cur_code < max_code_size)
				a_ctrl->step_position_table[step_index] =
					cur_code;
			else {
				for (; step_index <
					set_info->af_tuning_params.total_steps;
					step_index++)
					a_ctrl->
						step_position_table[
						step_index] =
						max_code_size;
			}
		}
	}
	for (step_index = 0;
	      step_index < set_info->af_tuning_params.total_steps;
		  step_index++) {
		  pr_err("step_position_table[%d] = %d", step_index,a_ctrl->step_position_table[step_index]);
	}
	CDBG("Exit\n");
	return 0;
}

static int32_t msm_actuator_set_default_focus(
	struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_move_params_t *move_params)
{
	int32_t rc = 0;
	CDBG("Enter\n");

	if (a_ctrl->curr_step_pos != move_params->num_steps_inf_pos)
		rc = a_ctrl->func_tbl->actuator_move_focus(a_ctrl, move_params);
	CDBG("Exit\n");
	return rc;
}

static int32_t msm_actuator_power_down(struct msm_actuator_ctrl_t *a_ctrl)
{
	int32_t rc = 0;
#if defined(CONFIG_ARCH_MSM8974_APOLLO)
    	uint16_t code = 0;
    	uint8_t buf[2];
#endif
	CDBG("Enter\n");
	if (a_ctrl->actuator_state != ACTUATOR_POWER_DOWN) {
		if (a_ctrl->vcm_enable) {
			rc = gpio_direction_output(a_ctrl->vcm_pwd, 0);
			if (!rc)
				gpio_free(a_ctrl->vcm_pwd);
		}
#if defined(CONFIG_ARCH_MSM8974_APOLLO)
	code = a_ctrl->current_lens_pos;
	CDBG("%s: a_ctrl->curr_lens_pos = %d\n",__func__,a_ctrl->current_lens_pos);
	while(code) {

	CDBG("%s: code is = %d\n",__func__,code);
	code = (code > 30)?(code - 30) : 0;
	buf[0] = (code >> 4);
	buf[1] = ((code << 4) | 0x7) & 0xFF;
	rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write_seq(&a_ctrl->i2c_client,
	buf[0],
	&buf[1], 1);
	usleep_range(15000, 16000);
	}
	buf[0] = buf[1] =  0;
	rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write_seq(&a_ctrl->i2c_client,
		buf[0],
		&buf[1], 1);

	usleep_range(8000, 9000);
#endif
		kfree(a_ctrl->step_position_table);
		a_ctrl->step_position_table = NULL;
		kfree(a_ctrl->i2c_reg_tbl);
		a_ctrl->i2c_reg_tbl = NULL;
		a_ctrl->i2c_tbl_index = 0;
		a_ctrl->actuator_state = ACTUATOR_POWER_DOWN;
	}
	CDBG("Exit\n");
	return rc;
}

static int32_t msm_actuator_init(struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_set_info_t *set_info) {
	struct reg_settings_t *init_settings = NULL;
	int32_t rc = -EFAULT;
	uint16_t i = 0;
	struct msm_camera_cci_client *cci_client = NULL;
	CDBG("Enter\n");

	for (i = 0; i < ARRAY_SIZE(actuators); i++) {
		if (set_info->actuator_params.act_type ==
			actuators[i]->act_type) {
			a_ctrl->func_tbl = &actuators[i]->func_tbl;
			rc = 0;
		}
	}

	if (rc < 0) {
		pr_err("Actuator function table not found\n");
		return rc;
	}
	if (set_info->af_tuning_params.total_steps
		>  MAX_ACTUATOR_AF_TOTAL_STEPS) {
		pr_err("%s: Max actuator totalsteps exceeded = %d\n",
		__func__, set_info->af_tuning_params.total_steps);
		return -EFAULT;
	}
	if (set_info->af_tuning_params.region_size
		> MAX_ACTUATOR_REGION) {
		pr_err("MAX_ACTUATOR_REGION is exceeded.\n");
		return -EFAULT;
	}

	a_ctrl->region_size = set_info->af_tuning_params.region_size;
	a_ctrl->pwd_step = set_info->af_tuning_params.pwd_step;
	a_ctrl->total_steps = set_info->af_tuning_params.total_steps;

	if (copy_from_user(&a_ctrl->region_params,
		(void *)set_info->af_tuning_params.region_params,
		a_ctrl->region_size * sizeof(struct region_params_t)))
		return -EFAULT;

	if (a_ctrl->act_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		cci_client = a_ctrl->i2c_client.cci_client;
		cci_client->sid =
			set_info->actuator_params.i2c_addr >> 1;
		cci_client->retries = 3;
		cci_client->id_map = 0;
		cci_client->cci_i2c_master = a_ctrl->cci_master;
	} else {
		a_ctrl->i2c_client.client->addr =
			set_info->actuator_params.i2c_addr;
	}

	a_ctrl->i2c_data_type = set_info->actuator_params.i2c_data_type;
	a_ctrl->i2c_client.addr_type = set_info->actuator_params.i2c_addr_type;
	if (set_info->actuator_params.reg_tbl_size <=
		MAX_ACTUATOR_REG_TBL_SIZE) {
		a_ctrl->reg_tbl_size = set_info->actuator_params.reg_tbl_size;
	} else {
		a_ctrl->reg_tbl_size = 0;
		pr_err("MAX_ACTUATOR_REG_TBL_SIZE is exceeded.\n");
		return -EFAULT;
	}

	kfree(a_ctrl->i2c_reg_tbl);

	a_ctrl->i2c_reg_tbl =
		kmalloc(sizeof(struct msm_camera_i2c_reg_tbl) *
		(set_info->af_tuning_params.total_steps + 1), GFP_KERNEL);
	if (!a_ctrl->i2c_reg_tbl) {
		pr_err("kmalloc fail\n");
		return -ENOMEM;
	}

	if (copy_from_user(&a_ctrl->reg_tbl,
		(void *)set_info->actuator_params.reg_tbl_params,
		a_ctrl->reg_tbl_size *
		sizeof(struct msm_actuator_reg_params_t))) {
		kfree(a_ctrl->i2c_reg_tbl);
		return -EFAULT;
	}

	if (set_info->actuator_params.init_setting_size &&
		set_info->actuator_params.init_setting_size
		<= MAX_ACTUATOR_REG_TBL_SIZE) {
		if (a_ctrl->func_tbl->actuator_init_focus) {
			init_settings = kmalloc(sizeof(struct reg_settings_t) *
				(set_info->actuator_params.init_setting_size),
				GFP_KERNEL);
			if (init_settings == NULL) {
				kfree(a_ctrl->i2c_reg_tbl);
				pr_err("Error allocating memory for init_settings\n");
				return -EFAULT;
			}
			if (copy_from_user(init_settings,
				(void *)set_info->actuator_params.init_settings,
				set_info->actuator_params.init_setting_size *
				sizeof(struct reg_settings_t))) {
				kfree(init_settings);
				kfree(a_ctrl->i2c_reg_tbl);
				pr_err("Error copying init_settings\n");
				return -EFAULT;
			}
			rc = a_ctrl->func_tbl->actuator_init_focus(a_ctrl,
				set_info->actuator_params.init_setting_size,
				a_ctrl->i2c_data_type,
				init_settings);
			kfree(init_settings);
			if (rc < 0) {
				kfree(a_ctrl->i2c_reg_tbl);
				pr_err("Error actuator_init_focus\n");
				return -EFAULT;
			}
		}
	}

	a_ctrl->initial_code = set_info->af_tuning_params.initial_code;
	if (a_ctrl->func_tbl->actuator_init_step_table)
		rc = a_ctrl->func_tbl->
			actuator_init_step_table(a_ctrl, set_info);

	a_ctrl->curr_step_pos = 0;
	a_ctrl->curr_region_index = 0;
	a_ctrl->actuator_state = ACTUATOR_POWER_UP;
	CDBG("Exit\n");

	return rc;
}

static int32_t msm_actuator_config(struct msm_actuator_ctrl_t *a_ctrl,
	void __user *argp)
{
	struct msm_actuator_cfg_data *cdata =
		(struct msm_actuator_cfg_data *)argp;
	int32_t rc = 0;
	mutex_lock(a_ctrl->actuator_mutex);
	CDBG("Enter\n");
	CDBG("%s type %d\n", __func__, cdata->cfgtype);
	switch (cdata->cfgtype) {
	case CFG_GET_ACTUATOR_INFO:
		cdata->is_af_supported = 1;
		cdata->cfg.cam_name = a_ctrl->cam_name;
		break;

	case CFG_SET_ACTUATOR_INFO:
		rc = msm_actuator_init(a_ctrl, &cdata->cfg.set_info);
		if (rc < 0)
			pr_err("init table failed %d\n", rc);
		break;

	case CFG_SET_DEFAULT_FOCUS:
		rc = a_ctrl->func_tbl->actuator_set_default_focus(a_ctrl,
			&cdata->cfg.move);
		if (rc < 0)
			pr_err("move focus failed %d\n", rc);
		break;

	case CFG_MOVE_FOCUS:
		rc = a_ctrl->func_tbl->actuator_move_focus(a_ctrl,
			&cdata->cfg.move);
		if (rc < 0)
			pr_err("move focus failed %d\n", rc);
		break;

	case CFG_ENABLE_OIS:
		if (a_ctrl->func_tbl)
		{
			if (a_ctrl->func_tbl->actuator_enable_ois)
				rc = a_ctrl->func_tbl->actuator_enable_ois(a_ctrl, cdata->cfg.enable_ois);
			else
				rc = -1;
			if (rc < 0)
				pr_err("ois enable failed %d\n", rc);
		}
		break;
#if defined(CONFIG_ARCH_MSM8974_APOLLO)
	case CFG_ACTUATOR_POWERDOWN:
		rc = msm_actuator_power_down(a_ctrl);
		if (rc < 0)
			pr_err("msm_actuator_power_down failed %d\n", rc);
		break;
#endif
	default:
		break;
	}
	mutex_unlock(a_ctrl->actuator_mutex);
	CDBG("Exit\n");
	return rc;
}

static int32_t msm_actuator_get_subdev_id(struct msm_actuator_ctrl_t *a_ctrl,
	void *arg)
{
	uint32_t *subdev_id = (uint32_t *)arg;
	CDBG("Enter\n");
	if (!subdev_id) {
		pr_err("failed\n");
		return -EINVAL;
	}
	if (a_ctrl->act_device_type == MSM_CAMERA_PLATFORM_DEVICE)
		*subdev_id = a_ctrl->pdev->id;
	else
		*subdev_id = a_ctrl->subdev_id;

	CDBG("subdev_id %d\n", *subdev_id);
	CDBG("Exit\n");
	return 0;
}

static struct msm_camera_i2c_fn_t msm_sensor_cci_func_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = msm_camera_cci_i2c_read_seq,
	.i2c_write = msm_camera_cci_i2c_write,
	.i2c_write_seq = msm_camera_cci_i2c_write_seq,
	.i2c_write_table = msm_camera_cci_i2c_write_table,
	.i2c_write_seq_table = msm_camera_cci_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_cci_i2c_write_table_w_microdelay,
	.i2c_util = msm_sensor_cci_i2c_util,
};

static struct msm_camera_i2c_fn_t msm_sensor_qup_func_tbl = {
	.i2c_read = msm_camera_qup_i2c_read,
	.i2c_read_seq = msm_camera_qup_i2c_read_seq,
	.i2c_write = msm_camera_qup_i2c_write,
	.i2c_write_table = msm_camera_qup_i2c_write_table,
	.i2c_write_seq_table = msm_camera_qup_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_qup_i2c_write_table_w_microdelay,
};

static int msm_actuator_open(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh) {
	int rc = 0;
	struct msm_actuator_ctrl_t *a_ctrl =  v4l2_get_subdevdata(sd);
	CDBG("Enter %s", __func__);
	if (!a_ctrl) {
		pr_err("failed\n");
		return -EINVAL;
	}
	if (a_ctrl->act_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_util(
			&a_ctrl->i2c_client, MSM_CCI_INIT);
		if (rc < 0)
			pr_err("cci_init failed\n");
		else
			a_ctrl->cci_initted = 1;
	}
	CDBG("Exit\n");
	return rc;
}

static int msm_actuator_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh) {
	int rc = 0;
	struct msm_actuator_ctrl_t *a_ctrl =  v4l2_get_subdevdata(sd);
	CDBG("Enter %s", __func__);
	if (!a_ctrl) {
		pr_err("failed\n");
		return -EINVAL;
	}
	if (a_ctrl->act_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		if (a_ctrl->cci_initted)
		{
			rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_util(
				&a_ctrl->i2c_client, MSM_CCI_RELEASE);
			if (rc < 0)
				pr_err("cci_release failed\n");
			else
				a_ctrl->cci_initted = 0;
		}
	}
	kfree(a_ctrl->i2c_reg_tbl);
	a_ctrl->i2c_reg_tbl = NULL;

	CDBG("Exit\n");
	return rc;
}

static const struct v4l2_subdev_internal_ops msm_actuator_internal_ops = {
	.open = msm_actuator_open,
	.close = msm_actuator_close,
};

static long msm_actuator_subdev_ioctl(struct v4l2_subdev *sd,
			unsigned int cmd, void *arg)
{
	struct msm_actuator_ctrl_t *a_ctrl = v4l2_get_subdevdata(sd);
	void __user *argp = (void __user *)arg;
	CDBG("Enter\n");
	CDBG("%s:%d a_ctrl %p argp %p\n", __func__, __LINE__, a_ctrl, argp);
	switch (cmd) {
	case VIDIOC_MSM_SENSOR_GET_SUBDEV_ID:
		return msm_actuator_get_subdev_id(a_ctrl, argp);
	case VIDIOC_MSM_ACTUATOR_CFG:
		return msm_actuator_config(a_ctrl, argp);
	case MSM_SD_SHUTDOWN:
		msm_actuator_close(sd, NULL);
		return 0;
	default:
		return -ENOIOCTLCMD;
	}
}

static int32_t msm_actuator_power_up(struct msm_actuator_ctrl_t *a_ctrl)
{
	int rc = 0;
	CDBG("%s called\n", __func__);

	CDBG("vcm info: %x %x\n", a_ctrl->vcm_pwd,
		a_ctrl->vcm_enable);
	if (a_ctrl->vcm_enable) {
		rc = gpio_request(a_ctrl->vcm_pwd, "msm_actuator");
		if (!rc) {
			CDBG("Enable VCM PWD\n");
			gpio_direction_output(a_ctrl->vcm_pwd, 1);
		}
	}
	CDBG("Exit\n");
	return rc;
}

static int32_t msm_actuator_power(struct v4l2_subdev *sd, int on)
{
	int rc = 0;
	struct msm_actuator_ctrl_t *a_ctrl = v4l2_get_subdevdata(sd);
	CDBG("Enter\n");
	mutex_lock(a_ctrl->actuator_mutex);
	if (on)
		rc = msm_actuator_power_up(a_ctrl);
	else
		rc = msm_actuator_power_down(a_ctrl);
	mutex_unlock(a_ctrl->actuator_mutex);
	CDBG("Exit\n");
	return rc;
}

static struct v4l2_subdev_core_ops msm_actuator_subdev_core_ops = {
	.ioctl = msm_actuator_subdev_ioctl,
	.s_power = msm_actuator_power,
};

static struct v4l2_subdev_ops msm_actuator_subdev_ops = {
	.core = &msm_actuator_subdev_core_ops,
};

static const struct i2c_device_id msm_actuator_i2c_id[] = {
	{"qcom,actuator", (kernel_ulong_t)NULL},
	{ }
};

static int32_t msm_actuator_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	struct msm_actuator_ctrl_t *act_ctrl_t = NULL;
	CDBG("Enter\n");

	if (client == NULL) {
		pr_err("msm_actuator_i2c_probe: client is null\n");
		rc = -EINVAL;
		goto probe_failure;
	}

	act_ctrl_t = kzalloc(sizeof(struct msm_actuator_ctrl_t),
		GFP_KERNEL);
	if (!act_ctrl_t) {
		pr_err("%s:%d failed no memory\n", __func__, __LINE__);
		return -ENOMEM;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality failed\n");
		goto probe_failure;
	}

	CDBG("client = %x\n", (unsigned int) client);

	rc = of_property_read_u32(client->dev.of_node, "cell-index",
		&act_ctrl_t->subdev_id);
	CDBG("cell-index %d, rc %d\n", act_ctrl_t->subdev_id, rc);
	if (rc < 0) {
		pr_err("failed rc %d\n", rc);
		return rc;
	}

	act_ctrl_t->i2c_driver = &msm_actuator_i2c_driver;
	act_ctrl_t->i2c_client.client = client;
	act_ctrl_t->curr_step_pos = 0,
	act_ctrl_t->curr_region_index = 0,
	/* Set device type as I2C */
	act_ctrl_t->act_device_type = MSM_CAMERA_I2C_DEVICE;
	act_ctrl_t->i2c_client.i2c_func_tbl = &msm_sensor_qup_func_tbl;
	act_ctrl_t->act_v4l2_subdev_ops = &msm_actuator_subdev_ops;
	act_ctrl_t->actuator_mutex = &msm_actuator_mutex;

	act_ctrl_t->cam_name = act_ctrl_t->subdev_id;
	CDBG("act_ctrl_t->cam_name: %d", act_ctrl_t->cam_name);
	/* Assign name for sub device */
	snprintf(act_ctrl_t->msm_sd.sd.name, sizeof(act_ctrl_t->msm_sd.sd.name),
		"%s", act_ctrl_t->i2c_driver->driver.name);

	/* Initialize sub device */
	v4l2_i2c_subdev_init(&act_ctrl_t->msm_sd.sd,
		act_ctrl_t->i2c_client.client,
		act_ctrl_t->act_v4l2_subdev_ops);
	v4l2_set_subdevdata(&act_ctrl_t->msm_sd.sd, act_ctrl_t);
	act_ctrl_t->msm_sd.sd.internal_ops = &msm_actuator_internal_ops;
	act_ctrl_t->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	media_entity_init(&act_ctrl_t->msm_sd.sd.entity, 0, NULL, 0);
	act_ctrl_t->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	act_ctrl_t->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_ACTUATOR;
	act_ctrl_t->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x2;
	msm_sd_register(&act_ctrl_t->msm_sd);
	pr_info("msm_actuator_i2c_probe: succeeded\n");
	CDBG("Exit\n");

probe_failure:
	return rc;
}

static int32_t msm_actuator_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	struct msm_camera_cci_client *cci_client = NULL;
	struct msm_actuator_ctrl_t *msm_actuator_t = NULL;
	CDBG("Enter\n");

	if (!pdev->dev.of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}

	msm_actuator_t = kzalloc(sizeof(struct msm_actuator_ctrl_t),
		GFP_KERNEL);
	if (!msm_actuator_t) {
		pr_err("%s:%d failed no memory\n", __func__, __LINE__);
		return -ENOMEM;
	}
	rc = of_property_read_u32((&pdev->dev)->of_node, "cell-index",
		&pdev->id);
	CDBG("cell-index %d, rc %d\n", pdev->id, rc);
	if (rc < 0) {
		kfree(msm_actuator_t);
		pr_err("failed rc %d\n", rc);
		return rc;
	}

	rc = of_property_read_u32((&pdev->dev)->of_node, "qcom,cci-master",
		&msm_actuator_t->cci_master);
	CDBG("qcom,cci-master %d, rc %d\n", msm_actuator_t->cci_master, rc);
	if (rc < 0) {
		kfree(msm_actuator_t);
		pr_err("failed rc %d\n", rc);
		return rc;
	}

	msm_actuator_t->act_v4l2_subdev_ops = &msm_actuator_subdev_ops;
	msm_actuator_t->actuator_mutex = &msm_actuator_mutex;
	msm_actuator_t->cam_name = pdev->id;

	/* Set platform device handle */
	msm_actuator_t->pdev = pdev;
	/* Set device type as platform device */
	msm_actuator_t->act_device_type = MSM_CAMERA_PLATFORM_DEVICE;
	msm_actuator_t->i2c_client.i2c_func_tbl = &msm_sensor_cci_func_tbl;
	msm_actuator_t->i2c_client.cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!msm_actuator_t->i2c_client.cci_client) {
		kfree(msm_actuator_t);
		pr_err("failed no memory\n");
		return -ENOMEM;
	}

	cci_client = msm_actuator_t->i2c_client.cci_client;
	cci_client->cci_subdev = msm_cci_get_subdev();
	v4l2_subdev_init(&msm_actuator_t->msm_sd.sd,
		msm_actuator_t->act_v4l2_subdev_ops);
	v4l2_set_subdevdata(&msm_actuator_t->msm_sd.sd, msm_actuator_t);
	msm_actuator_t->msm_sd.sd.internal_ops = &msm_actuator_internal_ops;
	msm_actuator_t->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(msm_actuator_t->msm_sd.sd.name,
		ARRAY_SIZE(msm_actuator_t->msm_sd.sd.name), "msm_actuator");
	media_entity_init(&msm_actuator_t->msm_sd.sd.entity, 0, NULL, 0);
	msm_actuator_t->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	msm_actuator_t->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_ACTUATOR;
	msm_actuator_t->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x2;
	msm_sd_register(&msm_actuator_t->msm_sd);
	CDBG("Exit\n");
	return rc;
}

static const struct of_device_id msm_actuator_i2c_dt_match[] = {
	{.compatible = "qcom,actuator"},
	{}
};

MODULE_DEVICE_TABLE(of, msm_actuator_i2c_dt_match);

static struct i2c_driver msm_actuator_i2c_driver = {
	.id_table = msm_actuator_i2c_id,
	.probe  = msm_actuator_i2c_probe,
	.remove = __exit_p(msm_actuator_i2c_remove),
	.driver = {
		.name = "qcom,actuator",
		.owner = THIS_MODULE,
		.of_match_table = msm_actuator_i2c_dt_match,
	},
};

static const struct of_device_id msm_actuator_dt_match[] = {
	{.compatible = "qcom,actuator", .data = NULL},
	{}
};

MODULE_DEVICE_TABLE(of, msm_actuator_dt_match);

static struct platform_driver msm_actuator_platform_driver = {
	.driver = {
		.name = "qcom,actuator",
		.owner = THIS_MODULE,
		.of_match_table = msm_actuator_dt_match,
	},
};

static int __init msm_actuator_init_module(void)
{
	int32_t rc = 0;
	CDBG("Enter\n");
	rc = platform_driver_probe(&msm_actuator_platform_driver,
		msm_actuator_platform_probe);
	if (!rc)
		return rc;
	CDBG("%s:%d rc %d\n", __func__, __LINE__, rc);

	return i2c_add_driver(&msm_actuator_i2c_driver);
}


static struct msm_actuator msm_vcm_actuator_table = {
	.act_type = ACTUATOR_VCM,
	.func_tbl = {
		.actuator_init_step_table = msm_actuator_init_step_table,
		.actuator_move_focus = msm_actuator_move_focus,
		.actuator_write_focus = msm_actuator_write_focus,
		.actuator_set_default_focus = msm_actuator_set_default_focus,
		.actuator_init_focus = msm_actuator_init_focus,
		.actuator_parse_i2c_params = msm_actuator_parse_i2c_params,
		.actuator_enable_ois = NULL,
	},
};

static struct msm_actuator msm_piezo_actuator_table = {
	.act_type = ACTUATOR_PIEZO,
	.func_tbl = {
		.actuator_init_step_table = NULL,
		.actuator_move_focus = msm_actuator_piezo_move_focus,
		.actuator_write_focus = NULL,
		.actuator_set_default_focus =
			msm_actuator_piezo_set_default_focus,
		.actuator_init_focus = msm_actuator_init_focus,
		.actuator_parse_i2c_params = msm_actuator_parse_i2c_params,
		.actuator_enable_ois = NULL,
	},
};

static struct msm_actuator msm_ois_actuator_table = {
	.act_type = ACTUATOR_OIS,
	.func_tbl = {
		.actuator_init_step_table = msm_actuator_init_step_table,
		.actuator_move_focus = msm_actuator_move_focus,
		.actuator_write_focus = msm_actuator_write_focus,
		.actuator_set_default_focus = msm_actuator_set_default_focus,
		.actuator_init_focus = msm_actuator_ois_init_focus,
		.actuator_parse_i2c_params = msm_actuator_ois_parse_i2c_params,
		.actuator_enable_ois = msm_actuator_ois_enable,
	},
};

module_init(msm_actuator_init_module);
MODULE_DESCRIPTION("MSM ACTUATOR");
MODULE_LICENSE("GPL v2");
