/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
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
 * Portions of the code were taken from OIS_func.c
 * Copyright(c)	Rohm Co.,Ltd. All rights reserved
 *
 */

#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include "msm_camera_io_util.h"
#include <linux/clk.h>
#include "msm_sd.h"
#include "msm_ois.h"
#include "msm_cci.h"
#include "bu63164.h"
#include "bu63164_fw.h"
#include "bu63164_coeff.h"

#define BU63164_NAME "qcom,bu63164"

//#define CONFIG_MSMB_CAMERA_DEBUG
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

#define MCLK2_GPIO 17
#define BLOCK_XFER_SIZE 8

#define EEPROM_SID 0x50
#define EEPROM_FACT_ADJ_START 0x0F20

#define VCOSET
#define OIS_CLK 12000000

DEFINE_MSM_MUTEX(bu63164_mut);

static struct msm_ois_ctrl_t fctrl;
static struct bu63164_fact_adj_t fact_adj_table;

static u16 vcm_value = 0;
static u32 cmd = 0;

// Wakes up DSP
static uint32_t bu63164_spl_cmd(void)
{
	uint8_t spl_cmd_array[1] = {0x01};
	int rc =0;

	fctrl.i2c_client.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	fctrl.i2c_client.cci_client->sid = 0x0E;

	rc = msm_camera_cci_i2c_write_seq(&fctrl.i2c_client, _OP_SpecialCMD, spl_cmd_array, 1);
	if(rc < 0) {
		pr_err("OIS special cmd failed\n");
		return rc;
	}
	return rc;
}

// Memory write
static uint32_t bu63164_mem_write(uint8_t reg, uint16_t data)
{
	int rc =0;
	uint8_t mem_write_array[2] = {0x00,0x00};

	fctrl.i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	fctrl.i2c_client.cci_client->sid = 0x0E;

	mem_write_array[0] = data & 0xff;
	mem_write_array[1] = data>>8 & 0xff;
	rc = msm_camera_cci_i2c_write_seq(&fctrl.i2c_client, _OP_Memory__RW << 8 | reg, mem_write_array, 2);
	if(rc < 0) {
		pr_err("OIS memory write failed\n");
		return rc;
	}
	return rc;
}

// Memory read
static uint32_t bu63164_mem_read(uint8_t reg, uint16_t *data)
{
	int rc =0;
	uint8_t tmp[2];

	fctrl.i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	fctrl.i2c_client.cci_client->sid = 0x0E;

	rc = msm_camera_cci_i2c_read_seq(&fctrl.i2c_client, (_OP_Memory__RW << 8) | reg, tmp, 2);
	if(rc < 0) {
		pr_err("OIS memory read failed\n");
	}

	*data = (tmp[0] << 8 | tmp[1]);
	return rc;
}

// Peripheral write
static uint32_t bu63164_per_write(uint8_t reg, uint16_t data)
{
	int rc =0;
	uint8_t per_write_array[2] = {0x00,0x00};

	fctrl.i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	fctrl.i2c_client.cci_client->sid = 0x0E;

	per_write_array[0] = data & 0xff;
	per_write_array[1] = data>>8 & 0xff;
	rc = msm_camera_cci_i2c_write_seq(&fctrl.i2c_client, _OP_Periphe_RW << 8 | reg, per_write_array, 2);
	if(rc < 0) {
		pr_err("OIS peripheral write failed\n");
		return rc;
	}
	return rc;
}

// Peripheral read
#if 0
static uint32_t bu63164_per_read(uint8_t reg, uint16_t *data)
{
	int rc =0;
	uint8_t tmp[2];

	fctrl.i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	fctrl.i2c_client.cci_client->sid = 0x0E;

	rc = msm_camera_cci_i2c_read_seq(&fctrl.i2c_client, (_OP_Periphe_RW << 8) | reg, tmp, 2);
	if(rc < 0) {
		pr_err("OIS peripheral read failed\n");
	}

	*data = (tmp[0] << 8 | tmp[1]);
	return rc;
}
#endif
// Reads 16-bit calibration values from the EEPROM
// TODO: Not do this here.
static uint32_t msm_ois_eeprom_read(struct msm_ois_ctrl_t *fctrl, uint16_t addr, uint16_t *buf)
{
	int rc = 0;
	uint16_t old_sid;
	uint8_t tmp[2];

	fctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	old_sid = fctrl->i2c_client.cci_client->sid;
	fctrl->i2c_client.cci_client->sid = EEPROM_SID;

	rc = msm_camera_cci_i2c_read_seq(&fctrl->i2c_client, addr, tmp, 2);

	fctrl->i2c_client.cci_client->sid = old_sid;
	*buf = (tmp[1] << 8) | tmp[0];
	return rc;
}

static uint32_t bu63164_fw_load(struct msm_ois_ctrl_t *fctrl, uint8_t *fw, uint16_t size, uint8_t op_code)
{
	int rc = 0;
	uint16_t xfer_cnt = 0;
	uint16_t bytes_remaining = size;

	fctrl->i2c_client.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	fctrl->i2c_client.cci_client->sid = 0x0E;

	for (; bytes_remaining > 0; xfer_cnt++) {
		uint8_t xfer_size = (bytes_remaining >= BLOCK_XFER_SIZE) ? BLOCK_XFER_SIZE : bytes_remaining;

		rc = msm_camera_cci_i2c_write_seq(&fctrl->i2c_client, op_code, fw+(xfer_cnt*BLOCK_XFER_SIZE), xfer_size);
		if(rc < 0) {
			pr_err("OIS FW load failed\n");
			break;
		}
		bytes_remaining -= xfer_size;
	}
	return rc;
}

// Recalibrates the digital gyro offsets.  Device needs to be still for this.
static void bu63164_dig_gyro_adjust(struct bu63164_fact_adj_t *param){

	int32_t s32_dat1;
	int32_t s32_dat2;
	uint16_t u16_i;
	uint16_t avr_GYRO;

	s32_dat1 = 0;
	s32_dat2 = 0;
	avr_GYRO = 2;
	for(u16_i=1; u16_i<=avr_GYRO; u16_i+=1){
		uint16_t u16_tmp_read1;
		uint16_t u16_tmp_read2;

		usleep(5000);
		bu63164_mem_read( _M_DigGx, &u16_tmp_read1 );
		bu63164_mem_read( _M_DigGy, &u16_tmp_read2 );
		pr_err("Gx:%05d   Gy:%05d\n", (int32_t)u16_tmp_read1,(int32_t)u16_tmp_read2);
		s32_dat1 = s32_dat1 + (int32_t)( u16_tmp_read1 );
		s32_dat2 = s32_dat2 + (int32_t)( u16_tmp_read2 );
	}
	pr_err("AvGx:%05d AvGy:%05d\n", s32_dat1 / avr_GYRO, s32_dat2 / avr_GYRO);
	param->gl_GX_OFS = (uint32_t)( s32_dat1 / avr_GYRO );
	param->gl_GY_OFS = (uint32_t)( s32_dat2 / avr_GYRO );

	bu63164_mem_write( _M_Kgx00, param->gl_GX_OFS );
	bu63164_mem_write( _M_Kgy00, param->gl_GY_OFS );
	pr_err("gl_GX_OFS:%04x\n", param->gl_GX_OFS );
	pr_err("gl_GY_OFS:%04x\n", param->gl_GY_OFS );
}

static void bu63164_set_scene_param(uint8_t u16_scene, uint8_t u16_mode, uint8_t filter, uint8_t range, struct bu63164_fact_adj_t *param)
{
	uint16_t u16_i;
	uint16_t u16_dat;
									// <---------  LPF for HPF  ----------> <--1/s--> <-PosLMT-> <-OmgLMT->
	uint8_t u16_adr_target[7]        = { _M_Kgx09, _M_Kgx0A, _M_Kgx0D, _M_Kgx0E, _M_Kgxdr, _M_X_LMT, _M_X_TGT,  };

	uint16_t u16_dat_SCENE_NIGHT_1[7] = { 0x0DEE,   0x7C85,   0x0DEE,   0x7C85,   0x7FF2,   0x0F00,   0x1900,    };	// 0.3 deg	16dps
	uint16_t u16_dat_SCENE_NIGHT_2[7] = { 0x1000,   0x7C00,   0x1000,   0x7C00,   0x7FF2,   0x0F00,   0x1900,    };	// 0.3 deg	16dps
	uint16_t u16_dat_SCENE_NIGHT_3[7] = { 0x1261,   0x7B68,   0x1261,   0x7B68,   0x7FF2,   0x0F00,   0x1900,    };	// 0.3 deg	16dps

	uint16_t u16_dat_SCENE_D_A_Y_1[7] = { 0x1BDC,   0x7909,   0x1BDC,   0x7909,   0x7FC0,   0x1400,   0x3E80,    };	// 0.4 deg	40dps
	uint16_t u16_dat_SCENE_D_A_Y_2[7] = { 0x2000,   0x7800,   0x2000,   0x7800,   0x7FC0,   0x1B80,   0x3E80,    };	// 0.55deg	40dps
	uint16_t u16_dat_SCENE_D_A_Y_3[7] = { 0x24C2,   0x76CF,   0x24C2,   0x76CF,   0x7FC0,   0x1B80,   0x3E80,    };	// 0.55deg	40dps

	uint16_t u16_dat_SCENE_SPORT_1[7] = { 0x37B7,   0x7212,   0x37B7,   0x7212,   0x7F50,   0x2300,   0x5DC0,    };	// 0.7 deg	60dps
	uint16_t u16_dat_SCENE_SPORT_2[7] = { 0x4000,   0x7000,   0x4000,   0x7000,   0x7F40,   0x2800,   0x6D60,    };	// 0.8 deg	70dps
	uint16_t u16_dat_SCENE_SPORT_3[7] = { 0x4984,   0x6D9F,   0x4984,   0x6D9F,   0x7F00,   0x2800,   0x6D60,    };	// 0.8 deg	70dps
	uint16_t u16_dat_SCENE_TEST___[7] = { 0x0DEE,   0x7C85,   0x0DEE,   0x7C85,   0x7FF2,   0x7FFF,   0x7FFF,    };	// Limmiter OFF

	uint16_t *u16_dat_SCENE_;

	uint8_t	size_SCENE_tbl = sizeof( u16_dat_SCENE_NIGHT_1 ) / sizeof(uint16_t);

	// Disable OIS ( position Servo is not disable )
	bu63164_mem_read( _M_EQCTL, &u16_dat );
	u16_dat = ( u16_dat &  0xFEFE );
	bu63164_mem_write( _M_EQCTL, u16_dat );

	// Scene parameter select
	switch( u16_scene ){
		case _SCENE_NIGHT_1 : u16_dat_SCENE_ = u16_dat_SCENE_NIGHT_1; break;
		case _SCENE_NIGHT_2 : u16_dat_SCENE_ = u16_dat_SCENE_NIGHT_2; break;
		case _SCENE_NIGHT_3 : u16_dat_SCENE_ = u16_dat_SCENE_NIGHT_3; break;
		case _SCENE_D_A_Y_1 : u16_dat_SCENE_ = u16_dat_SCENE_D_A_Y_1; break;
		case _SCENE_D_A_Y_2 : u16_dat_SCENE_ = u16_dat_SCENE_D_A_Y_2; break;
		case _SCENE_D_A_Y_3 : u16_dat_SCENE_ = u16_dat_SCENE_D_A_Y_3; break;
		case _SCENE_SPORT_1 : u16_dat_SCENE_ = u16_dat_SCENE_SPORT_1; break;
		case _SCENE_SPORT_2 : u16_dat_SCENE_ = u16_dat_SCENE_SPORT_2; break;
		case _SCENE_SPORT_3 : u16_dat_SCENE_ = u16_dat_SCENE_SPORT_3; break;
		case _SCENE_TEST___ : u16_dat_SCENE_ = u16_dat_SCENE_TEST___; break;
		default             : u16_dat_SCENE_ = u16_dat_SCENE_TEST___; break;
	}

	// Set parameter to the OIS controller
	for( u16_i = 0; u16_i < size_SCENE_tbl; u16_i += 1 ){
		bu63164_mem_write( u16_adr_target[u16_i],          	u16_dat_SCENE_[u16_i]   );
	}
	for( u16_i = 0; u16_i < size_SCENE_tbl; u16_i += 1 ){
		bu63164_mem_write( u16_adr_target[u16_i] + 0x80,	u16_dat_SCENE_[u16_i] );
	}

	// Set/Reset Notch filter
	if ( filter == 1 ) {
		bu63164_mem_read( _M_EQCTL, &u16_dat );
		u16_dat |= 0x4000;
		bu63164_mem_write( _M_EQCTL, u16_dat );

		bu63164_mem_write( _M_Kgx13, 0x5900 );
		bu63164_mem_write( _M_Kgx14, 0xC700 );
		bu63164_mem_write( _M_Kgy13, 0x5900 );
		bu63164_mem_write( _M_Kgy14, 0xC700 );
	}
	else{
		bu63164_mem_read( _M_EQCTL, &u16_dat );
		u16_dat &= 0xBFFF;
		bu63164_mem_write( _M_EQCTL, u16_dat );

		bu63164_mem_write( _M_Kgx13, 0x7470 );
		bu63164_mem_write( _M_Kgx14, 0xCB90 );
		bu63164_mem_write( _M_Kgy13, 0x7470 );
		bu63164_mem_write( _M_Kgy14, 0xCB90 );
	}

	// Clear the register of the OIS controller
	bu63164_mem_write( _M_wDgx02, 0x0000 );
	bu63164_mem_write( _M_wDgx03, 0x0000 );
	bu63164_mem_write( _M_wDgx06, 0x7FFF );
	bu63164_mem_write( _M_Kgx15,  0x0000 );

	bu63164_mem_write( _M_wDgy02, 0x0000 );
	bu63164_mem_write( _M_wDgy03, 0x0000 );
	bu63164_mem_write( _M_wDgy06, 0x7FFF );
	bu63164_mem_write( _M_Kgy15,  0x0000 );

	// Set the pre-Amp offset value (X and Y)
	if	( range == 1 ) {
		//bu63164_per_write( _P_31_ADC_CH1, param->gl_SFTHAL_X );
		//bu63164_per_write( _P_32_ADC_CH2, param->gl_SFTHAL_Y );
	}
	else{
		bu63164_per_write( _P_31_ADC_CH1, param->gl_HALOFS_X );
		bu63164_per_write( _P_32_ADC_CH2, param->gl_HALOFS_Y );
	}

	// Enable OIS (if u16_mode = 1)
	if(	( u16_mode == 1 ) ){
		bu63164_mem_read( _M_EQCTL, &u16_dat );
		u16_dat = ( u16_dat |  0x0101 );
		bu63164_mem_write( _M_EQCTL, u16_dat );
	}
}
#ifdef VCOSET
static struct msm_cam_clk_info ois_clk_info[] = {
	{"ois_src_clk", OIS_CLK},
};
void *clk_data;

static void bu63164_vcoset(void)
{
	uint16_t CLK_PS = OIS_CLK/1000;     // Input Frequency [kHz] of CLK/PS terminal (Depend on your system)     RHM_HT 2013.03.19       Change 6M -> 12M
	uint16_t FVCO_1 = 27000;     // Target Frequency [kHz]
	uint16_t FREF = 25;          // Reference Clock Frequency [kHz]

	uint16_t DIV_N = CLK_PS / FREF - 1;  // calc DIV_N
	uint16_t DIV_M = FVCO_1 / FREF - 1;  // calc DIV_M
	bu63164_per_write (0x62, DIV_N);      // Divider for internal reference clock
	bu63164_per_write (0x63, DIV_M);      // Divider for internal PLL clock
	bu63164_per_write (0x64, 0x4060);     // Loop Filter

	bu63164_per_write (0x60, 0x3011);     // PLL
	bu63164_per_write (0x65, 0x0080);     //
	bu63164_per_write (0x61, 0x8002);     // VCOON
	bu63164_per_write (0x61, 0x8003);     // Circuit ON
	bu63164_per_write (0x61, 0x8809);     // PLL ON

	msleep(30);                    // Wait for PLL lock

	bu63164_per_write (0x05, 0x000C);     // Prepare for PLL clock as master clock
	bu63164_per_write (0x05, 0x000D);     // Change to PLL clock
}
#endif

static int32_t bu63164_fact_adj_apply(struct msm_ois_ctrl_t *fctrl, struct bu63164_fact_adj_t *fact_adj_table)
{
	uint16_t eeprom_addr = EEPROM_FACT_ADJ_START;
	//*********************
	// HALL ADJUST
	//*********************
	msm_ois_eeprom_read(fctrl, eeprom_addr, &fact_adj_table->gl_CURDAT);
	bu63164_per_write( _P_30_ADC_CH0, fact_adj_table->gl_CURDAT );	// Hall Current DAC
	pr_err("gl_CURDAT %x\n", fact_adj_table->gl_CURDAT);
	eeprom_addr += 2;

	msm_ois_eeprom_read(fctrl, eeprom_addr, &fact_adj_table->gl_HALOFS_X);
	bu63164_per_write( _P_31_ADC_CH1,	fact_adj_table->gl_HALOFS_X );	// Pre Amp Offset
	pr_err("gl_HALOFS_X %x\n", fact_adj_table->gl_HALOFS_X);
	eeprom_addr += 2;

	msm_ois_eeprom_read(fctrl, eeprom_addr, &fact_adj_table->gl_HALOFS_Y);
	bu63164_per_write( _P_32_ADC_CH2,	fact_adj_table->gl_HALOFS_Y );	// Pre Amp Offset
	pr_err("gl_HALOFS_Y %x\n", fact_adj_table->gl_HALOFS_Y);
	eeprom_addr += 2;

	msm_ois_eeprom_read(fctrl, eeprom_addr, &fact_adj_table->gl_HX_OFS);
	bu63164_mem_write( _M_X_H_ofs,	fact_adj_table->gl_HX_OFS );	// PostAmp Offset
	pr_err("gl_HX_OFS %x\n", fact_adj_table->gl_HX_OFS);
	eeprom_addr += 2;

	msm_ois_eeprom_read(fctrl, eeprom_addr, &fact_adj_table->gl_HY_OFS);
	bu63164_mem_write( _M_Y_H_ofs,	fact_adj_table->gl_HY_OFS );
	pr_err("gl_HY_OFS %x\n", fact_adj_table->gl_HY_OFS);
	eeprom_addr += 2;

	msm_ois_eeprom_read(fctrl, eeprom_addr, &fact_adj_table->gl_PSTXOF);
	bu63164_per_write( _P_39_Ch3_VAL_1,	fact_adj_table->gl_PSTXOF );	// Residual Offset
	pr_err("gl_PSTXOF %x\n", fact_adj_table->gl_PSTXOF);
	eeprom_addr += 2;

	msm_ois_eeprom_read(fctrl, eeprom_addr, &fact_adj_table->gl_PSTYOF);
	bu63164_per_write( _P_3B_Ch3_VAL_3,	fact_adj_table->gl_PSTYOF);
	pr_err("gl_PSTYOF %x\n", fact_adj_table->gl_PSTYOF);
	eeprom_addr += 2;

	//*********************
	// DIGITAL GYRO OFFSET
	//*********************
	msm_ois_eeprom_read(fctrl, eeprom_addr, &fact_adj_table->gl_GX_OFS);
	bu63164_mem_write( _M_Kgx00,		fact_adj_table->gl_GX_OFS );
	pr_err("gl_GX_OFS %x\n", fact_adj_table->gl_GX_OFS);
	eeprom_addr += 2;

	msm_ois_eeprom_read(fctrl, eeprom_addr, &fact_adj_table->gl_GY_OFS);
	bu63164_mem_write( _M_Kgy00, fact_adj_table->gl_GY_OFS );
	pr_err("gl_GY_OFS %x\n", fact_adj_table->gl_GY_OFS);
	eeprom_addr += 2;

	//*********************
	// HALL SENSE
	//*********************
	msm_ois_eeprom_read(fctrl, eeprom_addr, &fact_adj_table->gl_KgxHG);
	bu63164_mem_write( _M_KgxHG, fact_adj_table->gl_KgxHG);	// X axis
	pr_err("gl_KgxHG %x\n", fact_adj_table->gl_KgxHG);
	eeprom_addr += 2;

	msm_ois_eeprom_read(fctrl, eeprom_addr, &fact_adj_table->gl_KgyHG);
	bu63164_mem_write( _M_KgyHG, fact_adj_table->gl_KgyHG );	// Y axis
	pr_err("gl_KgyHG %x\n", fact_adj_table->gl_KgyHG);
	eeprom_addr += 2;

	//*********************
	// LOOPGAIN
	//*********************
	msm_ois_eeprom_read(fctrl, eeprom_addr, &fact_adj_table->gl_KGXG);
	bu63164_mem_write( _M_KgxG,	fact_adj_table->gl_KGXG);
	pr_err("gl_KGXG %x\n", fact_adj_table->gl_KGXG);
	eeprom_addr += 2;

	msm_ois_eeprom_read(fctrl, eeprom_addr, &fact_adj_table->gl_KGYG);
	bu63164_mem_write( _M_KgyG,	fact_adj_table->gl_KGYG );
	pr_err("gl_KGYG %x\n", fact_adj_table->gl_KGYG);
	eeprom_addr += 2;

	// Position Servo ON ( OIS OFF )
	bu63164_mem_write( _M_EQCTL, 0x0C0C );
	return 0;
}

static int32_t bu63164_power_up(struct msm_ois_ctrl_t *fctrl)
{
	int rc = 0;
	uint16_t fw_size = sizeof(bu63164_fw)/sizeof(bu63164_fw[0]);
	uint16_t coeff_size = sizeof(bu63164_coeff)/sizeof(bu63164_coeff[0]);
	uint16_t sts;

	// I seriously should not be hardcoding this.
	rc = gpio_request(MCLK2_GPIO, "bu63164_ps");
	if (!rc) {
#ifndef VCOSET
		gpio_direction_output(MCLK2_GPIO, 1);
#endif
	} else {
		pr_err("gpio failed");
		goto bu63164_power_up_failed;
	}

#ifdef VCOSET
	rc = msm_cam_clk_enable(&fctrl->pdev->dev, &ois_clk_info[0],(struct clk **)&clk_data,1,1);
	if (rc < 0) {
		pr_err("msm_cam_clk_enable failed\n");
		goto bu63164_power_up_failed;
	}
#endif

	usleep(1000);
	rc = msm_sensor_cci_i2c_util(&fctrl->i2c_client, MSM_CCI_INIT);
	if (rc < 0) {
		pr_err("cci_init failed\n");
		goto bu63164_power_up_failed;
	}

	/* Download Firmware*/
	rc = bu63164_fw_load(fctrl, bu63164_fw, fw_size, _OP_FIRM_DWNLD);
	if (rc < 0) {
		pr_err("BU63164 FW load failed\n");
		goto bu63164_power_up_failed;
	}


	/* check STS status */
	bu63164_mem_read(_M_OIS_STS, &sts);
	if (!(sts & 0x04)) {
		pr_err("BU63164 FW failed to start\n");
		goto bu63164_power_up_failed;
	}

	/* Download Coefficients */
	rc = bu63164_fw_load(fctrl, bu63164_coeff, coeff_size, _OP_COEF_DWNLD);
	if (rc < 0) {
		pr_err("BU63164 coefficient load failed\n");
		goto bu63164_power_up_failed;
	}

	bu63164_mem_read(_M_FIRMVER, &sts);
	pr_err("BU63164 FW version: %04x\n", sts);

	bu63164_mem_read(_M_CEFTYP, &sts);
	pr_err("BU63164 coefficient version: %04x\n", sts);
#ifdef VCOSET
	bu63164_vcoset();
#endif

	/* Turn on/wake up DSP to start operation*/
	bu63164_spl_cmd();

	// Set factory adjustment parameters
	bu63164_fact_adj_apply(fctrl, &fact_adj_table);
	// Recalibrate gyro due to drift
	// TODO: Remove this.  It's going to be lots of trouble.
	bu63164_dig_gyro_adjust(&fact_adj_table);

	// Configure OIS, but leave disabled
	bu63164_set_scene_param(_SCENE_TEST___, 0, 0, 0, &fact_adj_table);

bu63164_power_up_failed:
	return rc;
}

static int32_t bu63164_power_down(struct msm_ois_ctrl_t *fctrl)
{
	int rc = 0;

	rc = msm_sensor_cci_i2c_util(&fctrl->i2c_client, MSM_CCI_RELEASE);

	gpio_free(MCLK2_GPIO);
#ifdef VCOSET
	rc = msm_cam_clk_enable(&fctrl->pdev->dev, &ois_clk_info[0],(struct clk **)&clk_data,1,0);
#endif

	return rc;
}

static int32_t bu63164_ois_enable(struct msm_ois_ctrl_t *fctrl, uint8_t enable)
{
	int rc = 0;
	uint16_t data;
	if (enable) {
		bu63164_mem_read( _M_EQCTL, &data );
		data = ( data |  0x0101 );
		bu63164_mem_write( _M_EQCTL, data );
	} else {
		bu63164_mem_read( _M_EQCTL, &data );
		data = ( data &  ~0x0101 );
		bu63164_mem_write( _M_EQCTL, data );
	}
	return rc;
}

static int bu63164_cmd_set(void *data, u64 val)
{
	int rc;
	uint8_t vcm_write_array[4] = {0x90,0x00,0x00,0x00};

	vcm_write_array[2] = vcm_value>>8 & 0xff;
	vcm_write_array[3] = vcm_value & 0x00ff;

	fctrl.i2c_client.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	fctrl.i2c_client.cci_client->sid = 0x0E;

	rc = msm_sensor_cci_i2c_util(&fctrl.i2c_client, MSM_CCI_INIT);
	rc = msm_camera_cci_i2c_write_seq(&fctrl.i2c_client, _OP_FX_Cmnd_0, vcm_write_array, 4);
	if(rc < 0) {
		pr_err("VCM write failed\n");
		return 0;
	}
	rc = msm_sensor_cci_i2c_util(&fctrl.i2c_client, MSM_CCI_RELEASE);

	return 0;
}

static int bu63164_cmd_get(void *data, u64 * val)
{
	printk("%s \n", __func__);
	*val = *(u32 *) data;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(bu63164_cmd_fops, bu63164_cmd_get,
                        bu63164_cmd_set, "%llu\n");

struct dentry *mydebugfs_create_cmd32(const char *name, mode_t mode,
                                      struct dentry *parent, u32 * value)
{
	/* if there are no write bits set, make read only */
	if (!(mode & S_IWUGO))
		return debugfs_create_file(name, mode, parent, value,
								  &bu63164_cmd_fops);
	/* if there are no read bits set, make write only */
	if (!(mode & S_IRUGO))
		return debugfs_create_file(name, mode, parent, value,
								   &bu63164_cmd_fops);

	return debugfs_create_file(name, mode, parent, value,
							   &bu63164_cmd_fops);
}


void bu63164_debugfs(void)
{
	struct dentry *ois_dir;
	ois_dir = debugfs_create_dir("ois_ctrl", NULL);

	if (!ois_dir) {
		printk("Unable to create debugfs directory \n");
		return;
	} else {
		mydebugfs_create_cmd32("cmd", 0666, ois_dir, &cmd);
		debugfs_create_x16("vcm_value", 0666, ois_dir, &vcm_value);
	}
}

static const struct of_device_id bu63164_dt_match[] = {
	{.compatible = "qcom,bu63164",
	 .data = &fctrl},
	{}
};

MODULE_DEVICE_TABLE(of, bu63164_dt_match);

static struct platform_driver bu63164_driver = {
	.driver = {
		.name = BU63164_NAME,
		.owner = THIS_MODULE,
		.of_match_table = bu63164_dt_match,
	},
};

static int32_t bu63164_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(bu63164_dt_match, &pdev->dev);
	rc = msm_ois_probe(pdev, match->data);
	bu63164_debugfs();
	return rc;
}

static int __init bu63164_add_driver(void)
{
	return platform_driver_probe(&bu63164_driver,
		bu63164_platform_probe);
}

static struct msm_ois_fn_t bu63164_func_tbl = {
	.ois_power_up = bu63164_power_up,
	.ois_power_down = bu63164_power_down,
	.ois_enable = bu63164_ois_enable,
};

static struct msm_ois_ctrl_t fctrl = {
	.func_tbl = &bu63164_func_tbl,
	.msm_ois_mutex = &bu63164_mut,
};

module_init(bu63164_add_driver);
MODULE_DESCRIPTION("BU63164 OIS");
MODULE_LICENSE("GPL v2");
