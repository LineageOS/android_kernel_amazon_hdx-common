/*
 * ov680_flash_ozl003.h
 *
 * Copyright (c) 2012-2013 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * PROPRIETARY/CONFIDENTIAL
 *
 * Use is subject to license terms.
 */


#ifndef OV680_FLASH_OZL003_H
#define OV680_FLASH_OZL003_H

#define OZL003_DEVICE_ADDR0 0x4A
#define OZL003_DEVICE_ADDR1 0x4B

#define OZL003_DEVICE_ID   0x80

// OZL003 register addresses
#define OZL003_LED_CTRL 0x00
#define OZL003_ISEN1    0x01
#define OZL003_ISEN2    0x02
#define OZL003_WDT      0x03
#define OZL003_STATUS   0x04
#define OZL003_PROT     0x05

#define OZLOO3_BOOST_VOLTAGE 4800000
#define OZL003_PROT_SETTINGS 0xD4

// OZL003_LED_CTRL
#define OZ_VLED 3             // Boost voltage (2.0V - 4.8V)
  #define OZ_BOOST_CTRL_MIN      2087500
  #define OZ_BOOST_CTRL_MAX      4800000
  #define OZ_BOOST_CTRL_UV_TO_REG(uv) ((uint8_t)(((uv)-2087500)/87500))

#define OZ_MODE 2                // Set boost mode
  #define OZ_MODE_ADAPTIVE 0     // Adaptive output - changes on load
  #define OZ_MODE_FIXED    1     // Fixed output - based on VLED[7:3]
#define OZ_IC_ENA 1
#define OZ_CNTRL  0
  #define OZ_CNTRL_STROBE 0      // LED on/off is controlled via external strobe
  #define OZ_CNTRL_I2C    1      // LED on/off is controlled via i2c command

// OZL003_ISENx
#define OZ_ISENx 0  // FLEDx flash mode current (0mA to 250mA)
  #define OZ_FLASH_CUR_UA_TO_REG(ua) ((uint8_t)((ua)/1920))

// OZL003_WDT
#define OZ_TMR_DUR  0         // Safety timer duration
  #define OZ_TMR_DUR_MS_TO_REG(ms) ((uint8_t)(((ms))/100))

#endif
