/*
 * ov680_flash_max8834.h
 *
 * Copyright (c) 2012-2013 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * PROPRIETARY/CONFIDENTIAL
 *
 * Use is subject to license terms.
 */


#ifndef OV680_FLASH_MAX8834_H
#define OV680_FLASH_MAX8834_H

#define MAX8834_DEVICE_ADDR 0x4A
#define MAX8834_DEVICE_ID   0x11

// MAX8834 register addresses
#define MAX8834_BOOST_CTRL 0x00
#define MAX8834_FLASH1_CUR 0x01
#define MAX8834_FLASH2_CUR 0x02
#define MAX8834_MOVIE_CUR  0x03
#define MAX8834_IND_CUR    0x05
#define MAX8834_IND_CTRL   0x07
#define MAX8834_LED_CTRL   0x09
#define MAX8834_TMR_DUR    0x0A
#define MAX8834_NTC_CTRL   0x0B
#define MAX8834_GSMB_CUR   0x0C
#define MAX8834_MAXFLASH1  0x0D
#define MAX8834_MAXFLASH2  0x0E
#define MAX8834_WDT_RST    0x16
#define MAX8834_STATUS1    0x17
#define MAX8834_STATUS2    0x18
#define MAX8834_CHIP_ID1   0x1A
#define MAX8834_CHIP_ID2   0x1B

// MAX8834_BOOST_CTRL
#define BOOST_EN   6                  // Enable boost converter
#define BOOST_MODE 4                  // Set boost mode
  #define BOOST_MODE_ADAPTIVE 0x0     // Adaptive output - changes on load
  #define BOOST_MODE_PROG     0x1     // Output set programmatically according to BOOST_CTRL[3:0]
  #define BOOST_MODE_DROPOUT  0x2     // Dropout mode
  #define BOOST_MODE_AUTO     0x3     // Automatic mode - combo of dropout/adaptive mode

#define BOOST_CTRL 0                  // Boost voltage (3.7V - 5.2V)
  #define BOOST_CTRL_MIN      3700
  #define BOOST_CTRL_MAX      5200
  #define BOOST_CTRL_MV_TO_REG(mv) ((uint8_t)((mv)-3700)/100)

// FLASHx_CUR
#define FLASHx 3  // FLEDx flash mode current (23.44mA - 750mA)
  #define FLASH_CUR_UA_TO_REG(ua) ((uint8_t)(((ua)==750000) ? 0x1F : ((ua)-23440)/23440))

// MOVIE_CUR
#define MOVIE1 4 // FLED1 Movie mode current (15.625mA - 125mA)
#define MOVIE2 0 // FLED1 Movie mode current (15.625mA - 125mA)
  #define MOVIE_CUR_UA_TO_REG(ma) ((uint8_t)(((ma)-15625)/15625))

// LED_CTRL
#define MOVIE_EN 3         // Movie mode output settings
#define FLASH_EN 0         // Flash mode output settings
  #define FLED1_EN    0x1  // Enable FLED1 output
  #define FLED2_EN    0x2  // Enable FLED2 output
  #define LED_EN_CTRL 0x4  // Output is controlled by LED_EN

// TMR_DUR
#define WDT_EN   7         // Enable watchdog timer
#define TMR_MODE 4         // Safety timer mode (0=oneshot, 1=maxtimer)
#define TMR_DUR  0         // Safety timer duration (50ms - 800ms)
  #define TMR_DUR_MS_TO_REG(ms) ((uint8_t)(((ms)-50)/50))

// NTC_CTRL
#define FLASH_TMR_CNTL 7   // 0=flash reset timer, 1=disable timer
#define NTC_EN         3   // Enable temperature regulator
#define NTC_THRESHOLD  0
  #define NTC_MV_TO_REG(mv) ((uint8_t)(((mv)-200)/50))

// GSMB_CUR
#define GSMB_EN   7        // Enable GSMB input
#define GSMB_POL  6        // Set GSMB polarity (0=active low, 1=active high)
#define GSMB_ILIM 2        // Input current limit during GSMB event (50mA - 800mA)
  #define GSMB_ILIM_CUR_MA_TO_REG(ma) ((uint8_t)(((ma)-50)/50))

// MAXFLASH1
#define LB_EN        7     // Enable low battery detection
#define LB_THRESHOLD 2     // Low battery threshold voltage
  // Subtract 1 from every 100mV to compensate for slightly uneven steps (33, 33, 34mV)
  #define LB_TH_MV_TO_REG(mv) ((uint8_t)((((mv)-2500)-((mv)-2500)/100) / 33) + 3)

// STATUS1
#define NTC_FLT    7  // NTC fault
#define GSMB_EVENT 6  // GSMB event occurred
#define POK_FLT    5  // POK fault occurred (output voltage drop)
#define OVER_TMP   4  // Die overtemp has occurred
#define NTC_OVT    3  // NTC overtemp has occurred
#define INDLED_FLT 2  // INDLED fault
#define FLED2_FLT  1  // FLED2 fault
#define FLED1_FLT  0  // FLED1 fault

// STATUS2
#define MAXFLASH_STAT   7  // MAXFLASH event has occurred
#define GSMB_ILIM_EVENT 6  // GSMB input current limit reached

#define NUM_MAX8834_DEVICES 2

#define MAX8834_BOOST_VOLTAGE 5000
/**
 * MAX8834 current limitations.
 */
enum {
	MAX8834_IRLED_CURRENT_MIN = 24,
	MAX8834_IRLED_CURRENT_MAX = 750,
};

#endif
