/**
*\mainpage
* AD7146 Controller Driver
\n
* @copyright 2013 Analog Devices Inc.
\n
* Licensed under the GPL version 2 or later.
* \date      July-2013
* \version   Driver 1.2.2-lab126
* \version   Android ICS 4.0
* \version   Linux 3.0.15
*/

/**
* \file ad7146.c
* This file is the core driver part of AD7146 with Event interface.
 It also has routines for interrupt handling for
* Sensor active and Convertion complete interrupt modes,
* suspend, resume, initialization routines etc.
* AD7146 Controller Driver
* Copyright 2013 Analog Devices Inc.
* Licensed under the GPL version 2 or later.
*/

#include <linux/device.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/kfifo.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/pm.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <asm/uaccess.h>
#include <asm/system.h>
#include <linux/input/ad7146.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/wakelock.h>
#include <linux/earlysuspend.h>
#include <linux/suspend.h>
#endif
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>

//#define SENSOR_ACTIVE_ON
#define CONV_COMP_ON

/*
#define AD7146_DEBUG
*/

#ifdef AD7146_DEBUG
#define AD7146_Driver_Dbg(format, arg...)	printk(KERN_INFO "[AD7146] :" format , ## arg)
#else
#define AD7146_Driver_Dbg(format, arg...)	if (0)
#endif

#define AD7146_Driver_Info(format, arg...)      printk(KERN_INFO "[AD7146] :" format , ## arg)
/**
*	The global mutex for the locking of the ISR.
*/
DEFINE_MUTEX(interrupt_thread_mutex);


typedef int (*ad7146_read_t)(struct device *, unsigned short, unsigned short *);
typedef int (*ad7146_write_t)(struct device *, unsigned short, unsigned short);
/**
Sensor active value
*/
#define SENSOR_ACTIVE_INT	(1)
/**
Conversion complete value
*/
#define CONV_COMP_INT		(0)
/**
Driver name of this ad7146 driver
*/
#define DRIVER_NAME "ad7146"
/**
 The Configuration File name and location in the System.
 This will be used for the Initialization by hardware_initialization
 and also  in Run time by "filp_config Sysfs attribute.
 */
#define AD7146_CONFIG_FILE "/data/misc/ad7146cfg.cfg"
/**
This hold the product ID of AD7146
*/
#define AD7146_PRODUCT_ID (0x7146)
/**
Low Bit Mask
*/
#define LOW_MASK(s) (0 << s)
/**
High Bit Mask
*/
#define HIGH_MASK(s) (1 << s)
/**
The Stage count of AD7146 chip.
*/
#define STAGE_NUM              12
/**
Ad7146 shutdown mask used in power management
*/
#define AD7146_SHUTDOWN_MASK (0x03)
#define AD7146_PWRUP_MASK (0xFC)
/**
\def AD7146_PWR_CTRL
Power control Register
*/
#define AD7146_PWR_CTRL			(0x0)
/**
\def AD7146_STG_CAL_EN_REG
Calibration and Control Register
*/
#define AD7146_STG_CAL_EN_REG		(0x1)
/**
\def AD7146_AMB_COMP_CTRL0_REG
Device Control Register 0
*/
#define AD7146_AMB_COMP_CTRL0_REG	(0x2)
/**
\def AD7146_PARTID_REG
Device ID Register Address
*/
#define AD7146_PARTID_REG		(0x17)
/**
\def AD7146_PARTID
Device ID
*/
#define AD7146_PARTID			(0x1490)
/**
\def AD7146_STAGECFG_REG
Stage Configuration Address
*/
#define AD7146_STAGECFG_REG		(0x80)
/**
\def AD7146_SYSCFG_REG
System config Register Address
*/
#define AD7146_SYSCFG_REG		(0x0)
/**
\def STG_LOW_INT_EN_REG
lower Threshold Enable register
*/
#define STG_LOW_INT_EN_REG		(0x5)
/**
\def STG_HIGH_INT_EN_REG
Higher Threshold Enable register
*/
#define STG_HIGH_INT_EN_REG		(0x6)
/**
\def STG_COM_INT_EN_REG
Conversion complete Interrupt Enable register
*/
#define STG_COM_INT_EN_REG		(0x7)
/**
\def STG_LOW_INT_STA_REG
lower Threshold Status register
*/
#define STG_LOW_INT_STA_REG		(0x8)
/**
\def STG_HIGH_INT_STA_REG
Higher Threshold Status register
*/
#define STG_HIGH_INT_STA_REG	(0x9)
/**
\def STG_COM_INT_STA_REG
Conversion complete Interrupt Status register
*/
#define STG_COM_INT_STA_REG		(0xA)
/**
\def CDC_RESULT_S0
Register address of Stage 1 CDC result
*/
#define CDC_RESULT_S0			(0xB)
/**
\def CDC_RESULT_S1
Register address of Stage 2 CDC result
*/
#define CDC_RESULT_S1			(0xC)
/**
\def CDC_RESULT_S2
Register address of Stage 3 CDC result
*/
#define CDC_RESULT_S2			(0xD)
/**
\def CDC_RESULT_S3
Register address of Stage 4 CDC result
*/
#define CDC_RESULT_S3			(0xE)
/**
\def CDC_RESULT_S4
Register address of Stage 5 CDC result
*/
#define CDC_RESULT_S4			(0xF)
/**
\def CDC_RESULT_S5
Register address of Stage 6 CDC result
*/
#define CDC_RESULT_S5			(0x10)
/**
\def CDC_RESULT_S6
Register address of Stage 7 CDC result
*/
#define CDC_RESULT_S6			(0x11)
/**
\def CDC_RESULT_S7
Register address of Stage 8 CDC result
*/
#define CDC_RESULT_S7			(0x12)
/**
\def CDC_RESULT_S8
Register address of Stage 9 CDC result
*/
#define CDC_RESULT_S8			(0x13)
/**
\def CDC_RESULT_S9
Register address of Stage 10 CDC result
*/
#define CDC_RESULT_S9			(0x14)
/**
\def CDC_RESULT_S10
Register address of Stage 11 CDC result
*/
#define CDC_RESULT_S10			(0x15)
/**
\def CDC_RESULT_S11
Register address of Stage 12 CDC result
*/
#define CDC_RESULT_S11			(0x16)
/**
Mask For the Force calibration
*/
#define AD7146_FORCED_CAL_MASK  HIGH_MASK(14)
/**
The maximum amount of data to be stored in the FILP data Buffer.
*/
#define MAX_FILP_DATA_CNT (128)
/**
The buffer size of the filp initialization buffer.
MAX_FILP_DATA_CNT Integers to be stored
*/
#define FILP_BUFF_SIZE ((MAX_FILP_DATA_CNT)*(sizeof(unsigned int)))
/**
The Read line Buffer size in the configuration
*/
#define VFS_READ_SIZE (64)
/**
General Disable macro for AD7146
*/
#define DISABLE_AD7146 (0)
/**
General Enable macro for AD7146
*/
#define ENABLE_AD7146 (1)
/**
Register Read size
*/
#define REG_READ_SIZE	(16)
/**
Retry attempt on no data in line in the Filp reader
*/
#define RETRY_ATTEMPT_READ (20)
/**
Skip Count for the Reading of the Data upon address access
*/
#define SKIP_DATA_CNT	(6)
/**
Minimum number of characters to Read in filp
*/
#define MIN_CHAR_TO_READ (8)
/**
Minimum Force Calibration sleep in the driver
*/
#define MIN_FORCED_CAL_SLEEP (20)
/**
Maximum Force Calibration sleep in the driver
*/
#define MAX_FORCED_CAL_SLEEP (50)
/**
Hexa-decimal Number Base Value
*/
#define HEX_BASE (16)
/**
Decimal Base Value
 */
#define DECIMAL_BASE (10)
/**
 * I2C Transfer Length for a single Read operation
 */
#define I2C_READ_LEN (2)
/**
 * I2C Transfer Length for a single Write operation
 */
#define I2C_WRITE_LEN (4)

#ifdef CONFIG_ARCH_MSM8974_APOLLO
#define MAX_AFE (1024*14)
#define MIN_AFE (1024*6)
#else
#define MAX_AFE (1024*29)
#define MIN_AFE (1024*24)
#endif
/**
* Driver information which will be used to maintain the software flow
*/
enum ad7146_device_state { IDLE, ACTIVE};
enum ad7146_state { SUSPEND, RESUME, INIT };
enum ad7146_status { DISABLED, ENABLED };
/**
This holds the Stage connection register addresses for the 12 stages
*/
unsigned short u16_StageConnRegister[STAGE_NUM*2] = {
0x0080,/* STAGE0_CONNECTION_6_0*/
0x0081,/* STAGE0_CONNECTION_12_7*/
0x0088,/* STAGE1_CONNECTION_6_0*/
0x0089,/* STAGE1_CONNECTION_12_7*/
0x0090,/* STAGE2_CONNECTION_6_0*/
0x0091,/* STAGE2_CONNECTION_12_7*/
0x0098,/* STAGE3_CONNECTION_6_0*/
0x0099,/* STAGE3_CONNECTION_12_7*/
0x00A0,/* STAGE4_CONNECTION_6_0*/
0x00A1,/* STAGE4_CONNECTION_12_7*/
0x00A8,/* STAGE5_CONNECTION_6_0*/
0x00A9,/* STAGE5_CONNECTION_12_7*/
0x00B0,/* STAGE6_CONNECTION_6_0*/
0x00B1,/* STAGE6_CONNECTION_12_7*/
0x00B8,/* STAGE7_CONNECTION_6_0*/
0x00B9,/* STAGE7_CONNECTION_12_7*/
0x00C0,/* STAGE8_CONNECTION_6_0*/
0x00C1,/* STAGE8_CONNECTION_12_7*/
0x00C8,/* STAGE9_CONNECTION_6_0*/
0x00C9,/* STAGE9_CONNECTION_12_7*/
0x00D0,/* STAGE10_CONNECTION_6_0*/
0x00D1,/* STAGE10_CONNECTION_12_7*/
0x00D8,/* STAGE11_CONNECTION_6_0*/
0x00D9,/* STAGE11_CONNECTION_12_7*/
};

unsigned short u16_StageAfeRegister[] = {
  0x0082,/* STAGE0_AFE_OFFSET*/
  0x008A,/* STAGE1_AFE_OFFSET*/
  0x0092,/* STAGE2_AFE_OFFSET*/
  0x009A,/* STAGE3_AFE_OFFSET*/
  0x00A2,/* STAGE4_AFE_OFFSET*/
  0x00AA,/* STAGE5_AFE_OFFSET*/
  0x00B2,/* STAGE6_AFE_OFFSET*/
  0x00BA,/* STAGE7_AFE_OFFSET*/
  0x00C2,/* STAGE8_AFE_OFFSET*/
  0x00CA,/* STAGE9_AFE_OFFSET*/
  0x00D2,/* STAGE10_AFE_OFFSET*/
  0x00DA,/* STAGE11_AFE_OFFSET*/
};

unsigned short u16_ConversionDataRegister[] = {
  CDC_RESULT_S0,/* STAGE0_CDC*/
  CDC_RESULT_S1,/* STAGE1_CDC*/
  CDC_RESULT_S2,/* STAGE2_CDC*/
  CDC_RESULT_S3,/* STAGE3_CDC*/
  CDC_RESULT_S4,/* STAGE4_CDC*/
  CDC_RESULT_S5,/* STAGE5_CDC*/
  CDC_RESULT_S6,/* STAGE6_CDC*/
  CDC_RESULT_S7,/* STAGE7_CDC*/
  CDC_RESULT_S8,/* STAGE8_CDC*/
  CDC_RESULT_S9,/* STAGE9_CDC*/
  CDC_RESULT_S10,/* STAGE10_CDC*/
  CDC_RESULT_S11,/* STAGE11_CDC*/
};

struct sensor_regulator {
  struct regulator *vreg;
  const char *name;
  u32 min_uV;
  u32 max_uV;
};

struct sensor_regulator adi7146_vcc_vreg[] = {
  {NULL, "vcc", 2600000, 3600000}, /* 2.85V */
};

struct sensor_regulator adi7146_vdrive_vreg[] = {
  {NULL, "vdrive", 1800000, 1800000},
};

/**
This structure holds the driver data of AD7146
*/
struct ad7146_driver_data {
	enum ad7146_device_state state;
	unsigned short index;
};

static struct ad7146_platform_data ad7146_i2c_platform_data = {
  .regs =  {
#ifdef CONFIG_ARCH_MSM8974_APOLLO
	0x00801FEF, 0x00813FFF, 0x00821B15, 0x00832626,
	0x00840064, 0x00850064, 0x00860064, 0x00870064,
	0x00883BFF, 0x00893FFD, 0x008A0D06, 0x008B2626,
	0x008C0064, 0x008D0064, 0x008E0078, 0x008F0078,
	0x00903FFF, 0x00913DEF, 0x00920F07, 0x00932626,
	0x00940064, 0x00950064, 0x00960078, 0x00970078,
	0x00983F7B, 0x00993FFF, 0x009A3327, 0x009B2626,
	0x009C0064, 0x009D0064, 0x009E0078, 0x009F0078,
	0x00a03FFF, 0x00a11FFF, 0x00a20000, 0x00a32626,
	0x00a40064, 0x00a50064, 0x00a60078, 0x00a70078,
	0x00a8FFFF, 0x00a93FFF, 0x00aa0000, 0x00ab2626,
	0x00ac0064, 0x00ad0064, 0x00ae0078, 0x00af0078,
	0x00b0FFFF, 0x00b13FFF, 0x00b20000, 0x00b32626,
	0x00b40064, 0x00b50064, 0x00b60078, 0x00b70078,
	0x00b8FFFF, 0x00b93FFF, 0x00ba0000, 0x00bb4624,
	0x00bc0064, 0x00bd0064, 0x00be0078, 0x00bf0078,
	0x00c0FFFF, 0x00c13FFF, 0x00c20000, 0x00c34646,
	0x00c40064, 0x00c50064, 0x00c60788, 0x00c70078,
	0x00c8FFFF, 0x00c93FFF, 0x00ca0000, 0x00cb4646,
	0x00cc0258, 0x00cd0258, 0x00ce0258, 0x00cf0258,
	0x00d0FFFF, 0x00d13FFF, 0x00d20000, 0x00d34646,
	0x00d40258, 0x00d50258, 0x00d60258, 0x00d70258,
	0x00d8FFFF, 0x00d93FFF, 0x00da0000, 0x00db4646,
	0x00dc0258, 0x00dd0258, 0x00de0258, 0x00df0258,
	0x0000C040, 0x00023230, 0x00030419, 0x00040832,
	0x00050000, 0x00060000, 0x00070008, 0x00450D00,
	0x0001000F,
#else
#ifdef CONFIG_ARCH_MSM8974_THOR
	0x00803FFE, 0x00811FFF, 0x00822000, 0x00832626,
	0x00840064, 0x00850064, 0x00860078, 0x00870078,
	0x00883EFF, 0x00891FFF, 0x008A1A00, 0x008B2626,
	0x008C0064, 0x008D0064, 0x008E0078, 0x008F0078,
	0x00903FFF, 0x00911EFF, 0x00921800, 0x00932626,
	0x00940064, 0x00950064, 0x00960078, 0x00970078,
	0x00983FFF, 0x00991BFF, 0x009A1B00, 0x009B2626,
	0x009C0064, 0x009D0064, 0x009E0078, 0x009F0078,
	0x00a03FFF, 0x00a11FBF, 0x00a21900, 0x00a32626,
	0x00a40258, 0x00a50258, 0x00a60258, 0x00a70258,
	0x00a83FFF, 0x00a91FFF, 0x00aa0006, 0x00ab4646,
	0x00ac0258, 0x00ad0258, 0x00ae0258, 0x00af0258,
	0x00b0FFFF, 0x00b13FFF, 0x00b20000, 0x00b32626,
	0x00b40258, 0x00b50258, 0x00b60258, 0x00b70258,
	0x00b8FFFF, 0x00b93FFF, 0x00ba0000, 0x00bb2626,
	0x00bc0258, 0x00bd0258, 0x00be0258, 0x00bf0258,
	0x00c0FFFF, 0x00c13FFF, 0x00c20000, 0x00c32636,
	0x00c40258, 0x00c50258, 0x00c60258, 0x00c70258,
	0x00c8FFFF, 0x00c93FFF, 0x00ca0000, 0x00cb2626,
	0x00cc0258, 0x00cd0258, 0x00ce0258, 0x00cf0258,
	0x00d0FFFF, 0x00d13FFF, 0x00d20000, 0x00d34646,
	0x00d40258, 0x00d50258, 0x00d60258, 0x00d70258,
	0x00d8FFFF, 0x00d93FFF, 0x00da0000, 0x00db4646,
	0x00dc0258, 0x00dd0258, 0x00de0258, 0x00df0258,
	0x0000C060, 0x00023230, 0x00030419, 0x00040832,
	0x00050000, 0x00060000, 0x00070008, 0x00450D00,
	0x0001000F,
#endif
#endif
  },
};

#define D_M (1000)
/**
This is used to hold the temperature compensation factors
namely gain and offset.
*/
struct channeltemp_t {
	int s32_gain;
	int s32_offset;
};
/**
  This array holds the gain and offset settings for all stages
*/
static struct channeltemp_t init_tcomp_data[STAGE_NUM] = {
  {0*D_M, 0}, {0*D_M, 0}, {0*D_M, 0},
  {0*D_M, 0}, {0*D_M, 0}, {0*D_M, 0},
  {0*D_M, 0}, {0*D_M, 0}, {0*D_M, 0},
  {0*D_M, 0}, {0*D_M, 0}, {0*D_M, 0}
};

/**
 * This structure provides chip information of AD7146.
 * \note Contains chip information of the AD7146 chip with attributes
 * like Product, Status ,version,drive data etc.
 * which are used in the control or to read the status of the device
 */
struct ad7146_chip {
	unsigned short high_status;
	unsigned short low_status;
	unsigned short complete_status;
	unsigned short high_int_enable;
	unsigned short low_int_enable;
	unsigned short com_int_enable;
	unsigned short sensor_int_enable;
	unsigned short complete_enable;
	unsigned short power_reg_value;
	unsigned short No_of_Stages;
	unsigned short temp_comp_en;
	unsigned short filp_buff_cnt;
	unsigned short temperature_comp_data;
	unsigned short sensor_val[STAGE_NUM];
	unsigned char LastStageEnabled;
	unsigned int intr_mode;
	struct ad7146_platform_data *hw;
	struct ad7146_driver_data *sw;
	int irq;
        int irq_gpio;
	char    reg_data[REG_READ_SIZE];
	struct device *dev;
	struct input_dev *input;
	struct work_struct work;
	ad7146_read_t read;
	ad7146_write_t write;
	struct channeltemp_t ast_chtempdata[STAGE_NUM];
	struct mutex mutex; /*Mutex for AD7146 Chip*/
	int *filp_buffer;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct  early_suspend    early_suspend;
#endif
        enum ad7146_status status;
        enum ad7146_state state;
	unsigned product;
	unsigned version;
};

/* SYSFS Attribute declaration & Initialization */
/*--------------------------------------------------------------*/
static ssize_t store_reg_read(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
static ssize_t show_reg_read(struct device *dev,
	struct device_attribute *attr, char *buf);
static DEVICE_ATTR(reg_read, S_IRUGO | S_IWUSR, show_reg_read, store_reg_read);
/*--------------------------------------------------------------*/
/*--------------------------------------------------------------*/
static ssize_t store_reg_write(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
static  DEVICE_ATTR(reg_write, S_IRUGO | S_IWUSR, NULL, store_reg_write);
/*--------------------------------------------------------------*/

/*--------------------------------------------------------------*/
static ssize_t store_temp_comp(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
static ssize_t show_temp_comp(struct device *dev,
	struct device_attribute *attr, char *buf);
static DEVICE_ATTR(temp_comp, S_IRUGO | S_IWUSR, show_temp_comp, store_temp_comp);
/*--------------------------------------------------------------*/

#ifdef SENSOR_ACTIVE_ON
#ifdef CONV_COMP_ON
static ssize_t store_intr_mode(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
static ssize_t show_intr_mode(struct device *dev,
	struct device_attribute *attr, char *buf);
static DEVICE_ATTR(intr_mode, S_IRUGO | S_IWUSR, show_intr_mode, store_intr_mode);
/*--------------------------------------------------------------*/
#endif
#endif
/*--------------------------------------------------------------*/
static ssize_t redo_filp_calib(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
static DEVICE_ATTR(filp_config, S_IRUGO | S_IWUSR, NULL, redo_filp_calib);
/*--------------------------------------------------------------*/
static ssize_t store_stage_info(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
static ssize_t show_stage_info(struct device *dev,
	struct device_attribute *attr, char *buf);
static DEVICE_ATTR(stage_info, S_IRUGO | S_IWUSR, show_stage_info, store_stage_info);
/*--------------------------------------------------------------*/
static ssize_t do_calibrate(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
static DEVICE_ATTR(calibrate, S_IRUGO | S_IWUSR, NULL, do_calibrate);
/*--------------------------------------------------------------*/
static ssize_t store_status(struct device *dev,
                            struct device_attribute *attr, const char *buf, size_t count);
static ssize_t show_status(struct device *dev,
                           struct device_attribute *attr, char *buf);
static DEVICE_ATTR(status, S_IRUGO | S_IWUSR, show_status, store_status);
static int ad7146_enable(struct ad7146_chip *ad7146);
static int ad7146_disable(struct ad7146_chip *ad7146);
/*--------------------------------------------------------------*/
static ssize_t do_autotune(struct device *dev,
                           struct device_attribute *attr, const char *buf, size_t count);
static ssize_t show_autotune(struct device *dev,
                           struct device_attribute *attr, char *buf);
static DEVICE_ATTR(autotune, S_IRUGO | S_IWUSR, show_autotune, do_autotune);
/*--------------------------------------------------------------*/
static ssize_t do_davetune(struct device *dev,
                           struct device_attribute *attr, const char *buf, size_t count);
static DEVICE_ATTR(davetune, S_IRUGO | S_IWUSR, show_autotune, do_davetune);
/*--------------------------------------------------------------*/
static struct attribute *ad7146_sysfs_entries[] = {
	&dev_attr_reg_read.attr,
	&dev_attr_reg_write.attr,
	&dev_attr_temp_comp.attr,
	&dev_attr_stage_info.attr,
	&dev_attr_calibrate.attr,
	&dev_attr_filp_config.attr,
#ifdef SENSOR_ACTIVE_ON
#ifdef CONV_COMP_ON
	&dev_attr_intr_mode.attr,
#endif
#endif
        &dev_attr_status.attr,
        &dev_attr_autotune.attr,
        &dev_attr_davetune.attr,
	NULL
};

static struct attribute_group ad7146_attr_group = {
	.name = NULL,
	.attrs = ad7146_sysfs_entries,
};

static int adi7146_config_regulator(struct i2c_client *client, bool on, struct sensor_regulator *adi7146_vreg)
{
  int rc = 0, i;
  int num_reg = 1;

  if (on) {
    for (i = 0; i < num_reg; i++) {
      if (adi7146_vreg[i].vreg == NULL){
        adi7146_vreg[i].vreg = regulator_get(&client->dev,
                                             adi7146_vreg[i].name);
        if (IS_ERR(adi7146_vreg[i].vreg)) {
          rc = PTR_ERR(adi7146_vreg[i].vreg);
          pr_err("%s:regulator get failed rc=%d\n",
                 __func__, rc);
          adi7146_vreg[i].vreg = NULL;
          goto error_vdd;
        }

        if (regulator_count_voltages(adi7146_vreg[i].vreg) > 0) {
          rc = regulator_set_voltage(adi7146_vreg[i].vreg,
                                     adi7146_vreg[i].min_uV, adi7146_vreg[i].max_uV);
          if (rc) {
            pr_err("%s:set_voltage failed rc=%d\n",
                   __func__, rc);
            regulator_put(adi7146_vreg[i].vreg);
            adi7146_vreg[i].vreg = NULL;
            goto error_vdd;
          }
        }

        rc = regulator_enable(adi7146_vreg[i].vreg);

        pr_info("%s: enable regulater %s rc = %d\n", __func__,
                adi7146_vreg[i].name, rc);
        if (rc) {
          pr_err("%s: regulator_enable failed rc =%d\n",
                 __func__,
                 rc);

          if (regulator_count_voltages(
                                       adi7146_vreg[i].vreg) > 0) {
            regulator_set_voltage(adi7146_vreg[i].vreg,
                                  0, adi7146_vreg[i].max_uV);
          }
          regulator_put(adi7146_vreg[i].vreg);
          adi7146_vreg[i].vreg = NULL;
          goto error_vdd;
        }
      }
    }
    return rc;
  } else {
    i = num_reg;
  }
 error_vdd:
  while (--i >= 0) {
    if (!IS_ERR_OR_NULL(adi7146_vreg[i].vreg)) {
      if (regulator_count_voltages(
                                   adi7146_vreg[i].vreg) > 0) {
        rc = regulator_set_voltage(adi7146_vreg[i].vreg, 0,
                                   adi7146_vreg[i].max_uV);
        if (rc)
          pr_err("%s:set_voltage failed rc=%d\n",__func__, rc);
      }
      regulator_disable(adi7146_vreg[i].vreg);
      regulator_put(adi7146_vreg[i].vreg);
      adi7146_vreg[i].vreg = NULL;
    }
  }
  return rc;
}
/*--------------------------------------------------------------*/

/**
Writes to the Device register through i2C.
Used to Write the data to the I2C client's Register through the i2c protocol

@param data The data to be written
@param reg The register address
@param dev The Device Structure
@return 0 on success

@see ad7146_i2c_read
*/
static int ad7146_i2c_write(struct device *dev, unsigned short reg,
		unsigned short data)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret = 0;
	u8 *_reg = (u8 *)&reg;
	u8 *_data = (u8 *)&data;
	char device_addr = client->addr;
	u8 tx[I2C_WRITE_LEN] = {
			_reg[1],
			_reg[0],
			_data[1],
			_data[0]
	};
	struct i2c_msg ad7146_wr_msg = {
			.addr = device_addr,
			.buf = tx,
			.len = I2C_WRITE_LEN,
			.flags = 0,
	};
	ret = i2c_transfer(client->adapter, &ad7146_wr_msg, 1);
	if (ret < 0)
			dev_err(&client->dev, "I2C write error\n");
	return ret;
}
/**
Reads data from the Device register through i2C.
This is used to read the data from the AD7146 I2C client

@param dev The Device Structure (Standard linux call)
@param reg The register address to be read.
@param data The data Read from the Given address.
@return The number of bytes transfered as an integer

@see ad7146_i2c_write
*/

static int ad7146_i2c_read(struct device *dev, unsigned short reg,
					unsigned short *data)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 *_reg = (u8 *)&reg;
	u8 *_data = (u8 *)data;
	char device_addr = client->addr;
	u8 tx[I2C_READ_LEN] = {
			_reg[1],
			_reg[0]
	};
	u8 rx[I2C_READ_LEN];
	int ret = 0;
	struct i2c_msg ad7146_rd_msg[I2C_READ_LEN] = {
			{
				.addr = device_addr,
				.buf = tx,
				.len = I2C_READ_LEN,
				.flags = 0,
			},
			{
				.addr = device_addr,
				.buf = rx,
				.len = I2C_READ_LEN,
				.flags = I2C_M_RD,
			}
	};
	ret = i2c_transfer(client->adapter, ad7146_rd_msg, 2);
	if (unlikely(ret < 0)) {
                printk(KERN_INFO "%s :I2C READ REG [0x%x] error %d\n", __func__, reg, ret);
		_data[0] = 0;
		_data[1] = 0;
	} else {
		_data[0] = rx[1];
		_data[1] = rx[0];
	}
	return ret;
}

/**
Command parsing function from echo/cat commands from command prompt.
This function is called when ever the User tries an echo / cat command
to the /../sysfs/<Device> especially during read/write registers.

@return void Returns Nothing
@see store_reg_read
 */
static void cmd_parsing(const char *buf, unsigned short *addr,
		unsigned short *cnt, unsigned short *data)
{
	char **bp = (char **)&buf;
	char *token, minus, parsing_cnt = 0;
	unsigned long int val;
	int pos;
	while ((token = strsep(bp, " "))) {
		pos = 0;
		minus = false;
		if ((char)token[pos] == '-') {
			minus = true;
			pos++;
		}
		if (token[pos] == '0' && (token[pos + 1] ==
					'x' || token[pos + 1] == 'x')) {
			if (kstrtoul(&token[pos + 2], HEX_BASE,
				     (long unsigned int  *)&val))
				val = 0;
			if (minus)
				val *= (-1);
		} else {
			if (kstrtoul(&token[pos], DECIMAL_BASE,
				     (long unsigned int  *)&val))
				val  = 0;
			if (minus)
				val *= (-1);
		}
		switch (parsing_cnt) {
		case 0:
				*addr = val;
				break;
		case 1:
				*cnt  = val;
				break;
		default:
		case 2:
				*data = val;
				data++;
				break;
		}
		parsing_cnt++;
		if (parsing_cnt > 2)
			return;
	}
}


/**
Command parsing temp comp function is used to parse commands
like echo/cat from command prompt.
This function is called when ever the User tries an echo / cat command
to the /../sysfs/<Device> especially during read/write registers.

@return void Returns Nothing
@see store_reg_read
 */
static void cmd_parsing_temp_comp(const char *buf, unsigned long *stage,
		unsigned long *gain, unsigned long *offset)
{
	char **bp = (char **)&buf;
	char **gain_token;
	char *token, minus;
	char *gain_whole;
	char *gain_decimal;
	char parsing_cnt = 0;

	unsigned long int val;
	int pos;
	int loc_addr = -1;
	unsigned long int g_whole = 0;
	unsigned long int g_decimal = 0;

	char str[2];
	unsigned long temp = 0;
	int factor = 100;
	int count;

	while ((token = strsep(bp, " "))) {
		pos = 0;
		minus = false;
		if ((char)token[pos] == '-') {
			minus = true;
			pos++;
		}

		if (parsing_cnt == 1) {
			gain_token = (char **)&token;
			gain_whole = strsep(gain_token, ".");
			if (kstrtoul(&gain_whole[pos], DECIMAL_BASE,
				     (long unsigned int *)&g_whole))
				g_whole = 0;

			if ((*gain_token) != NULL) {
				gain_decimal = strsep(gain_token, "\0");
				for (count = 0; count <= 2 ; count++) {
					str[0] = gain_decimal[count];
					if (str[0] == '\0')
						break;
					str[1] = '\0';
					if (!(kstrtoul(str, DECIMAL_BASE,
						       &temp))) {
						g_decimal = g_decimal +
							    (temp * factor);
					} else {
						printk(KERN_INFO"Error: DECI");
						printk(KERN_INFO"CONV\n");
						return;
					}
					factor = factor / 10;
				}

			}
			*gain = (g_whole * 1000) + g_decimal;
			if (minus)
				*gain *= (-1);
			parsing_cnt++;
			continue;
		}

		if (token[pos] == '0' && (token[pos + 1] == 'x' ||
					  token[pos + 1] == 'x')) {
			if (kstrtoul(&token[pos + 2], HEX_BASE,
				     (long unsigned int  *)&val))
				val = 0;
			if (minus)
				val *= (-1);
		} else {
			if (kstrtoul(&token[pos], DECIMAL_BASE,
				     (long unsigned int  *)&val))
				val  = 0;
			if (minus)
				val *= (-1);
		}
		switch (parsing_cnt) {
		case 0:
			*stage = val;
			loc_addr = val;
			break;
		default:
		case 2:
			*offset = val;
			break;
		}
		parsing_cnt++;
		if (parsing_cnt > 2)
			return;
	}
}

/**
  This is used to get the Stage information of the device using i2c.
  @param ad7146 The Device structure and other associated data
  @return Void Returns Nothing
 */
static void getStageInfo(struct ad7146_chip *ad7146)
{
	static unsigned short u16_IntrEn, u16_NumOfStages, u16_LastStageNum;
	unsigned int u32_StgConn;
	unsigned short u16Temp1, u16Temp2, u16Cnt;
	unsigned char u8_StageIndex = 0;
	unsigned char curnt_stg = 0;

	u16_IntrEn = 0;
	ad7146->sensor_int_enable = 0;
	ad7146->complete_enable = 0;
	ad7146->LastStageEnabled = 0;
	u16_NumOfStages = 0;

	while (u8_StageIndex < (STAGE_NUM * 2)) {
		ad7146->read(ad7146->dev,
			     u16_StageConnRegister[u8_StageIndex], &u16Temp1);
                AD7146_Driver_Dbg("CON_REG[0x%x] = %x \n", u16_StageConnRegister[u8_StageIndex], u16Temp1);
		u8_StageIndex++;
		ad7146->read(ad7146->dev,
			     u16_StageConnRegister[u8_StageIndex], &u16Temp2);
                AD7146_Driver_Dbg("CON_REG[0x%x] = %x \n", u16_StageConnRegister[u8_StageIndex], u16Temp2);
		u8_StageIndex++;
		u32_StgConn = ((u16Temp2 << 16) | (u16Temp1 & 0x3fff));
		curnt_stg = (u8_StageIndex/2)-1;

		AD7146_Driver_Info("STAGE %d CON_REG = %x\n",
				  curnt_stg, u32_StgConn);

		for (u16Cnt = 0; u16Cnt < 14; u16Cnt++) {
			if ((u32_StgConn & 0x3) == 0x02) {
				ad7146->sw[u16_NumOfStages].index = curnt_stg;
				u16_IntrEn = (u16_IntrEn |
					(0x1 << curnt_stg));
				u16_NumOfStages++;
				u16_LastStageNum = curnt_stg;
				break;
			}
			u32_StgConn = (u32_StgConn >> 2);
		}
	}
	ad7146->No_of_Stages = u16_NumOfStages;
	ad7146->sensor_int_enable = u16_IntrEn;
	ad7146->complete_enable = u16_LastStageNum;
	ad7146->LastStageEnabled = u16_LastStageNum;
        /* HACK FOR TEMP SENSOR */
        ad7146->LastStageEnabled++;

	AD7146_Driver_Info("sensor_int_enable  = 0x%x complete_enable = %d\n",
			  ad7146->sensor_int_enable, ad7146->complete_enable);
}

/**
This function is to identify the current configured interrupt mode from the
new configuration settings and reinitialize the registers accordingly.
Also, set the local variable intr_mode to corresponding interrupt mode.

@param ad7146 The Device structure and other associated data
@return Void Returns Nothing
*/
static void setInterruptMode(struct ad7146_chip *ad7146)
{
	unsigned short u16Temp;

	ad7146->read(ad7146->dev, STG_HIGH_INT_EN_REG, &u16Temp);
	if (u16Temp) {
		ad7146->intr_mode = SENSOR_ACTIVE_INT;
		ad7146->write(ad7146->dev,
			      STG_LOW_INT_EN_REG, ad7146->sensor_int_enable);
		ad7146->write(ad7146->dev,
			      STG_HIGH_INT_EN_REG, ad7146->sensor_int_enable);
		ad7146->write(ad7146->dev, STG_COM_INT_EN_REG, 0x0);
		AD7146_Driver_Dbg("Sensor Active interrupt configured\n");
	} else {
		ad7146->intr_mode = CONV_COMP_INT;
		ad7146->write(ad7146->dev, STG_LOW_INT_EN_REG, 0x0);
		ad7146->write(ad7146->dev, STG_HIGH_INT_EN_REG, 0x0);
		ad7146->write(ad7146->dev,
			STG_COM_INT_EN_REG, (0x1 << ad7146->complete_enable));
		AD7146_Driver_Dbg("Conversion complete interrupt configured\n");
	}
}

/*
@brief This API is used to start calibration from user
*	@param filename name of the file
*	@param ad7146 the AD7146 device pointer
*	@return int Neagative if ERROR
*/
static int ad7146_filp_start_calib(char *filename,
		struct ad7146_chip *ad7146,
		unsigned short Open_mode)
{
	mm_segment_t old_fs;
	struct file *fp_ad7146 = NULL;
	unsigned short data_cnt = 0;
	unsigned short tot_data_cnt = 0;
	unsigned int Start_Addr = 0;
	unsigned short line_no = 0;
	unsigned short loop_cnt = 0;
	unsigned short line_chk = 0;
	unsigned short cmt_lock = 0;
	unsigned short recv_data = 0;
	int scanf_ret = 0;
	int ret = 0;
	unsigned char Chk_comment = 0;
	char read_buff[VFS_READ_SIZE];
	char retry_cnt = RETRY_ATTEMPT_READ;
	char *temp_buf = NULL;
	char *pos_buf = NULL;
	int *filp_buf = ad7146->filp_buffer;
	loff_t pos = 0;
	int char1, char2;
	unsigned char line_char_count = 0;
	memset(read_buff, '\0', sizeof(read_buff));

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fp_ad7146 = filp_open(filename, O_RDONLY, 0666);
	if (IS_ERR(fp_ad7146)) {
		printk(KERN_INFO"[AD7146]: Unable to Open the file %ld\n",
		       PTR_ERR(fp_ad7146));
		set_fs(old_fs);
		ret = -1;
		goto err_filp_open;
	}
	do {
		ret = vfs_read(fp_ad7146, read_buff, sizeof(read_buff), &pos);
		if ((ret == -EIO) | (ret <= MIN_CHAR_TO_READ)) {
			ret = 0;
			AD7146_Driver_Dbg("Reached EOF\n");
			if ((tot_data_cnt == 0)) {
				ret = -EIO;
				goto err_filp_nodata;
			} else {
				break;
			}

		} else {
			temp_buf = read_buff;
			temp_buf = strsep((char **)&temp_buf, "\n");
			if (temp_buf == NULL)
				goto err_filp_nodata;
			pos_buf = temp_buf;
			line_char_count = 0;
			line_chk = 0;
			while ((*pos_buf) != '\0') {
				/*traverse and  check for comments*/
				line_char_count += 1;
				if ((*pos_buf == '/') &&
				    (*(pos_buf + 1) == '/'))
					Chk_comment = 0x01;
				/*Check Multiple Comment*/
				else if ((*pos_buf == '/') &&
					 (*(pos_buf + 1) == '*'))
					cmt_lock = 1;
				else if ((*pos_buf == '*') &&
					 (*(pos_buf + 1) == '/'))
					line_chk |= 0x10;
				/*Check inline Data*/
				else if (*pos_buf == ':')
					line_chk |= 0x1;
				/*Check inline Data*/
				else if (*pos_buf == ';')
					line_chk |= 0x2;
				if ((!cmt_lock) && (Chk_comment) &&
				    (*pos_buf != '/') && (*pos_buf != ' ') &&
				    (*pos_buf != '\r') && (*pos_buf != '\t'))
					line_chk |= 0x10;
				pos_buf++;
			}
			pos = pos + line_char_count - (unsigned int) ret + 1;
			line_no++; /*New Line Found*/
			if (Chk_comment || cmt_lock) {
				if ((line_chk & 0x0f) == 0x03) {
					if (!cmt_lock) {
						/*Comment with Inline data*/
						Chk_comment = 0;
						goto read_line;
					}
				} else if (line_chk & 0xf0) {
					/*Reset Comment Status- single line*/
					cmt_lock = 0;
				} else { /*Check double / comment*/
					if (Chk_comment)
						cmt_lock = cmt_lock ^
							   (unsigned short)0x1;
				}
					Chk_comment = 0;
					continue;/*Skip the line parsing*/
			}
read_line:
			if (line_char_count >= MIN_CHAR_TO_READ) {
				/*Get Starting Address*/
				scanf_ret = sscanf(temp_buf, "%x: ",
						   &Start_Addr);
				if (scanf_ret < 1)
					goto err_filp_nodata;
			} else {
				AD7146_Driver_Dbg("No valid data in line %d\n",
						  line_no);
				if (!(retry_cnt--))
					goto err_filp_nodata;
				else
					continue;
			}
			temp_buf += SKIP_DATA_CNT;
			data_cnt = 0; /*Reset the Data writen, to the start*/
			while (loop_cnt <= VFS_READ_SIZE) {
				/*For Parsing a line*/
				scanf_ret = sscanf(temp_buf, "%x%x",
						   (unsigned int *)&char1,
						   (unsigned int *)&char2);
				if (scanf_ret < 2)
					goto err_filp_nodata;
				recv_data = (unsigned short)(char1<<8 | char2);
				/*
				AD7146_Driver_Dbg("Read addr %04x data %x\n",
						  (Start_Addr + data_cnt),
						  recv_data);
				*/
				if (tot_data_cnt >= MAX_FILP_DATA_CNT) {
					AD7146_Driver_Dbg("Filp_buff full\n");
					break;
				}
				*filp_buf = (unsigned int)
						(((unsigned short)
						  (Start_Addr + data_cnt) << 16)
						 | recv_data);
				filp_buf++;
				data_cnt++;
				tot_data_cnt++;
				temp_buf += SKIP_DATA_CNT;
				/*Skip the Read Stream*/
				if ((*temp_buf == ';')|(*(temp_buf+1) == ';'))
					break;/*Line parsed*/
			}
		}
	} while ((ret > MIN_CHAR_TO_READ) | (ret != -EIO));

	if (tot_data_cnt == 0)
		goto err_filp_nodata;
	else
		ad7146->filp_buff_cnt = tot_data_cnt;
	filp_close(fp_ad7146, NULL);
	set_fs(old_fs);
	return ret;
err_filp_nodata:
	ret = -1;
	ad7146->filp_buff_cnt = 0;
	printk(KERN_INFO"[AD7146]: File parser failed : %s\n", filename);
	AD7146_Driver_Dbg("No valid data found\n");
	filp_close(fp_ad7146, NULL);
err_filp_open:
	set_fs(old_fs);
	return ret;
}
/**
This is the Temperature compensation routine used for the
calculation of the compensation CDC to be removed from the other
stages of the AD7146 please use the init_tcomp_data for the inital
gain and offset settings. Support for the runtime modifications are provided
with the temp_comp attfibute in the sysfs.
@note To make this function inert set *.gain and *.offset each to zero
  */
static unsigned short ad7146_temp_comp(struct channeltemp_t *context,
			    unsigned short u16_comp_data)
{
	return (unsigned short)((((context->s32_gain)*(int)u16_comp_data)/D_M)
				 + context->s32_offset);
}


/**
  This is to configure the device with the register set defined in platform file.
  Finally calibration is done and status registers will be cleared.
 * @param  ad7146 The Device structure
 * @return void  Nothing Returned
 */

static int ad7146_hw_init(struct ad7146_chip *ad7146)
{
	int lcnt = 0, ret = -1;
	unsigned short data;
	unsigned short u16SeqStageNum = 0;
	unsigned int *buffer = NULL;
#ifndef CONFIG_NOSTG_CAL_LAST
	unsigned short val_stg_cal_en = 0;
#endif
	ad7146->No_of_Stages = 0;
	ret = ad7146_filp_start_calib(AD7146_CONFIG_FILE, ad7146, 0);
	mutex_lock(&interrupt_thread_mutex);
	for (lcnt = 0; lcnt < STAGE_NUM; lcnt++) {
		unsigned short addr;
		addr = u16_StageConnRegister[lcnt*2];
		/*Default No Connection Setting*/
		ad7146->write(ad7146->dev, addr, 0xffff);
		ad7146->write(ad7146->dev, (addr + 1), 0x3fff);
	}
	if (ret != 0) {
		buffer = ad7146->hw->regs;
		data = (sizeof(ad7146->hw->regs)/sizeof(int));
                AD7146_Driver_Dbg("REG CNT %d\n", data);
	} else {
		buffer = ad7146->filp_buffer;
		data = ad7146->filp_buff_cnt;
		AD7146_Driver_Dbg("FILP CNT %d\n", data);
	}
	/** configuration CDC and interrupts */
	for (lcnt = 0; lcnt < data; lcnt++) {
		unsigned short addr;
		unsigned short value;

		addr = (unsigned short)((buffer[lcnt]
					& 0xffff0000) >> 16);
		value = (unsigned short)(buffer[lcnt]
					 & 0x0000ffff);
		/*Force calibration done afterwards*/
		if (addr == AD7146_AMB_COMP_CTRL0_REG)
			value = value & 0xBFFF;
#ifndef CONFIG_NOSTG_CAL_LAST
		if (addr == AD7146_STG_CAL_EN_REG) {
			val_stg_cal_en = value;
			value = 0;
		}
#endif
		AD7146_Driver_Dbg("Writing 0x%x with Value %x\n",
				  addr, value);
		ad7146->write(ad7146->dev, addr, value);
	}
#ifndef CONFIG_NOSTG_CAL_LAST
	if (val_stg_cal_en != 0) {
		AD7146_Driver_Dbg("Writing 0x01 with Value %x\n",
				  val_stg_cal_en);
		ad7146->write(ad7146->dev, AD7146_STG_CAL_EN_REG,
			      val_stg_cal_en);
	}
#endif
	data = 0;
	/* Get Number of Stages used and Last stage enabled */
	getStageInfo(ad7146);
#ifndef SENSOR_ACTIVE_ON
#ifdef CONV_COMP_ON
	ad7146->write(ad7146->dev, STG_HIGH_INT_EN_REG, 0x0);
#endif
#endif

#ifndef CONV_COMP_ON
#ifdef SENSOR_ACTIVE_ON
	ad7146->write(ad7146->dev, STG_HIGH_INT_EN_REG, 0x1);
#endif
#endif

	/* identify and remember if the interrupt enable
	   registers are already configured */
	setInterruptMode(ad7146);

	AD7146_Driver_Info("%s called", __func__);
	/* Identify SEQUENCE_STAGE_NUM and re-write once  */
	ad7146->read(ad7146->dev, AD7146_PWR_CTRL, &u16SeqStageNum);
	u16SeqStageNum = ((u16SeqStageNum & 0xFF0F) |
			(ad7146->LastStageEnabled << 4)); /*chk*/
        AD7146_Driver_Info("%s u16SeqStageNum = 0x%x \n", __func__, u16SeqStageNum);
	ad7146->write(ad7146->dev, AD7146_PWR_CTRL, u16SeqStageNum);
        ad7146->power_reg_value = u16SeqStageNum; /*for power management*/

	/* Calibrate only the stages that have been used */
	ad7146->write(ad7146->dev, AD7146_STG_CAL_EN_REG,
			ad7146->sensor_int_enable);

	/* Do force recalibration of all the stages */
	ad7146->read(ad7146->dev, AD7146_AMB_COMP_CTRL0_REG, &data);
	data = (data | AD7146_FORCED_CAL_MASK) ; /*chk*/
	ad7146->write(ad7146->dev, AD7146_AMB_COMP_CTRL0_REG, data);
	msleep(MIN_FORCED_CAL_SLEEP);
	/* clear all interrupts */
	ad7146->read(ad7146->dev, STG_LOW_INT_STA_REG, &data);
	ad7146->read(ad7146->dev, STG_HIGH_INT_STA_REG, &data);
	ad7146->read(ad7146->dev, STG_COM_INT_STA_REG, &data);

	mutex_unlock(&interrupt_thread_mutex);
	if (ad7146->No_of_Stages)
		return 0;
	else
		return -ENODEV;
}



/**
This is used to redo a hardware init upon the request from the user.
This function calibrates the entire chip with the new configuration provided
or redos the initial platform configuration.
\note This called when the user requires to redo the configuration
\note in runtime using echo command to the files in /sysfs/ dir and

@return The Size of the parsed command
@param dev The Device Id and Information structure
@param attr The Device attributes to the AD7146
@param buf The buffer to store the Read data
@param count The count of bytes to be transfered to the Buffer

@return Returns the Size of the Read value
*/
static ssize_t redo_filp_calib(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	unsigned long val = 0;
	struct ad7146_chip *ad7146 = dev_get_drvdata(dev);

	ret = kstrtoul(buf, 10, &val);
	if ((ret) || (val != 1)) {
		printk(KERN_INFO"[AD7146]: %s INVALID CMD", __func__);
		return count;
	}

	ret = ad7146_hw_init(ad7146);
	return count;
}

/**
  This is used to get set a stage's offset and gain vaue

  This function Reads the value at the Device's Register for the i2c client
  \note This called when the user requires to read the configuration
\note in runtime using cat command to the files in /Sysfs/ dir and
\note it hold the register address to be read.

@return The Size of the Read Register 0 if not read
@param dev The Device Id and Information structure
@param attr The Device attributes to the AD7146
@param buf The buffer to store the Read data
@param count The count of bytes to be transfered to the Buffer

@return Returns the Size of the Read value
@see cmd_parsing_temp_comp
*/

static ssize_t store_temp_comp(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct ad7146_chip *ad7146 = dev_get_drvdata(dev);
	unsigned long stage = 0;
	unsigned long gain = 0;
	unsigned long offset = 0;

	mutex_lock(&interrupt_thread_mutex);
	cmd_parsing_temp_comp(buf, &stage, &gain, &offset);

	if (stage > 11) {
		printk(KERN_INFO "STAGE invalid\n");
		mutex_unlock(&interrupt_thread_mutex);
		return count;
	}
	ad7146->ast_chtempdata[stage].s32_gain = (int)gain;
	ad7146->ast_chtempdata[stage].s32_offset = (int)offset;

	printk(KERN_INFO "[AD7146]:command : stage %d,gain %d.%d,offset %d\n",
	       (int)stage, (int)gain/D_M, (int)abs((int)gain%D_M), (int)offset);

	mutex_unlock(&interrupt_thread_mutex);
	return count;
}

/**
This is used to read the data of the register address sent to reg_read

This functions Reads the value at the Device's Register for the given
client and Prints it in the output window
\note This is called from user space in the /../sysfs/<Device>/
The register pointed by store_reg_read is displayed to the console.
@return The Size of the Read Data, 0 if not read
@param dev The Device Id and Information structure(Linux Standard argument)
@param attr standard Linux Device attributes to the AD7146
@param buf The buffer to store the Read data

@see dev_get_drvdata
@see store_reg_read
*/

static ssize_t show_temp_comp(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	short ret = 0;
	unsigned short u16_lpcnt;
	struct ad7146_chip  *ad7146 = dev_get_drvdata(dev);

	for (u16_lpcnt = 0; u16_lpcnt < STAGE_NUM; u16_lpcnt++) {

		ret += sprintf(buf + ret, "STAGE %d GAIN %d.%d OFFSET %d\n",
			       u16_lpcnt,
			       ad7146->ast_chtempdata[u16_lpcnt].s32_gain/D_M,
			       (int)abs(ad7146->ast_chtempdata[u16_lpcnt].s32_gain%D_M),
			       ad7146->ast_chtempdata[u16_lpcnt].s32_offset);
	}
	return ret;
}



/**
  This is used to get register address whose data is to be read and size of data to be read

  This function Reads the value at the Device's Register for the i2c client
  \note This called when the user requires to read the configuration
\note in runtime using cat command to the files in /Sysfs/ dir and
\note it hold the register address to be read.

@return The Size of the Read Register 0 if not read
@param dev The Device Id and Information structure
@param attr The Device attributes to the AD7146
@param buf The buffer to store the Read data
@param count The count of bytes to be transfered to the Buffer

@return Returns the Size of the Read value
@see cmd_parsing
*/

static ssize_t store_reg_read(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct ad7146_chip *ad7146 = dev_get_drvdata(dev);
	unsigned short addr, cnt, val;
	unsigned short rd_data[2], i;
	char read_str[REG_READ_SIZE];

	mutex_lock(&interrupt_thread_mutex);
	cmd_parsing(buf, &addr, &cnt, &val);

	if (cnt > sizeof(rd_data))
		goto error;

	memset(ad7146->reg_data, 0x00, sizeof(ad7146->reg_data));

	AD7146_Driver_Dbg("Register Read command : reg = 0x%04X, size = %d\n",
			  addr, cnt);

	if (cnt > 1) {
		printk(KERN_INFO
		"I2C Multi Read not supported in function %s\n", __func__);
		goto error;
	} else {
		ad7146->read(ad7146->dev, addr, &rd_data[0]);
	}

	for (i = 0; i < cnt; i++) {
		memset(read_str, 0x00, sizeof(read_str));
		sprintf(read_str, "0x%04X ", rd_data[i]);
		printk(KERN_INFO "DATA = 0x%04x\n", rd_data[i]);
		strcpy(ad7146->reg_data, read_str);
	}

error:
	mutex_unlock(&interrupt_thread_mutex);
	return count;
}

/**
This is used to read the data of the register address sent to reg_read

This functions Reads the value at the Device's Register for the given
client and Prints it in the output window
\note This is called from user space in the /../sysfs/<Device>/
The register pointed by store_reg_read is displayed to the console.
@return The Size of the Read Data, 0 if not read
@param dev The Device Id and Information structure(Linux Standard argument)
@param attr standard Linux Device attributes to the AD7146
@param buf The buffer to store the Read data

@see dev_get_drvdata
@see store_reg_read
*/

static ssize_t show_reg_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ad7146_chip  *ad7146 = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", ad7146->reg_data);
}


/**
This is used to write data to a register through i2c.

This functions Writes the value of the buffer to the given client
provided the count value to write
\note This is used to store the register address to write the data.
Invoked by the used from the echo command in /../sysfs/<Device>
\note This also prints the Command received before writing the Registers.

@param dev The Device Id and Information structure(Linux Standard argument)
@param attr standard Linux Device attributes to the AD7146
@param count The number of bytes to write from the buffer
@param buf The buffer to store the Read data

@return The Size of the writen Data, 0 if not writen
*/

static ssize_t store_reg_write(struct device *dev,
	 struct device_attribute *attr, const char *buf, size_t count)
{
	struct ad7146_chip *ad7146 = dev_get_drvdata(dev);
	unsigned short addr, cnt;
	unsigned short wr_data[2], i;

	mutex_lock(&interrupt_thread_mutex);
	cmd_parsing(buf, &addr, &cnt, &wr_data[0]);
	if (cnt > sizeof(wr_data))
		goto error;

	AD7146_Driver_Dbg("Register Write command : reg = 0x%x, size = %d\n",
			  addr, cnt);
	for (i = 0; i < cnt; i++)
		AD7146_Driver_Dbg("DATA = 0x%04X\n", wr_data[i]);
	if (cnt > 1)
		printk(KERN_INFO
	"I2C Multi Write not supported in function %s\n", __func__);
	else
		ad7146->write(ad7146->dev, addr, wr_data[0]);

error:
	mutex_unlock(&interrupt_thread_mutex);

	return count;
}

/**
This is used to identify and store the Stages information of the device.
This is required to configure the interrupt modes
based on the stages enabled during runtime.

This functions Reads the value of the Stage status,
and stores it in the buffer given.
\note this is also invoked by the echo command in the /../sysfs/<Device> region.

@param dev The Device Id and Information structure(Linux Standard argument)
@param attr standard Linux Device attributes to the AD7146
@param buf The buffer to store the Read data
@param count The count of bytes to be transfered to the Buffer
@return The Size of the Read Data, 0 if not Read
@see getStageInfo
*/

static ssize_t store_stage_info(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct ad7146_chip *ad7146 = dev_get_drvdata(dev);
	int err;
	unsigned long val;
	err = kstrtoul(buf, 10, &val);
	if (err)
		return err;
	mutex_lock(&interrupt_thread_mutex);

	if (val)
		getStageInfo(ad7146);

	mutex_unlock(&interrupt_thread_mutex);/*Global Mutex*/
	return count;
}

#ifdef SENSOR_ACTIVE_ON
#ifdef CONV_COMP_ON
/**
This function is to configure interrupt mode to sensor active or conversion complete
Confired in runtime -> 1 = sensor active & 0 =  conversion complete

@param dev The Device Id and Information structure(Linux Standard argument)
@param attr standard Linux Device attributes to the AD7146
@param count The number of bytes to write from the buffer
@param buf The buffer to store the Read data
@param count The count of bytes to be transfered to the Buffer

@return The Size of the written Data, 0 if not written
@see store_reg_write
*/

static ssize_t store_intr_mode(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct ad7146_chip *ad7146 = dev_get_drvdata(dev);
	int err, val;
	unsigned short temp;
	err = kstrtoul(buf, 10, (unsigned long int *)&val);
	if (err)
		return err;

	mutex_lock(&interrupt_thread_mutex);

	if (val == SENSOR_ACTIVE_INT) {
		ad7146->intr_mode = SENSOR_ACTIVE_INT;
		ad7146->write(ad7146->dev,
				STG_LOW_INT_EN_REG, ad7146->sensor_int_enable);
		ad7146->write(ad7146->dev,
				STG_HIGH_INT_EN_REG, ad7146->sensor_int_enable);
		ad7146->write(ad7146->dev, STG_COM_INT_EN_REG, 0x0);
	} else if (val == CONV_COMP_INT) {
		ad7146->intr_mode = CONV_COMP_INT;
		ad7146->write(ad7146->dev, STG_LOW_INT_EN_REG, 0x0);
		ad7146->write(ad7146->dev, STG_HIGH_INT_EN_REG, 0x0);
		ad7146->write(ad7146->dev,
			STG_COM_INT_EN_REG, (0x1 << ad7146->complete_enable));
	}

	ad7146->read(ad7146->dev, AD7146_AMB_COMP_CTRL0_REG, &temp);
	/*Set the 14th bit For Force calibrate */
	temp = temp | 0x4000;
	ad7146->write(ad7146->dev, AD7146_AMB_COMP_CTRL0_REG, temp);

	mutex_unlock(&interrupt_thread_mutex);/*Global Mutex*/

	return count;
}

/**
This function is used to read the current configured interrupt mode

@param dev The Device Id and Information structure(Linux Standard argument)
@param attr standard Linux Device attributes to the AD7146
@param buf The buffer to store the Read data

@return The Size of the Read Data, 0 if not Read
*/

static ssize_t show_intr_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ad7146_chip *ad7146 = dev_get_drvdata(dev);
	if (ad7146->intr_mode == SENSOR_ACTIVE_INT)
		return sprintf(buf, "%d - SENSOR_ACTIVE_INT\n",
				ad7146->intr_mode);
	else
		return sprintf(buf, "%d - CONV_COMP_INT\n", ad7146->intr_mode);
}
#endif
#endif

/**
This function is to show the stages info to the user.
The output would be like [COUNT] [stageX] [stageY] .... [stageCOUNT]

@param dev The Device Id and Information structure(Linux Standard argument)
@param attr standard Linux Device attributes to the AD7146
@param buf The buffer to store the Read data

@return The Size of the Read Data, 0 if not Read
@see getStageInfo
 */
static ssize_t show_stage_info(struct device *dev, \
	struct device_attribute *attr, char *buf)
{
	struct ad7146_chip  *ad7146 = dev_get_drvdata(dev);
	unsigned short temp;
	unsigned int shift_count = 0;
	unsigned char stage[STAGE_NUM];
	unsigned int count;
	int i, j = 0;

	temp = ad7146->sensor_int_enable;
	count = 0;
	memset(stage, 0, sizeof(stage));
	while (temp) {
		if ((temp & 0x1) == 0x1) {
			temp >>= 1;
			shift_count++;
			stage[count++] = shift_count-1;
		}
	}
	j = sprintf(buf, "%d ", count);
	i = 0;
	while (i < count)
		j += sprintf(buf+j, "%d ", stage[i++]);
	j += sprintf(buf+j, "\n");
	return j;
}

/**
This is used to Force Calibrate the device through i2c.
This functions fore Calibrate the Device AD7146 at run time.

@param dev The Device Id and Information structure(Linux Standard argument)
@param attr standard Linux Device attributes to the AD7146
@param buf The buffer to store the data to be written
@param count The count of bytes to be transfered to the Device

\note This is evoked upon an echo request in the /../sysfs/<Device> region.
\note This also prints the results in the console for the user.
\note The calibration is invoked by this function forcefully upon user request.
\note Effects are immediate in this calibration.

@return count of data written
*/
static ssize_t do_calibrate(struct device *dev, \
struct device_attribute *attr, const char *buf, size_t count)
{
	struct ad7146_chip *ad7146 = dev_get_drvdata(dev);
	int err ;
	unsigned long val;
	unsigned short u16Temp;
	err = kstrtoul(buf, 10, &val);
	if (err)
		return err;
	mutex_lock(&interrupt_thread_mutex);
	getStageInfo(ad7146);
	setInterruptMode(ad7146);
	ad7146->write(ad7146->dev, AD7146_STG_CAL_EN_REG,\
	ad7146->sensor_int_enable);
	ad7146->read(ad7146->dev, AD7146_AMB_COMP_CTRL0_REG, &u16Temp);
	/*Set the 14th bit For Force calibrate*/
	u16Temp = u16Temp | AD7146_FORCED_CAL_MASK;
	ad7146->write(ad7146->dev, AD7146_AMB_COMP_CTRL0_REG, u16Temp);
	msleep(MAX_FORCED_CAL_SLEEP);
	ad7146->read(ad7146->dev, STG_LOW_INT_STA_REG, &u16Temp);
	ad7146->read(ad7146->dev, STG_HIGH_INT_STA_REG, &u16Temp);
	ad7146->read(ad7146->dev, STG_COM_INT_STA_REG, &u16Temp);
	AD7146_Driver_Info("Force calibration done\n");
	mutex_unlock(&interrupt_thread_mutex);
	return count;
}

static ssize_t store_status(struct device *dev,
                            struct device_attribute *attr, const char *buf, size_t count)
{
  struct ad7146_chip *ad7146 = dev_get_drvdata(dev);
  int err, val;
  bool en;
  enum ad7146_status status;
  unsigned short data;
  err = kstrtoul(buf, 10, (unsigned long int *)&val);
  if (err)
    return err;
  mutex_lock(&interrupt_thread_mutex);
  en = val ? true : false;
  status = val ? ENABLED : DISABLED;
  if (ad7146->status != status){
    if (status == ENABLED){
      ad7146->read(ad7146->dev, AD7146_PWR_CTRL, &data);
      data &= AD7146_PWRUP_MASK;
      ad7146->power_reg_value = data;
      /* resume to non-shutdown mode */
      ad7146->write(ad7146->dev, AD7146_PWR_CTRL, ad7146->power_reg_value);
      /* make sure the interrupt output line is not low level after resume,
       * otherwise we will get no chance to enter falling-edge irq again
       */
      ad7146->read(ad7146->dev, STG_LOW_INT_STA_REG, &data);
      ad7146->read(ad7146->dev, STG_HIGH_INT_STA_REG, &data);
      ad7146->read(ad7146->dev, STG_COM_INT_STA_REG, &data);
    }else {
      ad7146->read(ad7146->dev, AD7146_PWR_CTRL, &data);
      ad7146->power_reg_value = data;
      data |= AD7146_SHUTDOWN_MASK;
      ad7146->write(ad7146->dev, AD7146_PWR_CTRL, data);
    }
  }
  ad7146->status = status;
  mutex_unlock(&interrupt_thread_mutex);
  return count;
}

static ssize_t show_status(struct device *dev,
                           struct device_attribute *attr, char *buf)
{
  struct ad7146_chip *ad7146 = dev_get_drvdata(dev);
  return sprintf(buf, "%d - %s \n", ad7146->status, ad7146->status ? "ENABLED" : "DISABLED");
}

static ssize_t do_davetune(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
  struct ad7146_chip *ad7146 = dev_get_drvdata(dev);
  int err, i;
  unsigned long val;
  unsigned short u16Temp, pos_afe_offset, neg_afe_offset;
  unsigned short afe_offset_set_off_reback, afe_offset_read_value_reback;
  unsigned short u16Temp_offset,u16Temp_value;
  AD7146_Driver_Dbg(" %s \n", __func__);

  err = kstrtoul(buf, 10, &val);
  if (err)
    return err;
  mutex_lock(&interrupt_thread_mutex);

  printk(KERN_ERR "wiles_debug, 2013_0809-1A_50ms \n");
  ad7146->read(ad7146->dev, 0x0000, &u16Temp);
  printk(KERN_ERR "REG=0x0000, VALUE=0x%4x \n", u16Temp);
  msleep(10);
  ad7146->read(ad7146->dev, 0x0045, &u16Temp);
  printk(KERN_ERR "REG=0x0045, VALUE=0x%4x \n", u16Temp);

  /* Reset OFFSET REGS TO 0 */
  for (i = 0; i <= ad7146->LastStageEnabled; i++){
    ad7146->write(ad7146->dev, u16_StageAfeRegister[i], 0);
  }
  msleep(65);

  for (i = 0; i <= ad7146->LastStageEnabled; i++)
    {
      ad7146->read(ad7146->dev, u16_StageAfeRegister[i], &u16Temp);
      neg_afe_offset = u16Temp & 0x003F;
      pos_afe_offset = (u16Temp >> 8) & 0x003F;
      ad7146->read(ad7146->dev, u16_ConversionDataRegister[i], &u16Temp);
      printk(KERN_ERR "START, (P,N,R,V) = (0x%2x, 0x%2x, 0x%2x, %6d) \n",
             pos_afe_offset, neg_afe_offset, u16_ConversionDataRegister[i], u16Temp);
      if(u16Temp < MIN_AFE)
        {
          while(((u16Temp == 65535) || (u16Temp < MIN_AFE))
                && (neg_afe_offset < 0x40))
            {
              neg_afe_offset++;
              if(neg_afe_offset == 0x40)
		{
                  /*
                    nothing to do, due to the range of
                    neg_afe_offset is from 0x00 to 0x3F
                  */
		}
              else
		{
                  u16Temp = pos_afe_offset << 8 | neg_afe_offset;
                  ad7146->write(ad7146->dev, u16_StageAfeRegister[i], u16Temp);
                  msleep(3);
                  ad7146->read(ad7146->dev, u16_StageAfeRegister[i], &u16Temp);
                  afe_offset_set_off_reback = u16Temp;
                  msleep(65);
                  ad7146->read(ad7146->dev, u16_StageAfeRegister[i], &u16Temp);
                  afe_offset_read_value_reback = u16Temp;
                  ad7146->read(ad7146->dev, u16_ConversionDataRegister[i], &u16Temp);
                  printk(KERN_ERR
                         "1, V < MIN <   < MAX <   , (P,N,R,V) = (0x%2x, 0x%2x, 0x%2x, %6d) (R,set_off_reback,read_value_reback) = (0x%2x, 0x%4x, 0x%4x) \n",
                         pos_afe_offset, neg_afe_offset, u16_ConversionDataRegister[i],
                         u16Temp, u16_StageAfeRegister[i], afe_offset_set_off_reback,
                         afe_offset_read_value_reback);
		}
            }
          /* at this point, the value is (min < V < max),
             (min < max < V), (neg_afe_offset == 0x40,
             but the StageAfeRegister_NEG is 0x3F)
          */
          while((u16Temp > MAX_AFE) && (neg_afe_offset != 0x40))
            {
              pos_afe_offset++;
              u16Temp = pos_afe_offset << 8 | neg_afe_offset;
              ad7146->write(ad7146->dev, u16_StageAfeRegister[i], u16Temp);
              msleep(3);
              ad7146->read(ad7146->dev, u16_StageAfeRegister[i], &u16Temp);
              afe_offset_set_off_reback = u16Temp;
              msleep(65);
              ad7146->read(ad7146->dev, u16_StageAfeRegister[i], &u16Temp);
              afe_offset_read_value_reback = u16Temp;
              ad7146->read(ad7146->dev, u16_ConversionDataRegister[i], &u16Temp);
              printk(KERN_ERR
                     "1,   < MIN <   < MAX < V , (P,N,R,V) = (0x%2x, 0x%2x, 0x%2x, %6d) (R,set_off_reback,read_value_reback) = (0x%2x, 0x%4x, 0x%4x) \n",
                     pos_afe_offset, neg_afe_offset, u16_ConversionDataRegister[i],
                     u16Temp, u16_StageAfeRegister[i], afe_offset_set_off_reback,
                     afe_offset_read_value_reback);
            }
          /*
            at this point, the value is (min < V < max),
            (V < min), (neg_afe_offset == 0x40,
            but the StageAfeRegister_NEG is 0x3F)
          */
	}
      if((u16Temp > MAX_AFE) || (neg_afe_offset == 0x40))
	{
          neg_afe_offset = 0x00;
          while(((u16Temp == 0) || (u16Temp > MAX_AFE))
                && (pos_afe_offset < 0x40))
            {
              pos_afe_offset++;
              if(pos_afe_offset == 0x40)
                {
                  /*
                    nothing to do, due to the range of
                    pos_afe_offset is from 0x00 to 0x3F
                  */
                }
              else
                {
                  u16Temp = pos_afe_offset << 8 | neg_afe_offset;
                  ad7146->write(ad7146->dev, u16_StageAfeRegister[i], u16Temp);
                  msleep(3);
                  ad7146->read(ad7146->dev, u16_StageAfeRegister[i], &u16Temp);
                  afe_offset_set_off_reback = u16Temp;
                  msleep(65);
                  ad7146->read(ad7146->dev, u16_StageAfeRegister[i], &u16Temp);
                  afe_offset_read_value_reback = u16Temp;
                  ad7146->read(ad7146->dev, u16_ConversionDataRegister[i], &u16Temp);
                  printk(KERN_ERR
                         "2,   < MIN <   < MAX < V , (P,N,R,V) = (0x%2x, 0x%2x, 0x%2x, %6d) (R,set_off_reback,read_value_reback) = (0x%2x, 0x%4x, 0x%4x) \n",
                         pos_afe_offset, neg_afe_offset, u16_ConversionDataRegister[i], u16Temp,
                         u16_StageAfeRegister[i], afe_offset_set_off_reback, afe_offset_read_value_reback);
                }
            }
          /*
            at this point, the value is (min < V < max), ( V < min < max ),
            (pos_afe_offset == 0x40, but the StageAfeRegister_POS is 0x3F)
          */
          while((u16Temp < MIN_AFE) && (pos_afe_offset != 0x40))
            {
              neg_afe_offset++;
              u16Temp = pos_afe_offset << 8 | neg_afe_offset;
              ad7146->write(ad7146->dev, u16_StageAfeRegister[i], u16Temp);
              msleep(3);
              ad7146->read(ad7146->dev, u16_StageAfeRegister[i], &u16Temp);
              afe_offset_set_off_reback = u16Temp;
              msleep(65);
              ad7146->read(ad7146->dev, u16_StageAfeRegister[i], &u16Temp);
              afe_offset_read_value_reback = u16Temp;
              ad7146->read(ad7146->dev, u16_ConversionDataRegister[i], &u16Temp);
              printk(KERN_ERR
                     "2, V < MIN <   < MAX <   , (P,N,R,V) = (0x%2x, 0x%2x, 0x%2x, %6d) (R,set_off_reback,read_value_reback) = (0x%2x, 0x%4x, 0x%4x) \n",
                     pos_afe_offset, neg_afe_offset, u16_ConversionDataRegister[i],
                     u16Temp, u16_StageAfeRegister[i], afe_offset_set_off_reback,
                     afe_offset_read_value_reback);
            }
          /* at this point, the value is (min < V < max),
             (V > max), (pos_afe_offset == 0x40,
             but the StageAfeRegister_POS is 0x3F)
          */
	}
      if(pos_afe_offset == 0x40)
	{
          pos_afe_offset = 0x00;
          while((u16Temp == 65535) || (u16Temp < MIN_AFE))
            {
              neg_afe_offset++;
              u16Temp = pos_afe_offset << 8 | neg_afe_offset;
              ad7146->write(ad7146->dev, u16_StageAfeRegister[i], u16Temp);
              msleep(3);
              ad7146->read(ad7146->dev, u16_StageAfeRegister[i], &u16Temp);
              afe_offset_set_off_reback = u16Temp;
              msleep(65);
              ad7146->read(ad7146->dev, u16_StageAfeRegister[i], &u16Temp);
              afe_offset_read_value_reback = u16Temp;
              ad7146->read(ad7146->dev, u16_ConversionDataRegister[i], &u16Temp);
              printk(KERN_ERR
                     "3, V < MIN <   < MAX <   , (P,N,R,V) = (0x%2x, 0x%2x, 0x%2x, %6d) (R,set_off_reback,read_value_reback) = (0x%2x, 0x%4x, 0x%4x) \n",
                     pos_afe_offset, neg_afe_offset, u16_ConversionDataRegister[i],
                     u16Temp, u16_StageAfeRegister[i], afe_offset_set_off_reback,
                     afe_offset_read_value_reback);
            }
          while((u16Temp > MAX_AFE) && (neg_afe_offset != 0x40))
            {
              pos_afe_offset++;
              u16Temp = pos_afe_offset << 8 | neg_afe_offset;
              ad7146->write(ad7146->dev, u16_StageAfeRegister[i], u16Temp);
              msleep(3);
              ad7146->read(ad7146->dev, u16_StageAfeRegister[i], &u16Temp);
              afe_offset_set_off_reback = u16Temp;
              msleep(65);
              ad7146->read(ad7146->dev, u16_StageAfeRegister[i], &u16Temp);
              afe_offset_read_value_reback = u16Temp;
              ad7146->read(ad7146->dev, u16_ConversionDataRegister[i], &u16Temp);
              printk(KERN_ERR
                     "3,   < MIN <   < MAX < V , (P,N,R,V) = (0x%2x, 0x%2x, 0x%2x, %6d) (R,set_off_reback,read_value_reback) = (0x%2x, 0x%4x, 0x%4x) \n",
                     pos_afe_offset, neg_afe_offset, u16_ConversionDataRegister[i],
                     u16Temp, u16_StageAfeRegister[i], afe_offset_set_off_reback,
                     afe_offset_read_value_reback);
            }

	}
    }
  for (i = 0; i <= ad7146->LastStageEnabled; i++)
    {
      ad7146->read(ad7146->dev, u16_StageAfeRegister[i], &u16Temp);
      u16Temp_offset = u16Temp;
      ad7146->read(ad7146->dev, u16_ConversionDataRegister[i], &u16Temp);
      u16Temp_value = u16Temp;
      printk(KERN_ERR "(AFE_REG,AFE_DATA,VALUE_REG,VALUE_DATA) = (0x%2x, 0x%4x, 0x%2x, %6d) \n",
             u16_StageAfeRegister[i], u16Temp_offset,
             u16_ConversionDataRegister[i], u16Temp_value);
    }
  mutex_unlock(&interrupt_thread_mutex);
  AD7146_Driver_Info("Auto tune done\n");
  return count;
}

static ssize_t do_autotune(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct ad7146_chip *ad7146 = dev_get_drvdata(dev);
	int err, i;
	unsigned long val;
	unsigned short u16Temp, pos_afe_offset, neg_afe_offset;
	AD7146_Driver_Dbg("%s\n", __func__);

	err = kstrtoul(buf, 10, &val);
	if (err)
		return err;

	mutex_lock(&interrupt_thread_mutex);

	/* Reset OFFSET REGS TO 0 */
	for (i = 0; i <= ad7146->LastStageEnabled; i++)
		ad7146->write(ad7146->dev, u16_StageAfeRegister[i], 0);
	msleep(65);
	for (i = 0; i <= ad7146->LastStageEnabled; i++) {
		ad7146->read(ad7146->dev,
			u16_ConversionDataRegister[i], &u16Temp);
		while ((u16Temp > MAX_AFE) || (u16Temp < MIN_AFE)) {
			ad7146->read(ad7146->dev,
				u16_StageAfeRegister[i], &u16Temp);
			neg_afe_offset = u16Temp & 0x003F;
			pos_afe_offset = (u16Temp >> 8) & 0x003F;
			pos_afe_offset++;
			pos_afe_offset = pos_afe_offset & 0x003F;
			if (pos_afe_offset == 0)
				neg_afe_offset++;
			neg_afe_offset = neg_afe_offset & 0x003F;
			u16Temp = pos_afe_offset << 8 | neg_afe_offset;
			ad7146->write(ad7146->dev,
				u16_StageAfeRegister[i], u16Temp);
			msleep(65);
			ad7146->read(ad7146->dev,
				u16_ConversionDataRegister[i], &u16Temp);
		}
		printk(KERN_ERR "REG=0x%x CDC_RESULT_S[%d]=%d\n",
			u16_ConversionDataRegister[i], i, u16Temp);
		ad7146->read(ad7146->dev, u16_StageAfeRegister[i], &u16Temp);
		printk(KERN_ERR "REG=0x%x STAGE[%d]_AFE_OFFSET=0x%x\n",
			u16_StageAfeRegister[i], i, u16Temp);
	}

	mutex_unlock(&interrupt_thread_mutex);
	AD7146_Driver_Info("Auto tune done\n");
	return count;
}

static ssize_t show_autotune(struct device *dev,
                           struct device_attribute *attr, char *buf)
{
  unsigned short u16Temp;
  int i=0;
  int n=0;
  int ret=0;
  struct ad7146_chip *ad7146 = dev_get_drvdata(dev);
  AD7146_Driver_Info(" %s \n", __func__);
  for(i=0; i <= ad7146->LastStageEnabled; i++){
    ad7146->read(ad7146->dev, u16_StageAfeRegister[i], &u16Temp);
    n = sprintf(buf+ret, "0x%x ", u16Temp);
    ret = ret + n;
  }
  n = sprintf(buf+ret, "\n");
  ret = ret + n;
  return ret;
}

/**
 Common enable routine for the AD7146 driver.
 used for the recovery from shutdown to normal mode
 @param ad7146 the Chip to be enabled
 @return Returns 0 on success.
 */
int ad7146_enable(struct ad7146_chip *ad7146)
{
	unsigned short data;
        AD7146_Driver_Info("%s enter\n", __func__);
	mutex_lock(&interrupt_thread_mutex);
	/* resume to non-shutdown mode */
	ad7146->write(ad7146->dev, AD7146_PWR_CTRL,
			ad7146->power_reg_value);
	/* make sure the interrupt output line is not low level after resume,
	 * otherwise we will get no chance to enter falling-edge irq again
	 */
	ad7146->read(ad7146->dev, STG_LOW_INT_STA_REG, &data);
	ad7146->read(ad7146->dev, STG_HIGH_INT_STA_REG, &data);
	ad7146->read(ad7146->dev, STG_COM_INT_STA_REG, &data);
	mutex_unlock(&interrupt_thread_mutex);
	return 0;
}
/**
 Common disable routine for the AD7146 driver.
 used for the shutdown of the chip
 @param ad7146 the Chip to be Disabled
 @return Returns 0 on success.
 */

int ad7146_disable(struct ad7146_chip *ad7146)
{
	unsigned short data;
        AD7146_Driver_Info("%s enter\n", __func__);

	cancel_work_sync(&ad7146->work);

	mutex_lock(&interrupt_thread_mutex);
	ad7146->read(ad7146->dev, AD7146_PWR_CTRL, &data);
	ad7146->power_reg_value = data;
	data |= AD7146_SHUTDOWN_MASK;
	ad7146->write(ad7146->dev, AD7146_PWR_CTRL, data);
	mutex_unlock(&interrupt_thread_mutex);

	return 0;
}

#if defined(CONFIG_HAS_EARLYSUSPEND)
/**
  Suspends the Device.
  This is used to Suspend the I2C client from its operation in the system

  @param dev The Device to be suspended
  @return 0 on success

  @see ad7146_i2c_resume
 */
static int ad7146_i2c_suspend(struct early_suspend *h)
{
	struct ad7146_chip *ad7146;
	ad7146 = container_of(h, struct ad7146_chip, early_suspend);
	ad7146_disable(ad7146);

	AD7146_Driver_Dbg("%s finished\n", __func__);
	return 0;
}

/**
  Resumes the Device.
  This is used to Resume the I2C client from its suspention in the system

  @param dev The Device to be Resumed
  @return 0 on success

  @see ad7146_i2c_suspend
 */
static int ad7146_i2c_resume(struct early_suspend *h)
{
	struct ad7146_chip *ad7146;
	ad7146 = container_of(h, struct ad7146_chip, early_suspend);

	ad7146_enable(ad7146);

	return 0;
}
#endif

/**
* \fn static void ad7146_proximity_state_machine(struct ad7146_chip *ad7146, int idx)
* This is to handle the Sensor Active and the Conversion Complete interrupts
* Based on the interrupt mode configured, the data will be packed and sent
* to userspace via event interface.

@param  ad7146 Device structure for ad7146 chip
@param idx stage number to be handled by the state machine
@return void Nothing Returned

*/

static void ad7146_sensor_state_machine(struct ad7146_chip *ad7146)
{
	struct ad7146_driver_data *sw = NULL;
	unsigned short loop_cnt;
	unsigned int event_value;
	for (loop_cnt = 0; loop_cnt < ad7146->No_of_Stages; loop_cnt++) {
		/* Handle sensor interrupt*/
		sw = &(ad7146->sw[loop_cnt]); /*Loop for Each Stage*/
		if ((ad7146->low_status & HIGH_MASK(sw->index)) &&
		    (ad7146->high_int_enable & HIGH_MASK(sw->index))) {
			unsigned short data_read_ctl;
			/*Error case - Handling
			  Do force calibrate -lower threshold should not Hit*/
			ad7146->read(ad7146->dev,
				AD7146_AMB_COMP_CTRL0_REG, &data_read_ctl);
			data_read_ctl = (data_read_ctl |
					AD7146_FORCED_CAL_MASK);/*chk*/
			ad7146->write(ad7146->dev,
				AD7146_AMB_COMP_CTRL0_REG, data_read_ctl);
			return;
		}

		if (ad7146->high_int_enable & HIGH_MASK(sw->index)) {
			switch (sw->state) {
			case IDLE:
				/*Sensor active*/
				if ((ad7146->high_status & HIGH_MASK(sw->index))
				     == HIGH_MASK(sw->index)) {
					AD7146_Driver_Dbg(
					"proximity %d touched\n", loop_cnt);
					event_value = ((unsigned char)
						       (sw->index << 1)) | 0x81;
					input_event(ad7146->input, EV_MSC,
						    MSC_RAW, event_value);
					input_sync(ad7146->input);
					sw->state = ACTIVE;
				}
				break;
			case ACTIVE:
				/* Sensor inactive*/
				if ((ad7146->high_status & HIGH_MASK(sw->index))
				     != HIGH_MASK(sw->index)) {
					AD7146_Driver_Dbg(
					"proximity %d released\n", loop_cnt);
					event_value = ((unsigned char)
						       (sw->index << 1)) & 0x7F;
					event_value |=  0x1;
					input_event(ad7146->input, EV_MSC,
						    MSC_RAW, event_value);
					input_sync(ad7146->input);
					sw->state = IDLE;
				}
				break;

			default:
					break;
			}
		}
	}
}

/**
 This is to handle the Conversion Complete interrupts
 Based on the interrupt mode configured, the data will be packed and sent
 to userspace via event interface.
 This also has the compensation routine added for the temperature compensation
 The stages 1 through stage 11 can have the tempereature compensation applied
 based on the sensors at STAGE 0.
@note The temperature compensation sensor shouls ne connected to stage 0 only.
@param  ad7146 Device structure for ad7146 chip
@return void Nothing Returned

*/
static void ad7146_conv_complete_state_machine(struct ad7146_chip *ad7146)
{
	unsigned int event_value = 0;

	/* Handle conversion complete interrupt*/
	if ((ad7146->complete_status & ad7146->com_int_enable)) {
		unsigned short data;
		int count;
		unsigned short u16_tp_val = 0;
		for (count = 0; count <= ad7146->LastStageEnabled; count++) {
			if (ad7146->sensor_int_enable & (HIGH_MASK(count))) {
				ad7146->read(ad7146->dev,
						CDC_RESULT_S0 + count, &data);
				if (count == 0) {
					/* This is used for temp compensation*/
					ad7146->temperature_comp_data = data;
				} else {
					u16_tp_val = ad7146_temp_comp(
					&(ad7146->ast_chtempdata[count]),
					ad7146->temperature_comp_data);
					data = data - u16_tp_val;
				}
				event_value =  (data << 16) | (count << 1);
				input_event(ad7146->input, EV_MSC, MSC_RAW,
					    event_value);
#if 0
				input_sync(ad7146->input);
#endif
			}
                        if(count == ad7146->LastStageEnabled){
                          ad7146->read(ad7146->dev,
                                       CDC_RESULT_S0 + count, &data);
                          event_value =  (data << 16) | (count << 1);
                          input_event(ad7146->input, EV_MSC, MSC_RAW,
                                      event_value);
                          input_sync(ad7146->input);
                        }
		}
	}
}

/**
 * \fn static int ad7146_hw_detect(struct ad7146_chip *ad7146)
 * This Routine reads the Device ID to confirm the existance
 * of the Device in the System.

 @param  ad7146 The Device structure
 @return 0 on Successful detection of the device,-ENODEV on err.
 */

static int ad7146_hw_detect(struct ad7146_chip *ad7146)
{
	unsigned short data;

	ad7146->read(ad7146->dev, AD7146_PARTID_REG, &data);
	switch (data & 0xFFF0) {
	case AD7146_PARTID:
		ad7146->product = AD7146_PRODUCT_ID;
		ad7146->version = data & 0xF;
		dev_info(ad7146->dev, "found AD7146 , rev:%d\n",
			 ad7146->version);
		return 0;

	default:
		dev_err(ad7146->dev,
			"ad7146 captouch Not Found, read ID is %04x\n", data);
		return -ENODEV;
	}
}

/**
  WORK Handler -- starts proximity state machine to detect proximity.
  Pack data and send to userspace for all 12 stages.

  @param work The work structure registered.

  @return void nothing returned
 */

static void ad7146_interrupt_thread(struct work_struct *work)
{

	struct ad7146_chip *ad7146 =  container_of(work,
			struct ad7146_chip, work);

	mutex_lock(&interrupt_thread_mutex) ;
	if (ad7146->intr_mode == SENSOR_ACTIVE_INT) {
		ad7146->read(ad7146->dev, STG_LOW_INT_STA_REG,
				&ad7146->low_status);
		ad7146->read(ad7146->dev, STG_HIGH_INT_STA_REG,
				&ad7146->high_status);
		ad7146->read(ad7146->dev, STG_HIGH_INT_EN_REG,
				&ad7146->high_int_enable);
		ad7146_sensor_state_machine(ad7146);

	} else if (ad7146->intr_mode == CONV_COMP_INT) {
		ad7146->read(ad7146->dev, STG_COM_INT_STA_REG,
				&ad7146->complete_status);
		ad7146->read(ad7146->dev, STG_COM_INT_EN_REG,
				&ad7146->com_int_enable);
		ad7146_conv_complete_state_machine(ad7146);
	}
	mutex_unlock(&interrupt_thread_mutex);

}
/**
  IRQ Handler -- services the Interrupt and schedules the work for uploading
  of events to the user space.

  @param handle The data of the AD7146 Device
  @param irq The Interrupt Request queue to be assigned for the device.

  @return IRQ_HANDLED
 */
static irqreturn_t ad7146_isr(int irq, void *handle)
{

	struct ad7146_chip *ad7146 = handle;

	mutex_lock(&interrupt_thread_mutex) ;

	if (!work_pending(&ad7146->work)) {
		schedule_work(&ad7146->work);
	} else {
		ad7146->read(ad7146->dev, STG_COM_INT_STA_REG,
				&ad7146->complete_status);
		ad7146->read(ad7146->dev, STG_LOW_INT_STA_REG,
				&ad7146->low_status);
		ad7146->read(ad7146->dev, STG_HIGH_INT_STA_REG,
				&ad7146->high_status);
		AD7146_Driver_Dbg("WK_pend!\n");
	}

	mutex_unlock(&interrupt_thread_mutex);

	return IRQ_HANDLED;
}

static int ad7146_parse_dt(struct i2c_client *client, struct ad7146_chip *chip)
{
  int ret = 0;
  enum of_gpio_flags flags;

  chip->irq_gpio = of_get_named_gpio_flags(client->dev.of_node,
                                           "adi,irq-gpio", 0, &flags);
  chip->irq = client->irq;
  return ret;
}
/**
  Device probe function
  All initialization routines are handled here
  ISR registration, Work initialization, Input Event registration,
  Sysfs Attributes creation...etc.,

  @param client The I2C Device client of the AD7146
  @param id the i2c device ID from the Supported device ID list give
  during the addition of driver.

  @return 0(Zero) on Sucess,On failure -ENOMEM, -EINVAL ,etc.,
 */

static int __devinit ad7146_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int error = -1;
	struct device *dev = &client->dev;
	struct input_dev *input = NULL;
        struct ad7146_platform_data *pdata = &ad7146_i2c_platform_data;
	struct ad7146_chip *ad7146 = NULL;
	void *drv_mem = NULL;
	int irq = client->irq;
	const struct i2c_device_id *ad7146_ID = id;
	struct ad7146_driver_data *driver_data;

	if (client == NULL) {
		AD7146_Driver_Dbg(KERN_INFO "I2C Client doesn't exist\n");
		error = -EINVAL;
		goto err_out;
	}
	if (pdata == NULL) {
		AD7146_Driver_Dbg(KERN_INFO "Device Platform Data Not Found\n");
		error = -EINVAL;
		goto err_out;

	}
	printk(KERN_INFO "[AD7146]: %s called", __func__);

	AD7146_Driver_Dbg(KERN_INFO "############ %s ###########", __func__);

	if (irq <= 0) {
		dev_err(dev, "IRQ not configured!\n");
		error = -EINVAL;
		goto err_out;
	}

	ad7146 = kzalloc(sizeof(*ad7146) + sizeof(*ad7146->sw) +
			sizeof(*driver_data) * STAGE_NUM, GFP_KERNEL);

	if (!ad7146) {
		error = -ENOMEM;
		goto err_out;
	}
	ad7146->hw = pdata;
	drv_mem = ad7146 + 1;
	ad7146->sw = drv_mem;
	drv_mem += sizeof(*ad7146->sw);

	ad7146->sw = drv_mem;
	drv_mem += sizeof(*driver_data) * STAGE_NUM;

	ad7146->read = ad7146_i2c_read;
	ad7146->write = ad7146_i2c_write;
	ad7146->irq = irq;
	ad7146->dev = dev;
	ad7146->filp_buffer = kzalloc(FILP_BUFF_SIZE, GFP_KERNEL);
	if (!(ad7146->filp_buffer)) {
		error = -ENOMEM;
		goto err_free_chip;
	}

        error = adi7146_config_regulator(client, true, adi7146_vcc_vreg);
        if(error){
          AD7146_Driver_Info("%s Failed to enable REG %d\n", __func__, error);
          goto err_free_chip;
        }
        msleep(100);

	/* check if the device is existing by reading device id of AD7146 */
	error = ad7146_hw_detect(ad7146);
	if (error)
		goto err_free_mem;

	/* initialize and request sw/hw resources */
	INIT_WORK(&ad7146->work, ad7146_interrupt_thread);
	mutex_init(&ad7146->mutex);
	error =  ad7146_hw_init(ad7146);
	if (error < 0) {
          AD7146_Driver_Info("%s H/W init failed\n", __func__);
		goto err_free_mem;
	}
	/* Use the initial table for the temp comp calculation*/
	memcpy(ad7146->ast_chtempdata, init_tcomp_data,
	       sizeof(init_tcomp_data));
	i2c_set_clientdata(client, ad7146);

#ifdef CONFIG_HAS_EARLYSUSPEND
	ad7146->early_suspend.suspend   = (void *)ad7146_i2c_suspend;
	ad7146->early_suspend.resume    = (void *)ad7146_i2c_resume;
	ad7146->early_suspend.level     = EARLY_SUSPEND_LEVEL_DISABLE_FB-1;
	register_early_suspend(&ad7146->early_suspend);
#endif

	/*
	 * Allocate and register ad7146 input device
	 */
	/* all proximitys use one input node */

	input = input_allocate_device();
	if (!input) {
		error = -ENOMEM;
		goto err_free_mem;
	}

	__set_bit(EV_MSC, input->evbit);
	__set_bit(MSC_RAW, input->mscbit);

	ad7146->input = input;

	input->id.bustype = BUS_I2C;
	input->id.product = ad7146->product;
	input->id.version = ad7146->version;
	if (ad7146_ID->name != NULL) {
		AD7146_Driver_Dbg("ad7146_ID->name %s",
				  ad7146_ID->name);
		input->name = ad7146_ID->name;
		AD7146_Driver_Dbg("input->name %s",
				  input->name);
	} else {
		input->name = "ad7146";
	}
        AD7146_Driver_Info("input->name %s", input->name);
	input->dev.parent = dev;

	error = input_register_device(input);
	if (error)
		goto err_free_dev;

	error = sysfs_create_group(&dev->kobj, &ad7146_attr_group);
	if (error)
		goto err_unreg_dev;

        if (client->dev.of_node) {
          error = ad7146_parse_dt(client, ad7146);
          if (error) {
            AD7146_Driver_Info("%s fail to parse device tree!\n", __func__);
            error = -EFAULT;
            goto err_free_sysfs;
          }
          AD7146_Driver_Info("%s called irq_gpio = %d\n", __func__,ad7146->irq_gpio);
        }

        if (gpio_is_valid(ad7146->irq_gpio)) {
          error = gpio_request(ad7146->irq_gpio, "ad7146_irq");
          if (error) {
            AD7146_Driver_Info("%s failed gpio request!\n", __func__);
            goto err_free_irq;
          }

          error = gpio_direction_input(ad7146->irq_gpio);
          if (error) {
            pr_err("%s failed set gpio directiont!\n", __func__);
            goto err_free_gpio;
          }
          pr_info("%s gpio setup properly \n", __func__);
        }

	error = request_threaded_irq(ad7146->irq, NULL, ad7146_isr,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"ad7146_irq", ad7146);
	if (error) {
		dev_err(dev, "irq %d busy?\nDriver init Failed", ad7146->irq);
		goto err_free_gpio;
	}

        ad7146_disable(ad7146);
        ad7146->status = DISABLED;
        ad7146->state = INIT;

	return 0;

err_free_gpio:
        gpio_free(ad7146->irq_gpio);
err_free_irq:
        free_irq(ad7146->irq, ad7146);
err_free_sysfs:
	sysfs_remove_group(&ad7146->dev->kobj, &ad7146_attr_group);
err_unreg_dev:
	input_unregister_device(input);
err_free_dev:
	input_free_device(input);
err_free_mem:
	kfree(ad7146->filp_buffer);
err_free_chip:
	kfree(ad7146);
err_out:
	dev_err(dev, "Failed to setup ad7146 device\n");
	return -ENODEV;
}

/**
  Used to remove device.
  This function is used to remove AD7146 from the system by unregistering AD7146

  @param ad7146 The Device Structure

  @return void Nothing returned
 */
void ad7146_remove(struct ad7146_chip *ad7146)
{
	cancel_work_sync(&ad7146->work);

	sysfs_remove_group(&ad7146->dev->kobj, &ad7146_attr_group);
	free_irq(ad7146->irq, ad7146);
	input_unregister_device(ad7146->input);
	kfree(ad7146->filp_buffer);
	kfree(ad7146);
	AD7146_Driver_Dbg("Event Interface Remove Done\n");
}
#ifdef CONFIG_PM
#ifndef CONFIG_HAS_EARLYSUSPEND
int ad7146_i2c_suspend(struct device *dev)
{
	struct ad7146_chip *ad7146 = i2c_get_clientdata(to_i2c_client(dev));

	printk(KERN_INFO "%s\n", __func__);

	disable_irq(ad7146->irq);

	return ad7146_disable(ad7146);
}

int ad7146_i2c_resume(struct device *dev)
{
	struct ad7146_chip *ad7146 = i2c_get_clientdata(to_i2c_client(dev));
	int err;

	printk(KERN_INFO "%s\n", __func__);
	err = ad7146_enable(ad7146);
	enable_irq(ad7146->irq);

	return err;
}
/**
  Linux device Power manager ops structure
 */
SIMPLE_DEV_PM_OPS(ad7146_pm, ad7146_i2c_suspend, ad7146_i2c_resume);
#endif
#endif


/**
  Removes the Device.
  This is used to Remove the device or the I2C client from the system

  @param client The Client Id to be removed
  @return 0 on success
 */

static int __devexit ad7146_i2c_remove(struct i2c_client *client)
{
	struct ad7146_chip *chip = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&chip->early_suspend);
#endif
	ad7146_remove(chip);

	return 0;
}

/**
  This is the Device ID table for the supported devices.
 */
static const struct i2c_device_id ad7146_id[] = {
	{ "ad7146_SAR_NORM", 0 },
	{ "ad7146_SAR_PROX", 1 },
	{ "ad7146_SAR", 2 }, {},
};
MODULE_DEVICE_TABLE(i2c, ad7146_id);

static const struct of_device_id ad7146_of_id_table[] = {
  {.compatible = "adi,ad7146_SAR_NORM", (void *)0},
  {.compatible = "adi,ad7146_SAR_PROX", (void *)1},
  {.compatible = "adi,ad7146_SAR", (void *)2},
  { },
};

/**
  The file Operation Table
 */
struct i2c_driver ad7146_i2c_driver = {
  .driver = {
    .name = DRIVER_NAME,
    .owner = THIS_MODULE,
    .of_match_table = ad7146_of_id_table,
#ifdef CONFIG_PM
#ifndef CONFIG_HAS_EARLYSUSPEND
    .pm   = &ad7146_pm,
#endif
#endif
  },
  .probe    = ad7146_probe,
  .remove   = __devexit_p(ad7146_i2c_remove),
  .id_table = ad7146_id,
};

/**
  This is an init function called during module insertion -- calls inturn i2c driver probe function
 */
static __init int ad7146_i2c_init(void)
{
	return i2c_add_driver(&ad7146_i2c_driver);
}
module_init(ad7146_i2c_init);

/**
  Called during the module removal
 */
static __exit void ad7146_i2c_exit(void)
{
	i2c_del_driver(&ad7146_i2c_driver);
}

module_exit(ad7146_i2c_exit);
MODULE_DESCRIPTION("Analog Devices ad7146 Sensor Driver");
MODULE_AUTHOR("");
MODULE_LICENSE("GPL");
