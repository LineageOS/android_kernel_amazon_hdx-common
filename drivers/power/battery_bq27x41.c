/*
 * bq27x41_battery.c
 *
 * Copyright (C) Amazon Technologies Inc. All rights reserved.
 * Manish Lachwani (lachwani@lab126.com)
 * Donald Chan (hoiho@lab126.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/reboot.h>
#include <linux/timer.h>
#include <linux/syscalls.h>
#include <linux/sysdev.h>
#include <linux/power_supply.h>
#include <linux/slab.h>

#if defined(CONFIG_ARCH_MSM8974_APOLLO)
#include "battery_bq27x41.h"
#endif

/*
 * I2C registers that need to be read
 */
#define BQ27X41_CONTROL			0x00
#define BQ27X41_TEMP_LOW		0x06
#define BQ27X41_TEMP_HI			0x07
#define BQ27X41_VOLTAGE_LOW		0x08
#define BQ27X41_VOLTAGE_HI		0x09
#define BQ27X41_BATTERY_ID		0x7E
#define BQ27X41_AI_LO			0x14	/* Average Current */
#define BQ27X41_AI_HI			0x15
#define BQ27X41_FLAGS_LO		0x0a
#define BQ27X41_FLAGS_HI		0x0b
#define BQ27X41_BATTERY_RESISTANCE	20
#define BQ27X41_CSOC_L			0x2c	/* Compensated state of charge */
#define BQ27X41_CSOC_H			0x2d
#define BQ27X41_CAC_L			0x10	/* milliamp-hour */
#define BQ27X41_CAC_H			0x11
#define BQ27X41_FCC_L			0x12
#define BQ27X41_FCC_H			0x13
#define BQ27X41_AE_L			0x22
#define BQ27X41_AE_H			0x23
#define BQ27X41_CYCL_H			0x29
#define BQ27X41_CYCL_L			0x28
#define BQ27X41_FAC_H			0x0F
#define BQ27X41_FAC_L			0x0E
#define BQ27X41_NAC_H			0x0D
#define BQ27X41_NAC_L			0x0C
#define BQ27X41_CYCT_L			0x2A
#define BQ27X41_CYCT_H			0x2B
#define BQ27X41_TTE_L			0x16
#define BQ27X41_TTE_H			0x17
#define BQ27X41_DATA_FLASH_BLOCK	0x3f
#define BQ27X41_MANUFACTURER_OFFSET	0x40	/* Offset for manufacturer ID */
#define BQ27X41_MANUFACTURER_LENGTH	0x8	/* Length of manufacturer ID */
#define BQ27X41_FLAGS_DSG		(1 << 0)
#define BQ27X41_FLAGS_CHG		(1 << 8)
#define BQ27X41_FLAGS_FC		(1 << 9)
#define BQ27X41_FLAGS_OTD		(1 << 14)
#define BQ27X41_FLAGS_OTC		(1 << 15)

#define BQ27X41_I2C_ADDRESS		0x55	/* Battery I2C address on the bus */
#define BQ27X41_TEMP_LOW_THRESHOLD	0	/* Low temp cutoff in 0.1 C */
#define BQ27X41_TEMP_HI_THRESHOLD	600	/* High temp cutoff in 0.1 C */
#define BQ27X41_TEMP_MID_THRESHOLD	100	/* Mid temperature threshold in 0.1 C */
#define BQ27X41_VOLT_LOW_THRESHOLD	2500	/* Low voltage threshold in mV */
#define BQ27X41_VOLT_HI_THRESHOLD	4400	/* High voltage threshold in mV */
#define BQ27X41_VOLT_CRIT_THRESHOLD	3200	/* Critically low votlage threshold in mV */
#define BQ27X41_BATTERY_INTERVAL	5000	/* 5 second duration */
#define BQ27X41_BATTERY_INTERVAL_EARLY	1000	/* 1 second on probe */
#define BQ27X41_BATTERY_INTERVAL_START	5000	/* 5 second timer on startup */
#define BQ27X41_BATTERY_INTERVAL_ERROR	10000	/* 10 second timer after an error */
#define BQ27X41_DEVICE_TYPE		0x0541	/* Device type of the gas gauge */
#define BQ27X41_DEVICE_TYPE_V2		0x0741

#define DRIVER_NAME			"bq27x41"
#define DRIVER_VERSION			"1.0"
#define DRIVER_AUTHOR			"Donald Chan"

#define GENERAL_ERROR			0x0001
#define ID_ERROR			0x0002
#define TEMP_RANGE_ERROR		0x0004
#define VOLTAGE_ERROR			0x0008
#define DATA_CHANGED			0x0010

#define BQ27X41_BATTERY_RETRY_THRESHOLD	5	/* Failed retry case - 5 */

#define BQ27X41_ERROR_THRESHOLD		4	/* Max of 5 errors at most before sending to userspace */

#define BQ27X41_BATTERY_RELAXED_THRESH	7200	/* Every 10 hours or 36000 seconds */

int bq27x41_battery_tte = 0;

EXPORT_SYMBOL(bq27x41_battery_tte);

struct bq27x41_info {
	int battery_voltage;
	int battery_temperature;
	int battery_current;
	int battery_capacity;
	int battery_status;
	int battery_health;
	int battery_remaining_charge;
	int battery_remaining_charge_design;
	int battery_full_charge;
	int battery_full_charge_design;
	int battery_available_energy;
	int i2c_err;
	int err_flags;
	u8 manufacturer_id[BQ27X41_MANUFACTURER_LENGTH + 1];
	struct i2c_client *client;
	struct power_supply battery;
	struct delayed_work battery_work;
	/* Time when system enters full suspend */
	struct timespec suspend_time;
	/* Time when system enters early suspend */
	struct timespec early_suspend_time;
	/* Battery capacity when system enters full suspend */
	int suspend_capacity;
	/* Battery capacity when system enters early suspend */
	int early_suspend_capacity;
	unsigned int poll_interval;
};

static int temp_error_counter = 0;
static int volt_error_counter = 0;

#if defined(CONFIG_ARCH_MSM8974_APOLLO)
static int32_t current_battery_voltage;
static int32_t current_battery_temperature;
static int32_t current_battery_current;
#endif

static struct i2c_client *bq27x41_battery_i2c_client;

static int bq27x41_i2c_read(u8 reg_num, void *value, int size)
{
	s32 error;

	if (size == 1) {
		error = i2c_smbus_read_byte_data(bq27x41_battery_i2c_client, reg_num);

		if (error < 0) {
			dev_warn(&bq27x41_battery_i2c_client->dev,
					"i2c read retry\n");
			return -EIO;
		}

		*((u8 *)value) = (u8)(error & 0xff);

		return 0;
	} else if (size == 2) {
		error = i2c_smbus_read_word_data(bq27x41_battery_i2c_client, reg_num);

		if (error < 0) {
			dev_warn(&bq27x41_battery_i2c_client->dev,
					"i2c read retry\n");
			return -EIO;
		}

		*((u16 *)value) = (u16)(le16_to_cpu(error) & 0xffff);

		return 0;
	} else {
		dev_err(&bq27x41_battery_i2c_client->dev,
				"Invalid data size: %d\n", size);
		return -1;
	}
}

static int bq27x41_battery_read_voltage(int *voltage)
{
	u16 volts = 0;
	int error = 0;

	error = bq27x41_i2c_read(BQ27X41_VOLTAGE_LOW, &volts, sizeof(volts));

	if (!error)
		*voltage = volts;

	return error;
}

static int bq27x41_battery_read_current(int *curr)
{
	s16 c = 0;
	int error = 0;

	error = bq27x41_i2c_read(BQ27X41_AI_LO, &c, sizeof(c));

	if (!error)
		*curr = c;

	return error;
}

static int bq27x41_battery_read_capacity(int *capacity)
{
	u16 c = 0;
	int error = 0;

	error = bq27x41_i2c_read(BQ27X41_CSOC_L, &c, sizeof(c));

	if (!error)
		*capacity = c;

	return error;
}

static int bq27x41_battery_read_flags(int *flags)
{
	u16 f = 0;
	int error = 0;

	error = bq27x41_i2c_read(BQ27X41_FLAGS_LO, &f, sizeof(f));

	if (!error)
		*flags = f;

	return error;
}

static int bq27x41_battery_read_remaining_charge(int *mAh)
{
	u16 charge = 0;
	int error = 0;

	error = bq27x41_i2c_read(BQ27X41_CAC_L, &charge, sizeof(charge));

	if (!error)
		*mAh = charge;

	return error;
}

static int bq27x41_battery_read_remaining_charge_design(int *mAh)
{
	u16 charge = 0;
	int error = 0;

	error = bq27x41_i2c_read(BQ27X41_NAC_L, &charge, sizeof(charge));

	if (!error)
		*mAh = charge;

	return error;
}

static int bq27x41_battery_read_full_charge(int *mAh)
{
	u16 charge = 0;
	int error = 0;

	error = bq27x41_i2c_read(BQ27X41_FCC_L, &charge, sizeof(charge));

	if (!error)
		*mAh = charge;

	return error;
}

static int bq27x41_battery_read_full_charge_design(int *mAh)
{
	u16 charge = 0;
	int error = 0;

	error = bq27x41_i2c_read(BQ27X41_FAC_L, &charge, sizeof(charge));

	if (!error)
		*mAh = charge;

	return error;
}


static int bq27x41_battery_read_available_energy(int *energy)
{
	u16 ae = 0;
	int error = 0;

	error = bq27x41_i2c_read(BQ27X41_AE_L, &ae, sizeof(ae));

	if (!error)
		*energy = ae;

	return error;
}

/* Read TTE */
static int bq27x41_battery_read_tte(int *tte)
{
	u16 value = 0;
	int error = 0;

	error = bq27x41_i2c_read(BQ27X41_TTE_L, &value, sizeof(value));

	if (!error)
		*tte = value;

	return error;
}

#if 0
/* Read Full Available Capacity */
static int bq27x41_battery_read_fac(int *fac)
{
	u16 value = 0;
	int error = 0;

	error = bq27x41_i2c_read(BQ27X41_FAC_L, &value, sizeof(value));

	if (!error)
		*fac = value;

	return error;
}
#endif

static int bq27x41_battery_read_temperature(int *temperature)
{
	s16 temp = 0;
	int celsius = 0, error = 0;

	error = bq27x41_i2c_read(BQ27X41_TEMP_LOW, &temp, sizeof(temp));

	if (!error) {
		/* Convert 0.1 K to 0.1 C */
		celsius = temp - 2732;
		*temperature = celsius;
	}

	return error;
}

static int bq27x41_read_manufacturer_id(struct bq27x41_info *info)
{
	/* Enable access to manufacturer info block A */
	i2c_smbus_write_byte_data(info->client, BQ27X41_DATA_FLASH_BLOCK, 1);
	msleep(10);

	memset(info->manufacturer_id, 0, sizeof(info->manufacturer_id));

	if (i2c_smbus_read_i2c_block_data(info->client,
			BQ27X41_MANUFACTURER_OFFSET,
			BQ27X41_MANUFACTURER_LENGTH,
			info->manufacturer_id) < 0) {
		printk(KERN_WARNING "bq27x41: Unable to get manufacturer ID\n");
		return -1;
	}

	/* Check for valid battery id */
#if defined(CONFIG_ARCH_MSM8974_THOR)
	if (strncmp(info->manufacturer_id,
			"MCN KC4 ", BQ27X41_MANUFACTURER_LENGTH) &&
		strncmp(info->manufacturer_id,
			"SWE KC4 ", BQ27X41_MANUFACTURER_LENGTH))
#elif defined(CONFIG_ARCH_MSM8974_APOLLO)
	if (strncmp(info->manufacturer_id,
			"DSY APL ", BQ27X41_MANUFACTURER_LENGTH) &&
		strncmp(info->manufacturer_id,
			"SWE APL ", BQ27X41_MANUFACTURER_LENGTH))
#else
	if (0)
#endif
	{
		strlcpy(info->manufacturer_id,
			"UNKNOWN", BQ27X41_MANUFACTURER_LENGTH);
	}

	return 0;
}

#if defined(CONFIG_ARCH_MSM8974_APOLLO)
int32_t get_battery_parameters(int prop_type)
{
	switch (prop_type) {
	case GET_BAT_VOLTAGE:
			return current_battery_voltage;
			break;
	case GET_BAT_TEMPERATURE:
			return current_battery_temperature;
			break;
	case GET_BAT_CURRENT:
			return current_battery_current;
			break;
	default:
			break;
	}
	return 0;

}
#endif

/* Main battery timer task */
static void battery_handle_work(struct work_struct *work)
{
	int err = 0;
	int batt_err_flag = 0;
	int value = 0, flags = 0;
	struct bq27x41_info *info = container_of(work,
		struct bq27x41_info, battery_work.work);

	err = bq27x41_battery_read_temperature(&value);
	if (err) {
		batt_err_flag |= GENERAL_ERROR;
		goto out;
	} else {
		if (info->battery_temperature != value) {
			info->battery_temperature = value;
			batt_err_flag |= DATA_CHANGED;
		}

		info->i2c_err = 0;
	}

	/*
	 * Check for the temperature range violation
	 */
	if ( (info->battery_temperature <= BQ27X41_TEMP_LOW_THRESHOLD) ||
		(info->battery_temperature >= BQ27X41_TEMP_HI_THRESHOLD) ) {
		temp_error_counter++;
	} else {
		temp_error_counter = 0;
		info->err_flags &= ~TEMP_RANGE_ERROR;
	}

	if (temp_error_counter > BQ27X41_ERROR_THRESHOLD) {
		info->err_flags |= TEMP_RANGE_ERROR;
		temp_error_counter = 0;

		printk(KERN_WARNING
			"bq27x41: battery has reached critical thermal level, "
			"(%dC) shutting down...\n",
			info->battery_temperature/10);

		machine_power_off();
	}

	err = bq27x41_battery_read_voltage(&value);
	if (err) {
		batt_err_flag |= GENERAL_ERROR;
		goto out;
	} else {
		if (info->battery_voltage != value) {
			info->battery_voltage = value;
			batt_err_flag |= DATA_CHANGED;
		}

		info->i2c_err = 0;
	}

	/* Check for critical battery voltage */
	if (info->battery_voltage <= BQ27X41_VOLT_CRIT_THRESHOLD) {
		printk(KERN_WARNING
			"bq27x41: battery has reached critically low level, "
			"shutting down...\n");

		sys_sync();
		orderly_poweroff(true);
	}

	/*
	 * Check for the battery voltage range violation
	 */
	if ( (info->battery_voltage <= BQ27X41_VOLT_LOW_THRESHOLD) ||
		(info->battery_voltage >= BQ27X41_VOLT_HI_THRESHOLD) ) {
		volt_error_counter++;
	} else {
		volt_error_counter = 0;
		info->err_flags &= ~VOLTAGE_ERROR;
	}

	if (volt_error_counter > BQ27X41_ERROR_THRESHOLD) {
		printk(KERN_ERR "battery driver voltage - %d mV\n",
				info->battery_voltage);
		info->err_flags |= VOLTAGE_ERROR;
		volt_error_counter = 0;
	}

	/*
	 * Check for the battery current
	 */
	err = bq27x41_battery_read_current(&value);
	if (err) {
		batt_err_flag |= GENERAL_ERROR;
		goto out;
	} else {
		if (info->battery_current != value) {
			info->battery_current = value;
			batt_err_flag |= DATA_CHANGED;
		}

		info->i2c_err = 0;
	}

	/*
	 * Read the gas gauge capacity
	 */
	err = bq27x41_battery_read_capacity(&value);
	if (err) {
		batt_err_flag |= GENERAL_ERROR;
		goto out;
	} else {
		if (info->battery_capacity != value) {
			info->battery_capacity = value;
			batt_err_flag |= DATA_CHANGED;
		}

		info->i2c_err = 0;
	}

	/*
	 * Check the battery status
	 */
	err = bq27x41_battery_read_flags(&flags);
	if (err) {
		batt_err_flag |= GENERAL_ERROR;
		goto out;
	} else {
		value = info->battery_status;

		if ((info->battery_capacity == 100)
		    && (info->battery_current == 0)) {
		  value = POWER_SUPPLY_STATUS_FULL;
		} else if (info->battery_current == 0) {
		  value = POWER_SUPPLY_STATUS_NOT_CHARGING;
		} else if (info->battery_current > 0) {
		  value = POWER_SUPPLY_STATUS_CHARGING;
		} else {
		  value = POWER_SUPPLY_STATUS_DISCHARGING;
		}

		if (info->battery_status != value) {
			info->battery_status = value;
			batt_err_flag |= DATA_CHANGED;
		}

		if (flags & (BQ27X41_FLAGS_OTC | BQ27X41_FLAGS_OTD)) {
			value = POWER_SUPPLY_HEALTH_OVERHEAT;
		} else {
			value = POWER_SUPPLY_HEALTH_GOOD;
		}

		if (info->battery_health != value) {
			info->battery_health = value;
			batt_err_flag |= DATA_CHANGED;
		}

		info->i2c_err = 0;
	}

	/*
	 * Read the current battery mAH
	 */
	err = bq27x41_battery_read_remaining_charge(&value);
	if (err) {
		batt_err_flag |= GENERAL_ERROR;
		goto out;
	} else {
		if (info->battery_remaining_charge != value) {
			info->battery_remaining_charge = value;
			batt_err_flag |= DATA_CHANGED;
		}

		info->i2c_err = 0;
	}

	/*
	 * Read the current battery mAH (uncompensated)
	 */
	err = bq27x41_battery_read_remaining_charge_design(&value);
	if (err) {
		batt_err_flag |= GENERAL_ERROR;
		goto out;
	} else {
		if (info->battery_remaining_charge_design != value) {
			info->battery_remaining_charge_design = value;
			batt_err_flag |= DATA_CHANGED;
		}

		info->i2c_err = 0;
	}

	/*
	 * Read the manufacturer ID
	 */
	err = bq27x41_read_manufacturer_id(info);
	if (err) {
		batt_err_flag |= GENERAL_ERROR;
		goto out;
	} else {
		info->i2c_err = 0;
	}

	/*
	 * Read the full battery mAH
	 */
	err = bq27x41_battery_read_full_charge(&value);
	if (err) {
		batt_err_flag |= GENERAL_ERROR;
		goto out;
	} else {
		if (info->battery_full_charge != value) {
			info->battery_full_charge = value;
			batt_err_flag |= DATA_CHANGED;
		}

		info->i2c_err = 0;
	}

	/*
	 * Read the full battery mAH (uncompensated)
	 */
	err = bq27x41_battery_read_full_charge_design(&value);
	if (err) {
		batt_err_flag |= GENERAL_ERROR;
		goto out;
	} else {
		if (info->battery_full_charge_design != value) {
			info->battery_full_charge_design = value;
			batt_err_flag |= DATA_CHANGED;
		}

		info->i2c_err = 0;
	}

	/*
	 * Read the available energy
	 */
	err = bq27x41_battery_read_available_energy(&value);
	if (err) {
		batt_err_flag |= GENERAL_ERROR;
		goto out;
	} else {
		if (info->battery_available_energy != value) {
			info->battery_available_energy = value;
			batt_err_flag |= DATA_CHANGED;
		}

		info->i2c_err = 0;
	}

	/* TTE readings */
	err =  bq27x41_battery_read_tte(&bq27x41_battery_tte);
	if (err) {
		batt_err_flag |= GENERAL_ERROR;
		goto out;
	} else {
		info->i2c_err = 0;
	}

out:
	if (batt_err_flag) {
		if (++info->i2c_err == BQ27X41_BATTERY_RETRY_THRESHOLD) {
			printk(KERN_ERR "bq27x41 battery: i2c read error, retry exceeded\n");
			info->err_flags |= GENERAL_ERROR;
			info->i2c_err = 0;
		}
	} else {
		info->err_flags &= ~GENERAL_ERROR;
		info->i2c_err = 0;
	}

	pr_debug("temp: %d, volt: %d, current: %d, capacity: %d%%, mAH: %d\n",
		info->battery_temperature / 10, info->battery_voltage,
		info->battery_current, info->battery_capacity,
		info->battery_remaining_charge);

	/* Send uevent up if data has changed */
	if (batt_err_flag & DATA_CHANGED)
		power_supply_changed(&info->battery);

	if (batt_err_flag & GENERAL_ERROR) {
		/* Notify upper layers battery is dead */
		info->battery_health = POWER_SUPPLY_HEALTH_UNKNOWN;
		power_supply_changed(&info->battery);

		schedule_delayed_work(&info->battery_work,
			msecs_to_jiffies(BQ27X41_BATTERY_INTERVAL_ERROR));
	} else {
		schedule_delayed_work(&info->battery_work,
			msecs_to_jiffies(info->poll_interval));
	}

	return;
}

static const struct i2c_device_id bq27x41_id[] =  {
        { "bq27x41", 0 },
        { },
};
MODULE_DEVICE_TABLE(i2c, bq27x41_id);

static int bq27x41_get_property(struct power_supply *ps,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	struct bq27x41_info *info = container_of(ps,
			struct bq27x41_info, battery);
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = info->battery_status;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		/* Convert mV to uV */
		val->intval = info->battery_voltage * 1000;
#if defined(CONFIG_ARCH_MSM8974_APOLLO)
		current_battery_voltage = info->battery_voltage * 1000;
#endif
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		/* Convert mA to uA */
		val->intval = info->battery_current * 1000;
#if defined(CONFIG_ARCH_MSM8974_APOLLO)
		current_battery_current = info->battery_current * 1000;
#endif
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = info->battery_capacity;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = info->battery_temperature;
#if defined(CONFIG_ARCH_MSM8974_APOLLO)
		current_battery_temperature = info->battery_temperature;
#endif
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = info->battery_health;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		/* Convert mAh to uAh */
		val->intval = info->battery_remaining_charge * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW_DESIGN:
		/* Convert mAh to uAh */
		val->intval = info->battery_remaining_charge_design * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		/* Convert mAh to uAh */
		val->intval = info->battery_full_charge * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		/* Convert mAh to uAh */
		val->intval = info->battery_full_charge_design * 1000;
		break;
	case POWER_SUPPLY_PROP_ENERGY_AVG:
		/* Convert mW to uW */
		val->intval = info->battery_available_energy * 1000;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = info->manufacturer_id;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property bq27x41_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_NOW_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_AVG,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};

static ssize_t bq27x41_poll_interval_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct bq27x41_info *info = i2c_get_clientdata(to_i2c_client(dev));

	return snprintf(buf, 50, "%d\n", info->poll_interval);
}

static ssize_t bq27x41_poll_interval_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct bq27x41_info *info = i2c_get_clientdata(to_i2c_client(dev));
	unsigned long interval;
	int ret = -1;

	ret = kstrtoul(buf, 10, &interval);
	if (ret) {
		printk(KERN_ERR "%s: unable to parse value\n", __func__);
		return ret;
	}

	if (interval != info->poll_interval) {
		info->poll_interval = interval;

		/* Reschedule with new interval */
		cancel_delayed_work_sync(&info->battery_work);
		schedule_delayed_work(&info->battery_work,
			msecs_to_jiffies(info->poll_interval));
	}

	return len;
}
static DEVICE_ATTR(poll_interval, S_IRUGO | S_IWUSR | S_IWGRP,
			bq27x41_poll_interval_show,
			bq27x41_poll_interval_store);

static struct attribute *bq27x41_attrs[] = {
	&dev_attr_poll_interval.attr,
	NULL,
};

static struct attribute_group bq27x41_attrs_group = {
	.attrs = bq27x41_attrs,
};

static int bq27x41_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct bq27x41_info *info = NULL;
	int ret = 0;
	int dev_ver = 0;

	pr_info("%s: Entering probe...\n", DRIVER_NAME);

        bq27x41_battery_i2c_client = client;
        //bq27x41_battery_i2c_client->addr = BQ27X41_I2C_ADDRESS;

	if ((ret = i2c_smbus_write_word_data(client,
				BQ27X41_CONTROL, 0x0001)) < 0) {
		pr_err("error writing device type: %d\n", ret);
		bq27x41_battery_i2c_client = NULL;
		return -ENODEV;
	}

	dev_ver = i2c_smbus_read_word_data(client, BQ27X41_CONTROL);

	if (dev_ver != BQ27X41_DEVICE_TYPE &&
		dev_ver != BQ27X41_DEVICE_TYPE_V2) {
		pr_warning("%s: Failed to detect device type (%d), "
				"possibly drained battery?\n",
				DRIVER_NAME, dev_ver);
	}

        if (!(info = kzalloc(sizeof(*info), GFP_KERNEL))) {
		bq27x41_battery_i2c_client = NULL;
                return -ENOMEM;
        }

        client->addr = BQ27X41_I2C_ADDRESS;

        i2c_set_clientdata(client, info);

        info->client = client;
	info->battery.name = "bq27x41";
	info->battery.type = POWER_SUPPLY_TYPE_BATTERY;
	info->battery.get_property = bq27x41_get_property;
	info->battery.properties = bq27x41_battery_props;
	info->battery.num_properties = ARRAY_SIZE(bq27x41_battery_props);

	/* Set some initial dummy values */
	info->battery_capacity = 5;
	info->battery_voltage = 3500;
	info->battery_temperature = 0;
	info->battery_status = POWER_SUPPLY_STATUS_UNKNOWN;

	info->suspend_capacity = -1;
	info->early_suspend_capacity = -1;

	bq27x41_read_manufacturer_id(info);

	ret = power_supply_register(&client->dev, &info->battery);
	if (ret) {
		dev_err(&client->dev, "failed: power supply register\n");
		i2c_set_clientdata(client, NULL);
		kfree(info);
		return ret;
	}

	INIT_DELAYED_WORK(&info->battery_work, battery_handle_work);

	info->poll_interval = BQ27X41_BATTERY_INTERVAL;

	/* Register the sysfs nodes */
	ret = sysfs_create_group(&client->dev.kobj, &bq27x41_attrs_group);
	if (ret) {
		dev_err(&client->dev, "Unable to create sysfs group\n");
		i2c_set_clientdata(client, NULL);
		kfree(info);
		return ret;
	}

	schedule_delayed_work(&info->battery_work,
		msecs_to_jiffies(BQ27X41_BATTERY_INTERVAL_EARLY));

	pr_info("%s: probe succeeded\n", DRIVER_NAME);

        return 0;
}

static int bq27x41_remove(struct i2c_client *client)
{
        struct bq27x41_info *info = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&info->battery_work);

	sysfs_remove_group(&client->dev.kobj, &bq27x41_attrs_group);

        i2c_set_clientdata(client, info);

        return 0;
}

static void bq27x41_shutdown(struct i2c_client *client)
{
	struct bq27x41_info *info = i2c_get_clientdata(client);
	cancel_delayed_work_sync(&info->battery_work);
}

static int bq27x41_battery_suspend(struct i2c_client *client, pm_message_t state)
{
        struct bq27x41_info *info = i2c_get_clientdata(client);
	int value;
	int err;

	cancel_delayed_work_sync(&info->battery_work);

	/* Cache the current capacity */
	info->suspend_capacity = info->battery_capacity;

	info->i2c_err = 0;

	err = bq27x41_battery_read_remaining_charge(&value);
	if (!err) {
		info->battery_remaining_charge = value;
	} else {
		info->i2c_err++;
	}

	err = bq27x41_battery_read_full_charge(&value);
	if (!err) {
		info->battery_full_charge = value;
	} else {
		info->i2c_err++;
	}

	err = bq27x41_battery_read_remaining_charge_design(&value);
	if (!err) {
		info->battery_remaining_charge_design = value;
	} else {
		info->i2c_err++;
	}

	err = bq27x41_battery_read_full_charge_design(&value);
	if (!err) {
		info->battery_full_charge_design = value;
	} else {
		info->i2c_err++;
	}

	if (info->i2c_err) {
		printk(KERN_ERR "bq27x41 battery: "
			"%d i2c read errors, sysfs entries invalid\n",
			info->i2c_err);
		info->i2c_err = 0;
	}

	/* Mark the suspend time */
	info->suspend_time = current_kernel_time();

	return 0;
}

static int bq27x41_battery_resume(struct i2c_client *client)
{
	int err = 0, value = 0;
	struct bq27x41_info *info = i2c_get_clientdata(client);

	/*
	 * Check for the battery current
	 */
	err += bq27x41_battery_read_current(&value);

	if (!err) {
		info->battery_current = value;
		info->i2c_err = 0;
	} else {
		info->i2c_err++;
	}
	/*
	 * Check for the battery voltage
	 */
	err += bq27x41_battery_read_voltage(&value);

	if (!err) {
		info->battery_voltage = value;
		info->i2c_err = 0;
	} else {
		info->i2c_err++;
	}

	/*
	 * Check for remaining charge
	 */
	err += bq27x41_battery_read_remaining_charge(&value);

	if (!err) {
		info->battery_remaining_charge = value;
		info->i2c_err = 0;
	} else {
		info->i2c_err++;
	}

	/*
	 * Check for full battery mAH
	 */
	err += bq27x41_battery_read_full_charge(&value);

	if (!err) {
		info->battery_full_charge = value;
		info->i2c_err = 0;
	} else {
		info->i2c_err++;
	}

	/*
	 * Check for remaining charge (uncompensated)
	 */
	err += bq27x41_battery_read_remaining_charge_design(&value);

	if (!err) {
		info->battery_remaining_charge_design = value;
		info->i2c_err = 0;
	} else {
		info->i2c_err++;
	}

	/*
	 * Read the gas gauge capacity
	 */
	err += bq27x41_battery_read_capacity(&value);

	if (!err) {
		info->battery_capacity = value;
		info->i2c_err = 0;
	} else {
		info->i2c_err++;
	}

	/*
	 * Check for battery temperature
	 */
	err += bq27x41_battery_read_temperature(&value);

	if (!err) {
		info->battery_temperature = value;
		info->i2c_err = 0;
	} else {
		info->i2c_err++;
	}

	err = bq27x41_battery_read_remaining_charge_design(&value);
	if (!err) {
		info->battery_remaining_charge_design = value;
	} else {
		info->i2c_err++;
	}

	err = bq27x41_battery_read_full_charge_design(&value);
	if (!err) {
		info->battery_full_charge_design = value;
	} else {
		info->i2c_err++;
	}

	/* Report to upper layers */
	power_supply_changed(&info->battery);

	schedule_delayed_work(&info->battery_work,
		msecs_to_jiffies(info->poll_interval));

	return 0;
}

static unsigned short normal_i2c[] = { BQ27X41_I2C_ADDRESS, I2C_CLIENT_END };

static const struct of_device_id bq27x41_match[] = {
	{ .compatible = "ti,bq27741", },
	{ },
};

static struct i2c_driver bq27x41_i2c_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(bq27x41_match),
	},
	.probe = bq27x41_probe,
	.remove = bq27x41_remove,
	.shutdown = bq27x41_shutdown,
	.suspend = bq27x41_battery_suspend,
	.resume = bq27x41_battery_resume,
	.id_table = bq27x41_id,
	.address_list = normal_i2c,
};

static int __init bq27x41_battery_init(void)
{
	int ret = 0;

	ret = i2c_add_driver(&bq27x41_i2c_driver);

	if (ret) {
		printk(KERN_ERR "bq27x41 battery: Could not add driver\n");
		return ret;
	}
	return 0;
}

static void __exit bq27x41_battery_exit(void)
{
	i2c_del_driver(&bq27x41_i2c_driver);
}

module_init(bq27x41_battery_init);
module_exit(bq27x41_battery_exit);

MODULE_DESCRIPTION("BQ27X41 Battery Driver");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_LICENSE("GPL");
