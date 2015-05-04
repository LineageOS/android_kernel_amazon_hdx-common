#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/kobject.h>
#include "msm_cci.h"

enum {
	SUCCESS,
	FAILED,
};

struct msm_camera_cci_ctrl cci_ctrl;
struct msm_camera_i2c_reg_conf write_conf = {
	.cmd_type = MSM_CAMERA_I2C_CMD_WRITE,
	.mask = 0,
};

struct msm_camera_cci_client cci0_client = {
	.cci_i2c_master = MASTER_0,
	.retries = 3,
	.id_map = 0,
};

struct msm_camera_cci_client cci1_client = {
	.cci_i2c_master = MASTER_1,
	.retries = 3,
	.id_map = 0,
};

uint8_t read_data[8192];
uint8_t read_size = 1;
uint8_t status = FAILED;

char type_string[100] = "---INVALID---";

inline int setup_cci_client(int cci_master, uint8_t slave_addr)
{
	if (cci_master == 0) {
		cci0_client.cci_subdev = msm_cci_get_subdev();
		cci0_client.sid = slave_addr;
		cci0_client.cid = 0;
		cci0_client.freq = 0;
		cci0_client.timeout = 0;
		cci_ctrl.cci_info = &cci0_client;
	} else if (cci_master == 1) {
		cci1_client.cci_subdev = msm_cci_get_subdev();
		cci1_client.sid = slave_addr;
		cci1_client.cid = 0;
		cci1_client.freq = 0;
		cci1_client.timeout = 0;
		cci_ctrl.cci_info = &cci1_client;
	} else {
		printk("Only CCI Masters 0 & 1 available\n");
		return -ENODEV;
	}

	return 0;
}

char *print_type(int addr_type, int data_type)
{
	sprintf(type_string, "AddrType:");
	switch (addr_type) {
	case MSM_CAMERA_I2C_WORD_ADDR:
		strcat(type_string, "WORD");
		break;
	case MSM_CAMERA_I2C_BYTE_ADDR:
		strcat(type_string, "BYTE");
		break;
	}

	strcat(type_string, "DataType:");
	switch (data_type) {
	case MSM_CAMERA_I2C_WORD_DATA:
		strcat(type_string, "WORD");
		break;
	case MSM_CAMERA_I2C_BYTE_DATA:
		strcat(type_string, "BYTE");
		break;
	}
	return type_string;
}

int32_t read_cci(int cci_master, uint8_t slave_addr, uint16_t address,
		 enum msm_camera_i2c_reg_addr_type addr_type, int num_bytes)
{
	int rc = 0, rc2 = 0;
	pr_err("[%s] master[%d] SlaveAddr[%x] -- %s addr[%x] \n", __func__,
	       cci_master, slave_addr, print_type(addr_type, num_bytes),
	       address);

	rc = setup_cci_client(cci_master, slave_addr);
	if (rc)
		return rc;
	cci_ctrl.cfg.cci_i2c_read_cfg.addr = address;
	cci_ctrl.cfg.cci_i2c_read_cfg.data = read_data;
	cci_ctrl.cfg.cci_i2c_read_cfg.num_byte = num_bytes;
	cci_ctrl.cfg.cci_i2c_read_cfg.addr_type = addr_type;

	cci_ctrl.cmd = MSM_CCI_INIT;
	rc = v4l2_subdev_call(msm_cci_get_subdev(),
			      core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);

	if (rc) {
		pr_err("MSM_CCI_INIT Failed rc=%d", rc);
		goto exit1;
	}

	cci_ctrl.cmd = MSM_CCI_I2C_READ;
	rc = v4l2_subdev_call(msm_cci_get_subdev(),
			      core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
	if (rc) {
		pr_err("MSM_CCI_I2C_READ Failed rc=%d", rc);
		goto exit1;
	}

	read_size = num_bytes;

 exit1:
	cci_ctrl.cmd = MSM_CCI_RELEASE;
	rc2 = v4l2_subdev_call(msm_cci_get_subdev(),
			      core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
	if (rc2) {
		pr_err("MSM_CCI_RELEASE Failed rc=%d", rc2);
	}
	return (rc != 0) ? rc : rc2;
}

int32_t write_cci(int cci_master, uint8_t slave_addr, uint16_t address,
		  uint16_t value, enum msm_camera_i2c_reg_addr_type addr_type,
		  enum msm_camera_i2c_data_type data_type)
{
	int rc = 0, rc2 = 0;
	pr_err("[%s] master[%d] SlaveAddr[%x] -- %s addr[%x] value[%x] \n",
	       __func__, cci_master, slave_addr, print_type(addr_type,
							    data_type), address,
	       value);
	setup_cci_client(cci_master, slave_addr);
	cci_ctrl.cfg.cci_i2c_write_cfg.data_type = data_type;
	cci_ctrl.cfg.cci_i2c_write_cfg.addr_type = addr_type;

	write_conf.dt = data_type;
	write_conf.reg_addr = address;
	write_conf.reg_data = value;

	cci_ctrl.cfg.cci_i2c_write_cfg.reg_conf_tbl = &write_conf;
	cci_ctrl.cfg.cci_i2c_write_cfg.size = 1;

	cci_ctrl.cmd = MSM_CCI_INIT;
	rc = v4l2_subdev_call(msm_cci_get_subdev(),
			      core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);

	if (rc) {
		pr_err("MSM_CCI_INIT Failed rc=%d", rc);
		goto exit2;
	}

	cci_ctrl.cmd = MSM_CCI_I2C_WRITE;
	rc = v4l2_subdev_call(msm_cci_get_subdev(),
			      core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
	if (rc) {
		pr_err("MSM_CCI_I2C_WRITE Failed rc=%d", rc);
		goto exit2;
	}

 exit2:
	cci_ctrl.cmd = MSM_CCI_RELEASE;
	rc2 = v4l2_subdev_call(msm_cci_get_subdev(),
			      core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
	if (rc2) {
		pr_err("MSM_CCI_RELEASE Failed rc=%d", rc2);
	}
	return (rc != 0) ? rc : rc2;
}

ssize_t evaluate_cmd(int cci_master, const char *buf, size_t n)
{

	char A, X, V;
	uint16_t address = 0xffff, value = 0xffff;
	uint8_t slave_addr = 0xff;
	int rc = 0;

	status = SUCCESS;

	sscanf(buf, "%c%hhx%c%hx%c%hx", &A, &slave_addr, &X, &address, &V,
	       &value);

	if (X == 'W') {
		printk
		    ("CCI%d: WRITE SlaveAddress[%x] on Address[%x] Value[%x]\n",
		     cci_master, slave_addr, address, value);

		if ((address >> 8) == 0xff) {	/*  BYTE ADDRESS */
			if ((value >> 8) == 0) {	/* BYTE DATA */
				rc = write_cci(cci_master, slave_addr, address,
					       value, MSM_CAMERA_I2C_BYTE_ADDR,
					       MSM_CAMERA_I2C_BYTE_DATA);
			} else {	/* WORD DATA */

				rc = write_cci(cci_master, slave_addr, address,
					       value, MSM_CAMERA_I2C_BYTE_ADDR,
					       MSM_CAMERA_I2C_WORD_DATA);
			}

		} else {	/* WORD ADDRESS */

			if ((value >> 8) == 0) {	/* BYTE DATA */
				rc = write_cci(cci_master, slave_addr, address,
					       value, MSM_CAMERA_I2C_WORD_ADDR,
					       MSM_CAMERA_I2C_BYTE_DATA);
			} else {	/* WORD DATA */

				rc = write_cci(cci_master, slave_addr, address,
					       value, MSM_CAMERA_I2C_WORD_ADDR,
					       MSM_CAMERA_I2C_WORD_DATA);

			}

		}
	} else if (X == 'R') {
		printk
		    ("CCI%d: READ SlaveAddress[%x] on Address[%x] Value[%x]\n",
		     cci_master, slave_addr, address, value);

		if ((address >> 8) == 0xff) {	/*  BYTE ADDRESS */
			rc = read_cci(cci_master, slave_addr, address,
					      MSM_CAMERA_I2C_BYTE_ADDR, value);
#if 0
			if (value == 0xff) {	/* BYTE DATA */
				rc = read_cci(cci_master, slave_addr, address,
					      MSM_CAMERA_I2C_BYTE_ADDR, 128);
			} else {	/* WORD DATA */

				rc = read_cci(cci_master, slave_addr, address,
					      MSM_CAMERA_I2C_BYTE_ADDR, 128);
			}
#endif

		} else {	/* WORD ADDRESS */
			rc = read_cci(cci_master, slave_addr, address,
					      MSM_CAMERA_I2C_WORD_ADDR, value);
#if 0
			if (value == 0xff) {	/* BYTE DATA */
				rc = read_cci(cci_master, slave_addr, address,
					      MSM_CAMERA_I2C_WORD_ADDR, 128);
			} else {	/* WORD DATA */

				rc = read_cci(cci_master, slave_addr, address,
					      MSM_CAMERA_I2C_WORD_ADDR, 128);

			}
#endif

		}

	}
	if (rc) {
		pr_err("cci operation failed");
		status = FAILED;
	}

	return n;
}

static ssize_t i2c_func_store_cci0(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   const char *buf, size_t n)
{
	return evaluate_cmd(0, buf, n);

}

static ssize_t i2c_func_store_cci1(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   const char *buf, size_t n)
{
	return evaluate_cmd(1, buf, n);

}

static ssize_t i2c_func_show(struct kobject *kobj,
			     struct kobj_attribute *attr, char *buf)
{
	ssize_t size = 0;
	if (status == SUCCESS) {
		uint8_t i;
		for( i = 0; i < read_size; i++ ) {
			size += sprintf(buf + size, "%02x", read_data[i]);
		}
	} else {
		size = sprintf(buf, "failed");
	}
	return size;
}

static struct kobj_attribute cci1 = {
	.attr = {.name = "i2c_func_cci1",.mode = 0644},
	.store = i2c_func_store_cci1,
	.show = i2c_func_show,
};

static struct kobj_attribute cci0 = {
	.attr = {.name = "i2c_func_cci0",.mode = 0644},
	.store = i2c_func_store_cci0,
	.show = i2c_func_show,
};

static const struct attribute *i2c_func_attrs[] = {
	&cci0.attr,
	&cci1.attr,
	NULL,
};

int add_i2c_func_sysfs_file(struct device *dev)
{
	int ret;
	printk("Add i2c func sysfs file\n");

	ret = sysfs_create_files(&dev->kobj, i2c_func_attrs);
	return ret;
}
