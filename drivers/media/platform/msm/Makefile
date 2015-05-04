obj-$(CONFIG_MSM_VIDC_V4L2) += vidc/
obj-$(CONFIG_MSM_WFD) += wfd/
obj-$(CONFIG_DVB_MPQ) += dvb/

ifeq ($(CONFIG_ARCH_MSM8974_THOR),y)
obj-$(CONFIG_MSMB_CAMERA) += camera_v2_amz/
else
ifeq ($(CONFIG_ARCH_MSM8974_APOLLO),y)
obj-$(CONFIG_MSMB_CAMERA) += camera_v2_amz/
else
endif
obj-$(CONFIG_MSMB_CAMERA) += camera_v2/
endif
obj-y += vcap/
