ccflags-y += -Idrivers/media/platform/msm/camera_v2
ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor
ccflags-y += -Idrivers/media/platform/msm/camera_v2/msm_vb2
ccflags-y += -Idrivers/media/platform/msm/camera_v2/camera
ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor/io
ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor/cci
ccflags-y += -Idrivers/media/platform/msm/camera_v2/ispif
#ccflags-y += -Iarch/arm/mach-msm
#ccflags-y += -Iarch/arm/mach-msm/include

# Objects required by the driver file
OBJS :=  ov680.o ov680_fw_mgr.o ov680_standby.o ov680_flash.o

obj-$(CONFIG_OV680)   := $(OBJS)
