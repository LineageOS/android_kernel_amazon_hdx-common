ccflags-y += -Idrivers/media/platform/msm/camera_v2_amz
ccflags-y += -Idrivers/media/platform/msm/camera_v2_amz/sensor/io
ccflags-y += -Idrivers/media/platform/msm/camera_v2_amz/sensor/cci
ifneq (,$(filter apollo thor,$(TARGET_PRODUCT)))
obj-$(CONFIG_MSMB_CAMERA) += msm_actuator_8074.o
else
obj-$(CONFIG_MSMB_CAMERA) += msm_actuator.o
endif
obj-$(CONFIG_MSMB_CAMERA) += msm_actuator_debugfs.o
