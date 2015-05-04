#ifndef __LINUX_MSMB_CAMERA_H
#define __LINUX_MSMB_CAMERA_H

#include <linux/videodev2.h>
#include <linux/types.h>
#include <linux/ioctl.h>

#define MSM_CAM_V4L2_IOCTL_NOTIFY \
	_IOW('V', BASE_VIDIOC_PRIVATE + 30, struct v4l2_event)

#define MSM_CAM_V4L2_IOCTL_NOTIFY_META \
	_IOW('V', BASE_VIDIOC_PRIVATE + 31, struct v4l2_event)

#define MSM_CAM_V4L2_IOCTL_CMD_ACK \
	_IOW('V', BASE_VIDIOC_PRIVATE + 32, struct v4l2_event)

#define MSM_CAM_V4L2_IOCTL_NOTIFY_ERROR \
	_IOW('V', BASE_VIDIOC_PRIVATE + 33, struct v4l2_event)

#define QCAMERA_DEVICE_GROUP_ID	1
#define QCAMERA_VNODE_GROUP_ID	2
#define MSM_CAMERA_NAME					"msm_camera"
#define MSM_CONFIGURATION_NAME	"msm_config"

#define MSM_CAMERA_SUBDEV_CSIPHY       0
#define MSM_CAMERA_SUBDEV_CSID         1
#define MSM_CAMERA_SUBDEV_ISPIF        2
#define MSM_CAMERA_SUBDEV_VFE          3
#define MSM_CAMERA_SUBDEV_AXI          4
#define MSM_CAMERA_SUBDEV_VPE          5
#define MSM_CAMERA_SUBDEV_SENSOR       6
#define MSM_CAMERA_SUBDEV_ACTUATOR     7
#define MSM_CAMERA_SUBDEV_EEPROM       8
#define MSM_CAMERA_SUBDEV_CPP          9
#define MSM_CAMERA_SUBDEV_CCI          10
#define MSM_CAMERA_SUBDEV_LED_FLASH    11
#define MSM_CAMERA_SUBDEV_STROBE_FLASH 12
#define MSM_CAMERA_SUBDEV_BUF_MNGR     13
#define MSM_CAMERA_SUBDEV_OIS          14

#define MSM_MAX_CAMERA_SENSORS  5

/* The below macro is defined to put an upper limit on maximum
 * number of buffer requested per stream. In case of extremely
 * large value for number of buffer due to data structure corruption
 * we return error to avoid integer overflow. This value may be
 * configured in future*/
#define MSM_CAMERA_MAX_STREAM_BUF 40

/* The below macro is defined to put an upper limit on maximum
 * number of buffer requested per stream. In case of extremely
 * large value for number of buffer due to data structure corruption
 * we return error to avoid integer overflow. This value may be
 * configured in future*/
#define MSM_CAMERA_MAX_STREAM_BUF 40

/* featur base */
#define MSM_CAMERA_FEATURE_BASE     0x00010000
#define MSM_CAMERA_FEATURE_SHUTDOWN (MSM_CAMERA_FEATURE_BASE + 1)

#define MSM_CAMERA_STATUS_BASE      0x00020000
#define MSM_CAMERA_STATUS_FAIL      (MSM_CAMERA_STATUS_BASE + 1)
#define MSM_CAMERA_STATUS_SUCCESS   (MSM_CAMERA_STATUS_BASE + 2)

/* event type */
#define MSM_CAMERA_V4L2_EVENT_TYPE (V4L2_EVENT_PRIVATE_START + 0x00002000)

/* event id */
#define MSM_CAMERA_EVENT_MIN    0
#define MSM_CAMERA_NEW_SESSION  (MSM_CAMERA_EVENT_MIN + 1)
#define MSM_CAMERA_DEL_SESSION  (MSM_CAMERA_EVENT_MIN + 2)
#define MSM_CAMERA_SET_PARM     (MSM_CAMERA_EVENT_MIN + 3)
#define MSM_CAMERA_GET_PARM     (MSM_CAMERA_EVENT_MIN + 4)
#define MSM_CAMERA_MAPPING_CFG  (MSM_CAMERA_EVENT_MIN + 5)
#define MSM_CAMERA_MAPPING_SES  (MSM_CAMERA_EVENT_MIN + 6)
#define MSM_CAMERA_MSM_NOTIFY   (MSM_CAMERA_EVENT_MIN + 7)
#define MSM_CAMERA_EVENT_MAX    (MSM_CAMERA_EVENT_MIN + 8)

/* data.command */
#define MSM_CAMERA_PRIV_S_CROP		 (V4L2_CID_PRIVATE_BASE + 1)
#define MSM_CAMERA_PRIV_G_CROP		 (V4L2_CID_PRIVATE_BASE + 2)
#define MSM_CAMERA_PRIV_G_FMT			 (V4L2_CID_PRIVATE_BASE + 3)
#define MSM_CAMERA_PRIV_S_FMT			 (V4L2_CID_PRIVATE_BASE + 4)
#define MSM_CAMERA_PRIV_TRY_FMT		 (V4L2_CID_PRIVATE_BASE + 5)
#define MSM_CAMERA_PRIV_METADATA	 (V4L2_CID_PRIVATE_BASE + 6)
#define MSM_CAMERA_PRIV_QUERY_CAP  (V4L2_CID_PRIVATE_BASE + 7)
#define MSM_CAMERA_PRIV_STREAM_ON  (V4L2_CID_PRIVATE_BASE + 8)
#define MSM_CAMERA_PRIV_STREAM_OFF (V4L2_CID_PRIVATE_BASE + 9)
#define MSM_CAMERA_PRIV_NEW_STREAM (V4L2_CID_PRIVATE_BASE + 10)
#define MSM_CAMERA_PRIV_DEL_STREAM (V4L2_CID_PRIVATE_BASE + 11)
#define MSM_CAMERA_PRIV_SHUTDOWN   (V4L2_CID_PRIVATE_BASE + 12)
#define MSM_CAMERA_PRIV_STREAM_INFO_SYNC \
	(V4L2_CID_PRIVATE_BASE + 13)

#define OV680_PRIV_BASE V4L2_CID_PRIVATE_BASE + 100

/* Lab126 specific structs for IOCTLs */

struct ov680_roi {
       uint32_t  isp_idx;
       uint32_t  sensor_idx;
       uint8_t  zone_weight[4][4];
};

struct ov680_exposure {
        uint32_t sensor_idx;
        uint32_t exposure;
};

struct ov680_gain {
        uint32_t sensor_idx;
        uint32_t gain;
};

#define MSM_SENSOR_METADATA_MAGIC 0xC0DEBABA

struct msm_sensor_metadata {
        uint32_t magic; /* must be the first field always*/
        uint32_t input;
        uint32_t output_format;
        uint16_t  exposure1; /* usec, sensor exposure for whole frame or left subframe (side by side layout) */
        uint16_t  exposure2; /* usec, sensor exposure for right subframe */
        uint16_t  gain1; /* sensor gain for whole frame or left subframe (side by side layout) */
        uint16_t  gain2; /* sensor gain for right subframe */
        uint32_t  frame_rate;
        union {
            uint32_t flags;
            struct {
                uint32_t AEC_on : 1;
                uint32_t AGC_on : 1;
		/* the following bits indicate if  the async IOCTL calls have failed */
                uint32_t exposure_failed : 1;
                uint32_t gain_failed : 1;
                uint32_t flash_duration_failed : 1;
                uint32_t flash_current_failed : 1;
            };
        };
	uint16_t flash_duration; /* usec, duration of flash for all sensors */
	uint16_t flash_current[4]; /* mA, current of flash for each sensor */
};

struct ov680_irled_config {
        uint32_t flash_current[4];
        uint32_t duration;
};

struct ov680_irled_max_current_req {
	/* These will be filled in by the user request */
        uint32_t frame_rate;
        uint32_t duration;
	/* This will be filled in by the driver */
	uint32_t max_current;
};

/* Lab126 specific IOCTLs */

#define BASE_VIDIOC_LAB126 BASE_VIDIOC_PRIVATE + 100

#define VIDIOC_OV680_SENSOR_SET_ROI \
        _IOW('V', BASE_VIDIOC_LAB126, struct ov680_roi)

#define VIDIOC_OV680_SENSOR_SET_EXPOSURE\
        _IOW('V', BASE_VIDIOC_LAB126 + 1, struct ov680_exposure)

#define VIDIOC_OV680_SENSOR_SET_GAIN\
        _IOW('V', BASE_VIDIOC_LAB126 + 2, struct ov680_gain)

#define VIDIOC_OV680_SENSOR_SET_CTRL\
        _IOW('V', BASE_VIDIOC_LAB126 + 3, struct v4l2_control)

#define VIDIOC_SENSOR_GET_METADATA\
        _IOW('V', BASE_VIDIOC_LAB126 + 4, struct msm_sensor_metadata)

#define VIDIOC_OV680_SENSOR_LOAD_FIRMWARE\
        _IO('V', BASE_VIDIOC_LAB126 + 5)

#define VIDIOC_OV680_SENSOR_SET_IRLED_CONFIG\
        _IOW('V', BASE_VIDIOC_LAB126 + 6, struct ov680_irled_config)

#define VIDIOC_OV680_SENSOR_REQUEST_FIRMWARE\
        _IO('V', BASE_VIDIOC_LAB126 + 7)

#define ISPIF_FRAME_EVENT_BASE BASE_VIDIOC_LAB126 + 8

#define VIDIOC_OV680_SENSOR_GET_IRLED_MAX_CURRENT\
        _IO('V', BASE_VIDIOC_LAB126 + 9)


/* Lab126 specific sensor controls */

#define MSM_V4L2_PID_OV680_SENSOR_FRAME_MODE  (OV680_PRIV_BASE+0)
#define MSM_V4L2_PID_OV680_AEC_AGC_MODE       (OV680_PRIV_BASE+1)
#define MSM_V4L2_PID_OV680_AEC_AGC_TARGET     (OV680_PRIV_BASE+2)
#define MSM_V4L2_PID_OV680_FPS                (OV680_PRIV_BASE+3)
#define MSM_V4L2_PID_OV680_SENSOR_ISP_CONFIG  (OV680_PRIV_BASE+4)
#define MSM_V4L2_PID_OV680_SENSOR_LENC_CONFIG (OV680_PRIV_BASE+5)
#define MSM_V4L2_PID_OV680_SENSOR_DPC_CONFIG  (OV680_PRIV_BASE+6)

#define  OV680_PRIV_MAX MSM_V4L2_PID_OV680_SENSOR_DPC_CONFIG

/* data.status - success */
#define MSM_CAMERA_CMD_SUCESS      0x00000001
#define MSM_CAMERA_BUF_MAP_SUCESS  0x00000002

/* data.status - error */
#define MSM_CAMERA_ERR_EVT_BASE 0x00010000
#define MSM_CAMERA_ERR_CMD_FAIL (MSM_CAMERA_ERR_EVT_BASE + 1)
#define MSM_CAMERA_ERR_MAPPING  (MSM_CAMERA_ERR_EVT_BASE + 2)

/* The msm_v4l2_event_data structure should match the
 * v4l2_event.u.data field.
 * should not exceed 16 elements */
struct msm_v4l2_event_data {
	/*word 0*/
	unsigned int command;
	/*word 1*/
	unsigned int status;
	/*word 2*/
	unsigned int session_id;
	/*word 3*/
	unsigned int stream_id;
	/*word 4*/
	unsigned int map_op;
	/*word 5*/
	unsigned int map_buf_idx;
	/*word 6*/
	unsigned int notify;
	/*word 7*/
	unsigned int arg_value;
	/*word 8*/
	unsigned int ret_value;
	/*word 9*/
	unsigned int nop3;
	/*word 10*/
	unsigned int nop4;
	/*word 11*/
	unsigned int nop5;
	/*word 12*/
	unsigned int nop6;
	/*word 13*/
	unsigned int nop7;
	/*word 14*/
	unsigned int nop8;
	/*word 15*/
	unsigned int nop9;
};

/* map to v4l2_format.fmt.raw_data */
struct msm_v4l2_format_data {
	enum v4l2_buf_type type;
	unsigned int width;
	unsigned int height;
	unsigned int pixelformat; /* FOURCC */
	unsigned char num_planes;
	unsigned int plane_sizes[VIDEO_MAX_PLANES];
};

/*  MSM Four-character-code (FOURCC) */
#define msm_v4l2_fourcc(a, b, c, d)\
	((__u32)(a) | ((__u32)(b) << 8) | ((__u32)(c) << 16) |\
	((__u32)(d) << 24))

/* Composite stats */
#define MSM_V4L2_PIX_FMT_STATS_COMB v4l2_fourcc('S', 'T', 'C', 'M')
/* AEC stats */
#define MSM_V4L2_PIX_FMT_STATS_AE   v4l2_fourcc('S', 'T', 'A', 'E')
/* AF stats */
#define MSM_V4L2_PIX_FMT_STATS_AF   v4l2_fourcc('S', 'T', 'A', 'F')
/* AWB stats */
#define MSM_V4L2_PIX_FMT_STATS_AWB  v4l2_fourcc('S', 'T', 'W', 'B')
/* IHIST stats */
#define MSM_V4L2_PIX_FMT_STATS_IHST v4l2_fourcc('I', 'H', 'S', 'T')
/* Column count stats */
#define MSM_V4L2_PIX_FMT_STATS_CS   v4l2_fourcc('S', 'T', 'C', 'S')
/* Row count stats */
#define MSM_V4L2_PIX_FMT_STATS_RS   v4l2_fourcc('S', 'T', 'R', 'S')
/* Bayer Grid stats */
#define MSM_V4L2_PIX_FMT_STATS_BG   v4l2_fourcc('S', 'T', 'B', 'G')
/* Bayer focus stats */
#define MSM_V4L2_PIX_FMT_STATS_BF   v4l2_fourcc('S', 'T', 'B', 'F')
/* Bayer hist stats */
#define MSM_V4L2_PIX_FMT_STATS_BHST v4l2_fourcc('B', 'H', 'S', 'T')

#endif /* __LINUX_MSMB_CAMERA_H */
