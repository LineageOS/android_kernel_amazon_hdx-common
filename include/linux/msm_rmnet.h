#ifndef _MSM_RMNET_H_
#define _MSM_RMNET_H_

/* Bitmap macros for RmNET driver operation mode. */
#define RMNET_MODE_NONE     (0x00)
#define RMNET_MODE_LLP_ETH  (0x01)
#define RMNET_MODE_LLP_IP   (0x02)
#define RMNET_MODE_QOS      (0x04)
#define RMNET_MODE_MASK     (RMNET_MODE_LLP_ETH | \
			     RMNET_MODE_LLP_IP  | \
			     RMNET_MODE_QOS)

#define RMNET_IS_MODE_QOS(mode)  \
	((mode & RMNET_MODE_QOS) == RMNET_MODE_QOS)
#define RMNET_IS_MODE_IP(mode)   \
	((mode & RMNET_MODE_LLP_IP) == RMNET_MODE_LLP_IP)

/* IOCTL command enum
 * Values chosen to not conflict with other drivers in the ecosystem */
enum rmnet_ioctl_cmds_e {
	RMNET_IOCTL_SET_LLP_ETHERNET = 0x000089F1, /* Set Ethernet protocol  */
	RMNET_IOCTL_SET_LLP_IP       = 0x000089F2, /* Set RAWIP protocol     */
	RMNET_IOCTL_GET_LLP          = 0x000089F3, /* Get link protocol      */
	RMNET_IOCTL_SET_QOS_ENABLE   = 0x000089F4, /* Set QoS header enabled */
	RMNET_IOCTL_SET_QOS_DISABLE  = 0x000089F5, /* Set QoS header disabled*/
	RMNET_IOCTL_GET_QOS          = 0x000089F6, /* Get QoS header state   */
	RMNET_IOCTL_GET_OPMODE       = 0x000089F7, /* Get operation mode     */
	RMNET_IOCTL_OPEN             = 0x000089F8, /* Open transport port    */
	RMNET_IOCTL_CLOSE            = 0x000089F9, /* Close transport port   */
	RMNET_IOCTL_FLOW_ENABLE	     = 0x000089FA, /* Flow enable	     */
	RMNET_IOCTL_FLOW_DISABLE     = 0x000089FB, /* Flow disable	     */
#if defined(CONFIG_ARCH_MSM8974_THOR) || defined(CONFIG_ARCH_MSM8974_APOLLO)
	RMNET_IOCTL_EXTENDED         = 0x000089FD, /* Extended IOCTLs        */
#endif
	RMNET_IOCTL_MAX
};

#if defined(CONFIG_ARCH_MSM8974_THOR) || defined(CONFIG_ARCH_MSM8974_APOLLO)
enum rmnet_ioctl_extended_cmds_e {
	/* OEM */
	RMNET_IOCTL_PM_ENABLE	     = 0x00009000, /* PM runtime enable	     */
	RMNET_IOCTL_PM_DISABLE	     = 0x00009001, /* PM runtime disable     */
	RMNET_IOCTL_PM_WAKE_ENABLE   = 0x00009002, /* Remote wakeup enable   */
	RMNET_IOCTL_PM_WAKE_DISABLE  = 0x00009003, /* Remote wakeup disable  */
	RMNET_IOCTL_EXTENDED_MAX
};

struct rmnet_ioctl_extended_s {
	uint32_t   extended_ioctl;
	union {
		uint32_t data; /* Generic data field for most extended IOCTLs */

		/* Return values for
		 *    RMNET_IOCTL_GET_DRIVER_NAME
		 *    RMNET_IOCTL_GET_DFLT_CONTROL_CHANNEL */
		int8_t if_name[IFNAMSIZ];

		/* Input values for the RMNET_IOCTL_ADD_MUX_CHANNEL IOCTL */
		struct {
			uint32_t  mux_id;
			int8_t    vchannel_name[IFNAMSIZ];
		} rmnet_mux_val;

		/* Input values for the RMNET_IOCTL_FLOW_CONTROL IOCTL */
		struct {
			uint8_t   flow_mode;
			uint8_t   mux_id;
		} flow_control_prop;

		/* Return values for RMNET_IOCTL_GET_EP_PAIR */
		struct {
			uint32_t   consumer_pipe_num;
			uint32_t   producer_pipe_num;
		} ipa_ep_pair;
	} u;
};
#endif

/* QMI QoS header definition */
#define QMI_QOS_HDR_S  __attribute((__packed__)) qmi_qos_hdr_s
struct QMI_QOS_HDR_S {
	unsigned char    version;
	unsigned char    flags;
	unsigned long    flow_id;
};

#endif /* _MSM_RMNET_H_ */
