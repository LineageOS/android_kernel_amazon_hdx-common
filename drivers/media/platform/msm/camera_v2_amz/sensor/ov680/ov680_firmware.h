#ifndef OV680_FIRMWARE_H
#define OV680_FIRMWARE_H

#define FW_STRLEN 50

struct ov680_fw_header{
	char name[FW_STRLEN];
	char date[FW_STRLEN];
	uint16_t num_of_sections;
};

struct ov680_section_header{
	char name[FW_STRLEN];
	uint16_t id;
	uint16_t num_of_sub_sections;
};

struct ov680_subsection_header{
	char name[FW_STRLEN];
	uint16_t size;
};

struct ov680_subsection{
	uint16_t data[3];
};


struct ov680_fw_subsection{
	char name[FW_STRLEN];
	struct msm_camera_i2c_reg_conf *conf;
	uint16_t size;
	uint8_t enabled;
};

struct ov680_fw_section{
	char name[FW_STRLEN];
	struct ov680_fw_subsection* subsections;
        uint32_t number_of_subsections;
	uint8_t enabled;
};

#endif /* OV680_FIRMWARE_H */
