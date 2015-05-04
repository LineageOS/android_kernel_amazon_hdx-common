#include "msm_ispif.h"
#include <linux/debugfs.h>

void  add_debugfs_entry(struct ispif_device* device)
{
	struct dentry * ispif;
	struct dentry *vfe0, *vfe1 ;
        printk("Add ispif debugfs hierarchy \n");

	ispif = debugfs_create_dir("ispif_events", NULL);
	if(ispif)
	{
		vfe0 = debugfs_create_dir("vfe0", ispif);
		if(vfe0)
		{
			debugfs_create_u32("pix0",0444, vfe0, &device->sof_count[VFE0].sof_cnt[PIX0]);
			debugfs_create_u32("pix1", 0444, vfe0, &device->sof_count[VFE0].sof_cnt[PIX1]);
			debugfs_create_u32("rdi0", 0444, vfe0, &device->sof_count[VFE0].sof_cnt[RDI0]);
			debugfs_create_u32("rdi1",0444, vfe0, &device->sof_count[VFE0].sof_cnt[RDI1]);
			debugfs_create_u32("rdi2", 0444, vfe0, &device->sof_count[VFE0].sof_cnt[RDI2]);

		}
		vfe1 = debugfs_create_dir("vfe1", ispif);
		if(vfe1)
		{
			debugfs_create_u32("pix0", 0444, vfe1, &device->sof_count[VFE1].sof_cnt[PIX0]);
			debugfs_create_u32("pix1", 0444, vfe1, &device->sof_count[VFE1].sof_cnt[PIX1]);
			debugfs_create_u32("rdi0", 0444, vfe1, &device->sof_count[VFE1].sof_cnt[RDI0]);
			debugfs_create_u32("rdi1", 0444, vfe1, &device->sof_count[VFE1].sof_cnt[RDI1]);
			debugfs_create_u32("rdi2", 0444, vfe1, &device->sof_count[VFE1].sof_cnt[RDI2]);

		}


	}
}

