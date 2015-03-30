/* Copyright (c) 2013, Amazon.com. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ARCH_ARM_MACH_MSM_BOARD_DETECT_H
#define __ARCH_ARM_MACH_MSM_BOARD_DETECT_H

#include <asm/system_info.h>

// Devicetree defines
#ifdef CONFIG_OF
#define early_machine_is_apollo()		\
	of_flat_dt_is_compatible(of_get_flat_dt_root(), "amazon,apollo")
#define early_machine_is_galvajem()		\
	of_flat_dt_is_compatible(of_get_flat_dt_root(), "amazon,galvajem")
#define early_machine_is_thor()		\
	of_flat_dt_is_compatible(of_get_flat_dt_root(), "amazon,thor")
#define early_machine_is_ursa()		\
	of_flat_dt_is_compatible(of_get_flat_dt_root(), "lab126,ursa")
#define machine_is_apollo()		\
	of_machine_is_compatible("amazon,apollo")
#define machine_is_galvajem()		\
	of_machine_is_compatible("amazon,galvajem")
#define machine_is_thor()		\
	of_machine_is_compatible("amazon,thor")
#define machine_is_ursa()		\
	of_machine_is_compatible("lab126,ursa")
#endif

// Thor/Apollo section
static inline bool board_is_thor(void)
{
	return (bool) machine_is_thor();
}

static inline bool board_is_apollo(void)
{
	return (bool) machine_is_apollo();
}

// Ursa section
#define URSA_REVISION_INVALID	0
#define URSA_REVISION_P0	1
#define URSA_REVISION_PRE_P1	3
#define URSA_REVISION_P1	4
#define URSA_REVISION_P0_5	5
#define URSA_REVISION_P2	6
#define URSA_REVISION_PRE_EVT	7
#define URSA_REVISION_EVT	8
#define URSA_REVISION_P0_E	9
#define URSA_REVISION_PRE_DVT	10
#define URSA_REVISION_DVT	11
#define URSA_REVISION_MAX	URSA_REVISION_DVT

static inline bool board_is_ursa(void)
{
	return (bool) machine_is_ursa();
}

static inline unsigned int ursa_board_revision(void)
{
	return socinfo_get_platform_version();
}

// MSM8974 and mixed section
static inline bool board_has_qca6234(void)
{
	if (machine_is_thor() || machine_is_apollo() || machine_is_galvajem())
		return true;

	return false;
}

#endif
