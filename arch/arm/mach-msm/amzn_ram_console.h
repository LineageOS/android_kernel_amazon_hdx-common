/*
 *
 * Copyright (C) 2012 Amazon Incorporated
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __AMZN_RAM_CONSOLE_H
#define __AMZN_RAM_CONSOLE_H

#define AMZN_RAM_CONSOLE_START_DEFAULT (UL(0x50000000))
#define AMZN_RAM_CONSOLE_SIZE_DEFAULT  SZ_2M

#ifdef CONFIG_AMZN_RAM_CONSOLE
extern int amzn_ram_console_init(phys_addr_t phy_addr, size_t size);
#else
static inline int amzn_ram_console_init(phys_addr_t phy_addr, size_t size)
{
       return 0;
}
#endif /* CONFIG_AMZN_RAM_CONSOLE */

#endif


