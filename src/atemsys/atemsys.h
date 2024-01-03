/*-----------------------------------------------------------------------------
 * atemsys.h
 * Copyright (c) 2009 - 2020 acontis technologies GmbH, Ravensburg, Germany
 * All rights reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * Original author           Paul Bussmann
 * Description               atemsys.ko headerfile
 * Note: This header is also included by userspace!
 * 
 * 
 * 
 * Modified by               Omar Abdul-hadi
 *            Forked from atemsys version 1.4.25
 *            Fix for kernel >= 6.6.1:
 *            replace "vma->vm_flags |=" with "vm_flags_set()"
 *            replace class_create(THIS_MODULE, ATEMSYS_DEVICE_NAME) with
 *                    class_create(ATEMSYS_DEVICE_NAME)
 *            commented out pci_enable_pcie_error_reporting() and pci_disable_pcie_error_reporting()
 *            these api's no longer seem to be supported
 *            removed support for legacy versions, device tree, rockChip, STM32mp135, Xeno Cobalt, arm
 *            removed support for interrupt info, irq num already present in ATEMSYS_T_PCI_SELECT_DESC
 *            modified code to always support 64 bit dma
 * 
 *----------------------------------------------------------------------------*/

#ifndef ATEMSYS_H
#define ATEMSYS_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define DRIVER_SUCCESS  0
#define ATEMSYS_DEVICE_NAME "atemsys"
#define ATEMSYS_FILE_NAME "/dev/" ATEMSYS_DEVICE_NAME

#define ATEMSYS_PCI_MAXBAR (6)
#define ATEMSYS_USE_PCI_INT (0xFFFFFFFF) /* Query the selected PCI device for the assigned IRQ number */

typedef struct
{
    __u64       qwIOMem;          /* [out] IO Memory of PCI card (physical address) */
    __u32       dwIOLen;          /* [out] Length of the IO Memory area*/
} __attribute__((packed)) ATEMSYS_T_PCI_MEMBAR;

typedef struct
{
    __s32       nVendID;          /* [in] vendor ID */
    __s32       nDevID;           /* [in] device ID */
    __s32       nInstance;        /* [in] instance to look for (0 is the first instance) */
    __s32       nPciBus;          /* [in/out] bus */
    __s32       nPciDev;          /* [in/out] device */
    __s32       nPciFun;          /* [in/out] function */
    __s32       nBarCnt;          /* [out] Number of entries in aBar */
    __u32       dwIrq;            /* [out] IRQ or USE_PCI_INT */
    ATEMSYS_T_PCI_MEMBAR  aBar[ATEMSYS_PCI_MAXBAR]; /* [out] IO memory */
    __s32       nPciDomain;       /* [in/out] domain */
} __attribute__((packed)) ATEMSYS_T_PCI_SELECT_DESC;


#define MAJOR_NUM 101

#define ATEMSYS_IOCTL_PCI_FIND_DEVICE           _IOWR(MAJOR_NUM,  0, ATEMSYS_T_PCI_SELECT_DESC)
#define ATEMSYS_IOCTL_PCI_CONF_DEVICE           _IOWR(MAJOR_NUM,  1, ATEMSYS_T_PCI_SELECT_DESC)
#define ATEMSYS_IOCTL_PCI_RELEASE_DEVICE        _IO(MAJOR_NUM,    2)
#define ATEMSYS_IOCTL_INT_CONNECT               _IOW(MAJOR_NUM,   3, __u32)
#define ATEMSYS_IOCTL_INT_DISCONNECT            _IOW(MAJOR_NUM,   4, __u32)
#define ATEMSYS_IOCTL_INT_SET_CPU_AFFINITY      _IOWR(MAJOR_NUM, 15, __u32)

#endif  /* ATEMSYS_H */

