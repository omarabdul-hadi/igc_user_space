/*-----------------------------------------------------------------------------
 * atemsys.c
 * Copyright (c) 2009 - 2020 acontis technologies GmbH, Ravensburg, Germany
 * All rights reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * Response                  Paul Bussmann
 * Description               Provides usermode access to:
 *   - PCI configuration space
 *   - Device IO memory
 *   - Contiguous DMA memory
 *   - Single device interrupt
 *
 *
 * The driver should be used in the following way:
 *
 * - Make sure this driver's device node is present. I.e. call "mknod /dev/atemsys c 101 0"
 *
 * - open()
 *   Open driver (There can be more then one file descriptor active in parallel).
 *
 * - close()
 *   Close driver. Free resources, if any were allocated.
 *
 * - ioctl(ATEMSYS_IOCTL_PCI_FIND_DEVICE)
 *   Scan for PCI Devices.
 *   Input:  VendorID, DeviceID, InstanceNo
 *   Output: BusNo, DevNo, FuncNo
 *
 * - ioctl(ATEMSYS_IOCTL_PCI_CONF_DEVICE)
 *   Configures PCI device. This ioctl pins the given PCI device to the current filedescriptor.
 *   Input:  BusNo, DevNo, FuncNo
 *   Output: Physical IO base address, IO area length, IRQ number
 *   The device must be released explicitly in order to configure the next device. The ioctl gets
 *   errno EBUSY if the device is in use by another device driver.
 *
 * - ioctl(ATEMSYS_IOCTL_PCI_RELEASE_DEVICE)
 *   Release PCI device and free resources assigned to PCI device (interrupt, DMA memory, ...).
 *
 * - mmap(0, dwSize, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_LOCKED, fd, 0);
 *   Allocates and maps DMA memory of size dwSize. Note that the last parameter (offset) must be 0.
 *   Input:  Length in byte
 *   Output: Pointer to the allocated memory and DMA physical address. On success this address is
 *           written into the first 8 bytes of the allocated memory.
 *
 * - mmap(0, IOphysSize, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_LOCKED, fd, IOphysAddr);
 *   Maps IO memory of size IOphysSize.
 *   PCI device:
 *     First call ioctl(ATEMSYS_IOCTL_PCI_CONF_DEVICE). The IOphysAddr and IOphysSize
 *     parameter must corespond with the base IO address and size returned by
 *     ioctl(ATEMSYS_IOCTL_PCI_CONF_DEVICE), or the ioctl will fail.
 *   Non-PCI device:
 *     Don't call ioctl(ATEMSYS_IOCTL_PCI_CONF_DEVICE) before and just pass
 *     IOphysAddr and IOphysSize. There are no checks done.
 *   Input:  Phys.IO base address, IO area length in byte
 *   Output: Pointer to the mapped IO memory.
 *   The user should call dev_munmap() if the requested DMA memory is not needed anymore. In any cases
 *   the allocated / mapped memory is released / unmapped if the module is unloaded.
 *
 * - ioctl(ATEMSYS_IOCTL_INT_CONNECT)
 *   Connect an ISR to the device's interrupt.
 *   If the parameter is USE_PCI_INT, then the IRQ is taken from the selected PCI device.
 *   So in this case you have to call ioctl(ATEMSYS_IOCTL_PCI_CONF_DEVICE) first, or it will fail.
 *   Input:  IRQ-Number or USE_PCI_INT
 *   Output: none
 *   The device interrupt is active if this ioctl succeeds. The caller should do a read() on the file
 *   descriptor. The read call unblocks if an interrupt is received. If the read is unblocked, the
 *   interrupt is disabled on the (A)PIC and the caller must acknowledge the interrupt on the device
 *   (write to mmaped IO register). If the next read() is executed, the interrupt is enabled again
 *   on the (A)PIC. So a missing interrupt acknowledge will hold the INT line active and interrupt
 *   trashing will happen (ISR is called again, read() unblocks, ...).
 *   Note that this ioctl will fail with errno EPERM if the interrupt line is shared.
 *   PCI device:
 *     The ioctl will try to use Message Signaled Interrupts (MSI) if supported
 *     by the PCI device. By definition, interrupts are never shared with MSI and MSI are mandatory
 *     for PCI-Express :).
 *
 * - ioctl(ATEMSYS_IOCTL_INT_DISCONNECT)
 *   Disconnect from device's interrupt.
 *
 * - read()
 *   see ioctl(ATEMSYS_IOCTL_INT_CONNECT)
 *
 *
 *  Changes see atemsys.h
 *
 *----------------------------------------------------------------------------*/

#include <linux/module.h>
#include "atemsys.h"
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/smp.h>
#include <linux/sched/signal.h>
#include <linux/irq.h>
#include <linux/list.h>
#include <linux/uaccess.h>
#include <asm/current.h>
#include <linux/compat.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/dma-direct.h>
#include <linux/aer.h>


MODULE_AUTHOR("acontis technologies GmbH <info@acontis.com>");
MODULE_DESCRIPTION("Generic usermode PCI driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");

static char* AllowedPciDevices = "PCI_ANY_ID";
module_param(AllowedPciDevices, charp, 0000);
MODULE_PARM_DESC(AllowedPciDevices, "Bind only pci devices in semicolon separated list e.g. AllowedPciDevices=\"0000:01:00.0\", empty string will turn off atemsys_pci driver.");

/* Workaround for older kernels */
/* from 'linux/kern_levels.h' */
/* integer equivalents of KERN_<LEVEL> */
#ifndef LOGLEVEL_ERR
#define LOGLEVEL_ERR        3   /* error conditions */
#endif
#ifndef LOGLEVEL_WARNING
#define LOGLEVEL_WARNING    4   /* warning conditions */
#endif
#ifndef LOGLEVEL_INFO
#define LOGLEVEL_INFO       6   /* informational */
#endif
#ifndef LOGLEVEL_DEBUG
#define LOGLEVEL_DEBUG      7   /* debug-level messages */
#endif

static int loglevel = LOGLEVEL_INFO;
module_param(loglevel, int, 0);
MODULE_PARM_DESC(loglevel, "Set log level default LOGLEVEL_INFO, see /include/linux/kern_levels.h");


#define PRINTK(prio, str, ...) printk(prio ATEMSYS_DEVICE_NAME ": " str,  ##__VA_ARGS__)

#define ERR(str, ...) (LOGLEVEL_ERR <= loglevel)?     PRINTK(KERN_ERR, str, ##__VA_ARGS__)     :0
#define WRN(str, ...) (LOGLEVEL_WARNING <= loglevel)? PRINTK(KERN_WARNING, str, ##__VA_ARGS__) :0
#define INF(str, ...) (LOGLEVEL_INFO <= loglevel)?    PRINTK(KERN_INFO, str, ##__VA_ARGS__)    :0
#define DBG(str, ...) (LOGLEVEL_DEBUG <= loglevel)?   PRINTK(KERN_INFO, str, ##__VA_ARGS__)    :0


#ifndef PAGE_UP
#define PAGE_UP(addr)   (((addr)+((PAGE_SIZE)-1))&(~((PAGE_SIZE)-1)))
#endif
#ifndef PAGE_DOWN
#define PAGE_DOWN(addr) ((addr)&(~((PAGE_SIZE)-1)))
#endif

#define ACCESS_OK(type, addr, size)     access_ok(addr, size)

#define OF_DMA_CONFIGURE(dev, of_node)

typedef struct _ATEMSYS_T_IRQ_DESC
{
    u32               irq;
    atomic_t          count;
    atomic_t          totalCount;
    atomic_t          irqStatus;
    wait_queue_head_t q;
    bool              irq_is_level;
} ATEMSYS_T_IRQ_DESC;

struct _ATEMSYS_T_PCI_DRV_DESC_PRIVATE;
struct _ATEMSYS_T_DRV_DESC_PRIVATE;
typedef struct _ATEMSYS_T_DEVICE_DESC
{
    struct list_head list;
    struct pci_dev* pPcidev;
    struct _ATEMSYS_T_PCI_DRV_DESC_PRIVATE* pPciDrvDesc;
    struct platform_device* pPlatformDev;

    ATEMSYS_T_IRQ_DESC  irqDesc;
} ATEMSYS_T_DEVICE_DESC;

typedef struct _ATEMSYS_T_MMAP_DESC
{
   struct list_head  list;
   ATEMSYS_T_DEVICE_DESC* pDevDesc;
   dma_addr_t        dmaAddr;
   void*             pVirtAddr;
   size_t            len;
} ATEMSYS_T_MMAP_DESC;



#define ATEMSYS_MAX_NUMBER_DRV_INSTANCES 10

typedef struct _ATEMSYS_T_PCI_DRV_DESC_PRIVATE
{
    struct pci_dev*             pPciDev;

    int                         nPciDomain;
    int                         nPciBus;
    int                         nPciDev;
    int                         nPciFun;

    unsigned short              wVendorId;
    unsigned short              wDevice;
    unsigned short              wRevision;
    unsigned short              wSubsystem_vendor;
    unsigned short              wSubsystem_device;

    ATEMSYS_T_PCI_MEMBAR        aBars[ATEMSYS_PCI_MAXBAR];
    int                         nBarCnt;

    ATEMSYS_T_DEVICE_DESC*      pDevDesc;
    unsigned int                dwIndex;
} ATEMSYS_T_PCI_DRV_DESC_PRIVATE;

static ATEMSYS_T_PCI_DRV_DESC_PRIVATE*  S_apPciDrvDescPrivate[ATEMSYS_MAX_NUMBER_DRV_INSTANCES];

static void dev_munmap(struct vm_area_struct* vma);
static irqreturn_t dev_interrupt_handler(int nIrq, void* pParam);

static struct vm_operations_struct mmap_vmop =
{
   .close = dev_munmap,
};

static DEFINE_MUTEX(S_mtx);
static ATEMSYS_T_DEVICE_DESC S_DevNode;
static struct class* S_pDevClass;
static struct device* S_pDev;
static struct platform_device* S_pPlatformDev = NULL;

static void dev_enable_irq(ATEMSYS_T_IRQ_DESC* pIrqDesc)
{
    /* enable/disable level type interrupts, not edge type interrupts */
    if (pIrqDesc->irq_is_level)
    {
        atomic_inc(&pIrqDesc->irqStatus);
        enable_irq(pIrqDesc->irq);
    }
}

static void dev_disable_irq(ATEMSYS_T_IRQ_DESC* pIrqDesc)
{
    /* enable/disable level type interrupts, not edge type interrupts */
    if (!pIrqDesc->irq_is_level) return;

    if (atomic_read(&pIrqDesc->irqStatus) > 0)
    {
        disable_irq_nosync(pIrqDesc->irq);
        atomic_dec(&pIrqDesc->irqStatus);
    }
}

static int dev_irq_disabled(ATEMSYS_T_IRQ_DESC* pIrqDesc)
{
    /* only level type interrupts get disabled */
    if (!pIrqDesc->irq_is_level) return 0;

    if (atomic_read(&pIrqDesc->irqStatus) == 0)
    {
        return 1;
    }
    return 0;
}

static void* dev_dma_alloc(u32 dwLen, dma_addr_t* pDmaAddr)
{
   unsigned long virtAddr;
   unsigned long tmpAddr;
   u32 tmpSize;

   virtAddr =  __get_free_pages(GFP_KERNEL | GFP_DMA, get_order(dwLen));
   if (! virtAddr)
   {
      ERR("mmap: __get_free_pages failed\n");
      return NULL;
   }

   tmpAddr = virtAddr;
   tmpSize = dwLen;

   while (tmpSize > 0)
   {
     SetPageReserved( virt_to_page(tmpAddr) );
     tmpAddr += PAGE_SIZE;
     tmpSize -= PAGE_SIZE;
   }

   *pDmaAddr = virt_to_phys((void*) virtAddr);

   return (void*) virtAddr;
}

static void dev_dma_free(u32 dwLen, void* virtAddr)
{
   unsigned long tmpAddr = (unsigned long) virtAddr;
   u32 tmpSize = dwLen;

   while (tmpSize > 0)
   {
     ClearPageReserved( virt_to_page(tmpAddr) );
     tmpAddr += PAGE_SIZE;
     tmpSize -= PAGE_SIZE;
   }

   free_pages((unsigned long) virtAddr, get_order(dwLen));
}

static void dev_munmap(struct vm_area_struct* vma)
{
   ATEMSYS_T_MMAP_DESC* pMmapDesc = (ATEMSYS_T_MMAP_DESC*) vma->vm_private_data;

   INF("dev_munmap: 0x%px -> 0x%px (%d)\n",
         (void*) pMmapDesc->pVirtAddr, (void*)(unsigned long)pMmapDesc->dmaAddr, (int) pMmapDesc->len);
    if (0 == pMmapDesc->dmaAddr) { INF("dev_munmap: 0 == pMmapDesc->dmaAddr!\n"); return; }
    if (NULL == pMmapDesc->pVirtAddr) { INF("dev_munmap: NULL == pMmapDesc->pVirtAddr!\n"); return; }

   /* free DMA memory */
   if (pMmapDesc->pDevDesc->pPcidev == NULL)
   {
      dev_dma_free(pMmapDesc->len, pMmapDesc->pVirtAddr);
   }
   else
   {
      dma_free_coherent(&pMmapDesc->pDevDesc->pPcidev->dev, pMmapDesc->len, pMmapDesc->pVirtAddr, pMmapDesc->dmaAddr);
   }
   kfree(pMmapDesc);
}
/*
 * Lookup PCI device
 */

static int dev_pci_select_device(ATEMSYS_T_DEVICE_DESC* pDevDesc, ATEMSYS_T_PCI_SELECT_DESC* pPciDesc, size_t size)
{
    int nRetVal = -EFAULT;
    s32 nPciBus, nPciDev, nPciFun, nPciDomain;

    if (size == sizeof(ATEMSYS_T_PCI_SELECT_DESC) )
    {
        ATEMSYS_T_PCI_SELECT_DESC oPciDesc;
        nRetVal = copy_from_user(&oPciDesc, (ATEMSYS_T_PCI_SELECT_DESC*)pPciDesc, sizeof(ATEMSYS_T_PCI_SELECT_DESC));
        if (0 != nRetVal)
        {
            ERR("dev_pci_select_device failed: %d\n", nRetVal);
            goto Exit;
        }
        nPciBus    = oPciDesc.nPciBus;
        nPciDev    = oPciDesc.nPciDev;
        nPciFun    = oPciDesc.nPciFun;
        nPciDomain = oPciDesc.nPciDomain;
    }
    else
    {
        nRetVal = -EFAULT;
        ERR("pci_conf: EFAULT\n");
        goto Exit;
    }

    /* Lookup for pci_dev object */
    pDevDesc->pPcidev       = NULL;
    pDevDesc->pPciDrvDesc   = NULL;
    {
        unsigned int i = 0;

        for (i = 0; i < ATEMSYS_MAX_NUMBER_DRV_INSTANCES; i++)
        {
            ATEMSYS_T_PCI_DRV_DESC_PRIVATE* pDrvInstance = S_apPciDrvDescPrivate[i];
            if (   (pDrvInstance                != NULL)
                && (pDrvInstance->nPciDomain    == nPciDomain)
                && (pDrvInstance->nPciBus       == nPciBus)
                && (pDrvInstance->nPciDev       == nPciDev)
                && (pDrvInstance->nPciFun       == nPciFun))
            {
                if (pDrvInstance->pDevDesc != NULL)
                {
                    ERR("dev_pci_select_device: device \"%s\" in use by another instance?\n", pci_name(pDrvInstance->pPciDev));
                    nRetVal = -EBUSY;
                    goto Exit;
                }
                pDevDesc->pPcidev        = pDrvInstance->pPciDev;
                pDevDesc->pPciDrvDesc    = pDrvInstance;
                pDrvInstance->pDevDesc   = pDevDesc;
                INF("pci_select: from pci driver %04x:%02x:%02x.%x\n", (u32)nPciDomain, (u32)nPciBus, (u32)nPciDev, (u32)nPciFun);
                break;
            }
        }
    }
    if (pDevDesc->pPcidev == NULL)
    {
        pDevDesc->pPcidev = pci_get_domain_bus_and_slot(nPciDomain, nPciBus, PCI_DEVFN(nPciDev, nPciFun));
        INF("pci_select: %04x:%02x:%02x.%x\n", (u32)nPciDomain, (u32)nPciBus, (u32)nPciDev, (u32)nPciFun);
    }
    if (pDevDesc->pPcidev == NULL)
    {
        WRN("pci_select: PCI-Device  %04x:%02x:%02x.%x not found\n",
            (unsigned) nPciDomain, (unsigned) nPciBus, (unsigned) nPciDev, (unsigned) nPciFun);
        goto Exit;
    }

    nRetVal = DRIVER_SUCCESS;

Exit:
    return nRetVal;
}

static int DefaultPciSettings(struct pci_dev* pPciDev)
{
    int nRetVal = -EIO;
    int nRes = -EIO;

    /* Turn on Memory-Write-Invalidate if it is supported by the device*/
    pci_try_set_mwi(pPciDev);

    nRes = dma_set_mask_and_coherent(&pPciDev->dev, DMA_BIT_MASK(32));
    if (nRes)
    {
        nRes = dma_set_mask_and_coherent(&pPciDev->dev, DMA_BIT_MASK(64));
        if (nRes)
        {
            ERR("%s: DefaultPciSettings: dma_set_mask_and_coherent failed\n", pci_name(pPciDev));
            nRetVal = nRes;
            goto Exit;
        }
    }
    pci_set_master(pPciDev);

    /* Try to enable MSI (Message Signaled Interrupts). MSI's are non shared, so we can
    * use interrupt mode, also if we have a non exclusive interrupt line with legacy
    * interrupts.
    */
    if (pci_enable_msi(pPciDev))
    {
        INF("%s: DefaultPciSettings: legacy INT configured\n", pci_name(pPciDev));
    }
    else
    {
        INF("%s: DefaultPciSettings: MSI configured\n", pci_name(pPciDev));
    }

    nRetVal = 0;

Exit:
   return nRetVal;
}

/*
 * See also kernel/Documentation/PCI/pci.txt for the recommended PCI initialization sequence
 */
static int ioctl_pci_configure_device(ATEMSYS_T_DEVICE_DESC* pDevDesc, unsigned long ioctlParam, size_t size)
{
    int nRetVal = -EIO;
    int nRc;
    int i;
    unsigned long ioBase;
    s32 nBar = 0;
    ATEMSYS_T_PCI_SELECT_DESC oPciDesc;
    memset(&oPciDesc, 0, sizeof(ATEMSYS_T_PCI_SELECT_DESC));
    if (size != sizeof(ATEMSYS_T_PCI_SELECT_DESC) )
    {
        nRetVal = -EIO;
        ERR("pci_conf: Invalid parameter\n");
        goto Exit;
    }

    if (pDevDesc->pPcidev != NULL)
    {
        WRN("pci_conf: error call ioctl(ATEMSYS_IOCTL_PCI_RELEASE_DEVICE) first\n");
        goto Exit;
    }
    if (dev_pci_select_device(pDevDesc, (ATEMSYS_T_PCI_SELECT_DESC*)ioctlParam, size) != DRIVER_SUCCESS)
    {
        goto Exit;
    }

    if (NULL != pDevDesc->pPciDrvDesc)
    {
        for (i = 0; i < pDevDesc->pPciDrvDesc->nBarCnt ; i++)
        {
            oPciDesc.aBar[i].qwIOMem = pDevDesc->pPciDrvDesc->aBars[i].qwIOMem;
            oPciDesc.aBar[i].dwIOLen = pDevDesc->pPciDrvDesc->aBars[i].dwIOLen;
        }

        oPciDesc.nBarCnt = pDevDesc->pPciDrvDesc->nBarCnt;
        oPciDesc.dwIrq   = (u32)pDevDesc->pPcidev->irq;
    }
    else
    {
        /* enable device */
        nRc = pci_enable_device(pDevDesc->pPcidev);
        if (nRc < 0)
        {
            ERR("pci_conf: pci_enable_device failed\n");
            pDevDesc->pPcidev = NULL;
            goto Exit;
        }

        /* Check if IO-memory is in use by another driver */
        nRc = pci_request_regions(pDevDesc->pPcidev, ATEMSYS_DEVICE_NAME);
        if (nRc < 0)
        {
            ERR("pci_conf: device \"%s\" in use by another driver?\n", pci_name(pDevDesc->pPcidev));
            pDevDesc->pPcidev = NULL;
            nRetVal = -EBUSY;
            goto Exit;
        }

        /* find the memory BAR */
        for (i = 0; i < ATEMSYS_PCI_MAXBAR ; i++)
        {
            if (pci_resource_flags(pDevDesc->pPcidev, i) & IORESOURCE_MEM)
            {
                /* IO area address */
                ioBase = pci_resource_start(pDevDesc->pPcidev, i);

                /* IO area length */
                oPciDesc.aBar[nBar].dwIOLen = pci_resource_len(pDevDesc->pPcidev, i);
                oPciDesc.aBar[nBar].qwIOMem = ioBase;

                nBar++;
            }
        }

        nRc = DefaultPciSettings(pDevDesc->pPcidev);
        if (nRc)
        {
            pci_release_regions(pDevDesc->pPcidev);
            pDevDesc->pPcidev = NULL;
            goto Exit;
        }

        /* number of memory BARs */
        /* assigned IRQ */
        oPciDesc.nBarCnt = nBar;
        oPciDesc.dwIrq   = pDevDesc->pPcidev->irq;
    }

    if (!ACCESS_OK(VERIFY_WRITE, (ATEMSYS_T_PCI_SELECT_DESC*)ioctlParam, sizeof(ATEMSYS_T_PCI_SELECT_DESC)))
    {
        nRetVal = -EFAULT;
        ERR("pci_conf: EFAULT\n");
        goto Exit;
    }
    nRetVal = copy_to_user((ATEMSYS_T_PCI_SELECT_DESC*)ioctlParam, &oPciDesc, sizeof(ATEMSYS_T_PCI_SELECT_DESC));
    if (0 != nRetVal)
    {
        ERR("ioctl_pci_configure_device failed: %d\n", nRetVal);
        goto Exit;
    }

   nRetVal = 0;

Exit:
   return nRetVal;
}

static int ioctl_pci_finddevice(ATEMSYS_T_DEVICE_DESC* pDevDesc, unsigned long ioctlParam, size_t size)
{
    int nRetVal = -EIO;
    struct pci_dev* pPciDev = NULL;
    s32 nVendor, nDevice, nInstance, nInstanceId;
    ATEMSYS_T_PCI_SELECT_DESC oPciDesc;

    if (size != sizeof(ATEMSYS_T_PCI_SELECT_DESC) )
        {
        nRetVal = -EIO;
        ERR("pci_conf: Invalid parameter\n");
        goto Exit;
    }

    memset(&oPciDesc, 0, sizeof(ATEMSYS_T_PCI_SELECT_DESC));
    if (!ACCESS_OK(VERIFY_WRITE, (ATEMSYS_T_PCI_SELECT_DESC*)ioctlParam, sizeof(ATEMSYS_T_PCI_SELECT_DESC)))
    {
        nRetVal = -EFAULT;
    }
    nRetVal = copy_from_user(&oPciDesc, (ATEMSYS_T_PCI_SELECT_DESC*)ioctlParam, sizeof(ATEMSYS_T_PCI_SELECT_DESC));
    if (0 != nRetVal)
    {
        ERR("ioctl_pci_finddevice failed: %d\n", nRetVal);
        goto Exit;
    }
    nVendor   = oPciDesc.nVendID;
    nDevice   = oPciDesc.nDevID;
    nInstance = oPciDesc.nInstance;

    if (-EFAULT == nRetVal)
    {
        ERR("pci_find: EFAULT\n");
        nRetVal = -EFAULT;
        goto Exit;
    }

    INF("pci_find: ven 0x%x dev 0x%x nInstance %d\n", nVendor, nDevice, nInstance);

    for (nInstanceId = 0; nInstanceId <= nInstance; nInstanceId++ )
    {
        pPciDev = pci_get_device (nVendor, nDevice, pPciDev);
    }

    if (pPciDev == NULL)
    {
        WRN("pci_find: device 0x%x:0x%x:%d not found\n", nVendor, nDevice, nInstance);
        nRetVal = -ENODEV;
        goto Exit;
    }

    INF("pci_find: found 0x%x:0x%x:%d -> %s\n",
       nVendor, nDevice, nInstance, pci_name(pPciDev));

    oPciDesc.nPciDomain = (s32)pci_domain_nr(pPciDev->bus);
    oPciDesc.nPciBus    = (s32)pPciDev->bus->number;
    oPciDesc.nPciDev    = (s32)PCI_SLOT(pPciDev->devfn);
    oPciDesc.nPciFun    = (s32)PCI_FUNC(pPciDev->devfn);

    nRetVal = copy_to_user((ATEMSYS_T_PCI_SELECT_DESC*)ioctlParam, &oPciDesc, sizeof(ATEMSYS_T_PCI_SELECT_DESC));
    if (0 != nRetVal)
    {
        ERR("ioctl_pci_finddevice failed: %d\n", nRetVal);
        goto Exit;
    }

    nRetVal = 0;

Exit:
    return nRetVal;
}

static bool atemsys_irq_is_level(unsigned int irq_id)
{
     bool irq_is_level = true;
     struct irq_data* irq_data = NULL;

    irq_data = irq_get_irq_data(irq_id);

    if (irq_data)
    {
        irq_is_level = irqd_is_level_type(irq_data);
    }

    return irq_is_level;
}

static int ioctl_int_connect(ATEMSYS_T_DEVICE_DESC* pDevDesc, unsigned long ioctlParam)
{
    int nRetVal = -EIO;
    int nRc;
    ATEMSYS_T_IRQ_DESC* pIrqDesc = NULL;
    unsigned int irq = 0;

    if (ioctlParam == ATEMSYS_USE_PCI_INT)
    {
        /* Use IRQ number from selected PCI device */

        if (pDevDesc->pPcidev == NULL)
        {
            WRN("intcon: error call ioctl(ATEMSYS_IOCTL_PCI_CONF_DEVICE) first\n");
            goto Exit;
        }

        irq = pDevDesc->pPcidev->irq;
        INF("intcon: Use IRQ (%d) from PCI config\n", irq);
    }
    else
    /* Use IRQ number passed as ioctl argument */
    irq = ioctlParam;
    INF("intcon: Use IRQ (%d) passed by user\n", irq);

    pIrqDesc = &pDevDesc->irqDesc;
    if (pIrqDesc->irq)
    {
        WRN("intcon: error IRQ %u already connected. Call ioctl(ATEMSYS_IOCTL_INT_DISCONNECT) first\n",
            (unsigned) pIrqDesc->irq);
        goto Exit;
    }

    /* Setup some data which is needed during Interrupt handling */
    memset(pIrqDesc, 0, sizeof(ATEMSYS_T_IRQ_DESC));
    atomic_set(&pIrqDesc->count, 0);
    atomic_set(&pIrqDesc->totalCount, 0);

    init_waitqueue_head(&pIrqDesc->q);
    atomic_set(&pIrqDesc->irqStatus, 1); /* IRQ enabled */

    /* Setup non shared IRQ */
    nRc = request_irq(irq, dev_interrupt_handler, 0, ATEMSYS_DEVICE_NAME, pDevDesc);
    if (nRc)
    {
        ERR("ioctl_int_connect: request_irq (IRQ %d) failed. Err %d\n", irq, nRc);
        nRetVal = -EPERM;
        goto Exit;
    }

    pIrqDesc->irq = irq;
    pIrqDesc->irq_is_level = atemsys_irq_is_level(irq);

    INF("intcon: IRQ %d connected, irq_is_level = %d\n", irq, pIrqDesc->irq_is_level);

    nRetVal = 0;
Exit:
    return nRetVal;
}

static int dev_int_disconnect(ATEMSYS_T_DEVICE_DESC* pDevDesc)
{
   int nCnt;
   ATEMSYS_T_IRQ_DESC* pIrqDesc = &(pDevDesc->irqDesc);

    if (pIrqDesc->irq)
    {
        /* Disable INT line. We can call this, because we only allow exclusive interrupts */
        disable_irq_nosync(pIrqDesc->irq);

        /* Unregister INT routine.This will block until all pending interrupts are handled */
        free_irq(pIrqDesc->irq, pDevDesc);

        nCnt = atomic_read(&pIrqDesc->totalCount);
        INF("pci_intdcon: IRQ %u disconnected. %d interrupts rcvd\n", (u32) pIrqDesc->irq, nCnt);

        pIrqDesc->irq = 0;

        /* Wakeup sleeping threads -> read() */
        wake_up(&pIrqDesc->q);
    }

   return 0;
}

static int SetIntCpuAffinityIoctl(ATEMSYS_T_DEVICE_DESC* pDevDesc, unsigned long ioctlParam, size_t size)
{
    int nRetVal = -EIO;
    ATEMSYS_T_IRQ_DESC* pIrqDesc = &(pDevDesc->irqDesc);
    struct cpumask* pCpuMask = 0;

    if (size > sizeof(struct cpumask))
    {
        ERR("SetIntCpuAffinityIoctl: cpu mask length mismatch\n");
        nRetVal = -EINVAL;
        goto Exit;
    }

    /* prepare cpu affinity mask*/
    pCpuMask = (struct cpumask*)kzalloc(sizeof(struct cpumask), GFP_KERNEL);
    if (NULL == pCpuMask)
    {
        ERR("SetIntCpuAffinityIoctl: no memory\n");
        nRetVal = -ENOMEM;
        goto Exit;
    }
    memset(pCpuMask, 0, sizeof(struct cpumask)>size? sizeof(struct cpumask): size);

    nRetVal = copy_from_user(pCpuMask, (struct cpumask *)ioctlParam, size);
    if (0 != nRetVal)
    {
        ERR("SetIntCpuAffinityIoctl failed: %d\n", nRetVal);
        goto Exit;
    }

    /* set cpu affinity mask*/
    if (pIrqDesc->irq)
    {
        nRetVal = irq_set_affinity(pIrqDesc->irq, pCpuMask);
        if (0 != nRetVal)
        {
            ERR("SetIntCpuAffinityIoctl: irq_set_affinity failed: %d\n", nRetVal);
            nRetVal = -EIO;
            goto Exit;
        }
    }

    nRetVal = 0;
Exit:
    if (NULL != pCpuMask)
        kfree(pCpuMask);

    return nRetVal;
}

static void dev_pci_release(ATEMSYS_T_DEVICE_DESC* pDevDesc)
{
    if (NULL != pDevDesc->pPciDrvDesc)
    {
        INF("pci_release: Disconnect from PCI device driver %s \n", pci_name(pDevDesc->pPcidev));
        pDevDesc->pPciDrvDesc->pDevDesc = NULL;
        pDevDesc->pPcidev               = NULL;
        pDevDesc->pPciDrvDesc           = NULL;
    }
    else

   if (pDevDesc->pPcidev)
   {
      pci_disable_device(pDevDesc->pPcidev);

      /* Make sure bus master DMA is disabled if the DMA buffers are finally released */
      pci_clear_master(pDevDesc->pPcidev);
      pci_release_regions(pDevDesc->pPcidev);

      pci_disable_msi(pDevDesc->pPcidev);

      INF("pci_release: PCI device %s released\n", pci_name(pDevDesc->pPcidev));

      pDevDesc->pPcidev = NULL;
   }
}

static irqreturn_t dev_interrupt_handler(int nIrq, void* pParam)
{
   ATEMSYS_T_DEVICE_DESC* pDevDesc = (ATEMSYS_T_DEVICE_DESC*) pParam;
   ATEMSYS_T_IRQ_DESC* pIrqDesc = &(pDevDesc->irqDesc);

   /* Disable IRQ on (A)PIC to prevent interrupt trashing if the ISR is left.
    * In usermode the IRQ must be acknowledged on the device (IO register).
    * The IRQ is enabled again in the read() handler!
    * Just disabling the IRQ here doesn't work with shared IRQs!
    */
   dev_disable_irq(pIrqDesc);

   atomic_inc(&pIrqDesc->count);
   atomic_inc(&pIrqDesc->totalCount);

   /* Wakeup sleeping threads -> read() */
   wake_up(&pIrqDesc->q);

   return IRQ_HANDLED;
}

/*
 * This is called whenever a process attempts to open the device file
 */
static int device_open(struct inode* inode, struct file* file)
{
   ATEMSYS_T_DEVICE_DESC* pDevDesc;

   INF("device_open(0x%px)\n", file);

   /* create device descriptor */
   pDevDesc = (ATEMSYS_T_DEVICE_DESC*) kzalloc(sizeof(ATEMSYS_T_DEVICE_DESC), GFP_KERNEL);
   if (pDevDesc == NULL)
   {
      return -ENOMEM;
   }

   file->private_data = (void*) pDevDesc;

   /* Add descriptor to descriptor list */
   mutex_lock(&S_mtx);
   list_add(&pDevDesc->list, &S_DevNode.list);
   mutex_unlock(&S_mtx);
   try_module_get(THIS_MODULE);

   /* use module's platform device for memory maping and allocation */
   pDevDesc->pPlatformDev = S_pPlatformDev;

   return DRIVER_SUCCESS;
}

static int device_release(struct inode* inode, struct file* file)
{
   ATEMSYS_T_DEVICE_DESC* pDevDesc = file->private_data;

   /* release device descriptor */
   if (pDevDesc != NULL )
   {
       INF("device_release, pDevDesc = 0x%px\n", pDevDesc);

       /* Try to tear down interrupts if they are on */
       dev_int_disconnect(pDevDesc);

       /* Try to release PCI resources */
       dev_pci_release(pDevDesc);

       /* Remove descriptor from descriptor list */
       mutex_lock(&S_mtx);

       list_del(&pDevDesc->list);

       mutex_unlock(&S_mtx);

       kfree(pDevDesc);
   }

   module_put(THIS_MODULE);

   return DRIVER_SUCCESS;
}

static ssize_t device_read(
      struct file* filp,   /* see include/linux/fs.h   */
      char __user* bufp,   /* buffer to be filled with data */
      size_t       len,    /* length of the buffer     */
      loff_t*      ppos)
{

   ATEMSYS_T_DEVICE_DESC*   pDevDesc = (ATEMSYS_T_DEVICE_DESC*) filp->private_data;
   ATEMSYS_T_IRQ_DESC*      pIrqDesc = NULL;
   s32 nPending;
   wait_queue_entry_t wait;

   if (! pDevDesc)
   {
      return -EINVAL;
   }

   pIrqDesc = &(pDevDesc->irqDesc);

   /* DBG("device_read...(0x%px,0x%px,%d)\n", filp, bufp, len); */

   init_wait(&wait);

   if (len < sizeof(u32))
   {
      return -EINVAL;
   }

   if (pIrqDesc->irq == 0) /* IRQ already disabled */
   {
      return -EINVAL;
   }

   nPending = atomic_read(&pIrqDesc->count);
   if (nPending == 0)
   {
      if (dev_irq_disabled(pIrqDesc))
      {
         dev_enable_irq(pIrqDesc);
      }
      if (filp->f_flags & O_NONBLOCK)
      {
         return -EWOULDBLOCK;
      }
   }

   while (nPending == 0)
   {
      prepare_to_wait(&pIrqDesc->q, &wait, TASK_INTERRUPTIBLE);
      nPending = atomic_read(&pIrqDesc->count);
      if (nPending == 0)
      {
         schedule();
      }
      finish_wait(&pIrqDesc->q, &wait);
      if (pIrqDesc->irq == 0) /* IRQ disabled while waiting for IRQ */
      {
         return -EINVAL;
      }
      if (signal_pending(current))
      {
         return -ERESTARTSYS;
      }
   }

   if (copy_to_user(bufp, &nPending, sizeof(nPending)))
   {
      return -EFAULT;
   }

   *ppos += sizeof(nPending);
   atomic_sub(nPending, &pIrqDesc->count);

   return sizeof(nPending);
}

/*
 * character device mmap method
 */
static int device_mmap(struct file* filp, struct vm_area_struct* vma)
{
   ATEMSYS_T_DEVICE_DESC*   pDevDesc = filp->private_data;

   int         nRet = -EIO;
   u32         dwLen;
   void*       pVa = NULL;
   dma_addr_t  dmaAddr;
   ATEMSYS_T_MMAP_DESC* pMmapNode;
   int         i;
   unsigned long ioBase;
   u32 dwIOLen, dwPageOffset;

   DBG("mmap: vm_pgoff 0x%px vm_start = 0x%px vm_end = 0x%px\n",
         (void*) vma->vm_pgoff, (void*) vma->vm_start, (void*) vma->vm_end);

   if (pDevDesc == NULL)
   {
      ERR("mmap: Invalid device dtor\n");
      goto Exit;
   }

   dwLen = PAGE_UP(vma->vm_end - vma->vm_start);

   vm_flags_set(vma, VM_DONTEXPAND | VM_DONTDUMP | VM_LOCKED | VM_DONTCOPY);

   if (vma->vm_pgoff != 0)
   {
      /* map device IO memory */
      if (pDevDesc->pPcidev != NULL)
      {
         INF("mmap: Doing PCI device sanity check\n");

         /* sanity check. Make sure that the offset parameter of the mmap() call in userspace
          * corresponds with the PCI base IO address.
          * Make sure the user doesn't map more IO memory than the device provides.
          */
         for (i = 0; i < ATEMSYS_PCI_MAXBAR; i++)
         {
            if (pci_resource_flags(pDevDesc->pPcidev, i) & IORESOURCE_MEM)
            {
               /* IO area address */
               ioBase = PAGE_DOWN( pci_resource_start(pDevDesc->pPcidev, i) );

               dwPageOffset = pci_resource_start(pDevDesc->pPcidev, i) - ioBase;

               /* IO area length */
               dwIOLen = PAGE_UP( pci_resource_len(pDevDesc->pPcidev, i) + dwPageOffset );

               if (    ((vma->vm_pgoff << PAGE_SHIFT) >= ioBase)
                    && (((vma->vm_pgoff << PAGE_SHIFT) + dwLen) <= (ioBase + dwIOLen))
                  )
               {
                  /* for systems where physical address is in x64 space, high dword is not passes from user io
                   * use correct address from pci_resource_start */
                  resource_size_t res_start = pci_resource_start(pDevDesc->pPcidev, i);
                  unsigned long pgoff_new = (res_start>>PAGE_SHIFT);
                  if (pgoff_new != vma->vm_pgoff)
                  {
                      INF("mmap: Correcting page offset from 0x%lx to 0x%lx, for Phys address 0x%llx",
                              vma->vm_pgoff, pgoff_new, (u64)res_start);
                      vma->vm_pgoff =  pgoff_new;
                  }

                  break;
               }
            }
         }

         /* IO bar not found? */
         if (i == ATEMSYS_PCI_MAXBAR)
         {
            ERR("mmap: Invalid arguments\n");
            nRet = -EINVAL;
            goto Exit;
         }
      }

      /* avoid swapping, request IO memory */
      vm_flags_set(vma, VM_IO);

      /*
       * avoid caching (this is at least needed for POWERPC,
       * or machine will lock on first IO access)
       */
      vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

      if ((nRet = remap_pfn_range(vma,
                                 vma->vm_start,
                                 vma->vm_pgoff,
                                 dwLen,
                                 vma->vm_page_prot)) < 0)
      {
         ERR("mmap: remap_pfn_range failed\n");
         goto Exit;
      }

      INF("mmap: mapped IO memory, Phys:0x%llx UVirt:0x%px Size:%u\n",
           (u64) (((u64)vma->vm_pgoff) << PAGE_SHIFT), (void*) vma->vm_start, dwLen);
   }
   else
   {
      /* allocated and map DMA memory */
      if (pDevDesc->pPcidev != NULL)
      {
         pVa = dma_alloc_coherent(&pDevDesc->pPcidev->dev, dwLen, &dmaAddr, GFP_KERNEL);
         if (NULL == pVa)
         {
            ERR("mmap: dma_alloc_coherent failed\n");
            nRet = -ENOMEM;
            goto Exit;
         }
      }
      else
      {
         pVa = dev_dma_alloc(dwLen, &dmaAddr);
         if (NULL == pVa)
         {
            ERR("mmap: dev_dma_alloc failed\n");
            nRet = -ENOMEM;
            goto Exit;
         }
      }

      /* zero memory for security reasons */
      memset(pVa, 0, dwLen);

      /* Always use noncached DMA memory for ARM. Otherwise cache invaliation/sync
       * would be necessary from usermode.
       * Can't do that without a kernel call because this OP's are privileged.
       */

      /* map the whole physically contiguous area in one piece */
      {
         struct device* pDmaDev = NULL;

         if (NULL != pDevDesc->pPcidev)
         {
            pDmaDev = &pDevDesc->pPcidev->dev;
         }
         else
         if (NULL != pDevDesc->pPlatformDev)
         {
            pDmaDev = &pDevDesc->pPlatformDev->dev;
         }
            /* for Platform Device */
         nRet = dma_mmap_coherent(pDmaDev,
                                     vma,       /* user space mapping                   */
                                     pVa,       /* kernel virtual address               */
                                     dmaAddr,   /* Phys address                         */
                                     dwLen);    /* size         */
         if (nRet < 0)
         {
            ERR("dma_mmap_coherent failed\n");
            goto ExitAndFree;
         }
      }

      /* Write the lower and upper parts of the physical DMA 64 bit address 
         into the first and second 4 bytes of allocated memory */
      ((u32*) pVa)[0] = (u32)((u64)dmaAddr & 0xFFFFFFFF);
      ((u32*) pVa)[1] = (u32)(((u64)dmaAddr >> 32) & 0xFFFFFFFF);

      /* Some housekeeping to be able to cleanup the allocated memory later */
      pMmapNode = kzalloc(sizeof(ATEMSYS_T_MMAP_DESC), GFP_KERNEL);
      if (! pMmapNode)
      {
         ERR("mmap: kmalloc() failed\n");
         nRet = -ENOMEM;
         goto ExitAndFree;
      }

      pMmapNode->pDevDesc = pDevDesc;
      pMmapNode->dmaAddr = dmaAddr;
      pMmapNode->pVirtAddr = pVa;
      pMmapNode->len = dwLen;

      /* Setup close callback -> deallocates DMA memory if region is unmapped by the system */
      vma->vm_ops = &mmap_vmop;
      vma->vm_private_data = pMmapNode;

      INF("mmap: mapped DMA memory, Phys:0x%px KVirt:0x%px UVirt:0x%px Size:%u\n",
             (void*)(unsigned long)dmaAddr, (void*)pVa, (void*)vma->vm_start, dwLen);
   }

   nRet = 0;

   goto Exit;

ExitAndFree:

   if (pVa == NULL) goto Exit;

   if (pDevDesc->pPcidev != NULL)
   {
      dma_free_coherent(&pDevDesc->pPcidev->dev, dwLen, pVa, dmaAddr);
   }
   else
   {
      dev_dma_free(dwLen, pVa);
   }

Exit:
   return nRet;
}


/*
 * This function is called whenever a process tries to do an ioctl on our
 * device file.
 *
 * If the ioctl is write or read/write (meaning output is returned to the
 * calling process), the ioctl call returns the output of this function.
 *
 */
static long atemsys_ioctl(
      struct file* file,
      unsigned int cmd,
      unsigned long arg)
{
   ATEMSYS_T_DEVICE_DESC*   pDevDesc = file->private_data;

   int nRetVal = -EFAULT;

   if (pDevDesc == NULL)
   {
      ERR("ioctl: Invalid device dtor\n");
      goto Exit;
   }

   /*
    * Switch according to the ioctl called
    */
   switch (cmd)
   {
      case ATEMSYS_IOCTL_PCI_FIND_DEVICE:
      {
         nRetVal = ioctl_pci_finddevice(pDevDesc, arg, _IOC_SIZE(cmd)); /* size determines version */
         if (0 != nRetVal)
         {
           /* be quiet. ioctl may fail */
           goto Exit;
         }
      } break;
      case ATEMSYS_IOCTL_PCI_CONF_DEVICE:
      {
         nRetVal = ioctl_pci_configure_device(pDevDesc, arg, _IOC_SIZE(cmd)); /* size determines version */
         if (0 != nRetVal)
         {
            ERR("ioctl ATEMSYS_IOCTL_PCI_CONF_DEVICE failed: %d\n", nRetVal);
            goto Exit;
         }
      } break;

      case ATEMSYS_IOCTL_PCI_RELEASE_DEVICE:
      {
         if (pDevDesc->pPcidev == NULL)
         {
            DBG("pci_release: No PCI device selected. Call ioctl(ATEMSYS_IOCTL_PCI_CONF_DEVICE) first\n");
            goto Exit;
         }
         /* do nothing */
         /* see device_release() -> dev_pci_release(pDevDesc)*/
      } break;
      case ATEMSYS_IOCTL_INT_CONNECT:
      {
         nRetVal = ioctl_int_connect(pDevDesc, arg);
         if (0 != nRetVal)
         {
            ERR("ioctl ATEMSYS_IOCTL_INT_CONNECT failed: %d\n", nRetVal);
            goto Exit;
         }
      } break;

      case ATEMSYS_IOCTL_INT_DISCONNECT:
      {
         nRetVal = dev_int_disconnect(pDevDesc);
         if (0 != nRetVal)
         {
            /* be quiet. ioctl may fail */
            goto Exit;
         }
      } break;

      case ATEMSYS_IOCTL_INT_SET_CPU_AFFINITY:
      {
          nRetVal = SetIntCpuAffinityIoctl(pDevDesc, arg, _IOC_SIZE(cmd));
          if (0 != nRetVal)
          {
              ERR("ioctl ATEMSYS_IOCTL_INT_SET_CPU_AFFINITY failed: %d\n", nRetVal);
              goto Exit;
          }
      } break;

      default:
      {
         nRetVal = -EOPNOTSUPP;
         goto Exit;
      } /* no break */
   }

   nRetVal = DRIVER_SUCCESS;

Exit:
   return nRetVal;
}

/* Module Declarations */

/*
 * This structure will hold the functions to be called
 * when a process does something to the device we
 * created. Since a pointer to this structure is kept in
 * the devices table, it can't be local to
 * module_init. NULL is for unimplemented functions.
 */


struct file_operations Fops = {
   .read = device_read,
   .unlocked_ioctl = atemsys_ioctl,
   .open = device_open,
   .mmap = device_mmap,
   .release = device_release,   /* a.k.a. close */
};

#define ATEMSYS_PCI_DRIVER_NAME "atemsys_pci"
#define PCI_VENDOR_ID_BECKHOFF  0x15EC

static void PciDriverRemove(struct pci_dev* pPciDev)
{
    ATEMSYS_T_PCI_DRV_DESC_PRIVATE* pPciDrvDescPrivate = (ATEMSYS_T_PCI_DRV_DESC_PRIVATE*)pci_get_drvdata(pPciDev);

    if (NULL != pPciDrvDescPrivate)
    {
        /* remove references to the device */
        if (NULL != pPciDrvDescPrivate->pDevDesc)
        {
            pPciDrvDescPrivate->pDevDesc->pPcidev = NULL;
            pPciDrvDescPrivate->pDevDesc->pPciDrvDesc = NULL;
            pPciDrvDescPrivate->pDevDesc = NULL;
        }
        S_apPciDrvDescPrivate[pPciDrvDescPrivate->dwIndex] = NULL;

        kfree(pPciDrvDescPrivate);
    }

    /* disable device */
    pci_disable_msi(pPciDev);
    pci_release_regions(pPciDev);
    // pci_disable_pcie_error_reporting(pPciDev);
    pci_disable_device(pPciDev);

    INF("%s: %s: disconnected\n", pci_name(pPciDev), ATEMSYS_PCI_DRIVER_NAME);
}

static int PciDriverProbe(struct pci_dev* pPciDev, const struct pci_device_id* id)
{
    ATEMSYS_T_PCI_DRV_DESC_PRIVATE* pPciDrvDescPrivate = NULL;
    int nRes = -ENODEV;
    int dwIndex = 0;

    /* check if is wanted pci device */
    if ((strcmp(AllowedPciDevices, "PCI_ANY_ID") != 0) && (strstr(AllowedPciDevices, pci_name(pPciDev)) == NULL))
    {
        /* don't attach driver */
        DBG("%s: PciDriverProbe: restricted by user parameters!\n", pci_name(pPciDev));

        return -ENODEV; /* error code doesn't create error message */
    }

    /* setup pci device */
    nRes = pci_enable_device_mem(pPciDev);
    if (nRes)
    {
        ERR("%s: PciDriverProbe: pci_enable_device_mem failed!\n", pci_name(pPciDev));
        goto Exit;
    }

    nRes = DefaultPciSettings(pPciDev);
    if (nRes)
    {
        ERR("%s: PciDriverProbe: DefaultPciSettings failed\n", pci_name(pPciDev));
        goto Exit;
    }
    pci_save_state(pPciDev);
    //pci_enable_pcie_error_reporting(pPciDev);
    nRes = pci_request_regions(pPciDev, ATEMSYS_DEVICE_NAME);
    if (nRes < 0)
    {
        ERR("%s: PciDriverProbe: device in use by another driver?\n", pci_name(pPciDev));
        nRes = -EBUSY;
        goto Exit;
    }

    /* create private desc */
    pPciDrvDescPrivate = (ATEMSYS_T_PCI_DRV_DESC_PRIVATE*)kzalloc(sizeof(ATEMSYS_T_PCI_DRV_DESC_PRIVATE), GFP_KERNEL);
    if (NULL == pPciDrvDescPrivate)
    {
        nRes = -ENOMEM;
        goto Exit;
    }
    pPciDrvDescPrivate->pPciDev = pPciDev;

    /* get Pci Info */
    pPciDrvDescPrivate->wVendorId         = pPciDev->vendor;
    pPciDrvDescPrivate->wDevice           = pPciDev->device;
    pPciDrvDescPrivate->wRevision         = pPciDev->revision;
    pPciDrvDescPrivate->wSubsystem_vendor = pPciDev->subsystem_vendor;
    pPciDrvDescPrivate->wSubsystem_device = pPciDev->subsystem_device;
    pPciDrvDescPrivate->nPciBus           = pPciDev->bus->number;
    pPciDrvDescPrivate->nPciDomain        = pci_domain_nr(pPciDev->bus);
    pPciDrvDescPrivate->nPciDev           = PCI_SLOT(pPciDev->devfn);
    pPciDrvDescPrivate->nPciFun           = PCI_FUNC(pPciDev->devfn);

    INF("%s: %s: connected vendor:0x%04x device:0x%04x rev:0x%02x - sub_vendor:0x%04x sub_device:0x%04x\n", pci_name(pPciDev), ATEMSYS_PCI_DRIVER_NAME,
            pPciDev->vendor, pPciDev->device, pPciDev->revision,
            pPciDev->subsystem_vendor, pPciDev->subsystem_device);

    /* find the memory BAR */
    {
       unsigned long ioBase  = 0;
       unsigned int  dwIOLen = 0;
       int i    = 0;
       int nBar = 0;

       for (i = 0; i < ATEMSYS_PCI_MAXBAR ; i++)
       {
          if (pci_resource_flags(pPciDev, i) & IORESOURCE_MEM)
          {
             /* IO area address */
             ioBase = pci_resource_start(pPciDev, i);
             pPciDrvDescPrivate->aBars[nBar].qwIOMem = ioBase;

             /* IO area length */
             dwIOLen = pci_resource_len(pPciDev, i);
             pPciDrvDescPrivate->aBars[nBar].dwIOLen = dwIOLen;

             nBar++;
          }
       }

       if (nBar == 0)
       {
          WRN("%s: PciDriverProbe: No memory BAR found\n", pci_name(pPciDev));
       }

       pPciDrvDescPrivate->nBarCnt = nBar;
    }

    /* insert device to array */
    for (dwIndex = 0; dwIndex < ATEMSYS_MAX_NUMBER_DRV_INSTANCES; dwIndex++)
    {
        if (NULL == S_apPciDrvDescPrivate[dwIndex])
        {
            S_apPciDrvDescPrivate[dwIndex] = pPciDrvDescPrivate;
            pPciDrvDescPrivate->dwIndex =  dwIndex;
            break;
        }
    }
    if (ATEMSYS_MAX_NUMBER_DRV_INSTANCES <= dwIndex)
    {
        ERR("%s: PciDriverProbe: insert device to array failed\n", pci_name(pPciDev));
        nRes = -EBUSY;
        goto Exit;
    }

    pci_set_drvdata(pPciDev, pPciDrvDescPrivate);

    nRes = 0; /* OK */
Exit:
    if (nRes != 0 /* OK */)
    {
        if (NULL != pPciDrvDescPrivate)
        {
            kfree(pPciDrvDescPrivate);
        }
    }
    return nRes;
}

typedef struct _ATEMSYS_PCI_INFO {
} ATEMSYS_PCI_INFO;

static const struct _ATEMSYS_PCI_INFO oAtemsysPciInfo = {
};


static const struct pci_device_id pci_devtype[] = {
    {
    /* all devices of class PCI_CLASS_NETWORK_ETHERNET */
    .vendor      = PCI_ANY_ID,
    .device      = PCI_ANY_ID,
    .subvendor   = PCI_ANY_ID,
    .subdevice   = PCI_ANY_ID,
    .class       = (PCI_CLASS_NETWORK_ETHERNET << 8),
    .class_mask  = (0xFFFF00),
    .driver_data = (kernel_ulong_t)&oAtemsysPciInfo
    },
    {
     /* all devices with BECKHOFF vendor id */
    .vendor      = PCI_VENDOR_ID_BECKHOFF,
    .device      = PCI_ANY_ID,
    .subvendor   = PCI_ANY_ID,
    .subdevice   = PCI_ANY_ID,
    .driver_data = (kernel_ulong_t)&oAtemsysPciInfo
    },
    {}
};

MODULE_DEVICE_TABLE(pci, pci_devtype);
static struct pci_driver oPciDriver = {
    .name     = ATEMSYS_PCI_DRIVER_NAME,
    .id_table = pci_devtype,
    .probe    = PciDriverProbe,
    .remove   = PciDriverRemove,
};

/*
 * Initialize the module - Register the character device
 */
int init_module(void)
{
    /* Register the character device */
    int major = register_chrdev(MAJOR_NUM, ATEMSYS_DEVICE_NAME, &Fops);
    if (major < 0)
    {
        INF("Failed to register %s (err: %d)\n",
               ATEMSYS_DEVICE_NAME, major);
        return major;
    }

    /* Register Pci and Platform Driver */

    memset(S_apPciDrvDescPrivate ,0, ATEMSYS_MAX_NUMBER_DRV_INSTANCES * sizeof(ATEMSYS_T_PCI_DRV_DESC_PRIVATE*));

    if (0 == strcmp(AllowedPciDevices, ""))
    {
        DBG("Atemsys PCI driver not registered\n");
    }
    else
    {
        if (0 != pci_register_driver(&oPciDriver))
        {
            INF("Register Atemsys PCI driver failed!\n");
        }
    }

    S_pDevClass = class_create(ATEMSYS_DEVICE_NAME);
    if (IS_ERR(S_pDevClass))
    {
        INF("class_create failed\n");
        return -1;
    }

    S_pDev = device_create(S_pDevClass, NULL, MKDEV(MAJOR_NUM, 0), NULL, ATEMSYS_DEVICE_NAME);

    S_pPlatformDev = NULL;

    if (IS_ERR(S_pDev))
    {
        INF("device_create failed\n");
        return -1;
    }

    S_pDev->coherent_dma_mask = DMA_BIT_MASK(32);
    if (!S_pDev->dma_mask)
    {
        S_pDev->dma_mask = &S_pDev->coherent_dma_mask;
    }

    INIT_LIST_HEAD(&S_DevNode.list);

    INF("%s loaded\n", ATEMSYS_DEVICE_NAME);
    return 0;
}

/*
 * Cleanup - unregister the appropriate file from /proc
 */
void cleanup_module(void)
{
   INF("%s unloaded\n", ATEMSYS_DEVICE_NAME);

    /* Unregister Pci and Platform Driver */
    if (0 != strcmp(AllowedPciDevices, ""))
    {
        pci_unregister_driver(&oPciDriver);
    }

   device_destroy(S_pDevClass, MKDEV(MAJOR_NUM, 0));
   class_destroy(S_pDevClass);
   unregister_chrdev(MAJOR_NUM, ATEMSYS_DEVICE_NAME);
}

