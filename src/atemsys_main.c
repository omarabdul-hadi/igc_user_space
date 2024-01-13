#define _GNU_SOURCE
#include <stdio.h>
#undef _GNU_SOURCE
#include <stdint.h>
#include <time.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sched.h>
#include <stdbool.h>
#include <linux/types.h>
#include "atemsys/atemsys.h"


int atemsys_pci_open(ATEMSYS_T_PCI_SELECT_DESC* pci_device_descriptor) {
	int fd = open(ATEMSYS_FILE_NAME, O_RDWR);
    if(fd < 0) {
        printf("Cannot open device file...\n");
        return 0;
    }

    // $ lspci -nn | grep -i 'Ethernet Controller'
    // 56:00.0 Ethernet controller [0200]: Intel Corporation Ethernet Controller I225-V [8086:15f3] (rev 03)
    pci_device_descriptor->nVendID   = 0x8086;   // intel
    pci_device_descriptor->nDevID    = 0x15f3;   // I225-V
    pci_device_descriptor->nInstance = 0;

    if (ioctl(fd, ATEMSYS_IOCTL_PCI_FIND_DEVICE, pci_device_descriptor) == -1) {
        printf("ioctl atemsys ATEMSYS_IOCTL_PCI_FIND_DEVICE request failed\n");
        return 0;
    }

    if (ioctl(fd, ATEMSYS_IOCTL_PCI_CONF_DEVICE, pci_device_descriptor) == -1) {
        printf("ioctl atemsys ATEMSYS_IOCTL_PCI_CONF_DEVICE request failed\n");
        return 0;
    }

    uint32_t io_phys_base_addr = pci_device_descriptor->aBar[0].qwIOMem;
    uint32_t size              = pci_device_descriptor->aBar[0].dwIOLen;

    printf("io_phys_base_addr is 0x%x\n", io_phys_base_addr);  // this should be: 0x84200000
    printf("io_phys_base_leng is 0x%x\n", size);               // this should be: 0x  100000

	return fd;
}

void* atemsys_map_io(int fd, ATEMSYS_T_PCI_SELECT_DESC* pci_device_descriptor) {
	uint32_t io_phys_base_addr = pci_device_descriptor->aBar[0].qwIOMem;
	uint32_t size              = pci_device_descriptor->aBar[0].dwIOLen;

	void* io_addr = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_LOCKED, fd, io_phys_base_addr);
    printf("io_addr              is 0x%lx\n", (uint64_t) io_addr);
    printf("io_addr uint32_t_val is 0x%x\n", *((uint32_t*) io_addr));
	return io_addr;
}

void* atemsys_map_dma(int fd, uint32_t size, int prot) {
	void* allocated_mem_ptr = mmap(0, size, prot, MAP_SHARED | MAP_LOCKED, fd, 0);
    //printf("allocated_mem_ptr is 0x%lx,  prot %i\n", (uint64_t) allocated_mem_ptr, prot);
	return allocated_mem_ptr;
}

uint64_t atemsys_get_dma_addr(void* allocated_mem_ptr) {
	uint64_t dma_phys_addr = 0;
    ((uint32_t*) &dma_phys_addr)[0] = ((uint32_t*) allocated_mem_ptr)[0];
    ((uint32_t*) &dma_phys_addr)[1] = ((uint32_t*) allocated_mem_ptr)[1];

	return dma_phys_addr;
}

void atemsys_pci_intr_enable(int fd, ATEMSYS_T_PCI_SELECT_DESC* pci_device_descriptor) {

    if (ioctl(fd, ATEMSYS_IOCTL_INT_CONNECT, pci_device_descriptor->dwIrq) == -1) {
        printf("ioctl atemsys ATEMSYS_IOCTL_INT_CONNECT request failed\n");
    }
}

void atemsys_pci_set_affin(int fd, int cpu) {
    cpu_set_t cpu_mask;
    CPU_ZERO(&cpu_mask);
    CPU_SET(cpu, &cpu_mask);

    if (ioctl(fd, ATEMSYS_IOCTL_INT_SET_CPU_AFFINITY, &cpu_mask) == -1) {
        printf("ioctl atemsys ATEMSYS_IOCTL_INT_SET_CPU_AFFINITY request failed\n");
    }
}

void atemsys_pci_intr_disable(int fd, ATEMSYS_T_PCI_SELECT_DESC* pci_device_descriptor) {

    if (ioctl(fd, ATEMSYS_IOCTL_INT_DISCONNECT, pci_device_descriptor->dwIrq) == -1) {
        printf("ioctl atemsys ATEMSYS_IOCTL_INT_DISCONNECT request failed\n");
    }
}

void atemsys_unmap_mem(void* allocated_mem_ptr, uint32_t size) {
	if (munmap(allocated_mem_ptr, size) == -1) {
        printf("munmap() api call failed\n");
    }
}

void atemsys_pci_close(int fd, ATEMSYS_T_PCI_SELECT_DESC* pci_device_descriptor) {

	if (ioctl(fd, ATEMSYS_IOCTL_PCI_RELEASE_DEVICE, &pci_device_descriptor) == -1) {
        printf("ioctl atemsys ATEMSYS_IOCTL_PCI_RELEASE_DEVICE request failed\n");
    }

    close(fd);
}