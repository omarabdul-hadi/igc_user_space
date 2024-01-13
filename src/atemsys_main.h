#ifndef _ATEMSYS_MAIN_H_
#define _ATEMSYS_MAIN_H_

#include <stdint.h>
#include "atemsys/atemsys.h"

int atemsys_pci_open(ATEMSYS_T_PCI_SELECT_DESC* pci_device_descriptor);
void* atemsys_map_io(int fd, ATEMSYS_T_PCI_SELECT_DESC* pci_device_descriptor);
void* atemsys_map_dma(int fd, uint32_t size, int prot);
uint64_t atemsys_get_dma_addr(void* allocated_mem_ptr);
void atemsys_pci_intr_enable(int fd, ATEMSYS_T_PCI_SELECT_DESC* pci_device_descriptor);
void atemsys_pci_set_affin(int fd, int cpu);
void atemsys_pci_intr_disable(int fd, ATEMSYS_T_PCI_SELECT_DESC* pci_device_descriptor);
void atemsys_unmap_mem(void* allocated_mem_ptr, uint32_t size);
void atemsys_pci_close(int fd, ATEMSYS_T_PCI_SELECT_DESC* pci_device_descriptor);

#endif /* _ATEMSYS_MAIN_H_ */