

#include <sys/time.h>

#include "atemsys_main.h"
#include "igc/igc.h"
#include "igc_user_space.h"

static struct igc_adapter adapter;
static ATEMSYS_T_PCI_SELECT_DESC pci_device_descriptor;

void spin_sleep(uint32_t delay_us) {
	
	struct timeval  bgn, end;

	gettimeofday(&bgn, NULL);
	while (true) {
		gettimeofday(&end, NULL);
		if ((end.tv_usec - bgn.tv_usec) + 1000000 * (end.tv_sec - bgn.tv_sec) >= delay_us)
			break;
	}
}

void init() {
	printf("Intel(R) 2.5G Ethernet Linux Driver\n");
	printf("driver name: igc\n");
	printf("auther: Intel Corporation, <linux.nics@intel.com>\n");
	printf("Copyright(c) 2018 Intel Corporation.\n");

	int fd = atemsys_pci_open(&pci_device_descriptor);
    void* io_addr = atemsys_map_io(fd, &pci_device_descriptor);

	igc_probe(&adapter, fd, (uint8_t*)io_addr);
	igc_open(&adapter);

	atemsys_pci_intr_enable(fd, &pci_device_descriptor);
    atemsys_pci_set_affin(fd, 15);

    uint32_t mac_addr_uint32_t_val_low = *((uint32_t*) (io_addr + IGC_RAL(0)) );
    uint32_t mac_addr_uint32_t_val_hig = *((uint32_t*) (io_addr + IGC_RAH(0)) );

    printf("mac addr uint32_t low val is 0x%x\n", mac_addr_uint32_t_val_low);
    printf("mac addr uint32_t hig val is 0x%x\n", mac_addr_uint32_t_val_hig);

	// wait for link
	usleep(2500000);

	// first interrupt is link status change, second one is unknown but needs to be cleared
	for (int i = 0; i < 2; i++) {
  		atemsys_pci_intr_wait(fd);
		igc_intr_msi(&adapter, NULL);
	}
	// at this point all interrupts are cleared and any call to read()
	// will block until a frame is sent or received, or the link status changes
}

void deinit() {
    atemsys_pci_intr_disable(adapter.fd, &pci_device_descriptor);
	igc_close(&adapter);
    igc_remove(&adapter);
	atemsys_unmap_mem(adapter.io_addr, pci_device_descriptor.aBar[0].dwIOLen); // unmap io memory
    atemsys_pci_close(adapter.fd, &pci_device_descriptor);
}

void send_frame(uint8_t* data, int len) {
	igc_send_frame(&adapter, data, len);
}

uint32_t receive_frame(uint8_t* receive_pkt) {
	uint32_t receive_pkt_len;

	// spin sleep is a necessary delay because interrupt is sometimes 7 us too early for cleaning the rx irq and receiving the frame

	atemsys_pci_intr_wait(adapter.fd);         // avg:  0 us, max: 130 us
	spin_sleep(7);                             // avg:  7 us, max:  23 us
	receive_pkt_len = igc_intr_msi(&adapter, receive_pkt);

	return receive_pkt_len;
}