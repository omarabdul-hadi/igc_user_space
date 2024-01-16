#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "atemsys_main.h"
#include "igc/igc.h"
#include "igc_user_space.h"

static struct igc_adapter adapter;
static ATEMSYS_T_PCI_SELECT_DESC pci_device_descriptor;



#define LINE_MAX_BUFFER_SIZE 255
 
int runExternalCommand(char *cmd, char lines[][LINE_MAX_BUFFER_SIZE]) {
	FILE *fp;
	char path[LINE_MAX_BUFFER_SIZE];

	/* Open the command for reading. */
	fp = popen(cmd, "r");
	if (fp == NULL) {
		return -1;
	}

	int cnt = 0;
	while (fgets(path, sizeof(path), fp) != NULL) {
		strcpy(lines[cnt++], path);
	}
	pclose(fp);
	return cnt;
}

typedef struct {
	uint32_t vendor_id;
	uint32_t device_id;
} PCI_ID;

int find_vendor_and_device_id_str(const char *str, PCI_ID* pci_id) {

	// on linux, if the pcie ethernet driver is supported, the following command should have the output below
	// $ lspci -nn | grep -i 'Ethernet Controller'
    // 56:00.0 Ethernet controller [0200]: Intel Corporation Ethernet Controller I225-V [8086:15f3] (rev 03)

	const char *bgn = str;
	const char *end = str;
	char buff[10];
	int len;

	//////////////////////////////////////////////////////////////////////////////////////
	// vendor and device id is located after the second "[" char in the returned string
	for (int i = 0; i < 2; i++) {
		bgn = strstr(bgn, "[");
		if (!bgn)
			return -1;
		bgn += 1;
	}
	//////////////////////////////////////////////////////////////////////////////////////
	// get vendor id, it is located between 2nd "[" and ":"
	end = bgn;

	end = strstr(end, ":");
	if (!end)
		return -1;

	len = end-bgn;
	strncpy(buff, bgn, len);
	pci_id->vendor_id = (int)strtol(buff, NULL, 16);

	//////////////////////////////////////////////////////////////////////////////////////
	// get device id, it is located between ":" and 2nd "]"
	bgn = end+1;

	end = strstr(bgn, "]");
	if (!end)
		return -1;

	len = end-bgn;
	strncpy(buff, bgn, len);
	pci_id->device_id = (int)strtol(buff, NULL, 16);

	return 0;
}

#define INTEL_VENDOR_ID 0x8086
#define I225V_DEVICE_ID 0x15f3

bool igc_user_space_supported() {

    char output[10][LINE_MAX_BUFFER_SIZE];
    int a = runExternalCommand("lspci -nn | grep -i 'Ethernet Controller'", output);

	// loop over all ethernet controllers incase there are multiple ones
    for (int i = 0; i < a; ++ i) {
        printf("%s", output[i]);

		PCI_ID pci_id;
		if (!find_vendor_and_device_id_str(output[i], &pci_id) &&
		     pci_id.vendor_id == INTEL_VENDOR_ID               &&
			 pci_id.device_id == I225V_DEVICE_ID)
			return true;
    }
	return false;
}

void spin_sleep(uint32_t delay_us) {
	
	struct timeval  bgn, end;

	// yes this is very ugly and terrible, but handing off the process to the OS via a usleep() introduces occational added latency of more than 100 us
	// which is quite detrimental! clock_nanosleep() also made no difference
	gettimeofday(&bgn, NULL);
	while (true) {
		gettimeofday(&end, NULL);
		if ((end.tv_usec - bgn.tv_usec) + 1000000 * (end.tv_sec - bgn.tv_sec) >= delay_us)
			break;
	}
}

void igc_user_space_init() {
	printf("Intel(R) 2.5G Ethernet Linux Driver\n");
	printf("driver name: igc\n");
	printf("auther: Intel Corporation, <linux.nics@intel.com>\n");
	printf("Copyright(c) 2018 Intel Corporation.\n");
	printf("Driver has been minimalized and moved to user space by Omar Abdul-hadi to reduce latency\n");

	int fd = atemsys_pci_open(&pci_device_descriptor);
    void* io_addr = atemsys_map_io(fd, &pci_device_descriptor);

	igc_probe(&adapter, fd, (uint8_t*)io_addr);
	igc_open(&adapter);

	atemsys_pci_intr_enable(fd, &pci_device_descriptor);
    atemsys_pci_set_affin(fd, 15);

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

void igc_user_space_deinit() {
    atemsys_pci_intr_disable(adapter.fd, &pci_device_descriptor);
	igc_close(&adapter);
    igc_remove(&adapter);
	atemsys_unmap_mem(adapter.io_addr, pci_device_descriptor.aBar[0].dwIOLen); // unmap io memory
    atemsys_pci_close(adapter.fd, &pci_device_descriptor);
}

void igc_user_space_get_mac(uint8_t* mac_addr) {
	memcpy(mac_addr, adapter.hw.mac.addr, ETH_ALEN);
}

void igc_user_space_send_frame(uint8_t* data, int len) {
	igc_send_frame(&adapter, data, len);
}

uint32_t igc_user_space_receive_frame(uint8_t* receive_pkt) {
	uint32_t receive_pkt_len;

	// spin sleep is a necessary delay because interrupt is sometimes 7 us too early for cleaning the rx irq and receiving the frame

	atemsys_pci_intr_wait(adapter.fd);         // avg:  0 us, max: 130 us
	spin_sleep(7);                             // avg:  7 us, max:  23 us
	receive_pkt_len = igc_intr_msi(&adapter, receive_pkt);

	return receive_pkt_len;
}