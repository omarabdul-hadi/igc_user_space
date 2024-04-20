#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <pthread.h>

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
	char buff[10] = {0};
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
	PCI_ID pci_id;

	// loop over all ethernet controllers incase there are multiple ones
    for (int i = 0; i < a; ++ i) {
		if (!find_vendor_and_device_id_str(output[i], &pci_id) &&
		     pci_id.vendor_id == INTEL_VENDOR_ID               &&
			 pci_id.device_id == I225V_DEVICE_ID)
			return true;
    }
	return false;
}

void* intr_handling_thread(void*) {
	printf("igc user space, now launching interrupt handling thread for updating link status\n");
	while (!adapter.state_down) {
		atemsys_pci_intr_wait(adapter.fd); // this blocks untill an intr is received
		igc_intr_msi(&adapter);            // this handles the intr according to the type, note that only link status change intr are enabled
	}
	printf("igc user space, now killing interrupt handling thread for updating link status\n");
	return NULL;
}

int launch_intr_handling_thread() {
	pthread_attr_t attr;
	pthread_t thread;
	int ret = 0;

	/* Initialize pthread attributes (default values) */
	ret = pthread_attr_init(&attr);
	if (ret) {
		printf("init pthread attributes failed\n");
		goto out;
	}

	/* Create a pthread with specified attributes */
	ret = pthread_create(&thread, &attr, intr_handling_thread, NULL);
	if (ret) {
		printf("create pthread failed\n");
		goto out;
	}
out:
    return ret;
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

	launch_intr_handling_thread(); // this handles link status changes
	usleep(2500000); // if link is present, it takes approx 2.5 secs to update 
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

bool igc_user_space_get_link_status() {
	return adapter.link;
}

void igc_user_space_send_frame(uint8_t* data, int len) {
	igc_send_frame(&adapter, data, len);
}

uint32_t igc_user_space_receive_frame(uint8_t* receive_pkt) {
	igc_clean_tx_irq(&adapter);
	return igc_clean_rx_irq(&adapter, receive_pkt);
}