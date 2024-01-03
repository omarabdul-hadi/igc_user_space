
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <pthread.h>

#include "atemsys_main.h"
#include "igc/igc.h"

static struct igc_adapter adapter;
static ATEMSYS_T_PCI_SELECT_DESC pci_device_descriptor;

void send_frame(uint8_t* data, int len) {

	// struct timeval  bgn, end;
	// static int iii = 0;
	// if (iii < 1000)
	// {
	// 	gettimeofday(&bgn, NULL);
	// }
	igc_xmit_frame(&adapter, data, len);
	// if (iii < 1000)
	// {
	// 	gettimeofday(&end, NULL);
	// 	printf ("Total time = %li us\n", (end.tv_usec - bgn.tv_usec) + 1000000 * (end.tv_sec - bgn.tv_sec));
	// 	iii++;
	// }
}

void init() {
	printf("Intel(R) 2.5G Ethernet Linux Driver\n");
	printf("driver name: igc\n");
	printf("auther: Intel Corporation, <linux.nics@intel.com>\n");
	printf("Copyright(c) 2018 Intel Corporation.");

	int32_t pending_interrupts = 0;
    int val = 0;
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
  		val = read(adapter.fd, &pending_interrupts, sizeof(int32_t));
		igc_intr_msi(&adapter);
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

void *thread_func(void *data) {
	// sample frame
	uint8_t buff[40] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00, 0x88, 0xa4, 0x0d, 0x10,
                        0x02, 0x01, 0x00, 0x00, 0x00, 0x05, 0x01, 0x00,  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	int len = 29;

	int32_t pending_interrupts = 0;
    int val = 0;

    init();

	while (true) {
		send_frame(buff, len);
		usleep(10);
		val = read(adapter.fd, &pending_interrupts, 4);
		igc_intr_msi(&adapter);
		usleep(100);
	}

    deinit();

	return NULL;
}

int main()
{
	struct sched_param param;
	pthread_attr_t attr;
	pthread_t thread;
	int ret;

	/* Lock memory */
	if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
		printf("mlockall failed: %m\n");
		exit(-2);
	}

	/* Initialize pthread attributes (default values) */
	ret = pthread_attr_init(&attr);
	if (ret) {
		printf("init pthread attributes failed\n");
		goto out;
	}

	/* Set a specific stack size  */
	ret = pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);
	if (ret) {
		printf("pthread setstacksize failed\n");
		goto out;
	}

	/* Set scheduler policy and priority of pthread */
	ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
	if (ret) {
		printf("pthread setschedpolicy failed\n");
		goto out;
	}
	param.sched_priority = 99;
	ret = pthread_attr_setschedparam(&attr, &param);
	if (ret) {
		printf("pthread setschedparam failed\n");
		goto out;
	}
	/* Use scheduling parameters of attr */
	ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	if (ret) {
		printf("pthread setinheritsched failed\n");
		goto out;
	}

	/* Create a pthread with specified attributes */
	ret = pthread_create(&thread, &attr, thread_func, NULL);
	if (ret) {
		printf("create pthread failed\n");
		goto out;
	}

	/* Join the thread and wait until it is done */
	ret = pthread_join(thread, NULL);
	if (ret)
		printf("join pthread failed: %m\n");
 
out:
    return ret;
}
