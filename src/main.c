#define _GNU_SOURCE  // for CPU_ZERO, CPU_SET, and pthread_setaffinity_np
#include <stdio.h>
#undef _GNU_SOURCE
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <pthread.h>
#include <signal.h>

#include "atemsys_main.h"
#include "igc/igc.h"

static struct igc_adapter adapter;
static ATEMSYS_T_PCI_SELECT_DESC pci_device_descriptor;

void print_debug_packet(uint8_t* data, int size){
	printf("%02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x \n",
	*(data +  0), *(data +  1), *(data +  2), *(data +  3), *(data +  4),
	*(data +  5), *(data +  6), *(data +  7), *(data +  8), *(data +  9),
	*(data + 10), *(data + 11), *(data + 12), *(data + 13), *(data + 14),
	*(data + 15));

	printf("%02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x \n",
	*(data + 16), *(data + 17), *(data + 18), *(data + 19), *(data + 20),
	*(data + 21), *(data + 22), *(data + 23), *(data + 24), *(data + 25),
	*(data + 26), *(data + 27), *(data + 28), *(data + 29), *(data + 30),
	*(data + 31));

	printf("%02x %02x %02x %02x %02x %02x %02x %02x \n",
	*(data + 32), *(data + 33), *(data + 34), *(data + 35), *(data + 36),
	*(data + 37), *(data + 38), *(data + 39));

	printf("size: %d \n", size);
	printf("\n");
}

void spin_sleep(int delay_us) {
	
	struct timeval  bgn, end;

	gettimeofday(&bgn, NULL);
	while (true) {
		gettimeofday(&end, NULL);
		if ((end.tv_usec - bgn.tv_usec) + 1000000 * (end.tv_sec - bgn.tv_sec) >= delay_us)
			break;
	}
}

void send_frame(uint8_t* data, int len) {
	igc_send_frame(&adapter, data, len);
}

uint32_t receive_frame(uint8_t* receive_pkt) {
	uint32_t len, val, pending_interrupts;

	// spin sleep is a necessary delay because interrupt is sometimes 7 us too early for cleaning the rx irq and receiving the frame

	val = read(adapter.fd, &pending_interrupts, 4);    // avg:  0 us, max: 130 us
	spin_sleep(7);                                     // avg:  7 us, max:  23 us
	len = igc_intr_msi(&adapter, receive_pkt);

	return len;
}

void init() {
	printf("Intel(R) 2.5G Ethernet Linux Driver\n");
	printf("driver name: igc\n");
	printf("auther: Intel Corporation, <linux.nics@intel.com>\n");
	printf("Copyright(c) 2018 Intel Corporation.\n");

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

static uint32_t min = 10000000;
static uint32_t max = 0;
static uint64_t sum = 0;
static uint64_t cyc = 0;

void update_stats(uint32_t cur) {
	if (cur < min)
		min = cur;

	if (cur > max)
		max = cur;

	sum += cur;
	cyc++;
}

void print_stats(int sig) {
	printf("cycletime in us min: %03d, avg: %03ld, max: %03d\n", min, sum/cyc, max);
	exit(0);
}

void send_receive_cycle(uint8_t* send_pkt, uint32_t send_pkt_len) {

	uint32_t receive_pkt_len;
	uint8_t* receive_pkt = malloc(128);
	memset(receive_pkt, 0, 128);

	struct timeval  bgn, end;

	gettimeofday(&bgn, NULL);
	send_frame(send_pkt, send_pkt_len);           // avg:  0 us, max:  22 us
	receive_pkt_len = receive_frame(receive_pkt);
	gettimeofday(&end, NULL);
	
	uint32_t cur = (end.tv_usec - bgn.tv_usec) + 1000000 * (end.tv_sec - bgn.tv_sec);

	update_stats(cur);


	// static int iii = 0;
	// if (iii < 4)
	// {
	// 	printf("Tx packet:\n");
	// 	print_debug_packet(send_pkt, send_pkt_len);

	// 	printf("Rx packet:\n");
	// 	print_debug_packet(receive_pkt, receive_pkt_len);
	// 	iii++;
	// }

	free(receive_pkt);
}

void test_cycle() {
	uint8_t data1[40] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x48, 0x21,  0x0b, 0x51, 0x98, 0x04, 0x88, 0xa4, 0x0d, 0x10,
                         0x02, 0x01, 0x00, 0x00, 0x00, 0x05, 0x01, 0x00,  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	int len1 = 29;


	uint8_t data2[40] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x4a, 0x21,  0x0b, 0x51, 0x98, 0x04, 0x88, 0xa4, 0x12, 0x10, 
					     0x02, 0x02, 0x00, 0x00, 0x02, 0x05, 0x06, 0x00,  0x00, 0x00, 0x00, 0x01, 0x08, 0x00, 0x00, 0x00, 
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	int len2 = 34;


	uint8_t data3[40]= {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x4a, 0x21,  0x0b, 0x51, 0x98, 0x04, 0x88, 0xa4, 0x10, 0x10, 
                        0x01, 0x03, 0x00, 0x00, 0x08, 0x05, 0x04, 0x00,  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	int len3 = 32;


    uint8_t data4[40]= {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x4a, 0x21,  0x0b, 0x51, 0x98, 0x04, 0x88, 0xa4, 0x12, 0x10, 
                        0x02, 0x04, 0x00, 0x00, 0x02, 0x05, 0x06, 0x00,  0x00, 0x00, 0x00, 0x01, 0x0a, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	int len4 = 34;

	send_receive_cycle(data1, len1);
	send_receive_cycle(data2, len2);
	send_receive_cycle(data3, len3);
	send_receive_cycle(data4, len4);
}

void *thread_func(void *data) {
	init();
	test_cycle();
    deinit();

	init();
	//static int iii = 0;
	//while (iii < 1000) {
	while (true) {
	//	iii++;
		test_cycle();
	}
    deinit();

	init();
	test_cycle();
    deinit();

	return NULL;
}

int main()
{
	signal(SIGINT, print_stats);

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
	ret = pthread_attr_setstacksize(&attr, 16*PTHREAD_STACK_MIN);
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

	/* Set pthread cpu affinity */
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(15, &cpuset);
    pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset);

	/* Join the thread and wait until it is done */
	ret = pthread_join(thread, NULL);
	if (ret)
		printf("join pthread failed: %m\n");
 
	print_stats(0);
out:
    return ret;
}
