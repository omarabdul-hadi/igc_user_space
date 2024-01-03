// SPDX-License-Identifier: GPL-2.0
// Copyright (c)  2018 Intel Corporation
// 
// Forked from linux version 6.6.1 by Omar Abdul-hadi
// Code was moved from kernel space to user space to reduce send/receive frame latency
//
// Removed support for vlan, tso, gso, ethtool, xdp, ptp, tsn, diag, stats, reset, and mac filters
// Removed connection to linux kernel, such as sk_buff, netapi, and pci
// Removed support for all ethernet cards besides I225-V
// Connected atemsys kernel module provided by acontis to provide hooks for
// dma allocation and freeing and pcie memory mapped io and interrupts

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "./../atemsys_main.h"
#include "igc.h"

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

void igc_reset(struct igc_adapter *adapter, bool power_down_phy)
{
	struct igc_hw *hw = &adapter->hw;

	hw->mac.ops.reset_hw(hw);

	if (hw->mac.ops.init_hw(hw))
		printf("igc driver: error: Error on hardware initialization\n");

	if (power_down_phy)
		igc_power_down_phy_copper_base(&adapter->hw);

	igc_get_phy_info(hw);
}

/**
 * igc_power_up_link - Power up the phy link
 * @adapter: address of board private structure
 */
static void igc_power_up_link(struct igc_adapter *adapter)
{
	igc_reset_phy(&adapter->hw);
	igc_power_up_phy_copper(&adapter->hw);
	igc_setup_link(&adapter->hw);
}

/**
 * igc_release_hw_control - release control of the h/w to f/w
 * @adapter: address of board private structure
 *
 * igc_release_hw_control resets CTRL_EXT:DRV_LOAD bit.
 * For ASF and Pass Through versions of f/w this means that the
 * driver is no longer loaded.
 */
static void igc_release_hw_control(struct igc_adapter *adapter)
{
	struct igc_hw *hw = &adapter->hw;
	uint32_t ctrl_ext;

	/* Let firmware take over control of h/w */
	ctrl_ext = rd32(IGC_CTRL_EXT);
	wr32(IGC_CTRL_EXT,
	     ctrl_ext & ~IGC_CTRL_EXT_DRV_LOAD);
}

/**
 * igc_get_hw_control - get control of the h/w from f/w
 * @adapter: address of board private structure
 *
 * igc_get_hw_control sets CTRL_EXT:DRV_LOAD bit.
 * For ASF and Pass Through versions of f/w this means that
 * the driver is loaded.
 */
static void igc_get_hw_control(struct igc_adapter *adapter)
{
	struct igc_hw *hw = &adapter->hw;
	uint32_t ctrl_ext;

	/* Let firmware know the driver has taken over */
	ctrl_ext = rd32(IGC_CTRL_EXT);
	wr32(IGC_CTRL_EXT,
	     ctrl_ext | IGC_CTRL_EXT_DRV_LOAD);
}

static void igc_unmap_tx_buffer(struct igc_tx_buffer *buf)
{
	atemsys_unmap_mem(buf->data, buf->len);
	buf->len = 0;
}

/**
 * igc_clean_tx_ring - Free Tx Buffers
 * @tx_ring: ring to be cleaned
 */
static void igc_clean_tx_ring(struct igc_adapter *adapter)
{
	struct igc_ring *tx_ring = &adapter->tx_ring;
	uint16_t i = tx_ring->next_to_clean;
	struct igc_tx_buffer *tx_buffer = &tx_ring->tx_buffer_info[i];

	while (i != tx_ring->next_to_use) {
		union igc_adv_tx_desc *eop_desc, *tx_desc;

		igc_unmap_tx_buffer(tx_buffer);

		/* check for eop_desc to determine the end of the packet */
		eop_desc = tx_buffer->next_to_watch;
		tx_desc = IGC_TX_DESC(tx_ring, i);

		/* unmap remaining buffers */
		while (tx_desc != eop_desc) {
			tx_buffer++;
			tx_desc++;
			i++;
			if (unlikely(i == tx_ring->count)) {
				i = 0;
				tx_buffer = tx_ring->tx_buffer_info;
				tx_desc = IGC_TX_DESC(tx_ring, 0);
			}

			/* unmap any remaining paged data */
			if (tx_buffer->len)
				igc_unmap_tx_buffer(tx_buffer);
		}

		tx_buffer->next_to_watch = NULL;

		/* move us one more past the eop_desc for start of next pkt */
		tx_buffer++;
		i++;
		if (unlikely(i == tx_ring->count)) {
			i = 0;
			tx_buffer = tx_ring->tx_buffer_info;
		}
	}

	/* Zero out the buffer ring */
	memset(tx_ring->tx_buffer_info, 0,
	       sizeof(*tx_ring->tx_buffer_info) * tx_ring->count);

	/* Zero out the descriptor ring */
	memset(tx_ring->desc, 0, tx_ring->size);

	/* reset next_to_use and next_to_clean */
	tx_ring->next_to_use = 0;
	tx_ring->next_to_clean = 0;
}

void igc_disable_tx_ring(struct igc_adapter *adapter)
{
	struct igc_hw *hw = &adapter->hw;

	uint32_t txdctl = rd32(IGC_TXDCTL);
	txdctl &= ~IGC_TXDCTL_QUEUE_ENABLE;
	txdctl |= IGC_TXDCTL_SWFLUSH;
	wr32(IGC_TXDCTL, txdctl);
}

/**
 * igc_free_tx_resources - Free Tx Resources per Queue
 * @tx_ring: Tx descriptor ring for a specific queue
 *
 * Free all transmit software resources
 */
void igc_free_tx_resources(struct igc_adapter *adapter)
{
	struct igc_ring *tx_ring = &adapter->tx_ring;

	igc_disable_tx_ring(adapter);
	igc_clean_tx_ring(adapter);

	free(tx_ring->tx_buffer_info);
	tx_ring->tx_buffer_info = NULL;

	/* if not set, then don't free */
	if (!tx_ring->desc)
		return;

	atemsys_unmap_mem(tx_ring->desc, tx_ring->size);

	tx_ring->desc = NULL;
}

/**
 * igc_setup_tx_resources - allocate Tx resources (Descriptors)
 * @tx_ring: tx descriptor ring (for a specific queue) to setup
 *
 * Return 0 on success, negative on failure
 */
int igc_setup_tx_resources(struct igc_adapter *adapter)
{
	struct igc_ring *tx_ring = &adapter->tx_ring;
	int size = sizeof(struct igc_tx_buffer) * tx_ring->count;

	tx_ring->tx_buffer_info = malloc(size);
	if (!tx_ring->tx_buffer_info)
		goto err;

	memset(tx_ring->tx_buffer_info, 0, size);

	tx_ring->size = tx_ring->count * sizeof(union igc_adv_tx_desc);
	tx_ring->desc = atemsys_map_dma(adapter->fd, tx_ring->size);
    
	if (!tx_ring->desc)
		goto err;

	tx_ring->dma  = atemsys_get_dma_addr(tx_ring->desc);

	tx_ring->next_to_use = 0;
	tx_ring->next_to_clean = 0;

	return 0;

err:
	free(tx_ring->tx_buffer_info);
	printf("igc driver: error: Unable to allocate memory for Tx descriptor ring\n");
	return -1;
}

/**
 * igc_clean_rx_ring - Free Rx Buffers per Queue
 * @ring: ring to free buffers from
 */
static void igc_clean_rx_ring(struct igc_adapter *adapter)
{
	struct igc_ring *rx_ring = &adapter->rx_ring;
	uint16_t i = rx_ring->next_to_clean;

	while (i != rx_ring->next_to_alloc) {
		struct igc_rx_buffer *buffer_info = &rx_ring->rx_buffer_info[i];

		atemsys_unmap_mem(buffer_info->page, PAGE_SIZE);

		i++;
		if (i == rx_ring->count)
			i = 0;
	}

	rx_ring->next_to_alloc = 0;
	rx_ring->next_to_clean = 0;
	rx_ring->next_to_use = 0;
}

/**
 * igc_free_rx_resources - Free Rx Resources
 * @rx_ring: ring to clean the resources from
 *
 * Free all receive software resources
 */
void igc_free_rx_resources(struct igc_adapter *adapter)
{
	struct igc_ring *rx_ring = &adapter->rx_ring;
	igc_clean_rx_ring(adapter);

	free(rx_ring->rx_buffer_info);
	rx_ring->rx_buffer_info = NULL;

	/* if not set, then don't free */
	if (!rx_ring->desc)
		return;

	atemsys_unmap_mem(rx_ring->desc, rx_ring->size);

	rx_ring->desc = NULL;
}

/**
 * igc_setup_rx_resources - allocate Rx resources (Descriptors)
 *
 * Returns 0 on success, negative on failure
 */
int igc_setup_rx_resources(struct igc_adapter *adapter)
{
	struct igc_ring *rx_ring = &adapter->rx_ring;
	int size = sizeof(struct igc_rx_buffer) * rx_ring->count;

	rx_ring->rx_buffer_info = malloc(size);
	if (!rx_ring->rx_buffer_info)
		goto err;
	memset(rx_ring->rx_buffer_info, 0, size);

	rx_ring->size = sizeof(union igc_adv_rx_desc) * rx_ring->count;
	rx_ring->desc = atemsys_map_dma(adapter->fd, rx_ring->size);

	if (!rx_ring->desc)
		goto err;

	rx_ring->dma  = atemsys_get_dma_addr(rx_ring->desc);

	rx_ring->next_to_alloc = 0;
	rx_ring->next_to_clean = 0;
	rx_ring->next_to_use = 0;

	return 0;

err:
	free(rx_ring->rx_buffer_info);
	rx_ring->rx_buffer_info = NULL;
	printf("igc driver: error: Unable to allocate memory for Rx descriptor ring\n");
	return -1;
}

/**
 * igc_configure_rx_ring - Configure a receive ring after Reset
 * @adapter: board private structure
 * @ring: receive ring to be configured
 *
 * Configure the Rx unit of the MAC after a reset.
 */
static void igc_configure_rx_ring(struct igc_adapter *adapter)
{
	struct igc_hw *hw = &adapter->hw;
	struct igc_ring *rx_ring = &adapter->rx_ring;
	union igc_adv_rx_desc *rx_desc;
	uint32_t srrctl = 0, rxdctl = 0;
	uint64_t rdba = rx_ring->dma;
	uint32_t buf_size;

	/* disable the queue */
	wr32(IGC_RXDCTL, 0);

	/* Set DMA base address registers */
	wr32(IGC_RDBAL,
	     rdba & 0x00000000ffffffffULL);
	wr32(IGC_RDBAH, rdba >> 32);
	wr32(IGC_RDLEN,
	     rx_ring->count * sizeof(union igc_adv_rx_desc));

	/* initialize head and tail */
	rx_ring->tail = adapter->io_addr + IGC_RDT;
	wr32(IGC_RDH, 0);
	wr32(IGC_RDT, 0);

	/* reset next-to- use/clean to place SW in sync with hardware */
	rx_ring->next_to_clean = 0;
	rx_ring->next_to_use = 0;

	buf_size = IGC_RXBUFFER_2048;

	srrctl = rd32(IGC_SRRCTL);
	srrctl &= ~(IGC_SRRCTL_BSIZEPKT_MASK | IGC_SRRCTL_BSIZEHDR_MASK |
		    IGC_SRRCTL_DESCTYPE_MASK);
	srrctl |= IGC_SRRCTL_BSIZEHDR(IGC_RX_HDR_LEN);
	srrctl |= IGC_SRRCTL_BSIZEPKT(buf_size);
	srrctl |= IGC_SRRCTL_DESCTYPE_ADV_ONEBUF;

	wr32(IGC_SRRCTL, srrctl);

	rxdctl |= IGC_RX_PTHRESH;
	rxdctl |= IGC_RX_HTHRESH << 8;
	rxdctl |= IGC_RX_WTHRESH << 16;

	/* initialize rx_buffer_info */
	memset(rx_ring->rx_buffer_info, 0,
	       sizeof(struct igc_rx_buffer) * rx_ring->count);

	/* initialize Rx descriptor 0 */
	rx_desc = IGC_RX_DESC(rx_ring, 0);
	rx_desc->wb.upper.length = 0;

	/* enable receive descriptor fetching */
	rxdctl |= IGC_RXDCTL_QUEUE_ENABLE;

	wr32(IGC_RXDCTL, rxdctl);
}

/**
 * igc_configure_tx_ring - Configure transmit ring after Reset
 * @adapter: board private structure
 * @ring: tx ring to configure
 *
 * Configure a transmit ring after a reset.
 */
static void igc_configure_tx_ring(struct igc_adapter *adapter)
{
	struct igc_hw *hw = &adapter->hw;
	struct igc_ring *tx_ring = &adapter->tx_ring;
	uint64_t tdba = tx_ring->dma;
	uint32_t txdctl = 0;

	/* disable the queue */
	wr32(IGC_TXDCTL, 0);
	wrfl();

	wr32(IGC_TDLEN,
	     tx_ring->count * sizeof(union igc_adv_tx_desc));
	wr32(IGC_TDBAL,
	     tdba & 0x00000000ffffffffULL);
	wr32(IGC_TDBAH, tdba >> 32);

	tx_ring->tail = adapter->io_addr + IGC_TDT;
	wr32(IGC_TDH, 0);
	wr32(IGC_TDT, 0);

	txdctl |= IGC_TX_PTHRESH;
	txdctl |= IGC_TX_HTHRESH << 8;
	txdctl |= IGC_TX_WTHRESH << 16;

	txdctl |= IGC_TXDCTL_QUEUE_ENABLE;
	wr32(IGC_TXDCTL, txdctl);
}

/**
 * igc_setup_rctl - configure the receive control registers
 * @adapter: Board private structure
 */
static void igc_setup_rctl(struct igc_adapter *adapter)
{
	struct igc_hw *hw = &adapter->hw;
	uint32_t rctl;

	rctl = rd32(IGC_RCTL);

	rctl &= ~(3 << IGC_RCTL_MO_SHIFT);
	rctl &= ~(IGC_RCTL_LBM_TCVR | IGC_RCTL_LBM_MAC);

	rctl |= IGC_RCTL_EN | IGC_RCTL_BAM | IGC_RCTL_RDMTS_HALF |
		(hw->mac.mc_filter_type << IGC_RCTL_MO_SHIFT);

	/* enable stripping of CRC. Newer features require
	 * that the HW strips the CRC.
	 */
	rctl |= IGC_RCTL_SECRC;

	/* disable store bad packets and clear size bits. */
	rctl &= ~(IGC_RCTL_SBP | IGC_RCTL_SZ_256);

	/* enable LPE to allow for reception of jumbo frames */
	rctl |= IGC_RCTL_LPE;

	/* disable queue 0 to prevent tail write w/o re-config */
	wr32(IGC_RXDCTL, 0);

	wr32(IGC_RCTL, rctl);
}

/**
 * igc_setup_tctl - configure the transmit control registers
 * @adapter: Board private structure
 */
static void igc_setup_tctl(struct igc_adapter *adapter)
{
	struct igc_hw *hw = &adapter->hw;
	uint32_t tctl;

	/* disable queue 0 which icould be enabled by default */
	wr32(IGC_TXDCTL, 0);

	/* Program the Transmit Control Register */
	tctl = rd32(IGC_TCTL);
	tctl &= ~IGC_TCTL_CT;
	tctl |= IGC_TCTL_PSP | IGC_TCTL_RTLC |
		(IGC_COLLISION_THRESHOLD << IGC_CT_SHIFT);

	/* Enable transmits */
	tctl |= IGC_TCTL_EN;

	wr32(IGC_TCTL, tctl);
}

void igc_xmit_frame(struct igc_adapter *adapter, uint8_t* data, int len)
{
	struct igc_hw *hw = &adapter->hw;
	struct igc_ring *tx_ring = &adapter->tx_ring;
	struct igc_tx_buffer *first;

	static int iii = 0;
	if (iii < 12)
	{
		printf("Tx packet:\n");
		print_debug_packet(data, len);
		iii++;
	}

	/* record the location of the first descriptor for this packet */
	first = &tx_ring->tx_buffer_info[tx_ring->next_to_use];

	struct igc_tx_buffer *tx_buffer;
	union igc_adv_tx_desc *tx_desc;
	uint16_t i = tx_ring->next_to_use;
	void* mem_ptr;
	uint64_t dma;
	uint32_t cmd_type;

	/* set type for advanced descriptor with frame checksum insertion */
	cmd_type = IGC_ADVTXD_DTYP_DATA | IGC_ADVTXD_DCMD_DEXT | IGC_ADVTXD_DCMD_IFCS;
	tx_desc = IGC_TX_DESC(tx_ring, i);
	tx_desc->read.olinfo_status = cpu_to_le32(len << IGC_ADVTXD_PAYLEN_SHIFT);

	// struct timeval  bgn, end;
	// static int iii = 0;
	// if (iii < 1000)
	// {
	// 	gettimeofday(&bgn, NULL);
	// }
	mem_ptr = atemsys_map_dma(adapter->fd, len);
	// if (iii < 1000)
	// {
	// 	gettimeofday(&end, NULL);
	// 	printf ("Total time = %li us\n", (end.tv_usec - bgn.tv_usec) + 1000000 * (end.tv_sec - bgn.tv_sec));
	// 	iii++;
	// }

	tx_buffer = first;

	if (!mem_ptr)
		goto dma_error;

	dma = atemsys_get_dma_addr(mem_ptr);

	/* record length, and DMA address */
	tx_buffer->data = mem_ptr;
	tx_buffer->len = len;
	tx_buffer->dma = dma;
	memcpy(mem_ptr, data, len);

	tx_desc->read.buffer_addr = cpu_to_le64(dma);

	/* write last descriptor with RS and EOP bits */
	cmd_type |= first->len | IGC_TXD_DCMD;
	tx_desc->read.cmd_type_len = cpu_to_le32(cmd_type);

	/* set next_to_watch value indicating a packet is present */
	first->next_to_watch = tx_desc;

	i++;
	if (i == tx_ring->count)
		i = 0;

	tx_ring->next_to_use = i;
	wr32(IGC_TDT, i);

	goto exit;
dma_error:
	printf("igc driver: error: TX DMA map failed\n");
	tx_buffer = &tx_ring->tx_buffer_info[i];

	/* clear dma mappings for failed tx_buffer_info map */
	while (tx_buffer != first) {
		if (tx_buffer->len)
			igc_unmap_tx_buffer(tx_buffer);

		if (i-- == 0)
			i += tx_ring->count;
		tx_buffer = &tx_ring->tx_buffer_info[i];
	}

	if (tx_buffer->len)
		igc_unmap_tx_buffer(tx_buffer);

	tx_ring->next_to_use = i;

exit:
}

static void igc_disable_vlan(struct igc_adapter *adapter)
{
	struct igc_hw *hw = &adapter->hw;
	uint32_t ctrl = rd32(IGC_CTRL);
	ctrl &= ~IGC_CTRL_VME;      // disable VLAN tag insert/strip
	wr32(IGC_CTRL, ctrl);
}

/**
 * igc_reuse_rx_page - page flip buffer and store it back on the ring
 * @rx_ring: rx descriptor ring to store buffers on
 * @old_buff: donor buffer to have page reused
 *
 * Synchronizes page for reuse by the adapter
 */
static void igc_reuse_rx_page(struct igc_ring *rx_ring,
			      struct igc_rx_buffer *old_buff)
{
	uint16_t nta = rx_ring->next_to_alloc;
	struct igc_rx_buffer *new_buff; 

	new_buff = &rx_ring->rx_buffer_info[nta];

	/* update, and store next to alloc */
	nta++;
	rx_ring->next_to_alloc = (nta < rx_ring->count) ? nta : 0;

	/* Transfer page from old buffer to new buffer.
	 * Move each member individually to avoid possible store
	 * forwarding stalls.
	 */
	new_buff->dma		= old_buff->dma;
	new_buff->page		= old_buff->page;
}

/**
 * igc_is_non_eop - process handling of non-EOP buffers
 * @rx_ring: Rx ring being processed
 * @rx_desc: Rx descriptor for current buffer
 *
 * This function updates next to clean.  If the buffer is an EOP buffer
 * this function exits returning false, otherwise it will place the
 * sk_buff in the next buffer to be chained and return true indicating
 * that this is in fact a non-EOP buffer.
 */
static bool igc_is_non_eop(struct igc_ring *rx_ring,
			   union igc_adv_rx_desc *rx_desc)
{
	uint32_t ntc = rx_ring->next_to_clean + 1;

	/* fetch, update, and store next to clean */
	ntc = (ntc < rx_ring->count) ? ntc : 0;
	rx_ring->next_to_clean = ntc;

	if (likely(igc_test_staterr(rx_desc, IGC_RXD_STAT_EOP)))
		return false;

	return true;
}

static bool igc_alloc_mapped_page(struct igc_adapter *adapter, struct igc_ring *rx_ring,
				  struct igc_rx_buffer *bi)
{
	void *page = bi->page;
	uint64_t dma;

	/* since we are recycling buffers we should seldom need to alloc */
	if (likely(page))
		return true;

	/* allocate and map dma rx page */
	page = atemsys_map_dma(adapter->fd, PAGE_SIZE);
	if (unlikely(!page)) {
		printf("igc driver: mem allocation failed\n");
		return false;
	}

	dma = atemsys_get_dma_addr(page);

	bi->dma = dma;
	bi->page = page;

	return true;
}

/**
 * igc_alloc_rx_buffers - Replace used receive buffers; packet split
 * @rx_ring: rx descriptor ring
 * @cleaned_count: number of buffers to clean
 */
static void igc_alloc_rx_buffers(struct igc_adapter *adapter, uint16_t cleaned_count)
{
	struct igc_hw *hw = &adapter->hw;
	struct igc_ring *rx_ring = &adapter->rx_ring;
	union igc_adv_rx_desc *rx_desc;
	uint16_t i = rx_ring->next_to_use;
	struct igc_rx_buffer *bi;

	/* nothing to do */
	if (!cleaned_count)
		return;

	rx_desc = IGC_RX_DESC(rx_ring, i);
	bi = &rx_ring->rx_buffer_info[i];
	i -= rx_ring->count;

	do {
		if (!igc_alloc_mapped_page(adapter, rx_ring, bi))
			break;

		/* Refresh the desc even if buffer_addrs didn't change
		 * because each write-back erases this info.
		 */
		rx_desc->read.pkt_addr = cpu_to_le64(bi->dma);

		rx_desc++;
		bi++;
		i++;
		if (unlikely(!i)) {
			rx_desc = IGC_RX_DESC(rx_ring, 0);
			bi = rx_ring->rx_buffer_info;
			i -= rx_ring->count;
		}

		/* clear the length for the next_to_use descriptor */
		rx_desc->wb.upper.length = 0;

		cleaned_count--;
	} while (cleaned_count);

	i += rx_ring->count;

	if (rx_ring->next_to_use != i) {
		/* record the next descriptor to use */
		rx_ring->next_to_use = i;

		/* update next to alloc since we have filled the ring */
		rx_ring->next_to_alloc = i;

		wr32(IGC_RDT, i);
	}
}

static void igc_clean_rx_irq(struct igc_adapter *adapter)
{
	struct igc_ring *rx_ring = &adapter->rx_ring;
	uint16_t cleaned_count = igc_desc_unused(rx_ring);

	while (true) {
		union igc_adv_rx_desc *rx_desc;
		struct igc_rx_buffer *rx_buffer;
		unsigned int size;
		void *pktbuf;

		/* return some buffers to hardware, one at a time is too slow */
		if (cleaned_count >= IGC_RX_BUFFER_WRITE) {
			igc_alloc_rx_buffers(adapter, cleaned_count);
			cleaned_count = 0;
		}

		rx_desc = IGC_RX_DESC(rx_ring, rx_ring->next_to_clean);
		size = le16_to_cpu(rx_desc->wb.upper.length);
		if (!size)
			break;

		rx_buffer = &rx_ring->rx_buffer_info[rx_ring->next_to_clean];

		pktbuf = rx_buffer->page;

		static int iii = 0;
		if (iii < 12)
		{
			printf("Rx packet:\n");
			print_debug_packet(pktbuf, size);
			iii++;
		}

		igc_reuse_rx_page(rx_ring, rx_buffer);
	    rx_buffer->page = NULL;
		cleaned_count++;

		/* fetch next buffer in frame if non-eop */
		if (igc_is_non_eop(rx_ring, rx_desc))
			continue;
	}

	if (cleaned_count)
		igc_alloc_rx_buffers(adapter, cleaned_count);
}

static void igc_clean_tx_irq(struct igc_adapter *adapter)
{
	struct igc_ring *tx_ring = &adapter->tx_ring;
	unsigned int i = tx_ring->next_to_clean;
	struct igc_tx_buffer *tx_buffer;
	union igc_adv_tx_desc *tx_desc;

	if (adapter->state_down)
		return;

	tx_buffer = &tx_ring->tx_buffer_info[i];
	tx_desc = IGC_TX_DESC(tx_ring, i);
	i -= tx_ring->count;

	while (true) {
		union igc_adv_tx_desc *eop_desc = tx_buffer->next_to_watch;

		/* if next_to_watch is not set then there is no work pending */
		if (!eop_desc)
			break;

		/* if DD is not set pending work has not been completed */
		if (!(eop_desc->wb.status & cpu_to_le32(IGC_TXD_STAT_DD)))
			break;

		/* clear next_to_watch to prevent false hangs */
		tx_buffer->next_to_watch = NULL;

		igc_unmap_tx_buffer(tx_buffer);

		/* clear last DMA location and unmap remaining buffers */
		while (tx_desc != eop_desc) {
			tx_buffer++;
			tx_desc++;
			i++;
			if (unlikely(!i)) {
				i -= tx_ring->count;
				tx_buffer = tx_ring->tx_buffer_info;
				tx_desc = IGC_TX_DESC(tx_ring, 0);
			}

			/* unmap any remaining paged data */
			if (tx_buffer->len)
				igc_unmap_tx_buffer(tx_buffer);
		}

		/* move us one more past the eop_desc for start of next pkt */
		tx_buffer++;
		tx_desc++;
		i++;
		if (unlikely(!i)) {
			i -= tx_ring->count;
			tx_buffer = tx_ring->tx_buffer_info;
			tx_desc = IGC_TX_DESC(tx_ring, 0);
		}
	}

	i += tx_ring->count;
	tx_ring->next_to_clean = i;
}

static void igc_set_rx_mode(struct igc_adapter *adapter)
{
	struct igc_hw *hw = &adapter->hw;
	uint32_t rctl = 0, rlpml = 0x670; // max frame size determined impirically

	// use unicast promiscuous and all multicast modes
	// because no dst or src mac hw filters are being used
	rctl |= IGC_RCTL_UPE | IGC_RCTL_MPE;

	/* update state of unicast and multicast */
	rctl |= rd32(IGC_RCTL) & ~(IGC_RCTL_UPE | IGC_RCTL_MPE);
	wr32(IGC_RCTL, rctl);
	wr32(IGC_RLPML, rlpml);
}

static void igc_configure(struct igc_adapter *adapter)
{
	igc_get_hw_control(adapter);
	igc_set_rx_mode(adapter);

	igc_disable_vlan(adapter);

	igc_setup_tctl(adapter);
	igc_setup_rctl(adapter);

	igc_configure_tx_ring(adapter);
	igc_configure_rx_ring(adapter);

	igc_rx_fifo_flush_base(&adapter->hw);

	/* call igc_desc_unused which always leaves
	 * at least 1 descriptor unused to make sure
	 * next_to_use != next_to_clean
	 */
	igc_alloc_rx_buffers(adapter, igc_desc_unused(&adapter->rx_ring));
}

/**
 * igc_write_ivar - configure ivar for given MSI-X vector
 * @hw: pointer to the HW structure
 * @offset: column offset of in IVAR, should be multiple of 8
 *
 * The IVAR table consists of 2 columns,
 * each containing a cause allocation for an Rx and Tx ring, and a
 * variable number of rows depending on the number of queues supported.
 */
static void igc_write_ivar(struct igc_hw *hw, int offset)
{
	uint32_t ivar = rd32(IGC_IVAR0);

	/* clear any bits that are currently set */
	ivar &= ~((uint32_t)0xFF << offset);

	/* write vector and valid bit */
	ivar |= IGC_IVAR_VALID << offset;

	wr32(IGC_IVAR0, ivar);
}

/**
 * igc_irq_enable - Enable default interrupt generation settings
 * @adapter: board private structure
 */
static void igc_irq_enable(struct igc_adapter *adapter)
{
	struct igc_hw *hw = &adapter->hw;

	wr32(IGC_IMS, IMS_ENABLE_MASK | IGC_IMS_DRSTA);
	wr32(IGC_IAM, IMS_ENABLE_MASK | IGC_IMS_DRSTA);
}

/**
 * igc_irq_disable - Mask off interrupt generation on the NIC
 * @adapter: board private structure
 */
static void igc_irq_disable(struct igc_adapter *adapter)
{
	struct igc_hw *hw = &adapter->hw;

	wr32(IGC_IAM, 0);
	wr32(IGC_IMC, ~0);
	wrfl();
}

/**
 * igc_down - Close the interface
 * @adapter: board private structure
 */
void igc_down(struct igc_adapter *adapter)
{
	struct igc_hw *hw = &adapter->hw;
	uint32_t tctl, rctl;

	adapter->state_down = true;

	/* disable receives in the hardware */
	rctl = rd32(IGC_RCTL);
	wr32(IGC_RCTL, rctl & ~IGC_RCTL_EN);
	/* flush and sleep below */

	/* disable transmits in the hardware */
	tctl = rd32(IGC_TCTL);
	tctl &= ~IGC_TCTL_EN;
	wr32(IGC_TCTL, tctl);
	/* flush both disables and wait for them to finish */
	wrfl();
	usleep(20000);

	igc_irq_disable(adapter);

	adapter->link_speed = 0;
	adapter->link_duplex = 0;

	igc_disable_tx_ring(adapter);
	igc_clean_tx_ring(adapter);
	igc_clean_rx_ring(adapter);
}

static void igc_watchdog_task_update_link(struct igc_adapter *adapter)
{
	struct igc_hw *hw = &adapter->hw;
	uint16_t phy_data, retry_count = 20;
	bool link;

	hw->mac.get_link_status = true;
	hw->mac.ops.check_for_link(hw);
	link = !hw->mac.get_link_status;

	if (link) {
		if (!adapter->link) {
			uint32_t ctrl;

			hw->mac.ops.get_speed_and_duplex(hw,
							 &adapter->link_speed,
							 &adapter->link_duplex);

			ctrl = rd32(IGC_CTRL);
			/* Link status message must follow this format */
			printf("igc driver: info NIC Link is Up %d Mbps %s Duplex, Flow Control: %s\n",
				    adapter->link_speed,
				    adapter->link_duplex == FULL_DUPLEX ?
				    "Full" : "Half",
				    (ctrl & IGC_CTRL_TFCE) &&
				    (ctrl & IGC_CTRL_RFCE) ? "RX/TX" :
				    (ctrl & IGC_CTRL_RFCE) ?  "RX" :
				    (ctrl & IGC_CTRL_TFCE) ?  "TX" : "None");

			if (adapter->link_speed != SPEED_1000)
				goto no_wait;

			/* wait for Remote receiver status OK */
retry_read_status:
			if (!igc_read_phy_reg(hw, PHY_1000T_STATUS,
					      &phy_data)) {
				if (!(phy_data & SR_1000T_REMOTE_RX_STATUS) &&
				    retry_count) {
					usleep(100000);
					retry_count--;
					goto retry_read_status;
				} else if (!retry_count) {
					printf("igc driver: error: exceed max 2 second\n");
				}
			} else {
				printf("igc driver: error: read 1000Base-T Status Reg\n");
			}
no_wait:
			adapter->link = true;
		}
	} else {
		if (adapter->link) {
			adapter->link_speed = 0;
			adapter->link_duplex = 0;

			/* Links status message must follow this format */
			printf("igc driver: info: NIC Link is Down\n");
			adapter->link = false;
		}
	}

	/* Cause software interrupt to ensure Rx ring is cleaned */
	wr32(IGC_ICS, IGC_ICS_RXDMT0);
}

void igc_intr_msi(struct igc_adapter *adapter)
{
	struct igc_hw *hw = &adapter->hw;
	/* read ICR disables interrupts using IAM */
	uint32_t icr = rd32(IGC_ICR);

	if (icr & (IGC_ICR_RXSEQ | IGC_ICR_LSC)) {
		printf("Link status change interrupt\n");
		if (!adapter->state_down)
			igc_watchdog_task_update_link(adapter);
	}

	// struct timeval  bgn, end;
	// static int iii = 0;
	// if (iii < 1000)
	// {
	// 	gettimeofday(&bgn, NULL);
	// }
	igc_clean_tx_irq(adapter);
	igc_clean_rx_irq(adapter);
	// if (iii < 1000)
	// {
	// 	gettimeofday(&end, NULL);
	// 	printf ("Total time = %li us\n", (end.tv_usec - bgn.tv_usec) + 1000000 * (end.tv_sec - bgn.tv_sec));
	// 	iii++;
	// }


	if (!adapter->state_down) {
		igc_irq_enable(adapter);
	}
}

int igc_open(struct igc_adapter *adapter)
{
	struct igc_hw *hw = &adapter->hw;
	int err = 0;

	err = igc_setup_tx_resources(adapter);
	if (err) {
		printf("Error during Tx queue setup\n");
		igc_free_tx_resources(adapter);
		goto err_setup_tx;
	}

	err = igc_setup_rx_resources(adapter);
	if (err) {
		printf("Error during Rx queue setup\n");
		igc_free_rx_resources(adapter);
		goto err_setup_rx;
	}

	igc_power_up_link(adapter);
	igc_configure(adapter);

	igc_write_ivar(hw, 0);  // rx_ring
	igc_write_ivar(hw, 8);  // tx_ring

	adapter->state_down = false;

	/* Clear any pending interrupts. */
	rd32(IGC_ICR);
	wr32(IGC_EITR(0), (IGC_ITR_USECS & IGC_QVECTOR_MASK) | IGC_EITR_CNT_IGNR); // set interrupt throttle rate
	igc_irq_enable(adapter);

	return IGC_SUCCESS;

err_setup_rx:
	igc_free_tx_resources(adapter);
err_setup_tx:
	igc_reset(adapter, false);

	return err;
}

int igc_close(struct igc_adapter *adapter)
{
	igc_down(adapter);
	igc_release_hw_control(adapter);
	igc_free_tx_resources(adapter);
	igc_free_rx_resources(adapter);
	return 0;
}

int igc_probe(struct igc_adapter *adapter, int fd, uint8_t* io_addr)
{
	int err;
	struct igc_hw *hw = &adapter->hw;

	adapter->fd      = fd;
	adapter->io_addr = io_addr;
	hw->hw_addr      = io_addr;

	/* Copy the default MAC and PHY function pointers */
	memcpy(&hw->mac.ops, igc_base_info.mac_ops, sizeof(hw->mac.ops));
	memcpy(&hw->phy.ops, igc_base_info.phy_ops, sizeof(hw->phy.ops));

	/* Initialize skew-specific constants */
	err = igc_base_info.get_invariants(hw);
	if (err)
		goto err_sw_init;

	//pci_enable_msi(adapter->pdev);

	// init the rings
	adapter->tx_ring.count = IGC_DEFAULT_TXD;
	adapter->rx_ring.count = IGC_DEFAULT_RXD;

	/* Explicitly disable IRQ since the NIC can be in any state. */
	igc_irq_disable(adapter);

	adapter->state_down = true;

	/* before reading the NVM, reset the controller to put the device in a
	 * known good starting state
	 */
	hw->mac.ops.reset_hw(hw);

	if (igc_get_flash_presence_i225(hw)) {
		if (hw->nvm.ops.validate(hw) < 0) {
			printf("igc driver: The NVM Checksum Is Not Valid\n");
			err = -1;
			goto err_eeprom;
		}
	}

	/* copy the MAC address out of the NVM */
	if (hw->mac.ops.read_mac_addr(hw))
		printf("igc driver: NVM Read Error\n");

	/* configure RXPBSIZE and TXPBSIZE */
	wr32(IGC_RXPBS, I225_RXPBSIZE_DEFAULT);
	wr32(IGC_TXPBS, I225_TXPBSIZE_DEFAULT);

	hw->mac.autoneg = true;
	hw->phy.autoneg_advertised = 0xaf;

	igc_reset(adapter, true);
	igc_get_hw_control(adapter);

	return 0;

err_eeprom:
	if (!igc_check_reset_block(hw))
		igc_reset_phy(hw);
err_sw_init:
	//pci_disable_msi(adapter->pdev);
	return err;
}

void igc_remove(struct igc_adapter *adapter)
{
	adapter->state_down = true;

	/* Release control of h/w to f/w.  If f/w is AMT enabled, this
	 * would have already happened in close and is redundant.
	 */
	igc_release_hw_control(adapter);
	//pci_disable_msi(adapter->pdev);
}

