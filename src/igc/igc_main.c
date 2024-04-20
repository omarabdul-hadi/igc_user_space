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
#include <sys/mman.h>

#include "../atemsys_main.h"
#include "igc.h"
#include "igc_log.h"

int global_log_level = IGC_LOG_ERR;

void igc_logger(int msg_log_level, char * msg, ...)
{
    va_list argp;
    va_start(argp, msg);

    if(msg_log_level >= global_log_level )
    {
		if (msg_log_level == IGC_LOG_INF)
	    	printf("igc driver INF: ");
		else if (msg_log_level == IGC_LOG_ERR)
	    	printf("igc driver ERR: ");
		else
	    	printf("igc driver UNSUPPORTED LOG LEVEL: ");

       vprintf(msg, argp);
    }
    va_end(argp);
}

void igc_reset(struct igc_adapter *adapter, bool power_down_phy)
{
	struct igc_hw *hw = &adapter->hw;

	hw->mac.ops.reset_hw(hw);

	if (hw->mac.ops.init_hw(hw))
		igc_logger(IGC_LOG_ERR, "hardware initialization\n");

	if (power_down_phy)
		igc_power_down_phy_copper_base(&adapter->hw);

	igc_get_phy_info(hw);
}

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

static void igc_clean_tx_ring(struct igc_adapter *adapter)
{
	struct igc_ring *tx_ring = &adapter->tx_ring;
	struct igc_tx_buffer *tx_buffer;

	for (int i = 0; i < tx_ring->count; i++) {
		tx_buffer = &tx_ring->tx_buffer_info[i];
		atemsys_unmap_mem(tx_buffer->data, TX_BUFF_SIZE);
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

int igc_setup_tx_resources(struct igc_adapter *adapter)
{
	struct igc_ring *tx_ring = &adapter->tx_ring;
	int size = sizeof(struct igc_tx_buffer) * tx_ring->count;

	tx_ring->tx_buffer_info = malloc(size);
	if (!tx_ring->tx_buffer_info)
		goto err;

	memset(tx_ring->tx_buffer_info, 0, size);

	tx_ring->size = tx_ring->count * sizeof(union igc_adv_tx_desc);
	tx_ring->desc = atemsys_map_dma(adapter->fd, tx_ring->size, PROT_READ | PROT_WRITE);
    
	if (!tx_ring->desc)
		goto err;

	tx_ring->dma  = atemsys_get_dma_addr(tx_ring->desc);

	tx_ring->next_to_use = 0;
	tx_ring->next_to_clean = 0;


	for (int i = 0; i < tx_ring->count; i++) {
		struct igc_tx_buffer *tx_buffer = &tx_ring->tx_buffer_info[i];
		tx_buffer->data = atemsys_map_dma(adapter->fd, TX_BUFF_SIZE, PROT_WRITE);
		tx_buffer->dma  = atemsys_get_dma_addr(tx_buffer->data);
	}

	return 0;

err:
	free(tx_ring->tx_buffer_info);
	igc_logger(IGC_LOG_ERR, "Unable to allocate memory for Tx descriptor ring\n");
	return -1;
}

static void igc_clean_rx_ring(struct igc_adapter *adapter)
{
	struct igc_ring *rx_ring = &adapter->rx_ring;
	uint16_t i = rx_ring->next_to_clean;

	while (i != rx_ring->next_to_alloc) {
		struct igc_rx_buffer *buffer_info = &rx_ring->rx_buffer_info[i];

		atemsys_unmap_mem(buffer_info->data, RX_BUFF_SIZE);

		i++;
		if (i == rx_ring->count)
			i = 0;
	}

	rx_ring->next_to_alloc = 0;
	rx_ring->next_to_clean = 0;
	rx_ring->next_to_use = 0;
}

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

int igc_setup_rx_resources(struct igc_adapter *adapter)
{
	struct igc_ring *rx_ring = &adapter->rx_ring;
	int size = sizeof(struct igc_rx_buffer) * rx_ring->count;

	rx_ring->rx_buffer_info = malloc(size);
	if (!rx_ring->rx_buffer_info)
		goto err;
	memset(rx_ring->rx_buffer_info, 0, size);

	rx_ring->size = sizeof(union igc_adv_rx_desc) * rx_ring->count;
	rx_ring->desc = atemsys_map_dma(adapter->fd, rx_ring->size, PROT_READ | PROT_WRITE);

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
	igc_logger(IGC_LOG_ERR, "Unable to allocate memory for Rx descriptor ring\n");
	return -1;
}

static void igc_configure_rx_ring(struct igc_adapter *adapter)
{
	struct igc_hw *hw = &adapter->hw;
	struct igc_ring *rx_ring = &adapter->rx_ring;
	union igc_adv_rx_desc *rx_desc;
	uint32_t srrctl = 0, rxdctl = 0;
	uint64_t rdba = rx_ring->dma;

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

	srrctl = rd32(IGC_SRRCTL);
	srrctl &= ~(IGC_SRRCTL_BSIZEPKT_MASK | IGC_SRRCTL_BSIZEHDR_MASK |
		    IGC_SRRCTL_DESCTYPE_MASK);
	srrctl |= IGC_SRRCTL_BSIZEHDR(IGC_RX_HDR_LEN);
	srrctl |= IGC_SRRCTL_BSIZEPKT(RX_BUFF_SIZE);
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

void igc_send_frame(struct igc_adapter *adapter, uint8_t* data, int len)
{
	struct igc_hw *hw = &adapter->hw;
	struct igc_ring *tx_ring = &adapter->tx_ring;
	uint16_t i = tx_ring->next_to_use;
	struct igc_tx_buffer *tx_buffer = &tx_ring->tx_buffer_info[i];
	union igc_adv_tx_desc *tx_desc = IGC_TX_DESC(tx_ring, i);
	uint32_t cmd_type = len                  | /* write last descriptor with RS and EOP bits */
	                    IGC_ADVTXD_DCMD_EOP  |
						IGC_ADVTXD_DCMD_RS   |
	                    IGC_ADVTXD_DTYP_DATA | /* set type for advanced descriptor with frame checksum insertion */
	                    IGC_ADVTXD_DCMD_DEXT | 
			            IGC_ADVTXD_DCMD_IFCS;

	tx_desc->read.olinfo_status = cpu_to_le32(len << IGC_ADVTXD_PAYLEN_SHIFT);

	/* record length, and DMA address */
	void* mem_ptr = tx_buffer->data;
	uint64_t dma = tx_buffer->dma;
	memcpy(mem_ptr, data, len);

	tx_desc->read.buffer_addr = cpu_to_le64(dma);
	tx_desc->read.cmd_type_len = cpu_to_le32(cmd_type);

	/* set next_to_watch value indicating a packet is present */
	tx_buffer->next_to_watch = tx_desc;

	i++;
	if (i == tx_ring->count)
		i = 0;

	tx_ring->next_to_use = i;
	wr32(IGC_TDT, i);
}

static void igc_disable_vlan(struct igc_adapter *adapter)
{
	struct igc_hw *hw = &adapter->hw;
	uint32_t ctrl = rd32(IGC_CTRL);
	ctrl &= ~IGC_CTRL_VME;      // disable VLAN tag insert/strip
	wr32(IGC_CTRL, ctrl);
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
		/* since we are recycling buffers we should seldom need to alloc */
		if (unlikely(!bi->data)) {
			bi->data = atemsys_map_dma(adapter->fd, RX_BUFF_SIZE, PROT_READ);
			if (unlikely(!bi->data)) {
				igc_logger(IGC_LOG_INF, " mem allocation failed\n");
				break;
			}
			bi->dma = atemsys_get_dma_addr(bi->data);
		}

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

int igc_clean_rx_irq(struct igc_adapter *adapter, uint8_t* receive_pkt)
{
	struct igc_ring *rx_ring = &adapter->rx_ring;
	uint16_t cleaned_count = igc_desc_unused(rx_ring);
	union igc_adv_rx_desc *rx_desc = IGC_RX_DESC(rx_ring, rx_ring->next_to_clean);
	struct igc_rx_buffer *rx_buffer = &rx_ring->rx_buffer_info[rx_ring->next_to_clean];
	int len = le16_to_cpu(rx_desc->wb.upper.length);

	while (!len) {
		usleep(10);
		len = le16_to_cpu(rx_desc->wb.upper.length);
	}
	memcpy(receive_pkt, rx_buffer->data, len);

	// reuse old buffer
	struct igc_rx_buffer *new_buff = &rx_ring->rx_buffer_info[rx_ring->next_to_alloc];
	new_buff->dma  = rx_buffer->dma;
	new_buff->data = rx_buffer->data;
	rx_buffer->data = NULL;

	rx_ring->next_to_alloc = (rx_ring->next_to_alloc + 1) % rx_ring->count;
	rx_ring->next_to_clean = (rx_ring->next_to_clean + 1) % rx_ring->count;

	cleaned_count++;

	if (cleaned_count) {
		igc_alloc_rx_buffers(adapter, cleaned_count);
	}

	return len;
}

void igc_clean_tx_irq(struct igc_adapter *adapter)
{
	struct igc_ring *tx_ring = &adapter->tx_ring;
	unsigned int ntc = tx_ring->next_to_clean;
	struct igc_tx_buffer *tx_buffer;
	union igc_adv_tx_desc *eop_desc;

	if (adapter->state_down)
		return;

	while (true) {
		tx_buffer = &tx_ring->tx_buffer_info[ntc];
		eop_desc = tx_buffer->next_to_watch;

		/* if next_to_watch is set then there is work pending */
		/* if DD is set, then pending work has been completed */
		if (eop_desc && (eop_desc->wb.status & cpu_to_le32(IGC_TXD_STAT_DD))) {
			tx_buffer->next_to_watch = NULL;
			ntc = (ntc + 1) % tx_ring->count;
		}
		else {
			break;
		}
	}
	tx_ring->next_to_clean = ntc;
}

static void igc_set_rx_mode(struct igc_adapter *adapter)
{
	struct igc_hw *hw = &adapter->hw;
	uint32_t rctl = 0, rlpml = 0x670; // max frame size determined impirically while running igc driver in kernel space

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
 * igc_irq_enable - Enable default interrupt generation settings
 * @adapter: board private structure
 */
static void igc_irq_enable(struct igc_adapter *adapter)
{
	struct igc_hw *hw = &adapter->hw;

	wr32(IGC_IMS, IMS_ENABLE_MASK);
	wr32(IGC_IAM, IMS_ENABLE_MASK);
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

static void igc_update_link(struct igc_adapter *adapter)
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
			igc_logger(IGC_LOG_INF, "NIC Link is Up %d Mbps %s Duplex, Flow Control: %s\n",
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
					igc_logger(IGC_LOG_ERR, "exceed max 2 second\n");
				}
			} else {
				igc_logger(IGC_LOG_ERR, "read 1000Base-T Status Reg\n");
			}
no_wait:
			adapter->link = true;
		}
	} else {
		if (adapter->link) {
			adapter->link_speed = 0;
			adapter->link_duplex = 0;

			/* Links status message must follow this format */
			igc_logger(IGC_LOG_INF, "NIC Link is Down\n");
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

	if (icr & IGC_ICR_LSC)
		igc_update_link(adapter);

	igc_irq_enable(adapter);
}

int igc_open(struct igc_adapter *adapter)
{
	struct igc_hw *hw = &adapter->hw;
	int err = 0;

	err = igc_setup_tx_resources(adapter);
	if (err) {
		igc_logger(IGC_LOG_ERR, "during Tx queue setup\n");
		igc_free_tx_resources(adapter);
		goto err_setup_tx;
	}

	err = igc_setup_rx_resources(adapter);
	if (err) {
		igc_logger(IGC_LOG_ERR, "during Rx queue setup\n");
		igc_free_rx_resources(adapter);
		goto err_setup_rx;
	}

	igc_power_up_link(adapter);
	igc_configure(adapter);

	adapter->state_down = false;

	/* Clear any pending interrupts. */
	rd32(IGC_ICR);
	// optimization: default interrupt throttle rate setting seems to yield the same results
	//wr32(IGC_EITR(0), (IGC_ITR_USECS & IGC_QVECTOR_MASK) | IGC_EITR_CNT_IGNR); // set interrupt throttle rate
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

uint32_t igc_rd32(struct igc_hw *hw, uint32_t reg) {
	if (hw->hw_addr) {
		return ( *((uint32_t*) (hw->hw_addr + reg)) );
	}
	else {
		igc_logger(IGC_LOG_ERR, "hardware io data pointer is null\n");
		return 0;
	}
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
			igc_logger(IGC_LOG_ERR, "The NVM Checksum Is Not Valid\n");
			err = -1;
			goto err_eeprom;
		}
	}

	/* copy the MAC address out of the NVM */
	if (hw->mac.ops.read_mac_addr(hw))
		igc_logger(IGC_LOG_ERR, "NVM Read\n");

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
	return err;
}

void igc_remove(struct igc_adapter *adapter)
{
	adapter->state_down = true;

	/* Release control of h/w to f/w.  If f/w is AMT enabled, this
	 * would have already happened in close and is redundant.
	 */
	igc_release_hw_control(adapter);
}

