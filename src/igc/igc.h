/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c)  2018 Intel Corporation */

#ifndef _IGC_H_
#define _IGC_H_

// system is little endian, so no conversion needed
#define cpu_to_le32
#define cpu_to_le64
#define le16_to_cpu
#define PAGE_SIZE     4096
#define BITS_PER_LONG   64

#define GENMASK(h, l) \
	(((~ 0UL) - ( 1UL << (l)) + 1) & \
	 (~ 0UL >> (BITS_PER_LONG - 1 - (h))))

#define BIT(nr)	 (1UL << (nr))

#define __bf_shf(x) (__builtin_ffsll(x) - 1)
#define FIELD_PREP(_mask, _val) (((typeof(_mask))(_val) << __bf_shf(_mask)) & (_mask))

#define likely(x)       __builtin_expect(!!(x), 1)
#define unlikely(x)     __builtin_expect(!!(x), 0)

#include "igc_hw.h"
#include "./../atemsys/atemsys.h"

/* Transmit and receive queues */
#define MAX_Q_VECTORS			8

struct igc_ring_container {
	struct igc_ring *ring;          /* pointer to linked list of rings */
};

struct igc_ring {
	struct igc_q_vector *q_vector;  /* backlink to q_vector */
	union {                         /* array of buffer info structs */
		struct igc_tx_buffer *tx_buffer_info;
		struct igc_rx_buffer *rx_buffer_info;
	};
	void *desc;                     /* descriptor ring memory */
	void *tail;             /* pointer to ring tail register */
	uint64_t dma;                 /* phys address of the ring */
	unsigned int size;              /* length of desc. ring in bytes */
	uint16_t count;                      /* number of desc. in the ring */

	/* everything past this point are written often */
	uint16_t next_to_clean;
	uint16_t next_to_use;
	uint16_t next_to_alloc;
};

/* Board specific private data structure */
struct igc_adapter {
	int fd;

	bool state_down;

	struct igc_ring *tx_ring;
	struct igc_ring *rx_ring;

	bool link;
	uint16_t link_speed;
	uint16_t link_duplex;

	uint8_t *io_addr;
	struct igc_hw hw;

	struct igc_q_vector *q_vector[MAX_Q_VECTORS];
};

int igc_probe(struct igc_adapter *adapter, int fd, uint8_t* io_addr);
int igc_open(struct igc_adapter *adapter);
int igc_close(struct igc_adapter *adapter);
void igc_xmit_frame(struct igc_adapter *adapter, uint8_t* data, int len);
void igc_intr_msi(struct igc_adapter *adapter);
void igc_remove(struct igc_adapter *adapter);


/* Interrupt defines */
#define IGC_ITR_USECS	10  // interrupt latency in us, min 10, max 10000, we set it to 10 to reduce it by maximum amount

/* TX/RX descriptor defines */
#define IGC_DEFAULT_TXD		256
#define IGC_DEFAULT_TX_WORK	128
#define IGC_MIN_TXD		64
#define IGC_MAX_TXD		4096

#define IGC_DEFAULT_RXD		256
#define IGC_MIN_RXD		64
#define IGC_MAX_RXD		4096

/* Supported Rx Buffer Sizes */
#define IGC_RXBUFFER_256		256
#define IGC_RXBUFFER_2048		2048
#define IGC_RXBUFFER_3072		3072

#define IGC_RX_HDR_LEN			IGC_RXBUFFER_256

/* RX and TX descriptor control thresholds.
 * PTHRESH - MAC will consider prefetch if it has fewer than this number of
 *           descriptors available in its onboard memory.
 *           Setting this to 0 disables RX descriptor prefetch.
 * HTHRESH - MAC will only prefetch if there are at least this many descriptors
 *           available in host memory.
 *           If PTHRESH is 0, this should also be 0.
 * WTHRESH - RX descriptor writeback threshold - MAC will delay writing back
 *           descriptors until either it has this many to write back, or the
 *           ITR timer expires.
 */
#define IGC_RX_PTHRESH			8
#define IGC_RX_HTHRESH			8
#define IGC_TX_PTHRESH			8
#define IGC_TX_HTHRESH			1
#define IGC_RX_WTHRESH			4
#define IGC_TX_WTHRESH			16

#define IGC_RX_DMA_ATTR \
	(DMA_ATTR_SKIP_CPU_SYNC | DMA_ATTR_WEAK_ORDERING)

/* How many Rx Buffers do we bundle into one write to the hardware ? */
#define IGC_RX_BUFFER_WRITE	16 /* Must be power of 2 */


/* igc_test_staterr - tests bits within Rx descriptor status and error fields */
static inline __le32 igc_test_staterr(union igc_adv_rx_desc *rx_desc,
				      const uint32_t stat_err_bits)
{
	return rx_desc->wb.upper.status_error & cpu_to_le32(stat_err_bits);
}

/* wrapper around a pointer to a socket buffer,
 * so a DMA handle can be stored along with the buffer
 */
struct igc_tx_buffer {
	union igc_adv_tx_desc *next_to_watch;
	void* data;

	uint64_t dma;
	uint32_t len;
};

struct igc_rx_buffer {
	uint64_t dma;
	void *page;
};

struct igc_q_vector {
	struct igc_adapter *adapter;    /* backlink */

	struct igc_ring_container rx, tx;

	/* for dynamic allocation of rings associated with this q_vector */
	struct igc_ring ring[];
};

/* igc_desc_unused - calculate if we have unused descriptors */
static inline uint16_t igc_desc_unused(const struct igc_ring *ring)
{
	uint16_t ntc = ring->next_to_clean;
	uint16_t ntu = ring->next_to_use;

	return ((ntc > ntu) ? 0 : ring->count) + ntc - ntu - 1;
}

static inline int32_t igc_get_phy_info(struct igc_hw *hw)
{
	if (hw->phy.ops.get_phy_info)
		return hw->phy.ops.get_phy_info(hw);

	return 0;
}

static inline int32_t igc_reset_phy(struct igc_hw *hw)
{
	if (hw->phy.ops.reset)
		return hw->phy.ops.reset(hw);

	return 0;
}

static inline int32_t igc_read_phy_reg(struct igc_hw *hw, uint32_t offset, uint16_t *data)
{
	if (hw->phy.ops.read_reg)
		return hw->phy.ops.read_reg(hw, offset, data);

	return -1;
}

#define IGC_TXD_DCMD	(IGC_ADVTXD_DCMD_EOP | IGC_ADVTXD_DCMD_RS)

#define IGC_RX_DESC(R, i)       \
	(&(((union igc_adv_rx_desc *)((R)->desc))[i]))
#define IGC_TX_DESC(R, i)       \
	(&(((union igc_adv_tx_desc *)((R)->desc))[i]))

#endif /* _IGC_H_ */