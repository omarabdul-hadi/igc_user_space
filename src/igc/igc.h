/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c)  2018 Intel Corporation */

#ifndef _IGC_H_
#define _IGC_H_

// system is little endian, so no conversion needed
#define cpu_to_le32
#define cpu_to_le64
#define le16_to_cpu
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
#include "../atemsys/atemsys.h"

struct igc_ring {
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

	struct igc_ring tx_ring;
	struct igc_ring rx_ring;

	bool link;
	uint16_t link_speed;
	uint16_t link_duplex;

	uint8_t *io_addr;
	struct igc_hw hw;
};

int igc_probe(struct igc_adapter *adapter, int fd, uint8_t* io_addr);
int igc_open(struct igc_adapter *adapter);
int igc_close(struct igc_adapter *adapter);
void igc_send_frame(struct igc_adapter *adapter, uint8_t* data, int len);
int igc_intr_msi(struct igc_adapter *adapter, uint8_t* receive_pkt);
void igc_remove(struct igc_adapter *adapter);

/* Interrupt defines */
#define IGC_ITR_USECS	100  // interrupt throttle rate, min 10, max 10000, 100 seems to yield the lowest latency when Tx Rx cycle is about 125 us
                             // this value vaguely translates to latency in us between interrupts

/* TX/RX descriptor defines */
#define IGC_DEFAULT_TXD		64 // original code documentation stated min is 64, but 8 seems to be the hard limit
#define IGC_DEFAULT_RXD		64 // original code documentation stated min is 64, but 8 seems to be the hard limit

#define IGC_RX_HDR_LEN			256

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

#define TX_BUFF_SIZE 2048
#define RX_BUFF_SIZE 2048

struct igc_tx_buffer {
	union igc_adv_tx_desc *next_to_watch;
	void* data;
	uint64_t dma;
};

struct igc_rx_buffer {
	void *data;
	uint64_t dma;
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

#define IGC_RX_DESC(R, i)       \
	(&(((union igc_adv_rx_desc *)((R)->desc))[i]))
#define IGC_TX_DESC(R, i)       \
	(&(((union igc_adv_tx_desc *)((R)->desc))[i]))

#endif /* _IGC_H_ */
