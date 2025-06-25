/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c)  2018 Intel Corporation */

#ifndef _IGC_HW_H_
#define _IGC_HW_H_

#include <linux/types.h>
#include <linux/if_ether.h>

#include "igc_regs.h"
#include "igc_defines.h"
#include "igc_mac.h"
#include "igc_phy.h"
#include "igc_nvm.h"
#include "igc_i225.h"
#include "igc_base.h"
#include "igc_log.h"

/* Function pointers for the MAC. */
struct igc_mac_operations {
	int32_t (*check_for_link)(struct igc_hw *hw);
	int32_t (*reset_hw)(struct igc_hw *hw);
	int32_t (*init_hw)(struct igc_hw *hw);
	int32_t (*setup_physical_interface)(struct igc_hw *hw);
	void (*rar_set)(struct igc_hw *hw, uint8_t *address, uint32_t index);
	int32_t (*read_mac_addr)(struct igc_hw *hw);
	int32_t (*get_speed_and_duplex)(struct igc_hw *hw, uint16_t *speed,
				    uint16_t *duplex);
	int32_t (*acquire_swfw_sync)(struct igc_hw *hw, uint16_t mask);
	void (*release_swfw_sync)(struct igc_hw *hw, uint16_t mask);
};

enum igc_media_type {
	igc_media_type_unknown = 0,
	igc_media_type_copper = 1,
	igc_num_media_types
};

enum igc_nvm_type {
	igc_nvm_unknown = 0,
	igc_nvm_eeprom_spi,
};

struct igc_info {
	int32_t (*get_invariants)(struct igc_hw *hw);
	struct igc_mac_operations *mac_ops;
	const struct igc_phy_operations *phy_ops;
	struct igc_nvm_operations *nvm_ops;
};

extern const struct igc_info igc_base_info;

struct igc_mac_info {
	struct igc_mac_operations ops;

	uint8_t addr[ETH_ALEN];
	uint8_t perm_addr[ETH_ALEN];

	uint32_t mc_filter_type;

	uint16_t mta_reg_count;
	uint16_t uta_reg_count;

	uint32_t mta_shadow[MAX_MTA_REG];
	uint16_t rar_entry_count;

	bool asf_firmware_present;
	bool arc_subsystem_valid;

	bool autoneg;
	bool autoneg_failed;
	bool get_link_status;
};

struct igc_nvm_operations {
	int32_t (*acquire)(struct igc_hw *hw);
	int32_t (*read)(struct igc_hw *hw, uint16_t offset, uint16_t i, uint16_t *data);
	void (*release)(struct igc_hw *hw);
	int32_t (*write)(struct igc_hw *hw, uint16_t offset, uint16_t i, uint16_t *data);
	int32_t (*update)(struct igc_hw *hw);
	int32_t (*validate)(struct igc_hw *hw);
};

struct igc_phy_operations {
	int32_t (*acquire)(struct igc_hw *hw);
	int32_t (*check_reset_block)(struct igc_hw *hw);
	int32_t (*force_speed_duplex)(struct igc_hw *hw);
	int32_t (*get_phy_info)(struct igc_hw *hw);
	int32_t (*read_reg)(struct igc_hw *hw, uint32_t address, uint16_t *data);
	void (*release)(struct igc_hw *hw);
	int32_t (*reset)(struct igc_hw *hw);
	int32_t (*write_reg)(struct igc_hw *hw, uint32_t address, uint16_t data);
};

struct igc_nvm_info {
	struct igc_nvm_operations ops;
	enum igc_nvm_type type;

	uint16_t word_size;
	uint16_t delay_usec;
	uint16_t address_bits;
	uint16_t opcode_bits;
	uint16_t page_size;
};

struct igc_phy_info {
	struct igc_phy_operations ops;

	uint32_t addr;
	uint32_t id;
	uint32_t reset_delay_us; /* in usec */
	uint32_t revision;

	enum igc_media_type media_type;

	uint16_t autoneg_advertised;
	uint16_t autoneg_mask;
	
	bool autoneg_wait_to_complete;
};

struct igc_hw {
	uint8_t *hw_addr;

	struct igc_mac_info  mac;
	struct igc_nvm_info  nvm;
	struct igc_phy_info  phy;

	bool clear_semaphore_once;

	uint16_t device_id;
	uint16_t vendor_id;
};

#endif
