/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c)  2018 Intel Corporation */

#ifndef _IGC_MAC_H_
#define _IGC_MAC_H_

#include <stdint.h>
#include "igc_hw.h"
#include "igc_phy.h"
#include "igc_defines.h"

/* forward declaration */
int32_t igc_disable_pcie_master(struct igc_hw *hw);
int32_t igc_check_for_copper_link(struct igc_hw *hw);
int32_t igc_config_fc_after_link_up(struct igc_hw *hw);
int32_t igc_force_mac_fc(struct igc_hw *hw);
void igc_init_rx_addrs(struct igc_hw *hw, uint16_t rar_count);
int32_t igc_setup_link(struct igc_hw *hw);
void igc_clear_hw_cntrs_base(struct igc_hw *hw);
int32_t igc_get_auto_rd_done(struct igc_hw *hw);
void igc_put_hw_semaphore(struct igc_hw *hw);
void igc_rar_set(struct igc_hw *hw, uint8_t *addr, uint32_t index);
void igc_config_collision_dist(struct igc_hw *hw);

int32_t igc_get_speed_and_duplex_copper(struct igc_hw *hw, uint16_t *speed,
				    uint16_t *duplex);

bool igc_enable_mng_pass_thru(struct igc_hw *hw);

enum igc_mng_mode {
	igc_mng_mode_none = 0,
	igc_mng_mode_asf,
	igc_mng_mode_pt,
	igc_mng_mode_ipmi,
	igc_mng_mode_host_if_only
};

#endif
