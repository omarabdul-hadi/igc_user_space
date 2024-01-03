/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c)  2018 Intel Corporation */

#ifndef _IGC_PHY_H_
#define _IGC_PHY_H_

#include <linux/types.h>
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include "igc_mac.h"

int32_t igc_check_reset_block(struct igc_hw *hw);
int32_t igc_phy_hw_reset(struct igc_hw *hw);
int32_t igc_get_phy_id(struct igc_hw *hw);
int32_t igc_phy_has_link(struct igc_hw *hw, uint32_t iterations,
		     uint32_t usec_interval, bool *success);
int32_t igc_setup_copper_link(struct igc_hw *hw);
void igc_power_up_phy_copper(struct igc_hw *hw);
void igc_power_down_phy_copper(struct igc_hw *hw);
int32_t igc_write_phy_reg_gpy(struct igc_hw *hw, uint32_t offset, uint16_t data);
int32_t igc_read_phy_reg_gpy(struct igc_hw *hw, uint32_t offset, uint16_t *data);
uint16_t igc_read_phy_fw_version(struct igc_hw *hw);

#endif
