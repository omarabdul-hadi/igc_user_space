/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c)  2018 Intel Corporation */

#ifndef _IGC_NVM_H_
#define _IGC_NVM_H_

int32_t igc_acquire_nvm(struct igc_hw *hw);
void igc_release_nvm(struct igc_hw *hw);
int32_t igc_read_mac_addr(struct igc_hw *hw);
int32_t igc_read_nvm_eerd(struct igc_hw *hw, uint16_t offset, uint16_t words, uint16_t *data);
int32_t igc_validate_nvm_checksum(struct igc_hw *hw);
int32_t igc_update_nvm_checksum(struct igc_hw *hw);

#endif
