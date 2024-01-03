// SPDX-License-Identifier: GPL-2.0
/* Copyright (c)  2018 Intel Corporation */

#include "igc_mac.h"
#include "igc_nvm.h"

/**
 * igc_poll_eerd_eewr_done - Poll for EEPROM read/write completion
 * @hw: pointer to the HW structure
 * @ee_reg: EEPROM flag for polling
 *
 * Polls the EEPROM status bit for either read or write completion based
 * upon the value of 'ee_reg'.
 */
static int32_t igc_poll_eerd_eewr_done(struct igc_hw *hw, int ee_reg)
{
	int32_t ret_val = -IGC_ERR_NVM;
	uint32_t attempts = 100000;
	uint32_t i, reg = 0;

	for (i = 0; i < attempts; i++) {
		if (ee_reg == IGC_NVM_POLL_READ)
			reg = rd32(IGC_EERD);
		else
			reg = rd32(IGC_EEWR);

		if (reg & IGC_NVM_RW_REG_DONE) {
			ret_val = 0;
			break;
		}

		usleep(5);
	}

	return ret_val;
}

/**
 * igc_acquire_nvm - Generic request for access to EEPROM
 * @hw: pointer to the HW structure
 *
 * Set the EEPROM access request bit and wait for EEPROM access grant bit.
 * Return successful if access grant bit set, else clear the request for
 * EEPROM access and return -IGC_ERR_NVM (-1).
 */
int32_t igc_acquire_nvm(struct igc_hw *hw)
{
	int32_t timeout = IGC_NVM_GRANT_ATTEMPTS;
	uint32_t eecd = rd32(IGC_EECD);
	int32_t ret_val = 0;

	wr32(IGC_EECD, eecd | IGC_EECD_REQ);
	eecd = rd32(IGC_EECD);

	while (timeout) {
		if (eecd & IGC_EECD_GNT)
			break;
		usleep(5);
		eecd = rd32(IGC_EECD);
		timeout--;
	}

	if (!timeout) {
		eecd &= ~IGC_EECD_REQ;
		wr32(IGC_EECD, eecd);
		printf("igc driver: Could not acquire NVM grant\n");
		ret_val = -IGC_ERR_NVM;
	}

	return ret_val;
}

/**
 * igc_release_nvm - Release exclusive access to EEPROM
 * @hw: pointer to the HW structure
 *
 * Stop any current commands to the EEPROM and clear the EEPROM request bit.
 */
void igc_release_nvm(struct igc_hw *hw)
{
	uint32_t eecd;

	eecd = rd32(IGC_EECD);
	eecd &= ~IGC_EECD_REQ;
	wr32(IGC_EECD, eecd);
}

/**
 * igc_read_nvm_eerd - Reads EEPROM using EERD register
 * @hw: pointer to the HW structure
 * @offset: offset of word in the EEPROM to read
 * @words: number of words to read
 * @data: word read from the EEPROM
 *
 * Reads a 16 bit word from the EEPROM using the EERD register.
 */
int32_t igc_read_nvm_eerd(struct igc_hw *hw, uint16_t offset, uint16_t words, uint16_t *data)
{
	struct igc_nvm_info *nvm = &hw->nvm;
	uint32_t i, eerd = 0;
	int32_t ret_val = 0;

	/* A check for invalid values:  offset too large, too many words,
	 * and not enough words.
	 */
	if (offset >= nvm->word_size || (words > (nvm->word_size - offset)) ||
	    words == 0) {
		printf("igc driver: nvm parameter(s) out of bounds\n");
		ret_val = -IGC_ERR_NVM;
		goto out;
	}

	for (i = 0; i < words; i++) {
		eerd = ((offset + i) << IGC_NVM_RW_ADDR_SHIFT) +
			IGC_NVM_RW_REG_START;

		wr32(IGC_EERD, eerd);
		ret_val = igc_poll_eerd_eewr_done(hw, IGC_NVM_POLL_READ);
		if (ret_val)
			break;

		data[i] = (rd32(IGC_EERD) >> IGC_NVM_RW_REG_DATA);
	}

out:
	return ret_val;
}

/**
 * igc_read_mac_addr - Read device MAC address
 * @hw: pointer to the HW structure
 */
int32_t igc_read_mac_addr(struct igc_hw *hw)
{
	uint32_t rar_high;
	uint32_t rar_low;
	uint16_t i;

	rar_high = rd32(IGC_RAH(0));
	rar_low = rd32(IGC_RAL(0));

	for (i = 0; i < IGC_RAL_MAC_ADDR_LEN; i++)
		hw->mac.perm_addr[i] = (uint8_t)(rar_low >> (i * 8));

	for (i = 0; i < IGC_RAH_MAC_ADDR_LEN; i++)
		hw->mac.perm_addr[i + 4] = (uint8_t)(rar_high >> (i * 8));

	for (i = 0; i < ETH_ALEN; i++)
		hw->mac.addr[i] = hw->mac.perm_addr[i];

	return 0;
}

/**
 * igc_validate_nvm_checksum - Validate EEPROM checksum
 * @hw: pointer to the HW structure
 *
 * Calculates the EEPROM checksum by reading/adding each word of the EEPROM
 * and then verifies that the sum of the EEPROM is equal to 0xBABA.
 */
int32_t igc_validate_nvm_checksum(struct igc_hw *hw)
{
	uint16_t checksum = 0;
	uint16_t i, nvm_data;
	int32_t ret_val = 0;

	for (i = 0; i < (NVM_CHECKSUM_REG + 1); i++) {
		ret_val = hw->nvm.ops.read(hw, i, 1, &nvm_data);
		if (ret_val) {
			printf("igc driver: NVM Read Error\n");
			goto out;
		}
		checksum += nvm_data;
	}

	if (checksum != (uint16_t)NVM_SUM) {
		printf("igc driver: NVM Checksum Invalid\n");
		ret_val = -IGC_ERR_NVM;
		goto out;
	}

out:
	return ret_val;
}

/**
 * igc_update_nvm_checksum - Update EEPROM checksum
 * @hw: pointer to the HW structure
 *
 * Updates the EEPROM checksum by reading/adding each word of the EEPROM
 * up to the checksum.  Then calculates the EEPROM checksum and writes the
 * value to the EEPROM.
 */
int32_t igc_update_nvm_checksum(struct igc_hw *hw)
{
	uint16_t checksum = 0;
	uint16_t i, nvm_data;
	int32_t  ret_val;

	for (i = 0; i < NVM_CHECKSUM_REG; i++) {
		ret_val = hw->nvm.ops.read(hw, i, 1, &nvm_data);
		if (ret_val) {
			printf("igc driver: NVM Read Error while updating checksum.\n");
			goto out;
		}
		checksum += nvm_data;
	}
	checksum = (uint16_t)NVM_SUM - checksum;
	ret_val = hw->nvm.ops.write(hw, NVM_CHECKSUM_REG, 1, &checksum);
	if (ret_val)
		printf("igc driver: NVM Write Error while updating checksum.\n");

out:
	return ret_val;
}
