// SPDX-License-Identifier: GPL-2.0
/* Copyright (c)  2018 Intel Corporation */

#include <linux/pci.h>
#include <linux/types.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
//#include <linux/delay.h>

#include "igc_mac.h"
#include "igc_hw.h"

/**
 * igc_disable_pcie_master - Disables PCI-express master access
 * @hw: pointer to the HW structure
 *
 * Returns 0 (0) if successful, else returns -10
 * (-IGC_ERR_MASTER_REQUESTS_PENDING) if master disable bit has not caused
 * the master requests to be disabled.
 *
 * Disables PCI-Express master access and verifies there are no pending
 * requests.
 */
int32_t igc_disable_pcie_master(struct igc_hw *hw)
{
	int32_t timeout = MASTER_DISABLE_TIMEOUT;
	int32_t ret_val = 0;
	uint32_t ctrl;

	ctrl = rd32(IGC_CTRL);
	ctrl |= IGC_CTRL_GIO_MASTER_DISABLE;
	wr32(IGC_CTRL, ctrl);

	while (timeout) {
		if (!(rd32(IGC_STATUS) &
		    IGC_STATUS_GIO_MASTER_ENABLE))
			break;
		usleep(3000);
		timeout--;
	}

	if (!timeout) {
		igc_logger(IGC_LOG_INF, " Master requests are pending.\n");
		ret_val = -IGC_ERR_MASTER_REQUESTS_PENDING;
		goto out;
	}

out:
	return ret_val;
}

/**
 * igc_init_rx_addrs - Initialize receive addresses
 * @hw: pointer to the HW structure
 * @rar_count: receive address registers
 *
 * Setup the receive address registers by setting the base receive address
 * register to the devices MAC address and clearing all the other receive
 * address registers to 0.
 */
void igc_init_rx_addrs(struct igc_hw *hw, uint16_t rar_count)
{
	uint8_t mac_addr[ETH_ALEN] = {0};
	uint32_t i;

	/* Setup the receive address */
	igc_logger(IGC_LOG_INF, " Programming MAC Address into RAR[0]\n");

	hw->mac.ops.rar_set(hw, hw->mac.addr, 0);

	/* Zero out the other (rar_entry_count - 1) receive addresses */
	igc_logger(IGC_LOG_INF, " Clearing RAR[1-%u]\n", rar_count - 1);
	for (i = 1; i < rar_count; i++)
		hw->mac.ops.rar_set(hw, mac_addr, i);
}

/**
 * igc_set_fc_watermarks - Set flow control high/low watermarks
 * @hw: pointer to the HW structure
 *
 * Sets the flow control high/low threshold (watermark) registers.  If
 * flow control XON frame transmission is enabled, then set XON frame
 * transmission as well.
 */
static int32_t igc_set_fc_watermarks(struct igc_hw *hw)
{
	uint32_t fcrtl = 0, fcrth = 0;

	/* Set the flow control receive threshold registers.  Normally,
	 * these registers will be set to a default threshold that may be
	 * adjusted later by the driver's runtime code.  However, if the
	 * ability to transmit pause frames is not enabled, then these
	 * registers will be set to 0.
	 */
	wr32(IGC_FCRTL, fcrtl);
	wr32(IGC_FCRTH, fcrth);

	return 0;
}

/**
 * igc_setup_link - Setup flow control and link settings
 * @hw: pointer to the HW structure
 *
 * Determines which flow control settings to use, then configures flow
 * control.  Calls the appropriate media-specific link configuration
 * function.  Assuming the adapter has a valid link partner, a valid link
 * should be established.  Assumes the hardware has previously been reset
 * and the transmitter and receiver are not enabled.
 */
int32_t igc_setup_link(struct igc_hw *hw)
{
	int32_t ret_val = 0;

	/* In the case of the phy reset being blocked, we already have a link.
	 * We do not need to set it up again.
	 */
	if (igc_check_reset_block(hw))
		goto out;

	/* Call the necessary media_type subroutine to configure the link. */
	ret_val = hw->mac.ops.setup_physical_interface(hw);
	if (ret_val)
		goto out;

	/* Initialize the flow control address, type, and PAUSE timer
	 * registers to their default values.  This is done even if flow
	 * control is disabled, because it does not hurt anything to
	 * initialize these registers.
	 */
	igc_logger(IGC_LOG_INF, " Initializing the Flow Control address, type and timer regs\n");
	wr32(IGC_FCT, FLOW_CONTROL_TYPE);
	wr32(IGC_FCAH, FLOW_CONTROL_ADDRESS_HIGH);
	wr32(IGC_FCAL, FLOW_CONTROL_ADDRESS_LOW);

	wr32(IGC_FCTTV, 0);

	ret_val = igc_set_fc_watermarks(hw);

out:
	return ret_val;
}

/**
 * igc_force_mac_fc - Force the MAC's flow control settings
 * @hw: pointer to the HW structure
 *
 * Force the MAC's flow control settings.  Sets the TFCE and RFCE bits in the
 * device control register to reflect the adapter settings.  TFCE and RFCE
 * need to be explicitly set by software when a copper PHY is used because
 * autonegotiation is managed by the PHY rather than the MAC.  Software must
 * also configure these bits when link is forced on a fiber connection.
 */
int32_t igc_force_mac_fc(struct igc_hw *hw)
{
	igc_logger(IGC_LOG_INF, " turn off flow control\n");

	uint32_t ctrl = rd32(IGC_CTRL);
	ctrl &= (~(IGC_CTRL_TFCE | IGC_CTRL_RFCE));
	wr32(IGC_CTRL, ctrl);

	return 0;
}

/**
 * igc_clear_hw_cntrs_base - Clear base hardware counters
 * @hw: pointer to the HW structure
 *
 * Clears the base hardware counters by reading the counter registers.
 */
void igc_clear_hw_cntrs_base(struct igc_hw *hw)
{
	rd32(IGC_CRCERRS);
	rd32(IGC_MPC);
	rd32(IGC_SCC);
	rd32(IGC_ECOL);
	rd32(IGC_MCC);
	rd32(IGC_LATECOL);
	rd32(IGC_COLC);
	rd32(IGC_RERC);
	rd32(IGC_DC);
	rd32(IGC_RLEC);
	rd32(IGC_XONRXC);
	rd32(IGC_XONTXC);
	rd32(IGC_XOFFRXC);
	rd32(IGC_XOFFTXC);
	rd32(IGC_FCRUC);
	rd32(IGC_GPRC);
	rd32(IGC_BPRC);
	rd32(IGC_MPRC);
	rd32(IGC_GPTC);
	rd32(IGC_GORCL);
	rd32(IGC_GORCH);
	rd32(IGC_GOTCL);
	rd32(IGC_GOTCH);
	rd32(IGC_RNBC);
	rd32(IGC_RUC);
	rd32(IGC_RFC);
	rd32(IGC_ROC);
	rd32(IGC_RJC);
	rd32(IGC_TORL);
	rd32(IGC_TORH);
	rd32(IGC_TOTL);
	rd32(IGC_TOTH);
	rd32(IGC_TPR);
	rd32(IGC_TPT);
	rd32(IGC_MPTC);
	rd32(IGC_BPTC);

	rd32(IGC_PRC64);
	rd32(IGC_PRC127);
	rd32(IGC_PRC255);
	rd32(IGC_PRC511);
	rd32(IGC_PRC1023);
	rd32(IGC_PRC1522);
	rd32(IGC_PTC64);
	rd32(IGC_PTC127);
	rd32(IGC_PTC255);
	rd32(IGC_PTC511);
	rd32(IGC_PTC1023);
	rd32(IGC_PTC1522);

	rd32(IGC_ALGNERRC);
	rd32(IGC_RXERRC);
	rd32(IGC_TNCRS);
	rd32(IGC_HTDPMC);
	rd32(IGC_TSCTC);

	rd32(IGC_MGTPRC);
	rd32(IGC_MGTPDC);
	rd32(IGC_MGTPTC);

	rd32(IGC_IAC);

	rd32(IGC_RPTHC);
	rd32(IGC_TLPIC);
	rd32(IGC_RLPIC);
	rd32(IGC_HGPTC);
	rd32(IGC_RXDMTC);
	rd32(IGC_HGORCL);
	rd32(IGC_HGORCH);
	rd32(IGC_HGOTCL);
	rd32(IGC_HGOTCH);
	rd32(IGC_LENERRS);
}

/**
 * igc_rar_set - Set receive address register
 * @hw: pointer to the HW structure
 * @addr: pointer to the receive address
 * @index: receive address array register
 *
 * Sets the receive address array register at index to the address passed
 * in by addr.
 */
void igc_rar_set(struct igc_hw *hw, uint8_t *addr, uint32_t index)
{
	uint32_t rar_low, rar_high;

	/* HW expects these in little endian so we reverse the byte order
	 * from network order (big endian) to little endian
	 */
	rar_low = ((uint32_t)addr[0] |
		   ((uint32_t)addr[1] << 8) |
		   ((uint32_t)addr[2] << 16) | ((uint32_t)addr[3] << 24));

	rar_high = ((uint32_t)addr[4] | ((uint32_t)addr[5] << 8));

	/* If MAC address zero, no need to set the AV bit */
	if (rar_low || rar_high)
		rar_high |= IGC_RAH_AV;

	/* Some bridges will combine consecutive 32-bit writes into
	 * a single burst write, which will malfunction on some parts.
	 * The flushes avoid this.
	 */
	wr32(IGC_RAL(index), rar_low);
	wrfl();
	wr32(IGC_RAH(index), rar_high);
	wrfl();
}

/**
 * igc_check_for_copper_link - Check for link (Copper)
 * @hw: pointer to the HW structure
 *
 * Checks to see of the link status of the hardware has changed.  If a
 * change in link status has been detected, then we read the PHY registers
 * to get the current speed/duplex if link exists.
 */
int32_t igc_check_for_copper_link(struct igc_hw *hw)
{
	struct igc_mac_info *mac = &hw->mac;
	bool link = false;
	int32_t ret_val;

	/* We only want to go out to the PHY registers to see if Auto-Neg
	 * has completed and/or if our link status has changed.  The
	 * get_link_status flag is set upon receiving a Link Status
	 * Change or Rx Sequence Error interrupt.
	 */
	if (!mac->get_link_status) {
		ret_val = 0;
		goto out;
	}

	/* First we want to see if the MII Status Register reports
	 * link.  If so, then we want to get the current speed/duplex
	 * of the PHY.
	 */
	ret_val = igc_phy_has_link(hw, 1, 0, &link);
	if (ret_val)
		goto out;

	if (!link)
		goto out; /* No link detected */

	mac->get_link_status = false;

	/* If we are forcing speed/duplex, then we simply return since
	 * we have already determined whether we have link or not.
	 */
	if (!mac->autoneg) {
		ret_val = -IGC_ERR_CONFIG;
		goto out;
	}

	/* Auto-Neg is enabled.  Auto Speed Detection takes care
	 * of MAC speed/duplex configuration.  So we only need to
	 * configure Collision Distance in the MAC.
	 */
	igc_config_collision_dist(hw);

	/* Configure Flow Control now that Auto-Neg has completed.
	 * First, we need to restore the desired flow control
	 * settings because we may have had to re-autoneg with a
	 * different link partner.
	 */
	ret_val = igc_config_fc_after_link_up(hw);
	if (ret_val)
		igc_logger(IGC_LOG_ERR, "configuring flow control\n");

out:
	/* Now that we are aware of our link settings, we can set the LTR
	 * thresholds.
	 */
	ret_val = igc_set_ltr_i225(hw, link);

	return ret_val;
}

/**
 * igc_config_collision_dist - Configure collision distance
 * @hw: pointer to the HW structure
 *
 * Configures the collision distance to the default value and is used
 * during link setup. Currently no func pointer exists and all
 * implementations are handled in the generic version of this function.
 */
void igc_config_collision_dist(struct igc_hw *hw)
{
	uint32_t tctl;

	tctl = rd32(IGC_TCTL);

	tctl &= ~IGC_TCTL_COLD;
	tctl |= IGC_COLLISION_DISTANCE << IGC_COLD_SHIFT;

	wr32(IGC_TCTL, tctl);
	wrfl();
}

/**
 * igc_config_fc_after_link_up - Configures flow control after link
 * @hw: pointer to the HW structure
 *
 * Checks the status of auto-negotiation after link up to ensure that the
 * speed and duplex were not forced.  If the link needed to be forced, then
 * flow control needs to be forced also.  If auto-negotiation is enabled
 * and did not fail, then we configure flow control based on our link
 * partner.
 */
int32_t igc_config_fc_after_link_up(struct igc_hw *hw)
{
	uint16_t mii_status_reg, mii_nway_adv_reg, mii_nway_lp_ability_reg;
	struct igc_mac_info *mac = &hw->mac;
	uint16_t speed, duplex;
	int32_t ret_val = 0;

	/* Check for the case where we have fiber media and auto-neg failed
	 * so we had to force link.  In this case, we need to force the
	 * configuration of the MAC to match the "fc" parameter.
	 */
	if (mac->autoneg_failed)
		ret_val = igc_force_mac_fc(hw);

	if (ret_val) {
		igc_logger(IGC_LOG_ERR, "forcing flow control settings\n");
		goto out;
	}

	/* Check for the case where we have copper media and auto-neg is
	 * enabled.  In this case, we need to check and see if Auto-Neg
	 * has completed, and if so, how the PHY and link partner has
	 * flow control configured.
	 */
	if (mac->autoneg) {
		/* Read the MII Status Register and check to see if AutoNeg
		 * has completed.  We read this twice because this reg has
		 * some "sticky" (latched) bits.
		 */
		ret_val = hw->phy.ops.read_reg(hw, PHY_STATUS,
					       &mii_status_reg);
		if (ret_val)
			goto out;
		ret_val = hw->phy.ops.read_reg(hw, PHY_STATUS,
					       &mii_status_reg);
		if (ret_val)
			goto out;

		if (!(mii_status_reg & MII_SR_AUTONEG_COMPLETE)) {
			igc_logger(IGC_LOG_INF, " Copper PHY and Auto Neg has not completed.\n");
			goto out;
		}

		/* The AutoNeg process has completed, so we now need to
		 * read both the Auto Negotiation Advertisement
		 * Register (Address 4) and the Auto_Negotiation Base
		 * Page Ability Register (Address 5) to determine how
		 * flow control was negotiated.
		 */
		ret_val = hw->phy.ops.read_reg(hw, PHY_AUTONEG_ADV,
					       &mii_nway_adv_reg);
		if (ret_val)
			goto out;
		ret_val = hw->phy.ops.read_reg(hw, PHY_LP_ABILITY,
					       &mii_nway_lp_ability_reg);
		if (ret_val)
			goto out;

		ret_val = hw->mac.ops.get_speed_and_duplex(hw, &speed, &duplex);
		if (ret_val) {
			igc_logger(IGC_LOG_ERR, "getting link speed and duplex\n");
			goto out;
		}

		//turn off flow control
		ret_val = igc_force_mac_fc(hw);
		if (ret_val) {
			igc_logger(IGC_LOG_ERR, "forcing flow control settings\n");
			goto out;
		}
	}

out:
	return ret_val;
}

/**
 * igc_get_auto_rd_done - Check for auto read completion
 * @hw: pointer to the HW structure
 *
 * Check EEPROM for Auto Read done bit.
 */
int32_t igc_get_auto_rd_done(struct igc_hw *hw)
{
	int32_t ret_val = 0;
	int32_t i = 0;

	while (i < AUTO_READ_DONE_TIMEOUT) {
		if (rd32(IGC_EECD) & IGC_EECD_AUTO_RD)
			break;
		usleep(2000);
		i++;
	}

	if (i == AUTO_READ_DONE_TIMEOUT) {
		igc_logger(IGC_LOG_INF, " Auto read by HW from NVM has not completed.\n");
		ret_val = -IGC_ERR_RESET;
		goto out;
	}

out:
	return ret_val;
}

/**
 * igc_get_speed_and_duplex_copper - Retrieve current speed/duplex
 * @hw: pointer to the HW structure
 * @speed: stores the current speed
 * @duplex: stores the current duplex
 *
 * Read the status register for the current speed/duplex and store the current
 * speed and duplex for copper connections.
 */
int32_t igc_get_speed_and_duplex_copper(struct igc_hw *hw, uint16_t *speed,
				    uint16_t *duplex)
{
	uint32_t status;

	status = rd32(IGC_STATUS);
	if (status & IGC_STATUS_SPEED_1000) {
		/* For I225, STATUS will indicate 1G speed in both 1 Gbps
		 * and 2.5 Gbps link modes. An additional bit is used
		 * to differentiate between 1 Gbps and 2.5 Gbps.
		 */
		if (status & IGC_STATUS_SPEED_2500) {
			*speed = SPEED_2500;
			igc_logger(IGC_LOG_INF, " 2500 Mbs, ");
		} else {
			*speed = SPEED_1000;
			igc_logger(IGC_LOG_INF, " 1000 Mbs, ");
		}
	} else if (status & IGC_STATUS_SPEED_100) {
		*speed = SPEED_100;
		igc_logger(IGC_LOG_INF, " 100 Mbs, ");
	} else {
		*speed = SPEED_10;
		igc_logger(IGC_LOG_INF, " 10 Mbs, ");
	}

	if (status & IGC_STATUS_FD) {
		*duplex = FULL_DUPLEX;
		igc_logger(IGC_LOG_INF, " Full Duplex\n");
	} else {
		*duplex = HALF_DUPLEX;
		igc_logger(IGC_LOG_INF, " Half Duplex\n");
	}

	return 0;
}

/**
 * igc_put_hw_semaphore - Release hardware semaphore
 * @hw: pointer to the HW structure
 *
 * Release hardware semaphore used to access the PHY or NVM
 */
void igc_put_hw_semaphore(struct igc_hw *hw)
{
	uint32_t swsm;

	swsm = rd32(IGC_SWSM);

	swsm &= ~(IGC_SWSM_SMBI | IGC_SWSM_SWESMBI);

	wr32(IGC_SWSM, swsm);
}

/**
 * igc_enable_mng_pass_thru - Enable processing of ARP's
 * @hw: pointer to the HW structure
 *
 * Verifies the hardware needs to leave interface enabled so that frames can
 * be directed to and from the management interface.
 */
bool igc_enable_mng_pass_thru(struct igc_hw *hw)
{
	bool ret_val = false;
	uint32_t fwsm, factps;
	uint32_t manc;

	if (!hw->mac.asf_firmware_present)
		goto out;

	manc = rd32(IGC_MANC);

	if (!(manc & IGC_MANC_RCV_TCO_EN))
		goto out;

	if (hw->mac.arc_subsystem_valid) {
		fwsm = rd32(IGC_FWSM);
		factps = rd32(IGC_FACTPS);

		if (!(factps & IGC_FACTPS_MNGCG) &&
		    ((fwsm & IGC_FWSM_MODE_MASK) ==
		    (igc_mng_mode_pt << IGC_FWSM_MODE_SHIFT))) {
			ret_val = true;
			goto out;
		}
	} else {
		if ((manc & IGC_MANC_SMBUS_EN) &&
		    !(manc & IGC_MANC_ASF_EN)) {
			ret_val = true;
			goto out;
		}
	}

out:
	return ret_val;
}
