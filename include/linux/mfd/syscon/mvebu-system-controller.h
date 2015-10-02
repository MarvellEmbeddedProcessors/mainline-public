/*
 * Copyright (C) 2015 Marvell
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __MVEBU_SYSTEM_CONTROLLER_H__
#define __MVEBU_SYSTEM_CONTROLLER_H__

#define SOC_CONTROL_REG		0x4
#define SAMPLE_AT_RESET		0x30
#define    SAR_PCIE_CLKOUT	BIT(2)
#define COMPHY_REF_CLK_ALIGN	0xF8 /* Only on Armada 370/XP */
#define    COMPHY_REF_CLK_ALIGN_PCIE_MASK  0xFFFF
#define QSGMII_CTRL0		0x200
#define QSGMII_CTRL1		0x204
#define    QSGMII_ENABLE	BIT(30)
#define QSMII_STATUS		0x208

#endif /* __MVEBU_SYSTEM_CONTROLLER_H__ */
