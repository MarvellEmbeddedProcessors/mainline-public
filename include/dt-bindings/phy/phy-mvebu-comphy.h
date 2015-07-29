/*
 * Copyright (C) 2015 Marvell
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#ifndef _DT_BINDINGS_PHY_MVEBU_COMPHY
#define _DT_BINDINGS_PHY_MVEBU_COMPHY

#define COMPHY_DEF(x, y)	(((x) << 8) | (y))

#define	COMPHY_UNUSED	COMPHY_DEF(0xff, 0xff)
#define COMPHY_SATA0	COMPHY_DEF(0x1, 0x0)
#define COMPHY_SATA1	COMPHY_DEF(0x1, 0x1)
#define COMPHY_SATA2	COMPHY_DEF(0x1, 0x2)
#define COMPHY_SATA3	COMPHY_DEF(0x1, 0x3)
#define COMPHY_SGMII0	COMPHY_DEF(0x2, 0x0)
#define COMPHY_SGMII1	COMPHY_DEF(0x2, 0x1)
#define COMPHY_SGMII2	COMPHY_DEF(0x2, 0x2)
#define COMPHY_SGMII3	COMPHY_DEF(0x2, 0x3)
#define COMPHY_USB3H0	COMPHY_DEF(0x3, 0x0)
#define COMPHY_USB3H1	COMPHY_DEF(0x3, 0x1)
#define COMPHY_USB3D0	COMPHY_DEF(0x4, 0x0)
#define COMPHY_PCIE00	COMPHY_DEF(0x5, 0x0)
#define COMPHY_PCIE01	COMPHY_DEF(0x5, 0x1)
#define COMPHY_PCIE02	COMPHY_DEF(0x5, 0x2)
#define COMPHY_PCIE03	COMPHY_DEF(0x5, 0x3)
#define COMPHY_PCIE10	COMPHY_DEF(0x5, 0x10)
#define COMPHY_PCIE20	COMPHY_DEF(0x5, 0x20)
#define COMPHY_PCIE30	COMPHY_DEF(0x5, 0x30)
#define COMPHY_QSGMII	COMPHY_DEF(0x6, 0x0)
#define COMPHY_XAUI0	COMPHY_DEF(0x7, 0x0)
#define COMPHY_XAUI1	COMPHY_DEF(0x7, 0x1)
#define COMPHY_XAUI2	COMPHY_DEF(0x7, 0x2)
#define COMPHY_XAUI3	COMPHY_DEF(0x7, 0x3)
#define COMPHY_RXAUI0	COMPHY_DEF(0x8, 0x0)
#define COMPHY_RXAUI1	COMPHY_DEF(0x8, 0x1)

#endif
