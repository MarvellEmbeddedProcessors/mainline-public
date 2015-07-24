/*
 * Copyright (C) 2015 Marvell
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#ifndef _DT_BINDINGS_PHY_MVEBU_SERDES
#define _DT_BINDINGS_PHY_MVEBU_SERDES

#define SERDES_DEF(x, y)	(((x) << 8) | (y))

#define SERDES_UNCONNECTED	SERDES_DEF(0xff, 0xff)
#define SERDES_SATA0		SERDES_DEF(0x1, 0x0)
#define SERDES_SATA1		SERDES_DEF(0x1, 0x1)
#define SERDES_SGMII0		SERDES_DEF(0x2, 0x0)
#define SERDES_SGMII1		SERDES_DEF(0x2, 0x1)
#define SERDES_SGMII2		SERDES_DEF(0x2, 0x2)
#define SERDES_SGMII3		SERDES_DEF(0x2, 0x3)
#define SERDES_PCIE00		SERDES_DEF(0x3, 0x0)
#define SERDES_PCIE01		SERDES_DEF(0x3, 0x1)
#define SERDES_PCIE02		SERDES_DEF(0x3, 0x2)
#define SERDES_PCIE03		SERDES_DEF(0x3, 0x3)
#define SERDES_PCIE10		SERDES_DEF(0x3, 0x10)
#define SERDES_PCIE11		SERDES_DEF(0x3, 0x11)
#define SERDES_PCIE12		SERDES_DEF(0x3, 0x12)
#define SERDES_PCIE13		SERDES_DEF(0x3, 0x13)
#define SERDES_PCIE20		SERDES_DEF(0x3, 0x20)
#define SERDES_PCIE21		SERDES_DEF(0x3, 0x21)
#define SERDES_PCIE22		SERDES_DEF(0x3, 0x22)
#define SERDES_PCIE23		SERDES_DEF(0x3, 0x23)
#define SERDES_PCIE30		SERDES_DEF(0x3, 0x30)
#define SERDES_PCIE31		SERDES_DEF(0x3, 0x31)
#define SERDES_PCIE32		SERDES_DEF(0x3, 0x32)
#define SERDES_PCIE33		SERDES_DEF(0x3, 0x33)
#define SERDES_QSGMII		SERDES_DEF(0x4, 0x0)
#define SERDES_UNCONNECTED	SERDES_DEF(0xff, 0xff)

#endif /* _DT_BINDINGS_PHY_MVEBU_SERDES */
