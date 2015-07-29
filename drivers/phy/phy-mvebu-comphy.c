/*
 * Copyright (C) 2015 Marvell
 *
 * Thomas Petazzoni <thomas.petazzoni@free-electrons.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <dt-bindings/phy/phy-mvebu-comphy.h>

#define MVEBU_COMPHY_MAX_CNT	7
#define MVEBU_COMPHY_FUNC_MAX   8

/* Common PHY bridge registers */

#define A375_COMPHY_SELECTOR_REG	0x0
#define A375_COMPHY_CONFIG1_REG(n)	(0x10 + ((n) * 0x4))
#define    A375_REF_CLK_DISABLE_BIT	BIT(0)
#define    A375_POWER_UP_IVREF_BIT	BIT(1)
#define    A375_PIPE_SELECT_BIT		BIT(2)
#define    A375_PHY_SOFT_RESET_BIT	BIT(11)
#define    A375_PHY_RESET_CORE_BIT	BIT(12)
#define    A375_PHY_CORE_RSTN_BIT	BIT(13)
#define    A375_PHY_POWERON_RST_BIT	BIT(14)
#define    A375_PHY_MODE_BIT		BIT(15)
#define	   A375_POWER_UP_PLL_BIT	BIT(16)
#define	   A375_POWER_UP_RX_BIT		BIT(17)
#define	   A375_POWER_UP_TX_BIT		BIT(18)
#define    A375_PIN_TX_IDLE_BIT		BIT(19)
#define    A375_GEN_RX_SHIFT		22
#define    A375_GEN_RX_MASK		(0xF << A375_GEN_RX_SHIFT)
#define    A375_GEN_TX_SHIFT		26
#define    A375_GEN_TX_MASK		(0xF << A375_GEN_TX_SHIFT)
#define A375_COMPHY_CONFIG2_REG(n)	(0x20 + ((n) * 0x4))
#define A375_COMPHY_STATUS1_REG(n)	(0x40 + ((n) * 0x4))
#define    A375_PHY_MAC_STATUS_BIT	BIT(1)
#define    A375_PLL_READY_RX_BIT	BIT(2)
#define A375_COMPHY_STATUS2_REG(n)	(0x50 + ((n) * 0x4))

/* Common PHY control and configuration */

#define A375_POWER_CONTROL_PLL_REG(n)	(((n) << 13) + 0x4)
#define   REF_FREF_SEL_SHIFT		0
#define   REF_FREF_SATA_20		0
#define   REF_FREF_SATA_25		1
#define   REF_FREF_SGMII_20		0
#define   REF_FREF_SGMII_25		1
#define   REF_FREF_PCIE			0
#define   PHY_MODE_SHIFT		5
#define   PHY_MODE_SATA			0
#define   PHY_MODE_PCIE			3
#define   PHY_MODE_SGMII		4
#define   PHY_MODE_USB3			5
#define   UNKNOWN_BIT			BIT(10)
#define   RESERVED_BIT			BIT(11)
#define	  POWERUP_TX_BIT		BIT(12)
#define	  POWERUP_RX_BIT		BIT(13)
#define	  POWERUP_PLL_BIT		BIT(14)
#define	  POWERUP_IVREF_BIT		BIT(15)
#define A375_DIGITAL_LOOPBACK_EN_REG(n)	(((n) << 13) + 0x8c)
#define   SEL_BITS_10			(0 << 10)
#define   SEL_BITS_20			(1 << 10)
#define   SEL_BITS_40			(2 << 10)
#define A375_PHY_ISOL_MODE_CTRL_REG(n)	(((n) << 13) + 0x98)

/* Undocumented */
#define A375_INTERFACE_REG1_REG(n)	(((n) << 13) + 0x94)
#define A375_DIGITAL_RESERVED0_REG(n)   (((n) << 13) + 0xe8)
#define A375_MISC_CONTROL0_REG(n)	(((n) << 13) + 0x13c)
#define A375_POWER_REG1_REG(n)		(((n) << 13) + 0x148)
#define A375_LANE_CONFIG4_REG(n)        (((n) << 13) + 0x220)
#define A375_RESET_CLOCK_CONTROL_REG(n)	(((n) << 13) + 0x304)

struct phy_mvebu_comphy_soc_info {
	int nlanes;
	int functions[MVEBU_COMPHY_MAX_CNT][MVEBU_COMPHY_FUNC_MAX];
};

struct phy_mvebu_comphy_priv {
	struct device *dev;
	void __iomem *comphy_regs;
	void __iomem *comphy_pipe_regs;
	spinlock_t lock;
	const struct phy_mvebu_comphy_soc_info *sinfo;
	struct mvebu_comphy {
		struct phy *phy;
		int mode;
		int index;
	} lanes[MVEBU_COMPHY_MAX_CNT];
};

#define to_phy_mvebu_comphy_priv(lane) \
        container_of((lane), struct phy_mvebu_comphy_priv, lanes[(lane)->index])

static void phy_mvebu_comphy_reset(struct phy_mvebu_comphy_priv *priv,
				   struct mvebu_comphy *comphy)
{
	u32 reg;

	reg = readl(priv->comphy_regs + A375_COMPHY_CONFIG1_REG(comphy->index));
	reg |= (A375_PHY_SOFT_RESET_BIT | A375_PHY_RESET_CORE_BIT);
	reg &= ~(A375_PHY_CORE_RSTN_BIT | A375_PHY_POWERON_RST_BIT);
	writel(reg, priv->comphy_regs + A375_COMPHY_CONFIG1_REG(comphy->index));
}

static void phy_mvebu_comphy_unreset(struct phy_mvebu_comphy_priv *priv,
				     struct mvebu_comphy *comphy)
{
	u32 reg;

	reg = readl(priv->comphy_regs + A375_COMPHY_CONFIG1_REG(comphy->index));
	reg &= ~(A375_PHY_SOFT_RESET_BIT | A375_PHY_RESET_CORE_BIT);
	reg |= (A375_PHY_CORE_RSTN_BIT | A375_PHY_POWERON_RST_BIT);
	writel(reg, priv->comphy_regs + A375_COMPHY_CONFIG1_REG(comphy->index));
}

static void phy_mvebu_comphy_select_mode(struct phy_mvebu_comphy_priv *priv,
					 struct mvebu_comphy *comphy)
{
	u32 reg;

	reg = readl(priv->comphy_regs + A375_COMPHY_SELECTOR_REG);
	switch (comphy->index) {
	case 0:
		if (comphy->mode == COMPHY_PCIE00)
			reg |= (1 << 1);
		break;
	case 1:
		reg &= ~(0x3 << 1);
		if (comphy->mode == COMPHY_PCIE10)
			reg |= (0 << 1);
		else if (comphy->mode == COMPHY_SGMII0)
			reg |= (1 << 1);
		else if (comphy->mode == COMPHY_SATA1)
			reg |= (2 << 1);
		break;
	case 2:
		if (comphy->mode == COMPHY_SGMII0)
			reg &= ~(1 << 3);
		else if (comphy->mode == COMPHY_SATA0)
			reg |= (1 << 3);
		break;
	case 3:
		if (comphy->mode == COMPHY_USB3H0)
			reg &= ~(1 << 4);
		else if (comphy->mode == COMPHY_SGMII0)
			reg |= (1 << 4);
		break;
	}

	writel(reg, priv->comphy_regs + A375_COMPHY_SELECTOR_REG);
}

static void phy_mvebu_comphy_phy_config(struct phy_mvebu_comphy_priv *priv,
					struct mvebu_comphy *comphy)
{
	u32 reg;

	reg = readl(priv->comphy_regs + A375_COMPHY_CONFIG1_REG(comphy->index));

	reg |= A375_POWER_UP_IVREF_BIT;
	reg &= ~A375_REF_CLK_DISABLE_BIT;
	reg |= A375_PIN_TX_IDLE_BIT;

	if (comphy->mode == COMPHY_USB3H0) {
		reg |= A375_PIPE_SELECT_BIT;
		reg |= A375_PHY_MODE_BIT;
	} else if (comphy->mode == COMPHY_PCIE00 || comphy->mode == COMPHY_PCIE10) {
		reg |= A375_PIPE_SELECT_BIT;
		reg &= ~A375_PHY_MODE_BIT;
	}
	else {
		reg &= ~A375_PIPE_SELECT_BIT;
	}

	reg &= ~(A375_GEN_RX_MASK | A375_GEN_TX_MASK);
	if (comphy->mode == COMPHY_SATA0 || comphy->mode == COMPHY_SATA1 ||
	    comphy->mode == COMPHY_USB3H0) {
		reg |= 0x1 << A375_GEN_RX_SHIFT;
		reg |= 0x1 << A375_GEN_TX_SHIFT;
	} else if (comphy->mode == COMPHY_SGMII0) {
		/* FIXME: GEN1 or GEN2 for SGMII. Assuming GEN1 for
		 * now. GEN2 would need 0x8 */
		reg |= 0x6 << A375_GEN_RX_SHIFT;
		reg |= 0x6 << A375_GEN_TX_SHIFT;
	}

	writel(reg, priv->comphy_regs + A375_COMPHY_CONFIG1_REG(comphy->index));
}

static void phy_mvebu_comphy_pcie_config(struct phy_mvebu_comphy_priv *priv,
					 struct mvebu_comphy *comphy)
{
	u32 reg;

	/* Enable soft reset */
	writel(0x25,
	       priv->comphy_pipe_regs + A375_RESET_CLOCK_CONTROL_REG(comphy->index));

	/* Set PHY mode to PCIe */
	reg = POWERUP_TX_BIT | POWERUP_RX_BIT | POWERUP_PLL_BIT |
		POWERUP_IVREF_BIT;
	reg |= RESERVED_BIT | UNKNOWN_BIT;
	reg |= PHY_MODE_PCIE << PHY_MODE_SHIFT;
	reg |= REF_FREF_PCIE << REF_FREF_SEL_SHIFT;
	writel(reg,
	       priv->comphy_pipe_regs + A375_POWER_CONTROL_PLL_REG(comphy->index));

	/* Refclk SEL to 100 Mhz */
	writel(0x6017,
	       priv->comphy_pipe_regs + A375_MISC_CONTROL0_REG(comphy->index));

	/* PHY Gen to 5G */
	writel(0x1400,
	       priv->comphy_pipe_regs + A375_INTERFACE_REG1_REG(comphy->index));

	/* SEL bits to 20 bits */
	writel(0x400,
	       priv->comphy_pipe_regs + A375_DIGITAL_LOOPBACK_EN_REG(comphy->index));

	/* Reset and unreset DFE sequence */
	writel(0xE409,
	       priv->comphy_pipe_regs + A375_POWER_REG1_REG(comphy->index));
	writel(0xE008,
	       priv->comphy_pipe_regs + A375_POWER_REG1_REG(comphy->index));

	/* Release soft reset */
	writel(0x24,
	       priv->comphy_pipe_regs + A375_RESET_CLOCK_CONTROL_REG(comphy->index));
}

static void phy_mvebu_comphy_usb3_config(struct phy_mvebu_comphy_priv *priv,
					 struct mvebu_comphy *comphy)
{
	u32 reg;

	/* Enable soft reset */
	writel(0x21,
	       priv->comphy_pipe_regs + A375_RESET_CLOCK_CONTROL_REG(comphy->index));

	/* Set PHY mode to USB3 */
	reg = POWERUP_TX_BIT | POWERUP_RX_BIT | POWERUP_PLL_BIT |
		POWERUP_IVREF_BIT;
	reg |= RESERVED_BIT | UNKNOWN_BIT;
	reg |= PHY_MODE_USB3 << PHY_MODE_SHIFT;
	writel(reg,
	       priv->comphy_pipe_regs + A375_POWER_CONTROL_PLL_REG(comphy->index));

	/* Refclk to 100 Mhz */
	writel(0x13,
	       priv->comphy_pipe_regs + A375_LANE_CONFIG4_REG(comphy->index));

	/* Refclk SEL to 100 Mhz */
	writel(0x6017,
	       priv->comphy_pipe_regs + A375_MISC_CONTROL0_REG(comphy->index));

	/* PHY Gen to 5G */
	writel(0x1400,
	       priv->comphy_pipe_regs + A375_INTERFACE_REG1_REG(comphy->index));

	/* SEL bits to 20 bits */
	writel(0x400,
	       priv->comphy_pipe_regs + A375_DIGITAL_LOOPBACK_EN_REG(comphy->index));

	/* Reset and unreset DFE sequence */
	writel(0xE409,
	       priv->comphy_pipe_regs + A375_POWER_REG1_REG(comphy->index));
	writel(0xE008,
	       priv->comphy_pipe_regs + A375_POWER_REG1_REG(comphy->index));

	/* Release soft reset */
	writel(0x20,
	       priv->comphy_pipe_regs + A375_RESET_CLOCK_CONTROL_REG(comphy->index));
}

static void phy_mvebu_comphy_sata_config(struct phy_mvebu_comphy_priv *priv,
					 struct mvebu_comphy *comphy)
{
	u32 reg;

	/* Set PHY mode to SATA */
	reg = POWERUP_TX_BIT | POWERUP_RX_BIT | POWERUP_PLL_BIT |
		POWERUP_IVREF_BIT;
	reg |= RESERVED_BIT | UNKNOWN_BIT;
	reg |= PHY_MODE_SATA << PHY_MODE_SHIFT;
	reg |= REF_FREF_SATA_25 << REF_FREF_SEL_SHIFT;
	writel(reg,
	       priv->comphy_pipe_regs + A375_POWER_CONTROL_PLL_REG(comphy->index));

	/* Refclk SEL to 25 Mhz */
	writel(0x6417,
	       priv->comphy_pipe_regs + A375_MISC_CONTROL0_REG(comphy->index));

	/* PHY Gen to 5G */
	writel(0x1400,
	       priv->comphy_pipe_regs + A375_INTERFACE_REG1_REG(comphy->index));

	/* SEL bits to 20 bits */
	writel(0x400,
	       priv->comphy_pipe_regs + A375_DIGITAL_LOOPBACK_EN_REG(comphy->index));

	/* Req_sq_de_glitch_en */
	writel(0xE,
	       priv->comphy_pipe_regs + A375_DIGITAL_RESERVED0_REG(comphy->index));

	/* Reset and unreset DFE sequence */
	writel(0xE409,
	       priv->comphy_pipe_regs + A375_POWER_REG1_REG(comphy->index));

	writel(0xE008,
	       priv->comphy_pipe_regs + A375_POWER_REG1_REG(comphy->index));
}

static void phy_mvebu_comphy_sgmii_config(struct phy_mvebu_comphy_priv *priv,
					  struct mvebu_comphy *comphy)
{
	u32 reg;

	/* Set PHY mode to SGMII */
	reg = POWERUP_TX_BIT | POWERUP_RX_BIT | POWERUP_PLL_BIT |
		POWERUP_IVREF_BIT;
	reg |= RESERVED_BIT | UNKNOWN_BIT;
	reg |= PHY_MODE_SGMII << PHY_MODE_SHIFT;
	reg |= REF_FREF_SGMII_25 << REF_FREF_SEL_SHIFT;
	writel(reg,
	       priv->comphy_pipe_regs + A375_POWER_CONTROL_PLL_REG(comphy->index));

	/* SEL bits to 10 bits */
	writel(0x0,
	       priv->comphy_pipe_regs + A375_DIGITAL_LOOPBACK_EN_REG(comphy->index));

	/* Refclk SEL to 25 Mhz */
	writel(0x6417,
	       priv->comphy_pipe_regs + A375_MISC_CONTROL0_REG(comphy->index));

	/* Req_sq_de_glitch_en */
	writel(0xE,
	       priv->comphy_pipe_regs + A375_DIGITAL_RESERVED0_REG(comphy->index));

	/* Reset and unreset DFE sequence */
	writel(0xE409,
	       priv->comphy_pipe_regs + A375_POWER_REG1_REG(comphy->index));
	writel(0xE008,
	       priv->comphy_pipe_regs + A375_POWER_REG1_REG(comphy->index));

	/* Set PHY_GEN_TX/RX to 1.25Gbps */
	writel(0x166,
	       priv->comphy_pipe_regs + A375_PHY_ISOL_MODE_CTRL_REG(comphy->index));
}

static void phy_mvebu_comphy_config(struct phy_mvebu_comphy_priv *priv,
				    struct mvebu_comphy *comphy)
{
	switch(comphy->mode) {
	case COMPHY_PCIE00:
	case COMPHY_PCIE10:
		return phy_mvebu_comphy_pcie_config(priv, comphy);
	case COMPHY_SATA0:
	case COMPHY_SATA1:
		return phy_mvebu_comphy_sata_config(priv, comphy);
	case COMPHY_USB3H0:
		return phy_mvebu_comphy_usb3_config(priv, comphy);
	case COMPHY_SGMII0:
		return phy_mvebu_comphy_sgmii_config(priv, comphy);
	}
}

static void phy_mvebu_comphy_powerup(struct phy_mvebu_comphy_priv *priv,
				     struct mvebu_comphy *comphy)
{
	u32 reg;

	reg = readl(priv->comphy_regs + A375_COMPHY_CONFIG1_REG(comphy->index));
	reg |= (A375_POWER_UP_PLL_BIT | A375_POWER_UP_RX_BIT |
		A375_POWER_UP_TX_BIT);
	writel(reg, priv->comphy_regs + A375_COMPHY_CONFIG1_REG(comphy->index));
}

static void phy_mvebu_comphy_wait_ready(struct phy_mvebu_comphy_priv *priv,
					struct mvebu_comphy *comphy)
{
	u32 reg;
	int i;

	/* Nothing to do for PCIe/USB3 */
	if (comphy->mode == COMPHY_PCIE00 ||
	    comphy->mode == COMPHY_PCIE10 ||
	    comphy->mode == COMPHY_USB3H0)
		return;

	for (i = 0; i < 10; i++) {
		reg = readl(priv->comphy_pipe_regs +
			    A375_COMPHY_STATUS1_REG(comphy->index));
		if (reg & A375_PHY_MAC_STATUS_BIT &&
		    reg & A375_PLL_READY_RX_BIT)
			return;

		msleep(1);
	}

	dev_err(priv->dev, "COMPHY%d not ready\n", comphy->index);
}

static int phy_mvebu_comphy_power_on(struct phy *phy)
{
	struct mvebu_comphy *comphy = phy_get_drvdata(phy);
        struct phy_mvebu_comphy_priv *priv = to_phy_mvebu_comphy_priv(comphy);

	spin_lock(&priv->lock);

	phy_mvebu_comphy_reset(priv, comphy);
	phy_mvebu_comphy_select_mode(priv, comphy);
	phy_mvebu_comphy_phy_config(priv, comphy);
	phy_mvebu_comphy_unreset(priv, comphy);
	phy_mvebu_comphy_config(priv, comphy);
	phy_mvebu_comphy_powerup(priv, comphy);
	phy_mvebu_comphy_wait_ready(priv, comphy);

	spin_unlock(&priv->lock);

	return 0;
}

static int phy_mvebu_comphy_power_off(struct phy *phy)
{
	struct mvebu_comphy *comphy = phy_get_drvdata(phy);
        struct phy_mvebu_comphy_priv *priv = to_phy_mvebu_comphy_priv(comphy);
	u32 reg;

	spin_lock(&priv->lock);

	/* Power down PLL, TX, RX */
	reg = readl(priv->comphy_regs + A375_COMPHY_CONFIG1_REG(comphy->index));
	reg &= ~(A375_POWER_UP_PLL_BIT | A375_POWER_UP_RX_BIT |
		 A375_POWER_UP_TX_BIT  | A375_PIN_TX_IDLE_BIT);
	writel(reg, priv->comphy_regs + A375_COMPHY_CONFIG1_REG(comphy->index));

	spin_unlock(&priv->lock);

	msleep(10);

	return 0;
}

static struct phy_ops phy_mvebu_comphy_ops = {
	.power_on	= phy_mvebu_comphy_power_on,
	.power_off	= phy_mvebu_comphy_power_off,
	.owner		= THIS_MODULE,
};

static const struct phy_mvebu_comphy_soc_info mv88f6720_comphy = {
	.nlanes = 4,
	.functions = {
		{ COMPHY_PCIE00 },
		{ COMPHY_PCIE10, COMPHY_SATA1, COMPHY_SGMII0 },
		{ COMPHY_SATA0, COMPHY_SGMII0 },
		{ COMPHY_SGMII0, COMPHY_USB3H0 },
	},
};

static const struct phy_mvebu_comphy_soc_info mv88f6810_comphy = {
	.nlanes = 6,
	.functions = {
		{ COMPHY_PCIE00, COMPHY_SATA0, COMPHY_SGMII0 },
		{ COMPHY_PCIE00, COMPHY_SATA0, COMPHY_SGMII0,
		  COMPHY_SGMII1, COMPHY_USB3H0 },
		{ COMPHY_SATA1, COMPHY_SGMII1 },
		{ COMPHY_PCIE30, COMPHY_USB3H1, COMPHY_USB3D0 },
		{ },
		{ COMPHY_PCIE20, COMPHY_USB3H1, COMPHY_USB3D0 },
	},
};

static const struct phy_mvebu_comphy_soc_info mv88f6820_comphy = {
	.nlanes = 6,
	.functions = {
		{ COMPHY_PCIE00, COMPHY_SATA0, COMPHY_SGMII0 },
		{ COMPHY_PCIE00, COMPHY_PCIE01, COMPHY_SATA0, COMPHY_SGMII0,
		  COMPHY_SGMII1, COMPHY_USB3H0, COMPHY_QSGMII },
		{ COMPHY_PCIE10, COMPHY_PCIE02, COMPHY_SATA1, COMPHY_SGMII1 },
		{ COMPHY_PCIE30, COMPHY_PCIE03, COMPHY_SGMII2, COMPHY_USB3H1,
		  COMPHY_USB3D0 },
		{ COMPHY_PCIE10, COMPHY_PCIE20, COMPHY_SGMII1, COMPHY_USB3H0,
		  COMPHY_USB3D0 },
		{ COMPHY_PCIE20, COMPHY_SGMII2, COMPHY_USB3H1, COMPHY_USB3D0 },
	},
};

static const struct phy_mvebu_comphy_soc_info mv88f6828_comphy = {
	.nlanes = 6,
	.functions = {
		{ COMPHY_PCIE00, COMPHY_SATA0, COMPHY_SGMII0 },
		{ COMPHY_PCIE00, COMPHY_PCIE01, COMPHY_SATA0, COMPHY_SGMII0,
		  COMPHY_SGMII1, COMPHY_USB3H0, COMPHY_QSGMII },
		{ COMPHY_PCIE10, COMPHY_PCIE02, COMPHY_SATA1, COMPHY_SGMII1 },
		{ COMPHY_PCIE30, COMPHY_PCIE03, COMPHY_SATA3, COMPHY_SGMII2,
		  COMPHY_USB3H1, COMPHY_USB3D0 },
		{ COMPHY_PCIE10, COMPHY_PCIE20, COMPHY_SATA2, COMPHY_SGMII1,
		  COMPHY_USB3H0, COMPHY_USB3D0 },
		{ COMPHY_PCIE20, COMPHY_SATA2, COMPHY_SGMII2, COMPHY_USB3H1,
		  COMPHY_USB3D0 },
	},
};

static const struct phy_mvebu_comphy_soc_info mv88f6920_comphy = {
	.nlanes = 7,
	.functions = {
		{ COMPHY_PCIE00, COMPHY_SGMII0 },
		{ COMPHY_PCIE00, COMPHY_PCIE01, COMPHY_SGMII0, COMPHY_SGMII1,
		  COMPHY_USB3H0 },
		{ COMPHY_PCIE10, COMPHY_PCIE02, COMPHY_SGMII1 },
		{ COMPHY_PCIE30, COMPHY_PCIE03, COMPHY_SGMII2, COMPHY_XAUI3 },
		{ COMPHY_PCIE10, COMPHY_PCIE20, COMPHY_SGMII3, COMPHY_XAUI2,
		  COMPHY_USB3H0 },
		{ COMPHY_PCIE20, COMPHY_SGMII2, COMPHY_RXAUI1, COMPHY_XAUI1 },
		{ COMPHY_PCIE10, COMPHY_SGMII3, COMPHY_RXAUI0, COMPHY_SGMII0 },
	},
};

static const struct phy_mvebu_comphy_soc_info mv88f6928_comphy = {
	.nlanes = 7,
	.functions = {
		{ COMPHY_PCIE00, COMPHY_SATA0, COMPHY_SGMII0 },
		{ COMPHY_PCIE00, COMPHY_PCIE01, COMPHY_SATA0, COMPHY_SGMII0,
		  COMPHY_SGMII1, COMPHY_USB3H0 },
		{ COMPHY_PCIE10, COMPHY_PCIE02, COMPHY_SATA1, COMPHY_SGMII1 },
		{ COMPHY_PCIE30, COMPHY_PCIE03, COMPHY_SGMII2, COMPHY_XAUI3,
		  COMPHY_USB3H1 },
		{ COMPHY_PCIE10, COMPHY_PCIE20, COMPHY_SATA2, COMPHY_SGMII3,
		  COMPHY_XAUI2,  COMPHY_USB3H0 },
		{ COMPHY_PCIE20, COMPHY_SGMII2, COMPHY_RXAUI1, COMPHY_XAUI1,
		  COMPHY_USB3H1 },
		{ COMPHY_PCIE10, COMPHY_SGMII3, COMPHY_RXAUI0, COMPHY_SGMII0 },
	},
};

static const struct of_device_id phy_mvebu_comphy_of_match[] = {
	{ .compatible = "marvell,mv88f6720-comphy", .data = &mv88f6720_comphy },
	{ .compatible = "marvell,mv88f6810-comphy", .data = &mv88f6810_comphy },
	{ .compatible = "marvell,mv88f6820-comphy", .data = &mv88f6820_comphy },
	{ .compatible = "marvell,mv88f6828-comphy", .data = &mv88f6828_comphy },
	{ .compatible = "marvell,mv88f6920-comphy", .data = &mv88f6920_comphy },
	{ .compatible = "marvell,mv88f6928-comphy", .data = &mv88f6928_comphy },
	{ },
};
MODULE_DEVICE_TABLE(of, phy_mvebu_comphy_of_match);

static struct phy *phy_mvebu_comphy_of_xlate(struct device *dev,
					     struct of_phandle_args *args)
{
	struct phy_mvebu_comphy_priv *priv = dev_get_drvdata(dev);
	int lane = args->args[0];
	int mode = args->args[1];
	int i;

	if (lane >= priv->sinfo->nlanes) {
		dev_err(dev, "Wrong lane number %d for PHY, max is %d\n",
			lane, priv->sinfo->nlanes);
		return ERR_PTR(-ENODEV);
	}

	for (i = 0; i < MVEBU_COMPHY_FUNC_MAX; i++)
		if (priv->sinfo->functions[lane][i] == mode)
			break;

	if (i == MVEBU_COMPHY_FUNC_MAX) {
		dev_err(dev, "Wrong mode 0x%x for COMPHY\n", mode);
		return ERR_PTR(-ENODEV);
	}

	priv->lanes[lane].mode = mode;

	return priv->lanes[lane].phy;
}

static int phy_mvebu_comphy_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct phy_mvebu_comphy_priv *priv;
	struct resource *res;
	struct phy_provider *phy_provider;
	const struct phy_mvebu_comphy_soc_info *sinfo;
	int i;

	match = of_match_device(phy_mvebu_comphy_of_match, &pdev->dev);
	sinfo = match->data;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->comphy_regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->comphy_regs))
		return PTR_ERR(priv->comphy_regs);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	priv->comphy_pipe_regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->comphy_pipe_regs))
		return PTR_ERR(priv->comphy_pipe_regs);

	priv->sinfo = sinfo;
	priv->dev = &pdev->dev;
	spin_lock_init(&priv->lock);

	for (i = 0; i < sinfo->nlanes; i++) {
		struct phy *phy;

		phy = devm_phy_create(&pdev->dev, NULL, &phy_mvebu_comphy_ops);
		if (IS_ERR(phy)) {
			dev_err(&pdev->dev, "failed to create PHY\n");
			return PTR_ERR(phy);
		}

		priv->lanes[i].phy = phy;
		priv->lanes[i].index = i;
		priv->lanes[i].mode = COMPHY_UNUSED;
		phy_set_drvdata(phy, &priv->lanes[i]);

		phy_mvebu_comphy_power_off(phy);
	}

	platform_set_drvdata(pdev, priv);

	phy_provider = devm_of_phy_provider_register(&pdev->dev,
						     phy_mvebu_comphy_of_xlate);

	return PTR_ERR_OR_ZERO(phy_provider);
}

static struct platform_driver phy_mvebu_comphy_driver = {
	.probe	= phy_mvebu_comphy_probe,
	.driver	= {
		.name		= "phy-mvebu-comphy",
		.owner		= THIS_MODULE,
		.of_match_table	= phy_mvebu_comphy_of_match,
	 },
};
module_platform_driver(phy_mvebu_comphy_driver);

MODULE_AUTHOR("Thomas Petazzoni <thomas.petazzoni@free-electrons.com>");
MODULE_DESCRIPTION("Marvell EBU COMPHY driver");
MODULE_LICENSE("GPL");
