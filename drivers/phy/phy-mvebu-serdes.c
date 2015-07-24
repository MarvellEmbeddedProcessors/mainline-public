/*
 * Copyright (C) 2015 Marvell
 *
 * Thomas Petazzoni <thomas.petazzoni@free-electrons.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <dt-bindings/phy/phy-mvebu-serdes.h>

#define MVEBU_SERDES_MAX_CNT	16
#define MVEBU_SERDES_FUNC_MAX   16

struct phy_mvebu_serdes_soc_info {
	int nlanes;
	int functions[MVEBU_SERDES_MAX_CNT][MVEBU_SERDES_FUNC_MAX];
};

struct phy_mvebu_serdes_priv {
	void __iomem *base;
	spinlock_t lock;
	const struct phy_mvebu_serdes_soc_info *sinfo;
	struct mvebu_serdes_lane {
		struct phy *phy;
		int index;
		int mode;
	} lanes[MVEBU_SERDES_MAX_CNT];
};

static void phy_mvebu_set_lane_mode(struct phy_mvebu_serdes_priv *priv,
				    int lane, int mode)
{
	int offset = (lane / 8) * 4;
	int shift = (lane % 8) * 4;
	int modeval = -1, i;
	u32 reg;

	/*
	 * No need for error checking, the xlate function has already
	 * verified that the mode is valid.
	 */
	for (i = 0; i < MVEBU_SERDES_FUNC_MAX; i++)
		if (priv->sinfo->functions[lane][i] == mode)
			modeval = i;

	if (modeval == -1)
		return;

	spin_lock(&priv->lock);

	reg = readl(priv->base + offset);
	reg &= ~(0xF << shift);
	reg |= modeval << shift;
	writel(reg, priv->base + offset);

	spin_unlock(&priv->lock);
}

#define to_phy_mvebu_serdes_phy(lane) \
        container_of((lane), struct phy_mvebu_serdes_priv, lanes[(lane)->index])

static int phy_mvebu_serdes_power_on(struct phy *phy)
{
        struct mvebu_serdes_lane *phy_lane = phy_get_drvdata(phy);
        struct phy_mvebu_serdes_priv *priv = to_phy_mvebu_serdes_phy(phy_lane);

	phy_mvebu_set_lane_mode(priv, phy_lane->index, phy_lane->mode);

	return 0;
}

static int phy_mvebu_serdes_power_off(struct phy *phy)
{
	struct mvebu_serdes_lane *phy_lane = phy_get_drvdata(phy);
        struct phy_mvebu_serdes_priv *priv = to_phy_mvebu_serdes_phy(phy_lane);

	phy_mvebu_set_lane_mode(priv, phy_lane->index, SERDES_UNCONNECTED);

	return 0;
}

static struct phy_ops phy_mvebu_serdes_ops = {
	.power_on	= phy_mvebu_serdes_power_on,
	.power_off	= phy_mvebu_serdes_power_off,
	.owner		= THIS_MODULE,
};

static const struct phy_mvebu_serdes_soc_info mv88f6710_serdes = {
	.nlanes = 4,
	.functions = {
		{ SERDES_UNCONNECTED, SERDES_PCIE00, SERDES_SATA0, SERDES_SGMII1 },
		{ SERDES_UNCONNECTED, SERDES_PCIE10, SERDES_SGMII0 },
		{ SERDES_UNCONNECTED, SERDES_SATA0, SERDES_SGMII0 },
		{ SERDES_UNCONNECTED, SERDES_SATA1, SERDES_SGMII1 },
	},
};

static const struct phy_mvebu_serdes_soc_info mv78230_serdes = {
	.nlanes = 7,
	.functions = {
		{ SERDES_UNCONNECTED, SERDES_PCIE00 },
		{ SERDES_UNCONNECTED, SERDES_PCIE01 },
		{ SERDES_UNCONNECTED, SERDES_PCIE02, SERDES_SGMII0 },
		{ SERDES_UNCONNECTED, SERDES_PCIE03, SERDES_SGMII1, SERDES_QSGMII },
		{ SERDES_UNCONNECTED, SERDES_PCIE10, SERDES_SATA0, SERDES_SGMII2 },
		{ SERDES_UNCONNECTED, 0, SERDES_SATA1, SERDES_SGMII1, SERDES_QSGMII },
		{ SERDES_UNCONNECTED, 0, SERDES_SATA0, SERDES_SGMII2 },
	},
};

static const struct phy_mvebu_serdes_soc_info mv78260_serdes = {
	.nlanes = 12,
	.functions = {
		{ SERDES_UNCONNECTED, SERDES_PCIE00 },
		{ SERDES_UNCONNECTED, SERDES_PCIE01 },
		{ SERDES_UNCONNECTED, SERDES_PCIE02, SERDES_SGMII0 },
		{ SERDES_UNCONNECTED, SERDES_PCIE03, SERDES_SGMII1, SERDES_QSGMII },
		{ SERDES_UNCONNECTED, SERDES_PCIE10, SERDES_SATA0, SERDES_SGMII2 },
		{ SERDES_UNCONNECTED, SERDES_PCIE11, SERDES_SATA1, SERDES_SGMII1, SERDES_QSGMII },
		{ SERDES_UNCONNECTED, SERDES_PCIE12, SERDES_SATA0, SERDES_SGMII2, SERDES_SGMII0 },
		{ SERDES_UNCONNECTED, SERDES_PCIE13, SERDES_SGMII0, SERDES_SGMII3 },
		{ SERDES_UNCONNECTED, SERDES_PCIE20 },
		{ SERDES_UNCONNECTED, SERDES_PCIE21 },
		{ SERDES_UNCONNECTED, SERDES_PCIE22 },
		{ SERDES_UNCONNECTED, SERDES_PCIE23 },
	},
};

static const struct phy_mvebu_serdes_soc_info mv78460_serdes = {
	.nlanes = 16,
	.functions = {
		{ SERDES_UNCONNECTED, SERDES_PCIE00 },
		{ SERDES_UNCONNECTED, SERDES_PCIE01 },
		{ SERDES_UNCONNECTED, SERDES_PCIE02, SERDES_SGMII0 },
		{ SERDES_UNCONNECTED, SERDES_PCIE03, SERDES_SGMII1, SERDES_QSGMII },
		{ SERDES_UNCONNECTED, SERDES_PCIE10, SERDES_SATA0, SERDES_SGMII2 },
		{ SERDES_UNCONNECTED, SERDES_PCIE11, SERDES_SATA1, SERDES_SGMII1, SERDES_QSGMII },
		{ SERDES_UNCONNECTED, SERDES_PCIE12, SERDES_SATA0, SERDES_SGMII2, SERDES_SGMII0 },
		{ SERDES_UNCONNECTED, SERDES_PCIE13, SERDES_SGMII0, SERDES_SGMII3 },
		{ SERDES_UNCONNECTED, SERDES_PCIE20 },
		{ SERDES_UNCONNECTED, SERDES_PCIE21 },
		{ SERDES_UNCONNECTED, SERDES_PCIE22 },
		{ SERDES_UNCONNECTED, SERDES_PCIE23 },
		{ SERDES_UNCONNECTED, SERDES_PCIE30 },
		{ SERDES_UNCONNECTED, SERDES_PCIE31 },
		{ SERDES_UNCONNECTED, SERDES_PCIE32 },
		{ SERDES_UNCONNECTED, SERDES_PCIE33 },
	},
};

static const struct of_device_id phy_mvebu_serdes_of_match[] = {
	{ .compatible = "marvell,mv88f6710-serdes", .data = &mv88f6710_serdes },
	{ .compatible = "marvell,mv78230-serdes", .data = &mv78230_serdes },
	{ .compatible = "marvell,mv78260-serdes", .data = &mv78260_serdes },
	{ .compatible = "marvell,mv78460-serdes", .data = &mv78460_serdes },
	{ },
};
MODULE_DEVICE_TABLE(of, phy_mvebu_serdes_of_match);

static struct phy *phy_mvebu_serdes_of_xlate(struct device *dev,
					     struct of_phandle_args *args)
{
	struct phy_mvebu_serdes_priv *priv = dev_get_drvdata(dev);
	int lane = args->args[0];
	int mode = args->args[1];
	int i;

	if (lane >= priv->sinfo->nlanes)
		return ERR_PTR(-ENODEV);

	for (i = 0; i < MVEBU_SERDES_FUNC_MAX; i++)
		if (priv->sinfo->functions[lane][i] == mode)
			break;

	if (i == MVEBU_SERDES_FUNC_MAX)
		return ERR_PTR(-ENODEV);

	priv->lanes[lane].mode = mode;

	return priv->lanes[lane].phy;
}

static int phy_mvebu_serdes_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct phy_mvebu_serdes_priv *priv;
	struct resource *res;
	struct phy_provider *phy_provider;
	const struct phy_mvebu_serdes_soc_info *sinfo;
	int i;

	match = of_match_device(phy_mvebu_serdes_of_match, &pdev->dev);
	sinfo = match->data;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->sinfo = sinfo;
	spin_lock_init(&priv->lock);

	for (i = 0; i < sinfo->nlanes; i++) {
		struct phy *phy;

		phy = devm_phy_create(&pdev->dev, NULL, &phy_mvebu_serdes_ops);
		if (IS_ERR(phy)) {
			dev_err(&pdev->dev, "failed to create PHY\n");
			return PTR_ERR(phy);
		}

		phy_mvebu_set_lane_mode(priv, i, SERDES_UNCONNECTED);

		priv->lanes[i].phy = phy;
		priv->lanes[i].index = i;
		priv->lanes[i].mode = SERDES_UNCONNECTED;

		phy_set_drvdata(phy, &priv->lanes[i]);
	}

	platform_set_drvdata(pdev, priv);

	phy_provider = devm_of_phy_provider_register(&pdev->dev,
						     phy_mvebu_serdes_of_xlate);

	return PTR_ERR_OR_ZERO(phy_provider);
}

static struct platform_driver phy_mvebu_serdes_driver = {
	.probe	= phy_mvebu_serdes_probe,
	.driver	= {
		.name		= "phy-mvebu-serdes",
		.owner		= THIS_MODULE,
		.of_match_table	= phy_mvebu_serdes_of_match,
	 },
};
module_platform_driver(phy_mvebu_serdes_driver);

MODULE_AUTHOR("Thomas Petazzoni <thomas.petazzoni@free-electrons.com>");
MODULE_DESCRIPTION("Marvell EBU SERDES driver");
MODULE_LICENSE("GPL");
