/*
 * PCIe driver for Marvell Armada 370 and Armada XP SoCs
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/mbus.h>
#include <linux/msi.h>
#include <linux/slab.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/mvebu-system-controller.h>
#include <linux/regmap.h>

/*
 * PCIe unit register offsets.
 */
#define PCIE_DEV_ID_OFF		0x0000
#define   PCIE_DEV_ID_SHIFT		16
#define   PCIE_DEV_ID_MASK		(0xffff << PCIE_DEV_ID_SHIFT)
#define PCIE_CMD_OFF		0x0004
#define PCIE_DEV_REV_OFF	0x0008
#define PCIE_BAR_LO_OFF(n)	(0x0010 + ((n) << 3))
#define PCIE_BAR_HI_OFF(n)	(0x0014 + ((n) << 3))
#define PCIE_CAPABILITIES_OFF	0x0060
#define   PCIE_CAP_DEVTYPE_MASK	(0xF << 20)
#define	  PCIE_CAP_DEVTYPE_RC	(0x4 << 20)
#define   PCIE_CAP_DEVTYPE_EP	(0x1 << 20)
#define PCIE_LINK_CAPS_OFF		0x006c
#define   PCIE_LINK_CAP_SPD_MASK	0xf
#define   PCIE_LINK_CAP_SPD_2_5G	0x1
#define   PCIE_LINK_CAP_SPD_5G		0x2
#define   PCIE_LINK_CAP_WIDTH_SHIFT	4
#define   PCIE_LINK_CAP_WIDTH_MASK	0x3f0
#define	  PCIE_LINK_CAP_WIDTH_1		(1 << PCIE_LINK_CAP_WIDTH_SHIFT)
#define   PCIE_LINK_CAP_WIDTH_4		(4 << PCIE_LINK_CAP_WIDTH_SHIFT)
#define PCIE_LINK_CTRL_STAT_OFF		0x0070
#define   PCIE_LINK_CTRL_COMMON_CLK	BIT(6)
#define   PCIE_LINK_CTRL_SPD_SHIFT	16
#define   PCIE_LINK_CTRL_SPD_MASK	(0xf << PCIE_LINK_CTRL_SPD_SHIFT)
#define   PCIE_LINK_CTRL_SPD_2_5G	(0x1 << PCIE_LINK_CTRL_SPD_SHIFT)
#define   PCIE_LINK_CTRL_SPD_5G		(0x2 << PCIE_LINK_CTRL_SPD_SHIFT)
#define   PCIE_LINK_CTRL_WIDTH_SHIFT	20
#define   PCIE_LINK_CTRL_WIDTH_MASK	(0x3f << PCIE_LINK_CTRL_WIDTH_SHIFT)
#define PCIE_LINK_CTRL_STAT2_OFF	0x0090
#define   PCIE_LINK_CTRL2_SPD_MASK	0xf
#define   PCIE_LINK_CTRL2_SPD_2_5G	0x1
#define   PCIE_LINK_CTRL2_SPD_5G	0x2
#define PCIE_HEADER_LOG_4_OFF	0x0128
#define PCIE_BAR_CTRL_OFF(n)	(0x1804 + (((n) - 1) * 4))
#define PCIE_WIN04_CTRL_OFF(n)	(0x1820 + ((n) << 4))
#define PCIE_WIN04_BASE_OFF(n)	(0x1824 + ((n) << 4))
#define PCIE_WIN04_REMAP_OFF(n)	(0x182c + ((n) << 4))
#define PCIE_WIN5_CTRL_OFF	0x1880
#define PCIE_WIN5_BASE_OFF	0x1884
#define PCIE_WIN5_REMAP_OFF	0x188c
#define PCIE_CONF_ADDR_OFF	0x18f8
#define  PCIE_CONF_ADDR_EN		0x80000000
#define  PCIE_CONF_REG(r)		((((r) & 0xf00) << 16) | ((r) & 0xfc))
#define  PCIE_CONF_BUS(b)		(((b) & 0xff) << 16)
#define  PCIE_CONF_DEV(d)		(((d) & 0x1f) << 11)
#define  PCIE_CONF_FUNC(f)		(((f) & 0x7) << 8)
#define  PCIE_CONF_ADDR(bus, devfn, where) \
	(PCIE_CONF_BUS(bus) | PCIE_CONF_DEV(PCI_SLOT(devfn))    | \
	 PCIE_CONF_FUNC(PCI_FUNC(devfn)) | PCIE_CONF_REG(where) | \
	 PCIE_CONF_ADDR_EN)
#define PCIE_CONF_DATA_OFF	0x18fc
#define PCIE_MASK_OFF		0x1910
#define  PCIE_MASK_ENABLE_INTS          0x0f000000
#define PCIE_CTRL_OFF		0x1a00
#define  PCIE_CTRL_X1_MODE		0x0001
#define  PCIE_CTRL_CONF_AUTOSPEED	BIT(10)
#define PCIE_STAT_OFF		0x1a04
#define  PCIE_STAT_BUS                  0xff00
#define  PCIE_STAT_DEV                  0x1f0000
#define  PCIE_STAT_LINK_DOWN		BIT(0)
#define PCIE_DEBUG_CTRL         0x1a60
#define  PCIE_DEBUG_SOFT_RESET		BIT(20)
#define PCIE_DEBUG_STATUS_OFF	0x1a64
#define PCIE_PHY_ACCESS_OFF	0x1b00
#define  PCIE_PHY_DATA_SHIFT		0
#define  PCIE_PHY_OFFSET_SHIFT		16
#define  PCIE_PHY_LANE_SHIFT		24
#define  PCIE_PHY_RD_ACCESS		BIT(31)

/*
 * PCIe PHY registers, accessed from the indirect PCIE_PHY_ACCESS_OFF
 * register.
 */
#define GLOB_CLK_CTRL		0xc1
#define GLOB_TEST_CTRL		0xc2
#define GLOB_CLK_SRC_LO		0xc3

/* PCI configuration space of a PCI-to-PCI bridge */
struct mvebu_sw_pci_bridge {
	u16 vendor;
	u16 device;
	u16 command;
	u16 class;
	u8 interface;
	u8 revision;
	u8 bist;
	u8 header_type;
	u8 latency_timer;
	u8 cache_line_size;
	u32 bar[2];
	u8 primary_bus;
	u8 secondary_bus;
	u8 subordinate_bus;
	u8 secondary_latency_timer;
	u8 iobase;
	u8 iolimit;
	u16 secondary_status;
	u16 membase;
	u16 memlimit;
	u16 iobaseupper;
	u16 iolimitupper;
	u8 cappointer;
	u8 reserved1;
	u16 reserved2;
	u32 romaddr;
	u8 intline;
	u8 intpin;
	u16 bridgectrl;
};

enum mvebu_pcie_variant {
	PCIE_VARIANT_AXP = 0x1,
	PCIE_VARIANT_A370,
	PCIE_VARIANT_A375,
	PCIE_VARIANT_A38X,
	PCIE_VARIANT_A39X,
	PCIE_VARIANT_KIRKWOOD,
	PCIE_VARIANT_DOVE,
};

struct mvebu_pcie_port;

/* Structure representing all PCIe interfaces */
struct mvebu_pcie {
	struct platform_device *pdev;
	struct mvebu_pcie_port *ports;
	struct msi_controller *msi;
	struct regmap *syscon_regmap;
	struct resource io;
	struct resource realio;
	struct resource mem;
	struct resource busn;
	enum mvebu_pcie_variant variant;
	int nports;
};

/* Structure representing one PCIe interface */
struct mvebu_pcie_port {
	char *name;
	void __iomem *base;
	u32 port;
	u32 lane;
	int nlanes;
	int devfn;
	unsigned int mem_target;
	unsigned int mem_attr;
	unsigned int io_target;
	unsigned int io_attr;
	struct clk *clk;
	int reset_gpio;
	int reset_active_low;
	char *reset_name;
	struct mvebu_sw_pci_bridge bridge;
	struct device_node *dn;
	struct mvebu_pcie *pcie;
	phys_addr_t memwin_base;
	size_t memwin_size;
	phys_addr_t iowin_base;
	size_t iowin_size;
	u32 saved_pcie_stat;
};

static inline void mvebu_writel(struct mvebu_pcie_port *port, u32 val, u32 reg)
{
	writel(val, port->base + reg);
}

static inline u32 mvebu_readl(struct mvebu_pcie_port *port, u32 reg)
{
	return readl(port->base + reg);
}

static void mvebu_phy_write(struct mvebu_pcie_port *port, int lane, int offset, u16 data)
{
	u32 value;

	value = (lane << PCIE_PHY_LANE_SHIFT) |
		(offset << PCIE_PHY_OFFSET_SHIFT) |
		(data << PCIE_PHY_DATA_SHIFT);
	mvebu_writel(port, value, PCIE_PHY_ACCESS_OFF);
}

static u16 mvebu_phy_read(struct mvebu_pcie_port *port, int lane, int offset)
{
	u32 value;

	value = PCIE_PHY_RD_ACCESS |
		(lane << PCIE_PHY_LANE_SHIFT) |
		(offset << PCIE_PHY_OFFSET_SHIFT);
	mvebu_writel(port, value, PCIE_PHY_ACCESS_OFF);

	return mvebu_readl(port, PCIE_PHY_ACCESS_OFF) & 0xFFFF;
}

static inline bool mvebu_has_ioport(struct mvebu_pcie_port *port)
{
	return port->io_target != -1 && port->io_attr != -1;
}

static bool mvebu_pcie_link_up(struct mvebu_pcie_port *port)
{
	return !(mvebu_readl(port, PCIE_STAT_OFF) & PCIE_STAT_LINK_DOWN);
}

static void mvebu_pcie_set_local_bus_nr(struct mvebu_pcie_port *port, int nr)
{
	u32 stat;

	stat = mvebu_readl(port, PCIE_STAT_OFF);
	stat &= ~PCIE_STAT_BUS;
	stat |= nr << 8;
	mvebu_writel(port, stat, PCIE_STAT_OFF);
}

static void mvebu_pcie_set_local_dev_nr(struct mvebu_pcie_port *port, int nr)
{
	u32 stat;

	stat = mvebu_readl(port, PCIE_STAT_OFF);
	stat &= ~PCIE_STAT_DEV;
	stat |= nr << 16;
	mvebu_writel(port, stat, PCIE_STAT_OFF);
}

/*
 * Setup PCIE BARs and Address Decode Wins:
 * BAR[0,2] -> disabled, BAR[1] -> covers all DRAM banks
 * WIN[0-3] -> DRAM bank[0-3]
 */
static void mvebu_pcie_setup_wins(struct mvebu_pcie_port *port)
{
	const struct mbus_dram_target_info *dram;
	u32 size;
	int i;

	dram = mv_mbus_dram_info();

	/* First, disable and clear BARs and windows. */
	for (i = 1; i < 3; i++) {
		mvebu_writel(port, 0, PCIE_BAR_CTRL_OFF(i));
		mvebu_writel(port, 0, PCIE_BAR_LO_OFF(i));
		mvebu_writel(port, 0, PCIE_BAR_HI_OFF(i));
	}

	for (i = 0; i < 5; i++) {
		mvebu_writel(port, 0, PCIE_WIN04_CTRL_OFF(i));
		mvebu_writel(port, 0, PCIE_WIN04_BASE_OFF(i));
		mvebu_writel(port, 0, PCIE_WIN04_REMAP_OFF(i));
	}

	mvebu_writel(port, 0, PCIE_WIN5_CTRL_OFF);
	mvebu_writel(port, 0, PCIE_WIN5_BASE_OFF);
	mvebu_writel(port, 0, PCIE_WIN5_REMAP_OFF);

	/* Setup windows for DDR banks.  Count total DDR size on the fly. */
	size = 0;
	for (i = 0; i < dram->num_cs; i++) {
		const struct mbus_dram_window *cs = dram->cs + i;

		mvebu_writel(port, cs->base & 0xffff0000,
			     PCIE_WIN04_BASE_OFF(i));
		mvebu_writel(port, 0, PCIE_WIN04_REMAP_OFF(i));
		mvebu_writel(port,
			     ((cs->size - 1) & 0xffff0000) |
			     (cs->mbus_attr << 8) |
			     (dram->mbus_dram_target_id << 4) | 1,
			     PCIE_WIN04_CTRL_OFF(i));

		size += cs->size;
	}

	/* Round up 'size' to the nearest power of two. */
	if ((size & (size - 1)) != 0)
		size = 1 << fls(size);

	/* Setup BAR[1] to all DRAM banks. */
	mvebu_writel(port, dram->cs[0].base, PCIE_BAR_LO_OFF(1));
	mvebu_writel(port, 0, PCIE_BAR_HI_OFF(1));
	mvebu_writel(port, ((size - 1) & 0xffff0000) | 1,
		     PCIE_BAR_CTRL_OFF(1));
}

static void mvebu_pcie_setup_hw(struct mvebu_pcie_port *port)
{
	u32 cmd, mask;

	/* Point PCIe unit MBUS decode windows to DRAM space. */
	mvebu_pcie_setup_wins(port);

	/* Master + slave enable. */
	cmd = mvebu_readl(port, PCIE_CMD_OFF);
	cmd |= PCI_COMMAND_IO;
	cmd |= PCI_COMMAND_MEMORY;
	cmd |= PCI_COMMAND_MASTER;
	mvebu_writel(port, cmd, PCIE_CMD_OFF);

	/* Enable interrupt lines A-D. */
	mask = mvebu_readl(port, PCIE_MASK_OFF);
	mask |= PCIE_MASK_ENABLE_INTS;
	mvebu_writel(port, mask, PCIE_MASK_OFF);
}

static int mvebu_pcie_hw_rd_conf(struct mvebu_pcie_port *port,
				 struct pci_bus *bus,
				 u32 devfn, int where, int size, u32 *val)
{
	mvebu_writel(port, PCIE_CONF_ADDR(bus->number, devfn, where),
		     PCIE_CONF_ADDR_OFF);

	*val = mvebu_readl(port, PCIE_CONF_DATA_OFF);

	if (size == 1)
		*val = (*val >> (8 * (where & 3))) & 0xff;
	else if (size == 2)
		*val = (*val >> (8 * (where & 3))) & 0xffff;

	return PCIBIOS_SUCCESSFUL;
}

static int mvebu_pcie_hw_wr_conf(struct mvebu_pcie_port *port,
				 struct pci_bus *bus,
				 u32 devfn, int where, int size, u32 val)
{
	u32 _val, shift = 8 * (where & 3);

	mvebu_writel(port, PCIE_CONF_ADDR(bus->number, devfn, where),
		     PCIE_CONF_ADDR_OFF);
	_val = mvebu_readl(port, PCIE_CONF_DATA_OFF);

	if (size == 4)
		_val = val;
	else if (size == 2)
		_val = (_val & ~(0xffff << shift)) | ((val & 0xffff) << shift);
	else if (size == 1)
		_val = (_val & ~(0xff << shift)) | ((val & 0xff) << shift);
	else
		return PCIBIOS_BAD_REGISTER_NUMBER;

	mvebu_writel(port, _val, PCIE_CONF_DATA_OFF);

	return PCIBIOS_SUCCESSFUL;
}

/*
 * Remove windows, starting from the largest ones to the smallest
 * ones.
 */
static void mvebu_pcie_del_windows(struct mvebu_pcie_port *port,
				   phys_addr_t base, size_t size)
{
	while (size) {
		size_t sz = 1 << (fls(size) - 1);

		mvebu_mbus_del_window(base, sz);
		base += sz;
		size -= sz;
	}
}

/*
 * MBus windows can only have a power of two size, but PCI BARs do not
 * have this constraint. Therefore, we have to split the PCI BAR into
 * areas each having a power of two size. We start from the largest
 * one (i.e highest order bit set in the size).
 */
static void mvebu_pcie_add_windows(struct mvebu_pcie_port *port,
				   unsigned int target, unsigned int attribute,
				   phys_addr_t base, size_t size,
				   phys_addr_t remap)
{
	size_t size_mapped = 0;

	while (size) {
		size_t sz = 1 << (fls(size) - 1);
		int ret;

		ret = mvebu_mbus_add_window_remap_by_id(target, attribute, base,
							sz, remap);
		if (ret) {
			phys_addr_t end = base + sz - 1;

			dev_err(&port->pcie->pdev->dev,
				"Could not create MBus window at [mem %pa-%pa]: %d\n",
				&base, &end, ret);
			mvebu_pcie_del_windows(port, base - size_mapped,
					       size_mapped);
			return;
		}

		size -= sz;
		size_mapped += sz;
		base += sz;
		if (remap != MVEBU_MBUS_NO_REMAP)
			remap += sz;
	}
}

static void mvebu_pcie_handle_iobase_change(struct mvebu_pcie_port *port)
{
	phys_addr_t iobase;

	/* Are the new iobase/iolimit values invalid? */
	if (port->bridge.iolimit < port->bridge.iobase ||
	    port->bridge.iolimitupper < port->bridge.iobaseupper ||
	    !(port->bridge.command & PCI_COMMAND_IO)) {

		/* If a window was configured, remove it */
		if (port->iowin_base) {
			mvebu_pcie_del_windows(port, port->iowin_base,
					       port->iowin_size);
			port->iowin_base = 0;
			port->iowin_size = 0;
		}

		return;
	}

	if (!mvebu_has_ioport(port)) {
		dev_WARN(&port->pcie->pdev->dev,
			 "Attempt to set IO when IO is disabled\n");
		return;
	}

	/*
	 * We read the PCI-to-PCI bridge emulated registers, and
	 * calculate the base address and size of the address decoding
	 * window to setup, according to the PCI-to-PCI bridge
	 * specifications. iobase is the bus address, port->iowin_base
	 * is the CPU address.
	 */
	iobase = ((port->bridge.iobase & 0xF0) << 8) |
		(port->bridge.iobaseupper << 16);
	port->iowin_base = port->pcie->io.start + iobase;
	port->iowin_size = ((0xFFF | ((port->bridge.iolimit & 0xF0) << 8) |
			    (port->bridge.iolimitupper << 16)) -
			    iobase) + 1;

	mvebu_pcie_add_windows(port, port->io_target, port->io_attr,
			       port->iowin_base, port->iowin_size,
			       iobase);
}

static void mvebu_pcie_handle_membase_change(struct mvebu_pcie_port *port)
{
	/* Are the new membase/memlimit values invalid? */
	if (port->bridge.memlimit < port->bridge.membase ||
	    !(port->bridge.command & PCI_COMMAND_MEMORY)) {

		/* If a window was configured, remove it */
		if (port->memwin_base) {
			mvebu_pcie_del_windows(port, port->memwin_base,
					       port->memwin_size);
			port->memwin_base = 0;
			port->memwin_size = 0;
		}

		return;
	}

	/*
	 * We read the PCI-to-PCI bridge emulated registers, and
	 * calculate the base address and size of the address decoding
	 * window to setup, according to the PCI-to-PCI bridge
	 * specifications.
	 */
	port->memwin_base  = ((port->bridge.membase & 0xFFF0) << 16);
	port->memwin_size  =
		(((port->bridge.memlimit & 0xFFF0) << 16) | 0xFFFFF) -
		port->memwin_base + 1;

	mvebu_pcie_add_windows(port, port->mem_target, port->mem_attr,
			       port->memwin_base, port->memwin_size,
			       MVEBU_MBUS_NO_REMAP);
}

/*
 * Initialize the configuration space of the PCI-to-PCI bridge
 * associated with the given PCIe interface.
 */
static void mvebu_sw_pci_bridge_init(struct mvebu_pcie_port *port)
{
	struct mvebu_sw_pci_bridge *bridge = &port->bridge;

	memset(bridge, 0, sizeof(struct mvebu_sw_pci_bridge));

	bridge->class = PCI_CLASS_BRIDGE_PCI;
	bridge->vendor = PCI_VENDOR_ID_MARVELL;
	bridge->device = mvebu_readl(port, PCIE_DEV_ID_OFF) >> 16;
	bridge->revision = mvebu_readl(port, PCIE_DEV_REV_OFF) & 0xff;
	bridge->header_type = PCI_HEADER_TYPE_BRIDGE;
	bridge->cache_line_size = 0x10;

	/* We support 32 bits I/O addressing */
	bridge->iobase = PCI_IO_RANGE_TYPE_32;
	bridge->iolimit = PCI_IO_RANGE_TYPE_32;
}

/*
 * Read the configuration space of the PCI-to-PCI bridge associated to
 * the given PCIe interface.
 */
static int mvebu_sw_pci_bridge_read(struct mvebu_pcie_port *port,
				  unsigned int where, int size, u32 *value)
{
	struct mvebu_sw_pci_bridge *bridge = &port->bridge;

	switch (where & ~3) {
	case PCI_VENDOR_ID:
		*value = bridge->device << 16 | bridge->vendor;
		break;

	case PCI_COMMAND:
		*value = bridge->command;
		break;

	case PCI_CLASS_REVISION:
		*value = bridge->class << 16 | bridge->interface << 8 |
			 bridge->revision;
		break;

	case PCI_CACHE_LINE_SIZE:
		*value = bridge->bist << 24 | bridge->header_type << 16 |
			 bridge->latency_timer << 8 | bridge->cache_line_size;
		break;

	case PCI_BASE_ADDRESS_0 ... PCI_BASE_ADDRESS_1:
		*value = bridge->bar[((where & ~3) - PCI_BASE_ADDRESS_0) / 4];
		break;

	case PCI_PRIMARY_BUS:
		*value = (bridge->secondary_latency_timer << 24 |
			  bridge->subordinate_bus         << 16 |
			  bridge->secondary_bus           <<  8 |
			  bridge->primary_bus);
		break;

	case PCI_IO_BASE:
		if (!mvebu_has_ioport(port))
			*value = bridge->secondary_status << 16;
		else
			*value = (bridge->secondary_status << 16 |
				  bridge->iolimit          <<  8 |
				  bridge->iobase);
		break;

	case PCI_MEMORY_BASE:
		*value = (bridge->memlimit << 16 | bridge->membase);
		break;

	case PCI_PREF_MEMORY_BASE:
		*value = 0;
		break;

	case PCI_IO_BASE_UPPER16:
		*value = (bridge->iolimitupper << 16 | bridge->iobaseupper);
		break;

	case PCI_ROM_ADDRESS1:
		*value = 0;
		break;

	case PCI_INTERRUPT_LINE:
		/* LINE PIN MIN_GNT MAX_LAT */
		*value = 0;
		break;

	default:
		*value = 0xffffffff;
		return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	if (size == 2)
		*value = (*value >> (8 * (where & 3))) & 0xffff;
	else if (size == 1)
		*value = (*value >> (8 * (where & 3))) & 0xff;

	return PCIBIOS_SUCCESSFUL;
}

/* Write to the PCI-to-PCI bridge configuration space */
static int mvebu_sw_pci_bridge_write(struct mvebu_pcie_port *port,
				     unsigned int where, int size, u32 value)
{
	struct mvebu_sw_pci_bridge *bridge = &port->bridge;
	u32 mask, reg;
	int err;

	if (size == 4)
		mask = 0x0;
	else if (size == 2)
		mask = ~(0xffff << ((where & 3) * 8));
	else if (size == 1)
		mask = ~(0xff << ((where & 3) * 8));
	else
		return PCIBIOS_BAD_REGISTER_NUMBER;

	err = mvebu_sw_pci_bridge_read(port, where & ~3, 4, &reg);
	if (err)
		return err;

	value = (reg & mask) | value << ((where & 3) * 8);

	switch (where & ~3) {
	case PCI_COMMAND:
	{
		u32 old = bridge->command;

		if (!mvebu_has_ioport(port))
			value &= ~PCI_COMMAND_IO;

		bridge->command = value & 0xffff;
		if ((old ^ bridge->command) & PCI_COMMAND_IO)
			mvebu_pcie_handle_iobase_change(port);
		if ((old ^ bridge->command) & PCI_COMMAND_MEMORY)
			mvebu_pcie_handle_membase_change(port);
		break;
	}

	case PCI_BASE_ADDRESS_0 ... PCI_BASE_ADDRESS_1:
		bridge->bar[((where & ~3) - PCI_BASE_ADDRESS_0) / 4] = value;
		break;

	case PCI_IO_BASE:
		/*
		 * We also keep bit 1 set, it is a read-only bit that
		 * indicates we support 32 bits addressing for the
		 * I/O
		 */
		bridge->iobase = (value & 0xff) | PCI_IO_RANGE_TYPE_32;
		bridge->iolimit = ((value >> 8) & 0xff) | PCI_IO_RANGE_TYPE_32;
		mvebu_pcie_handle_iobase_change(port);
		break;

	case PCI_MEMORY_BASE:
		bridge->membase = value & 0xffff;
		bridge->memlimit = value >> 16;
		mvebu_pcie_handle_membase_change(port);
		break;

	case PCI_IO_BASE_UPPER16:
		bridge->iobaseupper = value & 0xffff;
		bridge->iolimitupper = value >> 16;
		mvebu_pcie_handle_iobase_change(port);
		break;

	case PCI_PRIMARY_BUS:
		bridge->primary_bus             = value & 0xff;
		bridge->secondary_bus           = (value >> 8) & 0xff;
		bridge->subordinate_bus         = (value >> 16) & 0xff;
		bridge->secondary_latency_timer = (value >> 24) & 0xff;
		mvebu_pcie_set_local_bus_nr(port, bridge->secondary_bus);
		break;

	default:
		break;
	}

	return PCIBIOS_SUCCESSFUL;
}

static inline struct mvebu_pcie *sys_to_pcie(struct pci_sys_data *sys)
{
	return sys->private_data;
}

static struct mvebu_pcie_port *mvebu_pcie_find_port(struct mvebu_pcie *pcie,
						    struct pci_bus *bus,
						    int devfn)
{
	int i;

	for (i = 0; i < pcie->nports; i++) {
		struct mvebu_pcie_port *port = &pcie->ports[i];

		if (bus->number == 0 && port->devfn == devfn)
			return port;
		if (bus->number != 0 &&
		    bus->number >= port->bridge.secondary_bus &&
		    bus->number <= port->bridge.subordinate_bus)
			return port;
	}

	return NULL;
}

/* PCI configuration space write function */
static int mvebu_pcie_wr_conf(struct pci_bus *bus, u32 devfn,
			      int where, int size, u32 val)
{
	struct mvebu_pcie *pcie = sys_to_pcie(bus->sysdata);
	struct mvebu_pcie_port *port;
	int ret;

	port = mvebu_pcie_find_port(pcie, bus, devfn);
	if (!port)
		return PCIBIOS_DEVICE_NOT_FOUND;

	/* Access the emulated PCI-to-PCI bridge */
	if (bus->number == 0)
		return mvebu_sw_pci_bridge_write(port, where, size, val);

	if (!mvebu_pcie_link_up(port))
		return PCIBIOS_DEVICE_NOT_FOUND;

	/*
	 * On the secondary bus, we don't want to expose any other
	 * device than the device physically connected in the PCIe
	 * slot, visible in slot 0. In slot 1, there's a special
	 * Marvell device that only makes sense when the Armada is
	 * used as a PCIe endpoint.
	 */
	if (bus->number == port->bridge.secondary_bus &&
	    PCI_SLOT(devfn) != 0)
		return PCIBIOS_DEVICE_NOT_FOUND;

	/* Access the real PCIe interface */
	ret = mvebu_pcie_hw_wr_conf(port, bus, devfn,
				    where, size, val);

	return ret;
}

/* PCI configuration space read function */
static int mvebu_pcie_rd_conf(struct pci_bus *bus, u32 devfn, int where,
			      int size, u32 *val)
{
	struct mvebu_pcie *pcie = sys_to_pcie(bus->sysdata);
	struct mvebu_pcie_port *port;
	int ret;

	port = mvebu_pcie_find_port(pcie, bus, devfn);
	if (!port) {
		*val = 0xffffffff;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	/* Access the emulated PCI-to-PCI bridge */
	if (bus->number == 0)
		return mvebu_sw_pci_bridge_read(port, where, size, val);

	if (!mvebu_pcie_link_up(port)) {
		*val = 0xffffffff;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	/*
	 * On the secondary bus, we don't want to expose any other
	 * device than the device physically connected in the PCIe
	 * slot, visible in slot 0. In slot 1, there's a special
	 * Marvell device that only makes sense when the Armada is
	 * used as a PCIe endpoint.
	 */
	if (bus->number == port->bridge.secondary_bus &&
	    PCI_SLOT(devfn) != 0) {
		*val = 0xffffffff;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	/* Access the real PCIe interface */
	ret = mvebu_pcie_hw_rd_conf(port, bus, devfn,
				    where, size, val);

	return ret;
}

static struct pci_ops mvebu_pcie_ops = {
	.read = mvebu_pcie_rd_conf,
	.write = mvebu_pcie_wr_conf,
};

static int mvebu_pcie_setup(int nr, struct pci_sys_data *sys)
{
	struct mvebu_pcie *pcie = sys_to_pcie(sys);
	int i;

	pcie->mem.name = "PCI MEM";
	pcie->realio.name = "PCI I/O";

	if (request_resource(&iomem_resource, &pcie->mem))
		return 0;

	if (resource_size(&pcie->realio) != 0) {
		if (request_resource(&ioport_resource, &pcie->realio)) {
			release_resource(&pcie->mem);
			return 0;
		}
		pci_add_resource_offset(&sys->resources, &pcie->realio,
					sys->io_offset);
	}
	pci_add_resource_offset(&sys->resources, &pcie->mem, sys->mem_offset);
	pci_add_resource(&sys->resources, &pcie->busn);

	for (i = 0; i < pcie->nports; i++) {
		struct mvebu_pcie_port *port = &pcie->ports[i];

		if (!port->base)
			continue;
		mvebu_pcie_setup_hw(port);
	}

	return 1;
}

static resource_size_t mvebu_pcie_align_resource(struct pci_dev *dev,
						 const struct resource *res,
						 resource_size_t start,
						 resource_size_t size,
						 resource_size_t align)
{
	if (dev->bus->number != 0)
		return start;

	/*
	 * On the PCI-to-PCI bridge side, the I/O windows must have at
	 * least a 64 KB size and the memory windows must have at
	 * least a 1 MB size. Moreover, MBus windows need to have a
	 * base address aligned on their size, and their size must be
	 * a power of two. This means that if the BAR doesn't have a
	 * power of two size, several MBus windows will actually be
	 * created. We need to ensure that the biggest MBus window
	 * (which will be the first one) is aligned on its size, which
	 * explains the rounddown_pow_of_two() being done here.
	 */
	if (res->flags & IORESOURCE_IO)
		return round_up(start, max_t(resource_size_t, SZ_64K,
					     rounddown_pow_of_two(size)));
	else if (res->flags & IORESOURCE_MEM)
		return round_up(start, max_t(resource_size_t, SZ_1M,
					     rounddown_pow_of_two(size)));
	else
		return start;
}

static void mvebu_pcie_disable_all_ifaces(struct mvebu_pcie *pcie)
{
	u32 mask;

	/*
	 * Disable interfaces, and clear PCIe clkout bits and x4
	 * configuration
	 */
	switch(pcie->variant)
	{
	case PCIE_VARIANT_AXP:
		mask = BIT(8) | BIT(7) | BIT(5) | BIT(4) | 0xF;
		break;
	case PCIE_VARIANT_A370:
		mask = 0x3;
		break;
	case PCIE_VARIANT_A375:
		mask = BIT(5) | BIT(4) | 0x3;
		break;
	case PCIE_VARIANT_A38X:
	case PCIE_VARIANT_A39X:
		mask = BIT(15) | BIT(14) | BIT(5) | BIT(4) | 0xF;
		break;
	/*
	 * We don't support low-level PCIe init with Kirkwood and
	 * Dove
	 */
	default:
		return;
	}

	regmap_update_bits(pcie->syscon_regmap, SOC_CONTROL_REG, mask, 0);

	/* Clear the PCIe related bits in COMPHY_REF_CLK_ALIGN */
	if (pcie->variant == PCIE_VARIANT_AXP ||
	    pcie->variant == PCIE_VARIANT_A370)
		regmap_update_bits(pcie->syscon_regmap, COMPHY_REF_CLK_ALIGN,
				   COMPHY_REF_CLK_ALIGN_PCIE_MASK, 0);
}

static void mvebu_pcie_enable_iface(struct mvebu_pcie_port *port,
				    bool enable)
{
	u32 mask = BIT(port->port);

	regmap_update_bits(port->pcie->syscon_regmap,
			   SOC_CONTROL_REG, mask,
			   enable ? mask : 0);
}

/*
 * When enable is true, sets the interface in "quad" mode, i.e 4
 * independent x1 interfaces
 */
static int mvebu_pcie_set_quad_iface(struct mvebu_pcie_port *port,
				     bool enable)
{
	u32 mask;

	/* Make sure the PCIe interface is x1 */
	if (port->nlanes != 1)
		return -EINVAL;

	switch(port->pcie->variant)
	{
	case PCIE_VARIANT_AXP:
		/*
		 * On Armada XP, bits 7 and 8 control x1/x4 configuration
		 * for PCIe0 and PCIe1 respectively
		 */
		mask = BIT(port->port + 7);
		regmap_update_bits(port->pcie->syscon_regmap,
				   SOC_CONTROL_REG, mask,
				   enable ? mask : 0);
		break;
	case PCIE_VARIANT_A38X:
	case PCIE_VARIANT_A39X:
		/*
		 * On Armada 38x/39x, bit 14 controls x4 configuration
		 * for PCIe0. Other PCIe interfaces are not x4
		 * compatible.
		 */
		mask = BIT(14);
		regmap_update_bits(port->pcie->syscon_regmap,
				   SOC_CONTROL_REG, mask,
				   enable ? mask : 0);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int mvebu_pcie_set_clkout(struct mvebu_pcie_port *port,
				 bool enable)
{
	u32 mask;

	/* No such configuration on Armada 370 */
	if (port->pcie->variant == PCIE_VARIANT_A370)
		return -EINVAL;

	/* Only available for the first two interfaces */
	if (port->port > 1)
		return -EINVAL;

	/* Bits 4 and 5 controls clkout for PCIe0 and PCIe1 */
	mask = BIT(port->port + 4);
	regmap_update_bits(port->pcie->syscon_regmap, SOC_CONTROL_REG,
			   mask, enable ? mask : 0);

	return 0;
}


/*
 * Power on all PHYs needed for a given PCIe port, which will in fact
 * set up the SERDES lanes or COMPHY depending on the SoC.
 */
static void mvebu_pcie_phy_power_on(struct mvebu_pcie_port *port)
{
	struct device *dev = &port->pcie->pdev->dev;
	int phyi;

	for (phyi = 0; phyi < port->nlanes; phyi++) {
		struct phy *phy;

		phy = devm_of_phy_get_by_index(dev, port->dn, phyi);
		if (IS_ERR(phy)) {
			dev_err(dev, "PCIe%d.%d: cannot get PHY %d: %ld\n",
				port->port, port->lane, phyi, PTR_ERR(phy));
			continue;
		} else {
			int ret;

			ret = phy_power_on(phy);
			if (ret) {
				dev_err(dev, "PCIe%d.%d: cannot power-up PHY %d\n",
					port->port, port->lane, phyi);
				continue;
			}
		}
	}
}

/*
 * This function does the initialization of the PHY and SERDES, for
 * Armada 370 and Armada XP
 */
static void mvebu_pcie_serdes_phy_init(struct mvebu_pcie_port *port)
{
	u32 rx_impedance_mode, reg;
	u16 model = 0;

	/* Apply soft reset + fixed pclk mode */
	mvebu_phy_write(port, 0, GLOB_CLK_CTRL, 0x25);

	/* Multicast enable bit configuration */
	if (port->nlanes == 4)
		mvebu_phy_write(port, 0, GLOB_TEST_CTRL, 0x200);

	/* Clock control */
	if (port->nlanes == 1)
		mvebu_phy_write(port, 0, GLOB_CLK_SRC_LO, 0x0f);

	mvebu_phy_write(port, 0, 0xC5, 0x11f);

	/*
	 * Power UP the PHY here to get the proper muxing of the
	 * SERDES lanes.
	 */
	mvebu_pcie_phy_power_on(port);

	/*
	 * Activate the RX High Impedance Mode field (bit [2]) in
	 * register PCIe_USB Control. Set bit[12]: The analog part
	 * latches idle if PU_TX = 1 and PU_PLL = 1.
	 */
	rx_impedance_mode = 0x8080;
	if (port->pcie->variant == PCIE_VARIANT_AXP)
		rx_impedance_mode |= 0x4;

#if 0
	/* WHYY ? With this, value = 0x1084 instead of 0x9084 */
	if (port->nlanes == 4)
		rx_impedance_mode &= 0xff;
#endif

	mvebu_phy_write(port, port->lane, 0x48, 0x1000 | rx_impedance_mode);

	/* Configure x4 or x1 for the first two ports on Armada XP and clkout */

	if (port->pcie->variant == PCIE_VARIANT_AXP &&
	    port->port <= 1 && port->lane == 0) {
		u32 reg;

		mvebu_pcie_set_quad_iface(port, port->nlanes == 1);

		regmap_read(port->pcie->syscon_regmap, SAMPLE_AT_RESET, &reg);
		mvebu_pcie_set_clkout(port, reg & SAR_PCIE_CLKOUT);
	}


	/* PCI Express Link Capabilities */

	reg = mvebu_readl(port, PCIE_LINK_CTRL_STAT_OFF);
	reg |= PCIE_LINK_CTRL_COMMON_CLK;
	mvebu_writel(port, reg, PCIE_LINK_CTRL_STAT_OFF);

	reg = mvebu_readl(port, PCIE_LINK_CAPS_OFF);
	reg &= ~(PCIE_LINK_CAP_SPD_MASK | PCIE_LINK_CAP_WIDTH_MASK);
#if 1
	if (port->nlanes == 4)
		reg |= PCIE_LINK_CAP_WIDTH_4;
	else
		reg |= PCIE_LINK_CAP_WIDTH_1;
#else
	reg |= PCIE_LINK_CAP_WIDTH_1;
#endif

	/* Assume we support GEN2. FIXME: Check with Marvell. */
	reg |= PCIE_LINK_CAP_SPD_5G;
	mvebu_writel(port, reg, PCIE_LINK_CAPS_OFF);

	/* Configure the Reference Clock Align register for the x4 ports */
	if (port->nlanes == 4)
		regmap_update_bits(port->pcie->syscon_regmap, COMPHY_REF_CLK_ALIGN,
				   0xF << port->port, 0xF << port->port);

	/* Prepare PHY parameters */
	if ((port->nlanes == 4 && port->port == 0) ||
	    port->nlanes == 1) {
		/* Set operational mode and reference clock */
		mvebu_phy_write(port, port->lane, 0x1, 0xFC60);

		if (port->pcie->variant == PCIE_VARIANT_AXP) {
			u16 val;

			/* Configure max PLL rate (bit 8) in KVCO Calibration Control */
			val = mvebu_phy_read(port, port->lane, 0x2);
			/* TODO: WEIRD. The U-Boot code sets bit 8, but in practice it doesn't seem to be set */
///			val |= BIT(8);
			mvebu_phy_write(port, port->lane, 0x2, val);

			/* Use Maximum PLL Rate (bits [10:9]) */
			val = mvebu_phy_read(port, port->lane, 0x81);
			val |= BIT(10) | BIT(9);
			mvebu_phy_write(port, port->lane, 0x81, val);
		}
	}

	/* Last phase of PEX-PIPE Configuration */
	mvebu_phy_write(port, port->lane, GLOB_CLK_CTRL, 0x24);

	/* Make sure we're working in Root Complex mode */
	reg = mvebu_readl(port, PCIE_CAPABILITIES_OFF);
	reg &= ~PCIE_CAP_DEVTYPE_MASK;
	reg |= PCIE_CAP_DEVTYPE_RC;
	mvebu_writel(port, reg, PCIE_CAPABILITIES_OFF);

	/*
	 * Change of Slew Rate port0. U-Boot source code indicates
	 * 0x2a21, but dumping the register shows 0xaa21.
	 */
	mvebu_phy_write(port, port->lane, 0xF, 0xaa21);

	/*
	 * Change PLL BW port0. U-Boot source code indicates 0x6219,
	 * but dumping the register shows 0x6205.
	 */
	mvebu_phy_write(port, port->lane, 0x4F, 0x6205);

	mvebu_pcie_enable_iface(port, true);

	/* Update the PCIe device ID */
	if (port->pcie->variant == PCIE_VARIANT_A370)
		model = 0x6710;
	else {
		if (of_machine_is_compatible("marvell,armadaxp-mv78460"))
			model = 0x7846;
		else if (of_machine_is_compatible("marvell,armadaxp-mv78260"))
			model = 0x7826;
		else if (of_machine_is_compatible("marvell,armadaxp-mv78230"))
			model = 0x7823;
	}

	reg = mvebu_readl(port, PCIE_DEV_ID_OFF);
	reg &= ~PCIE_DEV_ID_MASK;
	reg |= (model << PCIE_DEV_ID_SHIFT);
	mvebu_writel(port, reg, PCIE_DEV_ID_OFF);

#if 0
	pr_info(" === Register dump\n");
	pr_info("  PHY @ 0, 0xC1: 0x%x\n", mvebu_phy_read(port, 0, 0xC1));
	pr_info("  PHY @ 0, 0xC2: 0x%x\n", mvebu_phy_read(port, 0, 0xC2));
	pr_info("  PHY @ 0, 0xC3: 0x%x\n", mvebu_phy_read(port, 0, 0xC3));
	pr_info("  PHY @ 0, 0xC3: 0x%x\n", mvebu_phy_read(port, 0, 0xC5));
	pr_info("  PHY @ lane, 0x48: 0x%x\n", mvebu_phy_read(port, port->lane, 0x48));
	regmap_read(port->pcie->syscon_regmap, SOC_CONTROL_REG, &reg);
	pr_info("  SOC_CONTROL_REG: 0x%x\n", reg);
	pr_info("  PCIE_LINK_CTRL_STAT_OFF: 0x%x\n", mvebu_readl(port, PCIE_LINK_CTRL_STAT_OFF));
	pr_info("  PCIE_LINK_CAPS_OFF: 0x%x\n", mvebu_readl(port, PCIE_LINK_CAPS_OFF));
	pr_info("  PHY @ lane, 0x2: 0x%x\n", mvebu_phy_read(port, port->lane, 0x2));
	pr_info("  PHY @ lane, 0x81: 0x%x\n", mvebu_phy_read(port, port->lane, 0x81));
	pr_info("  PHY @ lane, 0xC1: 0x%x\n", mvebu_phy_read(port, port->lane, 0xC1));
	pr_info("  PCIE_CAPABILITIES_OFF: 0x%x\n", mvebu_readl(port, PCIE_CAPABILITIES_OFF));
	pr_info("  PHY @ lane, 0xF: 0x%x\n", mvebu_phy_read(port, port->lane, 0xF));
	pr_info("  PHY @ lane, 0x4F: 0x%x\n", mvebu_phy_read(port, port->lane, 0x4F));
	pr_info("  PCIE_DEV_ID_OFF: 0x%x\n", mvebu_readl(port, PCIE_DEV_ID_OFF));
#endif
}

/*
 * This function does the initialization of the COMPHY for Armada 375,
 * 38x and 39x.
 */
static void mvebu_pcie_comphy_init(struct mvebu_pcie_port *port)
{
	/*
	 * Nothing else to be done, since the PHY initialization is
	 * actually done by the COMPHY driver.
	 */
	mvebu_pcie_phy_power_on(port);
}

static void mvebu_pcie_adjust_speed(struct pci_dev *dev)
{
	u32 reg;
	u16 pos, word;
	struct mvebu_pcie_port *port;
	struct mvebu_pcie *pcie;
	int width;

	/*
	 * We care only about the real devices, not the root bus that
	 * has the emulated PCI-to-PCI bridges.
	 */
	if (dev->bus->number == 0)
		return;

	pcie = sys_to_pcie(dev->bus->parent->sysdata);

	port = mvebu_pcie_find_port(pcie, dev->bus, dev->devfn);
	if (!port)
		return;

	reg = mvebu_readl(port, PCIE_DEBUG_STATUS_OFF);

	/* Undocumented magic values */
	if ((reg & 0x7f) != 0x7e)
		return;

	/*
	 * Query the actual negotiated link width, and if smaller than
	 * the width of the interface, reduce it.
	 */
	reg = mvebu_readl(port, PCIE_LINK_CTRL_STAT_OFF);
	width = (reg & PCIE_LINK_CTRL_WIDTH_MASK) >>
		PCIE_LINK_CTRL_WIDTH_SHIFT;
	if (width < port->nlanes) {
		reg = mvebu_readl(port, PCIE_LINK_CAPS_OFF);
		reg &= ~PCIE_LINK_CAP_WIDTH_MASK;
		reg |= width << PCIE_LINK_CAP_WIDTH_SHIFT;
		mvebu_writel(port, reg, PCIE_LINK_CAPS_OFF);
	}

	/*
	 * If we're capable of doing GEN2, but the link is currently
	 * configured as GEN1, then let's query the EP to see if it is
	 * GEN2 capable.
	 */

	/* Are we capable of doing GEN2 (5G) ? If not, bail out */
	reg = mvebu_readl(port, PCIE_LINK_CAPS_OFF);
	if ((reg & PCIE_LINK_CAP_SPD_MASK) != PCIE_LINK_CAP_SPD_5G)
		return;

	/* Is the link already GEN2 ? If yes, bail out */
	reg = mvebu_readl(port, PCIE_LINK_CTRL_STAT_OFF);
	if ((reg & PCIE_LINK_CTRL_SPD_MASK) != PCIE_LINK_CTRL_SPD_2_5G)
		return;

	pos = pci_bus_find_capability(dev->bus, dev->devfn, PCI_CAP_ID_EXP);
	pci_bus_read_config_word(dev->bus, dev->devfn, pos + PCI_EXP_LNKSTA, &word);

	/* If the device actually supports GEN2, upgrade the link speed */
	if ((word & PCI_EXP_LNKSTA_CLS) >= PCI_EXP_LNKSTA_CLS_5_0GB) {
		reg = mvebu_readl(port, PCIE_LINK_CTRL_STAT2_OFF);
		reg &= ~PCIE_LINK_CTRL2_SPD_MASK;
		reg |= PCIE_LINK_CTRL2_SPD_5G;
		mvebu_writel(port, reg, PCIE_LINK_CTRL_STAT2_OFF);

		reg = mvebu_readl(port, PCIE_CTRL_OFF);
		reg |= PCIE_CTRL_CONF_AUTOSPEED;
		mvebu_writel(port, reg, PCIE_CTRL_OFF);
	}
}

DECLARE_PCI_FIXUP_FINAL(PCI_ANY_ID, PCI_ANY_ID, mvebu_pcie_adjust_speed);

static void mvebu_pcie_enable(struct mvebu_pcie *pcie)
{
	struct hw_pci hw;

	memset(&hw, 0, sizeof(hw));

#ifdef CONFIG_PCI_MSI
	hw.msi_ctrl = pcie->msi;
#endif

	hw.nr_controllers = 1;
	hw.private_data   = (void **)&pcie;
	hw.setup          = mvebu_pcie_setup;
	hw.map_irq        = of_irq_parse_and_map_pci;
	hw.ops            = &mvebu_pcie_ops;
	hw.align_resource = mvebu_pcie_align_resource;

	pci_common_init_dev(&pcie->pdev->dev, &hw);
}

/*
 * Looks up the list of register addresses encoded into the reg =
 * <...> property for one that matches the given port/lane. Once
 * found, maps it.
 */
static void __iomem *mvebu_pcie_map_registers(struct platform_device *pdev,
					      struct device_node *np,
					      struct mvebu_pcie_port *port)
{
	struct resource regs;
	int ret = 0;

	ret = of_address_to_resource(np, 0, &regs);
	if (ret)
		return ERR_PTR(ret);

	return devm_ioremap_resource(&pdev->dev, &regs);
}

#define DT_FLAGS_TO_TYPE(flags)       (((flags) >> 24) & 0x03)
#define    DT_TYPE_IO                 0x1
#define    DT_TYPE_MEM32              0x2
#define DT_CPUADDR_TO_TARGET(cpuaddr) (((cpuaddr) >> 56) & 0xFF)
#define DT_CPUADDR_TO_ATTR(cpuaddr)   (((cpuaddr) >> 48) & 0xFF)

static int mvebu_get_tgt_attr(struct device_node *np, int devfn,
			      unsigned long type,
			      unsigned int *tgt,
			      unsigned int *attr)
{
	const int na = 3, ns = 2;
	const __be32 *range;
	int rlen, nranges, rangesz, pna, i;

	*tgt = -1;
	*attr = -1;

	range = of_get_property(np, "ranges", &rlen);
	if (!range)
		return -EINVAL;

	pna = of_n_addr_cells(np);
	rangesz = pna + na + ns;
	nranges = rlen / sizeof(__be32) / rangesz;

	for (i = 0; i < nranges; i++, range += rangesz) {
		u32 flags = of_read_number(range, 1);
		u32 slot = of_read_number(range + 1, 1);
		u64 cpuaddr = of_read_number(range + na, pna);
		unsigned long rtype;

		if (DT_FLAGS_TO_TYPE(flags) == DT_TYPE_IO)
			rtype = IORESOURCE_IO;
		else if (DT_FLAGS_TO_TYPE(flags) == DT_TYPE_MEM32)
			rtype = IORESOURCE_MEM;
		else
			continue;

		if (slot == PCI_SLOT(devfn) && type == rtype) {
			*tgt = DT_CPUADDR_TO_TARGET(cpuaddr);
			*attr = DT_CPUADDR_TO_ATTR(cpuaddr);
			return 0;
		}
	}

	return -ENOENT;
}

static void mvebu_pcie_msi_enable(struct mvebu_pcie *pcie)
{
	struct device_node *msi_node;

	msi_node = of_parse_phandle(pcie->pdev->dev.of_node,
				    "msi-parent", 0);
	if (!msi_node)
		return;

	pcie->msi = of_pci_find_msi_chip_by_node(msi_node);
	of_node_put(msi_node);

	if (pcie->msi)
		pcie->msi->dev = &pcie->pdev->dev;
}

static int mvebu_pcie_suspend(struct device *dev)
{
	struct mvebu_pcie *pcie;
	int i;

	pcie = dev_get_drvdata(dev);
	for (i = 0; i < pcie->nports; i++) {
		struct mvebu_pcie_port *port = pcie->ports + i;
		port->saved_pcie_stat = mvebu_readl(port, PCIE_STAT_OFF);
	}

	return 0;
}

static int mvebu_pcie_resume(struct device *dev)
{
	struct mvebu_pcie *pcie;
	int i;

	pcie = dev_get_drvdata(dev);
	for (i = 0; i < pcie->nports; i++) {
		struct mvebu_pcie_port *port = pcie->ports + i;
		mvebu_writel(port, port->saved_pcie_stat, PCIE_STAT_OFF);
		mvebu_pcie_setup_hw(port);
	}

	return 0;
}

static const struct of_device_id mvebu_pcie_of_match_table[] = {
	{ .compatible = "marvell,armada-xp-pcie",  .data = (void *) PCIE_VARIANT_AXP },
	{ .compatible = "marvell,armada-370-pcie", .data = (void *) PCIE_VARIANT_A370 },
	{ .compatible = "marvell,armada-375-pcie", .data = (void *) PCIE_VARIANT_A375 },
	{ .compatible = "marvell,armada-38x-pcie", .data = (void *) PCIE_VARIANT_A38X },
	{ .compatible = "marvell,armada-39x-pcie", .data = (void *) PCIE_VARIANT_A39X },
	{ .compatible = "marvell,dove-pcie",       .data = (void *) PCIE_VARIANT_DOVE },
	{ .compatible = "marvell,kirkwood-pcie",   .data = (void *) PCIE_VARIANT_KIRKWOOD },
	{},
};
MODULE_DEVICE_TABLE(of, mvebu_pcie_of_match_table);

static int mvebu_pcie_probe(struct platform_device *pdev)
{
	struct mvebu_pcie *pcie;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *child;
	const struct of_device_id *of_dev;
	int i, ret;

	pcie = devm_kzalloc(&pdev->dev, sizeof(struct mvebu_pcie),
			    GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;

	pcie->pdev = pdev;
	platform_set_drvdata(pdev, pcie);

	of_dev = of_match_device(mvebu_pcie_of_match_table, &pdev->dev);
	pcie->variant = (int) of_dev->data;

	/* Get the PCIe memory and I/O aperture */
	mvebu_mbus_get_pcie_mem_aperture(&pcie->mem);
	if (resource_size(&pcie->mem) == 0) {
		dev_err(&pdev->dev, "invalid memory aperture size\n");
		return -EINVAL;
	}

	mvebu_mbus_get_pcie_io_aperture(&pcie->io);

	if (resource_size(&pcie->io) != 0) {
		pcie->realio.flags = pcie->io.flags;
		pcie->realio.start = PCIBIOS_MIN_IO;
		pcie->realio.end = min_t(resource_size_t,
					 IO_SPACE_LIMIT,
					 resource_size(&pcie->io));
	} else
		pcie->realio = pcie->io;

	/* Get the bus range */
	ret = of_pci_parse_bus_range(np, &pcie->busn);
	if (ret) {
		dev_err(&pdev->dev, "failed to parse bus-range property: %d\n",
			ret);
		return ret;
	}

	/*
	 * We intentionally don't check the return value, since we
	 * want, for DT backward compatibility reason, to continue
	 * supporting platforms that don't have the syscon phandle.
	 */
	pcie->syscon_regmap =
		syscon_regmap_lookup_by_phandle(np, "marvell,syscon");
	if (!IS_ERR(pcie->syscon_regmap))
		mvebu_pcie_disable_all_ifaces(pcie);

	i = 0;
	for_each_child_of_node(pdev->dev.of_node, child) {
		if (!of_device_is_available(child))
			continue;
		i++;
	}

	pcie->ports = devm_kzalloc(&pdev->dev, i *
				   sizeof(struct mvebu_pcie_port),
				   GFP_KERNEL);
	if (!pcie->ports)
		return -ENOMEM;

	i = 0;
	for_each_child_of_node(pdev->dev.of_node, child) {
		struct mvebu_pcie_port *port = &pcie->ports[i];
		enum of_gpio_flags flags;

		if (!of_device_is_available(child))
			continue;

		port->pcie = pcie;

		if (of_property_read_u32(child, "marvell,pcie-port",
					 &port->port)) {
			dev_warn(&pdev->dev,
				 "ignoring PCIe DT node, missing pcie-port property\n");
			continue;
		}

		if (of_property_read_u32(child, "marvell,pcie-lane",
					 &port->lane))
			port->lane = 0;

		port->name = kasprintf(GFP_KERNEL, "pcie%d.%d",
				       port->port, port->lane);

		if (of_property_read_u32(child, "marvell,nlanes", &port->nlanes))
			port->nlanes = 1;

		port->devfn = of_pci_get_devfn(child);
		if (port->devfn < 0)
			continue;

		ret = mvebu_get_tgt_attr(np, port->devfn, IORESOURCE_MEM,
					 &port->mem_target, &port->mem_attr);
		if (ret < 0) {
			dev_err(&pdev->dev, "PCIe%d.%d: cannot get tgt/attr for mem window\n",
				port->port, port->lane);
			continue;
		}

		if (resource_size(&pcie->io) != 0)
			mvebu_get_tgt_attr(np, port->devfn, IORESOURCE_IO,
					   &port->io_target, &port->io_attr);
		else {
			port->io_target = -1;
			port->io_attr = -1;
		}

		port->reset_gpio = of_get_named_gpio_flags(child,
						   "reset-gpios", 0, &flags);
		if (gpio_is_valid(port->reset_gpio)) {
			u32 reset_udelay = 20000;

			port->reset_active_low = flags & OF_GPIO_ACTIVE_LOW;
			port->reset_name = kasprintf(GFP_KERNEL,
				     "pcie%d.%d-reset", port->port, port->lane);
			of_property_read_u32(child, "reset-delay-us",
					     &reset_udelay);

			ret = devm_gpio_request_one(&pdev->dev,
			    port->reset_gpio, GPIOF_DIR_OUT, port->reset_name);
			if (ret) {
				if (ret == -EPROBE_DEFER)
					return ret;
				continue;
			}

			gpio_set_value(port->reset_gpio,
				       (port->reset_active_low) ? 1 : 0);
			msleep(reset_udelay/1000);
		}

		port->clk = of_clk_get_by_name(child, NULL);
		if (IS_ERR(port->clk)) {
			dev_err(&pdev->dev, "PCIe%d.%d: cannot get clock\n",
			       port->port, port->lane);
			continue;
		}

		ret = clk_prepare_enable(port->clk);
		if (ret)
			continue;

		port->base = mvebu_pcie_map_registers(pdev, child, port);
		if (IS_ERR(port->base)) {
			dev_err(&pdev->dev, "PCIe%d.%d: cannot map registers\n",
				port->port, port->lane);
			port->base = NULL;
			clk_disable_unprepare(port->clk);
			continue;
		}

		mvebu_pcie_set_local_dev_nr(port, 1);
		port->dn = child;

		if (pcie->variant == PCIE_VARIANT_A370 ||
		    pcie->variant == PCIE_VARIANT_AXP)
			mvebu_pcie_serdes_phy_init(port);
		else
			mvebu_pcie_comphy_init(port);

		mvebu_sw_pci_bridge_init(port);
		i++;
	}

	pcie->nports = i;

	for (i = 0; i < (IO_SPACE_LIMIT - SZ_64K); i += SZ_64K)
		pci_ioremap_io(i, pcie->io.start + i);

	mvebu_pcie_msi_enable(pcie);
	mvebu_pcie_enable(pcie);

	platform_set_drvdata(pdev, pcie);

	return 0;
}

static struct dev_pm_ops mvebu_pcie_pm_ops = {
	.suspend_noirq = mvebu_pcie_suspend,
	.resume_noirq = mvebu_pcie_resume,
};

static struct platform_driver mvebu_pcie_driver = {
	.driver = {
		.name = "mvebu-pcie",
		.of_match_table = mvebu_pcie_of_match_table,
		/* driver unloading/unbinding currently not supported */
		.suppress_bind_attrs = true,
		.pm = &mvebu_pcie_pm_ops,
	},
	.probe = mvebu_pcie_probe,
};
module_platform_driver(mvebu_pcie_driver);

MODULE_AUTHOR("Thomas Petazzoni <thomas.petazzoni@free-electrons.com>");
MODULE_DESCRIPTION("Marvell EBU PCIe driver");
MODULE_LICENSE("GPL v2");
