// SPDX-License-Identifier: GPL-2.0+
/* Microchip MDIO initiator driver
 *
 * Copyright (c) 2025 Microchip Technology Inc. and its subsidiaries.
 *
 * This driver is based on "drivers/net/mdio/mdio-mscc-miim.c"
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_mdio.h>
#include <linux/platform_device.h>

#define P64H_MDIO_REG_PRESCALER 0x20
#define P64H_MDIO_CFG_PRESCALE_MASK GENMASK(7, 0)

#define P64H_MDIO_REG_FRAME_CFG_1 0x24
#define P64H_MDIO_WDATA_MASK GENMASK(15, 0)

#define P64H_MDIO_REG_FRAME_CFG_2 0x28
#define P64H_MDIO_TRIGGER_BIT BIT(31)
#define P64H_MDIO_REG_DEV_ADDR_MASK GENMASK(20, 16)
#define P64H_MDIO_PHY_PRT_ADDR_MASK GENMASK(8, 4)
#define P64H_MDIO_OPERATION_MASK GENMASK(3, 2)
#define P64H_MDIO_START_OF_FRAME_MASK GENMASK(1, 0)

#define P64H_MDIO_OPERATION_WRITE BIT(0)
#define P64H_MDIO_OPERATION_READ BIT(1)

#define P64H_MDIO_REG_FRAME_STATUS 0x2C
#define P64H_MDIO_READOK_BIT BIT(24)
#define P64H_MDIO_RDATA_MASK GENMASK(15, 0)

#define P64H_MDIO_REG_INT_I 0x30
#define P64H_MDIO_INT_I BIT(0)

#define P64H_MDIO_REG_INT_E 0x34

struct p64h_mdio_dev {
	void __iomem *regs;
	struct clk *clk;
	u32 bus_freq;
};

static int p64h_mdio_wait_trigger(struct mii_bus *bus)
{
	struct p64h_mdio_dev *p64h_mdio = bus->priv;
	u32 val;
	int ret;

	ret = readl_poll_timeout(p64h_mdio->regs + P64H_MDIO_REG_FRAME_CFG_2,
				 val, !(val & P64H_MDIO_TRIGGER_BIT), 50,
				 10000);

	if (ret < 0) {
		dev_err(&bus->dev, "TRIGGER bit timeout: %x\n", val);
	}

	return ret;
}

static int p64h_mdio_read(struct mii_bus *bus, int mii_id, int regnum)
{
	struct p64h_mdio_dev *p64h_mdio = bus->priv;
	u32 val;
	int ret;

	ret = p64h_mdio_wait_trigger(bus);
	if (ret)
		goto out;

	writel(P64H_MDIO_TRIGGER_BIT |
		       FIELD_PREP(P64H_MDIO_REG_DEV_ADDR_MASK, regnum) |
		       FIELD_PREP(P64H_MDIO_PHY_PRT_ADDR_MASK, mii_id) |
		       FIELD_PREP(P64H_MDIO_OPERATION_MASK,
				  P64H_MDIO_OPERATION_READ) |
		       FIELD_PREP(P64H_MDIO_START_OF_FRAME_MASK, 1),
	       p64h_mdio->regs + P64H_MDIO_REG_FRAME_CFG_2);

	ret = p64h_mdio_wait_trigger(bus);
	if (ret)
		goto out;

	val = readl(p64h_mdio->regs + P64H_MDIO_REG_FRAME_STATUS);

	if (!FIELD_GET(P64H_MDIO_READOK_BIT, val)) {
		dev_err(&bus->dev, "READOK bit cleared\n");
		ret = -EIO;
		goto out;
	}

	ret = FIELD_GET(P64H_MDIO_RDATA_MASK, val);

out:
	return ret;
}

static int p64h_mdio_write(struct mii_bus *bus, int mii_id, int regnum,
			   u16 value)
{
	struct p64h_mdio_dev *p64h_mdio = bus->priv;
	int ret;

	ret = p64h_mdio_wait_trigger(bus);
	if (ret < 0)
		goto out;

	writel(FIELD_PREP(P64H_MDIO_WDATA_MASK, value),
	       p64h_mdio->regs + P64H_MDIO_REG_FRAME_CFG_1);

	writel(P64H_MDIO_TRIGGER_BIT |
		       FIELD_PREP(P64H_MDIO_REG_DEV_ADDR_MASK, regnum) |
		       FIELD_PREP(P64H_MDIO_PHY_PRT_ADDR_MASK, mii_id) |
		       FIELD_PREP(P64H_MDIO_OPERATION_MASK,
				  P64H_MDIO_OPERATION_WRITE) |
		       FIELD_PREP(P64H_MDIO_START_OF_FRAME_MASK, 1),
	       p64h_mdio->regs + P64H_MDIO_REG_FRAME_CFG_2);

out:
	return ret;
}

static int p64h_mdio_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct p64h_mdio_dev *p64h_mdio;
	struct mii_bus *bus;
	void __iomem *regs;
	unsigned long rate;
	u32 div;
	int ret;

	regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(regs)) {
		dev_err(dev, "Failed to get memory resource\n");
		return PTR_ERR(regs);
	}

	bus = devm_mdiobus_alloc_size(dev, sizeof(*p64h_mdio));
	if (!bus)
		return -ENOMEM;

	p64h_mdio = bus->priv;
	p64h_mdio->regs = regs;

	bus->name = KBUILD_MODNAME;
	bus->read = p64h_mdio_read;
	bus->write = p64h_mdio_write;
	snprintf(bus->id, MII_BUS_ID_SIZE, "%s-mdio", dev_name(dev));
	bus->parent = dev;

	p64h_mdio->clk = devm_clk_get_optional(dev, NULL);
	if (IS_ERR(p64h_mdio->clk))
		return PTR_ERR(p64h_mdio->clk);

	ret = clk_prepare_enable(p64h_mdio->clk);
	if (ret)
		return ret;

	of_property_read_u32(np, "clock-frequency", &p64h_mdio->bus_freq);

	if (p64h_mdio->bus_freq) {
		if (!p64h_mdio->clk) {
			dev_err(dev,
				"cannot use clock-frequency without a clock\n");
			ret = -EINVAL;
			goto out_disable_clk;
		}

		rate = clk_get_rate(p64h_mdio->clk);

		div = DIV_ROUND_UP(rate, 2 * p64h_mdio->bus_freq) - 1;
		if (div == 0 || div & ~P64H_MDIO_CFG_PRESCALE_MASK) {
			dev_err(dev, "Incorrect MDIO clock frequency\n");
			ret = -EINVAL;
			goto out_disable_clk;
		}

		writel(div, p64h_mdio->regs + P64H_MDIO_REG_PRESCALER);
	}

	ret = of_mdiobus_register(bus, np);
	if (ret < 0) {
		dev_err(dev, "Cannot register MDIO bus (%d)\n", ret);
		goto out_disable_clk;
	}

	platform_set_drvdata(pdev, bus);

	return 0;

out_disable_clk:
	clk_disable_unprepare(p64h_mdio->clk);
	return ret;
}

static void p64h_mdio_remove(struct platform_device *pdev)
{
	struct mii_bus *bus = platform_get_drvdata(pdev);
	struct p64h_mdio_dev *p64h_mdio = bus->priv;

	clk_disable_unprepare(p64h_mdio->clk);
	mdiobus_unregister(bus);

	return ;
}

static const struct of_device_id p64h_mdio_match[] = {
	{ .compatible = "microchip,p64h-mdio" },
	{}
};
MODULE_DEVICE_TABLE(of, p64h_mdio_match);

static struct platform_driver p64h_mdio_driver = {
	.probe = p64h_mdio_probe,
	.remove = p64h_mdio_remove,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = p64h_mdio_match,
	},
};
module_platform_driver(p64h_mdio_driver);

MODULE_DESCRIPTION("Microchip P64H MDIO driver");
MODULE_LICENSE("GPL");
