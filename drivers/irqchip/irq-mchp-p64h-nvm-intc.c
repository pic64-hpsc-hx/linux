// SPDX-License-Identifier: GPL-2.0
/*
 * Microchip P64H NVM shared interrupt controller.
 *
 * Demultiplexes the xSPI, NAND and eMMC/SD interrupts, which share one
 * level-high RISC-V APLIC source.
 *
 * Copyright (C) 2021 Microchip Technology Inc. All rights reserved.
 * Author: Romain Caritey <romain.caritey@microchip.com>
 *
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>

/* W1C, a bit is set on a rising edge of its source line */
#define P64H_NVM_INTC_STATUS	0x500
/* Gates each STATUS bit to the APLIC output */
#define P64H_NVM_INTC_ENABLE	0x504
/* Current level of the source lines (read only) */
#define P64H_NVM_INTC_VALUE	0x508

#define P64H_NVM_INTC_NUM_IRQS		7
#define P64H_NVM_INTC_VALID_MASK	GENMASK(P64H_NVM_INTC_NUM_IRQS - 1, 0)

/**
 * struct p64h_nvm_intc - P64H NVM interrupt controller state
 * @dev:          owning platform device
 * @base:         MMIO base of the register block
 * @domain:       linear irq_domain of the local sources
 * @parent_irq:   Linux IRQ of the APLIC source
 * @lock:         serialises the read-modify-write of ENABLE
 * @saved_enable: ENABLE saved across system suspend
 */
struct p64h_nvm_intc {
	struct device *dev;
	void __iomem *base;
	struct irq_domain *domain;
	int parent_irq;
	raw_spinlock_t lock;
	u32 saved_enable;
};

static inline bool p64h_nvm_intc_hwirq_is_valid(irq_hw_number_t hwirq)
{
	return hwirq < P64H_NVM_INTC_NUM_IRQS;
}

static inline void p64h_nvm_intc_clear_status(struct p64h_nvm_intc *intc,
					      u32 mask)
{
	writel(mask, intc->base + P64H_NVM_INTC_STATUS);
}

static inline u32 p64h_nvm_intc_read_value(struct p64h_nvm_intc *intc)
{
	return readl(intc->base + P64H_NVM_INTC_VALUE);
}

static inline u32 p64h_nvm_intc_read_enable(struct p64h_nvm_intc *intc)
{
	return readl(intc->base + P64H_NVM_INTC_ENABLE);
}

static inline void p64h_nvm_intc_write_enable(struct p64h_nvm_intc *intc,
					      u32 value)
{
	writel(value, intc->base + P64H_NVM_INTC_ENABLE);
}

static void p64h_nvm_intc_update_enable(struct p64h_nvm_intc *intc, u32 clear,
					u32 set)
{
	unsigned long flags;
	u32 enable;

	raw_spin_lock_irqsave(&intc->lock, flags);
	enable = p64h_nvm_intc_read_enable(intc);
	enable &= ~clear;
	enable |= set;
	p64h_nvm_intc_write_enable(intc, enable);

	raw_spin_unlock_irqrestore(&intc->lock, flags);
}

static void p64h_nvm_intc_irq_ack(struct irq_data *d)
{
	struct p64h_nvm_intc *intc = irq_data_get_irq_chip_data(d);

	p64h_nvm_intc_clear_status(intc, BIT(d->hwirq));
}

static void p64h_nvm_intc_irq_mask(struct irq_data *d)
{
	struct p64h_nvm_intc *intc = irq_data_get_irq_chip_data(d);

	p64h_nvm_intc_update_enable(intc, BIT(d->hwirq), 0);
}

static void p64h_nvm_intc_irq_unmask(struct irq_data *d)
{
	struct p64h_nvm_intc *intc = irq_data_get_irq_chip_data(d);

	p64h_nvm_intc_update_enable(intc, 0, BIT(d->hwirq));
}

static int p64h_nvm_intc_irq_set_type(struct irq_data *d, unsigned int type)
{
	if ((type & IRQ_TYPE_SENSE_MASK) != IRQ_TYPE_LEVEL_HIGH)
		return -EINVAL;

	return 0;
}

static void p64h_nvm_intc_irq_print_chip(struct irq_data *d, struct seq_file *p)
{
	struct p64h_nvm_intc *intc = irq_data_get_irq_chip_data(d);

	seq_printf(p, "P64H-NVM-%s", dev_name(intc->dev));
}

static const struct irq_chip p64h_nvm_intc_irq_chip = {
	.irq_print_chip	= p64h_nvm_intc_irq_print_chip,
	.irq_ack	= p64h_nvm_intc_irq_ack,
	.irq_disable	= p64h_nvm_intc_irq_mask,
	.irq_mask	= p64h_nvm_intc_irq_mask,
	.irq_unmask	= p64h_nvm_intc_irq_unmask,
	.irq_set_type	= p64h_nvm_intc_irq_set_type,
};

static int p64h_nvm_intc_domain_map(struct irq_domain *domain,
				    unsigned int virq, irq_hw_number_t hwirq)
{
	if (!p64h_nvm_intc_hwirq_is_valid(hwirq))
		return -EINVAL;

	irq_set_chip_data(virq, domain->host_data);
	irq_set_chip_and_handler(virq, &p64h_nvm_intc_irq_chip,
				 handle_level_irq);
	irq_set_noprobe(virq);

	return 0;
}

static void p64h_nvm_intc_domain_unmap(struct irq_domain *domain,
				       unsigned int virq)
{
	irq_set_chip_and_handler(virq, NULL, NULL);
	irq_set_chip_data(virq, NULL);
}

static int p64h_nvm_intc_domain_xlate(struct irq_domain *domain,
				      struct device_node *controller,
				      const u32 *intspec, unsigned int intsize,
				      unsigned long *out_hwirq,
				      unsigned int *out_type)
{
	if (intsize != 1 || !p64h_nvm_intc_hwirq_is_valid(intspec[0]))
		return -EINVAL;

	*out_hwirq = intspec[0];
	*out_type = IRQ_TYPE_LEVEL_HIGH;

	return 0;
}

static const struct irq_domain_ops p64h_nvm_intc_domain_ops = {
	.map	= p64h_nvm_intc_domain_map,
	.unmap	= p64h_nvm_intc_domain_unmap,
	.xlate	= p64h_nvm_intc_domain_xlate,
};

static irqreturn_t p64h_nvm_intc_handler(int irq, void *data)
{
	struct p64h_nvm_intc *intc = data;
	unsigned long pending;
	unsigned int hwirq;

	raw_spin_lock(&intc->lock);
	pending = readl(intc->base + P64H_NVM_INTC_STATUS) &
		  p64h_nvm_intc_read_enable(intc) & P64H_NVM_INTC_VALID_MASK;
	raw_spin_unlock(&intc->lock);

	if (!pending)
		return IRQ_NONE;

	/*
	 * STATUS may hold a stale edge, already handled while masked.
	 * Clear it, then dispatch only the sources whose line is still high.
	 */
	p64h_nvm_intc_clear_status(intc, pending);

	pending &= p64h_nvm_intc_read_value(intc);

	for_each_set_bit(hwirq, &pending, P64H_NVM_INTC_NUM_IRQS)
		generic_handle_domain_irq(intc->domain, hwirq);

	return IRQ_HANDLED;
}

static int p64h_nvm_intc_suspend(struct device *dev)
{
	struct p64h_nvm_intc *intc = dev_get_drvdata(dev);
	unsigned long flags;

	raw_spin_lock_irqsave(&intc->lock, flags);
	intc->saved_enable = p64h_nvm_intc_read_enable(intc);
	p64h_nvm_intc_write_enable(intc, 0);
	raw_spin_unlock_irqrestore(&intc->lock, flags);

	return 0;
}

static int p64h_nvm_intc_resume(struct device *dev)
{
	struct p64h_nvm_intc *intc = dev_get_drvdata(dev);
	unsigned long flags;

	raw_spin_lock_irqsave(&intc->lock, flags);
	p64h_nvm_intc_clear_status(intc, P64H_NVM_INTC_VALID_MASK);
	p64h_nvm_intc_write_enable(intc, intc->saved_enable);
	raw_spin_unlock_irqrestore(&intc->lock, flags);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(p64h_nvm_intc_pm_ops, p64h_nvm_intc_suspend,
				p64h_nvm_intc_resume);

static int p64h_nvm_intc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct p64h_nvm_intc *intc;
	int ret;

	intc = devm_kzalloc(dev, sizeof(*intc), GFP_KERNEL);
	if (!intc)
		return -ENOMEM;

	intc->dev = dev;
	raw_spin_lock_init(&intc->lock);

	intc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(intc->base))
		return PTR_ERR(intc->base);

	intc->parent_irq = platform_get_irq(pdev, 0);
	if (intc->parent_irq < 0)
		return intc->parent_irq;

	p64h_nvm_intc_write_enable(intc, 0);
	p64h_nvm_intc_clear_status(intc, P64H_NVM_INTC_VALID_MASK);

	intc->domain = irq_domain_create_linear(dev_fwnode(dev),
						P64H_NVM_INTC_NUM_IRQS,
						&p64h_nvm_intc_domain_ops,
						intc);
	if (!intc->domain)
		return dev_err_probe(dev, -ENOMEM,
				     "failed to create IRQ domain\n");

	dev_set_drvdata(dev, intc);

	/*
	 * Use a regular interrupt, not a chained one: it gets a
	 * /proc/irq entry, the only place where the affinity of the NVM
	 * sources is set. ENABLE is 0, the parent stays quiet until a source
	 * is unmasked.
	 */
	ret = devm_request_irq(dev, intc->parent_irq,
			       p64h_nvm_intc_handler, IRQF_NO_THREAD,
			       dev_name(dev), intc);
	if (ret) {
		irq_domain_remove(intc->domain);
		return dev_err_probe(dev, ret, "failed to request IRQ %d\n",
				     intc->parent_irq);
	}

	return 0;
}

static const struct of_device_id p64h_nvm_intc_of_match[] = {
	{ .compatible = "microchip,pic64hpsc-nvm-intc" },
	{}
};
MODULE_DEVICE_TABLE(of, p64h_nvm_intc_of_match);

static struct platform_driver p64h_nvm_intc_driver = {
	.probe	= p64h_nvm_intc_probe,
	.driver	= {
		.name			= "p64h-nvm-intc",
		.of_match_table		= p64h_nvm_intc_of_match,
		.pm			= pm_sleep_ptr(&p64h_nvm_intc_pm_ops),
		.suppress_bind_attrs	= true,
	},
};
builtin_platform_driver(p64h_nvm_intc_driver);

MODULE_DESCRIPTION("Microchip P64H NVM shared interrupt controller");
MODULE_LICENSE("GPL");
