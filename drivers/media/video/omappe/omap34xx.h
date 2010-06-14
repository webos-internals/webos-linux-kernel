#ifndef OMAP34XX_H
#define OMAP34XX_H

#include <asm/io.h>
#include <asm/arch/io.h>

static inline void omap_clearl(unsigned int pa, unsigned int bits)
{
	unsigned int val;

	val = omap_readl(pa);
	val &= ~bits;
	omap_writel(val, pa);
}

static inline void omap_setl(unsigned int pa, unsigned int bits)
{
	unsigned int val;

	val = omap_readl(pa);
	val |= bits;
	omap_writel(val, pa);
}

static inline int omap_testl(unsigned int pa, unsigned int bits)
{
	unsigned int val;

	val = omap_readl(pa);

	return (!!(val & bits));
}

static inline void omap_masked_writel(
			unsigned int pa,
			unsigned int _val,
			unsigned int bits
			)
{
	unsigned int val;

	val = omap_readl(pa);
	val &= ~bits;
	val |= _val & bits;
	omap_writel(val, pa);
}

#endif // OMAP34XX_H
