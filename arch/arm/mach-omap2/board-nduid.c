#include <asm/arch/nduid.h>
#include <asm/io.h>
#if defined(CONFIG_ARCH_OMAP24XX)
#include <asm/arch/omap24xx.h>
#elif defined(CONFIG_ARCH_OMAP34XX)
#include <asm/arch/omap34xx.h>
#else
#error Unsupported arch!
#endif

#define CONTROL_TAP_BASE          IO_ADDRESS(OMAP_TAP_BASE)
#define CONTROL_TAP_IDCODE        (CONTROL_TAP_BASE + 0x204)
#define CONTROL_TAP_PROD_ID       (CONTROL_TAP_BASE + 0x210)
#define CONTROL_TAP_DIE_ID_0      (CONTROL_TAP_BASE + 0x218)
#define CONTROL_TAP_DIE_ID_1      (CONTROL_TAP_BASE + 0x21c)
#define CONTROL_TAP_DIE_ID_2      (CONTROL_TAP_BASE + 0x220)
#define CONTROL_TAP_DIE_ID_3      (CONTROL_TAP_BASE + 0x224)

unsigned int omap_nduid_get_device_salt(void)
{
#if defined(CONFIG_MACH_SIRLOIN) || defined(CONFIG_MACH_FLANK)
	return 0xdeadbeef;
#elif defined(CONFIG_MACH_BRISKET)
	return 0xf005ba11;
#else
#error Unsupported machine type!
#endif
}

int omap_nduid_get_cpu_id(char *id, unsigned int maxlen)
{
	uint32_t *buf = (uint32_t *)id;

	if (maxlen < 4 + 4 + 16) {
		return -1;
	}

	__raw_readl(CONTROL_TAP_IDCODE);
	buf[0] = __raw_readl(CONTROL_TAP_IDCODE);
	buf[1] = __raw_readl(CONTROL_TAP_PROD_ID);
	buf[2] = __raw_readl(CONTROL_TAP_DIE_ID_0);
	buf[3] = __raw_readl(CONTROL_TAP_DIE_ID_1);
	buf[4] = __raw_readl(CONTROL_TAP_DIE_ID_2);
	buf[5] = __raw_readl(CONTROL_TAP_DIE_ID_3);

	return sizeof(uint32_t) * 6;
}
