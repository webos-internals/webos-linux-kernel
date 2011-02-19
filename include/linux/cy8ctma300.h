#ifndef _LINUX_CY8CTMA300_H
#define _LINUX_CY8CTMA300_H

#ifdef __KERNEL__
#include <asm/types.h>

#define CY8CTMA300_DEVICE	"cy8ctma300"
#define CY8CTMA300_DRIVER	"cy8ctma300"

struct cy8ctma300_platform_data {
	u8		block_len;	/* block bytes */
	u16		prec_len;	/* program record bytes */
	int		nr_srecs;	/* security record count */
	int		nr_blocks;	/* block count */
	int		(*sclk_request)(int request);
	int		(*sdata_request)(int request);
	void		(*vdd_enable)(int enable);
	void		(*vcpin_enable)(int enable);
	void		(*xres_assert)(int assert);
	unsigned	sclk;		/* sclk gpio */
	unsigned	sdata;		/* sdata gpio */
	unsigned long	xres_us;	/* xres pulse */
	unsigned long	reset_ns;	/* reset delay */
	unsigned long	ssclk_ns;	/* data setup */
	unsigned long	hsclk_ns;	/* data hold */
	unsigned long	dsclk_ns;	/* data out delay */
	unsigned long	wait_and_poll_ms;
};
#endif // __KERNEL__
#endif // _LINUX_CY8CTMA300_H
