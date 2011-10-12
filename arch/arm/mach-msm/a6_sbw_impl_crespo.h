#ifndef _a6_sbw_impl_crespo_h_
#define _a6_sbw_impl_crespo_h_

#include <linux/types.h>
#include <linux/irqflags.h>

/****/
/* per-device functions: format fn_n where x is the 0-based device enumeration */
/****/

uint16_t a6_0_set_sbwtck(void);
uint16_t a6_0_clr_sbwtck(void);
uint16_t a6_0_set_sbwtdio(void);
uint16_t a6_0_clr_sbwtdio(void);
uint16_t a6_0_set_in_sbwtdio(void);
uint16_t a6_0_set_out_sbwtdio(void);
uint16_t a6_0_get_sbwtdio(void);
uint16_t a6_0_set_sbwakeup(void);
uint16_t a6_0_clr_sbwakeup(void);

/****/
/* per-target function(s) */
/****/
void a6_delay_impl(uint32_t delay_us);

#endif //_a6_sbw_impl_crespo_h_

