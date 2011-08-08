#ifndef _a6_sbw_impl_rib_h_
#define _a6_sbw_impl_rib_h_

#include <linux/types.h>
#include <linux/irqflags.h>

/****/
/* per-device functions: format fn_n where x is the 0-based device enumeration */
/****/

uint16_t a6_set_sbwtck_evt1_0(void);
uint16_t a6_clr_sbwtck_evt1_0(void);
uint16_t a6_set_sbwtdio_evt1_0(void);
uint16_t a6_clr_sbwtdio_evt1_0(void);

uint16_t a6_set_in_sbwtdio_evt1_0(void);
uint16_t a6_set_out_sbwtdio_evt1_0(void);
uint16_t a6_get_sbwtdio_evt1_0(void);
uint16_t a6_set_sbwakeup_evt1_0(void);
uint16_t a6_clr_sbwakeup_evt1_0(void);
uint16_t a6_set_sbwakeup_evt1_0(void);
uint16_t a6_clr_sbwakeup_evt1_0(void);

/****/
/* per-target function(s) */
/****/
void a6_delay_impl(uint32_t delay_us);

#endif //_a6_sbw_impl_rib_h_
