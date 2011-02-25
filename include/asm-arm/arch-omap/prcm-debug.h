#ifndef __PRCM_DEBUG_H__
#define __PRCM_DEBUG_H__ 1

extern u32 current_vdd1_opp;
extern u32 current_vdd2_opp;

int power_configuration_test(void);
int powerapi_test(void);
int clkapi_test(void);
int dpllapi_test(void);
int voltage_scaling_tst (u32 target_opp_id, u32 current_opp_id);
int voltage_scaling_set_of_test (void);
int frequency_scaling_test(void);

#endif /* #ifndef __PRCM_DEBUG_H__ */
