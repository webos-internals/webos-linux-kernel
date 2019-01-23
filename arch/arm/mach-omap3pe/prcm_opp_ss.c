#ifdef CONFIG_CPU_FREQ_OVERRIDE
void omap_pm_opp_get_volts(u8 vdd1_volts[]) {
 #ifdef CONFIG_MACH_SIRLOIN_3630
        memcpy(vdd1_volts,mpu_iva2_vdd1_volts[tidx],
                                        sizeof(mpu_iva2_vdd1_volts[tidx]));
 #else
        memcpy(vdd1_volts,mpu_iva2_vdd1_volts,sizeof(mpu_iva2_vdd1_volts));
 #endif
}
EXPORT_SYMBOL(omap_pm_opp_get_volts);

void omap_pm_opp_set_volts(u8 vdd1_volts[]) {
 #ifdef CONFIG_MACH_SIRLOIN_3630
        memcpy(mpu_iva2_vdd1_volts[tidx],vdd1_volts,
                                        sizeof(mpu_iva2_vdd1_volts[tidx]));
        prcm_do_voltage_scaling(s_current_vdd1_opp, s_current_vdd1_opp-1);
 #else
        memcpy(mpu_iva2_vdd1_volts,vdd1_volts,sizeof(mpu_iva2_vdd1_volts));
        prcm_do_voltage_scaling(current_vdd1_opp, current_vdd1_opp-1);
 #endif
}
EXPORT_SYMBOL(omap_pm_opp_set_volts);

void omap_pm_opp_get_vdd2_volts(u8 *vdd2_volt) {
 #ifdef CONFIG_MACH_SIRLOIN_3630
        *(vdd2_volt)=(u8 )core_l3_vdd2_volts[tidx][2];
 #else
        *(vdd2_volt)=(u8 )core_l3_vdd2_volts[2];
 #endif
}
EXPORT_SYMBOL(omap_pm_opp_get_vdd2_volts);

void omap_pm_opp_set_vdd2_volts(u8 vdd2_volt) {
 #ifdef CONFIG_MACH_SIRLOIN_3630
        core_l3_vdd2_volts[tidx][2]=(u8)vdd2_volt;
        prcm_do_voltage_scaling(s_current_vdd2_opp, s_current_vdd2_opp-1);
 #else
        core_l3_vdd2_volts[2]=(u8)vdd2_volt;
        prcm_do_voltage_scaling(current_vdd2_opp, current_vdd2_opp-1);
 #endif
}
EXPORT_SYMBOL(omap_pm_opp_set_vdd2_volts);

void omap_pm_opp_get_vdd2_freq(u8 *vdd2_freq) {
        *(vdd2_freq)=(u8)vdd2_core_freq[2].freq;
}
EXPORT_SYMBOL(omap_pm_opp_get_vdd2_freq);

unsigned int prcm_get_current_vdd1_opp_no(void) {
 #ifdef CONFIG_MACH_SIRLOIN_3630
        return get_opp_no(s_current_vdd1_opp);
 #else
        return get_opp_no(current_vdd1_opp);
 #endif
}
EXPORT_SYMBOL(prcm_get_current_vdd1_opp_no);

unsigned short get_vdd1_arm_opp_for_freq(unsigned int freq)
{
        int i;
        for (i = 0; i < ARRAY_SIZE(vdd1_arm_dsp_freq); i++) {
                if (vdd1_arm_dsp_freq[i].freq_mpu == (freq / 1000)) {
                return i+1;
                }
        }
        return 0;
}
EXPORT_SYMBOL(get_vdd1_arm_opp_for_freq);
#endif
 
