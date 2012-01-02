/*
 *  override_plug.c
 *
 *      Marco Benton <marco@unixpsycho.com>.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#ifdef CONFIG_CPU_FREQ_OVERRIDE_L2_HACK

// define L2 overclock table indexes from l2_freq_tbl_v2 L2(x)
//  set it the same to render it useless
#define L2_OCLK_IDX 17
#define L2_NORM_IDX 16

// Minimum LVAL freq for L2 overclock 54 MHz * L_VAL
#define MIN_LVAL_L2 0x20

void acpuclk_set_l2_hack(bool state)
{
	struct clkctl_acpu_speed *f;

	for (f = acpu_freq_tbl_v2 + 7; f->acpuclk_khz != 0; f++) {
		if(f->use_for_scaling[0] == 0) continue;
		if(f->l_val < MIN_LVAL_L2) continue;
		f->l2_level = (state) ? L2(L2_OCLK_IDX) : L2(L2_NORM_IDX);
	}
}
EXPORT_SYMBOL(acpuclk_set_l2_hack);
#endif

#ifdef CONFIG_CPU_FREQ_OVERRIDE_VOLT_CONFIG
void acpuclk_get_voltages(unsigned int acpu_freq_vlt_tbl[])
{
	int i=0;
	struct clkctl_acpu_speed *f;

	for (f = acpu_freq_tbl_v2; f->acpuclk_khz != 0; f++) {
		if(f->use_for_scaling[0] == 0) continue;
		acpu_freq_vlt_tbl[i] = f->vdd_sc;
		i++;
	}

}
EXPORT_SYMBOL(acpuclk_get_voltages);

void acpuclk_set_voltages(unsigned int acpu_freq_vlt_tbl[])
{
	int i=0;
	struct clkctl_acpu_speed *f;

	for (f = acpu_freq_tbl_v2; f->acpuclk_khz != 0; f++) {
		if(f->use_for_scaling[0] == 0) continue;
		f->vdd_sc = acpu_freq_vlt_tbl[i];
		i++;
	}
}
EXPORT_SYMBOL(acpuclk_set_voltages);
#endif

unsigned int acpuclk_get_freqs(unsigned int acpu_freq_tbl[])
{
	int i=0;
	struct clkctl_acpu_speed *f;

	for (f = acpu_freq_tbl_v2; f->acpuclk_khz != 0; f++) {
		if(!f->use_for_scaling[0]) continue;
		acpu_freq_tbl[i] = f->acpuclk_khz;
		i++;
	}

	return i;
}
EXPORT_SYMBOL(acpuclk_get_freqs);
