/*
 * drivers/media/video/vx6852-dbg.c
 *
 * Copyright (C) 2008-2009 Palm, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
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
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <media/vx6852.h>

#define VX6852_DBG_ATTR_8_RO(_name, _idx) \
	.attr = { \
		.attr = { \
			.name = #_name, \
			.owner = THIS_MODULE, \
			.mode = S_IRUGO, \
		}, \
		.show = vx6852_dbg_attribute_8_show, \
	}, \
	.index = _idx,

#define VX6852_DBG_ATTR_8_RW(_name, _idx) \
	.attr = { \
		.attr = { \
			.name = #_name, \
			.owner = THIS_MODULE, \
			.mode = S_IRUGO | S_IWUGO, \
		}, \
		.show = vx6852_dbg_attribute_8_show, \
		.store = vx6852_dbg_attribute_8_store, \
	}, \
	.index = _idx,

#define VX6852_DBG_ATTR_16_RO(_name, _idx) \
	.attr = { \
		.attr = { \
			.name = #_name, \
			.owner = THIS_MODULE, \
			.mode = S_IRUGO, \
		}, \
		.show = vx6852_dbg_attribute_16_show, \
	}, \
	.index = _idx,

#define VX6852_DBG_ATTR_16_RW(_name, _idx) \
	.attr = { \
		.attr = { \
			.name = #_name, \
			.owner = THIS_MODULE, \
			.mode = S_IRUGO | S_IWUGO, \
		}, \
		.show = vx6852_dbg_attribute_16_show, \
		.store = vx6852_dbg_attribute_16_store, \
	}, \
	.index = _idx,

#define VX6852_DBG_ATTR_32_RO(_name, _idx) \
	.attr = { \
		.attr = { \
			.name = #_name, \
			.owner = THIS_MODULE, \
			.mode = S_IRUGO, \
		}, \
		.show = vx6852_dbg_attribute_32_show, \
	}, \
	.index = _idx,

struct vx6852_dbg_attribute {
	struct device_attribute	attr;
	u16			index;
};

static ssize_t
vx6852_dbg_attribute_8_show(struct device *dev, struct device_attribute *attr,
				char *begin)
{
	u8 value;
	char *end = begin;
	struct i2c_client *i2c;
	struct vx6852_dbg_attribute *reg;

	i2c = to_i2c_client(dev);
	reg = (struct vx6852_dbg_attribute *)attr;

	vx6852_i2c_read_u8(i2c, reg->index, &value);
	end += sprintf(end, "0x%02X(%u)\n", value, value);

	return (end - begin);
}

ssize_t
vx6852_dbg_attribute_8_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	u8 value;
	unsigned int inval;
	struct i2c_client *i2c;
	struct vx6852_dbg_attribute *reg;

	i2c = to_i2c_client(dev);
	reg = (struct vx6852_dbg_attribute *)attr;

	if (1 != sscanf(buf, "0x%x", &inval) && 1 != sscanf(buf, "%u", &inval))
		goto exit;

	value = inval;
	vx6852_i2c_write_u8(i2c, reg->index, value);

exit:
	return (count);
}

ssize_t
vx6852_dbg_attribute_16_show(struct device *dev, struct device_attribute *attr,
				char *begin)
{
	u16 value;
	char *end = begin;
	struct i2c_client *i2c;
	struct vx6852_dbg_attribute *reg;

	i2c = to_i2c_client(dev);
	reg = (struct vx6852_dbg_attribute *)attr;

	vx6852_i2c_read_u16(i2c, reg->index, &value);
	end += sprintf(end, "0x%04X(%u)\n", value, value);

	return (end - begin);
}

ssize_t
vx6852_dbg_attribute_16_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	u16 value;
	unsigned int inval;
	struct i2c_client *i2c;
	struct vx6852_dbg_attribute *reg;

	i2c = to_i2c_client(dev);
	reg = (struct vx6852_dbg_attribute *)attr;

	if (1 != sscanf(buf, "0x%x", &inval) && 1 != sscanf(buf, "%u", &inval))
		goto exit;

	value = inval;
	vx6852_i2c_write_u16(i2c, reg->index, value);

exit:
	return (count);
}

ssize_t
vx6852_dbg_attribute_32_show(struct device *dev, struct device_attribute *attr,
				char *begin)
{
	u32 value;
	char *end = begin;
	struct i2c_client *i2c;
	struct vx6852_dbg_attribute *reg;

	i2c = to_i2c_client(dev);
	reg = (struct vx6852_dbg_attribute *)attr;

	vx6852_i2c_read_u32(i2c, reg->index, &value);
	end += sprintf(end, "0x%08X(%u)\n", value, value);

	return (end - begin);
}

static struct vx6852_dbg_attribute vx6852_dbg_attrs[] = {
	{ VX6852_DBG_ATTR_16_RO(model_id, 0x0000) },
	{ VX6852_DBG_ATTR_8_RO(revision_number, 0x0002) },
	{ VX6852_DBG_ATTR_8_RO(manufacturer_id, 0x0003) },
	{ VX6852_DBG_ATTR_8_RO(smia_version, 0x0004) },
	{ VX6852_DBG_ATTR_8_RO(frame_count, 0x0005) },
	{ VX6852_DBG_ATTR_8_RO(pixel_order, 0x0006) },
	{ VX6852_DBG_ATTR_16_RO(data_pedestal, 0x0008) },
	{ VX6852_DBG_ATTR_8_RO(pixel_depth, 0x000C) },
	{ VX6852_DBG_ATTR_8_RO(frame_format_model_type, 0x0040) },
	{ VX6852_DBG_ATTR_8_RO(frame_format_model_subtype, 0x0041) },
	{ VX6852_DBG_ATTR_16_RO(frame_format_descriptor_0, 0x0042) },
	{ VX6852_DBG_ATTR_16_RO(frame_format_descriptor_1, 0x0044) },
	{ VX6852_DBG_ATTR_16_RO(frame_format_descriptor_2, 0x0046) },
	{ VX6852_DBG_ATTR_16_RO(frame_format_descriptor_3, 0x0048) },
	{ VX6852_DBG_ATTR_16_RO(frame_format_descriptor_4, 0x004A) },
	{ VX6852_DBG_ATTR_16_RO(frame_format_descriptor_5, 0x004C) },
	{ VX6852_DBG_ATTR_16_RO(analogue_gain_capability, 0x0080) },
	{ VX6852_DBG_ATTR_16_RO(analogue_gain_code_min, 0x0084) },
	{ VX6852_DBG_ATTR_16_RO(analogue_gain_code_max, 0x0086) },
	{ VX6852_DBG_ATTR_16_RO(analogue_gain_code_step, 0x0088) },
	{ VX6852_DBG_ATTR_16_RO(analogue_gain_type, 0x008A) },
	{ VX6852_DBG_ATTR_16_RO(analogue_gain_m0, 0x008C) },
	{ VX6852_DBG_ATTR_16_RO(analogue_gain_c0, 0x008E) },
	{ VX6852_DBG_ATTR_16_RO(analogue_gain_m1, 0x0090) },
	{ VX6852_DBG_ATTR_16_RO(analogue_gain_c1, 0x0092) },
	{ VX6852_DBG_ATTR_8_RO(data_format_model_type, 0x00C0) },
	{ VX6852_DBG_ATTR_8_RO(data_format_model_subtype, 0x00C1) },
	{ VX6852_DBG_ATTR_16_RO(data_format_descriptor_0, 0x00C2) },
	{ VX6852_DBG_ATTR_16_RO(data_format_descriptor_1, 0x00C4) },
	{ VX6852_DBG_ATTR_16_RO(data_format_descriptor_2, 0x00C6) },
	{ VX6852_DBG_ATTR_8_RW(mode_select, 0x0100) },
	{ VX6852_DBG_ATTR_8_RW(image_orientation, 0x0101) },
	{ VX6852_DBG_ATTR_8_RW(software_reset, 0x0103) },
	{ VX6852_DBG_ATTR_8_RW(grouped_parameter_hold, 0x0104) },
	{ VX6852_DBG_ATTR_8_RW(mask_corrupted_frames, 0x0105) },
	{ VX6852_DBG_ATTR_8_RW(CCP2_channel_identifier, 0x0110) },
	{ VX6852_DBG_ATTR_8_RW(CCP2_signaling_mode, 0x0111) },
	{ VX6852_DBG_ATTR_16_RW(CCP2_data_format, 0x0112) },
	{ VX6852_DBG_ATTR_8_RO(gain_mode, 0x0120) },
	{ VX6852_DBG_ATTR_16_RW(fine_integration_time, 0x0200) },
	{ VX6852_DBG_ATTR_16_RW(coarse_integration_time, 0x0202) },
	{ VX6852_DBG_ATTR_16_RW(analogue_gain_code_global, 0x0204) },
	{ VX6852_DBG_ATTR_16_RW(digital_gain_greenr, 0x020E) },
	{ VX6852_DBG_ATTR_16_RW(digital_gain_red, 0x0210) },
	{ VX6852_DBG_ATTR_16_RW(digital_gain_blue, 0x0212) },
	{ VX6852_DBG_ATTR_16_RW(digital_gain_greenb, 0x0214) },
	{ VX6852_DBG_ATTR_16_RW(vt_pix_clk_div, 0x0300) },
	{ VX6852_DBG_ATTR_16_RW(vt_sys_clk_div, 0x0302) },
	{ VX6852_DBG_ATTR_16_RW(pre_pll_clk_div, 0x0304) },
	{ VX6852_DBG_ATTR_16_RW(pll_multiplier, 0x0306) },
	{ VX6852_DBG_ATTR_16_RW(frame_length_lines, 0x0340) },
	{ VX6852_DBG_ATTR_16_RW(line_length_pck, 0x0342) },
	{ VX6852_DBG_ATTR_16_RW(x_addr_start, 0x0344) },
	{ VX6852_DBG_ATTR_16_RW(y_addr_start, 0x0346) },
	{ VX6852_DBG_ATTR_16_RW(x_addr_end, 0x0348) },
	{ VX6852_DBG_ATTR_16_RW(y_addr_end, 0x034A) },
	{ VX6852_DBG_ATTR_16_RW(x_output_size, 0x034C) },
	{ VX6852_DBG_ATTR_16_RW(y_output_size, 0x034E) },
	{ VX6852_DBG_ATTR_16_RW(x_odd_inc, 0x0382) },
	{ VX6852_DBG_ATTR_16_RW(y_odd_inc, 0x0386) },
	{ VX6852_DBG_ATTR_16_RO(compression_mode, 0x0500) },
	{ VX6852_DBG_ATTR_16_RW(test_pattern_mode, 0x0600) },
	{ VX6852_DBG_ATTR_16_RW(test_data_red, 0x0602) },
	{ VX6852_DBG_ATTR_16_RW(test_data_greenR, 0x0604) },
	{ VX6852_DBG_ATTR_16_RW(test_data_blue, 0x0606) },
	{ VX6852_DBG_ATTR_16_RW(test_data_greenB, 0x0608) },
	{ VX6852_DBG_ATTR_16_RW(horizontal_cursor_width, 0x060A) },
	{ VX6852_DBG_ATTR_16_RW(horizontal_cursor_position, 0x060C) },
	{ VX6852_DBG_ATTR_16_RW(vertical_cursor_width, 0x060E) },
	{ VX6852_DBG_ATTR_16_RW(vertical_cursor_position, 0x0610) },
	{ VX6852_DBG_ATTR_16_RO(integration_time_capability, 0x1000) },
	{ VX6852_DBG_ATTR_16_RO(course_integration_time_min, 0x1004) },
	{ VX6852_DBG_ATTR_16_RO(course_integration_time_max_margin, 0x1006) },
	{ VX6852_DBG_ATTR_16_RO(fine_integration_time_min, 0x1008) },
	{ VX6852_DBG_ATTR_16_RO(fine_integration_time_max_margin, 0x100A) },
	{ VX6852_DBG_ATTR_16_RO(digital_gain_capability, 0x1080) },
	{ VX6852_DBG_ATTR_16_RO(digital_gain_min, 0x1084) },
	{ VX6852_DBG_ATTR_16_RO(digital_gain_max, 0x1086) },
	{ VX6852_DBG_ATTR_16_RO(digital_gain_step_size, 0x1088) },
	{ VX6852_DBG_ATTR_32_RO(min_ext_clk_freq_mhz, 0x1100) },
	{ VX6852_DBG_ATTR_32_RO(max_ext_clk_freq_mhz, 0x1104) },
	{ VX6852_DBG_ATTR_16_RO(min_pre_pll_clk_div, 0x1108) },
	{ VX6852_DBG_ATTR_16_RO(max_pre_pll_clk_div, 0x110A) },
	{ VX6852_DBG_ATTR_32_RO(min_pll_ip_freq_mhz, 0x110C) },
	{ VX6852_DBG_ATTR_32_RO(max_pll_ip_freq_mhz, 0x1110) },
	{ VX6852_DBG_ATTR_16_RO(min_pll_multiplier, 0x1114) },
	{ VX6852_DBG_ATTR_16_RO(max_pll_multiplier, 0x1116) },
	{ VX6852_DBG_ATTR_32_RO(min_pll_op_freq_mhz, 0x1118) },
	{ VX6852_DBG_ATTR_32_RO(max_pll_op_freq_mhz, 0x111C) },
	{ VX6852_DBG_ATTR_16_RO(min_vt_sys_clk_div, 0x1120) },
	{ VX6852_DBG_ATTR_16_RO(max_vt_sys_clk_div, 0x1122) },
	{ VX6852_DBG_ATTR_32_RO(min_vt_sys_clk_freq_mhz, 0x1124) },
	{ VX6852_DBG_ATTR_32_RO(max_vt_sys_clk_freq_mhz, 0x1128) },
	{ VX6852_DBG_ATTR_32_RO(min_vt_pix_clk_freq_mhz, 0x112C) },
	{ VX6852_DBG_ATTR_32_RO(max_vt_pix_clk_freq_mhz, 0x1130) },
	{ VX6852_DBG_ATTR_16_RO(min_vt_pix_clk_div, 0x1134) },
	{ VX6852_DBG_ATTR_16_RO(max_vt_pix_clk_div, 0x1136) },
	{ VX6852_DBG_ATTR_16_RO(min_frame_length_lines, 0x1140) },
	{ VX6852_DBG_ATTR_16_RO(max_frame_length_lines, 0x1142) },
	{ VX6852_DBG_ATTR_16_RO(min_line_length_pck, 0x1144) },
	{ VX6852_DBG_ATTR_16_RO(max_line_length_pck, 0x1146) },
	{ VX6852_DBG_ATTR_16_RO(min_line_blanking_pck, 0x1148) },
	{ VX6852_DBG_ATTR_16_RO(min_frame_blanking_lines, 0x114A) },
	{ VX6852_DBG_ATTR_16_RO(x_addr_min, 0x1180) },
	{ VX6852_DBG_ATTR_16_RO(y_addr_min, 0x1182) },
	{ VX6852_DBG_ATTR_16_RO(x_addr_max, 0x1184) },
	{ VX6852_DBG_ATTR_16_RO(y_addr_max, 0x1186) },
	{ VX6852_DBG_ATTR_16_RO(min_x_output_size, 0x1188) },
	{ VX6852_DBG_ATTR_16_RO(min_y_output_size, 0x118A) },
	{ VX6852_DBG_ATTR_16_RO(max_x_output_size, 0x118C) },
	{ VX6852_DBG_ATTR_16_RO(max_y_output_size, 0x118E) },
	{ VX6852_DBG_ATTR_16_RO(min_even_inc, 0x11C0) },
	{ VX6852_DBG_ATTR_16_RO(max_even_inc, 0x11C2) },
	{ VX6852_DBG_ATTR_16_RO(min_odd_inc, 0x11C4) },
	{ VX6852_DBG_ATTR_16_RO(max_odd_inc, 0x11C6) },
	{ VX6852_DBG_ATTR_16_RO(scaling_capablility, 0x1200) },
	{ VX6852_DBG_ATTR_16_RO(scale_m_min, 0x1204) },
	{ VX6852_DBG_ATTR_16_RO(scale_m_max, 0x1206) },
	{ VX6852_DBG_ATTR_16_RO(scale_n_min, 0x1208) },
	{ VX6852_DBG_ATTR_16_RO(scale_n_max, 0x120A) },
	{ VX6852_DBG_ATTR_16_RO(compression_capablility, 0x1300) },
	{ VX6852_DBG_ATTR_16_RO(matrix_element_RedInRed, 0x1400) },
	{ VX6852_DBG_ATTR_16_RO(matrix_element_GreenInRed, 0x1402) },
	{ VX6852_DBG_ATTR_16_RO(matrix_element_BlueInRed, 0x1404) },
	{ VX6852_DBG_ATTR_16_RO(matrix_element_RedInGreen, 0x1406) },
	{ VX6852_DBG_ATTR_16_RO(matrix_element_GreenInGreen, 0x1408) },
	{ VX6852_DBG_ATTR_16_RO(matrix_element_BlueInGreen, 0x140A) },
	{ VX6852_DBG_ATTR_16_RO(matrix_element_RedInBlue, 0x140C) },
	{ VX6852_DBG_ATTR_16_RO(matrix_element_GreenInBlue, 0x140E) },
	{ VX6852_DBG_ATTR_16_RO(matrix_element_BlueInBlue, 0x1410) },
	{ VX6852_DBG_ATTR_16_RO(dark_average, 0x3000) },
	{ VX6852_DBG_ATTR_8_RO(dark_flags, 0x3002) },
	{ VX6852_DBG_ATTR_8_RW(dark_setup, 0x3003) },
	{ VX6852_DBG_ATTR_16_RW(dark_offset, 0x3004) },
	{ VX6852_DBG_ATTR_16_RW(dark_offset_req, 0x3006) },
	{ VX6852_DBG_ATTR_8_RW(__bayer_vtiming_major, 0x3015) },
	{ VX6852_DBG_ATTR_8_RW(bruce_enable, 0x31F0) },
	{ VX6852_DBG_ATTR_8_RW(bayer_average_enable, 0x3300) },
	{ VX6852_DBG_ATTR_16_RW(__vt_sys_clk_div, 0x3614) },
	{ VX6852_DBG_ATTR_8_RO(ILP_revision_id, 0x3800) },
	{ VX6852_DBG_ATTR_8_RO(firmware_ver_major, 0x3801) },
	{ VX6852_DBG_ATTR_8_RO(firmware_ver_minor, 0x3802) },
	{ VX6852_DBG_ATTR_8_RO(patch_ver_major, 0x3803) },
	{ VX6852_DBG_ATTR_8_RO(patch_ver_minor, 0x3804) },
	{ VX6852_DBG_ATTR_8_RW(av_filter_disable, 0x3874) },
	{ VX6852_DBG_ATTR_8_RW(defect_correction_disable, 0x3903) },
	{ VX6852_DBG_ATTR_8_RW(fine_green_enable, 0x3942) },
	{ VX6852_DBG_ATTR_8_RW(fine_green_control, 0x3943) },
	{ VX6852_DBG_ATTR_8_RW(fine_green_damper_input, 0x3944) },
	{ VX6852_DBG_ATTR_8_RW(fine_green_max_coring_level, 0x3945) },
	{ VX6852_DBG_ATTR_8_RW(fine_green_min_coring_level, 0x3946) },
	{ VX6852_DBG_ATTR_8_RO(fine_green_coring_level, 0x3947) },
	{ VX6852_DBG_ATTR_16_RW(fine_green_low_threshold, 0x3948) },
	{ VX6852_DBG_ATTR_16_RW(fine_green_high_threshold, 0x394A) },
	{ VX6852_DBG_ATTR_8_RW(daf_sharpness_level_macro, 0x3960) },
	{ VX6852_DBG_ATTR_8_RW(daf_sharpness_level_portrait, 0x3961) },
	{ VX6852_DBG_ATTR_8_RW(daf_sharpness_level_landscape, 0x3962) },
	{ VX6852_DBG_ATTR_8_RW(daf_denoising_value_outdoor, 0x3963) },
	{ VX6852_DBG_ATTR_8_RW(daf_denoising_value_indoor, 0x3964) },
	{ VX6852_DBG_ATTR_8_RW(daf_denoising_level_low_light, 0x3965) },
	{ VX6852_DBG_ATTR_8_RW(daf_noise_detail_outdoor, 0x3966) },
	{ VX6852_DBG_ATTR_8_RW(daf_noise_detail_indoor, 0x3967) },
	{ VX6852_DBG_ATTR_8_RW(daf_noise_detail_low_lightr, 0x3968) },
	{ VX6852_DBG_ATTR_8_RW(daf_blue_fringing_intensity, 0x3969) },
	{ VX6852_DBG_ATTR_8_RW(daf_mode, 0x3978) },
	{ VX6852_DBG_ATTR_8_RW(daf_focus_strategy, 0x3979) },
	{ VX6852_DBG_ATTR_8_RW(daf_focus_lock_mode, 0x397A) },
	{ VX6852_DBG_ATTR_8_RW(daf_focus_custom_center_weight, 0x397B) },
	{ VX6852_DBG_ATTR_8_RW(daf_focus_custom_middle_weight, 0x397C) },
	{ VX6852_DBG_ATTR_8_RW(daf_focus_custom_exterior_weight, 0x397D) },
	{ VX6852_DBG_ATTR_8_RW(daf_focus_control_coin, 0x397E) },
	{ VX6852_DBG_ATTR_16_RW(daf_stats_ROI_X, 0x3994) },
	{ VX6852_DBG_ATTR_16_RW(daf_stats_ROI_Y, 0x3996) },
	{ VX6852_DBG_ATTR_16_RW(daf_stats_ROI_width, 0x3998) },
	{ VX6852_DBG_ATTR_16_RW(daf_stats_ROI_height, 0x399A) },
	{ VX6852_DBG_ATTR_8_RW(daf_stats_ROI_control_coin, 0x399C) },
	{ VX6852_DBG_ATTR_8_RW(daf_stats_ROI_enable, 0x399D) },
	{ VX6852_DBG_ATTR_16_RW(daf_host_WB_stats_green_red, 0x399E) },
	{ VX6852_DBG_ATTR_16_RW(daf_host_WB_stats_red, 0x39A0) },
	{ VX6852_DBG_ATTR_16_RW(daf_host_WB_stats_blue, 0x39A2) },
	{ VX6852_DBG_ATTR_16_RW(daf_host_WB_stats_green_blue, 0x39A4) },
	{ VX6852_DBG_ATTR_8_RW(daf_host_WB_stats_control_coin, 0x39A6) },
	{ VX6852_DBG_ATTR_8_RW(daf_host_WB_stats_enable, 0x39A7) },
	{ VX6852_DBG_ATTR_8_RO(daf_estimated_focus, 0x39A8) },
};

int
vx6852_dbg_register_i2c_client(struct i2c_client *i2c)
{
	int i;
	int rc;
	struct device_attribute *attr;

	for (i = 0, rc = 0; ARRAY_SIZE(vx6852_dbg_attrs) > i; ++i) {
		attr = &vx6852_dbg_attrs[i].attr;
		if ((rc = device_create_file(&i2c->dev, attr))) {
			while (i--) {
				attr = &vx6852_dbg_attrs[i].attr;
				device_remove_file(&i2c->dev, attr);
			}
			break;
		}
	}

	return (rc);
}

EXPORT_SYMBOL(vx6852_dbg_register_i2c_client);

void
vx6852_dbg_unregister_i2c_client(struct i2c_client *i2c)
{
	int i;
	struct device_attribute *attr;

	for (i = 0; ARRAY_SIZE(vx6852_dbg_attrs) > i; ++i) {
		attr = &vx6852_dbg_attrs[i].attr;
		device_remove_file(&i2c->dev, attr);
	}
}

EXPORT_SYMBOL(vx6852_dbg_unregister_i2c_client);
