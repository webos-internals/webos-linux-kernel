#ifndef _high_level_funcs_h_
#define _high_level_funcs_h_

int program_device_sbw(struct a6_sbw_interface* sbw_ops, uint32_t read_address);
int verify_device_sbw(struct a6_sbw_interface* sbw_ops, uint32_t read_address);

#endif
