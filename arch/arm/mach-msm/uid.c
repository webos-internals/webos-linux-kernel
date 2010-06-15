#include <linux/nduid.h>
#include <linux/kernel.h>
#include <asm/io.h>

#include "proc_comm.h"

unsigned int msm_nduid_get_device_salt(void)
{
	return 0xc001cafe;
}

int msm_nduid_get_cpu_id(char *id, unsigned int maxlen)
{
	int ret, i;
	uint32_t cmd;
	uint32_t len;
	uint32_t data2;

	cmd = PCOMM_PALM_DIAG_NV_GET_LEN;
	len = 0;
	ret = msm_proc_comm(PCOM_PALM_DIAG_CMDS, &cmd, &len);
	if (ret) {
		return ret;
	}

	if (maxlen < len) {
		return -1;
	}

	maxlen = len;

	while ((ret == 0) && (len > 0)) {
		cmd = PCOMM_PALM_DIAG_NV_GET_DATA;
		ret = msm_proc_comm(PCOM_PALM_DIAG_CMDS, &cmd, &data2);
		if (ret) {
			return ret;
		}

		for (i = 0; i < 4; i++) {
			*id++ = (data2 >> 24);
			data2 = (data2 << 8);
			len -= 1;
			if (len == 0) {
				break;
			}
		}
	}

	return maxlen;
}
