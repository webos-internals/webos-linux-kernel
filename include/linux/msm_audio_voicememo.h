/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __MSM_AUDIO_VOICEMEMO_H
#define __MSM_AUDIO_VOICEMEMO_H

#include <linux/msm_audio.h>

#define AUDIO_GET_VOICEMEMO_CONFIG  _IOW(AUDIO_IOCTL_MAGIC, \
	(AUDIO_MAX_COMMON_IOCTL_NUM+0), unsigned)
#define AUDIO_SET_VOICEMEMO_CONFIG  _IOR(AUDIO_IOCTL_MAGIC, \
	(AUDIO_MAX_COMMON_IOCTL_NUM+1), unsigned)

/* rec_type */
enum rpc_voc_rec_dir_type {
	RPC_VOC_REC_NONE,
	RPC_VOC_REC_FORWARD,
	RPC_VOC_REC_REVERSE,
	RPC_VOC_REC_BOTH,
	RPC_VOC_MAX_REC_TYPE
};

/* capability */
enum rpc_voc_capability_type {
	RPC_VOC_CAP_IS733 = 4,
	RPC_VOC_CAP_IS127 = 8,
	RPC_VOC_CAP_AMR = 64,
	RPC_VOC_CAP_32BIT_DUMMY = 2147483647
};

/* Rate */
enum rpc_voc_rate_type {
	RPC_VOC_0_RATE = 0,
	RPC_VOC_8_RATE,
	RPC_VOC_4_RATE,
	RPC_VOC_2_RATE,
	RPC_VOC_1_RATE,
	RPC_VOC_ERASURE,
	RPC_VOC_ERR_RATE,
	RPC_VOC_AMR_RATE_475 = 0,
	RPC_VOC_AMR_RATE_515 = 1,
	RPC_VOC_AMR_RATE_590 = 2,
	RPC_VOC_AMR_RATE_670 = 3,
	RPC_VOC_AMR_RATE_740 = 4,
	RPC_VOC_AMR_RATE_795 = 5,
	RPC_VOC_AMR_RATE_1020 = 6,
	RPC_VOC_AMR_RATE_1220 = 7,
};

/* frame_format */
enum rpc_voc_pb_len_rate_var_type {
	RPC_VOC_PB_NATIVE_QCP = 3,
	RPC_VOC_PB_AMR,
	RPC_VOC_PB_EVB
};

struct msm_audio_voicememo_config {
	uint32_t rec_type;
	uint32_t rec_interval_ms;
	uint32_t auto_stop_ms;
	uint32_t capability;
	uint32_t max_rate;
	uint32_t min_rate;
	uint32_t frame_format;
	uint32_t dtx_enable;
	uint32_t data_req_ms;
};

#endif /* __MSM_AUDIO_VOICEMEMO_H */
