/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef _MACH_QDSP5_V2_AUDIO_ACDB_DEF_H
#define _MACH_QDSP5_V2_AUDIO_ACDB_DEF_H

/* Define ACDB device ID */
#define ACDB_ID_HANDSET_SPKR				1
#define ACDB_ID_HANDSET_MIC				2
#define ACDB_ID_HEADSET_MIC				3
#define ACDB_ID_HEADSET_SPKR_MONO			4
#define ACDB_ID_HEADSET_SPKR_STEREO			5
#define ACDB_ID_SPKR_PHONE_MIC				6
#define ACDB_ID_SPKR_PHONE_MONO				7
#define ACDB_ID_SPKR_PHONE_STEREO			8
#define ACDB_ID_BT_SCO_MIC				9
#define ACDB_ID_BT_SCO_SPKR				0x0A
#define ACDB_ID_BT_A2DP_SPKR				0x0B
#define ACDB_ID_BT_A2DP_TX				0x10
#define ACDB_ID_TTY_HEADSET_MIC				0x0C
#define ACDB_ID_TTY_HEADSET_SPKR			0x0D
#define ACDB_ID_HEADSET_MONO_PLUS_SPKR_MONO_RX		0x11
#define ACDB_ID_HEADSET_STEREO_PLUS_SPKR_STEREO_RX	0x14
#define ACDB_ID_FM_TX_LOOPBACK				0x17
#define ACDB_ID_FM_TX					0x18
#define ACDB_ID_LP_FM_SPKR_PHONE_STEREO_RX		0x19
#define ACDB_ID_LP_FM_HEADSET_SPKR_STEREO_RX		0x1A
#define ACDB_ID_I2S_RX					0x20
#define ACDB_ID_SPKR_PHONE_MIC_BROADSIDE		0x2B
#define ACDB_ID_HANDSET_MIC_BROADSIDE			0x2C
#define ACDB_ID_SPKR_PHONE_MIC_ENDFIRE			0x2D
#define ACDB_ID_HANDSET_MIC_ENDFIRE			0x2E
#define ACDB_ID_I2S_TX					0x30
#define ACDB_ID_HDMI					0x40

#define ACDB_ID_HANDSET_SPKR_SLIDER_CLOSED 0x300
#define ACDB_ID_HANDSET_MIC_ENDFIRE_SLIDER_CLOSED 0x100
#define ACDB_ID_SPKR_PHONE_MONO_SLIDER_CLOSED	0x310
#define ACDB_ID_SPKR_PHONE_MIC_SLIDER_CLOSED	0x110
#define ACDB_ID_HEADSET_NO_MIC_SLIDER_OPEN		0x120
#define ACDB_ID_HEADSET_NO_MIC_SLIDER_CLOSED	0x122

#define ACDB_ID_AUDEQOFF_TX					0x50
#define ACDB_ID_AUDEQOFF_RX					0x52

#define ACDB_ID_MEDIA_HEADSET_RX			0x60
#define ACDB_ID_MEDIA_HEADSET_TX			0x61
#define ACDB_ID_MEDIA_HEADSET_MIC_RX		0x62
#define ACDB_ID_MEDIA_HEADSET_MIC_TX		0x63
#define ACDB_ID_MEDIA_BACK_SPEAKER_RX		0x64
#define ACDB_ID_MEDIA_BACK_SPEAKER_TX		0x65
#define ACDB_ID_MEDIA_HANDSET_RX			0x66
#define ACDB_ID_MEDIA_HANDSET_TX			0x67

#define ACDB_ID_VC_HEADSET_RX				0x70
#define ACDB_ID_VC_HEADSET_TX				0x71
#define ACDB_ID_VC_HEADSET_MIC_RX			0x72
#define ACDB_ID_VC_HEADSET_MIC_TX			0x73
#define ACDB_ID_VC_BACK_SPEAKER_RX			0x74
#define ACDB_ID_VC_BACK_SPEAKER_TX			0x75
#define ACDB_ID_VC_HANDSET_RX				0x76
#define ACDB_ID_VC_HANDSET_TX				0x77

/* ID used for virtual devices */
#define PSEUDO_ACDB_ID 					0xFFFF

#endif /* _MACH_QDSP5_V2_AUDIO_ACDB_DEF_H */
