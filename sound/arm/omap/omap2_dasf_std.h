/*
 * sound/arm/omap/omap2_dasf_std.h
 *
 * OMAP alsa dsp driver interface
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License 
 * version 2. This program is licensed "as is" without any warranty of any 
 * kind, whether express or implied.
 *
 */

#ifndef __OMAP_DASF_STD_H
#define __OMAP_DASF_STD_H

typedef enum {
	alsa_false = 0,
	alsa_true = 1
} boolean_t;

typedef unsigned int ALSA_STATUS;

#define ALSA_SBASE	0x00000000
#define ALSA_EBASE	0x80008000

#define ALSA_SOK	(ALSA_SBASE + 0)

#define ALSA_EMEMORY		(ALSA_EBASE + 0x1)
#define ALSA_ETIMEOUT		(ALSA_EBASE + 0x2)
#define ALSA_EINTERRUPTED	(ALSA_EBASE + 0x3)
#define ALSA_ENOTIMPL		(ALSA_EBASE + 0x4)
#define ALSA_ESTRM_PAUSED	(ALSA_EBASE + 0x5)
#define ALSA_EERROR		(ALSA_EBASE + 0x6)

typedef enum {
	DASF_PB_ENQUEUE = 77,
	DASF_PB_DEQUEUE,
	DASF_CAP_ENQUEUE,
	DASF_CAP_DEQUEUE,
	DASF_GET_COMMAND,
	DASF_PUT_RESPONSE,
	DASF_WAIT_FOR_PB
} dasf_ioctl_cmd;

typedef struct {
	void *stream_ptr;
	unsigned char *userPtr;
	unsigned char *drvPtr;
	unsigned int size;
} dasf_buf_t;

typedef struct {
	unsigned int stream_type;
	unsigned int rate;	/* rate in Hz */
	unsigned int channels;	/* number of channels */
	unsigned int format;	/* PCM format */
	void *drv_pb_buf_ptr;	/* driver reference to playback buffer */
} dasf_params_t;

typedef enum {
	DASF_STREAM_OPEN,
	DASF_STREAM_PAUSE,
	DASF_STREAM_RESUME,
	DASF_STREAM_TERMINATE,
	DASF_STREAM_MUTE,
	DASF_STREAM_UNMUTE,
	DASF_STREAM_GAIN,
	DASF_STREAM_CLOSE
} dasf_stream_cmd_t;

typedef union {
	struct {
		// unsigned int stream_handle;
		unsigned int stream_type;
		unsigned int rate;
		unsigned int channels;
		unsigned int format;
	} ARGS_STREAM_OPEN;

	struct {
		unsigned int gain;
	} ARGS_STREAM_GAIN;

} dasf_trapped_args_t;

typedef struct {
	dasf_stream_cmd_t cmd;
	unsigned int stream_handle;
	dasf_trapped_args_t args;
} dasf_trapped_cmd_t;

typedef struct {
	dasf_stream_cmd_t cmd;
	unsigned int stream_handle;
	unsigned int result;
} dasf_trapped_response_t;

#endif				/* End of #ifndef __OMAP_DASF_STD_H */
