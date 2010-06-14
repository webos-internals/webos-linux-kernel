/*
 * twl4030-audio.h
 *
 */

#ifndef __TWL4030_AUDIO_H_
#define __TWL4030_AUDIO_H_

#include <linux/types.h>

int twl4030_audio_codec_enable(int);
int twl4030_audio_mic_bias_enable(bool);

#define CODEC_DISABLED    0
#define CODEC_ENABLED     1
typedef void (*twl4030_codec_event_callback)(void *cookie, int event);
int twl4030_register_codec_event_callback(twl4030_codec_event_callback cb, void *cookie);

#endif 
