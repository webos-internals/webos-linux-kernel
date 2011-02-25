#ifndef __TWL4030_AUDIO_H
#define __TWL4030_AUDIO_H

#include <sound/pcm.h>

#include "omap-mcbsp-alsa.h"

struct omap_alsa_codec_ops *twl4030_audio_get_codec_ops(void);
struct snd_pcm_hardware *twl4030_audio_get_hw_playback(void);
struct snd_pcm_hardware *twl4030_audio_get_hw_capture(void);
struct snd_pcm_hw_constraint_list *twl4030_audio_get_rate_constraint(void);

int twl4030_audio_write(u8 addr, u8 data);
int twl4030_audio_bit_set(u8 addr, u8 set);
int twl4030_audio_bit_clr(u8 addr, u8 clr);
int twl4030_audio_codec_phonecall_enable(int enable);

#endif
