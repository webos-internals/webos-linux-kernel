#ifndef __VIBRATOR_H__
#define __VIBRATOR_H__

struct vibe_state;
int _duty_cycle_store(struct vibe_state *state, int value);
int _direction_store(struct vibe_state *state, int value);
#endif
