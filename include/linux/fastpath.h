#ifndef _FASTPATH_H_
#define _FASTPATH_H_
int fastpath_prepare(void); 
int fastpath_fastsleep(int wakeup_is_rtc);

void fastpath_fastsleep_set(int flag);
int fastpath_fastsleep_get(void);
#endif // _FASTPATH_H_
