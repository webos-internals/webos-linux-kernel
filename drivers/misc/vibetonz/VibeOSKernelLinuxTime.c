/*
** =========================================================================
** File:
**     VibeOSKernelLinuxTime.c
**
** Description: 
**     Time helper functions for Linux.
**
** Portions Copyright (c) 2008 Immersion Corporation. All Rights Reserved. 
**
** This file contains Original Code and/or Modifications of Original Code 
** as defined in and that are subject to the GNU Public License v2 - 
** (the 'License'). You may not use this file except in compliance with the 
** License. You should have received a copy of the GNU General Public License 
** along with this program; if not, write to the Free Software Foundation, Inc.,
** 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or contact 
** VibeTonzSales@immersion.com.
**
** The Original Code and all software distributed under the License are 
** distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER 
** EXPRESS OR IMPLIED, AND IMMERSION HEREBY DISCLAIMS ALL SUCH WARRANTIES, 
** INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY, FITNESS 
** FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT. Please see 
** the License for the specific language governing rights and limitations 
** under the License.
** =========================================================================
*/

#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <../clock.h>

#include <linux/hrtimer.h>

#define WATCHDOG_TIMEOUT                4      /* 4 timer cycles = 20ms */

#define fastcall

/* Global variables */
static bool g_bTimerStarted = false;
//static struct timer_list g_timerList;
static int g_nWatchdogCounter = 0;

static struct hrtimer vibe_timer;

DECLARE_MUTEX(g_hMutex);

/* Forward declarations */
static void VibeOSKernelLinuxStartTimer(void);
static void VibeOSKernelLinuxStopTimer(void);

/**
 * sem_is_locked - is the semaphore locked
 * @lock: the semaphore to be queried
 *
 * Returns 1 if the semaphore is locked, 0 if unlocked.
 *
 * TODO/Open Issue: 
 *
 * 1)Removed fastcall keyword
 * 2)Not sure if we need to protect read the count
 *
 */
static inline int sem_is_locked(struct semaphore *lock)
{
	return (lock->count != 1);
}

void stop_vibrate(struct work_struct *w)
{
    printk(KERN_INFO "Vibetonz: New data value timed out!\n");
    _direction_store(g_state,0);
}
DECLARE_WORK(stop_vibe,stop_vibrate);


static enum hrtimer_restart VibeOSKernelTimerProc(struct hrtimer *handle)
{
    /* Return right away if timer is not supposed to run */
    if (!g_bTimerStarted)
        return HRTIMER_NORESTART;

    if (sem_is_locked(&g_hMutex))
    {
        up(&g_hMutex);
    }
    else
    {
        if ((++g_nWatchdogCounter) > WATCHDOG_TIMEOUT)
        {
            /* Something went wrong. "write" was not called: we need to stop the motor and the timer */
            schedule_work(&stop_vibe);
            g_bTimerStarted = false;

            /* Reset watchdog counter */
            g_nWatchdogCounter = 0;
	        return HRTIMER_NORESTART;
        }
    }
    //handle->expires = ktime_add(ktime_get(),ktime_set(0,5000000));
    // handle->_expires = ktime_add(ktime_get(),ktime_set(0,5000000));
    // Use the standard hrtimer API to schedule the timer accurately,
    // which also takes care of time overflow.
    hrtimer_forward (handle, ktime_get (), ktime_set (0, 5000000) );
    return HRTIMER_RESTART;
}

static void VibeOSKernelLinuxInitTimer(void)
{
	hrtimer_init(&vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibe_timer.function = VibeOSKernelTimerProc;
	
}

static void VibeOSKernelLinuxStartTimer(void)
{
/*
    struct timespec res;
    hrtimer_get_res(CLOCK_MONOTONIC,&res);
	printk(KERN_INFO "Vibetonz: start timer (ie, write)! %d, %d\n",res.tv_sec,res.tv_nsec);
*/

    /* Reset watchdog counter */
    g_nWatchdogCounter = 0;

    if (!g_bTimerStarted)
    {
        if (!sem_is_locked(&g_hMutex)) down_interruptible(&g_hMutex); /* start locked */

        /* start timer */
	    hrtimer_start(&vibe_timer, ktime_set(0, 5000000), HRTIMER_MODE_REL);

        g_bTimerStarted = true;
    }
    /* 
    ** Use interruptible version of down to be safe 
    ** (try to not being stuck here if the mutex is not freed for any reason)
    */
    down_interruptible(&g_hMutex);  /* wait for the mutex to be freed by the timer */
}

static void VibeOSKernelLinuxStopTimer(void)
{
	DbgOut((KERN_INFO "===>Calling Vibe Stop Timer\n"));
	hrtimer_cancel(&vibe_timer);
	
    if (g_bTimerStarted)
    {
        g_bTimerStarted = false;
    }
} 

static void VibeOSKernelLinuxTerminateTimer(void)
{
	DbgOut((KERN_INFO "===>Terminate the Timer..........\n"));
    VibeOSKernelLinuxStopTimer();
    if (sem_is_locked(&g_hMutex)) up(&g_hMutex);

}

