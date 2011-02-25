/*
** =========================================================================
** File:
**     vtmdrv.c
**
** Description: 
**     VibeTonz Kernel Module main entry-point.
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

#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/vibrator.h>
#include <asm/uaccess.h>
#include "vtmdrv.h"
#include "vibrator.h"

/* Device name and version information */
#define VERSION_STR " v2.0.92.1\n"                  /* PLEASE DO NOT CHANGE - this is auto-generated */
#define VERSION_STR_LEN 16                          /* account extra space for future extra digits in version number */
static char g_szDeviceName[   VIBE_MAX_DEVICE_NAME_LENGTH 
                            + VERSION_STR_LEN];     /* initialized in init_module */
static size_t g_cchDeviceName;                      /* initialized in init_module */

static struct vibe_state* g_state;
#if ((LINUX_VERSION_CODE & 0xFFFF00) < KERNEL_VERSION(2,6,0))
#error Unsupported Kernel version
#endif

/* Needs to be included after the global variables because it uses them */
#include "VibeOSKernelLinuxTime.c"

/* File IO */
static int open(struct inode *inode, struct file *file);
static int release(struct inode *inode, struct file *file);
static ssize_t read(struct file *file, char *buf, size_t count, loff_t *ppos);
static ssize_t write(struct file *file, const char *buf, size_t count, loff_t *ppos);
static int ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
static struct file_operations fops = 
{
    .owner =    THIS_MODULE,
    .read =     read,
    .write =    write,
    .ioctl =    ioctl,
    .open =     open,
    .release =  release
};

static struct miscdevice miscdev = 
{
	.minor =    MISC_DYNAMIC_MINOR,
	.name =     "vtmdrv",
	.fops =     &fops
};

int register_vibetonz(struct vibe_state *state)
{
    misc_register(&miscdev);
    VibeOSKernelLinuxInitTimer();

    /* Get device name */
    strcpy(g_szDeviceName,"Vibetonz Palm");
    /* Append version information and get buffer length */
    strcat(g_szDeviceName, VERSION_STR);
    g_cchDeviceName = strlen(g_szDeviceName);

    g_state = state;
    return 0;
}

int cleanup_vibetonz()
{
    VibeOSKernelLinuxTerminateTimer();
    misc_deregister(&miscdev);
    g_state = NULL;
    return 0;
}

static int open(struct inode *inode, struct file *file) 
{
    DbgOut((KERN_INFO "vtmdrv: open.\n"));

    return 0; 
}

static int release(struct inode *inode, struct file *file) 
{
    DbgOut((KERN_INFO "vtmdrv: release.\n"));

    /* 
    ** Reset force and stop timer when the driver is closed, to make sure
    ** no dangling semaphore remains in the system, especially when the
    ** driver is run outside of immvibed for testing purposes.
    */
    VibeOSKernelLinuxStopTimer();

    return 0; 
}

static ssize_t read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
    const size_t nBufSize = (g_cchDeviceName > (size_t)(*ppos)) ? min(count, g_cchDeviceName - (size_t)(*ppos)) : 0;

    /* End of buffer, exit */
    if (0 == nBufSize) return 0;

    if (0 != copy_to_user(buf, g_szDeviceName + (*ppos), nBufSize)) 
    {
        /* Failed to copy all the data, exit */
        DbgOut((KERN_ERR "vtmdrv: copy_to_user failed.\n"));
        return 0;
    }

    /* Update file position and return copied buffer size */
    *ppos += nBufSize;
    return nBufSize;
}

static ssize_t write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
    signed char cBuffer[1];
    int tmp;

    *ppos = 0;  /* file position not used, always set to 0 */

    /* 
    ** Prevent unauthorized caller to write data. 
    ** VibeTonz service is the only valid caller.
    */
    if (file->private_data != (void*)VTMDRV_MAGIC_NUMBER) 
    {
        DbgOut((KERN_ERR "vtmdrv: unauthorized write.\n"));
        return 0;
    }

    /* Check buffer size */
    if (count != 1) 
    {
        DbgOut((KERN_ERR "vtmdrv: invalid write buffer size.\n"));
        return 0;
    }

    if (0 != copy_from_user(cBuffer, buf, count)) 
    {
        /* Failed to copy all the data, exit */
        DbgOut((KERN_ERR "vtmdrv: copy_from_user failed.\n"));
        return 0;
    }

    if(cBuffer[0] == 0) {
        _direction_store(g_state,0);
    } else if(cBuffer[0] < 0) {
        tmp = (-1)*cBuffer[0]*100/127;
        _duty_cycle_store(g_state,tmp);
        _direction_store(g_state,-1);
    } else {
        tmp = cBuffer[0]*100/127;
        _duty_cycle_store(g_state,tmp);
        _direction_store(g_state,1);
    }

    /* Start the timer after receiving new output force */
    VibeOSKernelLinuxStartTimer();

    return count;
}

static int ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
    switch (cmd)
    {
        case VTMDRV_STOP_KERNEL_TIMER:
            _direction_store(g_state,0);
            VibeOSKernelLinuxStopTimer();
            break;

        case VTMDRV_IDENTIFY_CALLER:
            if (VTMDRV_MAGIC_NUMBER == arg) file->private_data = (void*)VTMDRV_MAGIC_NUMBER;
            break;
    }

    return 0;
}

EXPORT_SYMBOL(register_vibetonz);
EXPORT_SYMBOL(cleanup_vibetonz);
