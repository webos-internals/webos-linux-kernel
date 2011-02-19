/*
** =========================================================================
** File:
**     vtmdrv.h
**
** Description: 
**     Constants and type definitions for the VibeTonz Kernel Module.
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

#ifndef _VTMDRV_H
#define _VTMDRV_H

#define MODULE_NAME                         VIBE_DEVICE
#define VTMDRV                              "/dev/vtmdrv"
#define VTMDRV_MAGIC_NUMBER                 0x494D4D52
#define VTMDRV_STOP_KERNEL_TIMER            1
#define VTMDRV_IDENTIFY_CALLER              2
#define VIBE_MAX_DEVICE_NAME_LENGTH			64

struct vibe_state;
int register_vibetonz(struct vibe_state *);
int cleanup_vibetonz(void);

/* Type definition */
#ifdef __KERNEL__
typedef int8_t		VibeInt8;
typedef u_int8_t	VibeUInt8;
typedef int16_t		VibeInt16;
typedef u_int16_t	VibeUInt16;
typedef int32_t		VibeInt32;
typedef u_int32_t	VibeUInt32;
typedef u_int8_t	VibeBool;
typedef VibeInt32	VibeStatus;
#endif

/* Error and Return value codes */
#define VIBE_S_SUCCESS                  0	/*!< Success */
#define VIBE_E_FAIL						-4	/*!< Generic error */


/* Kernel Debug Macros */
#ifdef __KERNEL__
    #ifdef VIBE_DEBUG
        #define DbgOut(_x_) printk _x_
    #else   /* VIBE_DEBUG */
        #define DbgOut(_x_)
    #endif  /* VIBE_DEBUG */
#endif  /* __KERNEL__ */

#endif  /* _VTMDRV_H */
