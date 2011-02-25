/*
 * Copyright (C) 2008-2009 Palm, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __HRES_COUNTER_INCLUDED__
#define __HRES_COUNTER_INCLUDED__

#ifdef CONFIG_HRES_COUNTER

extern u32   hres_get_counter ( void );
extern void  hres_ch_reset    ( uint ch ); 
extern void  hres_event_cnt   ( uint ch );
extern void  hres_event_start ( uint ch );
extern u32   hres_event_end   ( uint ch );
extern void  hres_event       ( char *type, u32 arg1, u32 arg2 );
extern int   hres_evlog_enable  ( void );
extern int   hres_evlog_disable ( void );
extern void  hres_evlog_print   ( void );
extern void  hres_evlog_reset   ( void );

#else

#define hres_get_counter(args...)
#define hres_ch_reset(args...)
#define hres_event_cnt(args...)
#define hres_event_start(args...)
#define hres_event_end(args...)
#define hres_event(args...)
#define hres_evlog_enable(args...)
#define hres_evlog_disable(args...)
#define hres_evlog_print(args...)
#define hres_evlog_reset(args...)

#endif


#endif // __HRES_COUNTER_INCLUDED__



