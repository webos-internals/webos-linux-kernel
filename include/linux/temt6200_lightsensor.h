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

#ifndef _TEMT6200_LIGHTSENSOR_H
#define _TEMT6200_LIGHTSENSOR_H

#define TEMT6200_DRIVER	"temt6200_light"
#define TEMT6200_DEVICE	"temt6200_light"

/* temt6200 lightsensor platform data structure */
struct temt6200_platform_data {
    char *desc;       // device name
    int   channel;    // madc channel number
    int   average;    // enable averaging 
    void  (*enable_lgt)(int enable);
};

#endif // TEMT6200 
