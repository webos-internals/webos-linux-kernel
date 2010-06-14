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

#ifndef _MEDIA_SMIA10_H
#define _MEDIA_SMIA10_H

#include <linux/types.h>

enum {
	SMIA10_SIMPLIFIED_2_BYTE_FORMAT = 0x0A,
	SMIA10_CCI_REG_INDEX_MSB = 0xAA,
	SMIA10_CCI_REG_INDEX_LSB = 0xA5,
	SMIA10_AUTO_INC_VALID_DATA = 0x5A,
	SMIA10_AUTO_INC_NULL_DATA = 0x55,
	SMIA10_END_OF_DATA = 0x07
};

struct smia10_parse_state {
	char	*data;
	size_t	size;
	u8	flags;
	u8	format;
	u8	tag;
	u16	index;
};

#define smia10_parse_init(p, d, s)		\
	do {					\
		memset((p), 0, sizeof(*(p)));	\
		(p)->data = (d);		\
		(p)->size = (s);		\
	} while(0)
	
extern int
smia10_parse_embedded_data(struct smia10_parse_state *state);

#endif // _MEDIA_SMIA10_H
