/*
 * drivers/media/video/smia10.c
 *
 * Copyright (C) 2008-2009 Palm, Inc.
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
 */

#include <linux/errno.h>
#include <linux/module.h>
#include <media/smia10.h>

enum {
	FORMAT_VALID = 0x01,
	TAG_VALID = 0x02,
	MSB_VALID = 0x04,
	LSB_VALID = 0x08,
};

#define test_flags(p, f)	((f) == ((f) & (p)->flags))
#define set_flags(p, f)		((p)->flags |= (f))
#define clear_flags(p, f)	((p)->flags &= ~(f))
#define pop_byte(p)		((p)->size--, *(p)->data++)

static int
smia10_parse_simplified_2_byte(struct smia10_parse_state *state)
{
	int rc = 0;
	
	/* TODO: dummy bytes */
	
	if (test_flags(state, TAG_VALID)) {
		if (SMIA10_AUTO_INC_VALID_DATA == state->tag) {
			state->index++;
			(void)pop_byte(state);
		}
		else
			goto data;
	}
tag:
	clear_flags(state, TAG_VALID);
	
	if (!state->size)
		goto exit;
	
	state->tag = pop_byte(state);
	set_flags(state, TAG_VALID);
data:
	if (!state->size)
		goto exit;
		
	switch (state->tag) {
	case SMIA10_END_OF_DATA:
		break;
	case SMIA10_CCI_REG_INDEX_MSB:
		state->index = pop_byte(state) << 8;
		set_flags(state, MSB_VALID);
		clear_flags(state, LSB_VALID);
		break;
	case SMIA10_CCI_REG_INDEX_LSB:
		if (!test_flags(state, MSB_VALID)) {
			rc = -EINVAL;
			goto exit;
		}
		
		state->index &= 0xFF00;
		state->index |= pop_byte(state);
		set_flags(state, LSB_VALID);
		break;
	case SMIA10_AUTO_INC_VALID_DATA:
		if (!test_flags(state, MSB_VALID|LSB_VALID))
			rc = -EINVAL;
		
		goto exit;
	case SMIA10_AUTO_INC_NULL_DATA:
		if (!test_flags(state, MSB_VALID|LSB_VALID)) {
			rc = -EINVAL;
			goto exit;
		}
		
		(void)pop_byte(state);
		state->index++;
		break;
	default:
		rc = -EINVAL;
		goto exit;
	}
	
	goto tag;
exit:
	return (rc);
}

int
smia10_parse_embedded_data(struct smia10_parse_state *state)
{
	int rc = 0;
	
	if (!state->size)
		goto exit;
	
	if (!test_flags(state, FORMAT_VALID)) {
		state->format = pop_byte(state);
		set_flags(state, FORMAT_VALID);
	}
	
	switch (state->format) {
	case SMIA10_SIMPLIFIED_2_BYTE_FORMAT:
		rc = smia10_parse_simplified_2_byte(state);
		break;
	case 0x00:
	case SMIA10_END_OF_DATA:
	case 0xFF:
		break;
	}
exit:
	return (rc);
}
EXPORT_SYMBOL(smia10_parse_embedded_data);
