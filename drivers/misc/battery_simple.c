/*
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

#include <linux/battery_simple.h>
#include <linux/stddef.h>

struct battery_ops *battery_ops = NULL;

void battery_set_ops(struct battery_ops *ops)
{
	battery_ops = ops;
}

int battery_get_percent(int *ret_percent)
{
	if (!battery_ops) return -1;
	return battery_ops->get_percent(ret_percent);
}

int battery_get_voltage(int *ret_voltage)
{
	if (!battery_ops) return -1;
	return battery_ops->get_voltage(ret_voltage);
}

int battery_get_temperature(int *ret_temperature)
{
	if (!battery_ops) return -1;
	return battery_ops->get_temperature(ret_temperature);
}

int battery_get_current(int *ret_current)
{
	if (!battery_ops) return -1;
	return battery_ops->get_current(ret_current);
}

