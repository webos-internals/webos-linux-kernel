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

