#ifndef _LINUX_BATTERY_H_
#define _LINUX_BATTERY_H_

struct battery_ops {
	int (*get_percent)(int *ret_percent);           /* Percent charge */
	int (*get_voltage)(int *ret_voltage);           /* In uV */
	int (*get_temperature)(int *ret_temperature);   /* In C  */
	int (*get_current)(int *ret_current);           /* In uA */
};

extern void battery_set_ops(struct battery_ops *ops);

void battery_simple_register(struct battery_ops *ops);
void battery_simple_unegister(struct battery_ops *ops);

int battery_get_percent(int *ret_percent);
int battery_get_voltage(int *ret_voltage);
int battery_get_temperature(int *ret_temperature);
int battery_get_current(int *ret_current);

#endif // _LINUX_BATTERY_H_
