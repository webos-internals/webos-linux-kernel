#include <linux/notifier.h>

int timechange_notifier_register(struct notifier_block *nb);
int timechange_notifier_unregister(struct notifier_block *nb);

void timechange_notify(void);

