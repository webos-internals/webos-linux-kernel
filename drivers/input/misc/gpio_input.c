/* drivers/input/misc/gpio_input.c
 *
 * Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/gpio_event.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
#endif
#include <asm/gpio.h>
#include <asm/io.h>

#define GPIO_KEY_DEBOUNCE_STATE_UNSTABLE 1
#define GPIO_KEY_DEBOUNCE_STATE_PRESSED 2
#define GPIO_KEY_DEBOUNCE_STATE_NOTPRESSED 4
#define GPIO_KEY_DEBOUNCE_STATE_WAIT_IRQ 8
#define GPIO_KEY_DEBOUNCE_STATE_POLL 16

struct gpio_key_state {
	struct gpio_input_state *ds;
	signed char debounce_state;
};

struct gpio_input_state {
	struct input_dev *input_dev;
	const struct gpio_event_input_info *info;
	struct hrtimer timer;
	int use_irq;
	atomic_t debounce_active_count;
#ifdef CONFIG_ANDROID_POWER
	android_suspend_lock_t suspend_lock;
#endif
	struct gpio_key_state key_state[0];
};

static enum hrtimer_restart gpio_event_input_timer_func(struct hrtimer *timer)
{
	int i;
	int pressed;
	struct gpio_input_state *ds = container_of(timer, struct gpio_input_state, timer);
	int debounce = atomic_read(&ds->debounce_active_count);
	unsigned gpio_direct_flags = ds->info->flags;
	unsigned npolarity;
	int nkeys = ds->info->keymap_size;
	const struct gpio_event_direct_entry *key_entry;
	struct gpio_key_state *key_state;

#if 0
	key_entry = kp->keys_info->keymap;
	key_state = kp->key_state;
	for(i = 0; i < nkeys; i++, key_entry++, key_state++) {
		printk("gpio_read_detect_status %d %d\n", key_entry->gpio, gpio_read_detect_status(key_entry->gpio));
	}
#endif
	key_entry = ds->info->keymap;
	key_state = ds->key_state;
	for(i = 0; i < nkeys; i++, key_entry++, key_state++) {
		if (key_state->debounce_state & GPIO_KEY_DEBOUNCE_STATE_WAIT_IRQ)
			continue;
		if (gpio_read_detect_status(key_entry->gpio) > 0) {
			gpio_clear_detect_status(key_entry->gpio);
			if (key_state->debounce_state & GPIO_KEY_DEBOUNCE_STATE_POLL) {
				debounce = atomic_inc_return(&ds->debounce_active_count);
				if (gpio_direct_flags & GPIOEDF_PRINT_KEY_DEBOUNCE)
					printk("gpio_keys_scan_keys: key %x-%x, %d (%d) start debounce\n", ds->info->type, key_entry->code, i, key_entry->gpio);
			} else if(gpio_direct_flags & GPIOEDF_PRINT_KEY_DEBOUNCE)
					printk("gpio_keys_scan_keys: key %x-%x, %d (%d) continue debounce\n", ds->info->type, key_entry->code, i, key_entry->gpio);
			key_state->debounce_state = GPIO_KEY_DEBOUNCE_STATE_UNSTABLE;
			continue;
		}
		npolarity = !(gpio_direct_flags & GPIOEDF_ACTIVE_HIGH);
		pressed = gpio_get_value(key_entry->gpio) ^ npolarity;
		if (key_state->debounce_state & GPIO_KEY_DEBOUNCE_STATE_POLL) {
			if (pressed == !(key_state->debounce_state & GPIO_KEY_DEBOUNCE_STATE_PRESSED)) {
				debounce = atomic_inc_return(&ds->debounce_active_count);
				key_state->debounce_state = GPIO_KEY_DEBOUNCE_STATE_UNSTABLE;
				if (gpio_direct_flags & GPIOEDF_PRINT_KEY_DEBOUNCE)
					printk("gpio_keys_scan_keys: key %x-%x, %d (%d) start debounce\n", ds->info->type, key_entry->code, i, key_entry->gpio);
			}
			continue;
		}
		if (pressed && (key_state->debounce_state & (GPIO_KEY_DEBOUNCE_STATE_UNSTABLE | GPIO_KEY_DEBOUNCE_STATE_NOTPRESSED))) {
			if (gpio_direct_flags & GPIOEDF_PRINT_KEY_DEBOUNCE)
					printk("gpio_keys_scan_keys: key %x-%x, %d (%d) debounce pressed 1\n", ds->info->type, key_entry->code, i, key_entry->gpio);
			key_state->debounce_state = GPIO_KEY_DEBOUNCE_STATE_PRESSED;
			continue;
		}
		if (!pressed && (key_state->debounce_state & (GPIO_KEY_DEBOUNCE_STATE_UNSTABLE | GPIO_KEY_DEBOUNCE_STATE_PRESSED))) {
			if (gpio_direct_flags & GPIOEDF_PRINT_KEY_DEBOUNCE)
					printk("gpio_keys_scan_keys: key %x-%x, %d (%d) debounce pressed 0\n", ds->info->type, key_entry->code, i, key_entry->gpio);
			key_state->debounce_state = GPIO_KEY_DEBOUNCE_STATE_NOTPRESSED;
			continue;
		}
		/* key is stable */
		debounce = atomic_dec_return(&ds->debounce_active_count);
		if (ds->use_irq) {
			key_state->debounce_state |= GPIO_KEY_DEBOUNCE_STATE_WAIT_IRQ;
			enable_irq(gpio_to_irq(ds->info->keymap[i].gpio));
		} else {
			key_state->debounce_state |= GPIO_KEY_DEBOUNCE_STATE_POLL;
		}
		if (gpio_direct_flags & GPIOEDF_PRINT_KEYS)
			printk("gpio_keys_scan_keys: key %x-%x, %d (%d) changed to %d\n", ds->info->type, key_entry->code, i, key_entry->gpio, pressed);
		input_event(ds->input_dev, ds->info->type, key_entry->code, pressed);
	}

#if 0
	key_entry = kp->keys_info->keymap;
	key_state = kp->key_state;
	for(i = 0; i < nkeys; i++, key_entry++, key_state++) {
		printk("gpio_read_detect_status %d %d\n", key_entry->gpio, gpio_read_detect_status(key_entry->gpio));
	}
#endif

	if (debounce) {
		hrtimer_start(timer, ds->info->debounce_time, HRTIMER_MODE_REL);
	} else if(!ds->use_irq) {
		hrtimer_start(timer, ds->info->poll_time, HRTIMER_MODE_REL);
	}
#ifdef CONFIG_ANDROID_POWER
	else
		android_unlock_suspend(&ds->suspend_lock);
#endif

	return HRTIMER_NORESTART;
}

static irqreturn_t gpio_event_input_irq_handler(int irq, void *dev_id)
{
	struct gpio_key_state *ks = dev_id;
	struct gpio_input_state *ds = ks->ds;
	int keymap_index = ks - ds->key_state;
	const struct gpio_event_direct_entry *key_entry = &ds->info->keymap[keymap_index];
	int pressed;

	if (ds->info->debounce_time.tv64) {
		disable_irq(irq);
		ks->debounce_state = GPIO_KEY_DEBOUNCE_STATE_UNSTABLE;
		if (atomic_inc_return(&ds->debounce_active_count) == 1) {
#ifdef CONFIG_ANDROID_POWER
			android_lock_suspend(&ds->suspend_lock);
#endif
			hrtimer_start(&ds->timer, ds->info->debounce_time, HRTIMER_MODE_REL);
		}
		if (ds->info->flags & GPIOEDF_PRINT_KEY_DEBOUNCE)
			printk("gpio_event_input_irq_handler: key %x-%x, %d (%d) start debounce\n", ds->info->type, key_entry->code, keymap_index, key_entry->gpio);
	} else {
		pressed = gpio_get_value(key_entry->gpio) ^ !(ds->info->flags & GPIOEDF_ACTIVE_HIGH);
		if (ds->info->flags & GPIOEDF_PRINT_KEYS)
			printk("gpio_event_input_irq_handler: key %x-%x, %d (%d) changed to %d\n", ds->info->type, key_entry->code, keymap_index, key_entry->gpio, pressed);
		input_event(ds->input_dev, ds->info->type, key_entry->code, pressed);
	}
	return IRQ_HANDLED;
}

static int gpio_event_input_request_irqs(struct gpio_input_state *ds)
{
	int i;
	int err;
	unsigned int irq;
	unsigned long request_flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;
	unsigned long irq_flags;

	local_irq_save(irq_flags); // request_irq does not have an option to request an irq in the disabled state

	for(i = 0; i < ds->info->keymap_size; i++) {
		err = irq = gpio_to_irq(ds->info->keymap[i].gpio);
		if (err < 0)
			goto err_gpio_get_irq_num_failed;
		err = request_irq(irq, gpio_event_input_irq_handler, request_flags, "gpio_keys", &ds->key_state[i]);
		if (err) {
			printk(KERN_ERR "gpio_event_input_request_irqs: request_irq failed for input %d, irq %d\n", ds->info->keymap[i].gpio, irq);
			goto err_request_irq_failed;
		}
		disable_irq(irq);
		enable_irq_wake(irq);
	}
	local_irq_restore(irq_flags);
	return 0;

	for(i = ds->info->keymap_size - 1; i >= 0; i--) {
		free_irq(gpio_to_irq(ds->info->keymap[i].gpio), &ds->key_state[i]);
err_request_irq_failed:
err_gpio_get_irq_num_failed:
		;
	}
	local_irq_restore(irq_flags);
	return err;
}

int gpio_event_input_func(struct input_dev *input_dev, struct gpio_event_info *info, void **data, int func)
{
	int ret;
	int i;
	unsigned long irq_flags;
	struct gpio_event_input_info *di = container_of(info, struct gpio_event_input_info, info);
	struct gpio_input_state *ds = *data;

	if (func == GPIO_EVENT_FUNC_SUSPEND) {
		local_irq_save(irq_flags);
		hrtimer_cancel(&ds->timer);
		if (ds->use_irq)
			for(i = 0; i < di->keymap_size; i++)
				disable_irq(gpio_to_irq(di->keymap[i].gpio));
		local_irq_restore(irq_flags);
		return 0;
	}
	if (func == GPIO_EVENT_FUNC_RESUME) {
		local_irq_save(irq_flags);
		if (ds->use_irq)
			for(i = 0; i < di->keymap_size; i++)
				enable_irq(gpio_to_irq(di->keymap[i].gpio));
		hrtimer_start(&ds->timer, ktime_set(0, 0), HRTIMER_MODE_REL);
		local_irq_restore(irq_flags);
		return 0;
	}

	if (func == GPIO_EVENT_FUNC_INIT) {
		if (ktime_to_ns(di->poll_time) <= 0)
			di->poll_time = ktime_set(0, 20 * NSEC_PER_MSEC);

		*data = ds = kzalloc(sizeof(*ds) + sizeof(ds->key_state[0]) * di->keymap_size, GFP_KERNEL);
		if (ds == NULL) {
			ret = -ENOMEM;
			printk(KERN_ERR "gpio_event_input_func: Failed to allocate private data\n");
			goto err_ds_alloc_failed;
		}
		atomic_set(&ds->debounce_active_count, di->keymap_size);
		ds->input_dev = input_dev;
		ds->info = di;
#ifdef CONFIG_ANDROID_POWER
		ds->suspend_lock.name = "gpio_input";
		android_init_suspend_lock(&ds->suspend_lock);
#endif

		for(i = 0; i < di->keymap_size; i++) {
			input_set_capability(input_dev, di->type, di->keymap[i].code);
			ds->key_state[i].ds = ds;
			ds->key_state[i].debounce_state = GPIO_KEY_DEBOUNCE_STATE_UNSTABLE;
		}

		for(i = 0; i < di->keymap_size; i++) {
			ret = gpio_request(di->keymap[i].gpio, "gpio_kp_in");
			if (ret) {
				printk(KERN_ERR "gpio_event_input_func: gpio_request failed for %d\n", di->keymap[i].gpio);
				goto err_gpio_request_failed;
			}
			ret = gpio_configure(di->keymap[i].gpio, GPIOF_INPUT | IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING);
			if (ret) {
				printk(KERN_ERR "gpio_event_input_func: gpio_configure failed for %d\n", di->keymap[i].gpio);
				goto err_gpio_configure_failed;
			}
		}

		ret = gpio_event_input_request_irqs(ds);
		ds->use_irq = ret == 0;

		printk(KERN_INFO "GPIO Input Driver: Start gpio inputs for %s in %s mode\n", input_dev->name, ds->use_irq ? "interrupt" : "polling");

		hrtimer_init(&ds->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ds->timer.function = gpio_event_input_timer_func;
		hrtimer_start(&ds->timer, ktime_set(0, 0), HRTIMER_MODE_REL);
		return 0;
	}

	ret = 0;
	local_irq_save(irq_flags);
	hrtimer_cancel(&ds->timer);
	if (ds->use_irq) {
		for(i = di->keymap_size - 1; i >= 0; i--) {
			free_irq(gpio_to_irq(di->keymap[i].gpio), &ds->key_state[i]);
		}
	}
	local_irq_restore(irq_flags);

	for(i = di->keymap_size - 1; i >= 0; i--) {
err_gpio_configure_failed:
		gpio_free(di->keymap[i].gpio);
err_gpio_request_failed:
		;
	}
#ifdef CONFIG_ANDROID_POWER
	android_uninit_suspend_lock(&ds->suspend_lock);
#endif
	kfree(ds);
err_ds_alloc_failed:
	return ret;
}

