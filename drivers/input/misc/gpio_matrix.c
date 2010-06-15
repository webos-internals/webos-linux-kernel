/* drivers/input/misc/gpio_matrix.c
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

#include <asm/gpio.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/gpio_event.h>
#include <linux/hrtimer.h>
#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
#endif

struct gpio_kp {
	struct input_dev *input_dev;
	struct gpio_event_matrix_info *keypad_info;
	struct hrtimer timer;
#ifdef CONFIG_ANDROID_POWER
	android_suspend_lock_t suspend_lock;
#endif
	int current_output;
	unsigned int use_irq : 1;
	unsigned int key_state_changed : 1;
	unsigned int last_key_state_changed : 1;
	unsigned int some_keys_pressed : 2;
	unsigned long keys_pressed[0];
};

static void gpio_keypad_clear_phantom_key(struct gpio_kp *kp, int out, int in)
{
	int key_index = out * kp->keypad_info->ninputs + in;
	unsigned short keycode = kp->keypad_info->keymap[key_index];;

	if (!test_bit(keycode, kp->input_dev->key)) {
		if (kp->keypad_info->flags & GPIOKPF_PRINT_PHANTOM_KEYS)
			printk("gpio_keypad_scan_keys: phantom key %x, %d-%d (%d-%d) cleared\n", keycode, out, in, kp->keypad_info->output_gpios[out], kp->keypad_info->input_gpios[in]);
		__clear_bit(key_index, kp->keys_pressed);
	} else {
		if (kp->keypad_info->flags & GPIOKPF_PRINT_PHANTOM_KEYS)
			printk("gpio_keypad_scan_keys: phantom key %x, %d-%d (%d-%d) not cleared\n", keycode, out, in, kp->keypad_info->output_gpios[out], kp->keypad_info->input_gpios[in]);
	}
}

static int gpio_keypad_restore_keys_for_input(struct gpio_kp *kp, int out, int in)
{
	int rv = 0;
	int key_index;

	key_index = out * kp->keypad_info->ninputs + in;
	while (out < kp->keypad_info->noutputs) {
		if (test_bit(key_index, kp->keys_pressed)) {
			rv = 1;
			gpio_keypad_clear_phantom_key(kp, out, in);
		}
		key_index += kp->keypad_info->ninputs;
		out++;
	}
	return rv;
}

static void gpio_keypad_remove_phantom_keys(struct gpio_kp *kp)
{
	int out, in, inp;
	int key_index;

	if (kp->some_keys_pressed < 3)
		return;

	for(out = 0; out < kp->keypad_info->noutputs; out++) {
		inp = -1;
		key_index = out * kp->keypad_info->ninputs;
		for(in = 0; in < kp->keypad_info->ninputs; in++, key_index++) {
			if (test_bit(key_index, kp->keys_pressed)) {
				if (inp == -1) {
					inp = in;
					continue;
				}
				if (inp >= 0) {
					if (!gpio_keypad_restore_keys_for_input(kp, out + 1, inp)) {
						break;
					}
					gpio_keypad_clear_phantom_key(kp, out, inp);
					inp = -2;
				}
				gpio_keypad_restore_keys_for_input(kp, out, in);
			}
		}
	}
}

static enum hrtimer_restart gpio_keypad_timer_func(struct hrtimer *timer)
{
	int out, in;
	int key_index;
	struct gpio_kp *kp = container_of(timer, struct gpio_kp, timer);
	unsigned gpio_keypad_flags = kp->keypad_info->flags;
	unsigned npolarity = !(gpio_keypad_flags & GPIOKPF_ACTIVE_HIGH);

	out = kp->current_output;
	if (out == kp->keypad_info->noutputs) {
		out = 0;
		kp->last_key_state_changed = kp->key_state_changed;
		kp->key_state_changed = 0;
		kp->some_keys_pressed = 0;
	} else {
		key_index = out * kp->keypad_info->ninputs;
		for(in = 0; in < kp->keypad_info->ninputs; in++, key_index++) {
			if (gpio_get_value(kp->keypad_info->input_gpios[in]) ^ npolarity) {
				if (kp->some_keys_pressed < 3)
					kp->some_keys_pressed++;
				kp->key_state_changed |= !__test_and_set_bit(key_index, kp->keys_pressed);
			} else
				kp->key_state_changed |= __test_and_clear_bit(key_index, kp->keys_pressed);
		}
		if (gpio_keypad_flags & GPIOKPF_DRIVE_INACTIVE)
			gpio_set_value(kp->keypad_info->output_gpios[out], npolarity);
		else
			gpio_configure(kp->keypad_info->output_gpios[out], GPIOF_INPUT);
		out++;
	}
	kp->current_output = out;
	if (out < kp->keypad_info->noutputs) {
		if (gpio_keypad_flags & GPIOKPF_DRIVE_INACTIVE)
			gpio_set_value(kp->keypad_info->output_gpios[out], !npolarity);
		else
			gpio_configure(kp->keypad_info->output_gpios[out], GPIOF_DRIVE_OUTPUT);
		hrtimer_start(timer, kp->keypad_info->settle_time, HRTIMER_MODE_REL);
		return HRTIMER_NORESTART;
	}
	if (gpio_keypad_flags & GPIOKPF_DEBOUNCE) {
		if (kp->key_state_changed) {
			hrtimer_start(&kp->timer, kp->keypad_info->debounce_delay, HRTIMER_MODE_REL);
			return HRTIMER_NORESTART;
		}
		kp->key_state_changed = kp->last_key_state_changed;
	}
	if (kp->key_state_changed) {
		if (gpio_keypad_flags & GPIOKPF_REMOVE_SOME_PHANTOM_KEYS)
			gpio_keypad_remove_phantom_keys(kp);
		key_index = 0;
		for(out = 0; out < kp->keypad_info->noutputs; out++) {
			for(in = 0; in < kp->keypad_info->ninputs; in++, key_index++) {
				int pressed = test_bit(key_index, kp->keys_pressed);
				unsigned short keycode = kp->keypad_info->keymap[key_index];
				if (pressed != test_bit(keycode, kp->input_dev->key)) {
					if (keycode == KEY_RESERVED) {
						if (gpio_keypad_flags & GPIOKPF_PRINT_UNMAPPED_KEYS)
							printk("gpio_keypad_scan_keys: unmapped key, %d-%d (%d-%d) changed to %d\n", out, in, kp->keypad_info->output_gpios[out], kp->keypad_info->input_gpios[in], pressed);
					} else {
						if (gpio_keypad_flags & GPIOKPF_PRINT_MAPPED_KEYS)
							printk("gpio_keypad_scan_keys: key %x, %d-%d (%d-%d) changed to %d\n", keycode, out, in, kp->keypad_info->output_gpios[out], kp->keypad_info->input_gpios[in], pressed);
						input_report_key(kp->input_dev, keycode, pressed);
					}
				}
			}
		}
	}
	if (!kp->use_irq || kp->some_keys_pressed) {
		hrtimer_start(timer, kp->keypad_info->poll_time, HRTIMER_MODE_REL);
		return HRTIMER_NORESTART;
	}

	/* No keys are pressed, reenable interrupt */
	for(out = 0; out < kp->keypad_info->noutputs; out++) {
		if (gpio_keypad_flags & GPIOKPF_DRIVE_INACTIVE)
			gpio_set_value(kp->keypad_info->output_gpios[out], !npolarity);
		else
			gpio_configure(kp->keypad_info->output_gpios[out], GPIOF_DRIVE_OUTPUT);
	}
	for(in = 0; in < kp->keypad_info->ninputs; in++) {
		enable_irq(gpio_to_irq(kp->keypad_info->input_gpios[in]));
	}
#ifdef CONFIG_ANDROID_POWER
	android_unlock_suspend(&kp->suspend_lock);
#endif
	return HRTIMER_NORESTART;
}

static irqreturn_t gpio_keypad_irq_handler(int irq_in, void *dev_id)
{
	int i;
	struct gpio_kp *kp = dev_id;
	unsigned gpio_keypad_flags = kp->keypad_info->flags;

	for(i = 0; i < kp->keypad_info->ninputs; i++) {
		disable_irq(gpio_to_irq(kp->keypad_info->input_gpios[i]));
	}
	for(i = 0; i < kp->keypad_info->noutputs; i++) {
		if (gpio_keypad_flags & GPIOKPF_DRIVE_INACTIVE)
			gpio_set_value(kp->keypad_info->output_gpios[i], !(gpio_keypad_flags & GPIOKPF_ACTIVE_HIGH));
		else
			gpio_configure(kp->keypad_info->output_gpios[i], GPIOF_INPUT);
	}
#ifdef CONFIG_ANDROID_POWER
	android_lock_suspend(&kp->suspend_lock);
#endif
	hrtimer_start(&kp->timer, ktime_set(0, 0), HRTIMER_MODE_REL);
	return IRQ_HANDLED;
}

static int gpio_keypad_request_irqs(struct gpio_kp *kp)
{
	int i;
	int err;
	unsigned int irq;
	unsigned long request_flags;
	unsigned long irq_flags;

	switch(kp->keypad_info->flags & (GPIOKPF_ACTIVE_HIGH | GPIOKPF_LEVEL_TRIGGERED_IRQ)) {
		default:
			request_flags = IRQF_TRIGGER_FALLING;
			break;
		case GPIOKPF_ACTIVE_HIGH:
			request_flags = IRQF_TRIGGER_RISING;
			break;
		case GPIOKPF_LEVEL_TRIGGERED_IRQ:
			request_flags = IRQF_TRIGGER_LOW;
			break;
		case GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_ACTIVE_HIGH:
			request_flags = IRQF_TRIGGER_HIGH;
			break;
	}

	local_irq_save(irq_flags); // request_irq does not have an option to request an irq in the disabled state

	for(i = 0; i < kp->keypad_info->ninputs; i++) {
		err = irq = gpio_to_irq(kp->keypad_info->input_gpios[i]);
		if (err < 0)
			goto err_gpio_get_irq_num_failed;
		err = request_irq(irq, gpio_keypad_irq_handler, request_flags, "gpio_kp", kp);
		if (err) {
			printk(KERN_ERR "gpio_keypad_request_irqs: request_irq failed for input %d, irq %d\n", kp->keypad_info->input_gpios[i], irq);
			goto err_request_irq_failed;
		}
		err = set_irq_wake(irq, 1);
		if (err) {
			printk(KERN_ERR "gpio_keypad_request_irqs: set_irq_wake failed for input %d, irq %d\n", kp->keypad_info->input_gpios[i], irq);
		}
		disable_irq(irq);
	}
	local_irq_restore(irq_flags);
	return 0;

	for(i = kp->keypad_info->noutputs - 1; i >= 0; i--) {
		free_irq(gpio_to_irq(kp->keypad_info->input_gpios[i]), kp);
err_request_irq_failed:
err_gpio_get_irq_num_failed:
		;
	}
	local_irq_restore(irq_flags);
	return err;
}

int gpio_event_matrix_func(struct input_dev *input_dev, struct gpio_event_info *info, void **data, int func)
{
	int i;
	int err;
	int key_count;
	unsigned gpio_out_flags;
	struct gpio_kp *kp;
	struct gpio_event_matrix_info *mi = container_of(info, struct gpio_event_matrix_info, info);

	if (func == GPIO_EVENT_FUNC_SUSPEND || func == GPIO_EVENT_FUNC_RESUME) {
		/* TODO: disable scanning */
		return 0;
	}

	if (func == GPIO_EVENT_FUNC_INIT) {
		if (mi->keymap == NULL ||
		   mi->input_gpios == NULL ||
		   mi->output_gpios == NULL) {
			err = -ENODEV;
			printk(KERN_ERR "gpio_keypad_probe: Incomplete pdata\n");
			goto err_invalid_platform_data;
		}
		key_count = mi->ninputs * mi->noutputs;

		*data = kp = kzalloc(sizeof(*kp) + sizeof(kp->keys_pressed[0]) * BITS_TO_LONGS(key_count), GFP_KERNEL);
		if (kp == NULL) {
			err = -ENOMEM;
			printk(KERN_ERR "gpio_keypad_probe: Failed to allocate private data\n");
			goto err_kp_alloc_failed;
		}
		kp->input_dev = input_dev;
		kp->keypad_info = mi;
		set_bit(EV_KEY, input_dev->evbit);
		for(i = 0; i < key_count; i++) {
			if (kp->keypad_info->keymap[i])
				set_bit(kp->keypad_info->keymap[i] & KEY_MAX, input_dev->keybit);
		}

		switch(kp->keypad_info->flags & (GPIOKPF_ACTIVE_HIGH | GPIOKPF_DRIVE_INACTIVE)) {
			default:
				gpio_out_flags = GPIOF_INPUT | GPIOF_OUTPUT_LOW;
				break;
			case GPIOKPF_ACTIVE_HIGH:
				gpio_out_flags = GPIOF_INPUT | GPIOF_OUTPUT_HIGH;
				break;
			case GPIOKPF_DRIVE_INACTIVE:
				gpio_out_flags = GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_HIGH;
				break;
			case GPIOKPF_DRIVE_INACTIVE | GPIOKPF_ACTIVE_HIGH:
				gpio_out_flags = GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW;
				break;
		}

		for(i = 0; i < kp->keypad_info->noutputs; i++) {
			err = gpio_request(kp->keypad_info->output_gpios[i], "gpio_kp_out");
			if (err) {
				printk(KERN_ERR "gpio_keypad_probe: gpio_request failed for output %d\n", kp->keypad_info->output_gpios[i]);
				goto err_request_output_gpio_failed;
			}
			err = gpio_configure(kp->keypad_info->output_gpios[i], gpio_out_flags);
			if (err) {
				printk(KERN_ERR "gpio_keypad_probe: gpio_configure failed for output %d\n", kp->keypad_info->output_gpios[i]);
				goto err_output_gpio_configure_failed;
			}
		}
		for(i = 0; i < kp->keypad_info->ninputs; i++) {
			err = gpio_request(kp->keypad_info->input_gpios[i], "gpio_kp_in");
			if (err) {
				printk(KERN_ERR "gpio_keypad_probe: gpio_request failed for input %d\n", kp->keypad_info->input_gpios[i]);
				goto err_request_input_gpio_failed;
			}
			err = gpio_direction_input(kp->keypad_info->input_gpios[i]);
			if (err) {
				printk(KERN_ERR "gpio_keypad_probe: gpio_direction_input failed for input %d\n", kp->keypad_info->input_gpios[i]);
				goto err_gpio_direction_input_failed;
			}
		}
		kp->current_output = kp->keypad_info->noutputs;
		kp->key_state_changed = 1;

		hrtimer_init(&kp->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		kp->timer.function = gpio_keypad_timer_func;
#ifdef CONFIG_ANDROID_POWER
		kp->suspend_lock.name = "gpio_kp";
		android_init_suspend_lock(&kp->suspend_lock);
#endif
		err = gpio_keypad_request_irqs(kp);
		kp->use_irq = err == 0;

		printk(KERN_INFO "GPIO Keypad Driver: Start keypad matrix for %s in %s mode\n", input_dev->name, kp->use_irq ? "interrupt" : "polling");

#ifdef CONFIG_ANDROID_POWER
		if (kp->use_irq)
			android_lock_suspend(&kp->suspend_lock);
#endif
		hrtimer_start(&kp->timer, ktime_set(0, 0), HRTIMER_MODE_REL);

		return 0;
	}

	err = 0;
	kp = *data;

	if (kp->use_irq) {
		for(i = kp->keypad_info->noutputs - 1; i >= 0; i--) {
			free_irq(gpio_to_irq(kp->keypad_info->input_gpios[i]), kp);
		}
	}

#ifdef CONFIG_ANDROID_POWER
	android_uninit_suspend_lock(&kp->suspend_lock);
#endif
	hrtimer_cancel(&kp->timer);
	for(i = kp->keypad_info->noutputs - 1; i >= 0; i--) {
err_gpio_direction_input_failed:
		gpio_free(kp->keypad_info->input_gpios[i]);
err_request_input_gpio_failed:
		;
	}
	for(i = kp->keypad_info->noutputs - 1; i >= 0; i--) {
err_output_gpio_configure_failed:
		gpio_free(kp->keypad_info->output_gpios[i]);
err_request_output_gpio_failed:
		;
	}
	kfree(kp);
err_kp_alloc_failed:
err_invalid_platform_data:
	return err;
}



