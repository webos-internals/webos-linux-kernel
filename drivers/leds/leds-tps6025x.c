/* drivers/leds/leds-tps6025x.c
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/i2c.h>
#include <linux/i2c_tps6025x_led.h>

#undef TPS6025X_DEBUG
#ifdef TPS6025X_DEBUG
#include <linux/delay.h>
#endif


struct tps6025x_device_state {
	struct i2c_client		*i2c_dev;
	struct tps6025x_platform_data	*pdata;
	int				suspended;
};

static int tps6025x_i2c_read_reg(struct i2c_client* client, u8 addr, u8* out)
{
	int ret;
	struct i2c_msg msg[2];

	msg[0].addr = client->addr;
	msg[0].len = 1;
	msg[0].flags = 0;
	msg[0].buf = &addr;

	msg[1].addr = client->addr;
	msg[1].len = 1;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = out;

	ret = i2c_transfer(client->adapter, msg, 2);
	
	return ret;
}

static int tps6025x_i2c_write_reg(struct i2c_client* client, u8 addr, u8 val)
{
	u8 buf[2] = {addr, val};
	int ret;
	struct i2c_msg msg[1];

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = buf;
	
	ret = i2c_transfer(client->adapter, msg, 1);
	return ret;
}

#ifdef TPS6025X_DEBUG
static void tps6025x_dump_regs(struct i2c_client *client)
{
	u8 regs[4] = {0,0,0,0};

	tps6025x_i2c_read_reg(client, ENABLE_CONTROL, &regs[0]);
	tps6025x_i2c_read_reg(client, SUB_DISP_CURRENT_CONTROL, &regs[1]);
	tps6025x_i2c_read_reg(client, MAIN_DISP_CURRENT_CONTROL, &regs[2]);
	tps6025x_i2c_read_reg(client, AUX_BRIGHTNESS_OP_CONTROL, &regs[3]);

	printk("Enable Control Reg               : 0x%02X\n", regs[0]);
	printk("Sub Display Current Control Reg  : 0x%02X\n", regs[1]);
	printk("Main Display Current Control Reg : 0x%02X\n", regs[2]);
	printk("Aux Output Brightness Op Mode Reg: 0x%02X\n", regs[3]);

	return;
}

static void tps6025x_breathing_test(struct i2c_client *client)
{
	int i, brightness, inc;

	/* Do some breathing for fun. */
	brightness = 0;
	inc = 1;

	for (i = 0; i < 64*4; i++) {
		tps6025x_i2c_write_reg(client, SUB_DISP_CURRENT_CONTROL,
				       brightness);
		tps6025x_i2c_write_reg(client, MAIN_DISP_CURRENT_CONTROL,
				       brightness);
		tps6025x_i2c_write_reg(client, AUX_BRIGHTNESS_OP_CONTROL,
				       (brightness << 2));
		brightness += inc;

		if (!brightness || (brightness == 63))
			inc *= -1;
			
		mdelay ( 20 );
        }
}
#endif

static void tps6025x_configure_leds(struct tps6025x_device_state *devstate)
{
	u8 enable_control = 0;
	struct tps6025x_led_config *leds = devstate->pdata->leds; 
	struct i2c_client *client = devstate->i2c_dev;

	/* Turn off all current flow by writing zero to DM5. */
	tps6025x_i2c_write_reg(client, ENABLE_CONTROL,
			       enable_control);

	/* Enable Open Lamp Detection to save power. */
	enable_control |= ENOLD;

	/* Enable Main */
	if (leds[DM1].enable || leds[DM2].enable ||
	    leds[DM3].enable || leds[DM4].enable) {
		enable_control |= ENMAIN;
	}

	/* Enable Sub */
	if (leds[DS1].enable) {
		enable_control |= ENSUB1;
	}
	if (leds[DS2].enable) {
		enable_control |= ENSUB2;
	}

	/* Enable AUX. */
	if (leds[DM5].enable) {
		enable_control |= ENAUX;
	}

	/* Enable the IC in correct DM5 mode. */
	if  (devstate->pdata->dm5_aux_display_mode) {
		enable_control |= DM5_AUX_MODE;
	} else {
		enable_control |= DM5_MAIN_MODE;
	}

	/* Finally, write the register. */
	tps6025x_i2c_write_reg(client, ENABLE_CONTROL, enable_control);

	return;
}

static void tps6025x_set_brightness(struct led_classdev *led_cdev,
				    enum led_brightness value)
{
	struct tps6025x_led_config *led_config;
	struct i2c_client *client;
	struct tps6025x_device_state *devstate;
	u8 regdata = 0;

	/* Get the instance of LED configuration block. */
	led_config = container_of(led_cdev, struct tps6025x_led_config, cdev);

	/* Sanity check. We can't be here without these set up correctly. */
	if ((!led_config) || (!led_config->enable) || (!led_config->client)) {
		printk(KERN_ERR "%s: Invalid LED. Can't set led brightness.\n",
		       TPS6025X_I2C_DRIVER);
		return;
	}

	/* Get the handle to the I2C client. */
	client = led_config->client;

	/* Get the handle to the device state. */
	devstate = i2c_get_clientdata(client);

	/* Write to the TPS6025X register and change brightness. */
	switch(led_config->id) {
		case DM1:
		case DM2:
		case DM3:
		case DM4:
			/* Change DMx LED brightness. */
			tps6025x_i2c_read_reg(client,
					      MAIN_DISP_CURRENT_CONTROL,
					      &regdata);
			regdata &= ~LMAIN;

			/* Translate the range 0 through 256
			 * to 0 through 63.
			 */
			regdata |= value/4;
			tps6025x_i2c_write_reg(client,
					       MAIN_DISP_CURRENT_CONTROL,
					       regdata);
			break;

		case DS1:
		case DS2:
			/* Change DSx LED brightness. */
			tps6025x_i2c_read_reg(client,
					      SUB_DISP_CURRENT_CONTROL,
					      &regdata);
			regdata &= ~LSUB;

			/* Translate the range 0 through 256
			 * to 0 through 63.
			 */
			regdata |= value/4;
			tps6025x_i2c_write_reg(client,
					       SUB_DISP_CURRENT_CONTROL,
					       regdata);
			break;

		case DM5:
			tps6025x_i2c_read_reg(client, 
					      AUX_BRIGHTNESS_OP_CONTROL,
					      &regdata);
			regdata &= ~LAUX;

			/* If the DM5 is in AUX mode, we support
			 * only 4 steps of brightness.
			 */
			if (devstate->pdata->dm5_aux_display_mode) {
				/* Translate the range 0 through 256
				 * to 0 through 3.
				 */
				regdata |= ((value/64) << 2);
			} else {
				/* Translate the range 0 through 256
				 * to 0 through 63.
				 */
				regdata |= ((value/4) << 2);
			}
			tps6025x_i2c_write_reg(client,
					       AUX_BRIGHTNESS_OP_CONTROL,
					       regdata);
			break;
		default:
			printk(KERN_ERR "%s: Unsupported LED ID: %d\n",
			       TPS6025X_I2C_DRIVER, led_config->id);
	}
	return;
}

static int tps6025x_i2c_probe(struct i2c_client *client)
{
	struct tps6025x_device_state *devstate = NULL;
	struct tps6025x_platform_data *pdata = client->dev.platform_data;
	struct tps6025x_led_config *leds = NULL; 
	int i, ret = 0;

	/* Check the platform data. */
	if (pdata == NULL) {
		printk(KERN_ERR "%s: missing platform data.\n",
		       TPS6025X_I2C_DRIVER);
		return (-ENODEV);
	}

	/* Create the device state */
	devstate = kzalloc(sizeof(struct tps6025x_device_state), GFP_KERNEL);
	if (!devstate) {
		return (-ENOMEM);
	}

	/* Attach i2c_dev */
	devstate->i2c_dev = client;

	/* Attach platform data */
	devstate->pdata = pdata;

	/* Attach driver data. */
	i2c_set_clientdata(client, devstate);

	leds = pdata->leds;

	/* Configure the controller. */
	tps6025x_configure_leds(devstate);

#ifdef TPS6025X_DEBUG
	tps6025x_dump_regs(client);
	tps6025x_breathing_test(client);
#endif

	/* Register to led class. */
	for (i = 0; ret >= 0 && i < TPS_MAX_NUM_LEDS; i++) {
		if (leds[i].enable) {
			if (!leds[i].cdev.brightness_set) {
				leds[i].cdev.brightness_set =\
					tps6025x_set_brightness;
			}

			/* Save the i2c client information for each led. */
			leds[i].client = client;

			/* Register this LED instance to the LED class. */
			ret = led_classdev_register(&client->dev,
						    &leds[i].cdev);
		}
	}

	if (ret < 0 && i > 1) {
		for (i = i - 2; i >= 0; i--) {
			if (leds[i].enable) {
				led_classdev_unregister(&leds[i].cdev);
			}
		}
	}
	return ret;
}

static int tps6025x_i2c_remove(struct i2c_client *client)
{
	struct tps6025x_device_state *devstate = i2c_get_clientdata(client);
	struct tps6025x_platform_data *pdata = client->dev.platform_data;
	struct tps6025x_led_config *leds = NULL;
	int i;

	if (pdata) {
		leds = pdata->leds;
		for (i = 0; i < TPS_MAX_NUM_LEDS; i++) {
			if (leds[i].enable) {
				led_classdev_unregister(&leds[i].cdev);
			}
		}
	}

	i2c_set_clientdata(client, NULL);

	kfree(devstate);
	return 0;
}

#ifdef CONFIG_PM
static int tps6025x_i2c_suspend(struct i2c_client *client, pm_message_t state)
{
	struct tps6025x_device_state *devstate = i2c_get_clientdata(client);
	struct tps6025x_platform_data *pdata = client->dev.platform_data;
	struct tps6025x_led_config *leds = NULL;
	int i;

	if (devstate->suspended) {
		return 0;
	}
	devstate->suspended = 1;

	if (pdata) {
		leds = pdata->leds;
		for (i = 0; i < TPS_MAX_NUM_LEDS; i++) {
			if (leds[i].enable) {
				led_classdev_suspend(&leds[i].cdev);
			}
		}
	}

	/* TODO: Do we need to do something with TPS6025X? */
	return 0;
}

static int tps6025x_i2c_resume(struct i2c_client *client)
{
	struct tps6025x_device_state *devstate = i2c_get_clientdata(client);
	struct tps6025x_platform_data *pdata = client->dev.platform_data;
	struct tps6025x_led_config *leds = NULL;
	int i;

	if (!devstate->suspended) {
		return 0;
	}
	devstate->suspended = 0;

	if (pdata) {
		leds = pdata->leds;
		for (i = 0; i < TPS_MAX_NUM_LEDS; i++) {
			if (leds[i].enable) {
				led_classdev_resume(&leds[i].cdev);
			}
		}
	}

	/* TODO: Do we need to do something with TPS6025X? */
	return 0;
}
#else
#define tps6025x_i2c_suspend	NULL
#define tps6025x_i2c_resume	NULL
#endif

static struct i2c_driver tps6025x_i2c_driver = {
	.probe		= tps6025x_i2c_probe,
	.remove		= tps6025x_i2c_remove,
	.suspend	= tps6025x_i2c_suspend,
	.resume		= tps6025x_i2c_resume,
	.driver		= {
		.name		= TPS6025X_I2C_DRIVER,
		.owner		= THIS_MODULE,
	},
};

static int __init tps6025x_i2c_init(void)
{
	return i2c_add_driver(&tps6025x_i2c_driver);
}

static void __exit tps6025x_i2c_exit(void)
{
 	i2c_del_driver(&tps6025x_i2c_driver);
}

module_init(tps6025x_i2c_init);
module_exit(tps6025x_i2c_exit);

MODULE_AUTHOR("Ryan Lim<ryan.lim@palm.com>");
MODULE_DESCRIPTION("TI TPS6025X LED Controller Driver");
MODULE_LICENSE("GPL");
