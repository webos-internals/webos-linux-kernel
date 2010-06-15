/* drivers/video/msm_fb/mddi_client_lg.c
 *
 * Support for LG NT353xx mddi client devices
 *
 * Copyright (C) 2008 Palm, Inc.
 * Author: Dmitry Fink <dmitry.fink@palm.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <asm/gpio.h>
#include <asm/arch/msm_fb.h>
#include <asm/delay.h>
#include <linux/delay.h>
#include <asm/arch/vreg.h>

#define DISPON	(0x290)
#define SLEPCTRL (0x110)
#define DSB		(0xC20)
#define LCDRESET	(131)

static DECLARE_WAIT_QUEUE_HEAD(lg_vsync_wait);
static volatile int lg_got_int;
static struct msmfb_callback *lg_callback;
static struct msmfb_callback *lg_displayon_callback;
static struct msmfb_callback *lg_displayoff_callback;

static  volatile int vsync_gpio = -1;
static  volatile bool vsync_enabled = false;

static void lg_cut1_init(struct mddi_panel_info *panel)
{
	// Memory address set
	mddi_remote_write(panel->mddi, 0x0000,0x02A0); // XSA-LSB
	mddi_remote_write(panel->mddi, 0x0000,0x02A1); // XSA-MSB
	mddi_remote_write(panel->mddi, 0x003F,0x02A2); // XEA-LSB
	mddi_remote_write(panel->mddi, 0x0001,0x02A3); // XEA-MSB
	mddi_remote_write(panel->mddi, 0x0000, 0x02B0); // YSA-LSB
	mddi_remote_write(panel->mddi, 0x0000,0x02B1); // YSA-MSB
	mddi_remote_write(panel->mddi, 0x008F,0x02B2); // YEA-LSB
	mddi_remote_write(panel->mddi, 0x0001,0x02B3); // YEA-MSB
	mddi_remote_write(panel->mddi, 0x003D,0x02D0); // XAD-LSB
	mddi_remote_write(panel->mddi, 0x0001, 0x02D1); // XAD-MSB
	mddi_remote_write(panel->mddi, 0x0000, 0x02D2); // YAD-LSB
	mddi_remote_write(panel->mddi, 0x0000,0x02D3); // YAD-MSB
	
	mddi_remote_write(panel->mddi, 0x0000,0x0360); // Entry mode 1
	mddi_remote_write(panel->mddi, 0x00CD,0x0361); // Entry mode 2
	mddi_remote_write(panel->mddi, 0x0000, 0x0362); // I80 Interface 

	
	// Driver output control
	mddi_remote_write(panel->mddi, 0x00C7,0x0371); // memory data access
	mddi_remote_write(panel->mddi, 0x0005,0x0372); // memory data access
		

	// Driver output control	
	mddi_remote_write(panel->mddi, 0x0000,0x0B00); // Pixel sequence
	mddi_remote_write(panel->mddi, 0x0000,0x0B10); // Inversion mode


	// Panel Interface Control
	mddi_remote_write(panel->mddi, 0x0808,0x0B20); // BP // 8 line
	mddi_remote_write(panel->mddi, 0x0808,0x0B21); // FP // 8 line	
	mddi_remote_write(panel->mddi, 0x0059,0x0B30); // RTI // 160 clock
	mddi_remote_write(panel->mddi, 0x0000,0x0B40); // FS // 0 clock
	mddi_remote_write(panel->mddi, 0x0000,0x0B50); // STI // 0 clock
	mddi_remote_write(panel->mddi, 0x0000,0x0B52); // non-overlap 
	mddi_remote_write(panel->mddi, 0x0014,0x0B70); // MUXW // 18 clock
	mddi_remote_write(panel->mddi, 0x0023,0x0B80); // MUX1G, MUX2G // 2,2 clock
	mddi_remote_write(panel->mddi, 0x0009,0x0B90); // EQT // 8 clock		
	mddi_remote_write(panel->mddi, 0x0057,0x0BA0); // MUX3G, PCSG // 4,6 clock
		
	mddi_remote_write(panel->mddi, 0xFFFF,0x0BB0); // SOE control
	mddi_remote_write(panel->mddi, 0xFFFF,0x0BB1); // SOE control
	mddi_remote_write(panel->mddi, 0x0000,0x0BC0); // SOE control
	mddi_remote_write(panel->mddi, 0x0000,0x0BC1); // SOE control


	// Source output Control
	mddi_remote_write(panel->mddi, 0x0006,0x0BF0); //  Amp bias // high

	// Power control
	mddi_remote_write(panel->mddi, 0x0000,0x0C30); // DCDC clock freq
	mddi_remote_write(panel->mddi, 0x0010,0x0C31); // DCDC clock freq // AVDDP=2VCI, AVDDN=-2.5VCI
	mddi_remote_write(panel->mddi, 0x0050,0x0C40); //  GVDDP // 5.0V
	mddi_remote_write(panel->mddi, 0x0050,0x0C41); //  GVDDN // 5.0V
	mddi_remote_write(panel->mddi, 0x0024,0x0C50); //  VCOM // 0.4V
	mddi_remote_write(panel->mddi, 0x0001,0x0C51); //  VCOMDC // Register
	mddi_remote_write(panel->mddi, 0x0017,0x0C60); //  VR // VR=5.0V
	mddi_remote_write(panel->mddi, 0x000D,0x0C61); //  VGH, VGL // VGH=2VR , VGL=VCI2-2xVR
	mddi_remote_write(panel->mddi, 0x0017,0x0C70); //  VCI2 // VCI2=-5.0V
	mddi_remote_write(panel->mddi, 0x0010,0x0C80); //  VGLOFF // VGLOFF=-12V

			
	// Gamma
	mddi_remote_write(panel->mddi, 0x0003,0x0E00); // VP0
	mddi_remote_write(panel->mddi, 0x000B,0x0E01); // VP1			
	mddi_remote_write(panel->mddi, 0x0011,0x0E02); // VP2
	mddi_remote_write(panel->mddi, 0x0013,0x0E03); // VP4			
	mddi_remote_write(panel->mddi, 0x002D,0x0E04); // VP8
	mddi_remote_write(panel->mddi, 0x0027,0x0E05); // VP13			
	mddi_remote_write(panel->mddi, 0x0025,0x0E06); // VP20
	mddi_remote_write(panel->mddi, 0x0015,0x0E07); // VP27			
	mddi_remote_write(panel->mddi, 0x000F,0x0E08); // VP36
	mddi_remote_write(panel->mddi, 0x003E,0x0E09); // VP43			
	mddi_remote_write(panel->mddi, 0x0022,0x0E0A); // VP50
	mddi_remote_write(panel->mddi, 0x003D,0x0E0B); // VP55			
	mddi_remote_write(panel->mddi, 0x0054,0x0E0C); // VP59
	mddi_remote_write(panel->mddi, 0x0019,0x0E0D); // VP61			
	mddi_remote_write(panel->mddi, 0x001E,0x0E0E); // VP62
	mddi_remote_write(panel->mddi, 0x006F,0x0E0F); // VP63

	// Enter Engineer mode
	mddi_remote_write(panel->mddi,0x0055,0x0F20); //
	mddi_remote_write(panel->mddi,0x00AA,0x0F21); //
	mddi_remote_write(panel->mddi,0x0066,0x0F22); //
	mddi_remote_write(panel->mddi,0x0004,0x0F60); //

	mddi_remote_write(panel->mddi, 0x0000,0x0110); // Sleep out_offset
	msleep(10);

	// Fix for flickering
	mddi_remote_write(panel->mddi,0x00,0xC31); // Set AVDDN pump to -2xVCI mode
	mddi_remote_write(panel->mddi,0x55,0xF20); // Enter engineer mode
	mddi_remote_write(panel->mddi,0xAA,0xF21);
	mddi_remote_write(panel->mddi,0x66,0xF22);
	mddi_remote_write(panel->mddi,0x5E,0xF51); // Disable AVDDN clamp function_enable


	//Latch up workaround
	mddi_remote_write(panel->mddi, 0x0F,0x0BB1);
	mddi_remote_write(panel->mddi, 0x70,0x0BC1);

	mddi_remote_write(panel->mddi, 0x0001,0x0290);
	mddi_remote_write(panel->mddi, 0x0001,0x0290);
	msleep(500);

	mddi_remote_write(panel->mddi,0x0000,0x0F60); // 
	mddi_remote_write(panel->mddi,0x0000,0x0F60); //

	return ; 
}

static void tpo_cut2_init(struct mddi_panel_info *panel)
{
	// Sleep out 
	mddi_remote_write(panel->mddi,0x00,0x110); 
	msleep( 10 );

	// Entry mode 1 
	mddi_remote_write(panel->mddi,0x10,0x360);  
	// Entry mode 2 
	mddi_remote_write(panel->mddi,0xC4,0x361);  
	// Driver output control 
	mddi_remote_write(panel->mddi,0x50,0xB00); 
	// Panel interface control 
	mddi_remote_write(panel->mddi,0x0C,0xBD1); 
	// Panel interface control 
	mddi_remote_write(panel->mddi,0x13,0xB70); 
	// Panel interface control 
	mddi_remote_write(panel->mddi,0x02,0xBC1); 
	// Vcom voltage control 
	mddi_remote_write(panel->mddi,0x20,0xC52); 
	// VGH/VGL voltage control 
	mddi_remote_write(panel->mddi,0x02,0xC61); 
	// VCI2 voltage control 
	mddi_remote_write(panel->mddi,0x17,0xC70); 
	// VGLOff voltage control 
	mddi_remote_write(panel->mddi,0x1F,0xC80); 

	// Fix for flickering
	mddi_remote_write(panel->mddi,0x00,0xC31); // Set AVDDN pump to -2xVCI mode
	mddi_remote_write(panel->mddi,0x55,0xF20); // Enter engineer mode
	mddi_remote_write(panel->mddi,0xAA,0xF21);
	mddi_remote_write(panel->mddi,0x66,0xF22);
	mddi_remote_write(panel->mddi,0x5E,0xF51); // Disable AVDDN clamp function

	return;
}

static void tpo_evt3_init(struct mddi_panel_info *panel)
{
	mddi_remote_write(panel->mddi,0x00,0x0110);	//Sleep out
	mddi_remote_write(panel->mddi,0x10,0x0360);	//Entry mode 1
	mddi_remote_write(panel->mddi,0xC4,0x0361); //Entry mode 2

	mddi_remote_write(panel->mddi,0x50,0x0B30); //Panel interface control
	mddi_remote_write(panel->mddi,0x04,0x0B90); //Panel interface control
	mddi_remote_write(panel->mddi,0x07,0x0BF0);//Panel interface control

	mddi_remote_write(panel->mddi,0x00,0x0C31);	//Power control
	mddi_remote_write(panel->mddi,0x0D,0x0C60); //VR voltage control
	mddi_remote_write(panel->mddi,0x36,0x0BA0);	//Panel interface control
	mddi_remote_write(panel->mddi,0x50,0x0B00); //Driver output control

	mddi_remote_write(panel->mddi,0x09, 0x0BD1); //Panel interface control
	mddi_remote_write(panel->mddi,0x12, 0x0B70); //Panel interface control
	mddi_remote_write(panel->mddi,0x33, 0x0B80); //Panel interface control
	mddi_remote_write(panel->mddi,0x02, 0x0BC1); //Panel interface control
	mddi_remote_write(panel->mddi,0x02, 0x0C61); //VGH/VGL voltage control
	mddi_remote_write(panel->mddi,0x17, 0x0C70); //VCI2 voltage control
	mddi_remote_write(panel->mddi,0x1F, 0x0C80); //VGLOff voltage control

	return;
}

static void tpo_dvt_init(struct mddi_panel_info *panel)
{
	mddi_remote_write(panel->mddi,0x00,0x0110);	//Sleep out

	//Spec mentions there should be 10ms delay between
	//sleep out and display on, but we have seen cases that's not enough
	msleep(30); 

	return;
}

static void lg_evt2_init(struct mddi_panel_info *panel)
{
	mddi_remote_write(panel->mddi, 0x0000,0x0110); // Sleep out
	msleep(10);
	
	mddi_remote_write(panel->mddi, 0x00CC,0x0361); //   

	// Fix for flickering
	mddi_remote_write(panel->mddi,0x00,0xC31); // Set AVDDN pump to -2xVCI mode
	mddi_remote_write(panel->mddi,0x55,0xF20); // Enter engineer mode
	mddi_remote_write(panel->mddi,0xAA,0xF21);
	mddi_remote_write(panel->mddi,0x66,0xF22);
	mddi_remote_write(panel->mddi,0x5E,0xF51); // Disable AVDDN clamp function_enable    

	//Latch up workaround
	mddi_remote_write(panel->mddi, 0x04, 0xF60);  // Turn off the VGHO
	mddi_remote_write(panel->mddi, 0x0F,0x0BB1);
	mddi_remote_write(panel->mddi, 0x70,0x0BC1);
  	mddi_remote_write(panel->mddi, 0x00, 0xF60); // Turn on the VGHO 
	

	return;
}

static void lg_evt3_init(struct mddi_panel_info *panel)
{
	
	mddi_remote_write(panel->mddi, 0x0000,0x0110); // Sleep out
	
	mddi_remote_write(panel->mddi, 0x00CC,0x0361); 

	return;
}

static void lg_dvt_init(struct mddi_panel_info *panel)
{
	mddi_remote_write(panel->mddi, 0x0000,0x0110); // Sleep out

	msleep(30);
	
	return;
}

//Display on and off
static void lg_display_enable(struct mddi_panel_info *panel)
{
	
	if (panel->panel_ops->request_vsync) {
		//Set FTEP for Vysnc to 300th line
		mddi_remote_write(panel->mddi, 0x2c, 0x0350);
		mddi_remote_write(panel->mddi, 0x01, 0x0351);
        }
	

	mddi_remote_write(panel->mddi, 1, DISPON);

	// notify fb driver we're turning on the display
	if(lg_displayon_callback)
	{
		lg_displayon_callback->func(lg_displayon_callback);
	}

}

static void lg_display_disable(struct mddi_panel_info *panel)
{
	// notify fb driver we're turning off the display
	if(lg_displayoff_callback)
	{
		lg_displayoff_callback->func(lg_displayoff_callback);
	}

	// Disable display
	mddi_remote_write(panel->mddi, 0, DISPON);
}

//In and out of Deep Sleep mode 
//Support has been implemented for: LG Cut1, LG EVT2 and TPO Cut2
static void lg_deepsleep(struct mddi_panel_info *panel, int on)
{
	unsigned int user_id,revision;
	struct vreg *vreg_gp5, *vreg_rftx;

	vreg_gp5 = vreg_get(0, "gp5");
	vreg_rftx = vreg_get(0, "rftx");

	if (on) {

		// Get out of sleep mode and re-initialize all parameters
		// based on LCD type, then turn the display back on.
		 
		// Reset 
		gpio_request (LCDRESET, "display reset");

		if(mddi_suspend_mode(panel->mddi) == SUSPEND_SHUTDOWN_MODE) 
		{
			printk(KERN_INFO"%s: Powering on lcd\n",__func__);

			vreg_set_level(vreg_gp5, 1800);
			vreg_set_level(vreg_rftx, 3000);

			msleep(10);
                	vreg_enable(vreg_gp5);
                	msleep(50);
                	vreg_enable(vreg_rftx);
                	msleep(20);

			gpio_direction_output (LCDRESET, 1);
			msleep(10);
			gpio_direction_output (LCDRESET, 0);
			msleep(1);
			gpio_direction_output (LCDRESET, 1);
			msleep(10);
		}
		else if(mddi_suspend_mode(panel->mddi) == SUSPEND_DEEPSLEEP_MODE) 
		{
			gpio_direction_output (LCDRESET, 0);
			msleep(10);
			gpio_direction_output (LCDRESET, 1);
			msleep(20);
			mddi_remote_write(panel->mddi, 0, DSB);
		}
		

		user_id = mddi_remote_read(panel->mddi,0xD10);

		switch(user_id)
		{
			case 0x00:
			{
				revision = mddi_remote_read(panel->mddi,0xD20);
				if(1 == revision)
				{
					//lg_cut2_init(panel);
				}
				else			
				{
					printk(KERN_INFO"Resuming LCD: LG cut1\n");
					lg_cut1_init(panel);
				}
			}
			break;

			case 0x01:
			{
				printk(KERN_INFO"Resuming LCD: LG evt2\n");
				lg_evt2_init(panel);				
			}
			break;

			case 0x02:
			{
				printk(KERN_INFO"Resuming LCD: LG evt3\n");
				lg_evt3_init(panel);				
			}
			break;

			case 0x03:
			{
				printk(KERN_INFO"Resuming LCD: LG dvt\n");
				lg_dvt_init(panel);				
			}
			break;

			case 0x10:
			{
				printk(KERN_INFO"Resuming LCD: TPO cut2\n");
				tpo_cut2_init(panel);				
			}
			break;

			case 0x11:
			{
				printk(KERN_INFO"Resuming LCD: TPO evt3\n");
				tpo_evt3_init(panel);				
			}
			break;

			case 0x12:
			{
				printk(KERN_INFO"Resuming LCD: TPO dvt\n");
				tpo_dvt_init(panel);				
			}
			break;

			default:
			{
				printk(KERN_INFO"esuming LCD: Failed to identify user id\n");
			}
			break;
		}

	}
	else {
		// Disable display and go into sleep mode
		mddi_remote_write(panel->mddi, 1, SLEPCTRL);
		msleep(100);
		if(mddi_suspend_mode(panel->mddi) == SUSPEND_SHUTDOWN_MODE)
		{
			vreg_disable(vreg_gp5);
			vreg_disable(vreg_rftx);
			printk(KERN_INFO"%s: Powering off lcd\n",__func__);
		}
		if(mddi_suspend_mode(panel->mddi) == SUSPEND_DEEPSLEEP_MODE)
		{
			mddi_remote_write(panel->mddi, 1, DSB);
		}
	}
}



static void lg_request_vsync(struct mddi_panel_info *pi,
				  struct msmfb_callback *callback)
{
	if(vsync_gpio > 0 && !vsync_enabled){
		enable_irq(gpio_to_irq(vsync_gpio));
		vsync_enabled = true;
	}
	
	lg_callback = callback;

	if (lg_got_int) {
		lg_got_int = 0;
		mddi_activate_link(pi->mddi); /* clears interrupt */
	}
}

static void lg_display_on(struct mddi_panel_info *pi,
					struct msmfb_callback *callback)
{
	lg_displayon_callback = callback;
}

static void lg_display_off(struct mddi_panel_info *pi,
					struct msmfb_callback *callback)
{
	lg_displayoff_callback = callback;
}

static void lg_wait_vsync(struct mddi_panel_info *pi)
{
	if (lg_got_int) {
		lg_got_int = 0;
		mddi_activate_link(pi->mddi); /* clears interrupt */
	}
	if (wait_event_timeout(lg_vsync_wait, lg_got_int, HZ/2) == 0)
		printk(KERN_ERR "timeout waiting for VSYNC\n");
	lg_got_int = 0;

	/* interrupt clears when screen dma starts */

	
}


static struct mddi_panel_ops lg_panel_ops = {
	.enable = lg_display_enable,
	.disable = lg_display_disable,
	.power = lg_deepsleep,
	.wait_vsync = NULL,
	.request_vsync = NULL,
	.display_off = lg_display_off,
	.display_on = lg_display_on,
};

irqreturn_t lg_vsync_interrupt(int irq, void *data)
{
	lg_got_int = 1;
	if (lg_callback) {
		if(vsync_gpio > 0 && vsync_enabled){
			disable_irq(gpio_to_irq(vsync_gpio));
			vsync_enabled = false;
		}

		lg_callback->func(lg_callback);
		lg_callback = 0;
	}
	wake_up(&lg_vsync_wait);
	return IRQ_HANDLED;
}


static int mddi_lg_setup_vsync(struct mddi_info *mddi, int init)
{
	int ret;
	int gpio = vsync_gpio;
	unsigned int irq;

	if (!init) {
		ret = 0;
		goto uninit;
	}
	ret = gpio_request(gpio, "vsync");
	if (ret)
		goto err_request_gpio_failed;

	ret = gpio_direction_input(gpio);
	if (ret)
		goto err_gpio_direction_input_failed;

	ret = irq = gpio_to_irq(gpio);
	if (ret < 0)
		goto err_get_irq_num_failed;

	ret = request_irq(irq, lg_vsync_interrupt, IRQF_TRIGGER_RISING,
			  "vsync", NULL);
	if (ret)
		goto err_request_irq_failed;
	printk(KERN_INFO "vsync on gpio %d now %d\n",
	       gpio, gpio_get_value(gpio));

	disable_irq(gpio_to_irq(gpio));

	return 0;

uninit:
	free_irq(gpio_to_irq(gpio), mddi);
err_request_irq_failed:
err_get_irq_num_failed:
err_gpio_direction_input_failed:
	gpio_free(gpio);
err_request_gpio_failed:
	return ret;
}

static int mddi_lg_probe(struct platform_device *pdev)
{
	int ret;
	struct mddi_info *mddi;
	mddi = pdev->dev.platform_data;

	/*
	    The use of vsync is dynamically determined by bootie 
	    Please see boot parameter: "lcd_enable_vsync"
	*/
	if (mddi_use_vsync(mddi))
	{
		printk(KERN_INFO "mddi_lg_probe: LCD will be setup to use vsync\n");

		lg_panel_ops.wait_vsync = lg_wait_vsync;
		lg_panel_ops.request_vsync = lg_request_vsync;

		vsync_gpio = mddi_vsync_gpio(mddi);

		ret = mddi_lg_setup_vsync(mddi, 1);
		if (ret) {
			dev_err(&pdev->dev, "mddi_lg_setup_vsync failed\n");
			return ret;
		}
	}

	mddi_add_panel(pdev->dev.platform_data, &lg_panel_ops);

	return 0;
}

static int mddi_lg_remove(struct platform_device *pdev)
{
	struct mddi_info *mddi = pdev->dev.platform_data;
	if (lg_panel_ops.request_vsync) {
	         mddi_lg_setup_vsync(mddi, 0);
	 }
	return 0;
}

static struct platform_driver mddi_client_dead_beef = {
	.probe = mddi_lg_probe,
	.remove = mddi_lg_remove,
	.driver = { .name = "mddi_c_dead_beef" },
};

static int __init mddi_client_lg_init(void)
{
	platform_driver_register(&mddi_client_dead_beef);
	return 0;
}

module_init(mddi_client_lg_init);

