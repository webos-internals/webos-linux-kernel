#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include <asm/arch/mux.h>

#include "twl4030-audio.h"
#include "board-audio.h"

static struct platform_device joplin_audio_devices[] = {
	{
		/* twl4030-i2s (mcbsp2) */
		.name = OMAP_MCBSP_AUDIO_NAME,
		.id = 7,    /* FIXME why this id? */
		.dev = {
			.platform_data = &twl4030_alsa_dev_private,
		},
	},
	{
		/* bluetooth sco (mcbsp4) */
		.name = OMAP_MCBSP_AUDIO_NAME,
		.id = 8,
		.dev = {
			.platform_data = &btsco_alsa_dev_private,
		},
	},
	{
		.name = NULL,
	},
};

static void __init board_audio_init(void)
{
	struct platform_device *pdev;
	struct omap_alsa_dev_private *priv;

	for (pdev = joplin_audio_devices; pdev->name; pdev++) {
		if (platform_device_register(pdev) < 0) {
			priv = (struct omap_alsa_dev_private *)pdev->dev.platform_data;
			printk(KERN_ERR "Unable to register ALSA device %s\n", priv->name);
			continue;
		}
	}
}

#if defined(CONFIG_MACH_SIRLOIN)
int sirloin_btsco_probe(void)
{
	return 0;
}

int sirloin_btsco_mixer_init(struct snd_card * card)
{
	return 0;
}

int sirloin_btsco_configure(unsigned int sample_rate, unsigned int data_width,
		unsigned int channels)
{
	return 0;
}

int sirloin_btsco_enable(bool enable)
{
	if (enable) {
		twl4030_audio_codec_phonecall_enable(1);
		omap_cfg_reg("AF6_3430_MCBSP3_DX");
		omap_cfg_reg("AE6_3430_MCBSP3_DR");
		omap_cfg_reg("AF5_3430_MCBSP3_CLKX");
		omap_cfg_reg("AE5_3430_MCBSP3_FSX");
	} else {
		omap_cfg_reg("AF6_3430_GPIO_140");
		omap_cfg_reg("AE6_3430_GPIO_141");
		omap_cfg_reg("AF5_3430_GPIO_142");
		omap_cfg_reg("AE5_3430_GPIO_143");
		twl4030_audio_codec_phonecall_enable(0);
	}

	return 0;
}

int sirloin_btsco_suspend(void)
{
	return 0;
}

int sirloin_btsco_resume(void)
{
	return 0;
}

static struct omap_alsa_codec_ops sirloin_btsco_codec_ops = {
	.probe = sirloin_btsco_probe,
	.mixer_init = sirloin_btsco_mixer_init,
	.configure = sirloin_btsco_configure,
	.enable = sirloin_btsco_enable,
	.suspend = sirloin_btsco_suspend,
	.resume = sirloin_btsco_resume,
};

struct omap_alsa_codec_ops * sirloin_btsco_get_codec_ops(void)
{
	return &sirloin_btsco_codec_ops;
}
#endif

static int __init board_joplin_audio_init(void)
{
	board_audio_init();
	return 0;
}

static void __exit board_joplin_audio_exit(void)
{
	/* unimplemented */
}

module_init(board_joplin_audio_init);
module_exit(board_joplin_audio_exit);
