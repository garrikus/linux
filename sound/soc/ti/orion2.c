#include <sound/orion2-audio.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <sound/soc.h>
#include <sound/pcm.h>
#include <linux/gpio.h>

static struct orion2_audio_dev *orion2_dev;

static int orion2_codec_amp_start(struct snd_pcm_substream *substream)
{
  if(orion2_dev->amp)
  {
    printk(KERN_DEBUG "Amp on\n");
    gpio_direction_output(orion2_dev->amp_gpio, 1);
  }
  return 0;
}

static void orion2_codec_amp_shutdown(struct snd_pcm_substream *substream)
{
  if(orion2_dev->amp)
  {
    printk(KERN_DEBUG "Amp off\n");
    gpio_direction_output(orion2_dev->amp_gpio, 0);
  }
}

static int orion2_audio_hw_params(struct snd_pcm_substream *substream,
                                  struct snd_pcm_hw_params *params)
{
  struct snd_soc_pcm_runtime *rtd = substream->private_data;
  struct snd_soc_dai *codec_dai = rtd->codec_dai;
  struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
  int ret;

  /* Set codec DAI configuration */
  ret = snd_soc_dai_set_fmt(codec_dai,
          SND_SOC_DAIFMT_I2S |
          SND_SOC_DAIFMT_NB_NF |
          SND_SOC_DAIFMT_CBM_CFM);
  if (ret < 0) {
    printk(KERN_ERR "Can't set codec DAI configuration\n");
    return ret;
  }

  /* Set cpu DAI configuration */
  ret = snd_soc_dai_set_fmt(cpu_dai,
          SND_SOC_DAIFMT_I2S |
          SND_SOC_DAIFMT_NB_NF |
          SND_SOC_DAIFMT_CBM_CFM);
  if (ret < 0) {
    printk(KERN_ERR "Can't set cpu DAI configuration\n");
    return ret;
  }

  /* Set the codec system clock for DAC and ADC */
  ret = snd_soc_dai_set_sysclk(codec_dai, 0, 26000000,
             SND_SOC_CLOCK_IN);
  if (ret < 0) {
    printk(KERN_ERR "Can't set codec system clock\n");
    return ret;
  }

  return 0;
}
//! \brief ORION2 audio ops
static const struct snd_soc_ops orion2_audio_ops = {
  .hw_params = orion2_audio_hw_params,
  .startup = orion2_codec_amp_start,
  .shutdown = orion2_codec_amp_shutdown,
};

//! \brief Digital audio interface glue - connnects codec <---> CPU
SND_SOC_DAILINK_DEFS(sound,
  DAILINK_COMP_ARRAY(COMP_CPU("omap-mcbsp-dai.1")),       // Used from dt
  DAILINK_COMP_ARRAY(COMP_CODEC("twl4030-codec", "twl4030-hifi")),
  DAILINK_COMP_ARRAY(COMP_PLATFORM("omap-pcm-audio")));   // Used from dt and = mcbsp

static struct snd_soc_dai_link orion2_dai[] = {
  {
    .name = "TPS65951",
    .stream_name = "TPS65951",
    .ops = &orion2_audio_ops,
    SND_SOC_DAILINK_REG(sound),
  },
};
//! \brief Audio machine driver
static struct snd_soc_card orion2_card = {
  .owner = THIS_MODULE,
  .dai_link = orion2_dai,
  .num_links = ARRAY_SIZE(orion2_dai),
};

/*! \brief         Probe function for ORION2 Audio driver
 *  \param[in,out] *pdev - pointer to device structure
 *  \return        0 - if success, error code - if fault
 *****************************************************************************/
static int orion2_audio_probe(struct platform_device *pdev)
{
#ifdef CONFIG_OF
  struct device       *device = &pdev->dev;
  struct device_node  *node = pdev->dev.of_node;
  struct snd_soc_card *card = &orion2_card;
  struct device_node  *dai_node;

  dev_dbg(device, "probe has been started\n");

   /* \TODO Should init digmic through i2c */

  if (!node)
  {
    dev_err(device, "No node in devicetree\n");
    return (-ENODEV);
  }

  dai_node = of_parse_phandle( node, "ti,mcbsp", 0);

  if(!dai_node)
  {
    dev_err(device, "McBSP isn't not present in the device tree\n");
    return (-EINVAL);
  }

  card->dev = device;
  if(snd_soc_of_parse_card_name( card, "ti,model" ))
  {
    dev_err(device, "Card name isn't present in the device tree\n");
    return (-ENODEV);
  }

  orion2_dai[0].cpus->dai_name  = NULL;
  orion2_dai[0].cpus->of_node = dai_node;

  orion2_dai[0].platforms->name  = NULL;
  orion2_dai[0].platforms->of_node = dai_node;

  if( devm_snd_soc_register_card( device, card ) != 0 )
  {
    dev_err(device, "Card registration has FAILED!\n");
    return (-EINVAL);
  }

  orion2_dev = kzalloc(sizeof(*orion2_dev), GFP_KERNEL );
  if( orion2_dev == NULL )
  {
    dev_err(device, "No memory for device\n");
    return -ENOMEM;
  }

  if(!of_property_read_u32(node, "amp,gpio", &orion2_dev->amp_gpio))
  {
    orion2_dev->amp = true;
    dev_dbg(device, "Get gpio from dt %d\n", orion2_dev->amp_gpio);
  }

  dev_dbg(device, "probe has been successfully finished\n");
  return 0;

#else
  dev_err(&pdev->dev, "Driver doesn't support working without device tree\n");
  return (-ENODEV);
#endif
}

static int orion2_audio_remove(struct platform_device *pdev)
{
	kfree(orion2_dev);
	dev_dbg(&pdev->dev, "unregistered audio\n");
	return 0;
}

#ifdef CONFIG_OF
//! \brief Main device tree structure for ORION2 driver
static const struct of_device_id orion2_audio_of_match[] = {
  {.compatible = "orion2-audio", },
  {},
};
MODULE_DEVICE_TABLE(of, orion2_audio_of_match);
#endif

//! \brief Main driver structure for ORION2 driver
static struct platform_driver orion2_audio_driver =
{
  .driver = {
    .name = ORION_DRIVER_NAME,
#ifdef CONFIG_OF
    .of_match_table = orion2_audio_of_match,
#endif
  },
  .probe = orion2_audio_probe,
  .remove	= orion2_audio_remove,
};

module_platform_driver(orion2_audio_driver);

MODULE_AUTHOR("Vladimir Hrechko");
MODULE_DESCRIPTION("Audio driver for ORION2");
MODULE_LICENSE("GPL");
