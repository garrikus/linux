#include <sound/orion2-audio.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <sound/soc.h>
#include <sound/pcm.h>
#include <linux/gpio.h>


#define ORION2_SPK_AMP_GPIO 205

//! \brief Main device ORION2 Audio structure
//static struct platform_device *orion2_audio_dev;

static int orion2_codec_amp_start(struct snd_pcm_substream *substream)
{
  int ret = gpio_direction_output(ORION2_SPK_AMP_GPIO, 1);
  printk(KERN_DEBUG "Amp start gpio...\n");
  if( ret != 0)
  {
    printk(KERN_DEBUG "Audio Set gpio err: %d\n", ret);
  }
  printk(KERN_DEBUG "Amp start gpio ok\n");
  return 0;
}

static void orion2_codec_amp_shutdown(struct snd_pcm_substream *substream)
{
  printk(KERN_DEBUG "Amp shutdown gpio...\n");
  gpio_direction_output(ORION2_SPK_AMP_GPIO, 0);
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
  DAILINK_COMP_ARRAY(COMP_CPU("omap-mcbsp-dai.1")),
  DAILINK_COMP_ARRAY(COMP_CODEC("twl4030-codec", "twl4030-hifi")),
  DAILINK_COMP_ARRAY(COMP_PLATFORM("snd_dmaengine_pcm")));

static struct snd_soc_dai_link orion2_audio_dai_links[] = {
  {
    .name = "ORION2 sound",
    .stream_name = "ORION2 sound",
    .ops = &orion2_audio_ops,
    SND_SOC_DAILINK_REG(sound),
  },
};
//! \brief Audio machine driver
static struct snd_soc_card orion2_card = {
  .owner = THIS_MODULE,
  .dai_link = orion2_audio_dai_links,
  .num_links = ARRAY_SIZE(orion2_audio_dai_links),
};

/*! \brief         Probe function for ORION2 Audio driver
 *  \param[in,out] *pdev - pointer to device structure
 *  \return        0 - if success, error code - if fault
 *****************************************************************************/
static int orion2_audio_probe(struct platform_device *pdev)
{
#ifdef CONFIG_OF
  struct device *device = &pdev->dev;
  struct device_node *node = pdev->dev.of_node;

  printk(KERN_DEBUG "Hello audio");
  dev_dbg(device, "probe has been started\n");
  /* \TODO Should init digmic through i2c */

  if( node )
  {
    struct device_node  *dai_node = of_parse_phandle( node, "ti,mcbsp", 0);

    if( dai_node )
    {
      struct snd_soc_card *card = &orion2_card;
      card->dev = &pdev->dev;

      dev_dbg(device, "dai node name %s\n", dai_node->name);

      if(snd_soc_of_parse_card_name( card, "ti,model" ) == 0)
      {
        dev_dbg(device, "card get name %s\n", card->name);

        orion2_audio_dai_links[0].cpus->dai_name  = NULL;
        orion2_audio_dai_links[0].cpus->of_node = dai_node;

        orion2_audio_dai_links[0].platforms->name  = NULL;
        orion2_audio_dai_links[0].platforms->of_node = dai_node;

        dev_dbg(device, "try to register card\n");
        if( devm_snd_soc_register_card( &pdev->dev, card ) != 0 )
        {
          dev_err(device, "card registration has FAILED!\n");
          return (-EINVAL);
        }
        else
        {
          dev_dbg(device, "card registration has SUCCESS!\n");
        }
      }
      else
      {
        dev_err(device, "Card name isn't present in the device tree\n");
        return (-ENODEV);
      }
    }
    else
    {
      dev_err(device, "McBSP isn't not present in the device tree\n");
      return (-EINVAL);
    }
  }
  else
  {
    dev_err(device, "node not found\n");
    return (-ENODEV);
  }

  dev_dbg(device, "probe has been successfully finished\n");
  return 0;

#else
  dev_err(&pdev->dev, "Driver doesn't support working without device tree\n");
  return (-ENODEV);
#endif
}

#ifdef CONFIG_OF
//! \brief Main device tree structure for ORION2 driver
static const struct of_device_id orion2_audio_of_match[] = {
  {.compatible = ORION_DRIVER_NAME, },
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
};

module_platform_driver(orion2_audio_driver);

MODULE_AUTHOR("Vladimir Hrechko");
MODULE_DESCRIPTION("Audio driver for ORION2");
MODULE_LICENSE("GPL");
