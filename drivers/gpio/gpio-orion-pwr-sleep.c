#define DEBUG
#include <linux/of.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>

#define PAD_CONF_MCBSP3_FSX_ADDRESS       (0x48002172)
#define PAD_CONF_SET_UART2_RX             (0x119)
#define PAD_CONF_SET_GPIO_143_INPUT       (0x411C)

#define PRCM_CM_ICLKEN_PER_ADDRESS        (0x48005010)
#define GPIO_5_GPIO_OE_ADDRESS            (0x49056034)
#define GPIO_5_GPIO_FALLINGDETECT_ADDRESS (0x4905604C)
#define GPIO_5_GPIO_RISINGDETECT_ADDRESS  (0x49056048)
#define GPIO_5_GPIO_WAKEUPENABLE_ADDRESS  (0x49056020)
#define GPIO_5_GPIO_SYSCONFIG_ADDRESS     (0x49056010)

#define GPIO_5_GPIO_IRQENABLE1            (0x4905601C)
#define GPIO_5_GPIO_IRQENABLE2            (0x4905602C)

struct gu_pinmux_data {
  void __iomem *pad_conf;
  int  count;
};


static irqreturn_t gpio_hndl(int a, void *b)
{
  printk(KERN_DEBUG "IRQ GPIO 5\n");

  return IRQ_HANDLED;
}


#define SET_REG_BIT_OR  0
#define SET_REG_BIT_AND 1
#define SET_REG_SET_ALL 2
static int set_register(struct device	*dev, u32 adr, u32 reg, char *descr, int type)
{
  void __iomem *vm_register = ioremap(adr, 4);
  u32 cur_reg = 0;

  if(vm_register == NULL){
    if( descr != NULL )
      dev_err(dev, "cann't remap %s 0x%08X\n", descr, adr);
    else
      dev_err(dev, "cann't remap 0x%08X\n", adr);
    return (-ENXIO);
  } else {
    cur_reg = readl(vm_register);
    switch(type)
    {
      case SET_REG_BIT_OR:
        cur_reg |= reg;
        break;
      case SET_REG_BIT_AND:
        cur_reg &= ~reg;
        break;
      default: cur_reg = reg;
    }
    writel(cur_reg, vm_register);
    if( descr != NULL )
      dev_dbg(dev, "%s:   %08X\n", descr, readl(vm_register));
    iounmap(vm_register);
  }

  return 0;
}

static int gpio_uart_pinmux_probe(struct platform_device *pdev)
{
  struct device	*device = &pdev->dev;
  static struct gu_pinmux_data *data;
  void __iomem *vm_register;
  int err, irq;
  u32 reg = 0;

  dev_dbg(device, "probe starts\n");

  /*if( (err = set_register(device, PRCM_CM_ICLKEN_PER_ADDRESS,  // Unlock GPIO 5
                          (1 << 16), "CM_ICLKEN", SET_REG_BIT_OR) ) != 0)
  return err;

  if( (err = set_register(device, GPIO_5_GPIO_OE_ADDRESS,
                          (1 << 15), "GPIO_OE", SET_REG_BIT_OR) ) != 0)
  return err;

  if( (err = set_register(device, GPIO_5_GPIO_FALLINGDETECT_ADDRESS,
                          (1 << 15), "GPIO_FALLINGDETECT", SET_REG_BIT_OR) ) != 0)
  return err;

  if( (err = set_register(device, GPIO_5_GPIO_WAKEUPENABLE_ADDRESS,
                          (1 << 15), "GPIO_WAKEUPENABLE", SET_REG_BIT_OR) ) != 0)
  return err;

  if( (err = set_register(device, GPIO_5_GPIO_SYSCONFIG_ADDRESS,
                          (1 << 2), "GPIO_SYSCONFIG", SET_REG_BIT_OR) ) != 0)
  return err;*/

  data = kzalloc(sizeof(*data), GFP_KERNEL);
  if(data == NULL){
    dev_err(device, "no memory\n");
    return (-ENOMEM);
  }

  data->pad_conf = ioremap(PAD_CONF_MCBSP3_FSX_ADDRESS, 2);
  if(data->pad_conf == NULL){
    dev_err(device, "cann't remap PAD_CONF_MCBSP3_FSX (0x%08X)\n",
                     PAD_CONF_MCBSP3_FSX_ADDRESS);
    kfree(data);
    return (-ENXIO);
  }

  irq = platform_get_irq(pdev, 0);
  if( irq <= 0 ) {
    dev_err(device, "No free IRQs\n");
    kfree(data);
    return (-ENXIO);
  }

  printk(KERN_DEBUG "PINMUX IRQ id: %d\n", irq);

  err = devm_request_threaded_irq(device, irq,
					  NULL, gpio_hndl,
					  IRQF_SHARED | IRQF_ONESHOT,
					  "gu-pinmux-hello", data);

            if (err) {
          		dev_err(device, "Unable to claim irq %d; error %d\n",
          			irq, err);
                kfree(data);
          		return err;
          	}

  platform_set_drvdata(pdev, data);


  dev_dbg(device, "probe successfully ends\n");
  return 0;
}

static int gpio_uart_pinmux_remove(struct platform_device *pdev)
{
  struct gu_pinmux_data *data = platform_get_drvdata(pdev);

  iounmap(data->pad_conf);
  kfree(data);

  return 0;
}

static int gpio_uart_pinmux_suspend(struct platform_device *pdev, pm_message_t state)
{
  struct device	*device = &pdev->dev;
  struct gu_pinmux_data *data = platform_get_drvdata(pdev);

  dev_dbg(device, "suspend %d\n", state.event);
  writew(PAD_CONF_SET_GPIO_143_INPUT, data->pad_conf);
  /*if( !(data->count % 2) ) {
    dev_dbg(device, "reconfig uart_rx->gpio_147\n");
    writew(PAD_CONF_SET_GPIO_147_INPUT, data->pad_conf);
  }*/

  return 0;
}

static int gpio_uart_pinmux_resume(struct platform_device *pdev)
{
  struct device	*device = &pdev->dev;
  struct gu_pinmux_data *data = platform_get_drvdata(pdev);

  dev_dbg(device, "resume\n");

  writew(PAD_CONF_SET_UART2_RX, data->pad_conf);
  /*if( (data->count % 2) ) {
    dev_dbg(device, "reconfig gpio_147->uart_rx\n");
    writew(PAD_CONF_SET_UART2_RX, data->pad_conf);
  }

  data->count++;*/

  return 0;
}

static const struct of_device_id gpio_uart_pinmux_of_match[] = {
	{ .compatible = "orion-gu-pinmux" },
	{ },
};

static struct platform_driver gpio_uart_pinmux_driver = {
	.probe  = gpio_uart_pinmux_probe,
  .remove = gpio_uart_pinmux_remove,
  .suspend = gpio_uart_pinmux_suspend,
  .resume  = gpio_uart_pinmux_resume,
	.driver = {
		.name = "gu-pinmux",
		.of_match_table = of_match_ptr(gpio_uart_pinmux_of_match),
	},
};

module_platform_driver(gpio_uart_pinmux_driver);

MODULE_DESCRIPTION("Driver for reconfig pin UART2 in suspend/resume events");
MODULE_AUTHOR("Vladimir Hrechko");
MODULE_LICENSE("GPL");
