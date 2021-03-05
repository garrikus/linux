#define DEBUG
#include <linux/of.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>

#define PAD_CONF_MCBSP3_FSX_ADDRESS (0x48002172)
#define PAD_CONF_SET_UART2_RX       (0x119)
#define PAD_CONF_SET_GPIO_147_INPUT (0x11C)

#define PRCM_CM_ICLKEN_PER_ADDRESS        (0x48005010)
#define GPIO_5_GPIO_OE_ADDRESS            (0x49056034)
#define GPIO_5_GPIO_FALLINGDETECT_ADDRESS (0x4905604C)
#define GPIO_5_GPIO_RISINGDETECT_ADDRESS  (0x49056048)
#define GPIO_5_GPIO_WAKEUPENABLE_ADDRESS  (0x49056020)
#define GPIO_5_GPIO_SYSCONFIG_ADDRESS     (0x49056010)

#define GPIO_5_GPIO_IRQENABLE1    (0x4905601C)
#define GPIO_5_GPIO_IRQENABLE2    (0x4905602C)

struct gu_pinmux_data {
  void __iomem *pad_conf;
  int  count;
};


static irqreturn_t gpio_hndl(int a, void *b)
{
  printk(KERN_DEBUG "IRQ GPIO 5\n");
  return IRQ_HANDLED;
}

static int gpio_uart_pinmux_probe(struct platform_device *pdev)
{
  struct device	*device = &pdev->dev;
  static struct gu_pinmux_data *data;
  void __iomem *vm_register;
  int err, irq;
  u32 reg = 0;

  dev_dbg(device, "probe starts\n");

  data = kzalloc(sizeof(*data), GFP_KERNEL);
  if(data == NULL){
    dev_err(device, "no memory\n");
    return (-ENOMEM);
  }

  data->pad_conf = ioremap(PAD_CONF_MCBSP3_FSX_ADDRESS, 2);
  if(data->pad_conf == NULL){
    dev_err(device, "cann't remap PAD_CONF_MCBSP3_FSX (0x%08X)\n",
                     PAD_CONF_MCBSP3_FSX_ADDRESS);
    err = (-ENXIO);
    goto err_fail;
  }

  vm_register = ioremap(PRCM_CM_ICLKEN_PER_ADDRESS, 4);
  if(vm_register == NULL) {
    dev_err(device, "cann't remap PRCM_CM_ICLKEN_PER (0x%08X)\n",
                     PRCM_CM_ICLKEN_PER_ADDRESS);
    iounmap(data->pad_conf);
    err = (-ENXIO);
    goto err_fail;
  } else {
    reg = readl(vm_register);
    reg |= (1 << 16);      // set EN_GPIO5
    writel(reg, vm_register);
    dev_dbg(device, "CM_ICLKEN:   %08X\n", readl(vm_register));
    writew(PAD_CONF_SET_GPIO_147_INPUT, data->pad_conf);
    iounmap(vm_register);
  }

  vm_register = ioremap(GPIO_5_GPIO_OE_ADDRESS, 4);
  if(vm_register == NULL) {
    dev_err(device, "cann't remap GPIO_5_GPIO_OE (0x%08X)\n",
                     GPIO_5_GPIO_OE_ADDRESS);
    iounmap(data->pad_conf);
    err = (-ENXIO);
    goto err_fail;
  } else {
    reg = readl(vm_register);
    reg |= (1 << 19);          // set OUTPUTEN[19] = 0 - config GPIO 147 like input
    writel(reg, vm_register);
    dev_dbg(device, "GPIO_OE:   %08X\n", readl(vm_register));
    iounmap(vm_register);
  }

  vm_register = ioremap(GPIO_5_GPIO_FALLINGDETECT_ADDRESS, 4);
  if(vm_register == NULL) {
    dev_err(device, "cann't remap GPIO_5_GPIO_FALLINGDETECT (0x%08X)\n",
                     GPIO_5_GPIO_FALLINGDETECT_ADDRESS);
    iounmap(data->pad_conf);
    err = (-ENXIO);
    goto err_fail;
  } else {
    reg = readl(vm_register);
    reg |= (1 << 19);          // set OUTPUTEN[19] = 0 - config GPIO 147 like input
    writel(reg, vm_register);
    dev_dbg(device, "GPIO_FALLINGDETECT:   %08X\n", readl(vm_register));
    iounmap(vm_register);
  }

  vm_register = ioremap(GPIO_5_GPIO_RISINGDETECT_ADDRESS, 4);
  if(vm_register == NULL) {
    dev_err(device, "cann't remap GPIO_5_GPIO_RISINGDETECT (0x%08X)\n",
                     GPIO_5_GPIO_RISINGDETECT_ADDRESS);
    iounmap(data->pad_conf);
    err = (-ENXIO);
    goto err_fail;
  } else {
    reg = readl(vm_register);
    reg |= (1 << 19);          // set OUTPUTEN[19] = 0 - config GPIO 147 like input
    writel(reg, vm_register);
    dev_dbg(device, "GPIO_RISINGDETECT:   %08X\n", readl(vm_register));
    iounmap(vm_register);
  }

  vm_register = ioremap(GPIO_5_GPIO_WAKEUPENABLE_ADDRESS, 4);
  if(vm_register == NULL) {
    dev_err(device, "cann't remap GPIO_5_GPIO_WAKEUPENABLE (0x%08X)\n",
                     GPIO_5_GPIO_WAKEUPENABLE_ADDRESS);
    iounmap(data->pad_conf);
    err = (-ENXIO);
    goto err_fail;
  } else {
    reg = readl(vm_register);
    reg |= (1 << 19);          // set OUTPUTEN[19] = 0 - config GPIO 147 like input
    writel(reg, vm_register);
    dev_dbg(device, "GPIO_WAKEUPENABLE:   %08X\n", readl(vm_register));
    iounmap(vm_register);
  }

  vm_register = ioremap(GPIO_5_GPIO_SYSCONFIG_ADDRESS, 4);
  if(vm_register == NULL) {
    dev_err(device, "cann't remap GPIO_5_GPIO_SYSCONFIG (0x%08X)\n",
                     GPIO_5_GPIO_SYSCONFIG_ADDRESS);
    iounmap(data->pad_conf);
    err = (-ENXIO);
    goto err_fail;
  } else {
    reg = readl(vm_register);
    reg |= (1 << 2);          // set OUTPUTEN[19] = 0 - config GPIO 147 like input
    writel(reg, vm_register);
    dev_dbg(device, "GPIO_SYSCONFIG:   %08X\n", readl(vm_register));
    iounmap(vm_register);
  }

  vm_register = ioremap(GPIO_5_GPIO_IRQENABLE1, 4);
  if(vm_register == NULL) {
    dev_err(device, "cann't remap GPIO_5_GPIO_IRQENABLE1 (0x%08X)\n",
                     GPIO_5_GPIO_IRQENABLE1);
    iounmap(data->pad_conf);
    err = (-ENXIO);
    goto err_fail;
  } else {
    reg = readl(vm_register);
    reg |= (1 << 19);          // set OUTPUTEN[19] = 0 - config GPIO 147 like input
    writel(reg, vm_register);
    dev_dbg(device, "GPIO_5_GPIO_IRQENABLE1:   %08X\n", readl(vm_register));
    iounmap(vm_register);
  }

  vm_register = ioremap(GPIO_5_GPIO_IRQENABLE2, 4);
  if(vm_register == NULL) {
    dev_err(device, "cann't remap GPIO_5_GPIO_IRQENABLE1 (0x%08X)\n",
                     GPIO_5_GPIO_IRQENABLE2);
    iounmap(data->pad_conf);
    err = (-ENXIO);
    goto err_fail;
  } else {
    reg = readl(vm_register);
    reg |= (1 << 19);          // set OUTPUTEN[19] = 0 - config GPIO 147 like input
    writel(reg, vm_register);
    dev_dbg(device, "GPIO_5_GPIO_IRQENABLE2:   %08X\n", readl(vm_register));
    iounmap(vm_register);
  }

  platform_set_drvdata(pdev, data);

  free_irq(33, NULL);

  err = request_irq( 33,
			               gpio_hndl,
			               IRQF_SHARED,
                     "gpio_5_handler",
			               pdev );
  dev_dbg(device, "SET GPIO IRQ:   %d\n", err);

  dev_dbg(device, "probe successfully ends\n");
  return 0;

err_fail:
  dev_dbg(device, "FAIL!!!\n");
  kfree(data);
  return err;
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
  writew(PAD_CONF_SET_GPIO_147_INPUT, data->pad_conf);
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
