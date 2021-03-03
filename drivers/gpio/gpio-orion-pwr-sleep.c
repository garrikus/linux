#include <linux/of.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/irqdomain.h>
#include <linux/bitops.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio/driver.h>
#include <linux/platform_device.h>

#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf-generic.h>


static int gpio_uart_pinmux_probe(struct platform_device *pdev)
{
//  u16 reg = 0;

  printk("GPIO UART PINMUX PROBE BGN\n");

  /*reg = readw(0x48002170);
  printk("REG: 0x04X\n", reg);
  reg = readw(0x48002172);
  printk("REG: 0x04X\n", reg);*/

  printk("GPIO UART PINMUX PROBE END\n");
  return 0;
}

static int gpio_uart_pinmux_suspend(struct platform_device *pdev, pm_message_t state)
{
  printk("GPIO UART PINMUX SUSPEND\n");
  return 0;
}

static int gpio_uart_pinmux_resume(struct platform_device *pdev)
{
  printk("GPIO UART PINMUX RESUME\n");
  return 0;
}

static const struct of_device_id gpio_uart_pinmux_of_match[] = {
	{ .compatible = "orion-gpio-uart-pinmux" },
	{ },
};

static struct platform_driver gpio_uart_pinmux_driver = {
	.probe = gpio_uart_pinmux_probe,
  .suspend = gpio_uart_pinmux_suspend,
  .resume  = gpio_uart_pinmux_resume,
	.driver = {
		.name = "gpio-uart-pinmux",
		.of_match_table = of_match_ptr(gpio_uart_pinmux_of_match),
	},
};

module_platform_driver(gpio_uart_pinmux_driver);

MODULE_DESCRIPTION("Driver for reconfig pin UART2 in suspend/resume events");
MODULE_AUTHOR("Vladimir Hrechko");
MODULE_LICENSE("GPL");
