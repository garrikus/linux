#include <linux/of.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>

#define PAD_CONF_MCBSP3_FSX_ADDRESS       (0x48002172)
#define PAD_CONF_SET_UART2_RX             (0x119)
#define PAD_CONF_SET_GPIO_143_INPUT       (0x411C)

struct gu_pinmux_data {
	void __iomem *pad_conf;
};

static irqreturn_t gpio_hndl(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;

	dev_dbg(&pdev->dev, "GET IRQ %d\n", irq);

	return IRQ_HANDLED;
}

static int gpio_uart_pinmux_probe(struct platform_device *pdev)
{
	static struct gu_pinmux_data *data;
	int err, irq;

	dev_dbg(&pdev->dev, "probe starts\n");

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if(data == NULL){
		dev_err(&pdev->dev, "no memory\n");
		return (-ENOMEM);
	}

	data->pad_conf = ioremap(PAD_CONF_MCBSP3_FSX_ADDRESS, 2);
	if(data->pad_conf == NULL){
		dev_err(&pdev->dev, "cann't remap PAD_CONF_MCBSP3_FSX (0x%08X)\n",
										 PAD_CONF_MCBSP3_FSX_ADDRESS);
		kfree(data);
		return (-ENXIO);
	}

	irq = platform_get_irq(pdev, 0);
	if( irq <= 0 ) {
		dev_err(&pdev->dev, "No free IRQs\n");
		kfree(data);
		return (-ENXIO);
	}

	err = request_irq(irq, gpio_hndl, IRQF_NO_SUSPEND	 | IRQF_NO_THREAD,
							pdev->name, pdev);

	if (err) {
		dev_err(&pdev->dev, "Unable to claim irq %d; error %d\n", irq, err);
		kfree(data);
		return err;
	}

	platform_set_drvdata(pdev, data);

	dev_dbg(&pdev->dev, "probe successfully ends\n");
	return 0;
}

static int gpio_uart_pinmux_remove(struct platform_device *pdev)
{
	struct gu_pinmux_data *data = platform_get_drvdata(pdev);

	iounmap(data->pad_conf);
	kfree(data);

	return 0;
}

static int gpio_uart_pinmux_suspend(struct platform_device *pdev,
						pm_message_t state)
{
	struct gu_pinmux_data *data = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "suspend %d\n", state.event);
	writew(PAD_CONF_SET_GPIO_143_INPUT, data->pad_conf);

	return 0;
}

static int gpio_uart_pinmux_resume(struct platform_device *pdev)
{
	struct gu_pinmux_data *data = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "resume\n");

	writew(PAD_CONF_SET_UART2_RX, data->pad_conf);

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

MODULE_DESCRIPTION("Driver for reconfig pin UART2 on suspend/resume events");
MODULE_AUTHOR("Vladimir Hrechko");
MODULE_LICENSE("GPL");
