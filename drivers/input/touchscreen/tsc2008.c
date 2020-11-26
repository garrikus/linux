// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * TSC2005 touchscreen driver
 *
 * Copyright (C) 2006-2010 Nokia Corporation
 * Copyright (C) 2015 QWERTY Embedded Design
 * Copyright (C) 2015 EMAC Inc.
 *
 * Based on original tsc2005.c by Lauri Leukkunen <lauri.leukkunen@nokia.com>
 */

#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/spi/tsc2008.h>

static int tsc2008_get_pendown_state(uint32_t gpio)
{
	return !gpio_get_value(gpio);
}

static irqreturn_t tsc2008_irq(int irq, void *handle)
{
	struct tsc2008 *ts = handle;

	if (!ts->get_pendown_state || likely(ts->get_pendown_state(ts->gpio))) {
		disable_irq_nosync(ts->irq);
		schedule_delayed_work(&ts->work,
				      msecs_to_jiffies(TS_POLL_DELAY));
	}

	if (ts->clear_penirq)
		ts->clear_penirq();

	return IRQ_HANDLED;
}

static void tsc2008_send_up_event(struct tsc2008 *tsc)
{
	struct input_dev *input = tsc->input;

	dev_dbg(&tsc->spi->dev, "UP\n");

	input_report_key(input, BTN_TOUCH, 0);
	input_report_abs(input, ABS_PRESSURE, 0);
	input_sync(input);
}

static void tsc2008_read_values(struct tsc2008 *tsc, struct ts_event *tc)
{
	/* Read the values from the TSC2008 */
	tsc->xfer[1].rx_buf = &tc->x;
	tsc->xfer[3].rx_buf = &tc->y;
	tsc->xfer[5].rx_buf = &tc->z1;
	tsc->xfer[7].rx_buf = &tc->z2;

	spi_sync(tsc->spi, &tsc->msg);

	dev_dbg(&tsc->spi->dev, "data: x  %05u 0x%08x\n", tc->x, tc->x);
	dev_dbg(&tsc->spi->dev, "data: y  %05u 0x%08x\n", tc->y, tc->y);
	dev_dbg(&tsc->spi->dev, "data: z1 %05u 0x%08x\n", tc->z1, tc->z1);
	dev_dbg(&tsc->spi->dev, "data: z2 %05u 0x%08x\n", tc->z2, tc->z2);

	if (tsc->max_bits == MAX_8BIT) {
		tc->x  = (tc->x  << 1) & 0x00FF;
		tc->y  = (tc->y  << 1) & 0x00FF;
		tc->z1 = (tc->z1 << 1) & 0x00FF;
		tc->z2 = (tc->z2 << 1) & 0x00FF;
	} else if (tsc->max_bits == MAX_12BIT) {
		tc->x  = (be16_to_cpu(tc->x)  << 1) >> 4 & 0x0FFF;
		tc->y  = (be16_to_cpu(tc->y)  << 1) >> 4 & 0x0FFF;
		tc->z1 = (be16_to_cpu(tc->z1) << 1) >> 4 & 0x0FFF;
		tc->z2 = (be16_to_cpu(tc->z2) << 1) >> 4 & 0x0FFF;
	} else {
		dev_err(&tsc->spi->dev, "invalid number of bits expected\n");
	}

	dev_dbg(&tsc->spi->dev, "data: x  %05u 0x%08x\n", tc->x, tc->x);
	dev_dbg(&tsc->spi->dev, "data: y  %05u 0x%08x\n", tc->y, tc->y);
	dev_dbg(&tsc->spi->dev, "data: z1 %05u 0x%08x\n", tc->z1, tc->z1);
	dev_dbg(&tsc->spi->dev, "data: z2 %05u 0x%08x\n", tc->z2, tc->z2);
}

static uint32_t tsc2008_calculate_pressure(struct tsc2008 *tsc, struct ts_event *tc)
{
	uint32_t rt = 0;

	/* range filtering */
	if (tc->x == (tsc->max_bits - 1))
		tc->x = 0;

	if (likely(tc->x && tc->z1)) {
		rt = (tc->z2 / tc->z1) - 1;
		rt *= tc->y * tsc->x_plate_ohms;
		rt /= tsc->max_bits;

		if (tsc->max_bits == MAX_8BIT)
			rt &= 0x00FF;
		else
			rt &= 0x0FFF;
	}

	dev_dbg(&tsc->spi->dev, "pressure: %05u 0x%08x\n", rt, rt);

	return rt;
}

static void tsc2008_work(struct work_struct *work)
{
	struct tsc2008 *ts = container_of(to_delayed_work(work), struct tsc2008, work);
	struct ts_event tc;
	uint32_t        rt;

	/*
	 * NOTE: We can't rely on the pressure to determine the pen down
	 * state, even though this controller has a pressure sensor.
	 * The pressure value can fluctuate for quite a while after
	 * lifting the pen and in some cases may not even settle at the
	 * expected value.
	 *
	 * The only safe way to check for the pen up condition is in the
	 * work function by reading the pen signal state (it's a GPIO
	 * and IRQ). Unfortunately such callback is not always available,
	 * in that case we have rely on the pressure anyway.
	 */
	if (ts->get_pendown_state) {
		if (unlikely(!ts->get_pendown_state(ts->gpio))) {
			dev_dbg(&ts->spi->dev, "detected pen UP via GPIO read\n");

			tsc2008_send_up_event(ts);
			ts->pendown = false;
			goto out;
		}

		dev_dbg(&ts->spi->dev, "pen is still down\n");
	} else {
		dev_dbg(&ts->spi->dev, "get_pendown_state undefined, "
				"using pressure instead\n");
	}

	tsc2008_read_values(ts, &tc);

	rt = tsc2008_calculate_pressure(ts, &tc);
	if (rt >= ts->max_bits) {
		/*
		 * Sample found inconsistent by debouncing or pressure is
		 * beyond the maximum. Don't report it to user space,
		 * repeat at least once more the measurement.
		 */
		dev_dbg(&ts->spi->dev, "ignored pressure %d\n", rt);
		goto out;
	}

	if (rt) {
		struct input_dev *input = ts->input;

		if (!ts->pendown) {
			dev_dbg(&ts->spi->dev, "DOWN\n");

			input_report_key(input, BTN_TOUCH, 1);
			ts->pendown = true;
		}

		input_report_abs(input, ABS_X, tc.x);
		input_report_abs(input, ABS_Y, tc.y);
		input_report_abs(input, ABS_PRESSURE, rt);

		input_sync(input);

		dev_dbg(&ts->spi->dev, "point(%4d,%4d), pressure (%4u)\n",
			tc.x, tc.y, rt);
	} else if (!ts->get_pendown_state && ts->pendown) {
		/*
		 * We don't have callback to check pendown state, so we
		 * have to assume that since pressure reported is 0 the
		 * pen was lifted up.
		 */
		tsc2008_send_up_event(ts);
		ts->pendown = false;
	} else {
		dev_dbg(&ts->spi->dev, "no change in pen state detected\n");
	}

 out:
	if (ts->pendown)
		schedule_delayed_work(&ts->work,
				      msecs_to_jiffies(TS_POLL_PERIOD));
	else
		enable_irq(ts->irq);
}

static int tsc2008_probe(struct spi_device *spi)
{  
  struct tsc2008      *ts = 0;
  struct input_dev    *input_dev = 0;
  struct device_node  *node_ptr = spi->dev.of_node;
  struct spi_transfer *x = 0;
  uint32_t val32 = 0;
  uint64_t val64 = 0;
  uint8_t tx_buf = 0;
  int     error  = 0;
  int     num_bytes_rx = 0;

  printk(KERN_DEBUG "tsc2008: probe start...\n");

  printk(KERN_DEBUG "tsc2008: spi max speed %d\n", spi->max_speed_hz);

	if( spi->irq <= 0 ) {
		printk( KERN_DEBUG "tsc2008 spi irq err\n");
		return -ENODEV;
	}

	if( !node_ptr )
	{
    printk( KERN_DEBUG "tsc2008 no device tree data\n");
		return -EINVAL;
	}

  ts = devm_kzalloc(&spi->dev, sizeof(*ts), GFP_KERNEL );
  if( !ts )
  {
  	printk( KERN_DEBUG "tsc2008 no mem for tsc2008\n");
  	return -ENOMEM;
  }

  /* WARNING!!! FOR DEVICE TREE VERSION ONLY */
  ts->gpio = of_get_gpio(node_ptr, 0);
  if(gpio_is_valid(ts->gpio))
  {
  	printk( KERN_DEBUG "tsc2008 gpio OK\n");
    ts->get_pendown_state = tsc2008_get_pendown_state;
  }
  else
  {
  	printk( KERN_DEBUG "tsc2008 gpio ERR\n");
  	error = -EINVAL;
  	goto fail_free_spi_mem;
  }	

  if(!of_property_read_u32(node_ptr, "ti,x-plate-ohms", &val32))
  {
  	printk( KERN_DEBUG "tsc2008 x-plate-ohms read OK (%d)\n", val32);
  	ts->x_plate_ohms = val32;
  }
  else
  {
  	printk( KERN_DEBUG "tsc2008 x-plate-ohms read ERR\n");
  	return -EINVAL;
  }

  if(!of_property_read_u32(node_ptr, "ti,max-bits", &val32))
  {
  	printk( KERN_DEBUG "tsc2008 max bits read OK (%d)\n", val32);
  	if( (val32 == 8) || (val32 == 12) )
  	{
  		ts->max_bits = (1 << val32);
  		spi->bits_per_word = val32; 
  	}
  	else
  	{
  		printk( KERN_DEBUG "tsc2008 max bits wrong value %d\n", val32);
  	  ts->max_bits = (1 << 12);
  	  spi->bits_per_word = 12;
  	}
  }
  else
  {
  	printk( KERN_DEBUG "tsc2008 max bits read ERR\n");
  	ts->max_bits = (1 << 12);
  	spi->bits_per_word = 12;
  }

  spi->mode = SPI_MODE_0; // original MicroWave
  
  error = spi_setup(spi);
	if (error)
	{
		printk( KERN_DEBUG "tsc2008 spi setup error\n");
		return error;
	}

  input_dev = devm_input_allocate_device(&spi->dev);
  if( !input_dev )
  {
    printk( KERN_DEBUG "tsc2008 no mem for input_dev\n");
    error = -ENOMEM;
    goto fail_free_spi_mem;
  }

  ts->spi = spi;
  ts->irq = spi->irq;
  ts->input = input_dev;
  INIT_DELAYED_WORK(&ts->work, tsc2008_work);

  snprintf(ts->phys, sizeof(ts->phys),
		 "%s/input-ts", dev_name(&spi->dev));

  input_dev->name = devm_kasprintf(&spi->dev, 
  	                               GFP_KERNEL,
						                       "TSC2008 touchscreen");
  
  
  input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

  input_set_abs_params(input_dev, ABS_X, 0, ts->max_bits, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, ts->max_bits, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, ts->max_bits, 0, 0);

  error = request_irq(ts->irq, 
  	                  tsc2008_irq, 
  	                  IRQF_TRIGGER_FALLING,
			                spi->dev.driver->name, ts);
	if (error < 0) {
		printk(KERN_DEBUG "ts2008: irq error %d...\n", error);
		goto fail_free_input_device;
	}
	else
	{
    printk(KERN_DEBUG "ts2008 irq OK\n");
	}

	/* Setup SPI variables */
	spi_message_init(&ts->msg);

	/*
	 * TSC2008_MEASURE_X and TSC2008_MEASURE_Y are intentionally
	 * swapped below.
	 */
	if (ts->max_bits == MAX_8BIT) {
		dev_dbg(&spi->dev, "using 8-bit mode\n");
		num_bytes_rx = 1;
		ts->tx_buf[0] = ADC_ON_8BIT | TSC2008_MEASURE_Y;
		ts->tx_buf[1] = ADC_ON_8BIT | TSC2008_MEASURE_X;
		ts->tx_buf[2] = ADC_ON_8BIT | TSC2008_MEASURE_Z1;
		ts->tx_buf[3] = ADC_ON_8BIT | TSC2008_MEASURE_Z2;
	} else {
		dev_dbg(&spi->dev, "using 12-bit mode\n");
		num_bytes_rx = 2;
		ts->tx_buf[0] = ADC_ON_12BIT | TSC2008_MEASURE_Y;
		ts->tx_buf[1] = ADC_ON_12BIT | TSC2008_MEASURE_X;
		ts->tx_buf[2] = ADC_ON_12BIT | TSC2008_MEASURE_Z1;
		ts->tx_buf[3] = ADC_ON_12BIT | TSC2008_MEASURE_Z2;
	}

	x = &ts->xfer[0];
	memset(x, 0, sizeof(*x));
	x->tx_buf = &ts->tx_buf[0];
	x->len = 1;
	x->delay_usecs = TX_DELAY;
	spi_message_add_tail(x, &ts->msg);

	x = &ts->xfer[1];
	memset(x, 0, sizeof(*x));
	x->len = num_bytes_rx;
	x->delay_usecs = RX_DELAY;
	spi_message_add_tail(x, &ts->msg);

	x = &ts->xfer[2];
	memset(x, 0, sizeof(*x));
	x->tx_buf = &ts->tx_buf[1];
	x->len = 1;
	x->delay_usecs = TX_DELAY;
	spi_message_add_tail(x, &ts->msg);

	x = &ts->xfer[3];
	memset(x, 0, sizeof(*x));
	x->len = num_bytes_rx;
	x->delay_usecs = RX_DELAY;
	spi_message_add_tail(x, &ts->msg);

	x = &ts->xfer[4];
	memset(x, 0, sizeof(*x));
	x->tx_buf = &ts->tx_buf[2];
	x->len = 1;
	x->delay_usecs = TX_DELAY;
	spi_message_add_tail(x, &ts->msg);

	x = &ts->xfer[5];
	memset(x, 0, sizeof(*x));
	x->len = num_bytes_rx;
	x->delay_usecs = RX_DELAY;
	spi_message_add_tail(x, &ts->msg);

	x = &ts->xfer[6];
	memset(x, 0, sizeof(*x));
	x->tx_buf = &ts->tx_buf[3];
	x->len = 1;
	x->delay_usecs = TX_DELAY;
	spi_message_add_tail(x, &ts->msg);

	x = &ts->xfer[7];
	memset(x, 0, sizeof(*x));
	x->len = num_bytes_rx;
	spi_message_add_tail(x, &ts->msg);

	tx_buf = TSC2008_START_CMD
			   | TSC2008_SETUP
			   | TSC2008_SOFTWARE_RESET;

	error = spi_write(spi, &tx_buf, 1);
	if (error < 0)
	{
    printk(KERN_DEBUG "ts2008: spi write err 1\n");
		goto fail_free_input_device;
	}
	udelay(1);

	/* Configure the pull-up resistor and MAV settings */
	tx_buf = TSC2008_START_CMD
			| TSC2008_SETUP
			| TSC2008_USE_90K
			| TSC2008_DISABLE_MAV;
	error = spi_write(spi, &tx_buf, 1);
	if (error < 0)
	{
		printk(KERN_DEBUG "ts2008: spi write err 2...\n");
		goto fail_free_input_device;
	}

	/* Finish input device registration */
	error = input_register_device(input_dev);
	if (error)
	{
		printk(KERN_DEBUG "ts2008: input dev reg error...\n");
		goto fail_free_input_device;
	}

  printk(KERN_DEBUG "ts2008: probe end...\n");
	return 0;

  printk(KERN_DEBUG "ts2008: probe fail (code %d)...\n", error);
fail_free_input_device:
  input_free_device(input_dev);
fail_free_spi_mem:
  kfree(ts);
	return error;
}

static int tsc2008_remove(struct spi_device *spi)
{
	//printk(KERN_DEBUG "ts2008: remove...\n");
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id tsc2008_of_match[] = {
	{ .compatible = "ti,tsc2008" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, tsc2008_of_match);
#endif

static struct spi_driver tsc2008_driver = {
	.driver	= {
		.name	= "tsc2008",
		.of_match_table = of_match_ptr(tsc2008_of_match),
	},
	.probe	= tsc2008_probe,
	//.remove	= tsc2008_remove,
};

module_spi_driver(tsc2008_driver);

MODULE_AUTHOR("Michael Welling <mwelling@ieee.org>");
MODULE_DESCRIPTION("TSC2008 Touchscreen Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:tsc2008");