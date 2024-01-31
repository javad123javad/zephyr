/*
 * Copyright (c) 2024 Javad Rahimipetroudi <javad.rahimipetroudi@mind.be>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_tlc59731

/**
 * @file
 * @brief LED driver for the TLC59731 I2C LED driver
 */
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tlc59731, CONFIG_LED_LOG_LEVEL);

#include "led_context.h"

#define DELAY           1//us
#define T_CYCLE_0       4//us
#define T_CYCLE_1       1//us
#define HIGH            1
#define LOW             0
#define ON		0xFF
#define OFF		0x00


struct tlc59731_cfg {
	struct gpio_dt_spec sdi_gpio;
};

struct tlc59731_data {
	uint8_t	r;
	uint8_t	g;
	uint8_t b;
}rgb_latch;

/*****************************/
static inline int rgb_pulse(const struct gpio_dt_spec * led_dev)
{
    int32_t fret = 0;

    fret = gpio_pin_set_dt(led_dev, HIGH);
    if(0 != fret)
    {
        return fret;
    }

    fret = gpio_pin_set_dt(led_dev, LOW);
    if(0 != fret)
    {
        return fret;
    }

    return fret;
}

static int rgb_write_bit(const struct gpio_dt_spec * led_dev, int8_t data)
{
    int err = 0;
    rgb_pulse(led_dev);

    k_busy_wait(DELAY);

    if(data)
    {
        rgb_pulse(led_dev);
        k_busy_wait(T_CYCLE_1);
    }
    else
    {
        k_busy_wait(T_CYCLE_0);

    }

    return err;
}

static int rgb_write_data(const struct gpio_dt_spec * led_dev, uint8_t data )
{
    rgb_write_bit(led_dev, data & (1<<7));
    rgb_write_bit(led_dev, data & (1<<6));
    rgb_write_bit(led_dev, data & (1<<5));
    rgb_write_bit(led_dev, data & (1<<4));
    rgb_write_bit(led_dev, data & (1<<3));
    rgb_write_bit(led_dev, data & (1<<2));
    rgb_write_bit(led_dev, data & (1<<1));
    rgb_write_bit(led_dev, data & (1<<0));

    return 0;
}

static int rgb_write_led(const struct gpio_dt_spec * led_dev, struct tlc59731_data * rgb_data, bool latch)
{
    rgb_write_data(led_dev, 0x3A);//0x3A;write command
    rgb_write_data(led_dev, rgb_data->r);
    rgb_write_data(led_dev, rgb_data->g);
    rgb_write_data(led_dev, rgb_data->b);
#if 0
    if(latch)
    {
        rgb_latch();
    }
    else
    {
        rgb_eos();
    }
#endif

    return 0;

}

static inline void rgb_fill_latch(struct tlc59731_data * rgb_lt, const uint8_t led, const uint8_t value)
{
	switch(led)
        {
                case(0):
                        rgb_lt->r = value;
                        break;
                case(1):
                        rgb_lt->g = value;
                        break;
                case(2):
                        rgb_lt->b = value;
                        break;
                default:
			LOG_ERR("Illegal RGB number/value");
                        break;
        }


}
/**********************************/

static int tlc59731_led_set_brightness(const struct device *dev, uint32_t led,
		uint8_t value)
{
	const struct tlc59731_cfg  *tlc_conf = dev->config;
        const struct gpio_dt_spec *led_gpio = &tlc_conf->sdi_gpio;
        int err = 0;
	
	rgb_fill_latch(&rgb_latch, led, value);
	rgb_write_led(led_gpio, &rgb_latch, false);
	
	return err;
}

static inline int tlc59731_led_on(const struct device *dev, uint32_t led)
{
	const struct tlc59731_cfg  *tlc_conf = dev->config;
        const struct gpio_dt_spec *led_gpio = &tlc_conf->sdi_gpio;
        int err = 0;
	
        rgb_fill_latch(&rgb_latch, led, ON);
	rgb_write_led(led_gpio, &rgb_latch, false);

	return err;
}

static inline int tlc59731_led_off(const struct device *dev, uint32_t led)
{
	const struct tlc59731_cfg  *tlc_conf = dev->config;
        const struct gpio_dt_spec *led_gpio = &tlc_conf->sdi_gpio;
        int err = 0;
	
        rgb_fill_latch(&rgb_latch, led, OFF);
        rgb_write_led(led_gpio, &rgb_latch,false);

	return err;
}

static int tlc59731_led_blink(const struct device *dev, uint32_t led,
                uint32_t delay_on, uint32_t delay_off)
{
	tlc59731_led_on(dev, led);
        k_msleep(delay_on);
        tlc59731_led_off(dev, led);
        k_msleep(delay_off);


        return 0;
}

static int tlc59731_led_init(const struct device *dev)
{
	const struct tlc59731_cfg  *tlc_conf = dev->config;
	const struct gpio_dt_spec *led = &tlc_conf->sdi_gpio;
	int err = 0;

	if (!device_is_ready(led->port)) 
	{
		LOG_ERR("%s: no LEDs found (DT child nodes missing)", dev->name);
        	err = -ENODEV;
    	}

	err = gpio_pin_configure_dt(led, GPIO_OUTPUT_ACTIVE);
	if (err < 0) {
        	LOG_ERR("%s: no LEDs found (DT child nodes missing)", dev->name);

        	err = -EIO;
    	}

	err = gpio_pin_set_dt(led, LOW);
    	if(err < 0)
    	{
		LOG_ERR("%s: Unable to set the SDI-GPIO)", dev->name);
        	err = -EIO;
    	}
	gpio_pin_set_dt(led, HIGH);
    	gpio_pin_set_dt(led, LOW);
	
	k_busy_wait((DELAY + T_CYCLE_0 ));	
	
	return err;
}

static const struct led_driver_api tlc59731_led_api = {
	.blink = tlc59731_led_blink,
	.set_brightness = tlc59731_led_set_brightness,
	.on = tlc59731_led_on,
	.off = tlc59731_led_off,
};

#define TLC59731_DEVICE(i)                                      \
static struct tlc59731_cfg tlc59731_cfg_##i = {			\
        .sdi_gpio =  GPIO_DT_SPEC_INST_GET(i, gpios),		\
};                                                              \
								\
DEVICE_DT_INST_DEFINE(i, &tlc59731_led_init, NULL,              \
                      NULL, &tlc59731_cfg_##i,               	\
                      POST_KERNEL, CONFIG_LED_INIT_PRIORITY,	\
                      &tlc59731_led_api);			\


DT_INST_FOREACH_STATUS_OKAY(TLC59731_DEVICE)

