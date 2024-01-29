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
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

LOG_MODULE_REGISTER(tlc59731, CONFIG_LED_LOG_LEVEL);

#include "led_context.h"

#define DELAY           1//us
#define T_CYCLE_0       4//us
#define T_CYCLE_1       1//us
#define HIGH            1
#define LOW             0


struct tlc59731_cfg {
	const struct gpio_dt_spec *sdi_gpio;
	const struct gpio_dt_spec *cs_gpio;

};

struct tlc59731_data {
	struct led_data dev_data;
};

static int tlc59731_set_ledout(const struct device *dev, uint32_t led,
		uint8_t val)
{
	const struct tlc59731_cfg *config = dev->config;

	return 0;
}

static int tlc59731_led_blink(const struct device *dev, uint32_t led,
		uint32_t delay_on, uint32_t delay_off)
{
	const struct tlc59731_cfg *config = dev->config;
	struct tlc59731_data *data = dev->data;
	struct led_data *dev_data = &data->dev_data;
	return 0;
}

static int tlc59731_led_set_brightness(const struct device *dev, uint32_t led,
		uint8_t value)
{
	const struct tlc59731_cfg *config = dev->config;
	struct tlc59731_data *data = dev->data;
	struct led_data *dev_data = &data->dev_data;

	return 0;
}

static inline int tlc59731_led_on(const struct device *dev, uint32_t led)
{
	return 0;
}

static inline int tlc59731_led_off(const struct device *dev, uint32_t led)
{
	return 0;
}

static int tlc59731_led_init(const struct device *dev)
{
	const struct tlc59731_cfg *config = dev->config;
	struct tlc59731_data *data = dev->data;
	struct led_data *dev_data = &data->dev_data;



	return 0;
}

static const struct led_driver_api tlc59731_led_api = {
	.blink = tlc59731_led_blink,
	.set_brightness = tlc59731_led_set_brightness,
	.on = tlc59731_led_on,
	.off = tlc59731_led_off,
};

#define TLC59731_DEVICE(i)                                      \
                                                                \
static const struct tlc59731_cfg tlc59731_cfg_##i = {		\
        .sdi_gpio =  GPIO_DT_SPEC_INST_GET(i, sdi_gpios), 	\
	.cs_gpio  = GPIO_DT_SPEC_INST_GET(i, enable_gpios), 	\
};                                                              \
DEVICE_DT_INST_DEFINE(i, &tlc59731_led_init, NULL,              \
                      NULL, &tlc59731_cfg_##i,               \
                      POST_KERNEL, CONFIG_LED_INIT_PRIORITY,    \
                      &tlc59731_led_api);



DT_INST_FOREACH_STATUS_OKAY(TLC59731_DEVICE)
