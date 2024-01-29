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

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/led.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tlc59731, CONFIG_LED_LOG_LEVEL);

#include "led_context.h"

struct tlc59731_cfg {
	int sdi_port;
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

#define TLC59731_DEVICE(id) \
	static const struct tlc59731_cfg tlc59731_##id##_cfg = {	\
	};								\
	static struct tlc59731_data tlc59731_##id##_data;		\
									\
	DEVICE_DT_INST_DEFINE(id, &tlc59731_led_init, NULL,		\
			&tlc59731_##id##_data,				\
			&tlc59731_##id##_cfg, POST_KERNEL,		\
			CONFIG_LED_INIT_PRIORITY,			\
			&tlc59731_led_api);

DT_INST_FOREACH_STATUS_OKAY(TLC59731_DEVICE)
