/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
//#define LED0_NODE DT_ALIAS(led0)
#define LED0_NODE DT_NODELABEL(ir_led)
#define LED0_CS_NODE DT_NODELABEL(ir_cs)
/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec ir_cs = GPIO_DT_SPEC_GET(LED0_CS_NODE, gpios);

int main(void)
{
	int ret;
	bool led_state = true;

	if (!gpio_is_ready_dt(&led)
			|| !gpio_is_ready_dt(&ir_cs)) {
		return -1;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return -1;
	}

	ret = gpio_pin_configure_dt(&ir_cs, GPIO_OUTPUT_ACTIVE);
	if(ret < 0) {
		printk("CS chip conf err\n");
		return -1;
	}

	/* Enable IR LED CS */
	if((ret = gpio_pin_set_dt(&ir_cs, 1) != 0) )
	{
		printk("Unable to set CS Pin: %d\n", ret);
		return -1;
	}
	
	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}

		led_state = !led_state;
		printf("LED state: %s\n", led_state ? "ON" : "OFF");
		k_msleep(SLEEP_TIME_MS);
	}
	return 0;
}
