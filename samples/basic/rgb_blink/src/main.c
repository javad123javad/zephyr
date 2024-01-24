/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tlc59731,LOG_LEVEL_DBG);
/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000
#define T_WO			125 //ns
/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define LED0_CS_NODE DT_ALIAS(led0cs)
/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led_cs = GPIO_DT_SPEC_GET(LED0_CS_NODE, gpios);

int32_t rgb_pulse()
{
    int32_t fret = 0;
    fret = gpio_pin_set_dt(&led, 0);
    if(0 != fret)
    {
        LOG_ERR("Pulse error");
        return fret;
    }
    k_sleep(K_NSEC(T_WO));
    fret = gpio_pin_set_dt(&led, 1);
    if(0 != fret)
    {
        LOG_ERR("Pulse error");
        return fret;
    }

	return fret;

}
int32_t init_rgb_led()
{
    int32_t fret = -1;

    if (!gpio_is_ready_dt(&led)
            || !gpio_is_ready_dt(&led_cs)) {
        printf("RGB LED error.\r\n");
        return fret;
    }

    fret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (fret < 0) {
        printf("Error conf SDI pin\r\n");

        return fret;
    }

    fret = gpio_pin_configure_dt(&led_cs, GPIO_OUTPUT_ACTIVE);
    if (fret < 0) {
        printf("Error conf CS pin\r\n");
        return fret;
    }

    /* Enable RGB chip select */
    fret = gpio_pin_set_dt(&led_cs, 0);
    if(fret < 0)
    {
        return fret;
    }

    fret = gpio_pin_set_dt(&led, 1);
    if(fret < 0)
    {
        return fret;
    }



    return fret;
}
int main(void)
{

    LOG_DBG("Init RGB");
    init_rgb_led();



    while (1) {
        /*
        	ret = gpio_pin_toggle_dt(&led);
        	if (ret < 0) {
        		return 0;
        	}

        	led_state = !led_state;
        	printf("LED state: %s\n", led_state ? "ON" : "OFF");
        */
        k_msleep(SLEEP_TIME_MS);
    }
    return 0;
}
