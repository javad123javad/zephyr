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
#define T_SDI			500 //ns
#define latch_length 160
#define eos_length 70
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

    fret = gpio_pin_set_dt(&led, 1);
    if(0 != fret)
    {
        LOG_ERR("Pulse error");
        return fret;
    }
    fret = gpio_pin_set_dt(&led, 0);
    if(0 != fret)
    {
        LOG_ERR("Pulse error");
        return fret;
    }

    return fret;

}

int32_t rgb_gslat()
{
    int32_t fret = 0;
    fret = gpio_pin_set_dt(&led, 1);
    k_sleep(K_USEC(50));
    fret = gpio_pin_set_dt(&led, 0);
    k_sleep(K_NSEC(20));
    fret = gpio_pin_set_dt(&led, 1);
    k_sleep(K_USEC(50));
    fret = gpio_pin_set_dt(&led, 0);
    k_sleep(K_NSEC(20));
    fret = gpio_pin_set_dt(&led, 1);
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
void rgb_write_bit(uint8_t data)
{
    int32_t fret = 0;
    fret = rgb_pulse();

    if(data)
    {
        rgb_pulse();

        k_sleep(K_NSEC(30));
    }
    else
    {
        k_sleep(K_USEC(2));
    }/**/
}
int32_t rgb_write_data(uint8_t data )
{
    int32_t fret = 0;
    uint8_t idx = 0;
    rgb_write_bit(data & (1<<7));
    rgb_write_bit(data & (1<<6));
    rgb_write_bit(data & (1<<5));
    rgb_write_bit(data & (1<<4));
    rgb_write_bit(data & (1<<3));
    rgb_write_bit(data & (1<<2));
    rgb_write_bit(data & (1<<1));
    rgb_write_bit(data & (1<<0));
}

int32_t rgb_eos()
{
    k_sleep(K_USEC(eos_length));
    return 0;
}

int32_t rgb_latch()
{
    k_sleep(K_USEC(latch_length));
}

int32_t rgb_write_led(uint8_t r, uint8_t g, uint8_t b, bool latch)
{
    rgb_write_data(0b00111010);
    rgb_write_data(r);
    rgb_write_data(g);
    rgb_write_data(b);

    if(latch)
    {
        rgb_latch();
    }
    else
    {
        rgb_eos();
    }
}

int main(void)
{

    LOG_DBG("Init RGB");
    // init_rgb_led();

    // rgb_gslat();
	rgb_write_led(0xFA,0xEB,0xFF,false);



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
