/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/led.h>
LOG_MODULE_REGISTER(tlc59731_,LOG_LEVEL_DBG);
/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000
#define DELAY           1//us
#define T_CYCLE_0       4//us
#define T_CYCLE_1       1//us

#define HIGH            1
#define LOW             0

#define RED             0
#define GREEN           1
#define BLUE            2

#define latch_length 160
#define eos_length 70

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define LED0_CS_NODE DT_ALIAS(led0cs)
#define LED_CNTRL DT_ALIAS(rgblld)


const struct device *tlc59731 = DEVICE_DT_GET(LED_CNTRL);
static const struct gpio_dt_spec led_cs = GPIO_DT_SPEC_GET(LED0_CS_NODE, gpios);


/**
 * @brief Test RGB LED Brightness Levels
 *
 * @return int
 */
int test_rgb_brightness(void)
{
    int fret = 0;
    uint8_t rgb = 0;
    uint8_t level = 0;

    while (rgb<3)
    {


        while (level < 0xFF)
        {
            led_set_brightness(tlc59731, rgb, level++);
            k_msleep(4);
        }

        k_msleep(200);
        while (level>0)
        {
            led_set_brightness(tlc59731, rgb, level--);
            k_msleep(4);
        }

        rgb++;
        level = 0x00;
    }



    return fret;
}
/**
 * @brief
 *
 * @return int
 */
int main(void)
{
    int fret = gpio_pin_configure_dt(&led_cs, GPIO_OUTPUT_ACTIVE);
    if (fret < 0) {
        printf("Error conf CS pin\r\n");
        return fret;
    }

    /* Enable RGB chip select */
    fret = gpio_pin_set_dt(&led_cs, HIGH);
    if(fret < 0)
    {
        return fret;
    }
    k_msleep(5);

    if (tlc59731) {
        LOG_DBG("Found LED controller %s", tlc59731->name);
    } else if (!device_is_ready(tlc59731)) {
        LOG_ERR("LED device %s is not ready", tlc59731->name);
        return 0;
    } else {
        LOG_INF("Found LED device %s", tlc59731->name);
    }
    while (1)
    {
        led_on(tlc59731,RED);
        k_msleep(500);
        led_on(tlc59731,GREEN);
        k_msleep(500);
        led_on(tlc59731,BLUE);
        k_msleep(500);
        led_off(tlc59731,RED);
        k_msleep(500);
        led_off(tlc59731,GREEN);
        k_msleep(500);
        led_off(tlc59731,BLUE);
        led_on(tlc59731,RED);
        led_on(tlc59731,GREEN);
        k_msleep(500);
        led_off(tlc59731,RED);
        k_msleep(500);


        led_blink(tlc59731, RED, 500, 500);
        led_blink(tlc59731, GREEN, 500, 500);
        led_blink(tlc59731, BLUE, 500, 500);

        test_rgb_brightness();


    }


    return 0;
}
