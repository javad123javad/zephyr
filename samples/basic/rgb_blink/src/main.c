/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tlc59731_,LOG_LEVEL_DBG);
/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000
#define DELAY           1//us
#define T_CYCLE_0       4//us
#define T_CYCLE_1       1//us
#define HIGH            1
#define LOW             0

#define latch_length 160
#define eos_length 70

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define LED0_CS_NODE DT_ALIAS(led0cs)
#define LED_CNTRL DT_ALIAS(rgblld)
/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led_cs = GPIO_DT_SPEC_GET(LED0_CS_NODE, gpios);
const struct device *tlc59731 = DEVICE_DT_GET_ONE(ti_tlc59731);
// static const struct gpio_dt_spec led_cs = GPIO_DT_SPEC_GET(LED0_CS_NODE, gpios);;//GPIO_DT_SPEC_INST_GET_BY_IDX(DT_NODELABEL(led_cs), gpios, 1);

int32_t rgb_pulse()
{
    int32_t fret = 0;

    fret = gpio_pin_set_dt(&led, HIGH);
    if(0 != fret)
    {
        LOG_ERR("Pulse error");
        return fret;
    }

    fret = gpio_pin_set_dt(&led, LOW);
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
    fret = gpio_pin_set_dt(&led_cs, HIGH);
    if(fret < 0)
    {
        return fret;
    }

    fret = gpio_pin_set_dt(&led, 0);
    if(fret < 0)
    {
        return fret;
    }

    k_msleep(10);

    return fret;
}
void rgb_write_bit(uint8_t data)
{
    int32_t fret = 0;
    fret = rgb_pulse();
    
    k_busy_wait(DELAY);

    if(data)
    {
        rgb_pulse();
        k_busy_wait(T_CYCLE_1);
    }
    else
    {
        k_busy_wait(T_CYCLE_0);

    }/**/
}
int32_t rgb_write_data(uint8_t data )
{
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
    k_busy_wait(eos_length);
    return 0;
}

int32_t rgb_latch()
{
    k_busy_wait(latch_length);

    return 0;

}

int32_t rgb_write_led(uint8_t r, uint8_t g, uint8_t b, bool latch)
{
    rgb_write_data(0x3A);//0x3A;write command
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
    if (tlc59731) {
        LOG_DBG("Found LED controller %s", tlc59731->name);
    }



#if 0
    LOG_DBG("Init RGB");
    int32_t fret = init_rgb_led();
    if(fret < 0)
    {
        LOG_ERR("init_rgb_led");
        return fret;
    }


    gpio_pin_set_dt(&led, HIGH);
    gpio_pin_set_dt(&led, LOW);

    k_busy_wait((DELAY +T_CYCLE_0 ));

    uint8_t dimmer = 0xFF;
    while (1) {
        rgb_write_led(dimmer,-dimmer,dimmer,false);

        
        dimmer--;
        k_msleep(5);
    }
#endif
    return 0;
}
