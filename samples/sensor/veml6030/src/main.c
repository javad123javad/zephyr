/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define VEML6030_NODE  DT_NODELABEL(veml6030)

static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(VEML6030_NODE);

int main(void)
{
	int ret;
	uint8_t sensor_read[1] = {0x04};
	uint8_t als_reading[2]= {0};

	if (!device_is_ready(dev_i2c.bus)) {
		printk("I2C bus %s is not ready!\n\r",dev_i2c.bus->name);
		return -1 ;
	}

	uint8_t vme_turnon[3] = {0x00,0x00,0x00};

	ret = i2c_write_dt(&dev_i2c, vme_turnon, 3);

	if(ret != 0) {
		printk("Failed to turn on VEML6030 Light sensor. Address %x at reg. %x n",
				dev_i2c.addr,vme_turnon[0]);
	}


	while(1) {
		ret = i2c_write_read_dt(&dev_i2c,&sensor_read[0],1,&als_reading[0],2);
		if(ret != 0) {
			printk("Failed to write/read I2C device address %x at Reg. %x n",
					dev_i2c.addr,sensor_read[0]);
		}
		else {
			uint16_t light_value = als_reading[1]*256 + als_reading[0];
			printk("Ambient light value: %d\r\n", light_value);
		}

		k_msleep(SLEEP_TIME_MS);
	}

	return 0;
}
