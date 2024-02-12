/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "stts22h" temerature sensor alias. */
#define STTS22_NODE DT_NODELABEL(temp_sensor)
#define STTS22_NODE_T DO_ALIAS(tmp_sensor)
static const struct device *dev_i2c = DEVICE_DT_GET(STTS22_NODE);//DT_SPEC_GET(STTS22_NODE);

int main(void)
{
	int ret = 0;
	struct sensor_value temp;
	if (dev_i2c == NULL) {
		/* No such node, or the node does not have status "okay". */
		printk("\nError: no device found.\n");
		return NULL;
	}

	ret = device_is_ready(dev_i2c);
	if (ret != true) {
		printk("I2C bus %s is not ready! err: %d\n\r",dev_i2c->name, ret);
		return -1;
	}

	printk("Found device \"%s\", getting sensor data\n", dev_i2c->name);
	sensor_attr_set(dev_i2c, SENSOR_CHAN_ALL, SENSOR_ATTR_SAMPLING_FREQUENCY, 0x22);
	while(1)
	{
		sensor_sample_fetch(dev_i2c);
		sensor_channel_get(dev_i2c, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		printk("temp: %d.%06d\n", temp.val1, temp.val2);
		k_msleep(1000);

	}

	return 0;
}
