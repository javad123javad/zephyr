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

#define WHOAMI	0x01
#define TEMP_H_LIMIT	0x02
#define TEMP_L_LIMIT	0x03
#define CTRL		0x04
#define STATUS		0x05
#define TEMP_L_OUT	0x06
#define TEMP_H_OUT	0x07

uint8_t stts22h_reg_map[] = {
	WHOAMI, TEMP_H_LIMIT, TEMP_L_LIMIT,
	CTRL, STATUS, TEMP_L_OUT, TEMP_H_OUT};

#define GET_STTS22H_REG(x)	(&stts22h_reg_map[(x)-1])


/* Control register */
#define	ONE_SHOT	BIT(0)
#define TIME_OUT_DIS	BIT(1)
#define FREERUN		BIT(2)
#define IF_ADD_INC	BIT(3)
#define AVG		GENMASK(5,4)
#define BDU		BIT(6)
#define LOW_ODR_START	BIT(7)



/* The devicetree node identifier for the "stts22h" temerature sensor alias. */
#define STTS22_NODE DT_NODELABEL(temp_sensor)

static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(STTS22_NODE);
/* Sensor functions */
static int32_t stts22h_configure()
{
	int err = 0;
	uint8_t ctrl_reg = 0;

	ctrl_reg = FIELD_PREP(FREERUN, 1)
	           | FIELD_PREP(LOW_ODR_START, 0);
	uint8_t wr_buf[] = {CTRL, ctrl_reg};
	err = i2c_write_dt(&dev_i2c, wr_buf, 2);

	return err;
}
static int32_t stts22h_get_temperature(int32_t * temp)
{
	int err = 0;
	uint8_t buf[2] = {0};
	int32_t utemp = 0;

	if(temp == NULL)
	{
		printk("Inavlid temp argument\n");
		return -EINVAL;
	}
	err = i2c_write_read_dt(&dev_i2c, GET_STTS22H_REG(TEMP_H_OUT) , 1, &buf[0], 1);
	if(err)
	{
		return err;
	}

	err = i2c_write_read_dt(&dev_i2c, GET_STTS22H_REG(TEMP_L_OUT) , 1, &buf[1], 1);
	if(err)
	{
		return err;
	}

	utemp = (buf[0]<<8) | (buf[1]);
	utemp = (utemp < 0x8000) ? (utemp/100):((utemp-0x10000)/100);
	*temp = utemp;
	return err;
}

int main(void)
{
	int ret;
	uint8_t buf[8] = {0};
	int32_t temp = 0;

	if (!device_is_ready(dev_i2c.bus)) {
		printk("I2C bus %s is not ready!\n\r",dev_i2c.bus->name);
		return -1;
	}

	while(1)
	{
		ret = stts22h_configure();
		if(ret)
		{
			printk("Config error: %d\n", ret);
		}
		ret = i2c_write_read_dt(&dev_i2c, GET_STTS22H_REG(WHOAMI) , 1, &buf[0], 1);
		if(ret)
		{
			printk("Error reading from %02X reg. Erro: %d\n", stts22h_reg_map[WHOAMI], ret);
		}

		k_msleep(SLEEP_TIME_MS);

		ret = stts22h_get_temperature(&temp);
		if(ret)
		{
			printk("Error read temp: %d\n", ret);
		}

		printk("Temperature: %d\n", temp);

	}
	return 0;
}
