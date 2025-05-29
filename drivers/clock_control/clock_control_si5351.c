/*
 * Copyright (c) 2025  Javad Rahimipetroudi <javad.rahimipetroudi@mind.be>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT silabs_si5351_clock


#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/clock_control_silabs.h>
#include <zephyr/sys/util.h>

struct si5351_clock_config {
	int speed;
};

static int si5351_clock_control_on(const struct device *dev, clock_control_subsys_t sys)
{
	int err  = 0;


	return err;
}

static int si5351_clock_control_off(const struct device *dev, clock_control_subsys_t sys)
{
	int err = 0;


	return err;
}

static int si5351_clock_control_get_rate(const struct device *dev, clock_control_subsys_t sys,
					 uint32_t *rate)
{
	int err = 0;


	return err;
}

static enum clock_control_status si5351_clock_control_get_status(const struct device *dev,
					   clock_control_subsys_t sys)
{
	int err = 0;


	return err;
}

static int si5351_clock_control_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 0;
}

static DEVICE_API(clock_control, silabs_clock_control_api) = {
	.on = si5351_clock_control_on,
	.off = si5351_clock_control_off,
	.get_rate = si5351_clock_control_get_rate,
	.get_status = si5351_clock_control_get_status,
};

static const struct si5351_clock_config silabs_clock_control_config = {
	.speed = 1000,
};

DEVICE_DT_INST_DEFINE(0, si5351_clock_control_init, NULL, NULL, &silabs_clock_control_config,
		      PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &silabs_clock_control_api);
