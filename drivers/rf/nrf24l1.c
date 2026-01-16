/*
 * Copyright (C) 2024 Javad Rahimipetroudi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#define DT_DRV_COMPAT nordic_nrf24l1

#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/rf/rf.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(nrf24l1, CONFIG_RF_LOG_LEVEL);

struct nrf24l1_config {
	struct spi_dt_spec spi;
	const struct gpio_dt_spec ce;
        const struct gpio_dt_spec irq;
        const struct gpio_dt_spec csn;
};

struct nrf24l1_data {
};

int nrf24l1_config(const struct device *dev,
                                  struct rf_modem_config *config)
{
        return 0;
}

int nrf24l1_send(const struct device *dev,
                              uint8_t *data, uint32_t data_len)
{
        return 0;
}

int nrf24l1_send_async(const struct device *dev,
                                    uint8_t *data, uint32_t data_len,
                                    struct k_poll_signal *async)
{
        return 0;
}

int nrf24l1_recv(const struct device *dev, uint8_t *data,
                              uint8_t size,
                              k_timeout_t timeout, int16_t *rssi, int8_t *snr)
{
        return 0;
}

int nrf24l1_recv_async(const struct device *dev, rf_recv_cb cb,
                                 void *user_data)
{
        return 0;
}

int nrf24l1_init(const struct device *dev)
{
        return 0;
}


static const struct rf_driver_api nrf24l1_api = {
	.config = nrf24l1_config,
	.send = nrf24l1_send,
	.send_async = nrf24l1_send_async,
	.recv = nrf24l1_recv,
	.recv_async = nrf24l1_recv_async,
};

#define NRF24L1_DEVICE_INIT(n)                                                                        \
	static struct nrf24l1_data dev_data_##n;                                                      \
	static const struct nrf24l1_config dev_config_##n = {                                         \
		.ce = GPIO_DT_SPEC_INST_GET(n, ce_gpios),                                              \
                .csn = GPIO_DT_SPEC_INST_GET(n,csn_gpios),                                             \
                .irq = GPIO_DT_SPEC_INST_GET(n, irq_gpios),                                            \
	};                                                                                            \
	DEVICE_DT_INST_DEFINE(n, &nrf24l1_init, NULL, &dev_data_##n, &dev_config_##n, POST_KERNEL,    \
			      CONFIG_RF_INIT_PRIORITY, &nrf24l1_api);

DT_INST_FOREACH_STATUS_OKAY(NRF24L1_DEVICE_INIT)
