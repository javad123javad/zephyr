/*
 * Copyright (c) 2025 Zephyr Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief SPI communication implementation for LLCC68 LoRa Transceiver
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(llcc68, CONFIG_LLCC68_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/byteorder.h>

#include "llcc68.h"
#include "llcc68_internal.h"
#include "llcc68_regs.h"

/**
 * @brief Wait for LLCC68 to be ready (not busy)
 *
 * @param dev Pointer to LLCC68 device instance
 * @param timeout_ms Timeout in milliseconds
 * @return 0 on success, negative errno code on failure
 */
int llcc68_wait_ready(const struct device *dev, uint32_t timeout_ms)
{
    const struct llcc68_config *config = dev->config;
    int ret;
    int64_t start_time, elapsed_time;
    gpio_port_value_t busy_val;

    /* If busy pin is not configured, just delay a bit and return */
    if (!config->busy_gpio.port) {
        k_sleep(K_MSEC(1));
        return 0;
    }

    start_time = k_uptime_get();

    do {
        ret = gpio_port_get_raw(config->busy_gpio.port, &busy_val);
        if (ret < 0) {
            LOG_ERR("Failed to read busy pin: %d", ret);
            return ret;
        }

        if (!(busy_val & BIT(config->busy_gpio.pin))) {
            return 0;
        }

        k_sleep(K_MSEC(1));
        elapsed_time = k_uptime_get() - start_time;
    } while (elapsed_time < timeout_ms);

    LOG_ERR("Timeout waiting for device to be ready");
    return -ETIMEDOUT;
}

/**
 * @brief Execute SPI transaction with LLCC68
 *
 * @param dev Pointer to LLCC68 device instance
 * @param cmd Command opcode
 * @param tx_data Pointer to transmit data buffer
 * @param tx_len Length of transmit data
 * @param rx_data Pointer to receive data buffer
 * @param rx_len Length of receive data
 * @return 0 on success, negative errno code on failure
 */
static int llcc68_spi_transceive(const struct device *dev, uint8_t cmd,
                               const uint8_t *tx_data, size_t tx_len,
                               uint8_t *rx_data, size_t rx_len)
{
    const struct llcc68_config *config = dev->config;
    struct llcc68_data *data = dev->data;
    int ret;

    /* Acquire SPI bus lock */
    k_sem_take(&data->spi_sem, K_FOREVER);

    /* Wait for device to be ready */
    ret = llcc68_wait_ready(dev, LLCC68_TIMEOUT_READY);
    if (ret < 0) {
        k_sem_give(&data->spi_sem);
        return ret;
    }

    /* Setup SPI transaction */
    struct spi_buf tx_buf[2] = {
        {
            .buf = &cmd,
            .len = 1
        },
        {
            .buf = (void *)tx_data,
            .len = tx_len
        }
    };

    struct spi_buf rx_buf[2] = {
        {
            .buf = NULL,
            .len = 1
        },
        {
            .buf = rx_data,
            .len = rx_len
        }
    };

    struct spi_buf_set tx_bufs = {
        .buffers = tx_buf,
        .count = tx_len ? 2 : 1
    };

    struct spi_buf_set rx_bufs = {
        .buffers = rx_buf,
        .count = rx_len ? 2 : 1
    };

    /* Execute SPI transaction */
    ret = spi_transceive_dt(&config->bus, &tx_bufs, &rx_bufs);
    if (ret < 0) {
        LOG_ERR("SPI transaction failed: %d", ret);
    }

    /* Release SPI bus lock */
    k_sem_give(&data->spi_sem);

    return ret;
}

/**
 * @brief Write command to LLCC68
 *
 * @param dev Pointer to LLCC68 device instance
 * @param cmd Command opcode
 * @param data Pointer to data buffer
 * @param len Length of data
 * @return 0 on success, negative errno code on failure
 */
int llcc68_cmd_write(const struct device *dev, uint8_t cmd,
                    const uint8_t *data, uint8_t len)
{
    return llcc68_spi_transceive(dev, cmd, data, len, NULL, 0);
}

/**
 * @brief Read command from LLCC68
 *
 * @param dev Pointer to LLCC68 device instance
 * @param cmd Command opcode
 * @param data Pointer to data buffer
 * @param len Length of data to read
 * @return 0 on success, negative errno code on failure
 */
int llcc68_cmd_read(const struct device *dev, uint8_t cmd,
                   uint8_t *data, uint8_t len)
{
    return llcc68_spi_transceive(dev, cmd, NULL, 0, data, len);
}

/**
 * @brief Write to LLCC68 register
 *
 * @param dev Pointer to LLCC68 device instance
 * @param addr Register address
 * @param data Pointer to data buffer
 * @param len Length of data
 * @return 0 on success, negative errno code on failure
 */
int llcc68_reg_write(const struct device *dev, uint16_t addr,
                    const uint8_t *data, uint8_t len)
{
    uint8_t buf[2];
    int ret;

    buf[0] = (addr >> 8) & 0xFF;
    buf[1] = addr & 0xFF;

    /* First send register address */
    ret = llcc68_cmd_write(dev, LLCC68_CMD_WRITE_REGISTER, buf, 2);
    if (ret < 0) {
        return ret;
    }

    /* Then send data */
    return llcc68_spi_transceive(dev, 0, data, len, NULL, 0);
}

/**
 * @brief Read from LLCC68 register
 *
 * @param dev Pointer to LLCC68 device instance
 * @param addr Register address
 * @param data Pointer to data buffer
 * @param len Length of data to read
 * @return 0 on success, negative errno code on failure
 */
int llcc68_reg_read(const struct device *dev, uint16_t addr,
                   uint8_t *data, uint8_t len)
{
    uint8_t buf[2];
    int ret;

    buf[0] = (addr >> 8) & 0xFF;
    buf[1] = addr & 0xFF;

    /* First send register address */
    ret = llcc68_cmd_write(dev, LLCC68_CMD_READ_REGISTER, buf, 2);
    if (ret < 0) {
        return ret;
    }

    /* Then read data */
    return llcc68_spi_transceive(dev, 0, NULL, 0, data, len);
}

/**
 * @brief Write to LLCC68 buffer
 *
 * @param dev Pointer to LLCC68 device instance
 * @param offset Buffer offset
 * @param data Pointer to data buffer
 * @param len Length of data
 * @return 0 on success, negative errno code on failure
 */
int llcc68_write_buffer(const struct device *dev, uint8_t offset,
                       const uint8_t *data, uint8_t len)
{
    int ret;

    /* First send buffer offset */
    ret = llcc68_cmd_write(dev, LLCC68_CMD_WRITE_BUFFER, &offset, 1);
    if (ret < 0) {
        return ret;
    }

    /* Then send data */
    return llcc68_spi_transceive(dev, 0, data, len, NULL, 0);
}

/**
 * @brief Read from LLCC68 buffer
 *
 * @param dev Pointer to LLCC68 device instance
 * @param offset Buffer offset
 * @param data Pointer to data buffer
 * @param len Length of data to read
 * @return 0 on success, negative errno code on failure
 */
int llcc68_read_buffer(const struct device *dev, uint8_t offset,
                      uint8_t *data, uint8_t len)
{
    int ret;

    /* First send buffer offset */
    ret = llcc68_cmd_write(dev, LLCC68_CMD_READ_BUFFER, &offset, 1);
    if (ret < 0) {
        return ret;
    }

    /* Then read data */
    return llcc68_spi_transceive(dev, 0, NULL, 0, data, len);
}

/**
 * @brief Reset LLCC68
 *
 * @param dev Pointer to LLCC68 device instance
 * @return 0 on success, negative errno code on failure
 */
int llcc68_reset(const struct device *dev)
{
    const struct llcc68_config *config = dev->config;
    int ret;

    /* Configure reset pin as output */
    ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure reset pin: %d", ret);
        return ret;
    }

    /* Perform reset sequence */
    gpio_pin_set_dt(&config->reset_gpio, 1);
    k_sleep(K_MSEC(1));
    gpio_pin_set_dt(&config->reset_gpio, 0);
    k_sleep(K_MSEC(5));

    /* Wait for device to be ready after reset */
    return llcc68_wait_ready(dev, LLCC68_TIMEOUT_RESET);
}

/**
 * @brief Read LLCC68 IRQ status
 *
 * @param dev Pointer to LLCC68 device instance
 * @param irq_status Pointer to variable to store IRQ status
 * @return 0 on success, negative errno code on failure
 */
int llcc68_read_irq_status(const struct device *dev, uint16_t *irq_status)
{
    uint8_t buf[2];
    int ret;

    ret = llcc68_cmd_read(dev, LLCC68_CMD_GET_IRQ_STATUS, buf, 2);
    if (ret < 0) {
        return ret;
    }

    *irq_status = (buf[0] << 8) | buf[1];
    return 0;
}

/**
 * @brief Clear LLCC68 IRQ status
 *
 * @param dev Pointer to LLCC68 device instance
 * @param irq_status IRQ status to clear
 * @return 0 on success, negative errno code on failure
 */
int llcc68_clear_irq_status(const struct device *dev, uint16_t irq_status)
{
    uint8_t buf[2];

    buf[0] = (irq_status >> 8) & 0xFF;
    buf[1] = irq_status & 0xFF;

    return llcc68_cmd_write(dev, LLCC68_CMD_CLEAR_IRQ_STATUS, buf, 2);
}

/**
 * @brief Set LLCC68 DIO IRQ parameters
 *
 * @param dev Pointer to LLCC68 device instance
 * @param irq_mask IRQ mask
 * @return 0 on success, negative errno code on failure
 */
int llcc68_set_dio_irq_params(const struct device *dev, uint16_t irq_mask)
{
    uint8_t buf[8];

    /* Set IRQ mask for DIO1 only, disable other DIOs */
    buf[0] = (irq_mask >> 8) & 0xFF;
    buf[1] = irq_mask & 0xFF;
    buf[2] = (irq_mask >> 8) & 0xFF;  /* DIO1 mask */
    buf[3] = irq_mask & 0xFF;
    buf[4] = 0;  /* DIO2 mask */
    buf[5] = 0;
    buf[6] = 0;  /* DIO3 mask */
    buf[7] = 0;

    return llcc68_cmd_write(dev, LLCC68_CMD_SET_DIO_IRQ_PARAMS, buf, 8);
}
