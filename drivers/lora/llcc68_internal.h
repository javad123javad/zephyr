/*
 * Copyright (c) 2025 Zephyr Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Internal definitions for LLCC68 LoRa Transceiver
 */

#ifndef ZEPHYR_DRIVERS_LORA_LLCC68_INTERNAL_H_
#define ZEPHYR_DRIVERS_LORA_LLCC68_INTERNAL_H_

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>

#include "llcc68.h"
#include "llcc68_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/** LLCC68 driver configuration structure */
struct llcc68_config {
    /** SPI bus specification */
    struct spi_dt_spec bus;
    /** Reset GPIO specification */
    struct gpio_dt_spec reset_gpio;
    /** DIO1 interrupt GPIO specification */
    struct gpio_dt_spec dio1_gpio;
    /** Busy GPIO specification (optional) */
    struct gpio_dt_spec busy_gpio;
};

/** LLCC68 driver data structure */
struct llcc68_data {
    /** Device state */
    enum llcc68_state state;
    
    /** Reference to the device itself for callbacks */
    const struct device *dev;
    
    /** SPI transaction lock */
    struct k_sem spi_sem;
    
    /** Interrupt handling */
    struct gpio_callback dio1_cb;
    llcc68_evt_handler evt_handler;
    
    /** Packet buffer */
    uint8_t rx_buffer[LLCC68_MAX_PACKET_LEN];
    uint8_t rx_len;
    
    /** Last packet statistics */
    int16_t rssi;
    int8_t snr;
    
    /** Configuration parameters */
    struct llcc68_modem_config modem_config;
};

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
                    const uint8_t *data, uint8_t len);

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
                   uint8_t *data, uint8_t len);

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
                    const uint8_t *data, uint8_t len);

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
                   uint8_t *data, uint8_t len);

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
                       const uint8_t *data, uint8_t len);

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
                      uint8_t *data, uint8_t len);

/**
 * @brief Reset LLCC68
 *
 * @param dev Pointer to LLCC68 device instance
 * @return 0 on success, negative errno code on failure
 */
int llcc68_reset(const struct device *dev);

/**
 * @brief Set LLCC68 to sleep mode
 *
 * @param dev Pointer to LLCC68 device instance
 * @return 0 on success, negative errno code on failure
 */
int llcc68_set_sleep(const struct device *dev);

/**
 * @brief Set LLCC68 to standby mode
 *
 * @param dev Pointer to LLCC68 device instance
 * @param xosc Use XOSC (1) or RC (0) oscillator
 * @return 0 on success, negative errno code on failure
 */
int llcc68_set_standby(const struct device *dev, uint8_t xosc);

/**
 * @brief Set LLCC68 to FS mode
 *
 * @param dev Pointer to LLCC68 device instance
 * @return 0 on success, negative errno code on failure
 */
int llcc68_set_fs(const struct device *dev);

/**
 * @brief Set LLCC68 to TX mode
 *
 * @param dev Pointer to LLCC68 device instance
 * @param timeout_ms Timeout in milliseconds (0 for no timeout)
 * @return 0 on success, negative errno code on failure
 */
int llcc68_set_tx(const struct device *dev, uint32_t timeout_ms);

/**
 * @brief Read LLCC68 IRQ status
 *
 * @param dev Pointer to LLCC68 device instance
 * @param irq_status Pointer to variable to store IRQ status
 * @return 0 on success, negative errno code on failure
 */
int llcc68_read_irq_status(const struct device *dev, uint16_t *irq_status);

/**
 * @brief Clear LLCC68 IRQ status
 *
 * @param dev Pointer to LLCC68 device instance
 * @param irq_status IRQ status to clear
 * @return 0 on success, negative errno code on failure
 */
int llcc68_clear_irq_status(const struct device *dev, uint16_t irq_status);

/**
 * @brief Set LLCC68 DIO IRQ parameters
 *
 * @param dev Pointer to LLCC68 device instance
 * @param irq_mask IRQ mask
 * @return 0 on success, negative errno code on failure
 */
int llcc68_set_dio_irq_params(const struct device *dev, uint16_t irq_mask);

/**
 * @brief Set LLCC68 RF frequency
 *
 * @param dev Pointer to LLCC68 device instance
 * @param freq_hz Frequency in Hz
 * @return 0 on success, negative errno code on failure
 */
int llcc68_set_rf_frequency(const struct device *dev, uint32_t freq_hz);

/**
 * @brief Set LLCC68 packet type
 *
 * @param dev Pointer to LLCC68 device instance
 * @param packet_type Packet type (LLCC68_PACKET_TYPE_LORA)
 * @return 0 on success, negative errno code on failure
 */
int llcc68_set_packet_type(const struct device *dev, uint8_t packet_type);

/**
 * @brief Set LLCC68 TX parameters
 *
 * @param dev Pointer to LLCC68 device instance
 * @param power TX power in dBm
 * @param ramp_time PA ramp time
 * @return 0 on success, negative errno code on failure
 */
int llcc68_set_tx_params(const struct device *dev, int8_t power, uint8_t ramp_time);

/**
 * @brief Set LLCC68 modulation parameters for LoRa
 *
 * @param dev Pointer to LLCC68 device instance
 * @param sf Spreading factor
 * @param bw Bandwidth
 * @param cr Coding rate
 * @param ldro Low data rate optimization (0 or 1)
 * @return 0 on success, negative errno code on failure
 */
int llcc68_set_lora_mod_params(const struct device *dev, uint8_t sf, 
                              uint8_t bw, uint8_t cr, uint8_t ldro);

/**
 * @brief Set LLCC68 packet parameters for LoRa
 *
 * @param dev Pointer to LLCC68 device instance
 * @param preamble_len Preamble length
 * @param implicit_header Enable implicit header (0 or 1)
 * @param payload_len Payload length (for implicit header)
 * @param crc_on Enable CRC (0 or 1)
 * @param invert_iq Invert IQ (0 or 1)
 * @return 0 on success, negative errno code on failure
 */
int llcc68_set_lora_packet_params(const struct device *dev, uint16_t preamble_len,
                                 uint8_t implicit_header, uint8_t payload_len,
                                 uint8_t crc_on, uint8_t invert_iq);

/**
 * @brief Set LLCC68 buffer base addresses
 *
 * @param dev Pointer to LLCC68 device instance
 * @param tx_base_addr TX buffer base address
 * @param rx_base_addr RX buffer base address
 * @return 0 on success, negative errno code on failure
 */
int llcc68_set_buffer_base_addr(const struct device *dev, 
                               uint8_t tx_base_addr, uint8_t rx_base_addr);

/**
 * @brief Handle RX done interrupt
 *
 * @param dev Pointer to LLCC68 device instance
 * @return 0 on success, negative errno code on failure
 */
int llcc68_handle_rx_done(const struct device *dev);

/**
 * @brief Wait for LLCC68 to be ready (not busy)
 *
 * @param dev Pointer to LLCC68 device instance
 * @param timeout_ms Timeout in milliseconds
 * @return 0 on success, negative errno code on failure
 */
int llcc68_wait_ready(const struct device *dev, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_LORA_LLCC68_INTERNAL_H_ */
