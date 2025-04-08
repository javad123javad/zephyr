/*
 * Copyright (c) 2025 Zephyr Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public API for LLCC68 LoRa Transceiver
 */

#ifndef ZEPHYR_DRIVERS_LORA_LLCC68_H_
#define ZEPHYR_DRIVERS_LORA_LLCC68_H_

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LLCC68 LoRa Transceiver API
 * @defgroup llcc68_interface LLCC68 LoRa Transceiver API
 * @ingroup lora_interface
 * @{
 */

/** Maximum packet length */
#define LLCC68_MAX_PACKET_LEN 255

/** LLCC68 operational states */
enum llcc68_state {
    /** Sleep mode */
    LLCC68_STATE_SLEEP,
    /** Standby mode with RC oscillator */
    LLCC68_STATE_STANDBY_RC,
    /** Standby mode with XOSC oscillator */
    LLCC68_STATE_STANDBY_XOSC,
    /** Frequency synthesis mode */
    LLCC68_STATE_FS,
    /** Transmit mode */
    LLCC68_STATE_TX,
    /** Receive mode */
    LLCC68_STATE_RX,
    /** Channel activity detection mode */
    LLCC68_STATE_CAD
};

/** LLCC68 IRQ flags */
enum llcc68_irq_flags {
    LLCC68_IRQ_TX_DONE          = BIT(0),
    LLCC68_IRQ_RX_DONE          = BIT(1),
    LLCC68_IRQ_PREAMBLE_DETECTED = BIT(2),
    LLCC68_IRQ_SYNC_WORD_VALID  = BIT(3),
    LLCC68_IRQ_HEADER_VALID     = BIT(4),
    LLCC68_IRQ_HEADER_ERR       = BIT(5),
    LLCC68_IRQ_CRC_ERR          = BIT(6),
    LLCC68_IRQ_CAD_DONE         = BIT(7),
    LLCC68_IRQ_CAD_DETECTED     = BIT(8),
    LLCC68_IRQ_TIMEOUT          = BIT(9)
};

/** LLCC68 LoRa bandwidth options */
enum llcc68_lora_bw {
    /** 125 kHz bandwidth */
    LLCC68_LORA_BW_125_KHZ,
    /** 250 kHz bandwidth */
    LLCC68_LORA_BW_250_KHZ,
    /** 500 kHz bandwidth */
    LLCC68_LORA_BW_500_KHZ
};

/** LLCC68 LoRa spreading factor options */
enum llcc68_lora_sf {
    /** Spreading factor 5 */
    LLCC68_LORA_SF_5 = 5,
    /** Spreading factor 6 */
    LLCC68_LORA_SF_6,
    /** Spreading factor 7 */
    LLCC68_LORA_SF_7,
    /** Spreading factor 8 */
    LLCC68_LORA_SF_8,
    /** Spreading factor 9 */
    LLCC68_LORA_SF_9,
    /** Spreading factor 10 */
    LLCC68_LORA_SF_10,
    /** Spreading factor 11 */
    LLCC68_LORA_SF_11,
    /** Spreading factor 12 */
    LLCC68_LORA_SF_12
};

/** LLCC68 LoRa coding rate options */
enum llcc68_lora_cr {
    /** Coding rate 4/5 */
    LLCC68_LORA_CR_4_5 = 1,
    /** Coding rate 4/6 */
    LLCC68_LORA_CR_4_6,
    /** Coding rate 4/7 */
    LLCC68_LORA_CR_4_7,
    /** Coding rate 4/8 */
    LLCC68_LORA_CR_4_8
};

/** LLCC68 LoRa modem configuration */
struct llcc68_modem_config {
    /** Frequency in Hz */
    uint32_t frequency;
    /** Bandwidth */
    enum llcc68_lora_bw bandwidth;
    /** Spreading factor */
    enum llcc68_lora_sf datarate;
    /** Coding rate */
    enum llcc68_lora_cr coding_rate;
    /** Preamble length */
    uint16_t preamble_len;
    /** Enable CRC */
    bool crc_enable;
    /** Enable implicit header */
    bool implicit_header;
    /** Implicit payload length (only used with implicit header) */
    uint8_t implicit_len;
    /** TX output power in dBm */
    int8_t tx_power;
};

/** LLCC68 event handler function signature */
typedef void (*llcc68_evt_handler)(const struct device *dev, uint16_t events);

/**
 * @brief Get LLCC68 device instance
 *
 * @return Pointer to LLCC68 device instance
 */
const struct device *llcc68_get_device(void);

/**
 * @brief Set LLCC68 operational state
 *
 * @param dev Pointer to LLCC68 device instance
 * @param state Desired operational state
 * @return 0 on success, negative errno code on failure
 */
int llcc68_set_state(const struct device *dev, enum llcc68_state state);

/**
 * @brief Set LLCC68 frequency
 *
 * @param dev Pointer to LLCC68 device instance
 * @param frequency Frequency in Hz
 * @return 0 on success, negative errno code on failure
 */
int llcc68_set_frequency(const struct device *dev, uint32_t frequency);

/**
 * @brief Configure LLCC68 LoRa modem parameters
 *
 * @param dev Pointer to LLCC68 device instance
 * @param config Pointer to modem configuration
 * @return 0 on success, negative errno code on failure
 */
int llcc68_set_lora_config(const struct device *dev, 
                          const struct llcc68_modem_config *config);

/**
 * @brief Send data using LLCC68
 *
 * @param dev Pointer to LLCC68 device instance
 * @param data Pointer to data buffer
 * @param len Length of data to send
 * @return 0 on success, negative errno code on failure
 */
int llcc68_send(const struct device *dev, const uint8_t *data, uint8_t len);

/**
 * @brief Receive data using LLCC68
 *
 * This function retrieves data from the last received packet.
 *
 * @param dev Pointer to LLCC68 device instance
 * @param data Pointer to data buffer
 * @param len Pointer to variable to store received data length
 * @return 0 on success, negative errno code on failure
 */
int llcc68_receive(const struct device *dev, uint8_t *data, uint8_t *len);

/**
 * @brief Set LLCC68 to receive mode
 *
 * @param dev Pointer to LLCC68 device instance
 * @param timeout_ms Receive timeout in milliseconds (0 for continuous mode)
 * @return 0 on success, negative errno code on failure
 */
int llcc68_set_rx(const struct device *dev, uint32_t timeout_ms);

/**
 * @brief Set LLCC68 to channel activity detection mode
 *
 * @param dev Pointer to LLCC68 device instance
 * @return 0 on success, negative errno code on failure
 */
int llcc68_set_cad(const struct device *dev);

/**
 * @brief Set LLCC68 event handler
 *
 * @param dev Pointer to LLCC68 device instance
 * @param handler Event handler function
 * @return 0 on success, negative errno code on failure
 */
int llcc68_set_event_handler(const struct device *dev, llcc68_evt_handler handler);

/**
 * @brief Get RSSI of last received packet
 *
 * @param dev Pointer to LLCC68 device instance
 * @param rssi Pointer to variable to store RSSI value
 * @return 0 on success, negative errno code on failure
 */
int llcc68_get_rssi(const struct device *dev, int16_t *rssi);

/**
 * @brief Get SNR of last received packet
 *
 * @param dev Pointer to LLCC68 device instance
 * @param snr Pointer to variable to store SNR value
 * @return 0 on success, negative errno code on failure
 */
int llcc68_get_snr(const struct device *dev, int8_t *snr);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_LORA_LLCC68_H_ */
