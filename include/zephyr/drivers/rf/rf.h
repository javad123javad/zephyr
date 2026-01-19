/*
 * Copyright (c) 2026  Javad Rahimipetroudi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @ingroup rf_interface
 * @brief Main header file for generic RF driver API.
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_RF_H_
#define ZEPHYR_INCLUDE_DRIVERS_RF_H_

/**
 * @brief Interfaces for RF transceivers.
 * @defgroup rf_interface RF
 * @since 4.3
 * @version 0.1.0
 * @ingroup io_interfaces
 * @{
 */

#include "zephyr/toolchain.h"
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief RF bitrate
 *
 * This enumeration defines bitrates supported by the RF module.
 *
 * The bandwidth determines how much spectrum is used to transmit data. Wider bandwidths enable
 * higher data rates but typically reduce sensitivity and range.
 */
enum rf_signal_bitrate {
	BR_300_BPS = 0,	/**< 300 bps */
	BR_1200_BPS,	/**< 1200 bps */
	BR_2400_BPS,	/**< 2400 bps */
        BR_4800_BPS,    /**< 4800 bps */
        BR_9600_BPS,    /**< 9600 bps */
        BR_19200_BPS,   /**< 19200 bps */
        BR_38400_BPS,   /**< 38400 bps */
        BR_57600_BPS,   /**< 57600 bps */
        BR_115200_BPS,  /**< 115200 bps */
        BR_250000_BPS,  /**< 250000 bps */
        BR_1000000_BPS, /**< 1 Mbps */
        BR_2000000_BPS, /**< 2 Mbps */
};

/**
 * @struct rf_modem_config
 * Structure containing the configuration of a RF module
 */
struct rf_modem_config {
	/** Frequency in Hz to use for transceiving */
	uint32_t frequency;

	/** The bandwidth to use for transceiving */
	enum rf_signal_bitrate bitrate;

	/** Length of the preamble */
	uint16_t preamble_len;

	/** TX-power in dBm to use for transmission */
	int8_t tx_power;

	/** Set to true for transmission, false for receiving */
	bool tx;

	/**
	 * Invert the In-Phase and Quadrature (IQ) signals. Normally this
	 * should be set to false. In advanced use-cases where a
	 * differentation is needed between "uplink" and "downlink" traffic,
	 * the IQ can be inverted to create two different channels on the
	 * same frequency
	 */
	bool iq_inverted;

	/**
	 * Sets the sync-byte to use:
	 *  - false: for using the private network sync-byte
	 *  - true:  for using the public network sync-byte
	 * The public network sync-byte is only intended for advanced usage.
	 * Normally the private network sync-byte should be used for peer
	 * to peer communications and the LoRaWAN APIs should be used for
	 * interacting with a public network.
	 */
	bool public_network;

	/** Set to true to disable the 16-bit payload CRC */
	bool packet_crc_disable;
};

/**
 * @cond INTERNAL_HIDDEN
 *
 * For internal driver use only, skip these in public documentation.
 */

/**
 * @typedef rf_recv_cb()
 * @brief Callback API for receiving data asynchronously
 *
 * @see rf_recv() for argument descriptions.
 */
typedef void (*rf_recv_cb)(const struct device *dev, uint8_t *data, uint16_t size,
			     int16_t rssi, int8_t snr, void *user_data);

/**
 * @typedef rf_api_config()
 * @brief Callback API for configuring the RF module
 *
 * @see rf_config() for argument descriptions.
 */
typedef int (*rf_api_config)(const struct device *dev,
			       struct rf_modem_config *config);

/**
 * @typedef rf_api_send()
 * @brief Callback API for sending data over the RF module
 *
 * @see rf_send() for argument descriptions.
 */
typedef int (*rf_api_send)(const struct device *dev,
			     uint8_t *data, uint8_t data_len);

/**
 * @typedef rf_api_send_async()
 * @brief Callback API for sending data asynchronously over RF
 *
 * @see rf_send_async() for argument descriptions.
 */
typedef int (*rf_api_send_async)(const struct device *dev,
				   uint8_t *data, uint32_t data_len,
				   struct k_poll_signal *async);

/**
 * @typedef rf_api_recv()
 * @brief Callback API for receiving data over RF
 *
 * @see rf_recv() for argument descriptions.
 */
typedef int (*rf_api_recv)(const struct device *dev, uint8_t *data,
			     uint8_t size);

/**
 * @typedef rf_api_recv_async()
 * @brief Callback API for receiving data asynchronously over RF
 *
 * @param dev Modem to receive data on.
 * @param cb Callback to run on receiving data.
 */
typedef int (*rf_api_recv_async)(const struct device *dev, rf_recv_cb cb,
			     void *user_data);

__subsystem struct rf_driver_api {
	rf_api_config config;
	rf_api_send send;
	rf_api_send_async send_async;
	rf_api_recv recv;
	rf_api_recv_async recv_async;
};

/** @endcond */

/**
 * @brief Configure the RF modem
 *
 * @param dev     RF device
 * @param config  Data structure containing the intended configuration for the
		  modem
 * @return 0 on success, negative on error
 */

__syscall int rf_config(const struct device *dev,
                                struct rf_modem_config *config);

static inline int z_impl_rf_config(const struct device *dev,
			      struct rf_modem_config *config)
{
	const struct rf_driver_api *api =
		(const struct rf_driver_api *)dev->api;

	return api->config(dev, config);
}

/**
 * @brief Send data over RF
 *
 * @note This blocks until transmission is complete.
 *
 * @param dev       RF device
 * @param data      Data to be sent
 * @param data_len  Length of the data to be sent
 * @return 0 on success, negative on error
 */
__syscall int rf_send(const struct device *dev,
			    uint8_t *data, uint8_t data_len);

static inline int z_impl_rf_send(const struct device *dev,
			    uint8_t *data, uint8_t data_len)
{
	const struct rf_driver_api *api =
		(const struct rf_driver_api *)dev->api;

	return api->send(dev, data, data_len);
}

/**
 * @brief Asynchronously send data over RF
 *
 * @note This returns immediately after starting transmission, and locks
 *       the RF modem until the transmission completes.
 *
 * @param dev       RF device
 * @param data      Data to be sent
 * @param data_len  Length of the data to be sent
 * @param async A pointer to a valid and ready to be signaled
 *        struct k_poll_signal. (Note: if NULL this function will not
 *        notify the end of the transmission).
 * @return 0 on success, negative on error
 */

__syscall int rf_send_async(const struct device *dev,
				  uint8_t *data, uint32_t data_len,
				  struct k_poll_signal *async);

static inline int z_impl_rf_send_async(const struct device *dev,
				  uint8_t *data, uint32_t data_len,
				  struct k_poll_signal *async)
{
	const struct rf_driver_api *api =
		(const struct rf_driver_api *)dev->api;

	return api->send_async(dev, data, data_len, async);
}

/**
 * @brief Receive data over RF
 *
 * @note This is a blocking call.
 *
 * @param dev       RF device
 * @param data      Buffer to hold received data
 * @param size      Size of the buffer to hold the received data. Max size
		    allowed is 255.
 * @param timeout   Duration to wait for a packet.
 * @param rssi      RSSI of received data
 * @param snr       SNR of received data
 * @return Length of the data received on success, negative on error
 */
__syscall int rf_recv(const struct device *dev, uint8_t *data,
			    uint8_t size);

static inline int z_impl_rf_recv(const struct device *dev, uint8_t *data,
			    uint8_t size)
{
	const struct rf_driver_api *api =
		(const struct rf_driver_api *)dev->api;

	return api->recv(dev, data, size);
}

/**
 * @brief Receive data asynchronously over RF modem
 *
 * Receive packets continuously under the configuration previously setup
 * by @ref rf_config.
 *
 * Reception is cancelled by calling this function again with @p cb = NULL.
 * This can be done within the callback handler.
 *
 * @param dev Modem to receive data on.
 * @param cb Callback to run on receiving data. If NULL, any pending
 *	     asynchronous receptions will be cancelled.
 * @param user_data User data passed to callback
 * @return 0 when reception successfully setup, negative on error
 */

__syscall int rf_recv_async(const struct device *dev, rf_recv_cb cb,
			       void *user_data);

static inline int z_impl_rf_recv_async(const struct device *dev, rf_recv_cb cb,
			       void *user_data)
{
	const struct rf_driver_api *api =
		(const struct rf_driver_api *)dev->api;

	return api->recv_async(dev, cb, user_data);
}


#ifdef __cplusplus
}
#endif

/**
 * @}
 */
#include <zephyr/syscalls/rf.h>

#endif	/* ZEPHYR_INCLUDE_DRIVERS_RF_H_ */
