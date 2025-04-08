/*
 * Copyright (c) 2025 Zephyr Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Main implementation for LLCC68 LoRa Transceiver
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(llcc68, CONFIG_LLCC68_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/lora.h>

#include "llcc68.h"
#include "llcc68_internal.h"
#include "llcc68_regs.h"

/**
 * @brief Set LLCC68 to sleep mode
 *
 * @param dev Pointer to LLCC68 device instance
 * @return 0 on success, negative errno code on failure
 */
int llcc68_set_sleep(const struct device *dev)
{
	uint8_t sleep_config = LLCC68_SLEEP_START_WARM | LLCC68_SLEEP_RTC_DISABLE;
	int ret;

	ret = llcc68_cmd_write(dev, LLCC68_CMD_SET_SLEEP, &sleep_config, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set sleep mode: %d", ret);
		return ret;
	}

	return 0;
}

/**
 * @brief Set LLCC68 to standby mode
 *
 * @param dev Pointer to LLCC68 device instance
 * @param xosc Use XOSC (1) or RC (0) oscillator
 * @return 0 on success, negative errno code on failure
 */
int llcc68_set_standby(const struct device *dev, uint8_t xosc)
{
	uint8_t standby_config = xosc ? LLCC68_STANDBY_XOSC : LLCC68_STANDBY_RC;
	int ret;

	ret = llcc68_cmd_write(dev, LLCC68_CMD_SET_STANDBY, &standby_config, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set standby mode: %d", ret);
		return ret;
	}

	return 0;
}

/**
 * @brief Set LLCC68 to FS mode
 *
 * @param dev Pointer to LLCC68 device instance
 * @return 0 on success, negative errno code on failure
 */
int llcc68_set_fs(const struct device *dev)
{
	int ret;

	ret = llcc68_cmd_write(dev, LLCC68_CMD_SET_FS, NULL, 0);
	if (ret < 0) {
		LOG_ERR("Failed to set FS mode: %d", ret);
		return ret;
	}

	return 0;
}

/**
 * @brief Set LLCC68 to TX mode
 *
 * @param dev Pointer to LLCC68 device instance
 * @param timeout_ms Timeout in milliseconds (0 for no timeout)
 * @return 0 on success, negative errno code on failure
 */
int llcc68_set_tx(const struct device *dev, uint32_t timeout_ms)
{
	uint8_t buf[3];
	int ret;

	/* Convert timeout to steps of 15.625 μs */
	uint32_t timeout_steps = timeout_ms ? (timeout_ms * 64) : 0;

	buf[0] = (timeout_steps >> 16) & 0xFF;
	buf[1] = (timeout_steps >> 8) & 0xFF;
	buf[2] = timeout_steps & 0xFF;

	ret = llcc68_cmd_write(dev, LLCC68_CMD_SET_TX, buf, 3);
	if (ret < 0) {
		LOG_ERR("Failed to set TX mode: %d", ret);
		return ret;
	}

	return 0;
}

/**
 * @brief Set LLCC68 to RX mode
 *
 * @param dev Pointer to LLCC68 device instance
 * @param timeout_ms Timeout in milliseconds (0 for continuous mode)
 * @return 0 on success, negative errno code on failure
 */
int llcc68_set_rx(const struct device *dev, uint32_t timeout_ms)
{
	uint8_t buf[3];
	int ret;

	/* Convert timeout to steps of 15.625 μs */
	uint32_t timeout_steps = timeout_ms ? (timeout_ms * 64) : 0;

	buf[0] = (timeout_steps >> 16) & 0xFF;
	buf[1] = (timeout_steps >> 8) & 0xFF;
	buf[2] = timeout_steps & 0xFF;

	ret = llcc68_cmd_write(dev, LLCC68_CMD_SET_RX, buf, 3);
	if (ret < 0) {
		LOG_ERR("Failed to set RX mode: %d", ret);
		return ret;
	}

	return 0;
}

/**
 * @brief Set LLCC68 to CAD mode
 *
 * @param dev Pointer to LLCC68 device instance
 * @return 0 on success, negative errno code on failure
 */
int llcc68_set_cad(const struct device *dev)
{
	int ret;

	ret = llcc68_cmd_write(dev, LLCC68_CMD_SET_CAD, NULL, 0);
	if (ret < 0) {
		LOG_ERR("Failed to set CAD mode: %d", ret);
		return ret;
	}

	return 0;
}

/**
 * @brief Set LLCC68 RF frequency
 *
 * @param dev Pointer to LLCC68 device instance
 * @param freq_hz Frequency in Hz
 * @return 0 on success, negative errno code on failure
 */
int llcc68_set_rf_frequency(const struct device *dev, uint32_t freq_hz)
{
	uint8_t buf[4];
	int ret;

	/* Convert frequency to steps of 32 MHz / 2^25 = 0.95367431640625 Hz */
	uint32_t freq_steps = (uint32_t)((((uint64_t)freq_hz) << 25) / 32000000);

	buf[0] = (freq_steps >> 24) & 0xFF;
	buf[1] = (freq_steps >> 16) & 0xFF;
	buf[2] = (freq_steps >> 8) & 0xFF;
	buf[3] = freq_steps & 0xFF;

	ret = llcc68_cmd_write(dev, LLCC68_CMD_SET_RF_FREQUENCY, buf, 4);
	if (ret < 0) {
		LOG_ERR("Failed to set RF frequency: %d", ret);
		return ret;
	}

	return 0;
}

/**
 * @brief Set LLCC68 packet type
 *
 * @param dev Pointer to LLCC68 device instance
 * @param packet_type Packet type (LLCC68_PACKET_TYPE_LORA)
 * @return 0 on success, negative errno code on failure
 */
int llcc68_set_packet_type(const struct device *dev, uint8_t packet_type)
{
	int ret;

	ret = llcc68_cmd_write(dev, LLCC68_CMD_SET_PACKET_TYPE, &packet_type, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set packet type: %d", ret);
		return ret;
	}

	return 0;
}

/**
 * @brief Set LLCC68 TX parameters
 *
 * @param dev Pointer to LLCC68 device instance
 * @param power TX power in dBm
 * @param ramp_time PA ramp time
 * @return 0 on success, negative errno code on failure
 */
int llcc68_set_tx_params(const struct device *dev, int8_t power, uint8_t ramp_time)
{
	uint8_t buf[2];
	int ret;

	/* Clamp power to valid range */
	if (power > 22) {
		power = 22;
	} else if (power < -9) {
		power = -9;
	}

	buf[0] = power;
	buf[1] = ramp_time;

	ret = llcc68_cmd_write(dev, LLCC68_CMD_SET_TX_PARAMS, buf, 2);
	if (ret < 0) {
		LOG_ERR("Failed to set TX parameters: %d", ret);
		return ret;
	}

	return 0;
}

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
int llcc68_set_lora_mod_params(const struct device *dev, uint8_t sf, uint8_t bw, uint8_t cr,
			       uint8_t ldro)
{
	uint8_t buf[4];
	int ret;

	buf[0] = sf;
	buf[1] = bw;
	buf[2] = cr;
	buf[3] = ldro;

	ret = llcc68_cmd_write(dev, LLCC68_CMD_SET_MODULATION_PARAMS, buf, 4);
	if (ret < 0) {
		LOG_ERR("Failed to set LoRa modulation parameters: %d", ret);
		return ret;
	}

	return 0;
}

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
				  uint8_t implicit_header, uint8_t payload_len, uint8_t crc_on,
				  uint8_t invert_iq)
{
	uint8_t buf[6];
	int ret;

	buf[0] = (preamble_len >> 8) & 0xFF;
	buf[1] = preamble_len & 0xFF;
	buf[2] = implicit_header;
	buf[3] = payload_len;
	buf[4] = crc_on;
	buf[5] = invert_iq;

	ret = llcc68_cmd_write(dev, LLCC68_CMD_SET_PACKET_PARAMS, buf, 6);
	if (ret < 0) {
		LOG_ERR("Failed to set LoRa packet parameters: %d", ret);
		return ret;
	}

	return 0;
}

/**
 * @brief Set LLCC68 buffer base addresses
 *
 * @param dev Pointer to LLCC68 device instance
 * @param tx_base_addr TX buffer base address
 * @param rx_base_addr RX buffer base address
 * @return 0 on success, negative errno code on failure
 */
int llcc68_set_buffer_base_addr(const struct device *dev, uint8_t tx_base_addr,
				uint8_t rx_base_addr)
{
	uint8_t buf[2];
	int ret;

	buf[0] = tx_base_addr;
	buf[1] = rx_base_addr;

	ret = llcc68_cmd_write(dev, LLCC68_CMD_SET_BUFFER_BASE_ADDRESS, buf, 2);
	if (ret < 0) {
		LOG_ERR("Failed to set buffer base addresses: %d", ret);
		return ret;
	}

	return 0;
}

/**
 * @brief Handle RX done interrupt
 *
 * @param dev Pointer to LLCC68 device instance
 * @return 0 on success, negative errno code on failure
 */
int llcc68_handle_rx_done(const struct device *dev)
{
	struct llcc68_data *data = dev->data;
	uint8_t buf[3];
	uint8_t offset, len;
	int ret;

	/* Get RX buffer status */
	ret = llcc68_cmd_read(dev, LLCC68_CMD_GET_RX_BUFFER_STATUS, buf, 2);
	if (ret < 0) {
		LOG_ERR("Failed to get RX buffer status: %d", ret);
		return ret;
	}

	len = buf[0];
	offset = buf[1];

	if (len > LLCC68_MAX_PACKET_LEN) {
		LOG_ERR("Received packet too large: %d", len);
		return -EINVAL;
	}

	/* Read packet data */
	ret = llcc68_read_buffer(dev, offset, data->rx_buffer, len);
	if (ret < 0) {
		LOG_ERR("Failed to read RX buffer: %d", ret);
		return ret;
	}

	data->rx_len = len;

	/* Get packet status */
	ret = llcc68_cmd_read(dev, LLCC68_CMD_GET_PACKET_STATUS, buf, 3);
	if (ret < 0) {
		LOG_ERR("Failed to get packet status: %d", ret);
		return ret;
	}

	/* For LoRa, buf[0] is RSSI, buf[1] is SNR */
	data->rssi = -buf[0] / 2;
	data->snr = buf[1] / 4;

	LOG_DBG("Received %d bytes, RSSI: %d dBm, SNR: %d dB", data->rx_len, data->rssi, data->snr);

	return 0;
}

/**
 * @brief DIO1 interrupt callback
 *
 * @param port GPIO port
 * @param cb GPIO callback structure
 * @param pins GPIO pins that triggered the interrupt
 */
static void llcc68_dio1_callback(const struct device *port, struct gpio_callback *cb,
				 gpio_port_pins_t pins)
{
	struct llcc68_data *data = CONTAINER_OF(cb, struct llcc68_data, dio1_cb);
	const struct device *dev = data->dev;
	uint16_t irq_status;
	int ret;

	/* Read interrupt status */
	ret = llcc68_read_irq_status(dev, &irq_status);
	if (ret < 0) {
		LOG_ERR("Failed to read IRQ status: %d", ret);
		return;
	}

	/* Clear interrupts */
	ret = llcc68_clear_irq_status(dev, irq_status);
	if (ret < 0) {
		LOG_ERR("Failed to clear IRQ status: %d", ret);
	}

	/* Process interrupts */
	if (irq_status & LLCC68_IRQ_TX_DONE) {
		LOG_DBG("TX done");
	}

	if (irq_status & LLCC68_IRQ_RX_DONE) {
		LOG_DBG("RX done");
		ret = llcc68_handle_rx_done(dev);
		if (ret < 0) {
			LOG_ERR("Failed to handle RX done: %d", ret);
		}
	}

	if (irq_status & LLCC68_IRQ_PREAMBLE_DETECTED) {
		LOG_DBG("Preamble detected");
	}

	if (irq_status & LLCC68_IRQ_SYNC_WORD_VALID) {
		LOG_DBG("Sync word valid");
	}

	if (irq_status & LLCC68_IRQ_HEADER_VALID) {
		LOG_DBG("Header valid");
	}

	if (irq_status & LLCC68_IRQ_HEADER_ERROR) {
		LOG_DBG("Header error");
	}

	if (irq_status & LLCC68_IRQ_CRC_ERROR) {
		LOG_DBG("CRC error");
	}

	if (irq_status & LLCC68_IRQ_CAD_DONE) {
		LOG_DBG("CAD done");
	}

	if (irq_status & LLCC68_IRQ_CAD_DETECTED) {
		LOG_DBG("CAD detected");
	}

	if (irq_status & LLCC68_IRQ_TIMEOUT) {
		LOG_DBG("Timeout");
	}

	/* Call user callback if registered */
	if (data->evt_handler) {
		data->evt_handler(dev, irq_status);
	}
}

/**
 * @brief Set LLCC68 operational state
 *
 * @param dev Pointer to LLCC68 device instance
 * @param state Desired operational state
 * @return 0 on success, negative errno code on failure
 */
int llcc68_set_state(const struct device *dev, enum llcc68_state state)
{
	struct llcc68_data *data = dev->data;
	int ret = 0;

	switch (state) {
	case LLCC68_STATE_SLEEP:
		ret = llcc68_set_sleep(dev);
		break;
	case LLCC68_STATE_STANDBY_RC:
		ret = llcc68_set_standby(dev, 0); /* RC oscillator */
		break;
	case LLCC68_STATE_STANDBY_XOSC:
		ret = llcc68_set_standby(dev, 1); /* XOSC oscillator */
		break;
	case LLCC68_STATE_FS:
		ret = llcc68_set_fs(dev);
		break;
	case LLCC68_STATE_TX:
		ret = llcc68_set_tx(dev, 0); /* No timeout */
		break;
	case LLCC68_STATE_RX:
		ret = llcc68_set_rx(dev, 0); /* Continuous mode */
		break;
	case LLCC68_STATE_CAD:
		ret = llcc68_set_cad(dev);
		break;
	default:
		return -EINVAL;
	}

	if (ret == 0) {
		data->state = state;
	}

	return ret;
}

/**
 * @brief Set LLCC68 frequency
 *
 * @param dev Pointer to LLCC68 device instance
 * @param frequency Frequency in Hz
 * @return 0 on success, negative errno code on failure
 */
int llcc68_set_frequency(const struct device *dev, uint32_t frequency)
{
	struct llcc68_data *data = dev->data;
	int ret;

	/* Store frequency in configuration */
	data->modem_config.frequency = frequency;

	/* Set device to standby mode before changing frequency */
	ret = llcc68_set_state(dev, LLCC68_STATE_STANDBY_RC);
	if (ret < 0) {
		return ret;
	}

	/* Set RF frequency */
	return llcc68_set_rf_frequency(dev, frequency);
}

/**
 * @brief Configure LLCC68 LoRa modem parameters
 *
 * @param dev Pointer to LLCC68 device instance
 * @param config Pointer to modem configuration
 * @return 0 on success, negative errno code on failure
 */
int llcc68_set_lora_config(const struct device *dev, const struct lora_modem_config *config)
{
	struct llcc68_data *data = dev->data;
	uint8_t sf, bw, cr, ldro;
	int ret;

	/* Copy configuration */
	memcpy(&data->modem_config, config, sizeof(struct lora_modem_config));

	/* Set device to standby mode before changing configuration */
	ret = llcc68_set_state(dev, LLCC68_STATE_STANDBY_RC);
	if (ret < 0) {
		return ret;
	}

	/* Set packet type to LoRa */
	ret = llcc68_set_packet_type(dev, LLCC68_PACKET_TYPE_LORA);
	if (ret < 0) {
		return ret;
	}

	/* Set RF frequency */
	ret = llcc68_set_rf_frequency(dev, config->frequency);
	if (ret < 0) {
		return ret;
	}

	/* Set buffer base addresses */
	ret = llcc68_set_buffer_base_addr(dev, 0, 0);
	if (ret < 0) {
		return ret;
	}

	/* Map configuration to device parameters */
	sf = config->datarate;

	switch (config->bandwidth) {
	case LLCC68_LORA_BW_125_KHZ:
		bw = LLCC68_LORA_BW_125;
		break;
	case LLCC68_LORA_BW_250_KHZ:
		bw = LLCC68_LORA_BW_250;
		break;
	case LLCC68_LORA_BW_500_KHZ:
		bw = LLCC68_LORA_BW_500;
		break;
	default:
		return -EINVAL;
	}

	cr = config->coding_rate;

	/* Calculate low data rate optimization */
	ldro = ((sf >= 11) && (bw == LLCC68_LORA_BW_125)) ? 1 : 0;

	/* Set modulation parameters */
	ret = llcc68_set_lora_mod_params(dev, sf, bw, cr, ldro);
	if (ret < 0) {
		return ret;
	}

	/* Set packet parameters */
	ret = llcc68_set_lora_packet_params(dev, config->preamble_len, config->implicit_header,
					    config->implicit_len, config->crc_enable ? 1 : 0, 0);
	if (ret < 0) {
		return ret;
	}

	/* Set TX parameters */
	ret = llcc68_set_tx_params(dev, config->tx_power, LLCC68_TX_RAMP_200U);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

/**
 * @brief Send data using LLCC68
 *
 * @param dev Pointer to LLCC68 device instance
 * @param data Pointer to data buffer
 * @param len Length of data to send
 * @return 0 on success, negative errno code on failure
 */
int llcc68_send(const struct device *dev, uint8_t *data, uint32_t len)
{
	int ret;

	if (len > LLCC68_MAX_PACKET_LEN) {
		return -EINVAL;
	}

	/* Set device to standby mode before sending */
	ret = llcc68_set_state(dev, LLCC68_STATE_STANDBY_RC);
	if (ret < 0) {
		return ret;
	}

	/* Write data to buffer */
	ret = llcc68_write_buffer(dev, 0, data, len);
	if (ret < 0) {
		return ret;
	}

	/* Set TX mode to start transmission */
	return llcc68_set_state(dev, LLCC68_STATE_TX);
}

/**
 * @brief Receive data using LLCC68
 *
 * @param dev Pointer to LLCC68 device instance
 * @param data Pointer to data buffer
 * @param len Pointer to variable to store received data length
 * @return 0 on success, negative errno code on failure
 */
int llcc68_receive(const struct device *dev, uint8_t *data, uint8_t *len)
{
	struct llcc68_data *data_struct = dev->data;

	if (data_struct->rx_len == 0) {
		*len = 0;
		return -ENODATA;
	}

	/* Copy data from internal buffer */
	memcpy(data, data_struct->rx_buffer, data_struct->rx_len);
	*len = data_struct->rx_len;

	/* Clear internal buffer */
	data_struct->rx_len = 0;

	return 0;
}

/**
 * @brief Set LLCC68 event handler
 *
 * @param dev Pointer to LLCC68 device instance
 * @param handler Event handler function
 * @return 0 on success, negative errno code on failure
 */
int llcc68_set_event_handler(const struct device *dev, llcc68_api_evt_handler handler)
{
	struct llcc68_data *data = dev->data;

	data->evt_handler = handler;

	return 0;
}

/**
 * @brief Get RSSI of last received packet
 *
 * @param dev Pointer to LLCC68 device instance
 * @param rssi Pointer to variable to store RSSI value
 * @return 0 on success, negative errno code on failure
 */
int llcc68_get_rssi(const struct device *dev, int16_t *rssi)
{
	struct llcc68_data *data = dev->data;

	*rssi = data->rssi;

	return 0;
}

/**
 * @brief Get SNR of last received packet
 *
 * @param dev Pointer to LLCC68 device instance
 * @param snr Pointer to variable to store SNR value
 * @return 0 on success, negative errno code on failure
 */
int llcc68_get_snr(const struct device *dev, int8_t *snr)
{
	struct llcc68_data *data = dev->data;

	*snr = data->snr;

	return 0;
}

/**
 * @brief Initialize LLCC68 device
 *
 * @param dev Pointer to LLCC68 device instance
 * @return 0 on success, negative errno code on failure
 */
static int llcc68_init(const struct device *dev)
{
	const struct llcc68_config *config = dev->config;
	struct llcc68_data *data = dev->data;
	int ret;

	/* Store device reference for callbacks */
	data->dev = dev;

	/* Initialize SPI semaphore */
	k_sem_init(&data->spi_sem, 1, 1);

	/* Check if SPI bus is ready */
	if (!spi_is_ready_dt(&config->bus)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	/* Configure reset GPIO */
	if (!gpio_is_ready_dt(&config->reset_gpio)) {
		LOG_ERR("Reset GPIO not ready");
		return -ENODEV;
	}

	/* Configure DIO1 interrupt GPIO */
	if (!gpio_is_ready_dt(&config->dio1_gpio)) {
		LOG_ERR("DIO1 GPIO not ready");
		return -ENODEV;
	}

	/* Configure busy GPIO if available */
	if (config->busy_gpio.port && !gpio_is_ready_dt(&config->busy_gpio)) {
		LOG_ERR("Busy GPIO not ready");
		return -ENODEV;
	}

	/* Reset the device */
	ret = llcc68_reset(dev);
	if (ret < 0) {
		LOG_ERR("Failed to reset device: %d", ret);
		return ret;
	}

	/* Set to standby mode */
	ret = llcc68_set_standby(dev, 0);
	if (ret < 0) {
		LOG_ERR("Failed to set standby mode: %d", ret);
		return ret;
	}

	/* Set packet type to LoRa */
	ret = llcc68_set_packet_type(dev, LLCC68_PACKET_TYPE_LORA);
	if (ret < 0) {
		LOG_ERR("Failed to set packet type: %d", ret);
		return ret;
	}

	/* Set buffer base addresses */
	ret = llcc68_set_buffer_base_addr(dev, 0, 0);
	if (ret < 0) {
		LOG_ERR("Failed to set buffer base addresses: %d", ret);
		return ret;
	}

	/* Configure DIO1 interrupt */
	ret = gpio_pin_configure_dt(&config->dio1_gpio, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure DIO1 pin: %d", ret);
		return ret;
	}

	/* Configure DIO1 interrupt callback */
	gpio_init_callback(&data->dio1_cb, llcc68_dio1_callback, BIT(config->dio1_gpio.pin));

	ret = gpio_add_callback(config->dio1_gpio.port, &data->dio1_cb);
	if (ret < 0) {
		LOG_ERR("Failed to add DIO1 callback: %d", ret);
		return ret;
	}

	/* Enable DIO1 interrupt */
	ret = gpio_pin_interrupt_configure_dt(&config->dio1_gpio, GPIO_INT_EDGE_RISING);
	if (ret < 0) {
		LOG_ERR("Failed to configure DIO1 interrupt: %d", ret);
		return ret;
	}

	/* Configure IRQ parameters */
	ret = llcc68_set_dio_irq_params(dev, LLCC68_IRQ_ALL);
	if (ret < 0) {
		LOG_ERR("Failed to set DIO IRQ parameters: %d", ret);
		return ret;
	}

	/* Set default state */
	data->state = LLCC68_STATE_STANDBY_RC;

	LOG_INF("LLCC68 initialized successfully");

	return 0;
}

/**
 * @brief Get LLCC68 device instance
 *
 * @return Pointer to LLCC68 device instance
 */
const struct device *llcc68_get_device(void)
{
	return DEVICE_DT_INST_GET(0);
}

/* Define driver API */
static  DEVICE_API(lora,lora_driver_api)  = {
	.config = llcc68_set_lora_config,
	.send = llcc68_send,
	.receive = llcc68_receive,
};

/* Define driver instance */
#define LLCC68_DEFINE(inst)                                                                        \
	static struct llcc68_data llcc68_data_##inst = {                                           \
		.state = LLCC68_STATE_SLEEP,                                                       \
	};                                                                                         \
                                                                                                   \
	static const struct llcc68_config llcc68_config_##inst = {                                 \
		.bus = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8), 0),                             \
		.reset_gpio = GPIO_DT_SPEC_INST_GET(inst, reset_gpios),                            \
		.dio1_gpio = GPIO_DT_SPEC_INST_GET(inst, dio1_gpios),                              \
		.busy_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, busy_gpios, {0}),                      \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, llcc68_init, NULL, &llcc68_data_##inst, &llcc68_config_##inst, \
			      POST_KERNEL, CONFIG_SPI_INIT_PRIORITY, &llcc68_lora_api);

/* Create all driver instances */
DT_INST_FOREACH_STATUS_OKAY(LLCC68_DEFINE)
