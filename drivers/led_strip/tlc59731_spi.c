/*
 * Copyright (c) 2024 Javad Rahimipetroudi <javad.rahimipetroudi@mind.be>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_tlc59731_spi

/**
 * @file
 * @brief LED driver for the TLC59731 LED driver.
 *
 * TLC59731 is a 3-Channel, 8-Bit, PWM LED Driver
 * With Single-Wire Interface (EasySet)
 *
 * The EasySet protocol is based on short pulses and the time between
 * them. At least one pulse must be sent every T_CYCLE, which can be
 * between 1.67us and 50us. We want to go as fast as possible, but
 * delays under 1us don't work very well, so we settle on 5us for the
 * cycle time.
 * A pulse must be high for at least 14ns. In practice, turning a GPIO on
 * and immediately off again already takes longer than that, so no delay
 * is needed there.
 * A zero is represented by no additional pulses within a cycle.
 * A one is represented by an additional pulse between 275ns and 2.5us
 * (half a cycle) after the first one. We need at least some delay to get to
 * 275ns, but because of the limited granularity of k_busy_wait we use a
 * full 1us. After the pulse, we wait an additional T_CYCLE_1 to complete
 * the cycle. This time can be slightly shorter because the second pulse
 * already closes the cycle.
 * Finally we need to keep the line low for T_H0 to complete the address
 * for a single chip, and T_H1 to complete the write for all chips.
 */

#include <zephyr/drivers/led_strip.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/math_extras.h>
#include <zephyr/sys/util.h>
#include <zephyr/dt-bindings/led/led.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tlc59731_spi, CONFIG_LED_STRIP_LOG_LEVEL);

/* Pulse timing */
#define TLC59731_DELAY     0x01 /* us */
#define TLC59731_T_CYCLE_0 0x04 /* us */
#define TLC59731_T_CYCLE_1 0x01 /* us */
#define TLC59731_T_H0      (4 * TLC59731_T_CYCLE_0)
#define TLC59731_T_H1      (8 * TLC59731_T_CYCLE_0)
/* Threshould levels */
#define TLC59731_HIGH      0x01
#define TLC59731_LOW       0x00
/* SPI Timing */
#define TLC59731_T_CYCLE 0x80 /* 1000 0000 */
#define TLC59731_ONE     0x90 /* 1001 0000 */
#define	TLC59731_ZERO    0x80 /* 1000 0000 */
/* Write command */
#define TLC59731_WR 0x3A
/* spi-one-frame and spi-zero-frame in DT are for 8-bit frames. */
#define SPI_FRAME_BITS 8
/*
 * SPI master configuration:
 *
 * - mode 0 (the default), 8 bit, MSB first (arbitrary), one-line SPI
 * - no shenanigans (don't hold CS, don't hold the device lock, this
 *   isn't an EEPROM)
 */
#define SPI_OPER(idx) (SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | \
		SPI_WORD_SET(SPI_FRAME_BITS))
static int TLC59731_EOS[] = {0x80, 0x00,0x00,0x01};
static int TLC59731_GSLAT[]= {0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01};
static int TLC59731_WR_CMD[] = {TLC59731_ZERO,TLC59731_ZERO, TLC59731_ONE, TLC59731_ONE, TLC59731_ONE, TLC59731_ZERO, TLC59731_ONE, TLC59731_ZERO};
struct tlc59731_spi_cfg {
	struct spi_dt_spec bus;
	uint8_t *px_buf;
	size_t px_buf_size;
	uint8_t one_frame;
	uint8_t zero_frame;
	uint8_t num_colors;
	const uint8_t *color_mapping;
	uint16_t reset_delay;

};

/*
 * Serialize an 8-bit color channel value into an equivalent sequence
 * of SPI frames, MSbit first, where a one bit becomes SPI frame
 * one_frame, and zero bit becomes zero_frame.
 */
static inline void tlc59731_spi_ser(uint8_t buf[8], uint8_t color,
		const uint8_t one_frame, const uint8_t zero_frame)
{
	int i;
	for (i = 0; i < 8; i++) {
		buf[i] = color & BIT(7 - i) ? one_frame : zero_frame;
	}
}
/*
 * Returns true if and only if cfg->px_buf is big enough to convert
 * num_pixels RGB color values into SPI frames.
 */
static inline bool num_pixels_ok(const struct tlc59731_spi_cfg *cfg,
		size_t num_pixels)
{
	size_t nbytes;
	bool overflow;

	overflow = size_mul_overflow(num_pixels, cfg->num_colors * 8, &nbytes);
	return !overflow && (nbytes <= cfg->px_buf_size);
}

static const struct tlc59731_spi_cfg *dev_cfg(const struct device *dev)
{
	return dev->config;
}

/*
 * Latch current color values on strip and reset its state machines.
 */
static inline void tlc59731_reset_delay(uint16_t delay)
{
	k_usleep(delay);
}

static int tlc59731_write_header(const struct device *dev)
{
	int ret = 0;
	const struct tlc59731_spi_cfg *cfg = dev_cfg(dev);
	struct spi_buf buf = {
		.buf = TLC59731_WR_CMD,
		.len = 8,
	};
	const struct spi_buf_set tx = {
		.buffers = &buf,
		.count = 1
	};

	ret = spi_write_dt(&cfg->bus, &tx);

	if(ret)
	{
		LOG_ERR("%s: SPI Write failed.", dev->name);
	}

	return 0;
}

static int tlc59731_write_EOS(const struct device *dev)
{
	int ret = 0;
	const struct tlc59731_spi_cfg *cfg = dev_cfg(dev);
	struct spi_buf buf = {
		.buf = TLC59731_EOS,
		.len = 4,
	};
	const struct spi_buf_set tx = {
		.buffers = &buf,
		.count = 1
	};

	ret = spi_write_dt(&cfg->bus, &tx);

	if(ret)
	{
		LOG_ERR("%s: SPI Write failed.", dev->name);
	}

	return 0;
}

static int tlc59731_gpio_update_rgb(const struct device *dev, struct led_rgb *pixels,
		size_t num_pixels)
{
	size_t i;
	int err = 0;

	tlc59731_write_header(dev);

	const struct tlc59731_spi_cfg *cfg = dev_cfg(dev);
	const uint8_t one = TLC59731_ONE, zero = TLC59731_ZERO;
	struct spi_buf buf = {
		.buf = cfg->px_buf,
		.len = cfg->px_buf_size,
	};
	const struct spi_buf_set tx = {
		.buffers = &buf,
		.count = 1
	};

	uint8_t *px_buf = cfg->px_buf;
	int rc;

	if (!num_pixels_ok(cfg, num_pixels)) {
		return -ENOMEM;
	}

	/*
	 * Convert pixel data into SPI frames. Each frame has pixel data
	 * in color mapping on-wire format (e.g. GRB, GRBW, RGB, etc).
	 */
	for (i = 0; i < num_pixels; i++) {
		uint8_t j;

		for (j = 0; j < cfg->num_colors; j++) {
			uint8_t pixel;

			switch (cfg->color_mapping[j]) {
				/* White channel is not supported by LED strip API. */
				case LED_COLOR_ID_RED:
					pixel = pixels[i].r;
					break;
				case LED_COLOR_ID_GREEN:
					pixel = pixels[i].g;
					break;
				case LED_COLOR_ID_BLUE:
					pixel = pixels[i].b;
					break;
				default:
					return -EINVAL;
			}
			tlc59731_spi_ser(px_buf, pixel, one, zero);
			px_buf += 8;
		}
	}
	/*
	 * Display the pixel data.
	 */
	rc = spi_write_dt(&cfg->bus, &tx);
	tlc59731_reset_delay(cfg->reset_delay);

	tlc59731_write_EOS(dev);
	return err;
}

static int tlc59731_gpio_update_channels(const struct device *dev, uint8_t *channels,
		size_t num_channels)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(channels);
	ARG_UNUSED(num_channels);

	return -ENOTSUP;
}

static const struct led_strip_driver_api tlc59731_spi_api = {
	.update_rgb = tlc59731_gpio_update_rgb,
	.update_channels = tlc59731_gpio_update_channels,
};

static int tlc59731_spi_init(const struct device *dev)
{
	const struct tlc59731_spi_cfg *cfg = dev_cfg(dev);
	uint8_t i;
	printk("Init Device\n");
	if (!spi_is_ready_dt(&cfg->bus)) {
		LOG_ERR("SPI device %s not ready", cfg->bus.bus->name);
		return -ENODEV;
	}

	for (i = 0; i < cfg->num_colors; i++) {
		switch (cfg->color_mapping[i]) {
			case LED_COLOR_ID_RED:
			case LED_COLOR_ID_GREEN:
			case LED_COLOR_ID_BLUE:
				break;
			default:
				LOG_ERR("%s: invalid channel to color mapping."
						"Check the color-mapping DT property",
						dev->name);
				return -EINVAL;
		}
	}
	/* Send T_CYCLE */
	uint8_t raw_buf[] = {TLC59731_T_CYCLE};
	struct spi_buf buf = {
		.buf = raw_buf,
		.len = 1,
	};
	const struct spi_buf_set tx = {
		.buffers = &buf,
		.count = 1
	};

	int rc = spi_write_dt(&cfg->bus, &tx);
	if(rc)
	{
		LOG_ERR("%s: Send Tcycle failed.\r\n");
	}


	k_busy_wait((TLC59731_DELAY + TLC59731_T_CYCLE_0));
	return 0;
}

#define TLC59731_SPI_NUM_PIXELS(idx) \
	(DT_INST_PROP(idx, chain_length))
#define TLC59731_SPI_HAS_WHITE(idx) \
	(DT_INST_PROP(idx, has_white_channel) == 1)
#define TLC59731_SPI_ONE_FRAME(idx) \
	(DT_INST_PROP(idx, spi_one_frame))
#define TLC59731_SPI_ZERO_FRAME(idx) \
	(DT_INST_PROP(idx, spi_zero_frame))
#define TLC59731_SPI_BUFSZ(idx) \
	(TLC59731_NUM_COLORS(idx) * 8 * TLC59731_SPI_NUM_PIXELS(idx))

/*
 * Retrieve the channel to color mapping (e.g. RGB, BGR, GRB, ...) from the
 * "color-mapping" DT property.
 */
#define TLC59731_COLOR_MAPPING(idx)                                 \
	static const uint8_t tlc59731_spi_##idx##_color_mapping[] = \
	DT_INST_PROP(idx, color_mapping)

#define TLC59731_NUM_COLORS(idx) (DT_INST_PROP_LEN(idx, color_mapping))

/* Get the latch/reset delay from the "reset-delay" DT property. */
#define TLC59731_RESET_DELAY(idx) DT_INST_PROP(idx, reset_delay)


#define TLC59731_SPI_DEVICE(idx)                                                                         \
	static uint8_t tlc59731_spi_##idx##_px_buf[TLC59731_SPI_BUFSZ(idx)]; \
	\
	TLC59731_COLOR_MAPPING(idx);                                       \
	\
	static struct tlc59731_spi_cfg tlc59731_cfg_##idx = {                                            \
		.bus = SPI_DT_SPEC_INST_GET(idx, SPI_OPER(idx), 0),      \
		.px_buf = tlc59731_spi_##idx##_px_buf,                     \
		.px_buf_size = TLC59731_SPI_BUFSZ(idx),                    \
		.one_frame = TLC59731_SPI_ONE_FRAME(idx),                  \
		.zero_frame = TLC59731_SPI_ZERO_FRAME(idx),                \
		.num_colors = TLC59731_NUM_COLORS(idx),                    \
		.color_mapping = tlc59731_spi_##idx##_color_mapping,       \
		.reset_delay = TLC59731_RESET_DELAY(idx),                  \
	};                                                                                         \
	\
	DEVICE_DT_INST_DEFINE(idx, tlc59731_spi_init, NULL, NULL, &tlc59731_cfg_##idx, POST_KERNEL,   \
			CONFIG_LED_STRIP_INIT_PRIORITY, &tlc59731_spi_api);
DT_INST_FOREACH_STATUS_OKAY(TLC59731_SPI_DEVICE)
