/*
 * Copyright (c) 2020 Andreas Sandberg
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/lora.h>
#include <inttypes.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>
#include <string.h>

LOG_MODULE_REGISTER(lora_shell, CONFIG_LORA_LOG_LEVEL);

#define DEFAULT_RADIO_NODE DT_ALIAS(lora0)
BUILD_ASSERT(DT_NODE_HAS_STATUS_OKAY(DEFAULT_RADIO_NODE), "No default LoRa radio specified in DT");


static const struct device *get_modem(const struct shell *sh)
{
    const struct device *dev;

    dev = DEVICE_DT_GET(DEFAULT_RADIO_NODE);

    if (!device_is_ready(dev)) {
        shell_error(sh, "RF device not ready");
        return NULL;
    }

    return dev;
}

static int lora_conf_dump(const struct shell *sh)
{
    // shell_print(sh, "  Frequency: %" PRIu32 " Hz",
    //             modem_config.frequency);

    return 0;
}

static int cmd_rf_send(const struct shell *sh, size_t argc, char **argv)
{
    int ret;
    const struct device *dev;

    modem_config.tx = true;
    dev = get_configured_modem(sh);
    if (!dev) {
        return -ENODEV;
    }

    ret = lora_send(dev, argv[1], strlen(argv[1]));
    if (ret < 0) {
        shell_error(sh, "LoRa send failed: %i", ret);
        return ret;
    }

    return 0;
}

static int cmd_rf_recv(const struct shell *sh, size_t argc, char **argv)
{
    static char buf[0xff];
    const struct device *dev;
    long timeout = 0;
    int ret;
    int16_t rssi;
    int8_t snr;

    modem_config.tx = false;
    dev = get_configured_modem(sh);
    if (!dev) {
        return -ENODEV;
    }

    if (argc >= 2 && parse_long_range(&timeout, sh, argv[1], "timeout", 0, INT_MAX) < 0) {
        return -EINVAL;
    }

    ret = lora_recv(dev, buf, sizeof(buf), timeout ? K_MSEC(timeout) : K_FOREVER, &rssi, &snr);
    if (ret < 0) {
        shell_error(sh, "LoRa recv failed: %i", ret);
        return ret;
    }

    shell_hexdump(sh, buf, ret);
    shell_print(sh, "RSSI: %" PRIi16 " dBm, SNR:%" PRIi8 " dBm", rssi, snr);

    return 0;
}

static int cmd_rf_test_cw(const struct shell *sh, size_t argc, char **argv)
{
    const struct device *dev;
    int ret;
    uint32_t freq;
    long power, duration;

    dev = get_modem(sh);
    if (!dev) {
        return -ENODEV;
    }

    if (parse_freq(&freq, sh, argv[1]) < 0 ||
            parse_long_range(&power, sh, argv[2], "power", INT8_MIN, INT8_MAX) < 0 ||
            parse_long_range(&duration, sh, argv[3], "duration", 0, UINT16_MAX) < 0) {
        return -EINVAL;
    }

    ret = lora_test_cw(dev, (uint32_t)freq, (int8_t)power, (uint16_t)duration);
    if (ret < 0) {
        shell_error(sh, "LoRa test CW failed: %i", ret);
        return ret;
    }

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
    sub_lora,
    SHELL_CMD(config, NULL,
              SHELL_HELP("Configure the LoRa radio",
                         "[freq <Hz>] [tx-power <dBm>] [bw <kHz>] [sf <int>] [cr <int>] "
                         "[pre-len <int>]"),
              cmd_lora_conf),
    SHELL_CMD_ARG(send, NULL, SHELL_HELP("Send a LoRa packet", "<data>"), cmd_lora_send, 2, 0),
    SHELL_CMD_ARG(recv, NULL, SHELL_HELP("Receive a LoRa packet", "[timeout (ms)]"),
                  cmd_lora_recv, 1, 1),
    SHELL_CMD_ARG(test_cw, NULL,
                  SHELL_HELP("Send a continuous wave",
                             "<freq (Hz)> <power (dBm)> <duration (s)>"),
                  cmd_lora_test_cw, 4, 0),
    SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_CMD_REGISTER(lora, &sub_lora, "LoRa commands", NULL);
