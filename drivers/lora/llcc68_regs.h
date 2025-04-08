/*
 * Copyright (c) 2025 Zephyr Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Register and command definitions for LLCC68 LoRa Transceiver
 */

#ifndef ZEPHYR_DRIVERS_LORA_LLCC68_REGS_H_
#define ZEPHYR_DRIVERS_LORA_LLCC68_REGS_H_

#ifdef __cplusplus
extern "C" {
#endif

/** LLCC68 Command Opcodes */
/** Operational mode commands */
#define LLCC68_CMD_SET_SLEEP                0x84
#define LLCC68_CMD_SET_STANDBY              0x80
#define LLCC68_CMD_SET_FS                   0xC1
#define LLCC68_CMD_SET_TX                   0x83
#define LLCC68_CMD_SET_RX                   0x82
#define LLCC68_CMD_STOP_TIMER_ON_PREAMBLE   0x9F
#define LLCC68_CMD_SET_RX_DUTY_CYCLE        0x94
#define LLCC68_CMD_SET_CAD                  0xC5
#define LLCC68_CMD_SET_TX_CONTINUOUS_WAVE   0xD1
#define LLCC68_CMD_SET_TX_INFINITE_PREAMBLE 0xD2
#define LLCC68_CMD_SET_REGULATOR_MODE       0x96
#define LLCC68_CMD_CALIBRATE                0x89
#define LLCC68_CMD_CALIBRATE_IMAGE          0x98
#define LLCC68_CMD_SET_PA_CONFIG            0x95
#define LLCC68_CMD_SET_RX_TX_FALLBACK_MODE  0x93

/** Register and buffer access commands */
#define LLCC68_CMD_WRITE_REGISTER           0x0D
#define LLCC68_CMD_READ_REGISTER            0x1D
#define LLCC68_CMD_WRITE_BUFFER             0x0E
#define LLCC68_CMD_READ_BUFFER              0x1E

/** DIO and IRQ control commands */
#define LLCC68_CMD_SET_DIO_IRQ_PARAMS       0x08
#define LLCC68_CMD_GET_IRQ_STATUS           0x12
#define LLCC68_CMD_CLEAR_IRQ_STATUS         0x02
#define LLCC68_CMD_SET_DIO2_AS_RF_SWITCH    0x9D
#define LLCC68_CMD_SET_DIO3_AS_TCXO_CTRL    0x97

/** RF modulation and packet commands */
#define LLCC68_CMD_SET_RF_FREQUENCY         0x86
#define LLCC68_CMD_SET_PACKET_TYPE          0x8A
#define LLCC68_CMD_GET_PACKET_TYPE          0x11
#define LLCC68_CMD_SET_TX_PARAMS            0x8E
#define LLCC68_CMD_SET_MODULATION_PARAMS    0x8B
#define LLCC68_CMD_SET_PACKET_PARAMS        0x8C
#define LLCC68_CMD_SET_CAD_PARAMS           0x88
#define LLCC68_CMD_SET_BUFFER_BASE_ADDRESS  0x8F
#define LLCC68_CMD_SET_LORA_SYMB_NUM_TIMEOUT 0xA0

/** Status commands */
#define LLCC68_CMD_GET_STATUS               0xC0
#define LLCC68_CMD_GET_RSSI_INST            0x15
#define LLCC68_CMD_GET_RX_BUFFER_STATUS     0x13
#define LLCC68_CMD_GET_PACKET_STATUS        0x14
#define LLCC68_CMD_GET_DEVICE_ERRORS        0x17
#define LLCC68_CMD_CLEAR_DEVICE_ERRORS      0x07
#define LLCC68_CMD_GET_STATS                0x10
#define LLCC68_CMD_RESET_STATS              0x00

/** Register addresses */
#define LLCC68_REG_DIO_OUTPUT_ENABLE        0x0580
#define LLCC68_REG_DIO_INPUT_ENABLE         0x0583
#define LLCC68_REG_WHITENING_INITIAL_MSB    0x06B8
#define LLCC68_REG_WHITENING_INITIAL_LSB    0x06B9
#define LLCC68_REG_CRC_INITIAL_MSB          0x06BC
#define LLCC68_REG_CRC_INITIAL_LSB          0x06BD
#define LLCC68_REG_CRC_POLYNOMIAL_MSB       0x06BE
#define LLCC68_REG_CRC_POLYNOMIAL_LSB       0x06BF
#define LLCC68_REG_SYNC_WORD_0              0x06C0
#define LLCC68_REG_SYNC_WORD_1              0x06C1
#define LLCC68_REG_SYNC_WORD_2              0x06C2
#define LLCC68_REG_SYNC_WORD_3              0x06C3
#define LLCC68_REG_SYNC_WORD_4              0x06C4
#define LLCC68_REG_SYNC_WORD_5              0x06C5
#define LLCC68_REG_SYNC_WORD_6              0x06C6
#define LLCC68_REG_SYNC_WORD_7              0x06C7
#define LLCC68_REG_NODE_ADDRESS             0x06CD
#define LLCC68_REG_BROADCAST_ADDRESS        0x06CE
#define LLCC68_REG_LORA_SYNC_WORD_MSB       0x0740
#define LLCC68_REG_LORA_SYNC_WORD_LSB       0x0741
#define LLCC68_REG_TX_MODULATION            0x0889
#define LLCC68_REG_RX_GAIN                  0x08AC
#define LLCC68_REG_TX_CLAMP_CONFIG          0x08D8
#define LLCC68_REG_OCP_CONFIGURATION        0x08E7
#define LLCC68_REG_RTC_CONTROL              0x0902
#define LLCC68_REG_XTA_TRIM                 0x0911
#define LLCC68_REG_XTB_TRIM                 0x0912

/** Packet type definitions */
#define LLCC68_PACKET_TYPE_GFSK             0x00
#define LLCC68_PACKET_TYPE_LORA             0x01

/** Sleep mode configuration */
#define LLCC68_SLEEP_START_COLD             0x00
#define LLCC68_SLEEP_START_WARM             0x04
#define LLCC68_SLEEP_RTC_DISABLE            0x00
#define LLCC68_SLEEP_RTC_ENABLE             0x01

/** Standby mode configuration */
#define LLCC68_STANDBY_RC                   0x00
#define LLCC68_STANDBY_XOSC                 0x01

/** Regulator mode */
#define LLCC68_REGULATOR_LDO                0x00
#define LLCC68_REGULATOR_DC_DC              0x01

/** Calibration settings */
#define LLCC68_CALIBRATE_ALL                0x7F
#define LLCC68_CALIBRATE_RC64K              0x01
#define LLCC68_CALIBRATE_RC13M              0x02
#define LLCC68_CALIBRATE_PLL                0x04
#define LLCC68_CALIBRATE_ADC_PULSE          0x08
#define LLCC68_CALIBRATE_ADC_BULK_N         0x10
#define LLCC68_CALIBRATE_ADC_BULK_P         0x20
#define LLCC68_CALIBRATE_IMAGE_OFF          0x00
#define LLCC68_CALIBRATE_IMAGE_ON           0x01

/** PA configuration */
#define LLCC68_PA_CONFIG_DUTY_CYCLE         0x04
#define LLCC68_PA_CONFIG_HP_MAX             0x07

/** TX parameters */
#define LLCC68_TX_RAMP_10U                  0x00
#define LLCC68_TX_RAMP_20U                  0x01
#define LLCC68_TX_RAMP_40U                  0x02
#define LLCC68_TX_RAMP_80U                  0x03
#define LLCC68_TX_RAMP_200U                 0x04
#define LLCC68_TX_RAMP_800U                 0x05
#define LLCC68_TX_RAMP_1700U                0x06
#define LLCC68_TX_RAMP_3400U                0x07

/** LoRa bandwidth settings */
#define LLCC68_LORA_BW_125                  0x04
#define LLCC68_LORA_BW_250                  0x05
#define LLCC68_LORA_BW_500                  0x06

/** LoRa coding rate denominator */
#define LLCC68_LORA_CR_4_5                  0x01
#define LLCC68_LORA_CR_4_6                  0x02
#define LLCC68_LORA_CR_4_7                  0x03
#define LLCC68_LORA_CR_4_8                  0x04

/** LoRa CAD parameters */
#define LLCC68_CAD_ON_1_SYMB                0x00
#define LLCC68_CAD_ON_2_SYMB                0x01
#define LLCC68_CAD_ON_4_SYMB                0x02
#define LLCC68_CAD_ON_8_SYMB                0x03
#define LLCC68_CAD_ON_16_SYMB               0x04

/** Fallback modes */
#define LLCC68_FALLBACK_FS                  0x40
#define LLCC68_FALLBACK_STDBY_XOSC          0x30
#define LLCC68_FALLBACK_STDBY_RC            0x20

/** TCXO control voltages */
#define LLCC68_TCXO_CTRL_1_6V               0x00
#define LLCC68_TCXO_CTRL_1_7V               0x01
#define LLCC68_TCXO_CTRL_1_8V               0x02
#define LLCC68_TCXO_CTRL_2_2V               0x03
#define LLCC68_TCXO_CTRL_2_4V               0x04
#define LLCC68_TCXO_CTRL_2_7V               0x05
#define LLCC68_TCXO_CTRL_3_0V               0x06
#define LLCC68_TCXO_CTRL_3_3V               0x07

/** IRQ definitions */
#define LLCC68_IRQ_NONE                     0x0000
#define LLCC68_IRQ_ALL                      0x03FF
#define LLCC68_IRQ_TX_DONE                  0x0001
#define LLCC68_IRQ_RX_DONE                  0x0002
#define LLCC68_IRQ_PREAMBLE_DETECTED        0x0004
#define LLCC68_IRQ_SYNC_WORD_VALID          0x0008
#define LLCC68_IRQ_HEADER_VALID             0x0010
#define LLCC68_IRQ_HEADER_ERROR             0x0020
#define LLCC68_IRQ_CRC_ERROR                0x0040
#define LLCC68_IRQ_CAD_DONE                 0x0080
#define LLCC68_IRQ_CAD_DETECTED             0x0100
#define LLCC68_IRQ_TIMEOUT                  0x0200

/** Status masks */
#define LLCC68_STATUS_MODE_MASK             0x70
#define LLCC68_STATUS_MODE_SLEEP            0x00
#define LLCC68_STATUS_MODE_STDBY_RC         0x10
#define LLCC68_STATUS_MODE_STDBY_XOSC       0x20
#define LLCC68_STATUS_MODE_FS               0x30
#define LLCC68_STATUS_MODE_TX               0x40
#define LLCC68_STATUS_MODE_RX               0x50
#define LLCC68_STATUS_MODE_CAD              0x60

/** Error codes */
#define LLCC68_ERROR_NONE                   0x0000
#define LLCC68_ERROR_RC64K_CALIBRATION      0x0001
#define LLCC68_ERROR_RC13M_CALIBRATION      0x0002
#define LLCC68_ERROR_PLL_CALIBRATION        0x0004
#define LLCC68_ERROR_ADC_CALIBRATION        0x0008
#define LLCC68_ERROR_IMG_CALIBRATION        0x0010
#define LLCC68_ERROR_XOSC_START             0x0020
#define LLCC68_ERROR_PLL_LOCK               0x0040
#define LLCC68_ERROR_PA_RAMP                0x0100

/** Timeout values */
#define LLCC68_TIMEOUT_RESET                50    /* ms */
#define LLCC68_TIMEOUT_BUSY                 100   /* ms */
#define LLCC68_TIMEOUT_READY                1000  /* ms */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_LORA_LLCC68_REGS_H_ */
