/** @file
 *  @brief Blood Pressure Service (BPS) API
 */

/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_BLUETOOTH_SERVICES_BPS_H_
#define ZEPHYR_INCLUDE_BLUETOOTH_SERVICES_BPS_H_

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/sys/slist.h>
#include <zephyr/bluetooth/conn.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Blood Pressure Service (BPS)
 * @defgroup bt_bps Blood Pressure Service (BPS)
 * @ingroup bluetooth
 * @{
 */

/* BPS Characteristic UUIDs */
//#define BT_UUID_BPS_FEATURE_VAL     0x2A49
//#define BT_UUID_BPS_FEATURE         BT_UUID_DECLARE_16(BT_UUID_BPS_FEATURE_VAL)

/** @brief Blood Pressure Measurement structure */
struct bt_bps_measurement {
	/** Systolic pressure, IEEE-11073 SFLOAT (mmHg or kPa) */
	uint16_t systolic;
	/** Diastolic pressure, IEEE-11073 SFLOAT (mmHg or kPa) */
	uint16_t diastolic;
	/** Mean arterial pressure, IEEE-11073 SFLOAT (mmHg or kPa) */
	uint16_t map;
	/** Pulse rate, IEEE-11073 SFLOAT (bpm). Only sent if @p pulse_rate_present is true */
	uint16_t pulse_rate;
	/** Set true to include pulse rate in the indication */
	bool pulse_rate_present;
	/** Set true to report in kPa instead of mmHg */
	bool unit_kpa;
};

/**
 * @brief Blood Pressure Service callback structure.
 *
 * This structure is used to register callbacks for BPS events.
 * Multiple listeners can register by calling @ref bt_bps_cb_register.
 */
struct bt_bps_cb {
	/**
	 * @brief Indication status changed.
	 *
	 * Called when a peer enables or disables indications on the
	 * Blood Pressure Measurement characteristic.
	 *
	 * @param enabled  true if indications were enabled, false if disabled.
	 */
	void (*ind_changed)(bool enabled);

	/** @internal Linked list node for internal use. */
	sys_snode_t _node;
};

/**
 * @brief Register Blood Pressure Service callbacks.
 *
 * @param cb  Pointer to callback structure. Must remain valid for the
 *            lifetime of the application.
 *
 * @retval 0        Success.
 * @retval -EINVAL  @p cb is NULL.
 */
int bt_bps_cb_register(struct bt_bps_cb *cb);

/**
 * @brief Unregister Blood Pressure Service callbacks.
 *
 * @param cb  Pointer to callback structure to unregister.
 *
 * @retval 0        Success.
 * @retval -EINVAL  @p cb is NULL.
 * @retval -ENOENT  @p cb was not registered.
 */
int bt_bps_cb_unregister(struct bt_bps_cb *cb);

/**
 * @brief Send a Blood Pressure Measurement indication.
 *
 * Sends a BPS indication to all connected peers that have enabled it.
 * Use @ref bt_bps_sfloat_encode to convert whole-number mmHg values
 * into the required IEEE-11073 SFLOAT format.
 *
 * @param meas  Pointer to the measurement to send.
 *
 * @retval 0        Success.
 * @retval -ENOTCONN No peer has enabled indications.
 * @retval <0       Other bt_gatt_indicate() error.
 */
int bt_bps_indicate(const struct bt_bps_measurement *meas);

/**
 * @brief Encode a value as IEEE-11073 SFLOAT.
 *
 * Suitable for whole-number mmHg or bpm values (exponent = 0).
 *
 * @param mantissa  12-bit signed value (e.g. 120 for 120 mmHg).
 * @param exponent  4-bit signed exponent (typically 0).
 *
 * @return Encoded SFLOAT as uint16_t, ready for use in
 *         @ref bt_bps_measurement fields.
 */
static inline uint16_t bt_bps_sfloat_encode(int16_t mantissa, int8_t exponent)
{
	return (uint16_t)(((exponent & 0x0F) << 12) | (mantissa & 0x0FFF));
}

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_BLUETOOTH_SERVICES_BPS_H_ */
