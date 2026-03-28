/** @file
 *  @brief BPS Service sample
 */

/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/check.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bps.h>
#include <zephyr/logging/log.h>
#include <sys/errno.h>
LOG_MODULE_REGISTER(bps);

/* BPS Measurement flags (spec Table 3.3) */
#define BPS_FLAG_UNIT_KPA           BIT(0)
#define BPS_FLAG_TIMESTAMP          BIT(1)
#define BPS_FLAG_PULSE_RATE         BIT(2)
#define BPS_FLAG_USER_ID            BIT(3)
#define BPS_FLAG_MEASUREMENT_STATUS BIT(4)

/* Blood Pressure Feature flags - no optional features by default */
static const uint16_t bp_feature = 0x0000;

static sys_slist_t bps_cbs = SYS_SLIST_STATIC_INIT(&bps_cbs);

static struct bt_gatt_indicate_params ind_params;
static bool ind_enabled;

static void bpmc_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);

	struct bt_bps_cb *listener;

	ind_enabled = (value == BT_GATT_CCC_INDICATE);

	LOG_INF("BPS indications %s", ind_enabled ? "enabled" : "disabled");

	SYS_SLIST_FOR_EACH_CONTAINER(&bps_cbs, listener, _node) {
		if (listener->ind_changed) {
			listener->ind_changed(ind_enabled);
		}
	}
}

static ssize_t read_bp_feature(struct bt_conn *conn,
			       const struct bt_gatt_attr *attr,
			       void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 &bp_feature, sizeof(bp_feature));
}

/* Blood Pressure Service Declaration */
BT_GATT_SERVICE_DEFINE(bps_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_BPS),
	/* Blood Pressure Measurement - Indicate (mandatory, no Read per spec) */
	BT_GATT_CHARACTERISTIC(BT_UUID_GATT_BPM,
			       BT_GATT_CHRC_INDICATE,
			       BT_GATT_PERM_NONE,
			       NULL, NULL, NULL),
	BT_GATT_CCC(bpmc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	/* Blood Pressure Feature - Read (mandatory) */
	BT_GATT_CHARACTERISTIC(BT_UUID_GATT_BPF,
			       BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ,
			       read_bp_feature, NULL, NULL),
);

static void indicate_cb(struct bt_conn *conn,
			struct bt_gatt_indicate_params *params,
			uint8_t err)
{
	LOG_INF("BPS indication %s", err == 0U ? "acknowledged" : "failed");
}

static int bps_init(void)
{
	return 0;
}

int bt_bps_cb_register(struct bt_bps_cb *cb)
{
	CHECKIF(cb == NULL) {
		return -EINVAL;
	}

	sys_slist_append(&bps_cbs, &cb->_node);

	return 0;
}

int bt_bps_cb_unregister(struct bt_bps_cb *cb)
{
	CHECKIF(cb == NULL) {
		return -EINVAL;
	}

	if (!sys_slist_find_and_remove(&bps_cbs, &cb->_node)) {
		return -ENOENT;
	}

	return 0;
}

int bt_bps_indicate(const struct bt_bps_measurement *meas)
{
	static uint8_t buf[9]; /* flags(1) + sys(2) + dia(2) + map(2) + pr(2) */
	uint8_t *p = buf;
	uint8_t flags = 0U;

        if(!ind_enabled){
                return -EACCES;
        }
	if (meas->unit_kpa) {
		flags |= BPS_FLAG_UNIT_KPA;
	}
	if (meas->pulse_rate_present) {
		flags |= BPS_FLAG_PULSE_RATE;
	}

	*p++ = flags;

	/* Systolic, Diastolic, MAP - pre-encoded SFLOAT, little-endian */
	sys_put_le16(meas->systolic,  p); p += 2;
	sys_put_le16(meas->diastolic, p); p += 2;
	sys_put_le16(meas->map,       p); p += 2;

	if (meas->pulse_rate_present) {
		sys_put_le16(meas->pulse_rate, p); p += 2;
	}

	ind_params.attr    = &bps_svc.attrs[1];
	ind_params.func    = indicate_cb;
	ind_params.destroy = NULL;
	ind_params.data    = buf;
	ind_params.len     = (uint16_t)(p - buf);

	return bt_gatt_indicate(NULL, &ind_params);
}

SYS_INIT(bps_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
