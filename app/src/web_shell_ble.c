/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <web_shell_ble.h>

#if defined(CONFIG_SHELL_BT_NUS)

#include <errno.h>
#include <string.h>

#include <bt_app_callbacks.h>
#include <bluetooth/services/nus.h>
#include <shell/shell_bt_nus.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(web_shell_ble, CONFIG_SIDEWALK_LOG_LEVEL);

#define WEB_SHELL_DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define WEB_SHELL_DEVICE_NAME_LEN (sizeof(WEB_SHELL_DEVICE_NAME) - 1)
#define WEB_SHELL_ADV_INTERVAL_MIN BT_GAP_MS_TO_ADV_INTERVAL(500)
#define WEB_SHELL_ADV_INTERVAL_MAX BT_GAP_MS_TO_ADV_INTERVAL(750)
#define WEB_SHELL_CONN_INTERVAL_MIN BT_GAP_MS_TO_CONN_INTERVAL(150)
#define WEB_SHELL_CONN_INTERVAL_MAX BT_GAP_MS_TO_CONN_INTERVAL(250)
#define WEB_SHELL_CONN_LATENCY 4U
#define WEB_SHELL_CONN_TIMEOUT 600U

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, WEB_SHELL_DEVICE_NAME, WEB_SHELL_DEVICE_NAME_LEN),
};

static struct bt_le_ext_adv *web_shell_adv;
static struct bt_conn *web_shell_conn;
static struct k_work_delayable adv_restart_work;
static atomic_t initialized;
static const struct bt_le_conn_param web_shell_conn_params = BT_LE_CONN_PARAM_INIT(
	WEB_SHELL_CONN_INTERVAL_MIN, WEB_SHELL_CONN_INTERVAL_MAX, WEB_SHELL_CONN_LATENCY,
	WEB_SHELL_CONN_TIMEOUT);

static int web_shell_adv_start(void)
{
	int err;

	if (web_shell_conn != NULL) {
		return 0;
	}

	if (web_shell_adv == NULL) {
		struct bt_le_adv_param param = {
			.id = BT_ID_DEFAULT,
			.sid = 0,
			.secondary_max_skip = 0,
			.options = BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_IDENTITY,
			.interval_min = WEB_SHELL_ADV_INTERVAL_MIN,
			.interval_max = WEB_SHELL_ADV_INTERVAL_MAX,
			.peer = NULL,
		};

		err = bt_le_ext_adv_create(&param, NULL, &web_shell_adv);
		if (err) {
			LOG_ERR("web shell adv create failed: %d", err);
			return err;
		}

		err = bt_le_ext_adv_set_data(web_shell_adv, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
		if (err) {
			LOG_ERR("web shell adv set data failed: %d", err);
			return err;
		}
	}

	err = bt_le_ext_adv_start(web_shell_adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err == -EALREADY) {
		return 0;
	}
	if (err) {
		LOG_ERR("web shell adv start failed: %d", err);
		return err;
	}

	LOG_INF("Web shell advertising as %s (%u-%u ms)", WEB_SHELL_DEVICE_NAME,
		(unsigned int)BT_GAP_ADV_INTERVAL_TO_MS(WEB_SHELL_ADV_INTERVAL_MIN),
		(unsigned int)BT_GAP_ADV_INTERVAL_TO_MS(WEB_SHELL_ADV_INTERVAL_MAX));
	return 0;
}

static void adv_restart_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	(void)web_shell_adv_start();
}

static void web_shell_bt_ready(int err)
{
	if (err) {
		LOG_ERR("web shell bt ready failed: %d", err);
		return;
	}

	err = shell_bt_nus_init();
	if (err) {
		LOG_ERR("web shell bt nus init failed: %d", err);
		return;
	}

	(void)web_shell_adv_start();
}

static void web_shell_request_conn_params(struct bt_conn *conn)
{
	int err = bt_conn_le_param_update(conn, &web_shell_conn_params);

	if (err && err != -EALREADY) {
		LOG_WRN("Web shell conn param update failed: %d", err);
		return;
	}

	LOG_INF("Web shell requested conn params %u-%u ms latency %u timeout %u ms",
		(unsigned int)BT_CONN_INTERVAL_TO_MS(WEB_SHELL_CONN_INTERVAL_MIN),
		(unsigned int)BT_CONN_INTERVAL_TO_MS(WEB_SHELL_CONN_INTERVAL_MAX),
		(unsigned int)WEB_SHELL_CONN_LATENCY,
		(unsigned int)(WEB_SHELL_CONN_TIMEOUT * 10U));
}

static void web_shell_connected(struct bt_conn *conn, uint8_t err)
{
	struct bt_conn_info info;

	if (err || bt_conn_get_info(conn, &info) != 0 || info.id != BT_ID_DEFAULT) {
		return;
	}

	if (web_shell_conn != NULL) {
		return;
	}

	web_shell_conn = bt_conn_ref(conn);
	LOG_INF("Web shell connected");
	shell_bt_nus_enable(conn);
	web_shell_request_conn_params(conn);
}

static void web_shell_disconnected(struct bt_conn *conn, uint8_t reason)
{
	if (conn != web_shell_conn) {
		return;
	}

	shell_bt_nus_disable();
	bt_conn_unref(web_shell_conn);
	web_shell_conn = NULL;
	LOG_INF("Web shell disconnected: 0x%02x", reason);
	(void)k_work_reschedule(&adv_restart_work, K_MSEC(100));
}

static void web_shell_le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency,
				       uint16_t timeout)
{
	struct bt_conn_info info;

	if (bt_conn_get_info(conn, &info) != 0 || info.id != BT_ID_DEFAULT) {
		return;
	}

	LOG_INF("Web shell conn params active: interval %u ms latency %u timeout %u ms",
		(unsigned int)BT_CONN_INTERVAL_TO_MS(interval), (unsigned int)latency,
		(unsigned int)(timeout * 10U));
}

static struct bt_conn_cb web_shell_conn_cb = {
	.connected = web_shell_connected,
	.disconnected = web_shell_disconnected,
	.le_param_updated = web_shell_le_param_updated,
};

int web_shell_ble_start(void)
{
	int err;

	if (!atomic_cas(&initialized, 0, 1)) {
		return 0;
	}

	k_work_init_delayable(&adv_restart_work, adv_restart_work_handler);
	bt_conn_cb_register(&web_shell_conn_cb);

	err = sid_ble_bt_enable(web_shell_bt_ready);
	if (err) {
		LOG_ERR("web shell bt enable failed: %d", err);
		return err;
	}

	return 0;
}

#else

int web_shell_ble_start(void)
{
	return 0;
}

#endif /* CONFIG_SHELL_BT_NUS */
