/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <cli/app_cli_ui.h>

#include <errno.h>
#include <string.h>

#include <bluetooth/services/nus.h>
#include <buttons.h>
#include <dk_buttons_and_leds.h>
#include <shell/shell_bt_nus.h>

#include <bt_app_callbacks.h>
#include <json_printer/sidTypes2str.h>
#include <sid_api.h>
#include <sid_error.h>
#include <sid_pal_radio_ifc.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(app_cli_ui, CONFIG_SIDEWALK_LOG_LEVEL);

#define APP_CLI_UI_LED_ID DK_LED1
#define APP_CLI_UI_CONN_LATENCY 0
#define APP_CLI_UI_CONN_TIMEOUT_10MS 400

#define MS_TO_CONN_INTERVAL(ms) ((uint16_t)(((ms) * 4) / 5))
#define MS_TO_ADV_INTERVAL(ms) ((uint16_t)(((ms) * 8) / 5))

#if DT_NODE_EXISTS(DT_PATH(buttons))
#define APP_CLI_UI_BUTTON_COUNT DT_CHILD_NUM(DT_PATH(buttons))
#else
#define APP_CLI_UI_BUTTON_COUNT 0
#endif

#if DT_HAS_ALIAS(sw0) && DT_NODE_HAS_STATUS(DT_ALIAS(sw0), okay) && (APP_CLI_UI_BUTTON_COUNT == 1)
#define APP_CLI_UI_HAS_SINGLE_BUTTON 1
static const struct gpio_dt_spec sw0 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
#else
#define APP_CLI_UI_HAS_SINGLE_BUTTON 0
#endif

#if DT_HAS_ALIAS(led0) && DT_NODE_HAS_STATUS(DT_ALIAS(led0), okay)
#define APP_CLI_UI_HAS_LED 1
#else
#define APP_CLI_UI_HAS_LED 0
#endif

#if DT_HAS_ALIAS(lora_transceiver) && DT_NODE_HAS_STATUS(DT_ALIAS(lora_transceiver), okay) && \
	DT_NODE_HAS_PROP(DT_ALIAS(lora_transceiver), reset_gpios)
#define APP_CLI_UI_HAS_LORA_RESET 1
static const struct gpio_dt_spec lora_reset =
	GPIO_DT_SPEC_GET(DT_ALIAS(lora_transceiver), reset_gpios);
#else
#define APP_CLI_UI_HAS_LORA_RESET 0
#endif

#if DT_HAS_CHOSEN(zephyr_console)
static const struct device *const console_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
#endif

struct led_pattern_state {
	uint16_t on_ms;
	uint16_t off_ms;
	uint8_t remaining_pulses;
	bool led_on;
};

struct sidewalk_poweroff_ctx {
	struct k_sem done;
};

static sidewalk_ctx_t *sidewalk_ctx;

#if APP_CLI_UI_HAS_LED
static struct led_pattern_state led_pattern;
static void led_pattern_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(led_pattern_work, led_pattern_work_handler);
#endif

#if APP_CLI_UI_HAS_SINGLE_BUTTON
static void system_off_work_handler(struct k_work *work);
static K_WORK_DEFINE(system_off_work, system_off_work_handler);
#endif

#if defined(CONFIG_SID_END_DEVICE_NUS_SHELL)
static struct bt_le_ext_adv *nus_adv;
static struct bt_conn *nus_conn;
static atomic_t nus_restart_pending = ATOMIC_INIT(false);
static void nus_adv_restart_work_handler(struct k_work *work);
static K_WORK_DEFINE(nus_adv_restart_work, nus_adv_restart_work_handler);

static const struct bt_data nus_ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

static const struct bt_data nus_sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_SID_END_DEVICE_NUS_DEVICE_NAME,
		sizeof(CONFIG_SID_END_DEVICE_NUS_DEVICE_NAME) - 1),
};

static struct bt_le_adv_param nus_adv_param = {
	.id = BT_ID_DEFAULT,
	.options = BT_LE_ADV_OPT_CONN,
	.interval_min = MS_TO_ADV_INTERVAL(CONFIG_SID_END_DEVICE_NUS_ADV_INTERVAL_MS),
	.interval_max = MS_TO_ADV_INTERVAL(CONFIG_SID_END_DEVICE_NUS_ADV_INTERVAL_MS),
};

static const struct bt_le_conn_param nus_conn_param = {
	.interval_min = MS_TO_CONN_INTERVAL(CONFIG_SID_END_DEVICE_NUS_CONN_INTERVAL_MIN_MS),
	.interval_max = MS_TO_CONN_INTERVAL(CONFIG_SID_END_DEVICE_NUS_CONN_INTERVAL_MAX_MS),
	.latency = APP_CLI_UI_CONN_LATENCY,
	.timeout = APP_CLI_UI_CONN_TIMEOUT_10MS,
};
#endif

#if APP_CLI_UI_HAS_LED
static void led_apply(bool on)
{
	if (on) {
		dk_set_led_on(APP_CLI_UI_LED_ID);
	} else {
		dk_set_led_off(APP_CLI_UI_LED_ID);
	}
}

static void led_pattern_start(uint8_t pulses, uint16_t on_ms, uint16_t off_ms)
{
	if (pulses == 0U) {
		return;
	}

	led_pattern.on_ms = on_ms;
	led_pattern.off_ms = off_ms;
	led_pattern.remaining_pulses = pulses;
	led_pattern.led_on = true;

	(void)k_work_cancel_delayable(&led_pattern_work);
	led_apply(true);
	(void)k_work_schedule(&led_pattern_work, K_MSEC(on_ms));
}

static void led_pattern_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (led_pattern.led_on) {
		led_apply(false);
		led_pattern.led_on = false;
		led_pattern.remaining_pulses--;

		if (led_pattern.remaining_pulses == 0U) {
			return;
		}

		(void)k_work_schedule(&led_pattern_work, K_MSEC(led_pattern.off_ms));
		return;
	}

	led_apply(true);
	led_pattern.led_on = true;
	(void)k_work_schedule(&led_pattern_work, K_MSEC(led_pattern.on_ms));
}
#endif

static void sidewalk_prepare_for_poweroff(sidewalk_ctx_t *sid, void *ctx)
{
	struct sidewalk_poweroff_ctx *poweroff_ctx = ctx;

	if (sid && sid->handle) {
		uint32_t link_mask = sid->config.link_mask ? sid->config.link_mask : SID_LINK_TYPE_ANY;
		sid_error_t err = sid_stop(sid->handle, link_mask);

		LOG_INF("sid_stop before system off returned %d (%s)", err, SID_ERROR_T_STR(err));

		err = sid_deinit(sid->handle);
		LOG_INF("sid_deinit before system off returned %d (%s)", err, SID_ERROR_T_STR(err));
		sid->handle = NULL;
	}

	k_sem_give(&poweroff_ctx->done);
}

static void prepare_radio_for_poweroff(void)
{
#if defined(CONFIG_SIDEWALK_SUBGHZ_RADIO_SX126X)
	int32_t err = sid_pal_radio_sleep(0);

	if (err != RADIO_ERROR_NONE) {
		LOG_WRN("sid_pal_radio_sleep before system off returned %d", err);
	}
#endif

#if APP_CLI_UI_HAS_LORA_RESET
	if (device_is_ready(lora_reset.port)) {
		(void)gpio_pin_configure_dt(&lora_reset, GPIO_OUTPUT_ACTIVE);
	}
#endif
}

static void prepare_button_wakeup(void)
{
#if APP_CLI_UI_HAS_SINGLE_BUTTON
	if (!device_is_ready(sw0.port)) {
		return;
	}

	(void)gpio_pin_configure_dt(&sw0, GPIO_INPUT);
	(void)gpio_pin_interrupt_configure_dt(&sw0, GPIO_INT_LEVEL_ACTIVE);
#endif
}

static void suspend_console_for_poweroff(void)
{
#if DT_HAS_CHOSEN(zephyr_console)
	if (device_is_ready(console_dev)) {
		(void)pm_device_action_run(console_dev, PM_DEVICE_ACTION_SUSPEND);
	}
#endif
}

static void enter_system_off(void)
{
	if (sidewalk_ctx != NULL) {
		struct sidewalk_poweroff_ctx poweroff_ctx;

		k_sem_init(&poweroff_ctx.done, 0, 1);

		if (sidewalk_event_send(sidewalk_prepare_for_poweroff, &poweroff_ctx, NULL) == 0) {
			(void)k_sem_take(&poweroff_ctx.done, K_MSEC(1500));
		}
	}

	prepare_radio_for_poweroff();
	prepare_button_wakeup();
	suspend_console_for_poweroff();
	sys_poweroff();
}

#if APP_CLI_UI_HAS_SINGLE_BUTTON
static void system_off_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	enter_system_off();
}

static void system_off_request(uint32_t unused)
{
	ARG_UNUSED(unused);

#if APP_CLI_UI_HAS_LED
	led_pattern_start(1, 200, 0);
#endif
	k_work_submit(&system_off_work);
}
#endif

#if defined(CONFIG_SID_END_DEVICE_NUS_SHELL)
static bool is_nus_connection(struct bt_conn *conn)
{
	struct bt_conn_info info = {};

	if (conn == NULL || bt_conn_get_info(conn, &info) != 0) {
		return false;
	}

	return info.id == BT_ID_DEFAULT;
}

static int nus_adv_start(void)
{
	if (nus_adv == NULL) {
		int err = bt_le_ext_adv_create(&nus_adv_param, NULL, &nus_adv);

		if (err) {
			LOG_ERR("Failed to create NUS advertiser (err %d)", err);
			return err;
		}
	}

	int err = bt_le_ext_adv_set_data(nus_adv, nus_ad, ARRAY_SIZE(nus_ad), nus_sd,
					 ARRAY_SIZE(nus_sd));

	if (err) {
		LOG_ERR("Failed to set NUS advertiser data (err %d)", err);
		return err;
	}

	err = bt_le_ext_adv_start(nus_adv, NULL);
	if (err && err != -EALREADY) {
		LOG_ERR("Failed to start NUS advertiser (err %d)", err);
		return err;
	}

	LOG_INF("NUS shell advertising as %s", CONFIG_SID_END_DEVICE_NUS_DEVICE_NAME);
	return 0;
}

static void nus_adv_restart_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	(void)nus_adv_start();
}

static void nus_connected(struct bt_conn *conn, uint8_t err)
{
	if (!is_nus_connection(conn)) {
		return;
	}

	if (err) {
		LOG_WRN("NUS shell connection failed (err 0x%02x)", err);
		(void)k_work_submit(&nus_adv_restart_work);
		return;
	}

	if (nus_conn != NULL) {
		bt_conn_unref(nus_conn);
	}

	nus_conn = bt_conn_ref(conn);
	shell_bt_nus_enable(conn);
	(void)bt_conn_le_param_update(conn, &nus_conn_param);
	LOG_INF("NUS shell connected");
	app_cli_ui_notify_shell_connected();
}

static void nus_disconnected(struct bt_conn *conn, uint8_t reason)
{
	if (!is_nus_connection(conn)) {
		return;
	}

	LOG_INF("NUS shell disconnected (reason 0x%02x)", reason);
	shell_bt_nus_disable();

	if (nus_conn != NULL) {
		bt_conn_unref(nus_conn);
		nus_conn = NULL;
	}

	atomic_set(&nus_restart_pending, true);
	app_cli_ui_notify_shell_disconnected();
}

static void nus_recycled(void)
{
	if (atomic_cas(&nus_restart_pending, true, false)) {
		(void)k_work_submit(&nus_adv_restart_work);
	}
}

BT_CONN_CB_DEFINE(nus_conn_callbacks) = {
	.connected = nus_connected,
	.disconnected = nus_disconnected,
	.recycled = nus_recycled,
};

static int nus_shell_init(void)
{
	int err = sid_ble_bt_enable(NULL);

	if (err) {
		LOG_ERR("Failed to enable Bluetooth for NUS shell (err %d)", err);
		return err;
	}

	err = shell_bt_nus_init();
	if (err) {
		LOG_ERR("Failed to initialize BT NUS shell transport (err %d)", err);
		return err;
	}

	return nus_adv_start();
}
#endif

int app_cli_ui_init(sidewalk_ctx_t *sid)
{
	sidewalk_ctx = sid;

#if APP_CLI_UI_HAS_LED
	if (dk_leds_init() != 0) {
		LOG_WRN("Failed to initialize LED indications");
	} else {
		dk_set_led_off(APP_CLI_UI_LED_ID);
	}
#endif

#if APP_CLI_UI_HAS_SINGLE_BUTTON
	(void)button_set_action_long_press(DK_BTN1, system_off_request, 0);
	if (buttons_init() != 0) {
		LOG_WRN("Failed to initialize button handling");
	}
#endif

#if defined(CONFIG_SID_END_DEVICE_NUS_SHELL)
	return nus_shell_init();
#else
	return 0;
#endif
}

void app_cli_ui_notify_activity(enum app_cli_ui_activity activity)
{
#if APP_CLI_UI_HAS_LED
	switch (activity) {
	case APP_CLI_UI_ACTIVITY_TX:
		led_pattern_start(1, 40, 0);
		break;
	case APP_CLI_UI_ACTIVITY_RX:
		led_pattern_start(2, 35, 35);
		break;
	case APP_CLI_UI_ACTIVITY_ERROR:
		led_pattern_start(3, 45, 45);
		break;
	default:
		break;
	}
#else
	ARG_UNUSED(activity);
#endif
}

void app_cli_ui_notify_shell_connected(void)
{
#if APP_CLI_UI_HAS_LED
	led_pattern_start(2, 80, 60);
#endif
}

void app_cli_ui_notify_shell_disconnected(void)
{
#if APP_CLI_UI_HAS_LED
	led_pattern_start(1, 140, 0);
#endif
}

bool app_cli_ui_bt_attr_is_nus(const struct bt_gatt_attr *attr)
{
	if (attr == NULL) {
		return false;
	}

	if (bt_uuid_cmp(attr->uuid, BT_UUID_NUS_SERVICE) == 0) {
		return true;
	}

	if (bt_uuid_cmp(attr->uuid, BT_UUID_NUS_RX) == 0) {
		return true;
	}

	if (bt_uuid_cmp(attr->uuid, BT_UUID_NUS_TX) == 0) {
		return true;
	}

	return false;
}
