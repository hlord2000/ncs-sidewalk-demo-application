/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <cli/app_cli_ui.h>

#include <errno.h>
#include <string.h>

#include <bluetooth/services/nus.h>
#include <dk_buttons_and_leds.h>
#include <shell/shell_bt_nus.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_backend.h>

#include <bt_app_callbacks.h>
#include <cli/prov_status.h>
#include <json_printer/sidTypes2str.h>
#include <sid_api.h>
#include <sid_error.h>
#include <sid_pal_mfg_store_ifc.h>
#include <sid_pal_radio_ifc.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(app_cli_ui, CONFIG_SIDEWALK_LOG_LEVEL);

#define APP_CLI_UI_LED_ID DK_LED1
#define APP_CLI_UI_CONN_LATENCY 0
#define APP_CLI_UI_CONN_TIMEOUT_10MS 400
#define APP_CLI_UI_BOOT_BLINK_PULSES 3
#define APP_CLI_UI_BOOT_BLINK_ON_MS 90
#define APP_CLI_UI_BOOT_BLINK_OFF_MS 90
#define APP_CLI_UI_SHIP_BLINK_PULSES 1
#define APP_CLI_UI_SHIP_BLINK_ON_MS 200
#define APP_CLI_UI_SHIP_BLINK_OFF_MS 0
#define APP_CLI_UI_SHIP_DELAY_MS APP_CLI_UI_SHIP_BLINK_ON_MS
#define APP_CLI_UI_NPM1300_LED_COUNT 3
#define APP_CLI_UI_COMPANY_ID_LSB 0x59
#define APP_CLI_UI_COMPANY_ID_MSB 0x00
#define APP_CLI_UI_IDENTITY_FORMAT_VERSION 0x01
#define APP_CLI_UI_IDENTITY_FINGERPRINT_SIZE 8
#define APP_CLI_UI_IDENTITY_MFG_DATA_SIZE                                                   \
	(3 + APP_CLI_UI_IDENTITY_FINGERPRINT_SIZE)
#define APP_CLI_UI_LEGACY_ADV_MAX_SIZE 31
#define APP_CLI_UI_FLAGS_AD_SIZE 3
#define APP_CLI_UI_NAME_AD_OVERHEAD 2
#define APP_CLI_UI_MFG_AD_OVERHEAD 2
#define APP_CLI_UI_IDENTITY_NAME_MAX_SIZE                                                  \
	(APP_CLI_UI_LEGACY_ADV_MAX_SIZE - APP_CLI_UI_FLAGS_AD_SIZE -                         \
	 APP_CLI_UI_NAME_AD_OVERHEAD - APP_CLI_UI_MFG_AD_OVERHEAD -                         \
	 APP_CLI_UI_IDENTITY_MFG_DATA_SIZE)

#define MS_TO_CONN_INTERVAL(ms) ((uint16_t)(((ms) * 4) / 5))
#define MS_TO_ADV_INTERVAL(ms) ((uint16_t)(((ms) * 8) / 5))

#define APP_CLI_UI_LONGPRESS_NODE DT_NODELABEL(app_cli_ui_longpress)
#define APP_CLI_UI_NPM1300_LEDS_NODE DT_NODELABEL(npm1300_leds)
#define APP_CLI_UI_NPM1300_REGULATORS_NODE DT_NODELABEL(npm1300_regulators)

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

#if IS_ENABLED(CONFIG_LED) && DT_NODE_HAS_STATUS(APP_CLI_UI_NPM1300_LEDS_NODE, okay)
#define APP_CLI_UI_HAS_NPM1300_LEDS 1
static const struct device *const npm1300_leds = DEVICE_DT_GET(APP_CLI_UI_NPM1300_LEDS_NODE);
#else
#define APP_CLI_UI_HAS_NPM1300_LEDS 0
#endif

#if DT_HAS_ALIAS(led0) && DT_NODE_HAS_STATUS(DT_ALIAS(led0), okay)
#define APP_CLI_UI_HAS_DK_LED 1
#else
#define APP_CLI_UI_HAS_DK_LED 0
#endif

#define APP_CLI_UI_HAS_LED (APP_CLI_UI_HAS_NPM1300_LEDS || APP_CLI_UI_HAS_DK_LED)

#if DT_NODE_HAS_STATUS(APP_CLI_UI_LONGPRESS_NODE, okay)
#define APP_CLI_UI_HAS_LONGPRESS_INPUT 1
static const struct device *const longpress_dev = DEVICE_DT_GET(APP_CLI_UI_LONGPRESS_NODE);
#else
#define APP_CLI_UI_HAS_LONGPRESS_INPUT 0
#endif

#if DT_NODE_HAS_STATUS(APP_CLI_UI_NPM1300_REGULATORS_NODE, okay)
#define APP_CLI_UI_HAS_NPM1300_SHIP 1
static const struct device *const npm1300_regulators =
	DEVICE_DT_GET(APP_CLI_UI_NPM1300_REGULATORS_NODE);
#else
#define APP_CLI_UI_HAS_NPM1300_SHIP 0
#endif

#if DT_HAS_ALIAS(lora_transceiver) && DT_NODE_HAS_STATUS(DT_ALIAS(lora_transceiver), okay) && \
	DT_NODE_HAS_PROP(DT_ALIAS(lora_transceiver), reset_gpios)
#define APP_CLI_UI_HAS_LORA_RESET 1
static const struct gpio_dt_spec lora_reset =
	GPIO_DT_SPEC_GET(DT_ALIAS(lora_transceiver), reset_gpios);
#else
#define APP_CLI_UI_HAS_LORA_RESET 0
#endif

struct led_pattern_state {
	uint16_t on_ms;
	uint16_t off_ms;
	uint8_t remaining_pulses;
	bool led_on;
};

struct sidewalk_ship_ctx {
	struct k_sem done;
};

static sidewalk_ctx_t *sidewalk_ctx;

#if APP_CLI_UI_HAS_LED
static struct led_pattern_state led_pattern;
static bool led_indications_ready;
static bool led_boot_indication_started;
static void led_pattern_start(uint8_t pulses, uint16_t on_ms, uint16_t off_ms);
static void led_pattern_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(led_pattern_work, led_pattern_work_handler);
#endif

static atomic_t ship_mode_requested = ATOMIC_INIT(false);
static void ship_mode_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(ship_mode_work, ship_mode_work_handler);

#if defined(CONFIG_SID_END_DEVICE_NUS_SHELL)
static struct bt_le_ext_adv *nus_adv;
static struct bt_conn *nus_conn;
static atomic_t nus_restart_pending = ATOMIC_INIT(false);
static void nus_adv_restart_work_handler(struct k_work *work);
static K_WORK_DEFINE(nus_adv_restart_work, nus_adv_restart_work_handler);
static uint8_t nus_identity_smsn[SID_PAL_MFG_STORE_SMSN_SIZE];
static uint8_t nus_identity_mfg_data[APP_CLI_UI_IDENTITY_MFG_DATA_SIZE] = {
	APP_CLI_UI_COMPANY_ID_LSB,
	APP_CLI_UI_COMPANY_ID_MSB,
	APP_CLI_UI_IDENTITY_FORMAT_VERSION,
};
static bool nus_identity_ready;

BUILD_ASSERT(sizeof(CONFIG_SID_END_DEVICE_NUS_DEVICE_NAME) - 1 <=
		     APP_CLI_UI_IDENTITY_NAME_MAX_SIZE,
	     "NUS device name must fit beside flags and identity in legacy advertising data");

static const struct bt_data nus_ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_SID_END_DEVICE_NUS_DEVICE_NAME,
		sizeof(CONFIG_SID_END_DEVICE_NUS_DEVICE_NAME) - 1),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, nus_identity_mfg_data,
		sizeof(nus_identity_mfg_data)),
};

static const struct bt_data nus_sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
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
	if (!led_indications_ready) {
		return;
	}

#if APP_CLI_UI_HAS_NPM1300_LEDS
	if (device_is_ready(npm1300_leds)) {
		for (uint8_t led = 0U; led < APP_CLI_UI_NPM1300_LED_COUNT; led++) {
			if (on) {
				(void)led_on(npm1300_leds, led);
			} else {
				(void)led_off(npm1300_leds, led);
			}
		}

		return;
	}
#endif

#if APP_CLI_UI_HAS_DK_LED
	if (on) {
		dk_set_led_on(APP_CLI_UI_LED_ID);
	} else {
		dk_set_led_off(APP_CLI_UI_LED_ID);
	}
#endif
}

static int led_indications_init(void)
{
	if (led_indications_ready) {
		return 0;
	}

#if APP_CLI_UI_HAS_NPM1300_LEDS
	if (device_is_ready(npm1300_leds)) {
		led_indications_ready = true;
		led_apply(false);
		LOG_INF("nPM1300 LED boot indications ready");
		return 0;
	}

	LOG_WRN("nPM1300 LED device is not ready");
#endif

#if APP_CLI_UI_HAS_DK_LED
	if (dk_leds_init() == 0) {
		led_indications_ready = true;
		led_apply(false);
		LOG_INF("DK LED boot indications ready");
		return 0;
	}

	LOG_WRN("Failed to initialize DK LED indications");
#endif

	return -ENODEV;
}

static void led_boot_indication_start_once(void)
{
	(void)led_indications_init();

	if (!led_boot_indication_started && led_indications_ready) {
		led_boot_indication_started = true;
		led_pattern_start(APP_CLI_UI_BOOT_BLINK_PULSES, APP_CLI_UI_BOOT_BLINK_ON_MS,
				  APP_CLI_UI_BOOT_BLINK_OFF_MS);
	}
}

static void led_pattern_start(uint8_t pulses, uint16_t on_ms, uint16_t off_ms)
{
	if (pulses == 0U || !led_indications_ready) {
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

static void led_prepare_for_ship(void)
{
	(void)k_work_cancel_delayable(&led_pattern_work);
	led_apply(false);
}
#endif

static void sidewalk_prepare_for_ship(sidewalk_ctx_t *sid, void *ctx)
{
	struct sidewalk_ship_ctx *ship_ctx = ctx;

	if (sid && sid->handle) {
		uint32_t link_mask = sid->config.link_mask ? sid->config.link_mask : SID_LINK_TYPE_ANY;
		sid_error_t err = sid_stop(sid->handle, link_mask);

		LOG_INF("sid_stop before ship mode returned %d (%s)", err, SID_ERROR_T_STR(err));

		err = sid_deinit(sid->handle);
		LOG_INF("sid_deinit before ship mode returned %d (%s)", err, SID_ERROR_T_STR(err));
		sid->handle = NULL;
	}

	k_sem_give(&ship_ctx->done);
}

static void prepare_radio_for_ship(void)
{
#if defined(CONFIG_SIDEWALK_SUBGHZ_RADIO_SX126X)
	int32_t err = sid_pal_radio_sleep(0);

	if (err != RADIO_ERROR_NONE) {
		LOG_WRN("sid_pal_radio_sleep before ship mode returned %d", err);
	}
#endif

#if APP_CLI_UI_HAS_LORA_RESET
	if (device_is_ready(lora_reset.port)) {
		(void)gpio_pin_configure_dt(&lora_reset, GPIO_OUTPUT_ACTIVE);
	}
#endif
}

static void wait_for_ship_button_release(void)
{
#if APP_CLI_UI_HAS_SINGLE_BUTTON
	for (uint8_t i = 0; i < 100U; i++) {
		int state = gpio_pin_get_dt(&sw0);

		if (state <= 0) {
			return;
		}

		k_sleep(K_MSEC(20));
	}

	LOG_WRN("Ship button still active before nPM1300 ship mode");
#endif
}

static int enter_ship_mode(void)
{
#if !APP_CLI_UI_HAS_NPM1300_SHIP
	LOG_ERR("nPM1300 regulator parent is not available");
	return -ENODEV;
#else
	if (!device_is_ready(npm1300_regulators)) {
		LOG_ERR("nPM1300 regulator parent is not ready");
		return -ENODEV;
	}

	if (sidewalk_ctx != NULL) {
		struct sidewalk_ship_ctx ship_ctx;

		k_sem_init(&ship_ctx.done, 0, 1);

		if (sidewalk_event_send(sidewalk_prepare_for_ship, &ship_ctx, NULL) == 0) {
			(void)k_sem_take(&ship_ctx.done, K_MSEC(1500));
		}
	}

	prepare_radio_for_ship();
	wait_for_ship_button_release();
#if APP_CLI_UI_HAS_LED
	led_prepare_for_ship();
#endif

	LOG_INF("Requesting nPM1300 ship mode");
	int err = regulator_parent_ship_mode(npm1300_regulators);

	if (err) {
		LOG_ERR("nPM1300 ship mode request failed (%d)", err);
		return err;
	}

	k_sleep(K_SECONDS(1));
	LOG_WRN("Still running after nPM1300 ship mode request; check VBUS/SHPHLD hardware");
	return 0;
#endif
}

static void ship_mode_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	int err = enter_ship_mode();

	if (err) {
		LOG_ERR("nPM1300 ship mode work failed (%d)", err);
	}

	atomic_set(&ship_mode_requested, false);
}

int app_cli_ui_request_ship_mode(void)
{
#if !APP_CLI_UI_HAS_NPM1300_SHIP
	return -ENODEV;
#else
	if (!device_is_ready(npm1300_regulators)) {
		return -ENODEV;
	}

	if (!atomic_cas(&ship_mode_requested, false, true)) {
		return -EALREADY;
	}

	LOG_INF("Shiphold requested, blinking before nPM1300 ship mode");
#if APP_CLI_UI_HAS_LED
	led_pattern_start(APP_CLI_UI_SHIP_BLINK_PULSES, APP_CLI_UI_SHIP_BLINK_ON_MS,
			  APP_CLI_UI_SHIP_BLINK_OFF_MS);
#endif
	(void)k_work_schedule(&ship_mode_work, K_MSEC(APP_CLI_UI_SHIP_DELAY_MS));
	return 0;
#endif
}

#if APP_CLI_UI_HAS_LONGPRESS_INPUT
static void shiphold_input_callback(struct input_event *evt, void *user_data)
{
	ARG_UNUSED(user_data);

	if (evt->type != INPUT_EV_KEY || evt->code != INPUT_KEY_POWER || evt->value == 0) {
		return;
	}

	int err = app_cli_ui_request_ship_mode();

	if (err) {
		LOG_ERR("Failed to request nPM1300 ship mode from long press (%d)", err);
	}
}

INPUT_CALLBACK_DEFINE(longpress_dev, shiphold_input_callback, NULL);
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

static bool identity_is_uniform(uint8_t value)
{
	for (size_t i = 0; i < ARRAY_SIZE(nus_identity_smsn); i++) {
		if (nus_identity_smsn[i] != value) {
			return false;
		}
	}

	return true;
}

static int nus_identity_init(void)
{
	uint16_t smsn_size = sid_pal_mfg_store_get_length_for_value(SID_PAL_MFG_STORE_SMSN);

	if (smsn_size != sizeof(nus_identity_smsn)) {
		LOG_ERR("Sidewalk SMSN has invalid size %u", smsn_size);
		return -ENOENT;
	}

	sid_pal_mfg_store_read(SID_PAL_MFG_STORE_SMSN, nus_identity_smsn,
			       sizeof(nus_identity_smsn));
	if (identity_is_uniform(0x00) || identity_is_uniform(0xff)) {
		LOG_ERR("Sidewalk SMSN is not initialized");
		return -ENOENT;
	}

	memcpy(&nus_identity_mfg_data[3], nus_identity_smsn,
	       APP_CLI_UI_IDENTITY_FINGERPRINT_SIZE);
	nus_identity_ready = true;
	LOG_HEXDUMP_INF(&nus_identity_mfg_data[3], APP_CLI_UI_IDENTITY_FINGERPRINT_SIZE,
			"NUS advertised Sidewalk identity fingerprint");

	return 0;
}

static int nus_adv_start(void)
{
	if (!nus_identity_ready) {
		int err = nus_identity_init();

		if (err) {
			return err;
		}
	}

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
	prov_status_notify_nus_connected();
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
	int err = nus_identity_init();

	if (err) {
		return err;
	}

	err = sid_ble_bt_enable(NULL);

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

int app_cli_ui_print_identity(const struct shell *shell)
{
#if defined(CONFIG_SID_END_DEVICE_NUS_SHELL)
	char smsn_hex[(SID_PAL_MFG_STORE_SMSN_SIZE * 2) + 1];
	char fingerprint_hex[(APP_CLI_UI_IDENTITY_FINGERPRINT_SIZE * 2) + 1];

	if (shell == NULL) {
		return -EINVAL;
	}

	if (!nus_identity_ready) {
		int err = nus_identity_init();

		if (err) {
			shell_error(shell, "Sidewalk identity is unavailable (%d)", err);
			return err;
		}
	}

	for (size_t i = 0; i < sizeof(nus_identity_smsn); i++) {
		snprintk(&smsn_hex[i * 2], 3, "%02X", nus_identity_smsn[i]);
	}

	for (size_t i = 0; i < APP_CLI_UI_IDENTITY_FINGERPRINT_SIZE; i++) {
		snprintk(&fingerprint_hex[i * 2], 3, "%02X", nus_identity_smsn[i]);
	}

	shell_print(shell, "Sidewalk manufacturing serial: %s", smsn_hex);
	shell_print(shell, "Advertised identity fingerprint: %s", fingerprint_hex);
	shell_print(shell, "EVT:{\"t\":\"identity\",\"smsn\":\"%s\",\"fp\":\"%s\"}",
		    smsn_hex, fingerprint_hex);

	return 0;
#else
	ARG_UNUSED(shell);
	return -ENOTSUP;
#endif
}

int app_cli_ui_nus_shell_init(void)
{
#if APP_CLI_UI_HAS_LED
	led_boot_indication_start_once();
#endif

#if defined(CONFIG_SID_END_DEVICE_NUS_SHELL)
	return nus_shell_init();
#else
	return 0;
#endif
}

int app_cli_ui_init(sidewalk_ctx_t *sid)
{
	sidewalk_ctx = sid;

#if APP_CLI_UI_HAS_LED
	led_boot_indication_start_once();
#endif

#if APP_CLI_UI_HAS_SINGLE_BUTTON
	LOG_INF("Button input on %s pin %u flags 0x%x", sw0.port->name, sw0.pin, sw0.dt_flags);
#endif

	atomic_set(&ship_mode_requested, false);
#if APP_CLI_UI_HAS_LONGPRESS_INPUT
	if (!device_is_ready(longpress_dev)) {
		LOG_WRN("Long-press input device is not ready");
	} else {
		LOG_INF("Long-press nPM1300 ship-mode input ready");
	}
#endif

	return app_cli_ui_nus_shell_init();
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

#if defined(CONFIG_SID_END_DEVICE_NUS_SHELL)
static const struct shell *app_cli_ui_nus_shell(void)
{
	return shell_backend_get_by_name("shell_bt_nus");
}
#endif

void app_cli_ui_report_status(bool registered, bool time_synced, uint32_t link_mask)
{
#if defined(CONFIG_SID_END_DEVICE_NUS_SHELL)
	const struct shell *sh = app_cli_ui_nus_shell();

	if (sh == NULL) {
		return;
	}

	/* shell_print() takes the shell write mutex and is a no-op until the
	 * shell is active (a NUS client is connected), so it is safe to call
	 * from the Sidewalk event thread without racing command output.
	 */
	shell_print(
		sh, "EVT:{\"t\":\"status\",\"reg\":%d,\"time\":%d,\"ble\":%d,\"fsk\":%d,\"lora\":%d}",
		registered ? 1 : 0, time_synced ? 1 : 0,
		(link_mask & SID_LINK_TYPE_1) ? 1 : 0, (link_mask & SID_LINK_TYPE_2) ? 1 : 0,
		(link_mask & SID_LINK_TYPE_3) ? 1 : 0);
#else
	ARG_UNUSED(registered);
	ARG_UNUSED(time_synced);
	ARG_UNUSED(link_mask);
#endif
}

void app_cli_ui_report_location(int status, int error, int mode, int link)
{
#if defined(CONFIG_SID_END_DEVICE_NUS_SHELL)
	const struct shell *sh = app_cli_ui_nus_shell();

	if (sh == NULL) {
		return;
	}

	shell_print(sh,
		    "EVT:{\"t\":\"location\",\"status\":%d,\"err\":%d,\"mode\":%d,\"link\":%d}",
		    status, error, mode, link);
#else
	ARG_UNUSED(status);
	ARG_UNUSED(error);
	ARG_UNUSED(mode);
	ARG_UNUSED(link);
#endif
}

void app_cli_ui_report_event(const char *type, int value)
{
#if defined(CONFIG_SID_END_DEVICE_NUS_SHELL)
	const struct shell *sh = app_cli_ui_nus_shell();

	if (sh == NULL || type == NULL) {
		return;
	}

	shell_print(sh, "EVT:{\"t\":\"%s\",\"v\":%d}", type, value);
#else
	ARG_UNUSED(type);
	ARG_UNUSED(value);
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
