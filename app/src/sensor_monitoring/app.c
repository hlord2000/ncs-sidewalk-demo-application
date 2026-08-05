/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <sidewalk.h>
#if defined(CONFIG_SID_END_DEVICE_CLI) || defined(CONFIG_SID_END_DEVICE_NUS_SHELL)
#include <cli/app_cli_ui.h>
#endif
#if defined(CONFIG_SID_END_DEVICE_CLI)
#include <cli/app_dut.h>
#endif
#include <sensor_monitoring/app_tx.h>
#include <sensor_monitoring/app_rx.h>
#include <sensor_monitoring/app_buttons.h>
#include <sensor_monitoring/app_leds.h>
#if IS_ENABLED(CONFIG_SID_END_DEVICE_SENSOR_NFC_ID)
#include <sensor_monitoring/app_nfc.h>
#endif
#include <app_ble_config.h>
#include <sidewalk_dfu/nordic_dfu.h>
#include <app_subGHz_config.h>
#include <sid_hal_reset_ifc.h>
#include <sid_hal_memory_ifc.h>
#include <buttons.h>
#include <zephyr/kernel.h>
#include <zephyr/smf.h>
#include <zephyr/logging/log.h>
#include <sid_demo_parser.h>
#include <json_printer/sidTypes2str.h>
#if IS_ENABLED(CONFIG_SID_END_DEVICE_SENSOR_SAMPLE_LOG) && IS_ENABLED(CONFIG_USE_SEGGER_RTT)
#include <SEGGER_RTT.h>
#endif
#ifdef CONFIG_SIDEWALK_FILE_TRANSFER_DFU
#include <sbdt/dfu_file_transfer.h>
#endif

#include <bt_app_callbacks.h>

LOG_MODULE_REGISTER(app, CONFIG_SIDEWALK_LOG_LEVEL);

#if IS_ENABLED(CONFIG_SID_END_DEVICE_SENSOR_SAMPLE_LOG) && IS_ENABLED(CONFIG_USE_SEGGER_RTT)
static void app_rtt_trace(const char *line)
{
	static bool initialized;

	if (!initialized) {
		SEGGER_RTT_Init();
		initialized = true;
	}

	(void)SEGGER_RTT_WriteString(0, line);
}
#else
static void app_rtt_trace(const char *line)
{
	ARG_UNUSED(line);
}
#endif

#define PARAM_UNUSED (0U)
#define NOTIFY_TIMER_INITIAL_DELAY_MS (5000)

K_THREAD_STACK_DEFINE(app_tx_stack, CONFIG_SID_END_DEVICE_TX_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(app_rx_stack, CONFIG_SID_END_DEVICE_RX_THREAD_STACK_SIZE);

static struct k_thread app_main;
static struct k_thread app_rx;

#if IS_ENABLED(CONFIG_SID_END_DEVICE_SENSOR_AUTO_TX)
static void notify_timer_cb(struct k_timer *timer_id);
K_TIMER_DEFINE(notify_timer, notify_timer_cb, NULL);

static void notify_timer_cb(struct k_timer *timer_id)
{
	ARG_UNUSED(timer_id);
	app_tx_event_send(APP_EVENT_NOTIFY_SENSOR);
}
#endif

#if IS_ENABLED(CONFIG_SID_END_DEVICE_SENSOR_LINK_AUTOSWITCH)
static void sensor_auto_switch_to_fsk(struct k_work *work);
static void sensor_auto_switch_to_lora(struct k_work *work);

static K_WORK_DELAYABLE_DEFINE(sensor_switch_to_fsk_work, sensor_auto_switch_to_fsk);
static K_WORK_DELAYABLE_DEFINE(sensor_switch_to_lora_work, sensor_auto_switch_to_lora);

static void sensor_auto_switch_to_fsk(struct k_work *work)
{
	ARG_UNUSED(work);

#if defined(CONFIG_SID_END_DEVICE_CLI)
	if (dut_flow_mode_is_enabled()) {
		LOG_INF("Sensor auto-switch to FSK skipped while hello-flow owns Sidewalk");
		return;
	}
#endif

	LOG_INF("Sensor auto-switch to FSK");
	app_rtt_trace("app sensor auto-switch to FSK\n");
	(void)sidewalk_event_send(sidewalk_event_link_switch, NULL, NULL);
}

static void sensor_auto_switch_to_lora(struct k_work *work)
{
	ARG_UNUSED(work);

#if defined(CONFIG_SID_END_DEVICE_CLI)
	if (dut_flow_mode_is_enabled()) {
		LOG_INF("Sensor auto-switch to BLE+LoRa skipped while hello-flow owns Sidewalk");
		return;
	}
#endif

	LOG_INF("Sensor auto-switch to BLE+LoRa");
	app_rtt_trace("app sensor auto-switch to BLE+LoRa\n");
	(void)sidewalk_event_send(sidewalk_event_link_switch, NULL, NULL);
}
#endif

static sidewalk_ctx_t sid_ctx;

static void on_sidewalk_event(bool in_isr, void *context)
{
	int err = sidewalk_event_send(sidewalk_event_process, NULL, NULL);
	if (err) {
		LOG_ERR("Send event err %d", err);
	};
}

static void on_sidewalk_msg_received(const struct sid_msg_desc *msg_desc, const struct sid_msg *msg,
				     void *context)
{
	LOG_DBG("Received message(type: %d, link_mode: %d, id: %u size %u)", (int)msg_desc->type,
		(int)msg_desc->link_mode, msg_desc->id, msg->size);
	LOG_HEXDUMP_INF((uint8_t *)msg->data, msg->size, "Received message: ");

#if defined(CONFIG_SID_END_DEVICE_NUS_SHELL)
	app_cli_ui_report_event("rx", (int)msg->size);
#endif

	if (msg_desc->type == SID_MSG_TYPE_RESPONSE && msg_desc->msg_desc_attr.rx_attr.is_msg_ack) {
		LOG_DBG("Received Ack for msg id %d", msg_desc->id);
	} else {
		struct app_rx_msg rx_msg = { 0 };
		rx_msg.pld_size = MIN(msg->size, APP_RX_PAYLOAD_MAX_SIZE);
		memcpy(rx_msg.rx_payload, msg->data, rx_msg.pld_size);
		int err = app_rx_msg_received(&rx_msg);
		if (err) {
			LOG_ERR("Rx msg err %d", err);
		}
	}
}

static void on_sidewalk_msg_sent(const struct sid_msg_desc *msg_desc, void *context)
{
	LOG_INF("Message send success");
	LOG_DBG("sent message(type: %d, id: %u)", (int)msg_desc->type, msg_desc->id);
#if defined(CONFIG_SID_END_DEVICE_NUS_SHELL)
	app_cli_ui_report_event("tx", (int)msg_desc->id);
#endif
#if defined(CONFIG_SID_END_DEVICE_CLI)
	dut_flow_notify_msg_sent(msg_desc);
#endif
}

static void on_sidewalk_send_error(sid_error_t error, const struct sid_msg_desc *msg_desc,
				   void *context)
{
	LOG_ERR("Send message err %d (%s)", (int)error, SID_ERROR_T_STR(error));
	LOG_DBG("Failed to send message(type: %d, id: %u)", (int)msg_desc->type, msg_desc->id);
#if defined(CONFIG_SID_END_DEVICE_NUS_SHELL)
	app_cli_ui_report_event("err", (int)error);
#endif
#if defined(CONFIG_SID_END_DEVICE_CLI)
	dut_flow_notify_send_error(error, msg_desc);
#endif
}

static void on_sidewalk_factory_reset(void *context)
{
	ARG_UNUSED(context);

	LOG_INF("Factory reset notification received from sid api");
	if (sid_hal_reset(SID_HAL_RESET_NORMAL)) {
		LOG_WRN("Cannot reboot");
	}
}

static void on_sidewalk_status_changed(const struct sid_status *status, void *context)
{
	struct sid_status *new_status = sid_hal_malloc(sizeof(struct sid_status));
	if (!new_status) {
		LOG_ERR("Failed to allocate memory for new status value");
	} else {
		memcpy(new_status, status, sizeof(struct sid_status));
	}
	sidewalk_event_send(sidewalk_event_new_status, new_status, sid_hal_free);

	int err = 0;
	switch (status->state) {
	case SID_STATE_READY:
	case SID_STATE_SECURE_CHANNEL_READY:
		LOG_INF("Status changed: ready");
		break;
	case SID_STATE_NOT_READY:
		LOG_INF("Status changed: not ready");
		break;
	case SID_STATE_ERROR:
		LOG_INF("Status not changed: error");
		break;
	}

	app_tx_last_link_mask_set(status->detail.link_status_mask);

	if (SID_STATUS_TIME_SYNCED == status->detail.time_sync_status) {
		err = app_tx_event_send(APP_EVENT_TIME_SYNC_SUCCESS);
	} else {
		err = app_tx_event_send(APP_EVENT_TIME_SYNC_FAIL);
	}

	if (err) {
		LOG_ERR("Send event err %d", err);
	}

	LOG_INF("Device %sregistered, Time Sync %s, Link status: {BLE: %s, FSK: %s, LoRa: %s}",
		(SID_STATUS_REGISTERED == status->detail.registration_status) ? "Is " : "Un",
		(SID_STATUS_TIME_SYNCED == status->detail.time_sync_status) ? "Success" : "Fail",
		(status->detail.link_status_mask & SID_LINK_TYPE_1) ? "Up" : "Down",
		(status->detail.link_status_mask & SID_LINK_TYPE_2) ? "Up" : "Down",
		(status->detail.link_status_mask & SID_LINK_TYPE_3) ? "Up" : "Down");

#if defined(CONFIG_SID_END_DEVICE_NUS_SHELL)
	/* Push the same status to a connected Web Bluetooth client. */
	app_cli_ui_report_status(SID_STATUS_REGISTERED == status->detail.registration_status,
				 SID_STATUS_TIME_SYNCED == status->detail.time_sync_status,
				 status->detail.link_status_mask);
#endif

	for (int i = 0; i < SID_LINK_TYPE_MAX_IDX; i++) {
		enum sid_link_mode mode =
			(enum sid_link_mode)status->detail.supported_link_modes[i];

		if (mode) {
			LOG_INF("Link mode on %s = {Cloud: %s, Mobile: %s}",
				(SID_LINK_TYPE_1_IDX == i) ? "BLE" :
				(SID_LINK_TYPE_2_IDX == i) ? "FSK" :
				(SID_LINK_TYPE_3_IDX == i) ? "LoRa" :
							     "unknow",
				(mode & SID_LINK_MODE_CLOUD) ? "True" : "False",
				(mode & SID_LINK_MODE_MOBILE) ? "True" : "False");
		}
	}
}

static void app_event_exit_dfu_mode(sidewalk_ctx_t *sid, void *ctx)
{
	int err = -ENOTSUP;
	// Exit from DFU state
#if defined(CONFIG_SIDEWALK_DFU_SERVICE_BLE)
	err = nordic_dfu_ble_stop();
#endif
	if (err) {
		LOG_ERR("dfu stop err %d", err);
	}
}

static void app_event_enter_dfu_mode(sidewalk_ctx_t *sid, void *ctx)
{
	int err = -ENOTSUP;

	LOG_INF("Entering into DFU mode");
#if defined(CONFIG_SIDEWALK_DFU_SERVICE_BLE)
	err = nordic_dfu_ble_start();
#endif
	if (err) {
		LOG_ERR("dfu start err %d", err);
	}
}

static void app_btn_dfu_state(uint32_t unused)
{
	ARG_UNUSED(unused);
	static bool go_to_dfu_state = true;
	if (go_to_dfu_state) {
		sidewalk_event_send(app_event_enter_dfu_mode, NULL, NULL);
	} else {
		sidewalk_event_send(app_event_exit_dfu_mode, NULL, NULL);
	}

	go_to_dfu_state = !go_to_dfu_state;
}

#if defined(CONFIG_BOARD_SIDEWALK_DEVKIT_NRF54L15_NRF54L15_CPUAPP) && \
	defined(CONFIG_SID_END_DEVICE_CLI)
static void app_btn_shiphold_state(uint32_t unused)
{
	ARG_UNUSED(unused);

	int err = app_cli_ui_request_ship_mode();

	if (err) {
		LOG_ERR("shiphold request err %d", err);
	}
}
#endif

static void app_btn_factory_reset(uint32_t unused)
{
	ARG_UNUSED(unused);
	(void)sidewalk_event_send(sidewalk_event_factory_reset, NULL, NULL);
}

static void app_btn_link_switch(uint32_t unused)
{
	ARG_UNUSED(unused);
	(void)sidewalk_event_send(sidewalk_event_link_switch, NULL, NULL);
}

static int app_buttons_init(void)
{
	button_set_action_short_press(DK_BTN1, app_btn_event_handler, DEMO_BTN_ID_0);
	button_set_action_short_press(DK_BTN2, app_btn_event_handler, DEMO_BTN_ID_1);
	button_set_action_short_press(DK_BTN3, app_btn_event_handler, DEMO_BTN_ID_2);
	button_set_action_short_press(DK_BTN4, app_btn_event_handler, DEMO_BTN_ID_3);

#if defined(CONFIG_BOARD_SIDEWALK_DEVKIT_NRF54L15_NRF54L15_CPUAPP) && \
	defined(CONFIG_SID_END_DEVICE_CLI)
	button_set_action_long_press(DK_BTN1, app_btn_shiphold_state, PARAM_UNUSED);
#else
	button_set_action_long_press(DK_BTN1, app_btn_dfu_state, PARAM_UNUSED);
#endif
	button_set_action_long_press(DK_BTN2, app_btn_factory_reset, PARAM_UNUSED);
	button_set_action_long_press(DK_BTN3, app_btn_link_switch, PARAM_UNUSED);

	return buttons_init();
}

void app_start_tasks(void)
{
	(void)k_thread_create(&app_main, app_tx_stack, K_THREAD_STACK_SIZEOF(app_tx_stack),
			      app_tx_task, NULL, NULL, NULL,
			      CONFIG_SID_END_DEVICE_TX_THREAD_PRIORITY, 0, K_NO_WAIT);

	(void)k_thread_create(&app_rx, app_rx_stack, K_THREAD_STACK_SIZEOF(app_rx_stack),
			      app_rx_task, NULL, NULL, NULL,
			      CONFIG_SID_END_DEVICE_RX_THREAD_PRIORITY, 0, K_NO_WAIT);

	k_thread_name_set(&app_main, "app_main");
	k_thread_name_set(&app_rx, "app_rx");
}

static bool gatt_authorize(struct bt_conn *conn, const struct bt_gatt_attr *attr)
{
	struct bt_conn_info cinfo = {};
	int ret = bt_conn_get_info(conn, &cinfo);
	if (ret != 0) {
		LOG_ERR("Failed to get id of connection err %d", ret);
		return false;
	}

	if (cinfo.id == BT_ID_SIDEWALK) {
		bool shell_attr = sid_ble_bt_attr_is_SMP(attr);

#if defined(CONFIG_SID_END_DEVICE_NUS_SHELL)
		shell_attr = shell_attr || app_cli_ui_bt_attr_is_nus(attr);
#endif

		if (shell_attr) {
			return false;
		}
	}

#if defined(CONFIG_SIDEWALK_DFU)
	if (cinfo.id == BT_ID_SMP_DFU) {
		if (sid_ble_bt_attr_is_SIDEWALK(attr)) {
			return false;
		}
	}
#endif //defined(CONFIG_SIDEWALK_DFU)
	return true;
}

static const struct bt_gatt_authorization_cb gatt_authorization_callbacks = {
	.read_authorize = gatt_authorize,
	.write_authorize = gatt_authorize,
};

#define MAX_TIME_SYNC_INTERVALS 10

static uint16_t default_sync_intervals_h[MAX_TIME_SYNC_INTERVALS] = { 2, 4, 8,
								      12 }; // default GCS intervals
static struct sid_time_sync_config default_time_sync_config = {
	.adaptive_sync_intervals_h = default_sync_intervals_h,
	.num_intervals = sizeof(default_sync_intervals_h) / sizeof(default_sync_intervals_h[0]),
};

void app_start(void)
{
	app_rtt_trace("app_start begin\n");

	if (app_buttons_init()) {
		LOG_ERR("Cannot init buttons");
		app_rtt_trace("app_start buttons init failed\n");
	}
	app_rtt_trace("app_start buttons ready\n");

	if (app_led_init()) {
		LOG_ERR("Cannot init leds");
		app_rtt_trace("app_start leds init failed\n");
	}
	app_rtt_trace("app_start leds ready\n");

#if IS_ENABLED(CONFIG_SID_END_DEVICE_SENSOR_NFC_ID)
	if (app_nfc_init()) {
		LOG_ERR("Cannot init NFC identity tag");
		app_rtt_trace("app_start nfc init failed\n");
	}
	app_rtt_trace("app_start nfc ready\n");
#endif

	static struct sid_event_callbacks event_callbacks = {
		.context = &sid_ctx,
		.on_event = on_sidewalk_event,
		.on_msg_received = on_sidewalk_msg_received,
		.on_msg_sent = on_sidewalk_msg_sent,
		.on_send_error = on_sidewalk_send_error,
		.on_status_changed = on_sidewalk_status_changed,
		.on_factory_reset = on_sidewalk_factory_reset,
	};

	struct sid_end_device_characteristics dev_ch = {
		.type = SID_END_DEVICE_TYPE_STATIC,
		.power_type = SID_END_DEVICE_POWERED_BY_BATTERY_AND_LINE_POWER,
		.qualification_id = 0x0001,
	};

	sid_ctx.config = (struct sid_config){
		.link_mask = 0,
		.dev_ch = dev_ch,
		.callbacks = &event_callbacks,
		.link_config = app_get_ble_config(),
		.sub_ghz_link_config = app_get_sub_ghz_config(),
		.log_config = NULL,
		.time_sync_config = &default_time_sync_config,
	};

	int err = bt_gatt_authorization_cb_register(&gatt_authorization_callbacks);
	if (err) {
		LOG_ERR("Registering GATT authorization callbacks failed (err %d)", err);
		app_rtt_trace("app_start gatt auth failed\n");
		return;
	}
	app_rtt_trace("app_start gatt auth ready\n");

#if defined(CONFIG_SID_END_DEVICE_NUS_SHELL)
	err = app_cli_ui_init(&sid_ctx);
	if (err) {
		LOG_ERR("Failed to initialize NUS shell advertiser (err %d)", err);
		app_rtt_trace("app_start nus shell failed\n");
	} else {
		app_rtt_trace("app_start nus shell ready\n");
	}
#endif

	app_start_tasks();
	app_rtt_trace("app_start tasks started\n");
	sidewalk_start(&sid_ctx);
	app_rtt_trace("app_start sidewalk thread started\n");
	sidewalk_event_send(sidewalk_event_platform_init, NULL, NULL);
	sidewalk_event_send(sidewalk_event_autostart, NULL, NULL);
	app_rtt_trace("app_start sidewalk events queued\n");

#if IS_ENABLED(CONFIG_SID_END_DEVICE_SENSOR_LINK_AUTOSWITCH)
	k_work_schedule(&sensor_switch_to_fsk_work,
			K_SECONDS(CONFIG_SID_END_DEVICE_SENSOR_AUTOSWITCH_FSK_DELAY_S));
	k_work_schedule(&sensor_switch_to_lora_work,
			K_SECONDS(CONFIG_SID_END_DEVICE_SENSOR_AUTOSWITCH_FSK_DELAY_S +
				  CONFIG_SID_END_DEVICE_SENSOR_AUTOSWITCH_LORA_DELAY_S));
#endif

#if IS_ENABLED(CONFIG_SID_END_DEVICE_SENSOR_AUTO_TX)
	k_timer_start(&notify_timer, K_MSEC(NOTIFY_TIMER_INITIAL_DELAY_MS),
		      K_MSEC(CONFIG_SID_END_DEVICE_NOTIFY_DATA_PERIOD_MS));
	app_rtt_trace("app_start notify timer started\n");
#else
	app_rtt_trace("app_start notify timer disabled\n");
#endif
}
