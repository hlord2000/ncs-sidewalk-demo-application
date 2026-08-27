/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <sensor_monitoring/app_rx.h>
#include <sensor_monitoring/app_tx.h>
#include <sensor_monitoring/app_buttons.h>
#include <sensor_monitoring/app_leds.h>
#include <sid_demo_parser.h>
#if defined(CONFIG_SID_END_DEVICE_MEMFAULT)
#include <memfault/app_memfault.h>
#endif
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(app_rx, CONFIG_SIDEWALK_LOG_LEVEL);

#define ACTION_RESP_BTN_ALL (0xFF)
#define ACTION_REQ_LED_ALL (0xFF)

K_MSGQ_DEFINE(rx_msgq, sizeof(struct app_rx_msg), CONFIG_SID_END_DEVICE_RX_THREAD_QUEUE_SIZE, 4);

static void app_rx_button_resp_process(struct sid_demo_msg *msg)
{
	// Deserialize message
	uint8_t button_id_arr[APP_BUTTONS_MAX] = { 0 };
	struct sid_demo_action_resp action_resp = { 0 };
	action_resp.button_action_resp.button_id_arr = button_id_arr;

	static struct sid_parse_state state = { 0 };
	sid_parse_state_init(&state, msg->payload, msg->payload_size);
	sid_demo_app_action_resp_deserialize(&state, &action_resp);
	if (state.ret_code != SID_ERROR_NONE) {
		LOG_ERR("Button response de-serialize failed %d", state.ret_code);
		return;
	}

	// Process action response
	if (action_resp.resp_type != SID_DEMO_ACTION_TYPE_BUTTON) {
		LOG_ERR("Rx msg action response %d not supported", action_resp.resp_type);
		return;
	}

	LOG_INF("Button response received");
	if (action_resp.button_action_resp.num_buttons == ACTION_RESP_BTN_ALL) {
		action_resp.button_action_resp.button_id_arr = app_btn_id_array_get();
		action_resp.button_action_resp.num_buttons = APP_BUTTONS_MAX;
	}
	for (size_t i = 0; i < action_resp.button_action_resp.num_buttons; i++) {
		app_btn_press_mask_bit_clear(action_resp.button_action_resp.button_id_arr[i]);
		app_btn_notify_mask_bit_clear(action_resp.button_action_resp.button_id_arr[i]);
	}
}

/* Diagnostic downlinks.
 *
 * sid_demo's descriptor byte carries a 2 bit command class and the demo protocol
 * only ever uses class 0, so class 1 is free for device diagnostics without
 * touching the demo message definitions. The whole downlink is one byte, which
 * matters on a link whose payload budget can be 19 bytes:
 *
 *   0x28 = WRITE, class 1, cmd 0 -> crash (MEMFAULT_ASSERT)
 *   0x29 = WRITE, class 1, cmd 1 -> crash (HardFault)
 *   0x2a = WRITE, class 1, cmd 2 -> clean reboot
 *
 * The device faults, the Memfault fault handler records the reason, and the
 * reboot event is drained over Sidewalk on the next boot.
 */
#define APP_RX_DIAG_CLASS 0x1
#define APP_RX_DIAG_CMD_CRASH_ASSERT 0x0
#define APP_RX_DIAG_CMD_CRASH_HARDFAULT 0x1
#define APP_RX_DIAG_CMD_REBOOT 0x2

static void app_rx_diag_process(const struct sid_demo_msg_desc *msg_desc)
{
#if defined(CONFIG_SID_END_DEVICE_MEMFAULT)
	switch (msg_desc->cmd_id) {
	case APP_RX_DIAG_CMD_CRASH_ASSERT:
		LOG_WRN("Diagnostic downlink: crash (assert) requested");
		(void)app_memfault_trigger_crash(APP_MEMFAULT_CRASH_ASSERT);
		break;
	case APP_RX_DIAG_CMD_CRASH_HARDFAULT:
		LOG_WRN("Diagnostic downlink: crash (hardfault) requested");
		(void)app_memfault_trigger_crash(APP_MEMFAULT_CRASH_HARDFAULT);
		break;
	case APP_RX_DIAG_CMD_REBOOT:
		LOG_WRN("Diagnostic downlink: reboot requested");
		(void)app_memfault_trigger_reboot();
		break;
	default:
		LOG_ERR("Diagnostic downlink cmd %d not supported", msg_desc->cmd_id);
		break;
	}
#else
	ARG_UNUSED(msg_desc);
	LOG_WRN("Diagnostic downlink ignored: Memfault support is not enabled");
#endif
}

static void app_rx_led_req_process(struct sid_demo_msg *msg)
{
	// Deserialize message
	uint8_t led_id_arr[APP_LEDS_MAX] = { 0 };
	struct sid_demo_led_action_req led_req = { 0 };
	led_req.led_id_arr = led_id_arr;

	static struct sid_parse_state state = { 0 };
	sid_parse_state_init(&state, msg->payload, msg->payload_size);
	sid_demo_app_action_req_deserialize(&state, &led_req);
	if (state.ret_code != SID_ERROR_NONE) {
		LOG_ERR("LED request de-serialize failed %d", state.ret_code);
		return;
	}

	// Process action request
	LOG_INF("LED request received");
	if (led_req.num_leds == ACTION_REQ_LED_ALL) {
		led_req.led_id_arr = app_led_id_array_get();
		led_req.num_leds = APP_LEDS_MAX;
	}

	if (led_req.action_req == SID_DEMO_ACTION_LED_ON) {
		for (uint8_t i = 0; i < led_req.num_leds; i++) {
			app_led_turn_on((enum leds_id_t)led_req.led_id_arr[i]);
		}
		app_tx_event_send(APP_EVENT_RESP_LED_ON);
	} else if (led_req.action_req == SID_DEMO_ACTION_LED_OFF) {
		for (uint8_t i = 0; i < led_req.num_leds; i++) {
			app_led_turn_off((enum leds_id_t)led_req.led_id_arr[i]);
		}
		app_tx_event_send(APP_EVENT_RESP_LED_OFF);
	} else {
		LOG_ERR("LED request invalid action %d", led_req.action_req);
	}
}

int app_rx_msg_received(struct app_rx_msg *msg)
{
	return k_msgq_put(&rx_msgq, msg, K_NO_WAIT);
}

void app_rx_task(void *dummy1, void *dummy2, void *dummy3)
{
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);

	while (1) {
		struct app_rx_msg rx_msg = { 0 };
		int err = k_msgq_get(&rx_msgq, &rx_msg, K_FOREVER);
		if (!err) {
			// Deserialize message
			static uint8_t msg_payload[APP_RX_PAYLOAD_MAX_SIZE] = { 0 };
			struct sid_demo_msg msg = { 0 };
			msg.payload = msg_payload;
			struct sid_demo_msg_desc msg_desc = { 0 };

			static struct sid_parse_state state = { 0 };
			sid_parse_state_init(&state, rx_msg.rx_payload, rx_msg.pld_size);
			sid_demo_app_msg_deserialize(&state, &msg_desc, &msg);
			if (!state.ret_code) {
				LOG_DBG("Opc %d, class %d cmd %d status indicator %d status_code %d paylaod size %d",
					msg_desc.opc, msg_desc.cmd_class, msg_desc.cmd_id,
					msg_desc.status_hdr_ind, msg_desc.status_code,
					msg.payload_size);
			} else {
				LOG_ERR("Rx msg de-serialize failed %d", state.ret_code);
				continue;
			}

			// Diagnostics ride on their own command class, handled
			// before the demo class check below rejects it.
			if (msg_desc.cmd_class == APP_RX_DIAG_CLASS) {
				app_rx_diag_process(&msg_desc);
				continue;
			}

			// Process demo app message
			if (msg_desc.cmd_class != SID_DEMO_APP_CLASS) {
				LOG_ERR("Rx msg cmd class %d not supported", msg_desc.cmd_class);
				continue;
			}

			switch (msg_desc.opc) {
			case SID_DEMO_MSG_TYPE_RESP:
				switch (msg_desc.cmd_id) {
				case SID_DEMO_APP_CLASS_CMD_CAP_DISCOVERY_ID:
					if (msg_desc.status_hdr_ind &&
					    msg_desc.status_code == SID_ERROR_NONE &&
					    msg.payload_size == 0) {
						LOG_INF("Capability response received");
						app_tx_event_send(APP_EVENT_CAPABILITY_SUCCESS);
					} else {
						LOG_ERR("Capability failed (code %d)",
							msg_desc.status_code);
					}
					break;
				case SID_DEMO_APP_CLASS_CMD_ACTION:
					if (msg_desc.status_hdr_ind &&
					    msg_desc.status_code == SID_ERROR_NONE) {
						app_rx_button_resp_process(&msg);
					} else {
						LOG_ERR("Action response failed (code %d)",
							msg_desc.status_code);
					}
					break;
				default:
					LOG_ERR("Rx msg cmd id %d not supported", msg_desc.cmd_id);
					break;
				}
				break;
			case SID_DEMO_MSG_TYPE_WRITE:
				switch (msg_desc.cmd_id) {
				case SID_DEMO_APP_CLASS_CMD_ACTION:
					app_rx_led_req_process(&msg);
					break;
				case SID_DEMO_APP_CLASS_CMD_CAP_DISCOVERY_ID:
				default:
					LOG_ERR("Rx msg cmd id %d not supported", msg_desc.cmd_id);
					break;
				}
				break;
			case SID_DEMO_MSG_TYPE_READ:
			case SID_DEMO_MSG_TYPE_NOTIFY:
			default:
				LOG_ERR("Rx msg op code %d not supported", msg_desc.opc);
				break;
			}
		} else {
			LOG_ERR("App RX msgq err %d", err);
		}
	}

	LOG_ERR("App RX thread ends. You should never see this message.");
}
