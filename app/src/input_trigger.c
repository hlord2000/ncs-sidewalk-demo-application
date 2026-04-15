/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <input_trigger.h>

#include "json_printer/sidTypes2str.h"
#include <sidewalk.h>
#include <sid_error.h>
#include <sid_hal_memory_ifc.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#if defined(CONFIG_GPIO) && defined(CONFIG_INPUT)
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#endif

LOG_MODULE_REGISTER(input_trigger, CONFIG_SIDEWALK_LOG_LEVEL);

#define BUTTON_PAYLOAD "button"
#define BLINK_ON_MS 100
#define BLINK_OFF_MS 120

#if defined(CONFIG_GPIO) && defined(CONFIG_INPUT) && DT_NODE_EXISTS(DT_NODELABEL(sidewalk_trigger_btn)) && \
	DT_NODE_HAS_STATUS(DT_ALIAS(led0), okay)

#define SIDEWALK_TRIGGER_BUTTON_NODE DT_NODELABEL(sidewalk_trigger_btn)
#define SIDEWALK_TRIGGER_BUTTON_DEV DEVICE_DT_GET(DT_PARENT(SIDEWALK_TRIGGER_BUTTON_NODE))

static const struct gpio_dt_spec blink_led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

static struct k_work button_send_work;
static struct k_work_delayable led_blink_work;
static atomic_t button_send_busy = ATOMIC_INIT(0);
static atomic_t button_send_pending = ATOMIC_INIT(0);
static atomic_t button_msg_id = ATOMIC_INIT(0);
static uint8_t blink_toggles_remaining;
static bool blink_led_on;

static bool sidewalk_ready_for_button_send(const sidewalk_ctx_t *sid)
{
	if ((sid == NULL) || (sid->handle == NULL)) {
		return false;
	}

	return ((sid->last_status.state == SID_STATE_READY) ||
		(sid->last_status.state == SID_STATE_SECURE_CHANNEL_READY)) &&
	       (sid->last_status.detail.link_status_mask != 0U);
}

static uint8_t link_type_to_blink_count(uint32_t link_type)
{
	switch (link_type) {
	case SID_LINK_TYPE_1:
		return 1;
	case SID_LINK_TYPE_2:
		return 2;
	case SID_LINK_TYPE_3:
		return 3;
	default:
		return 0;
	}
}

static void set_led(bool on)
{
	int err = gpio_pin_set_dt(&blink_led, on ? 1 : 0);

	if (err < 0) {
		LOG_ERR("Failed to drive LED: %d", err);
	}
}

static void led_blink_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (blink_toggles_remaining == 0U) {
		blink_led_on = false;
		set_led(false);
		return;
	}

	blink_led_on = !blink_led_on;
	set_led(blink_led_on);
	blink_toggles_remaining--;

	if (blink_toggles_remaining > 0U) {
		k_work_reschedule(&led_blink_work,
				  K_MSEC(blink_led_on ? BLINK_ON_MS : BLINK_OFF_MS));
		return;
	}

	blink_led_on = false;
	set_led(false);
}

static void start_led_blink(uint8_t blink_count)
{
	if (blink_count == 0U) {
		return;
	}

	blink_toggles_remaining = blink_count * 2U;
	blink_led_on = false;
	k_work_reschedule(&led_blink_work, K_NO_WAIT);
}

static void request_sidewalk_uplink(sidewalk_ctx_t *sid)
{
	if ((sid->config.link_mask & SID_LINK_TYPE_1) == 0U) {
		return;
	}

	if ((sid->last_status.detail.link_status_mask & SID_LINK_TYPE_1) != 0U) {
		return;
	}

	sid_error_t err = sid_ble_bcn_connection_request(sid->handle, true);

	if ((err != SID_ERROR_NONE) && (err != SID_ERROR_ALREADY_EXISTS)) {
		LOG_ERR("Button connection request failed: %d (%s)", (int)err, SID_ERROR_T_STR(err));
	}
}

static void button_send_event(sidewalk_ctx_t *sid, void *ctx)
{
	ARG_UNUSED(ctx);
	static const char payload[] = BUTTON_PAYLOAD;
	struct sid_msg msg = {
		.size = sizeof(payload) - 1U,
		.data = (void *)payload,
	};
	struct sid_msg_desc desc = {
		.type = SID_MSG_TYPE_NOTIFY,
		.link_type = SID_LINK_TYPE_ANY,
		.link_mode = SID_LINK_MODE_CLOUD,
	};

	if ((sid == NULL) || (sid->handle == NULL)) {
		LOG_WRN("Ignoring button send while Sidewalk is not started");
		atomic_clear(&button_send_pending);
		atomic_clear(&button_send_busy);
		return;
	}

	if (!sidewalk_ready_for_button_send(sid)) {
		atomic_set(&button_send_pending, 1);
		request_sidewalk_uplink(sid);
		LOG_INF("Button send queued until any Sidewalk uplink becomes ready");
		return;
	}

	sid_error_t err = sid_put_msg(sid->handle, &msg, &desc);

	if (err != SID_ERROR_NONE) {
		LOG_ERR("Button send failed: %d (%s)", (int)err, SID_ERROR_T_STR(err));
		atomic_clear(&button_send_pending);
		atomic_clear(&button_send_busy);
		return;
	}

	atomic_clear(&button_send_pending);
	atomic_set(&button_msg_id, desc.id);
	LOG_INF("Button Sidewalk send queued on ANY link, id=%u", desc.id);
}

static void button_send_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	int err = sidewalk_event_send(button_send_event, NULL, NULL);

	if (err != 0) {
		LOG_ERR("Failed to enqueue button send event: %d", err);
		atomic_clear(&button_send_pending);
		atomic_clear(&button_send_busy);
	}
}

static void input_trigger_callback(struct input_event *evt, void *user_data)
{
	ARG_UNUSED(user_data);

	if ((evt->type != INPUT_EV_KEY) ||
	    (evt->code != DT_PROP(SIDEWALK_TRIGGER_BUTTON_NODE, zephyr_code)) ||
	    (evt->value == 0)) {
		return;
	}

	if (!atomic_cas(&button_send_busy, 0, 1)) {
		LOG_WRN("Ignoring button press while a Sidewalk send is pending");
		return;
	}

	k_work_submit(&button_send_work);
}

INPUT_CALLBACK_DEFINE(SIDEWALK_TRIGGER_BUTTON_DEV, input_trigger_callback, NULL);

int input_trigger_init(void)
{
	if (!device_is_ready(SIDEWALK_TRIGGER_BUTTON_DEV)) {
		LOG_ERR("Input device %s is not ready", SIDEWALK_TRIGGER_BUTTON_DEV->name);
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&blink_led)) {
		LOG_ERR("LED GPIO is not ready");
		return -ENODEV;
	}

	int err = gpio_pin_configure_dt(&blink_led, GPIO_OUTPUT_INACTIVE);
	if (err < 0) {
		LOG_ERR("Failed to configure blink LED: %d", err);
		return err;
	}

	k_work_init(&button_send_work, button_send_work_handler);
	k_work_init_delayable(&led_blink_work, led_blink_work_handler);

	LOG_INF("Input trigger enabled on %s, LED blink on %s pin %u",
		SIDEWALK_TRIGGER_BUTTON_DEV->name, blink_led.port->name, blink_led.pin);

	return 0;
}

void input_trigger_on_status_changed(const struct sid_status *status)
{
	int err;

	if ((status == NULL) || !atomic_get(&button_send_busy) || !atomic_get(&button_send_pending)) {
		return;
	}

	if (((status->state != SID_STATE_READY) &&
	     (status->state != SID_STATE_SECURE_CHANNEL_READY)) ||
	    (status->detail.link_status_mask == 0U)) {
		return;
	}

	if (!atomic_cas(&button_send_pending, 1, 0)) {
		return;
	}

	err = k_work_submit(&button_send_work);
	if (err < 0) {
		LOG_ERR("Failed to submit queued button send: %d", err);
		atomic_clear(&button_send_busy);
	}
}

void input_trigger_on_msg_sent(const struct sid_msg_desc *msg_desc)
{
	uint16_t expected_id = (uint16_t)atomic_get(&button_msg_id);

	if ((msg_desc == NULL) || (expected_id == 0U) || (msg_desc->id != expected_id)) {
		return;
	}

	atomic_clear(&button_msg_id);
	atomic_clear(&button_send_busy);

	uint8_t blink_count = link_type_to_blink_count(msg_desc->link_type);
	if (blink_count == 0U) {
		LOG_WRN("Button send completed on unexpected link mask 0x%x",
			(unsigned int)msg_desc->link_type);
		return;
	}

	start_led_blink(blink_count);
}

void input_trigger_on_send_error(const struct sid_msg_desc *msg_desc)
{
	uint16_t expected_id = (uint16_t)atomic_get(&button_msg_id);

	if ((msg_desc != NULL) && (expected_id != 0U) && (msg_desc->id != expected_id)) {
		return;
	}

	atomic_clear(&button_msg_id);
	atomic_clear(&button_send_pending);
	atomic_clear(&button_send_busy);
}

#else

int input_trigger_init(void)
{
	return 0;
}

void input_trigger_on_status_changed(const struct sid_status *status)
{
	ARG_UNUSED(status);
}

void input_trigger_on_msg_sent(const struct sid_msg_desc *msg_desc)
{
	ARG_UNUSED(msg_desc);
}

void input_trigger_on_send_error(const struct sid_msg_desc *msg_desc)
{
	ARG_UNUSED(msg_desc);
}

#endif
