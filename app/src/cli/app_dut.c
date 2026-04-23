/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <sid_error.h>
#include <cli/app_dut.h>
#include <sidewalk.h>
#include <sid_900_cfg.h>
#include <sid_hal_memory_ifc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <app_mfg_config.h>
#ifdef CONFIG_SIDEWALK_FILE_TRANSFER_DFU
#include <sbdt/dfu_file_transfer.h>
#endif /* CONFIG_SIDEWALK_FILE_TRANSFER_DFU */
#include <json_printer/sidTypes2str.h>
LOG_MODULE_REGISTER(sid_cli, CONFIG_SIDEWALK_LOG_LEVEL);

#define DUT_FLOW_BLE_TIMEOUT K_SECONDS(30)
#define DUT_FLOW_FSK_TIMEOUT K_SECONDS(35)
#define DUT_FLOW_LORA_TIMEOUT K_SECONDS(120)
#define DUT_FLOW_MAX_BLE_RETRIES UINT32_MAX
#define DUT_FLOW_MAX_FSK_RETRIES UINT32_MAX
#define DUT_FLOW_MAX_LORA_RETRIES UINT32_MAX
#define DUT_FLOW_MAX_TX_RETRIES UINT32_MAX

enum dut_flow_stage {
	DUT_FLOW_STAGE_IDLE = 0,
	DUT_FLOW_STAGE_WAIT_BLE,
	DUT_FLOW_STAGE_WAIT_FSK,
	DUT_FLOW_STAGE_WAIT_LORA,
};

typedef struct {
	bool success;
	sid_error_t error;
	struct sid_msg_desc desc;
} dut_flow_tx_result_ctx_t;

static void dut_flow_timeout_work_handler(struct k_work *work);
static void dut_flow_timeout_event(sidewalk_ctx_t *sid, void *ctx);
static void dut_event_flow_tx_result(sidewalk_ctx_t *sid, void *ctx);
static K_WORK_DELAYABLE_DEFINE(dut_flow_timeout_work, dut_flow_timeout_work_handler);

static bool dut_flow_mode_enabled;
static bool dut_flow_cached_time_valid;
static bool dut_flow_ble_policy_configured;
static uint32_t dut_flow_current_link_mask;
static uint32_t dut_flow_target_link_mask;
static uint32_t dut_flow_ble_retries;
static uint32_t dut_flow_fsk_retries;
static uint32_t dut_flow_lora_retries;
static uint32_t dut_flow_tx_retries;
static enum dut_flow_stage dut_flow_stage;
static bool dut_flow_send_inflight;
static sidewalk_ctx_t *dut_flow_sid;
static dut_flow_send_ctx_t *dut_flow_pending_send;

static uint32_t dut_ctx_get_uint32(void *ctx)
{
	if (!ctx) {
		LOG_ERR("Invalid context!");
		return 0;
	}
	uint32_t ctx_val = *((uint32_t *)ctx);
	return ctx_val;
}

void dut_flow_send_ctx_free(void *ctx)
{
	dut_flow_send_ctx_t *flow_send = (dut_flow_send_ctx_t *)ctx;

	if (flow_send == NULL) {
		return;
	}
	if (flow_send->send.msg.data != NULL) {
		sid_hal_free(flow_send->send.msg.data);
	}
	sid_hal_free(flow_send);
}

static void dut_flow_reset_state(sidewalk_ctx_t *sid)
{
	(void)k_work_cancel_delayable(&dut_flow_timeout_work);
	if (dut_flow_pending_send != NULL) {
		dut_flow_send_ctx_free(dut_flow_pending_send);
		dut_flow_pending_send = NULL;
	}
	dut_flow_mode_enabled = false;
	dut_flow_cached_time_valid = false;
	dut_flow_ble_policy_configured = false;
	dut_flow_current_link_mask = 0U;
	dut_flow_target_link_mask = 0U;
	dut_flow_ble_retries = 0U;
	dut_flow_fsk_retries = 0U;
	dut_flow_lora_retries = 0U;
	dut_flow_tx_retries = 0U;
	dut_flow_stage = DUT_FLOW_STAGE_IDLE;
	dut_flow_send_inflight = false;
	dut_flow_sid = NULL;
	memset(&sid->last_status, 0, sizeof(sid->last_status));
	sid->last_status.detail.registration_status = SID_STATUS_NOT_REGISTERED;
	sid->last_status.detail.time_sync_status = SID_STATUS_NO_TIME;
}

static const char *dut_flow_name(uint32_t link_mask)
{
	switch (link_mask) {
	case SID_LINK_TYPE_1:
		return "BLE";
	case SID_LINK_TYPE_2:
		return "FSK";
	case SID_LINK_TYPE_3:
	case (SID_LINK_TYPE_1 | SID_LINK_TYPE_3):
		return "LoRa";
	default:
		return "unknown";
	}
}

static const char *dut_flow_stage_name(enum dut_flow_stage stage)
{
	switch (stage) {
	case DUT_FLOW_STAGE_WAIT_BLE:
		return "BLE connection";
	case DUT_FLOW_STAGE_WAIT_FSK:
		return "FSK bootstrap";
	case DUT_FLOW_STAGE_WAIT_LORA:
		return "LoRa transition";
	default:
		return "idle";
	}
}

static bool dut_flow_has_cached_time(const sidewalk_ctx_t *sid)
{
	ARG_UNUSED(sid);
	return dut_flow_cached_time_valid;
}

static bool dut_flow_is_fsk_ready(const sidewalk_ctx_t *sid)
{
	return (sid->last_status.state == SID_STATE_READY) &&
	       dut_flow_has_cached_time(sid) &&
	       ((sid->last_status.detail.link_status_mask & SID_LINK_TYPE_2) != 0U);
}

static bool dut_flow_link_is_ready(const sidewalk_ctx_t *sid, uint32_t link_mask)
{
	return (sid->last_status.state == SID_STATE_READY) &&
	       ((sid->last_status.detail.link_status_mask & link_mask) == link_mask);
}

static bool dut_flow_target_is_ready(const sidewalk_ctx_t *sid, uint32_t target_link_mask)
{
	uint32_t ready_mask = target_link_mask;

	if (target_link_mask == (SID_LINK_TYPE_1 | SID_LINK_TYPE_3)) {
		ready_mask = SID_LINK_TYPE_3;
	}

	return dut_flow_link_is_ready(sid, ready_mask);
}

static void dut_flow_log_status(const sidewalk_ctx_t *sid)
{
	LOG_INF("hello-flow status: stage=%s target=%s current=%s cached_time=%s "
		"pending_send=%s inflight_send=%s retries={BLE:%u, FSK:%u, LoRa:%u, TX:%u} "
		"link={BLE:%s, FSK:%s, LoRa:%s}",
		dut_flow_stage_name(dut_flow_stage), dut_flow_name(dut_flow_target_link_mask),
		dut_flow_name(dut_flow_current_link_mask),
		dut_flow_cached_time_valid ? "true" : "false",
		dut_flow_pending_send != NULL ? "true" : "false",
		dut_flow_send_inflight ? "true" : "false",
		(unsigned int)dut_flow_ble_retries, (unsigned int)dut_flow_fsk_retries,
		(unsigned int)dut_flow_lora_retries, (unsigned int)dut_flow_tx_retries,
		(sid->last_status.detail.link_status_mask & SID_LINK_TYPE_1) ? "Up" : "Down",
		(sid->last_status.detail.link_status_mask & SID_LINK_TYPE_2) ? "Up" : "Down",
		(sid->last_status.detail.link_status_mask & SID_LINK_TYPE_3) ? "Up" : "Down");
}

static bool dut_check_mfg_data(void)
{
	if (!app_mfg_cfg_is_empty()) {
		return true;
	}

	LOG_ERR("The mfg.hex version mismatch");
	LOG_ERR("Check if the file has been generated and flashed properly");
	LOG_ERR("START ADDRESS: 0x%08x", APP_MFG_CFG_FLASH_START);
	LOG_ERR("SIZE: 0x%08x", APP_MFG_CFG_FLASH_SIZE);
	return false;
}

static bool dut_configure_ble_auto_connect(sidewalk_ctx_t *sid)
{
#if CONFIG_SID_END_DEVICE_AUTO_CONN_REQ
	if ((sid->config.link_mask & SID_LINK_TYPE_1) == 0U) {
		return false;
	}

	enum sid_link_connection_policy set_policy = SID_LINK_CONNECTION_POLICY_AUTO_CONNECT;
	sid_error_t e = sid_option(sid->handle, SID_OPTION_SET_LINK_CONNECTION_POLICY, &set_policy,
				   sizeof(set_policy));

	if (e) {
		LOG_ERR("sid option multi link manager err %d (%s)", (int)e, SID_ERROR_T_STR(e));
		return false;
	}

	struct sid_link_auto_connect_params ac_params = {
		.link_type = SID_LINK_TYPE_1,
		.enable = true,
		.priority = 0,
		.connection_attempt_timeout_seconds = 30,
	};

	e = sid_option(sid->handle, SID_OPTION_SET_LINK_POLICY_AUTO_CONNECT_PARAMS, &ac_params,
		       sizeof(ac_params));
	if (e) {
		LOG_ERR("sid option multi link policy err %d (%s)", (int)e, SID_ERROR_T_STR(e));
		return false;
	}

	return true;
#else
	ARG_UNUSED(sid);
	return false;
#endif
}

static void dut_request_ble_connection(sidewalk_ctx_t *sid)
{
	if ((sid == NULL) || (sid->handle == NULL) ||
	    ((sid->config.link_mask & SID_LINK_TYPE_1) == 0U)) {
		return;
	}

	sid_error_t e = sid_ble_bcn_connection_request(sid->handle, true);
	if (e != SID_ERROR_NONE) {
		LOG_WRN("hello-flow BLE connection request failed: %d (%s)", (int)e,
			SID_ERROR_T_STR(e));
		return;
	}

	LOG_INF("hello-flow BLE connection request set");
}

static void dut_flow_cancel_wait(void)
{
	(void)k_work_cancel_delayable(&dut_flow_timeout_work);
	dut_flow_stage = DUT_FLOW_STAGE_IDLE;
}

static void dut_flow_schedule_wait(sidewalk_ctx_t *sid, enum dut_flow_stage stage)
{
	k_timeout_t timeout = K_NO_WAIT;

	dut_flow_sid = sid;
	dut_flow_stage = stage;

	switch (stage) {
	case DUT_FLOW_STAGE_WAIT_BLE:
		timeout = DUT_FLOW_BLE_TIMEOUT;
		break;
	case DUT_FLOW_STAGE_WAIT_FSK:
		timeout = DUT_FLOW_FSK_TIMEOUT;
		break;
	case DUT_FLOW_STAGE_WAIT_LORA:
		timeout = DUT_FLOW_LORA_TIMEOUT;
		break;
	default:
		dut_flow_cancel_wait();
		return;
	}

	(void)k_work_reschedule(&dut_flow_timeout_work, timeout);
}

static sid_error_t dut_flow_prepare_session(sidewalk_ctx_t *sid)
{
	const uint32_t flow_init_mask = SID_LINK_TYPE_1 | SID_LINK_TYPE_2 | SID_LINK_TYPE_3;

	if (sid->handle != NULL && !dut_flow_mode_enabled) {
#ifdef CONFIG_SIDEWALK_FILE_TRANSFER_DFU
		app_file_transfer_demo_deinit(sid->handle);
#endif
		sid_error_t e = sid_deinit(sid->handle);
		LOG_INF("hello-flow sid_deinit returned %d (%s)", (int)e, SID_ERROR_T_STR(e));
		sid->handle = NULL;
	}

	if (sid->handle != NULL) {
		return SID_ERROR_NONE;
	}

	if (!dut_check_mfg_data()) {
		return SID_ERROR_NOT_FOUND;
	}

	sid->config.link_mask = flow_init_mask;
	memset(&sid->last_status, 0, sizeof(sid->last_status));

	sid_error_t e = sid_init(&sid->config, &sid->handle);
	LOG_INF("hello-flow sid_init returned %d (%s)", e, SID_ERROR_T_STR(e));
	if (e != SID_ERROR_NONE) {
		return e;
	}

#ifdef CONFIG_SIDEWALK_FILE_TRANSFER_DFU
	app_file_transfer_demo_init(sid->handle);
#endif

	dut_flow_mode_enabled = true;
	dut_flow_cached_time_valid = false;
	dut_flow_ble_policy_configured = false;
	dut_flow_current_link_mask = 0U;
	dut_flow_target_link_mask = 0U;
	dut_flow_ble_retries = 0U;
	dut_flow_fsk_retries = 0U;
	dut_flow_lora_retries = 0U;
	dut_flow_tx_retries = 0U;
	dut_flow_stage = DUT_FLOW_STAGE_IDLE;
	dut_flow_send_inflight = false;
	dut_flow_sid = sid;

	return SID_ERROR_NONE;
}

static uint32_t dut_normalize_hello_mask(uint32_t link_mask)
{
	switch (link_mask) {
	case SID_LINK_TYPE_1:
	case SID_LINK_TYPE_2:
	case (SID_LINK_TYPE_1 | SID_LINK_TYPE_3):
		return link_mask;
	default:
		return SID_LINK_TYPE_1;
	}
}

static sid_error_t dut_flow_transition(sidewalk_ctx_t *sid, uint32_t target_link_mask, bool force_restart)
{
	sid_error_t e = SID_ERROR_NONE;

	if (!force_restart && (dut_flow_current_link_mask == target_link_mask)) {
		sid->config.link_mask = target_link_mask;
		if (target_link_mask == SID_LINK_TYPE_1) {
			dut_request_ble_connection(sid);
		}
		return SID_ERROR_NONE;
	}

	LOG_INF("Sidewalk link switch to %s", dut_flow_name(target_link_mask));

	if (dut_flow_current_link_mask != 0U) {
		e = sid_stop(sid->handle, dut_flow_current_link_mask);
		LOG_INF("hello-flow sid_stop returned %d (%s)", (int)e, SID_ERROR_T_STR(e));
		if (e != SID_ERROR_NONE) {
			return e;
		}
	}

	e = sid_start(sid->handle, target_link_mask);
	LOG_INF("hello-flow sid_start returned %d (%s)", (int)e, SID_ERROR_T_STR(e));
	if (e == SID_ERROR_NONE) {
		sid->config.link_mask = target_link_mask;
		if (((target_link_mask & SID_LINK_TYPE_1) != 0U) && !dut_flow_ble_policy_configured) {
			dut_flow_ble_policy_configured = dut_configure_ble_auto_connect(sid);
		}
		if (target_link_mask == SID_LINK_TYPE_1) {
			dut_request_ble_connection(sid);
		}
		dut_flow_current_link_mask = target_link_mask;
	}

	return e;
}

static void dut_flow_timeout_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (sidewalk_event_send(dut_flow_timeout_event, NULL, NULL) != 0) {
		LOG_ERR("hello-flow failed to queue timeout event");
	}
}

static void dut_flow_timeout_event(sidewalk_ctx_t *sid, void *ctx)
{
	uint32_t retry_mask = 0U;
	uint32_t *retry_counter = NULL;
	uint32_t max_retries = 0U;
	sid_error_t e;

	ARG_UNUSED(ctx);

	if ((sid == NULL) || !dut_flow_mode_enabled || (sid->handle == NULL)) {
		dut_flow_cancel_wait();
		return;
	}

	switch (dut_flow_stage) {
	case DUT_FLOW_STAGE_WAIT_BLE:
		retry_mask = SID_LINK_TYPE_1;
		retry_counter = &dut_flow_ble_retries;
		max_retries = DUT_FLOW_MAX_BLE_RETRIES;
		break;
	case DUT_FLOW_STAGE_WAIT_FSK:
		retry_mask = SID_LINK_TYPE_2;
		retry_counter = &dut_flow_fsk_retries;
		max_retries = DUT_FLOW_MAX_FSK_RETRIES;
		break;
	case DUT_FLOW_STAGE_WAIT_LORA:
		retry_mask = SID_LINK_TYPE_1 | SID_LINK_TYPE_3;
		retry_counter = &dut_flow_lora_retries;
		max_retries = DUT_FLOW_MAX_LORA_RETRIES;
		break;
	default:
		return;
	}

	if (*retry_counter >= max_retries) {
		LOG_ERR("hello-flow %s timed out after %u attempt(s)",
			dut_flow_stage_name(dut_flow_stage), (unsigned int)(*retry_counter + 1U));
		dut_flow_cancel_wait();
		return;
	}

	(*retry_counter)++;
	LOG_WRN("hello-flow %s still waiting, retrying attempt %u", dut_flow_stage_name(dut_flow_stage),
		(unsigned int)*retry_counter);

	e = dut_flow_transition(sid, retry_mask, true);
	if (e != SID_ERROR_NONE) {
		LOG_ERR("hello-flow retry to %s failed: %d (%s)", dut_flow_name(retry_mask), (int)e,
			SID_ERROR_T_STR(e));
		dut_flow_cancel_wait();
		return;
	}

	dut_flow_schedule_wait(sid, dut_flow_stage);
	dut_flow_log_status(sid);
}

static void dut_flow_try_pending_send(sidewalk_ctx_t *sid)
{
	if ((dut_flow_pending_send == NULL) || (sid == NULL) || (sid->handle == NULL)) {
		return;
	}

	if (dut_flow_send_inflight) {
		return;
	}

	if (!dut_flow_link_is_ready(sid, dut_flow_pending_send->send.desc.link_type)) {
		return;
	}

	if (dut_flow_pending_send->send.desc.type != SID_MSG_TYPE_RESPONSE) {
		dut_flow_pending_send->send.desc.id = 0U;
	}

	sid_error_t e = sid_put_msg(sid->handle, &dut_flow_pending_send->send.msg,
				    &dut_flow_pending_send->send.desc);
	if (e != SID_ERROR_NONE) {
		LOG_WRN("hello-flow pending send on %s failed: %d (%s), keeping queued",
			dut_flow_name(dut_flow_pending_send->send.desc.link_type), (int)e,
			SID_ERROR_T_STR(e));
		dut_flow_target_link_mask = dut_flow_pending_send->target_link_mask;
		if (dut_flow_pending_send->send.desc.link_type == SID_LINK_TYPE_1) {
			dut_flow_schedule_wait(sid, DUT_FLOW_STAGE_WAIT_BLE);
		} else if (dut_flow_pending_send->send.desc.link_type == SID_LINK_TYPE_2) {
			dut_flow_schedule_wait(sid, DUT_FLOW_STAGE_WAIT_FSK);
		} else if (dut_flow_pending_send->send.desc.link_type == SID_LINK_TYPE_3) {
			dut_flow_schedule_wait(sid, DUT_FLOW_STAGE_WAIT_LORA);
		}
		return;
	}

	dut_flow_send_inflight = true;
	LOG_INF("hello-flow pending send accepted on %s (id %u)",
		dut_flow_name(dut_flow_pending_send->send.desc.link_type),
		(unsigned int)dut_flow_pending_send->send.desc.id);
}

static sid_error_t dut_flow_reach_target(sidewalk_ctx_t *sid, uint32_t target_link_mask)
{
	sid_error_t e;

	dut_flow_target_link_mask = target_link_mask;

	if ((target_link_mask == (SID_LINK_TYPE_1 | SID_LINK_TYPE_3)) &&
	    !dut_flow_has_cached_time(sid)) {
		if (dut_flow_current_link_mask != SID_LINK_TYPE_2) {
			e = dut_flow_transition(sid, SID_LINK_TYPE_2, false);
			if (e != SID_ERROR_NONE) {
				return e;
			}
		}

		LOG_INF("hello-flow waiting for FSK time sync before switching to %s",
			dut_flow_name(target_link_mask));
		dut_flow_schedule_wait(sid, DUT_FLOW_STAGE_WAIT_FSK);
		return SID_ERROR_NONE;
	}

	e = dut_flow_transition(sid, target_link_mask, false);
	if (e != SID_ERROR_NONE) {
		LOG_ERR("hello-flow could not reach target mask 0x%x from 0x%x",
			(unsigned int)target_link_mask, (unsigned int)dut_flow_current_link_mask);
		return e;
	}

	if (dut_flow_target_is_ready(sid, target_link_mask)) {
		dut_flow_cancel_wait();
		dut_flow_try_pending_send(sid);
		return SID_ERROR_NONE;
	}

	if (target_link_mask == SID_LINK_TYPE_1) {
		dut_flow_schedule_wait(sid, DUT_FLOW_STAGE_WAIT_BLE);
	} else if (target_link_mask == SID_LINK_TYPE_2) {
		dut_flow_schedule_wait(sid, DUT_FLOW_STAGE_WAIT_FSK);
	} else if (target_link_mask == (SID_LINK_TYPE_1 | SID_LINK_TYPE_3)) {
		dut_flow_schedule_wait(sid, DUT_FLOW_STAGE_WAIT_LORA);
	}

	return SID_ERROR_NONE;
}

static bool dut_flow_desc_matches(const struct sid_msg_desc *expected,
				  const struct sid_msg_desc *actual)
{
	return (expected != NULL) && (actual != NULL) &&
	       (expected->id == actual->id) &&
	       (expected->link_type == actual->link_type) &&
	       (expected->type == actual->type);
}

static void dut_flow_submit_tx_result(bool success, sid_error_t error,
				      const struct sid_msg_desc *msg_desc)
{
	if (!dut_flow_mode_enabled || (dut_flow_pending_send == NULL) || !dut_flow_send_inflight) {
		return;
	}

	if (msg_desc == NULL) {
		return;
	}

	dut_flow_tx_result_ctx_t *result = sid_hal_malloc(sizeof(*result));
	if (result == NULL) {
		LOG_ERR("hello-flow failed to allocate tx result context");
		return;
	}

	result->success = success;
	result->error = error;
	memcpy(&result->desc, msg_desc, sizeof(result->desc));

	if (sidewalk_event_send(dut_event_flow_tx_result, result, sid_hal_free) != 0) {
		LOG_ERR("hello-flow failed to queue tx result event");
		sid_hal_free(result);
	}
}

void dut_flow_notify_msg_sent(const struct sid_msg_desc *msg_desc)
{
	dut_flow_submit_tx_result(true, SID_ERROR_NONE, msg_desc);
}

void dut_flow_notify_send_error(sid_error_t error, const struct sid_msg_desc *msg_desc)
{
	dut_flow_submit_tx_result(false, error, msg_desc);
}

static void dut_event_flow_tx_result(sidewalk_ctx_t *sid, void *ctx)
{
	dut_flow_tx_result_ctx_t *result = (dut_flow_tx_result_ctx_t *)ctx;

	if ((result == NULL) || !dut_flow_mode_enabled || (dut_flow_pending_send == NULL) ||
	    !dut_flow_send_inflight) {
		return;
	}

	if (!dut_flow_desc_matches(&dut_flow_pending_send->send.desc, &result->desc)) {
		return;
	}

	if (result->success) {
		LOG_INF("hello-flow send completed on %s (id %u)",
			dut_flow_name(result->desc.link_type), (unsigned int)result->desc.id);
		dut_flow_send_ctx_free(dut_flow_pending_send);
		dut_flow_pending_send = NULL;
		dut_flow_send_inflight = false;
		dut_flow_tx_retries = 0U;
		return;
	}

	if (dut_flow_tx_retries >= DUT_FLOW_MAX_TX_RETRIES) {
		LOG_ERR("hello-flow send on %s failed after %u retry attempt(s): %d (%s)",
			dut_flow_name(result->desc.link_type), (unsigned int)dut_flow_tx_retries,
			(int)result->error, SID_ERROR_T_STR(result->error));
		dut_flow_send_ctx_free(dut_flow_pending_send);
		dut_flow_pending_send = NULL;
		dut_flow_send_inflight = false;
		return;
	}

	dut_flow_tx_retries++;
	dut_flow_send_inflight = false;
	dut_flow_target_link_mask = dut_flow_pending_send->target_link_mask;
	LOG_WRN("hello-flow send on %s failed after queue: %d (%s), retrying attempt %u",
		dut_flow_name(result->desc.link_type), (int)result->error,
		SID_ERROR_T_STR(result->error), (unsigned int)dut_flow_tx_retries);

	if (dut_flow_link_is_ready(sid, dut_flow_pending_send->send.desc.link_type)) {
		dut_flow_try_pending_send(sid);
	} else {
		(void)dut_flow_reach_target(sid,
					    dut_normalize_hello_mask(dut_flow_target_link_mask));
	}

	dut_flow_log_status(sid);
}

void dut_event_init(sidewalk_ctx_t *sid, void *ctx)
{
	dut_flow_reset_state(sid);
	sid->config.link_mask = dut_ctx_get_uint32(ctx);
	if (!dut_check_mfg_data()) {
		return;
	}
	sid_error_t e = sid_init(&sid->config, &sid->handle);
	LOG_INF("sid_init returned %d (%s)", e, SID_ERROR_T_STR(e));
	if (e != SID_ERROR_NONE) {
		return;
	}
#ifdef CONFIG_SIDEWALK_FILE_TRANSFER_DFU
	app_file_transfer_demo_init(sid->handle);
#endif
}
void dut_event_deinit(sidewalk_ctx_t *sid, void *ctx)
{
	if (ctx) {
		LOG_WRN("Unexpected context");
	};
#ifdef CONFIG_SIDEWALK_FILE_TRANSFER_DFU
	app_file_transfer_demo_deinit(sid->handle);
#endif
	sid_error_t e = sid_deinit(sid->handle);
	LOG_INF("sid_deinit returned %d (%s)", (int)e, SID_ERROR_T_STR(e));
	sid->handle = NULL;
	dut_flow_reset_state(sid);
}
void dut_event_start(sidewalk_ctx_t *sid, void *ctx)
{
	dut_flow_reset_state(sid);
	uint32_t link_mask = dut_ctx_get_uint32(ctx);
	sid_error_t e = sid_start(sid->handle, link_mask);
	LOG_INF("sid_start returned %d (%s)", (int)e, SID_ERROR_T_STR(e));
}
void dut_event_stop(sidewalk_ctx_t *sid, void *ctx)
{
	dut_flow_reset_state(sid);
	uint32_t link_mask = dut_ctx_get_uint32(ctx);
	sid_error_t e = sid_stop(sid->handle, link_mask);
	LOG_INF("sid_stop returned %d (%s)", (int)e, SID_ERROR_T_STR(e));
}
void dut_event_get_mtu(sidewalk_ctx_t *sid, void *ctx)
{
	uint32_t link_mask = dut_ctx_get_uint32(ctx);
	size_t mtu = 0;
	sid_error_t e = sid_get_mtu(sid->handle, (enum sid_link_type)link_mask, &mtu);
	LOG_INF("sid_get_mtu returned %d (%s), MTU: %d", e, SID_ERROR_T_STR(e), mtu);
}
void dut_event_get_time(sidewalk_ctx_t *sid, void *ctx)
{
	uint32_t format = dut_ctx_get_uint32(ctx);
	struct sid_timespec curr_time = { 0 };
	sid_error_t e = sid_get_time(sid->handle, (enum sid_time_format)format, &curr_time);
	LOG_INF("sid_get_time returned %d (%s), SEC: %d NSEC: %d", e, SID_ERROR_T_STR(e),
		curr_time.tv_sec, curr_time.tv_nsec);
}
void dut_event_get_status(sidewalk_ctx_t *sid, void *ctx)
{
	LOG_INF("Device %sregistered, Time Sync %s, Link status: {BLE: %s, FSK: %s, LoRa: %s}",
		(SID_STATUS_REGISTERED == sid->last_status.detail.registration_status) ? "Is " :
											 "Un",
		(SID_STATUS_TIME_SYNCED == sid->last_status.detail.time_sync_status) ? "Success" :
										       "Fail",
		(sid->last_status.detail.link_status_mask & SID_LINK_TYPE_1) ? "Up" : "Down",
		(sid->last_status.detail.link_status_mask & SID_LINK_TYPE_2) ? "Up" : "Down",
		(sid->last_status.detail.link_status_mask & SID_LINK_TYPE_3) ? "Up" : "Down");

	for (int i = 0; i < SID_LINK_TYPE_MAX_IDX; i++) {
		enum sid_link_mode mode =
			(enum sid_link_mode)sid->last_status.detail.supported_link_modes[i];

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
void dut_event_get_option(sidewalk_ctx_t *sid, void *ctx)
{
	sidewalk_option_t *p_option = (sidewalk_option_t *)ctx;
	if (!p_option) {
		LOG_ERR("Invalid context!");
		return;
	}
	enum sid_option opt = p_option->option;
	switch (opt) {
	case SID_OPTION_GET_MSG_POLICY_FILTER_DUPLICATES: {
		uint8_t data = 0;
		sid_error_t e = sid_option(sid->handle, opt, &data, sizeof(data));
		LOG_INF("sid_option returned %d (%s); Filter Duplicates: %d", e, SID_ERROR_T_STR(e),
			data);
	} break;
	case SID_OPTION_GET_LINK_CONNECTION_POLICY: {
		uint8_t data = 0;
		sid_error_t e = sid_option(sid->handle, opt, &data, sizeof(data));
		LOG_INF("sid_option returned %d (%s); Link Connect Policy: %d", e,
			SID_ERROR_T_STR(e), data);
	} break;
	case SID_OPTION_GET_LINK_POLICY_MULTI_LINK_POLICY: {
		uint8_t data = 0;
		sid_error_t e = sid_option(sid->handle, opt, &data, sizeof(data));
		LOG_INF("sid_option returned %d (%s); Link Multi Link Policy: %d", e,
			SID_ERROR_T_STR(e), data);
	} break;
	case SID_OPTION_GET_STATISTICS: {
		struct sid_statistics stats = { 0 };
		sid_error_t e = sid_option(sid->handle, opt, &stats, sizeof(stats));
		LOG_INF("sid_option returned %d (%s); tx: %d, acks_sent %d, tx_fail: %d, retries: %d, dups: %d, acks_recv: %d rx: %d",
			e, SID_ERROR_T_STR(e), stats.msg_stats.tx, stats.msg_stats.acks_sent,
			stats.msg_stats.tx_fail, stats.msg_stats.retries,
			stats.msg_stats.duplicates, stats.msg_stats.acks_recv, stats.msg_stats.rx);
	} break;
	case SID_OPTION_GET_SIDEWALK_ID: {
		struct sid_id id = { 0 };
		sid_error_t e = sid_option(sid->handle, opt, &id, sizeof(id));
		LOG_INF("sid_option returned %d (%s); SIDEWALK_ID: %02X%02X%02X%02X%02X", e,
			SID_ERROR_T_STR(e), id.id[0], id.id[1], id.id[2], id.id[3], id.id[4]);
	} break;
	case SID_OPTION_GET_LINK_POLICY_AUTO_CONNECT_PARAMS: {
		struct sid_link_auto_connect_params params = { 0 };
		memcpy(&params.link_type, p_option->data, sizeof(uint32_t));
		sid_error_t e = sid_option(sid->handle, opt, (void *)&params, sizeof(params));
		LOG_INF("sid_option returned %d (%s); AC Policy, link %d, enable %d priority %d timeout %d",
			e, SID_ERROR_T_STR(e), params.link_type, params.enable, params.priority,
			params.connection_attempt_timeout_seconds);
	} break;
	case SID_OPTION_900MHZ_GET_DEVICE_PROFILE: {
		struct sid_device_profile dev_cfg = { 0 };
		if (p_option->data) {
			dev_cfg.unicast_params.device_profile_id =
				*((enum sid_device_profile_id *)p_option->data);
		}

		sid_error_t e = sid_option(sid->handle, SID_OPTION_900MHZ_GET_DEVICE_PROFILE,
					   &dev_cfg, sizeof(struct sid_device_profile));

		if (IS_LINK2_PROFILE_ID(dev_cfg.unicast_params.device_profile_id) ||
		    IS_LINK3_PROFILE_ID(dev_cfg.unicast_params.device_profile_id)) {
			if (dev_cfg.unicast_params.device_profile_id == SID_LINK2_PROFILE_2) {
				LOG_INF("sid_option returned %d (%s); Link_profile ID: %d Wndw_cnt: %d Rx_Int = %d",
					e, SID_ERROR_T_STR(e),
					dev_cfg.unicast_params.device_profile_id,
					dev_cfg.unicast_params.rx_window_count,
					dev_cfg.unicast_params.unicast_window_interval
						.sync_rx_interval_ms);
			} else {
				LOG_INF("sid_option returned %d (%s); Link_profile ID: %d Wndw_cnt: %d",
					e, SID_ERROR_T_STR(e),
					dev_cfg.unicast_params.device_profile_id,
					dev_cfg.unicast_params.rx_window_count);
			}
		}
	} break;
	default:
		LOG_INF("sid_option %d not supported", opt);
	}
}
void dut_event_set_option(sidewalk_ctx_t *sid, void *ctx)
{
	sidewalk_option_t *p_option = (sidewalk_option_t *)ctx;
	if (!p_option) {
		LOG_ERR("Invalid context!");
		return;
	}

	sid_error_t e =
		sid_option(sid->handle, p_option->option, p_option->data, p_option->data_len);
	LOG_INF("sid_option returned %d (%s)", e, SID_ERROR_T_STR(e));
}
void dut_event_set_dest_id(sidewalk_ctx_t *sid, void *ctx)
{
	uint32_t id = dut_ctx_get_uint32(ctx);
	sid_error_t e = sid_set_msg_dest_id(sid->handle, id);
	LOG_INF("sid_set_msg_dest_id returned %d (%s)", e, SID_ERROR_T_STR(e));
}
void dut_event_conn_req(sidewalk_ctx_t *sid, void *ctx)
{
	uint32_t event_req = dut_ctx_get_uint32(ctx);
	bool conn_req = (event_req == 1U);
	sid_error_t e = sid_ble_bcn_connection_request(sid->handle, conn_req);
	LOG_INF("sid_conn_request returned %d (%s)", e, SID_ERROR_T_STR(e));
}

void dut_event_flow_on_status(sidewalk_ctx_t *sid)
{
	if (!dut_flow_mode_enabled || sid->handle == NULL) {
		return;
	}

	if (sid->last_status.detail.time_sync_status == SID_STATUS_TIME_SYNCED) {
		dut_flow_cached_time_valid = true;
	}

	if ((dut_flow_target_link_mask == (SID_LINK_TYPE_1 | SID_LINK_TYPE_3)) &&
	    (dut_flow_current_link_mask == SID_LINK_TYPE_2) &&
	    dut_flow_is_fsk_ready(sid)) {
		uint32_t target = dut_flow_target_link_mask;
		sid_error_t e;

		dut_flow_cancel_wait();
		e = dut_flow_transition(sid, target, false);
		if (e != SID_ERROR_NONE) {
			dut_flow_schedule_wait(sid, DUT_FLOW_STAGE_WAIT_FSK);
		} else {
			dut_flow_schedule_wait(sid, DUT_FLOW_STAGE_WAIT_LORA);
		}
	}

	if ((dut_flow_target_link_mask != 0U) &&
	    dut_flow_target_is_ready(sid, dut_flow_target_link_mask)) {
		LOG_INF("hello-flow target %s is up", dut_flow_name(dut_flow_target_link_mask));
		dut_flow_cancel_wait();
		dut_flow_target_link_mask = 0U;
	}

	dut_flow_try_pending_send(sid);
}

void dut_event_flow_switch(sidewalk_ctx_t *sid, void *ctx)
{
	uint32_t target_link_mask = SID_LINK_TYPE_2;

	ARG_UNUSED(ctx);

	if (dut_flow_current_link_mask == SID_LINK_TYPE_2) {
		target_link_mask = SID_LINK_TYPE_1 | SID_LINK_TYPE_3;
	} else if (dut_flow_current_link_mask == (SID_LINK_TYPE_1 | SID_LINK_TYPE_3)) {
		target_link_mask = SID_LINK_TYPE_1;
	}

	dut_event_flow_set(sid, &target_link_mask);
}

void dut_event_flow_set(sidewalk_ctx_t *sid, void *ctx)
{
	sid_error_t e;
	uint32_t target_link_mask = dut_ctx_get_uint32(ctx);
	target_link_mask = dut_normalize_hello_mask(target_link_mask);

	e = dut_flow_prepare_session(sid);
	if (e != SID_ERROR_NONE) {
		return;
	}

	dut_flow_ble_retries = 0U;
	dut_flow_fsk_retries = 0U;
	dut_flow_lora_retries = 0U;
	dut_flow_tx_retries = 0U;
	(void)dut_flow_reach_target(sid, target_link_mask);
	dut_flow_log_status(sid);
}

void dut_event_flow_send(sidewalk_ctx_t *sid, void *ctx)
{
	dut_flow_send_ctx_t *flow_send = (dut_flow_send_ctx_t *)ctx;
	sid_error_t e;

	if (flow_send == NULL) {
		LOG_ERR("hello-flow send context is NULL");
		return;
	}

	e = dut_flow_prepare_session(sid);
	if (e != SID_ERROR_NONE) {
		dut_flow_send_ctx_free(flow_send);
		return;
	}

	if (dut_flow_pending_send != NULL) {
		LOG_WRN("hello-flow replacing older pending send");
		dut_flow_send_ctx_free(dut_flow_pending_send);
	}

	dut_flow_send_inflight = false;
	dut_flow_pending_send = flow_send;
	LOG_INF("hello-flow queued send for %s (%u byte payload)",
		dut_flow_name(dut_flow_pending_send->send.desc.link_type),
		(unsigned int)dut_flow_pending_send->send.msg.size);

	if (dut_flow_link_is_ready(sid, dut_flow_pending_send->send.desc.link_type)) {
		dut_flow_try_pending_send(sid);
		return;
	}

	dut_flow_ble_retries = 0U;
	dut_flow_fsk_retries = 0U;
	dut_flow_lora_retries = 0U;
	dut_flow_tx_retries = 0U;
	(void)dut_flow_reach_target(sid, dut_normalize_hello_mask(flow_send->target_link_mask));
	dut_flow_log_status(sid);
}

void dut_event_flow_status(sidewalk_ctx_t *sid, void *ctx)
{
	ARG_UNUSED(ctx);
	dut_flow_log_status(sid);
}

void dut_event_flow_cancel(sidewalk_ctx_t *sid, void *ctx)
{
	ARG_UNUSED(ctx);

	if (dut_flow_pending_send != NULL) {
		dut_flow_send_ctx_free(dut_flow_pending_send);
		dut_flow_pending_send = NULL;
	}
	dut_flow_send_inflight = false;
	dut_flow_target_link_mask = 0U;
	dut_flow_cancel_wait();
	LOG_INF("hello-flow pending target/send canceled");
	dut_flow_log_status(sid);
}
