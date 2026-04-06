/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <prop_radio.h>

#include <errno.h>
#include <string.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(prop_radio, CONFIG_SIDEWALK_LOG_LEVEL);

struct prop_radio_state {
	struct k_mutex lock;
	bool module_ready;
	bool claimed;
	bool rx_enabled;
	bool rx_running;
	bool sidewalk_subghz_paused;
	uint32_t sidewalk_subghz_paused_mask;
	sid_error_t last_sid_error;
	int32_t last_radio_error;
	struct prop_radio_config config;
};

static struct prop_radio_state radio_state = {
	.config =
		{
			.frequency_hz = PROP_RADIO_DEFAULT_FREQUENCY_HZ,
			.tx_power_dbm = PROP_RADIO_DEFAULT_TX_POWER_DBM,
			.spreading_factor = SID_PAL_RADIO_LORA_SF7,
			.bandwidth = SID_PAL_RADIO_LORA_BW_125KHZ,
			.coding_rate = SID_PAL_RADIO_LORA_CODING_RATE_4_5,
			.preamble_length = 8,
			.sync_word = SID_PAL_RADIO_LORA_PRIVATE_NETWORK_SYNC_WORD,
			.invert_iq = false,
			.crc_on = true,
		},
};
static sid_pal_radio_rx_packet_t prop_rx_packet;

static uint32_t sidewalk_subghz_link_mask(const sidewalk_ctx_t *sid)
{
	if (sid == NULL || sid->handle == NULL) {
		return 0U;
	}

	return sid->config.link_mask & (SID_LINK_TYPE_2 | SID_LINK_TYPE_3);
}

static int32_t apply_lora_config_locked(uint8_t payload_len)
{
	sid_pal_radio_lora_modulation_params_t mod = {
		.spreading_factor = radio_state.config.spreading_factor,
		.bandwidth = radio_state.config.bandwidth,
		.coding_rate = radio_state.config.coding_rate,
	};
	sid_pal_radio_lora_packet_params_t pkt = {
		.preamble_length = radio_state.config.preamble_length,
		.header_type = SID_PAL_RADIO_LORA_HEADER_TYPE_VARIABLE_LENGTH,
		.payload_length = payload_len,
		.crc_mode = radio_state.config.crc_on ? SID_PAL_RADIO_LORA_CRC_ON :
							 SID_PAL_RADIO_LORA_CRC_OFF,
		.invert_IQ = radio_state.config.invert_iq ? SID_PAL_RADIO_LORA_IQ_INVERTED :
							  SID_PAL_RADIO_LORA_IQ_NORMAL,
	};
	int32_t err;

	err = sid_pal_radio_standby();
	if (err != RADIO_ERROR_NONE) {
		return err;
	}

	err = sid_pal_radio_set_modem_mode(SID_PAL_RADIO_MODEM_MODE_LORA);
	if (err != RADIO_ERROR_NONE) {
		return err;
	}

	err = sid_pal_radio_set_frequency(radio_state.config.frequency_hz);
	if (err != RADIO_ERROR_NONE) {
		return err;
	}

	err = sid_pal_radio_set_tx_power(radio_state.config.tx_power_dbm);
	if (err != RADIO_ERROR_NONE) {
		return err;
	}

	err = sid_pal_radio_set_lora_sync_word(radio_state.config.sync_word);
	if (err != RADIO_ERROR_NONE) {
		return err;
	}

	err = sid_pal_radio_set_lora_modulation_params(&mod);
	if (err != RADIO_ERROR_NONE) {
		return err;
	}

	return sid_pal_radio_set_lora_packet_params(&pkt);
}

static int32_t start_continuous_rx_locked(void)
{
	int32_t err;

	err = apply_lora_config_locked(SID_PAL_RADIO_RX_PAYLOAD_MAX_SIZE);
	if (err != RADIO_ERROR_NONE) {
		return err;
	}

	return sid_pal_radio_start_continuous_rx();
}

static void set_success(struct prop_radio_request *req)
{
	req->result = 0;
	req->sid_error = SID_ERROR_NONE;
	req->radio_error = RADIO_ERROR_NONE;
}

static void set_failure(struct prop_radio_request *req, int result, sid_error_t sid_err,
			int32_t radio_err)
{
	if (sid_err != SID_ERROR_NONE) {
		radio_state.last_sid_error = sid_err;
		req->sid_error = sid_err;
	}
	if (radio_err != RADIO_ERROR_NONE) {
		radio_state.last_radio_error = radio_err;
		req->radio_error = radio_err;
	}
	req->result = result;
}

static void restore_default_client_locked(void)
{
	int32_t radio_err;

	radio_err = sid_pal_radio_sleep(0);
	if (radio_err != RADIO_ERROR_NONE) {
		radio_state.last_radio_error = radio_err;
	}

	radio_err = sid_pal_radio_restore_default_client();
	if (radio_err != RADIO_ERROR_NONE) {
		radio_state.last_radio_error = radio_err;
	}
}

static int claim_radio_locked(sidewalk_ctx_t *sid, struct prop_radio_request *req)
{
	sid_error_t sid_err = SID_ERROR_NONE;
	int32_t radio_err;

	if (radio_state.claimed) {
		if (radio_state.rx_enabled) {
			radio_err = start_continuous_rx_locked();
			if (radio_err != RADIO_ERROR_NONE) {
				set_failure(req, -EIO, SID_ERROR_NONE, radio_err);
				return req->result;
			}
			radio_state.rx_running = true;
			radio_state.last_radio_error = RADIO_ERROR_NONE;
		}
		radio_state.last_sid_error = SID_ERROR_NONE;
		if (!radio_state.rx_enabled) {
			radio_state.last_radio_error = RADIO_ERROR_NONE;
		}
		set_success(req);
		return 0;
	}

	uint32_t subghz_link_mask = sidewalk_subghz_link_mask(sid);

	if (subghz_link_mask != 0U) {
		sid_err = sid_stop(sid->handle, subghz_link_mask);
		if (sid_err != SID_ERROR_NONE && sid_err != SID_ERROR_STOPPED) {
			set_failure(req, -EIO, sid_err, RADIO_ERROR_NONE);
			return req->result;
		}
		radio_state.sidewalk_subghz_paused = true;
		radio_state.sidewalk_subghz_paused_mask = subghz_link_mask;
	}

	radio_err = sid_pal_radio_switch_client(prop_radio_event_notifier, prop_radio_irq_handler,
						&prop_rx_packet);
	if (radio_err != RADIO_ERROR_NONE) {
		if (radio_state.sidewalk_subghz_paused && sid != NULL && sid->handle != NULL) {
			(void)sid_start(sid->handle, radio_state.sidewalk_subghz_paused_mask);
			radio_state.sidewalk_subghz_paused = false;
			radio_state.sidewalk_subghz_paused_mask = 0U;
		}
		set_failure(req, -EIO, SID_ERROR_NONE, radio_err);
		return req->result;
	}

	radio_state.claimed = true;
	radio_state.last_sid_error = SID_ERROR_NONE;
	radio_state.last_radio_error = RADIO_ERROR_NONE;

	if (radio_state.rx_enabled) {
		radio_err = start_continuous_rx_locked();
		if (radio_err != RADIO_ERROR_NONE) {
			restore_default_client_locked();
			if (radio_state.sidewalk_subghz_paused && sid != NULL && sid->handle != NULL) {
				(void)sid_start(sid->handle, radio_state.sidewalk_subghz_paused_mask);
				radio_state.sidewalk_subghz_paused = false;
				radio_state.sidewalk_subghz_paused_mask = 0U;
			}
			radio_state.claimed = false;
			radio_state.rx_running = false;
			set_failure(req, -EIO, SID_ERROR_NONE, radio_err);
			return req->result;
		}
		radio_state.rx_running = true;
	}

	LOG_INF("proprietary LoRa claimed");
	set_success(req);
	return 0;
}

static int release_radio_locked(sidewalk_ctx_t *sid, struct prop_radio_request *req)
{
	sid_error_t sid_err = SID_ERROR_NONE;
	int32_t radio_err = RADIO_ERROR_NONE;

	if (radio_state.claimed) {
		radio_err = sid_pal_radio_sleep(0);
		if (radio_err != RADIO_ERROR_NONE) {
			set_failure(req, -EIO, SID_ERROR_NONE, radio_err);
			return req->result;
		}

		radio_err = sid_pal_radio_restore_default_client();
		if (radio_err != RADIO_ERROR_NONE) {
			set_failure(req, -EIO, SID_ERROR_NONE, radio_err);
			return req->result;
		}
	}

	if (radio_state.sidewalk_subghz_paused && sid != NULL && sid->handle != NULL) {
		sid_err = sid_start(sid->handle, radio_state.sidewalk_subghz_paused_mask);
		if (sid_err != SID_ERROR_NONE) {
			set_failure(req, -EIO, sid_err, RADIO_ERROR_NONE);
			return req->result;
		}
		radio_state.last_sid_error = SID_ERROR_NONE;
	}

	radio_state.claimed = false;
	radio_state.rx_enabled = false;
	radio_state.rx_running = false;
	radio_state.sidewalk_subghz_paused = false;
	radio_state.sidewalk_subghz_paused_mask = 0U;
	radio_state.last_sid_error = SID_ERROR_NONE;
	radio_state.last_radio_error = radio_err;
	LOG_INF("proprietary LoRa released");
	set_success(req);
	return 0;
}

static int config_radio_locked(sidewalk_ctx_t *sid, struct prop_radio_request *req)
{
	int32_t radio_err = RADIO_ERROR_NONE;

	ARG_UNUSED(sid);

	radio_state.config = req->params.config;

	if (radio_state.claimed && radio_state.rx_enabled) {
		radio_err = start_continuous_rx_locked();
		if (radio_err != RADIO_ERROR_NONE) {
			radio_state.rx_running = false;
			set_failure(req, -EIO, SID_ERROR_NONE, radio_err);
			return req->result;
		}
		radio_state.rx_running = true;
	}

	radio_state.last_sid_error = SID_ERROR_NONE;
	radio_state.last_radio_error = RADIO_ERROR_NONE;
	set_success(req);
	return 0;
}

static int tx_radio_locked(sidewalk_ctx_t *sid, struct prop_radio_request *req)
{
	int32_t radio_err;

	if (!radio_state.claimed) {
		int claim_err = claim_radio_locked(sid, req);

		if (claim_err != 0) {
			return claim_err;
		}
	}

	radio_err = apply_lora_config_locked((uint8_t)req->params.tx.len);
	if (radio_err == RADIO_ERROR_NONE) {
		radio_err = sid_pal_radio_set_tx_payload(req->params.tx.payload,
							 (uint8_t)req->params.tx.len);
	}
	if (radio_err == RADIO_ERROR_NONE) {
		radio_err = sid_pal_radio_start_tx(SID_PAL_RADIO_LORA_DEFAULT_TX_TIMEOUT);
	}
	if (radio_err != RADIO_ERROR_NONE) {
		set_failure(req, -EIO, SID_ERROR_NONE, radio_err);
		return req->result;
	}

	radio_state.rx_running = false;
	radio_state.last_sid_error = SID_ERROR_NONE;
	radio_state.last_radio_error = RADIO_ERROR_NONE;
	set_success(req);
	return 0;
}

static int rx_start_locked(sidewalk_ctx_t *sid, struct prop_radio_request *req)
{
	int32_t radio_err;

	radio_state.rx_enabled = true;

	if (!radio_state.claimed) {
		int claim_err = claim_radio_locked(sid, req);

		if (claim_err != 0) {
			return claim_err;
		}
	}

	if (!radio_state.rx_running) {
		radio_err = start_continuous_rx_locked();
		if (radio_err != RADIO_ERROR_NONE) {
			radio_state.rx_running = false;
			set_failure(req, -EIO, SID_ERROR_NONE, radio_err);
			return req->result;
		}
	}

	radio_state.rx_running = true;
	radio_state.last_sid_error = SID_ERROR_NONE;
	radio_state.last_radio_error = RADIO_ERROR_NONE;
	set_success(req);
	return 0;
}

static int rx_stop_locked(struct prop_radio_request *req)
{
	int32_t radio_err = RADIO_ERROR_NONE;

	radio_state.rx_enabled = false;
	radio_state.rx_running = false;

	if (radio_state.claimed) {
		radio_err = sid_pal_radio_standby();
		if (radio_err != RADIO_ERROR_NONE) {
			set_failure(req, -EIO, SID_ERROR_NONE, radio_err);
			return req->result;
		}
	}

	radio_state.last_sid_error = SID_ERROR_NONE;
	radio_state.last_radio_error = RADIO_ERROR_NONE;
	set_success(req);
	return 0;
}

void prop_radio_platform_init(void)
{
	k_mutex_init(&radio_state.lock);
	memset(&prop_rx_packet, 0, sizeof(prop_rx_packet));
	radio_state.module_ready = true;
	radio_state.claimed = false;
	radio_state.rx_enabled = false;
	radio_state.rx_running = false;
	radio_state.sidewalk_subghz_paused = false;
	radio_state.sidewalk_subghz_paused_mask = 0U;
	radio_state.last_sid_error = SID_ERROR_NONE;
	radio_state.last_radio_error = RADIO_ERROR_NONE;
}

void prop_radio_event_notifier(sid_pal_radio_events_t event)
{
	k_mutex_lock(&radio_state.lock, K_FOREVER);

	switch (event) {
	case SID_PAL_RADIO_EVENT_RX_DONE:
		LOG_INF("prop rx len=%u RSSI=%d SNR=%d", prop_rx_packet.payload_len,
			prop_rx_packet.lora_rx_packet_status.rssi,
			prop_rx_packet.lora_rx_packet_status.snr);
		LOG_HEXDUMP_INF(prop_rx_packet.rcv_payload, prop_rx_packet.payload_len,
				"prop payload");
		break;
	case SID_PAL_RADIO_EVENT_TX_DONE:
		if (radio_state.claimed && radio_state.rx_enabled) {
			radio_state.last_radio_error = start_continuous_rx_locked();
			radio_state.rx_running =
				(radio_state.last_radio_error == RADIO_ERROR_NONE);
		}
		LOG_INF("prop tx done");
		break;
	case SID_PAL_RADIO_EVENT_RX_ERROR:
	case SID_PAL_RADIO_EVENT_RX_TIMEOUT:
	case SID_PAL_RADIO_EVENT_TX_TIMEOUT:
	case SID_PAL_RADIO_EVENT_HEADER_ERROR:
		LOG_WRN("prop radio event %d", event);
		break;
	default:
		LOG_DBG("prop radio event %d", event);
		break;
	}

	k_mutex_unlock(&radio_state.lock);
}

void prop_radio_irq_handler(void)
{
	int32_t err = sid_pal_radio_irq_process();

	if (err != RADIO_ERROR_NONE) {
		k_mutex_lock(&radio_state.lock, K_FOREVER);
		radio_state.last_radio_error = err;
		k_mutex_unlock(&radio_state.lock);
		LOG_ERR("prop irq process failed: %d", err);
	}
}

void prop_radio_event_request(sidewalk_ctx_t *sid, void *ctx)
{
	struct prop_radio_request *req = (struct prop_radio_request *)ctx;

	if (req == NULL) {
		return;
	}

	k_mutex_lock(&radio_state.lock, K_FOREVER);

	switch (req->op) {
	case PROP_RADIO_REQ_CLAIM:
		(void)claim_radio_locked(sid, req);
		break;
	case PROP_RADIO_REQ_RELEASE:
		(void)release_radio_locked(sid, req);
		break;
	case PROP_RADIO_REQ_CONFIG:
		(void)config_radio_locked(sid, req);
		break;
	case PROP_RADIO_REQ_TX:
		(void)tx_radio_locked(sid, req);
		break;
	case PROP_RADIO_REQ_RX_START:
		(void)rx_start_locked(sid, req);
		break;
	case PROP_RADIO_REQ_RX_STOP:
		(void)rx_stop_locked(req);
		break;
	default:
		set_failure(req, -EINVAL, SID_ERROR_NONE, RADIO_ERROR_NONE);
		break;
	}

	k_mutex_unlock(&radio_state.lock);
	k_sem_give(&req->done);
}

int prop_radio_request_submit(struct prop_radio_request *req)
{
	int err;

	if (req == NULL) {
		return -EINVAL;
	}

	k_sem_init(&req->done, 0, 1);
	req->result = -EINPROGRESS;
	req->sid_error = SID_ERROR_NONE;
	req->radio_error = RADIO_ERROR_NONE;

	err = sidewalk_event_send(prop_radio_event_request, req, NULL);
	if (err != 0) {
		return err;
	}

	err = k_sem_take(&req->done, K_FOREVER);
	if (err != 0) {
		return err;
	}

	return req->result;
}

void prop_radio_get_status_snapshot(struct prop_radio_status_snapshot *status)
{
	if (status == NULL) {
		return;
	}

	k_mutex_lock(&radio_state.lock, K_FOREVER);
	status->module_ready = radio_state.module_ready;
	status->claimed = radio_state.claimed;
	status->rx_enabled = radio_state.rx_enabled;
	status->rx_running = radio_state.rx_running;
	status->sidewalk_subghz_paused = radio_state.sidewalk_subghz_paused;
	status->radio_state = sid_pal_radio_get_status();
	status->last_sid_error = radio_state.last_sid_error;
	status->last_radio_error = radio_state.last_radio_error;
	status->config = radio_state.config;
	k_mutex_unlock(&radio_state.lock);
}
