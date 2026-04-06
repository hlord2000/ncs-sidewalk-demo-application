/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef PROP_RADIO_H
#define PROP_RADIO_H

#include <sid_error.h>
#include <sid_pal_radio_ifc.h>
#include <sidewalk.h>

#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PROP_RADIO_DEFAULT_FREQUENCY_HZ 915000000U
#define PROP_RADIO_DEFAULT_TX_POWER_DBM 10
#define PROP_RADIO_MAX_PAYLOAD_LEN 240U

enum prop_radio_request_op {
	PROP_RADIO_REQ_CLAIM = 0,
	PROP_RADIO_REQ_RELEASE,
	PROP_RADIO_REQ_CONFIG,
	PROP_RADIO_REQ_TX,
	PROP_RADIO_REQ_RX_START,
	PROP_RADIO_REQ_RX_STOP,
};

struct prop_radio_config {
	uint32_t frequency_hz;
	int8_t tx_power_dbm;
	uint8_t spreading_factor;
	uint8_t bandwidth;
	uint8_t coding_rate;
	uint16_t preamble_length;
	uint16_t sync_word;
	bool invert_iq;
	bool crc_on;
};

struct prop_radio_status_snapshot {
	bool claimed;
	bool module_ready;
	bool rx_enabled;
	bool rx_running;
	bool sidewalk_subghz_paused;
	uint8_t radio_state;
	sid_error_t last_sid_error;
	int32_t last_radio_error;
	struct prop_radio_config config;
};

struct prop_radio_request {
	enum prop_radio_request_op op;
	struct k_sem done;
	int result;
	sid_error_t sid_error;
	int32_t radio_error;
	union {
		struct prop_radio_config config;
		struct {
			size_t len;
			uint8_t payload[PROP_RADIO_MAX_PAYLOAD_LEN];
		} tx;
	} params;
};

void prop_radio_platform_init(void);
void prop_radio_event_notifier(sid_pal_radio_events_t event);
void prop_radio_irq_handler(void);
void prop_radio_event_request(sidewalk_ctx_t *sid, void *ctx);

int prop_radio_request_submit(struct prop_radio_request *req);
void prop_radio_get_status_snapshot(struct prop_radio_status_snapshot *status);

#ifdef __cplusplus
}
#endif

#endif /* PROP_RADIO_H */
