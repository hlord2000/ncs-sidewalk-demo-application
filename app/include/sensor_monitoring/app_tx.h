/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef APP_TX_H
#define APP_TX_H

#include <stdint.h>

typedef enum event_type {
	APP_EVENT_TIME_SYNC_SUCCESS,
	APP_EVENT_TIME_SYNC_FAIL,
	APP_EVENT_CAPABILITY_SUCCESS,
	APP_EVENT_NOTIFY_SENSOR,
	APP_EVENT_NOTIFY_BUTTON,
	APP_EVENT_RESP_LED_ON,
	APP_EVENT_RESP_LED_OFF,
	APP_EVENT_MFLT_DRAIN,
	/* Pull one chunk and print it base64 on the shell instead of sending it
	 * over Sidewalk. Lets a host collect Memfault data over BLE from a device
	 * with no Sidewalk link, which is exactly when it is most useful.
	 */
	APP_EVENT_MFLT_EXPORT,
} app_event_t;

int app_tx_event_send(app_event_t event);

void app_tx_last_link_mask_set(uint32_t link_mask);

void app_tx_task(void *dummy1, void *dummy2, void *dummy3);

#endif /* APP_TX_H */
