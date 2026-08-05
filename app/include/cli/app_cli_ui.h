/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef APP_CLI_UI_H_
#define APP_CLI_UI_H_

#include <stdbool.h>
#include <stdint.h>

#include <sidewalk.h>
#include <zephyr/bluetooth/gatt.h>

struct shell;

enum app_cli_ui_activity {
	APP_CLI_UI_ACTIVITY_TX,
	APP_CLI_UI_ACTIVITY_RX,
	APP_CLI_UI_ACTIVITY_ERROR,
};

int app_cli_ui_init(sidewalk_ctx_t *sid);

int app_cli_ui_nus_shell_init(void);

int app_cli_ui_print_identity(const struct shell *shell);

int app_cli_ui_request_ship_mode(void);

void app_cli_ui_notify_activity(enum app_cli_ui_activity activity);

void app_cli_ui_notify_shell_connected(void);

void app_cli_ui_notify_shell_disconnected(void);

bool app_cli_ui_bt_attr_is_nus(const struct bt_gatt_attr *attr);

/**
 * @brief Report the current Sidewalk status to a connected NUS shell client.
 *
 * Emits a single structured "EVT:{...}" line over the NUS shell transport so a
 * Web Bluetooth client can render live device feedback. Safe to call from any
 * thread; it is a no-op when no NUS client is connected.
 *
 * @param registered  True if the device is registered with Sidewalk.
 * @param time_synced True if Sidewalk time is synchronized.
 * @param link_mask   Bitmask of currently up links (SID_LINK_TYPE_*).
 */
void app_cli_ui_report_status(bool registered, bool time_synced, uint32_t link_mask);

/**
 * @brief Report a Sidewalk Location library update to a connected NUS client.
 */
void app_cli_ui_report_location(int status, int error, int mode, int link);

/**
 * @brief Report a one-off device activity event to a connected NUS shell client.
 *
 * Emits "EVT:{"t":<type>,"v":<value>}". Safe to call from any thread; a no-op
 * when no NUS client is connected.
 *
 * @param type  Short event tag, e.g. "tx", "rx", "err".
 * @param value Associated value (message id, length, or error code).
 */
void app_cli_ui_report_event(const char *type, int value);

#endif /* APP_CLI_UI_H_ */
