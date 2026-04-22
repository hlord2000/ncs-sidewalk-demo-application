/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef APP_CLI_UI_H_
#define APP_CLI_UI_H_

#include <stdbool.h>

#include <sidewalk.h>
#include <zephyr/bluetooth/gatt.h>

enum app_cli_ui_activity {
	APP_CLI_UI_ACTIVITY_TX,
	APP_CLI_UI_ACTIVITY_RX,
	APP_CLI_UI_ACTIVITY_ERROR,
};

int app_cli_ui_init(sidewalk_ctx_t *sid);

void app_cli_ui_notify_activity(enum app_cli_ui_activity activity);

void app_cli_ui_notify_shell_connected(void);

void app_cli_ui_notify_shell_disconnected(void);

bool app_cli_ui_bt_attr_is_nus(const struct bt_gatt_attr *attr);

#endif /* APP_CLI_UI_H_ */
