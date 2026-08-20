/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef APP_MEMFAULT_H
#define APP_MEMFAULT_H

#include <stdbool.h>
#include <stdint.h>

#include <sidewalk.h>

struct shell;

/**
 * @brief One time Memfault integration setup.
 *
 * Restricts the packetizer to the Event data source (coredumps are never
 * drained over the radio), and seeds the Memfault device serial from the
 * Sidewalk SMSN. Call once from app_start(), before app_start_tasks().
 */
void app_memfault_init(void);

/**
 * @brief Drain queued Memfault chunks and send them as Sidewalk uplinks.
 *
 * Must only be called from the Sidewalk app TX thread: all Memfault
 * packetizer calls are required to happen on a single thread, and this
 * function is the only place that touches the packetizer.
 *
 * Sends at most CONFIG_SID_END_DEVICE_MEMFAULT_DRAIN_MAX_CHUNKS chunks. Each
 * chunk is queued onto the Sidewalk thread the same way regular telemetry is
 * queued; this call does not block waiting for the send to complete.
 */
void app_memfault_drain(void);

/**
 * @brief Refresh the cached battery snapshot used to answer Memfault battery
 * metric queries.
 *
 * Must be called from the app TX thread right after a sensor sample was
 * already taken there (e.g. in the APP_EVENT_NOTIFY_SENSOR handler). Never
 * triggers its own sensor I/O.
 *
 * @param pmic_valid               True if the PMIC fields below are valid.
 * @param battery_millivolts       Battery voltage in millivolts.
 * @param battery_level_percent    Fuel gauge state of charge, in percent.
 * @param vbus_present             True if external power (VBUS) is present.
 * @param charger_status           Raw charger status bitmask (0 = idle).
 * @param temperature_valid        True if temperature_millicelsius is valid.
 * @param temperature_millicelsius Board temperature, in millicelsius.
 */
void app_memfault_battery_sample_update(bool pmic_valid, int32_t battery_millivolts,
					int16_t battery_level_percent, bool vbus_present,
					int charger_status, bool temperature_valid,
					int32_t temperature_millicelsius);

/** @brief Increment the count of Sidewalk uplinks sent. Safe from any thread. */
void app_memfault_metric_uplink_sent(void);

/** @brief Increment the count of Sidewalk send errors. Safe from any thread. */
void app_memfault_metric_send_error(void);

/** @brief Increment the count of Sidewalk link status changes. Safe from any thread. */
void app_memfault_metric_link_status_changed(void);

/**
 * @brief Check whether the Memfault packetizer currently has data queued.
 *
 * Returns a cached flag refreshed by app_memfault_drain() rather than calling
 * into the packetizer directly, so it is safe to call from any thread (e.g.
 * shell commands) without violating the packetizer's single-thread rule.
 */
bool app_memfault_chunk_pending(void);

/**
 * @brief Sidewalk-thread event handler that refreshes the cached link MTU.
 *
 * Queued via sidewalk_event_send() (see app.c's on_sidewalk_status_changed)
 * because sid_get_mtu() is a sid_* SDK call and must run on the Sidewalk
 * thread. The result is cached for app_memfault_drain() (running on the app
 * TX thread) to size its chunk budget without calling into the SDK itself.
 */
void sidewalk_event_mflt_mtu_query(sidewalk_ctx_t *sid, void *ctx);

/* Shell command implementations, kept out of app_shell.c so it only needs to
 * wire up SHELL_CMD_ARG entries. See app/src/cli/app_shell.c "mflt" group.
 */
int app_memfault_shell_info(const struct shell *shell);
int app_memfault_shell_drain(const struct shell *shell);
int app_memfault_shell_heartbeat(const struct shell *shell);
int app_memfault_shell_crash(const struct shell *shell, int crash_type);
int app_memfault_shell_reboot(const struct shell *shell);

#endif /* APP_MEMFAULT_H */
