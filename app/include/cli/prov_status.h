/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef PROV_STATUS_H_
#define PROV_STATUS_H_

struct shell;

/**
 * @brief Report the boot-time Sidewalk provisioning state.
 *
 * Reads the mfg store version and, if provisioned, the SMSN, and emits
 * "EVT:{"t":"prov","provisioned":<bool>,"smsn":"<64 hex chars or empty>",
 * "mfg_ver":<uint>}".
 *
 * Pass NULL to target the NUS shell backend only; this is a no-op when no
 * NUS client is connected. Pass the invoking shell from a command handler
 * to also print a human-readable summary and receive the EVT line there.
 *
 * The mfg store version is cached at boot by sid_pal_mfg_store_init() and
 * does not change if the manufacturing page is rewritten at runtime, so the
 * result always reflects the state at the last boot.
 *
 * @param shell Shell to report to, or NULL for the NUS shell backend only.
 */
void prov_status_report(const struct shell *shell);

/**
 * @brief Notify prov_status that a NUS shell client has connected.
 *
 * Schedules a short delayed provisioning-status report on the NUS shell so
 * a browser that connects after boot still learns the state without
 * sending a command. Safe to call from a Bluetooth connection callback;
 * does not block.
 */
void prov_status_notify_nus_connected(void);

#endif /* PROV_STATUS_H_ */
