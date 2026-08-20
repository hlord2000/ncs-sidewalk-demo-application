/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#pragma once

/*
 * Platform overrides for the default configuration settings in the
 * memfault-firmware-sdk. Default configuration settings can be found in
 * "modules/lib/memfault-firmware-sdk/components/include/memfault/default_config.h"
 */

/* Collect heartbeat metrics every minute instead of the default hour, so the
 * demo has data to show without waiting around.
 */
#define MEMFAULT_METRICS_HEARTBEAT_INTERVAL_SECS 60
