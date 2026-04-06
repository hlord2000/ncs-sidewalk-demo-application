/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef SAMPLE_CLOUD_BRIDGE_H
#define SAMPLE_CLOUD_BRIDGE_H

#include <sid_api.h>

void cloud_bridge_on_msg_received(const struct sid_msg_desc *msg_desc, const struct sid_msg *msg);

#endif /* SAMPLE_CLOUD_BRIDGE_H */
