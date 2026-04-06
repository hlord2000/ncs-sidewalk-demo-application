/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef SAMPLE_INPUT_TRIGGER_H
#define SAMPLE_INPUT_TRIGGER_H

#include <sid_api.h>

int input_trigger_init(void);

void input_trigger_on_msg_sent(const struct sid_msg_desc *msg_desc);

void input_trigger_on_send_error(const struct sid_msg_desc *msg_desc);

#endif /* SAMPLE_INPUT_TRIGGER_H */
