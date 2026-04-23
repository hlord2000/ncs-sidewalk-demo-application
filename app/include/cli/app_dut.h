/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef APP_DUT_H
#define APP_DUT_H

#include <sidewalk.h>

typedef struct {
	uint32_t target_link_mask;
	sidewalk_msg_t send;
} dut_flow_send_ctx_t;

void dut_event_init(sidewalk_ctx_t *sid, void *ctx);
void dut_event_deinit(sidewalk_ctx_t *sid, void *ctx);
void dut_event_start(sidewalk_ctx_t *sid, void *ctx);
void dut_event_stop(sidewalk_ctx_t *sid, void *ctx);
void dut_event_get_mtu(sidewalk_ctx_t *sid, void *ctx);
void dut_event_get_time(sidewalk_ctx_t *sid, void *ctx);
void dut_event_get_status(sidewalk_ctx_t *sid, void *ctx);
void dut_event_get_option(sidewalk_ctx_t *sid, void *ctx);
void dut_event_set_option(sidewalk_ctx_t *sid, void *ctx);
void dut_event_set_dest_id(sidewalk_ctx_t *sid, void *ctx);
void dut_event_conn_req(sidewalk_ctx_t *sid, void *ctx);
void dut_event_flow_switch(sidewalk_ctx_t *sid, void *ctx);
void dut_event_flow_set(sidewalk_ctx_t *sid, void *ctx);
void dut_event_flow_send(sidewalk_ctx_t *sid, void *ctx);
void dut_event_flow_status(sidewalk_ctx_t *sid, void *ctx);
void dut_event_flow_cancel(sidewalk_ctx_t *sid, void *ctx);
void dut_event_flow_on_status(sidewalk_ctx_t *sid);
void dut_flow_notify_msg_sent(const struct sid_msg_desc *msg_desc);
void dut_flow_notify_send_error(sid_error_t error, const struct sid_msg_desc *msg_desc);
void dut_flow_send_ctx_free(void *ctx);

#endif /* APP_DUT_H */
