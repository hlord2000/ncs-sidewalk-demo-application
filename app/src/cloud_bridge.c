/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <cloud_bridge.h>

#include "json_printer/sidTypes2str.h"
#include <sidewalk.h>
#include <sid_error.h>
#include <sid_hal_memory_ifc.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <string.h>

LOG_MODULE_REGISTER(cloud_bridge, CONFIG_SIDEWALK_LOG_LEVEL);

#define DOWNLINK_RECEIPT_MAX_BYTES 24U
#define DOWNLINK_RECEIPT_MAX_HEX_CHARS (DOWNLINK_RECEIPT_MAX_BYTES * 2U)
#define DOWNLINK_RECEIPT_FORMAT "{\"event\":\"downlink_rx\",\"link\":\"%s\",\"hex\":\"%s\"}"

static void free_cloud_bridge_ctx(void *ctx)
{
	sidewalk_msg_t *msg = (sidewalk_msg_t *)ctx;

	if (msg == NULL) {
		return;
	}

	if (msg->msg.data != NULL) {
		sid_hal_free(msg->msg.data);
	}

	sid_hal_free(msg);
}

static const char *link_type_to_name(uint32_t link_type)
{
	switch (link_type) {
	case SID_LINK_TYPE_1:
		return "BLE";
	case SID_LINK_TYPE_2:
		return "FSK";
	case SID_LINK_TYPE_3:
		return "LoRa";
	default:
		return "UNKNOWN";
	}
}

static void hex_encode_limited(char *dst, size_t dst_size, const uint8_t *src, size_t src_size)
{
	static const char hex_chars[] = "0123456789abcdef";
	size_t bytes_to_encode = MIN(src_size, (size_t)DOWNLINK_RECEIPT_MAX_BYTES);
	size_t pos = 0U;

	if (dst_size == 0U) {
		return;
	}

	for (size_t i = 0U; i < bytes_to_encode; i++) {
		if ((pos + 2U) >= dst_size) {
			break;
		}

		dst[pos++] = hex_chars[(src[i] >> 4) & 0x0F];
		dst[pos++] = hex_chars[src[i] & 0x0F];
	}

	dst[pos] = '\0';
}

static void cloud_bridge_send_event(sidewalk_ctx_t *sid, void *ctx)
{
	sidewalk_msg_t *msg = (sidewalk_msg_t *)ctx;

	if ((sid == NULL) || (sid->handle == NULL)) {
		LOG_WRN("Ignoring downlink receipt while Sidewalk is not started");
		return;
	}

	if (msg == NULL) {
		LOG_ERR("Downlink receipt context is NULL");
		return;
	}

	sid_error_t err = sid_put_msg(sid->handle, &msg->msg, &msg->desc);

	if (err != SID_ERROR_NONE) {
		LOG_ERR("Failed to queue downlink receipt: %d (%s)", (int)err, SID_ERROR_T_STR(err));
		return;
	}

	LOG_INF("Queued downlink receipt event on ANY link, id=%u", msg->desc.id);
}

void cloud_bridge_on_msg_received(const struct sid_msg_desc *msg_desc, const struct sid_msg *msg)
{
	char payload_hex[DOWNLINK_RECEIPT_MAX_HEX_CHARS + 1U];
	char receipt_payload[sizeof(DOWNLINK_RECEIPT_FORMAT) + 8U + DOWNLINK_RECEIPT_MAX_HEX_CHARS];
	sidewalk_msg_t *receipt = NULL;
	int err;

	if ((msg_desc == NULL) || (msg == NULL) || (msg->data == NULL) || (msg->size == 0U)) {
		return;
	}

	if (msg_desc->msg_desc_attr.rx_attr.is_msg_ack) {
		return;
	}

	hex_encode_limited(payload_hex, sizeof(payload_hex), msg->data, msg->size);

	err = snprintk(receipt_payload, sizeof(receipt_payload), DOWNLINK_RECEIPT_FORMAT,
		       link_type_to_name(msg_desc->link_type), payload_hex);
	if ((err < 0) || ((size_t)err >= sizeof(receipt_payload))) {
		LOG_ERR("Failed to encode downlink receipt payload");
		return;
	}

	receipt = sid_hal_malloc(sizeof(*receipt));
	if (receipt == NULL) {
		LOG_ERR("Failed to allocate downlink receipt context");
		return;
	}

	memset(receipt, 0, sizeof(*receipt));

	receipt->msg.size = (size_t)err;
	receipt->msg.data = sid_hal_malloc(receipt->msg.size);
	if (receipt->msg.data == NULL) {
		LOG_ERR("Failed to allocate downlink receipt payload");
		sid_hal_free(receipt);
		return;
	}

	memcpy(receipt->msg.data, receipt_payload, receipt->msg.size);

	receipt->desc.type = SID_MSG_TYPE_NOTIFY;
	receipt->desc.link_type = SID_LINK_TYPE_ANY;
	receipt->desc.link_mode = SID_LINK_MODE_CLOUD;

	err = sidewalk_event_send(cloud_bridge_send_event, receipt, free_cloud_bridge_ctx);
	if (err != 0) {
		LOG_ERR("Failed to enqueue downlink receipt event: %d", err);
		free_cloud_bridge_ctx(receipt);
	}
}
