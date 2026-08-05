/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <sensor_monitoring/app_nfc.h>

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <nfc_t2t_lib.h>
#include <nfc/ndef/msg.h>
#include <nfc/ndef/text_rec.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

LOG_MODULE_REGISTER(app_nfc, CONFIG_SIDEWALK_LOG_LEVEL);

#define NFC_NDEF_MSG_BUF_SIZE 160
#define NFC_TEXT_PAYLOAD_SIZE 120
#define DEVICE_ID_MAX_SIZE 16
#define DEVICE_ID_HEX_SIZE ((DEVICE_ID_MAX_SIZE * 2) + 1)

static uint8_t ndef_msg_buf[NFC_NDEF_MSG_BUF_SIZE];
static uint8_t nfc_text_payload[NFC_TEXT_PAYLOAD_SIZE];
static const uint8_t nfc_lang_code[] = { 'e', 'n' };

static void nfc_callback(void *context, nfc_t2t_event_t event, const uint8_t *data,
			 size_t data_length)
{
	ARG_UNUSED(context);
	ARG_UNUSED(data);
	ARG_UNUSED(data_length);

	switch (event) {
	case NFC_T2T_EVENT_FIELD_ON:
		LOG_INF("NFC field on");
		break;
	case NFC_T2T_EVENT_FIELD_OFF:
		LOG_INF("NFC field off");
		break;
	case NFC_T2T_EVENT_DATA_READ:
		LOG_INF("NFC identity read");
		break;
	default:
		break;
	}
}

static void bytes_to_hex(const uint8_t *bytes, size_t bytes_len, char *hex, size_t hex_len)
{
	size_t pos = 0;

	for (size_t i = 0; i < bytes_len && pos + 2 < hex_len; i++) {
		pos += snprintk(&hex[pos], hex_len - pos, "%02x", bytes[i]);
	}

	hex[pos] = '\0';
}

static int nfc_identity_payload_prepare(void)
{
	uint8_t device_id[DEVICE_ID_MAX_SIZE] = { 0 };
	char device_id_hex[DEVICE_ID_HEX_SIZE] = "unknown";
	ssize_t device_id_len = hwinfo_get_device_id(device_id, sizeof(device_id));

	if (device_id_len > 0) {
		bytes_to_hex(device_id, (size_t)device_id_len, device_id_hex,
			     sizeof(device_id_hex));
	}

	int len = snprintk((char *)nfc_text_payload, sizeof(nfc_text_payload),
			   "sidewalk-devkit id=%s ble=%s", device_id_hex,
			   CONFIG_SIDEWALK_BLE_NAME);

	if (len < 0 || len >= (int)sizeof(nfc_text_payload)) {
		return -EMSGSIZE;
	}

	return len;
}

static int nfc_identity_msg_encode(uint8_t *buffer, uint32_t *len)
{
	int payload_len = nfc_identity_payload_prepare();

	if (payload_len < 0) {
		return payload_len;
	}

	NFC_NDEF_TEXT_RECORD_DESC_DEF(nfc_id_text_rec, UTF_8, nfc_lang_code,
				      sizeof(nfc_lang_code), nfc_text_payload, payload_len);
	NFC_NDEF_MSG_DEF(nfc_id_msg, 1);

	int err = nfc_ndef_msg_record_add(&NFC_NDEF_MSG(nfc_id_msg),
					  &NFC_NDEF_TEXT_RECORD_DESC(nfc_id_text_rec));

	if (err) {
		return err;
	}

	return nfc_ndef_msg_encode(&NFC_NDEF_MSG(nfc_id_msg), buffer, len);
}

int app_nfc_init(void)
{
	uint32_t len = sizeof(ndef_msg_buf);
	int err = nfc_t2t_setup(nfc_callback, NULL);

	if (err) {
		LOG_WRN("NFC T2T setup failed %d", err);
		return err;
	}

	err = nfc_identity_msg_encode(ndef_msg_buf, &len);
	if (err) {
		LOG_WRN("NFC identity encode failed %d", err);
		return err;
	}

	err = nfc_t2t_payload_set(ndef_msg_buf, len);
	if (err) {
		LOG_WRN("NFC payload set failed %d", err);
		return err;
	}

	err = nfc_t2t_emulation_start();
	if (err) {
		LOG_WRN("NFC emulation start failed %d", err);
		return err;
	}

	LOG_INF("NFC identity tag started");
	return 0;
}
