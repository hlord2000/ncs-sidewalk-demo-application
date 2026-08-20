/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <cli/prov_status.h>

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <app_mfg_config.h>
#include <sid_base64.h>
#include <sid_error.h>
#include <sid_pal_mfg_store_ifc.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

#if defined(CONFIG_SID_END_DEVICE_NUS_SHELL)
#include <zephyr/shell/shell_backend.h>
#endif /* CONFIG_SID_END_DEVICE_NUS_SHELL */

LOG_MODULE_REGISTER(prov_status, CONFIG_SIDEWALK_LOG_LEVEL);

#define PROV_STATUS_BOOT_REPORT_DELAY_MS 3000
#define PROV_STATUS_NUS_CONNECT_REPORT_DELAY_MS 300
#define PROV_REBOOT_FLUSH_DELAY_MS 300

#define CMD_PROV_STATUS_DESCRIPTION                                                                \
	"\n"                                                                                       \
	"Report whether this boot found a valid Sidewalk manufacturing page.\n"                    \
	"Prints a human-readable summary and an EVT:{\"t\":\"prov\",...} line."
#define CMD_PROV_STATUS_ARG_REQUIRED 1
#define CMD_PROV_STATUS_ARG_OPTIONAL 0

#define CMD_PROV_ERASE_DESCRIPTION                                                                 \
	"\n"                                                                                       \
	"Erase the Sidewalk manufacturing store and forget any pending prov set session.\n"        \
	"Requires CONFIG_SIDEWALK_MFG_STORAGE_DIAGNOSTIC (overlay-dut.conf)."
#define CMD_PROV_ERASE_ARG_REQUIRED 1
#define CMD_PROV_ERASE_ARG_OPTIONAL 0

#define CMD_PROV_SET_DESCRIPTION                                                                   \
	"<value_id> <total_len> <frag_index> <base64>\n"                                          \
	"Write one manufacturing store value, named by its numeric sid_pal_mfg_store_value_t id.\n" \
	"total_len is the full decoded byte length of the value. frag_index starts at 0 and must\n" \
	"be sent in order; a value is committed to the store once the fragments received add up\n" \
	"to total_len. Emits EVT:{\"t\":\"provwr\",...} when the value is fully written."
#define CMD_PROV_SET_ARG_REQUIRED 5
#define CMD_PROV_SET_ARG_OPTIONAL 0

#define CMD_PROV_FINALIZE_DESCRIPTION                                                              \
	"\n"                                                                                       \
	"Write the mfg flags and version last, after checking every required value id has been\n"  \
	"written. Emits EVT:{\"t\":\"provdone\",...}. Reboot afterward for it to take effect."
#define CMD_PROV_FINALIZE_ARG_REQUIRED 1
#define CMD_PROV_FINALIZE_ARG_OPTIONAL 0

#define CMD_PROV_REBOOT_DESCRIPTION "\nClean reboot, needed for new credentials to take effect."
#define CMD_PROV_REBOOT_ARG_REQUIRED 1
#define CMD_PROV_REBOOT_ARG_OPTIONAL 0

static void prov_status_read(bool *provisioned, uint32_t *mfg_ver, char *smsn_hex,
			      size_t smsn_hex_size)
{
	uint8_t smsn[SID_PAL_MFG_STORE_SMSN_SIZE];
	uint16_t smsn_size;

	*provisioned = !app_mfg_cfg_is_empty();
	*mfg_ver = sid_pal_mfg_store_get_version();
	smsn_hex[0] = '\0';

	if (!*provisioned) {
		return;
	}

	smsn_size = sid_pal_mfg_store_get_length_for_value(SID_PAL_MFG_STORE_SMSN);
	if (smsn_size != sizeof(smsn)) {
		LOG_ERR("Sidewalk SMSN has invalid size %u", smsn_size);
		return;
	}

	if (smsn_hex_size < (sizeof(smsn) * 2) + 1) {
		LOG_ERR("SMSN hex buffer is too small");
		return;
	}

	sid_pal_mfg_store_read(SID_PAL_MFG_STORE_SMSN, smsn, sizeof(smsn));
	for (size_t i = 0; i < sizeof(smsn); i++) {
		snprintk(&smsn_hex[i * 2], 3, "%02X", smsn[i]);
	}
}

#if defined(CONFIG_SID_END_DEVICE_NUS_SHELL)
static const struct shell *prov_status_nus_shell(void)
{
	return shell_backend_get_by_name("shell_bt_nus");
}

static void prov_status_boot_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	prov_status_report(NULL);
}
static K_WORK_DELAYABLE_DEFINE(prov_status_boot_work, prov_status_boot_work_handler);

static void prov_status_nus_connect_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	prov_status_report(NULL);
}
static K_WORK_DELAYABLE_DEFINE(prov_status_nus_connect_work, prov_status_nus_connect_work_handler);
#endif /* CONFIG_SID_END_DEVICE_NUS_SHELL */

void prov_status_report(const struct shell *shell)
{
	bool provisioned;
	uint32_t mfg_ver;
	char smsn_hex[(SID_PAL_MFG_STORE_SMSN_SIZE * 2) + 1];
	const struct shell *report_shell = shell;

	prov_status_read(&provisioned, &mfg_ver, smsn_hex, sizeof(smsn_hex));

	if (shell != NULL) {
		shell_print(shell, "Sidewalk provisioning: %s",
			    provisioned ? "provisioned" : "not provisioned");
		shell_print(shell, "Manufacturing page version: %u", mfg_ver);
		shell_print(shell, "Sidewalk manufacturing serial: %s",
			    provisioned ? smsn_hex : "(none)");
	}

#if defined(CONFIG_SID_END_DEVICE_NUS_SHELL)
	if (report_shell == NULL) {
		report_shell = prov_status_nus_shell();
	}
#endif /* CONFIG_SID_END_DEVICE_NUS_SHELL */

	if (report_shell == NULL) {
		return;
	}

	shell_print(report_shell,
		    "EVT:{\"t\":\"prov\",\"provisioned\":%s,\"smsn\":\"%s\",\"mfg_ver\":%u}",
		    provisioned ? "true" : "false", smsn_hex, mfg_ver);
}

void prov_status_notify_nus_connected(void)
{
#if defined(CONFIG_SID_END_DEVICE_NUS_SHELL)
	(void)k_work_reschedule(&prov_status_nus_connect_work,
				 K_MSEC(PROV_STATUS_NUS_CONNECT_REPORT_DELAY_MS));
#endif /* CONFIG_SID_END_DEVICE_NUS_SHELL */
}

#if defined(CONFIG_SID_END_DEVICE_NUS_SHELL)
static int prov_status_sys_init(void)
{
	(void)k_work_schedule(&prov_status_boot_work, K_MSEC(PROV_STATUS_BOOT_REPORT_DELAY_MS));

	return 0;
}

SYS_INIT(prov_status_sys_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
#endif /* CONFIG_SID_END_DEVICE_NUS_SHELL */

#if defined(CONFIG_SHELL)

/*
 * Writable manufacturing store values. This mirrors the field list that
 * sid_on_dev_cert_verify_and_store() writes (sid_on_dev_cert.c), minus the
 * device id / serial number / DTID fields that flow does not write either,
 * PLUS the two device private keys, which that reference flow deliberately
 * skips (see is_secret below for why this flow needs them anyway).
 *
 * The largest value is 64 bytes (P256R1 public keys and signatures); serial
 * numbers are nominally 4 bytes but the certificate format allows an
 * extended encoding, so they accept up to PROV_SERIAL_MAX_LEN.
 */
#define PROV_SET_MAX_VALUE_LEN 64u
#define PROV_SERIAL_MAX_LEN 20u
#define PROV_SET_MAX_B64_LEN SID_BASE64_ENCODED_LENGTH(PROV_SET_MAX_VALUE_LEN)

struct prov_value_spec {
	uint16_t id;
	uint16_t min_len;
	uint16_t max_len;
	const char *name;
	/*
	 * True for the two raw device private keys (ids 6 and 9). They come
	 * from AWS in the certificate in this flow (unlike on-device cert
	 * generation, where they are born inside PSA and never touch flash
	 * as plaintext), so prov_set() must still write them to the mfg
	 * store as ordinary TLV values; sid_pal_mfg_store_init()'s
	 * parse_mfg_raw_tlv() migration imports them into PSA on the next
	 * boot and strips them back out of flash (sid_mfg_hex_v8.c).
	 *
	 * Two consequences of that migration not having run yet on this
	 * boot: sid_pal_mfg_store_read() for these ids is intercepted by
	 * sid_mfg_storage_secure_read() -> sid_crypto_keys_buffer_set(),
	 * which looks up a PSA key that does not exist until after the
	 * migration and fails without touching the output buffer, so a
	 * write-then-read-back verification would always misreport a
	 * mismatch even though the flash write succeeded. prov_set()
	 * skips the read-back check for is_secret values and trusts the
	 * write() return code instead. It also omits the byte count from
	 * the human-readable confirmation for these values, since there is
	 * no reason to echo anything about secret material over the shell.
	 */
	bool is_secret;
};

static const struct prov_value_spec prov_value_table[] = {
	{ SID_PAL_MFG_STORE_SMSN, 32, 32, "smsn" },
	{ SID_PAL_MFG_STORE_APID, 4, 4, "apid" },
	{ SID_PAL_MFG_STORE_APP_PUB_ED25519, 32, 32, "app_pub_ed25519" },
	{ SID_PAL_MFG_STORE_DEVICE_PRIV_ED25519, 32, 32, "device_priv_ed25519", true },
	{ SID_PAL_MFG_STORE_DEVICE_PRIV_P256R1, 32, 32, "device_priv_p256r1", true },
	{ SID_PAL_MFG_STORE_DEVICE_PUB_ED25519, 32, 32, "device_pub_ed25519" },
	{ SID_PAL_MFG_STORE_DEVICE_PUB_ED25519_SIGNATURE, 64, 64, "device_pub_ed25519_sig" },
	{ SID_PAL_MFG_STORE_DEVICE_PUB_P256R1, 64, 64, "device_pub_p256r1" },
	{ SID_PAL_MFG_STORE_DEVICE_PUB_P256R1_SIGNATURE, 64, 64, "device_pub_p256r1_sig" },
	{ SID_PAL_MFG_STORE_DAK_PUB_ED25519, 32, 32, "dak_pub_ed25519" },
	{ SID_PAL_MFG_STORE_DAK_PUB_ED25519_SIGNATURE, 64, 64, "dak_pub_ed25519_sig" },
	{ SID_PAL_MFG_STORE_DAK_ED25519_SERIAL, 4, PROV_SERIAL_MAX_LEN, "dak_ed25519_serial" },
	{ SID_PAL_MFG_STORE_DAK_PUB_P256R1, 64, 64, "dak_pub_p256r1" },
	{ SID_PAL_MFG_STORE_DAK_PUB_P256R1_SIGNATURE, 64, 64, "dak_pub_p256r1_sig" },
	{ SID_PAL_MFG_STORE_DAK_P256R1_SERIAL, 4, PROV_SERIAL_MAX_LEN, "dak_p256r1_serial" },
	{ SID_PAL_MFG_STORE_PRODUCT_PUB_ED25519, 32, 32, "product_pub_ed25519" },
	{ SID_PAL_MFG_STORE_PRODUCT_PUB_ED25519_SIGNATURE, 64, 64, "product_pub_ed25519_sig" },
	{ SID_PAL_MFG_STORE_PRODUCT_ED25519_SERIAL, 4, PROV_SERIAL_MAX_LEN,
	  "product_ed25519_serial" },
	{ SID_PAL_MFG_STORE_PRODUCT_PUB_P256R1, 64, 64, "product_pub_p256r1" },
	{ SID_PAL_MFG_STORE_PRODUCT_PUB_P256R1_SIGNATURE, 64, 64, "product_pub_p256r1_sig" },
	{ SID_PAL_MFG_STORE_PRODUCT_P256R1_SERIAL, 4, PROV_SERIAL_MAX_LEN,
	  "product_p256r1_serial" },
	{ SID_PAL_MFG_STORE_MAN_PUB_ED25519, 32, 32, "man_pub_ed25519" },
	{ SID_PAL_MFG_STORE_MAN_PUB_ED25519_SIGNATURE, 64, 64, "man_pub_ed25519_sig" },
	{ SID_PAL_MFG_STORE_MAN_ED25519_SERIAL, 4, PROV_SERIAL_MAX_LEN, "man_ed25519_serial" },
	{ SID_PAL_MFG_STORE_MAN_PUB_P256R1, 64, 64, "man_pub_p256r1" },
	{ SID_PAL_MFG_STORE_MAN_PUB_P256R1_SIGNATURE, 64, 64, "man_pub_p256r1_sig" },
	{ SID_PAL_MFG_STORE_MAN_P256R1_SERIAL, 4, PROV_SERIAL_MAX_LEN, "man_p256r1_serial" },
	{ SID_PAL_MFG_STORE_SW_PUB_ED25519, 32, 32, "sw_pub_ed25519" },
	{ SID_PAL_MFG_STORE_SW_PUB_ED25519_SIGNATURE, 64, 64, "sw_pub_ed25519_sig" },
	{ SID_PAL_MFG_STORE_SW_ED25519_SERIAL, 4, PROV_SERIAL_MAX_LEN, "sw_ed25519_serial" },
	{ SID_PAL_MFG_STORE_SW_PUB_P256R1, 64, 64, "sw_pub_p256r1" },
	{ SID_PAL_MFG_STORE_SW_PUB_P256R1_SIGNATURE, 64, 64, "sw_pub_p256r1_sig" },
	{ SID_PAL_MFG_STORE_SW_P256R1_SERIAL, 4, PROV_SERIAL_MAX_LEN, "sw_p256r1_serial" },
	{ SID_PAL_MFG_STORE_AMZN_PUB_ED25519, 32, 32, "amzn_pub_ed25519" },
	{ SID_PAL_MFG_STORE_AMZN_PUB_P256R1, 64, 64, "amzn_pub_p256r1" },
};

/* Bit i sees value id i written and read-back verified since the last erase. */
static uint64_t prov_written_mask;

struct prov_set_session {
	bool active;
	uint16_t value_id;
	uint16_t total_len;
	uint32_t next_frag_index;
	const struct prov_value_spec *spec;
	struct sid_base64_ctx b64;
	uint8_t buffer[PROV_SET_MAX_VALUE_LEN];
};

static struct prov_set_session prov_set_session;

static const struct prov_value_spec *prov_value_spec_find(uint32_t value_id)
{
	for (size_t i = 0; i < ARRAY_SIZE(prov_value_table); i++) {
		if (prov_value_table[i].id == value_id) {
			return &prov_value_table[i];
		}
	}

	return NULL;
}

static void prov_set_session_reset(void)
{
	memset(&prov_set_session, 0, sizeof(prov_set_session));
}

static bool prov_parse_u32(const char *arg, uint32_t *out)
{
	char *end = NULL;
	unsigned long value;

	if (arg == NULL || *arg == '\0') {
		return false;
	}

	value = strtoul(arg, &end, 10);
	if (*end != '\0' || value > UINT32_MAX) {
		return false;
	}

	*out = (uint32_t)value;
	return true;
}

static int cmd_prov_status(const struct shell *shell, int32_t argc, const char **argv)
{
	ARG_UNUSED(argv);

	if (argc != CMD_PROV_STATUS_ARG_REQUIRED) {
		return -EINVAL;
	}

	prov_status_report(shell);

	return 0;
}

static int cmd_prov_erase(const struct shell *shell, int32_t argc, const char **argv)
{
	ARG_UNUSED(argv);

	if (argc != CMD_PROV_ERASE_ARG_REQUIRED) {
		return -EINVAL;
	}

	int32_t err = sid_pal_mfg_store_erase();

	prov_set_session_reset();
	prov_written_mask = 0;

	if (err == (int32_t)SID_ERROR_NOSUPPORT) {
		shell_error(shell,
			    "mfg store diagnostic writes are disabled; rebuild with "
			    "CONFIG_SID_END_DEVICE_CLI=y, which selects it");
		return -ENOTSUP;
	}
	if (err) {
		shell_error(shell, "mfg store erase failed (%d)", err);
		return -EIO;
	}

	shell_print(shell, "mfg store erased");
	return 0;
}

static int cmd_prov_set(const struct shell *shell, int32_t argc, const char **argv)
{
	uint32_t value_id;
	uint32_t total_len;
	uint32_t frag_index;
	const char *fragment;
	size_t fragment_len;
	sid_error_t decode_err;

	if (argc != CMD_PROV_SET_ARG_REQUIRED) {
		return -EINVAL;
	}

	if (!prov_parse_u32(argv[1], &value_id) || !prov_parse_u32(argv[2], &total_len) ||
	    !prov_parse_u32(argv[3], &frag_index) || total_len > UINT16_MAX) {
		shell_error(shell, "invalid value_id, total_len or frag_index");
		return -EINVAL;
	}
	fragment = argv[4];
	fragment_len = strlen(fragment);

	if (fragment_len > PROV_SET_MAX_B64_LEN) {
		shell_error(shell, "fragment too long, max %u base64 chars",
			    (unsigned int)PROV_SET_MAX_B64_LEN);
		prov_set_session_reset();
		return -EMSGSIZE;
	}

	if (frag_index == 0) {
		const struct prov_value_spec *spec = prov_value_spec_find(value_id);

		if (spec == NULL) {
			shell_error(shell, "unknown value id %u", value_id);
			return -EINVAL;
		}
		if (total_len < spec->min_len || total_len > spec->max_len) {
			shell_error(shell, "value id %u (%s) expects %u..%u bytes, got %u",
				    value_id, spec->name, spec->min_len, spec->max_len,
				    total_len);
			return -EINVAL;
		}

		prov_set_session_reset();
		prov_set_session.active = true;
		prov_set_session.value_id = (uint16_t)value_id;
		prov_set_session.total_len = (uint16_t)total_len;
		prov_set_session.spec = spec;
		sid_base64_init(&prov_set_session.b64);
		prov_set_session.b64.next_out = prov_set_session.buffer;
		prov_set_session.b64.avail_out = total_len;
	} else {
		if (!prov_set_session.active || prov_set_session.value_id != value_id ||
		    prov_set_session.total_len != total_len) {
			shell_error(shell, "no matching pending value; send frag_index 0 first");
			return -ENOENT;
		}
		if (frag_index != prov_set_session.next_frag_index) {
			shell_error(shell, "unexpected frag_index %u, expected %u", frag_index,
				    prov_set_session.next_frag_index);
			prov_set_session_reset();
			return -EINVAL;
		}
	}

	prov_set_session.b64.next_in = (const uint8_t *)fragment;
	prov_set_session.b64.avail_in = fragment_len;
	decode_err = sid_base64_decode(&prov_set_session.b64);
	if (decode_err != SID_ERROR_NONE || prov_set_session.b64.avail_in != 0) {
		shell_error(shell, "malformed base64 fragment or value overflow");
		prov_set_session_reset();
		return -EINVAL;
	}
	prov_set_session.next_frag_index++;

	if (prov_set_session.b64.total_out < prov_set_session.total_len) {
		shell_print(shell, "fragment %u accepted, %u/%u bytes", frag_index,
			    (unsigned int)prov_set_session.b64.total_out,
			    prov_set_session.total_len);
		return 0;
	}

	/* Value complete: write it once, then read it back to confirm, unless
	 * this is a secret value (see is_secret in struct prov_value_spec):
	 * sid_pal_mfg_store_read() for those ids is intercepted by the PSA
	 * lookup path before the next-boot migration has run, and a read-back
	 * check would always report a mismatch even on a successful write.
	 */
	bool ok = false;
	bool is_secret = prov_set_session.spec->is_secret;
	int32_t wr = sid_pal_mfg_store_write(value_id, prov_set_session.buffer,
					     prov_set_session.total_len);

	if (wr == (int32_t)SID_ERROR_NOSUPPORT) {
		shell_error(shell,
			    "mfg store diagnostic writes are disabled; rebuild with "
			    "CONFIG_SID_END_DEVICE_CLI=y, which selects it");
	} else if (wr != 0) {
		shell_error(shell, "value id %u write failed (%d)", value_id, wr);
	} else if (is_secret) {
		ok = true;
	} else {
		uint8_t readback[PROV_SET_MAX_VALUE_LEN];

		sid_pal_mfg_store_read((uint16_t)value_id, readback, prov_set_session.total_len);
		if (memcmp(readback, prov_set_session.buffer, prov_set_session.total_len) == 0) {
			ok = true;
		} else {
			shell_error(shell, "value id %u read-back mismatch", value_id);
		}
		memset(readback, 0, sizeof(readback));
	}

	if (ok) {
		prov_written_mask |= (1ULL << value_id);
		if (is_secret) {
			shell_print(shell, "value id %u (%s) stored", value_id,
				    prov_set_session.spec->name);
		} else {
			shell_print(shell, "value id %u (%s) stored, %u bytes", value_id,
				    prov_set_session.spec->name, prov_set_session.total_len);
		}
	}

	shell_print(shell, "EVT:{\"t\":\"provwr\",\"id\":%u,\"ok\":%s}", value_id,
		    ok ? "true" : "false");

	prov_set_session_reset();
	return ok ? 0 : -EIO;
}

static void prov_finalize_list_missing(char *out, size_t out_size)
{
	size_t used = 0;

	out[0] = '\0';
	for (size_t i = 0; i < ARRAY_SIZE(prov_value_table); i++) {
		int n;

		if (prov_written_mask & (1ULL << prov_value_table[i].id)) {
			continue;
		}

		n = snprintk(out + used, out_size - used, "%s%s", (used == 0) ? "" : ",",
			     prov_value_table[i].name);
		if (n < 0 || (size_t)n >= (out_size - used)) {
			break;
		}
		used += (size_t)n;
	}
}

static int cmd_prov_finalize(const struct shell *shell, int32_t argc, const char **argv)
{
	char missing[192];
	uint8_t version[SID_PAL_MFG_STORE_VERSION_SIZE] = { 0, 0, 0, SID_PAL_MFG_STORE_TLV_VERSION };
	int32_t wr;

	ARG_UNUSED(argv);
	if (argc != CMD_PROV_FINALIZE_ARG_REQUIRED) {
		return -EINVAL;
	}

	prov_finalize_list_missing(missing, sizeof(missing));
	if (missing[0] != '\0') {
		shell_error(shell, "cannot finalize, missing values: %s", missing);
		shell_print(shell, "EVT:{\"t\":\"provdone\",\"ok\":false,\"err\":\"missing values\"}");
		return -ENODATA;
	}

	/*
	 * Deliberately do not write the mfg_flags entry (MFG_FLAGS_TYPE_ID)
	 * here. Leaving it absent makes sid_pal_mfg_store_init() read -ENODATA
	 * for it on the next boot, which sets need_to_parse and runs
	 * parse_mfg_raw_tlv() (sid_mfg_hex_v8.c). That is the code path that
	 * imports value ids 6 and 9 into PSA, strips them back out of the
	 * rebuilt flash page, and then writes mfg_flags itself with the
	 * correct initialized/keys_in_psa values. Writing keys_in_psa=1
	 * ourselves here would be a lie (the keys are still plaintext in
	 * flash, not in PSA, until that migration runs) and would skip the
	 * migration entirely, leaving the device with no usable signing keys
	 * and no indication anything was wrong.
	 *
	 * The version/magic header is still written last, and still on its
	 * own: sid_pal_mfg_store_init() only considers the device provisioned
	 * once it can read the "SID0" magic and version at the very start of
	 * the partition, and parse_mfg_raw_tlv() only starts reading TLV
	 * entries from start_offset + tlv_storage_start_marker_size onward,
	 * i.e. right after that same header, so the header must exist for
	 * either the boot-time provisioned check or the migration to do
	 * anything. Writing it last still preserves the interrupted-write
	 * safety property: if finalize is interrupted before this write, the
	 * header still never matched, and the next boot reports the device
	 * unprovisioned rather than provisioned with missing or un-migrated
	 * data. This is the opposite order of sid_on_dev_cert_verify_and_store(),
	 * which writes the version right after erase and can leave a device
	 * that reports itself provisioned when the write was in fact
	 * interrupted.
	 */
	wr = sid_pal_mfg_store_write(SID_PAL_MFG_STORE_VERSION, version, sizeof(version));
	if (wr == (int32_t)SID_ERROR_NOSUPPORT) {
		shell_error(shell,
			    "mfg store diagnostic writes are disabled; rebuild with "
			    "CONFIG_SID_END_DEVICE_CLI=y, which selects it");
		shell_print(shell, "EVT:{\"t\":\"provdone\",\"ok\":false,\"err\":\"diagnostic disabled\"}");
		return -ENOTSUP;
	}
	if (wr) {
		shell_error(shell, "failed to write mfg version (%d); device left unprovisioned",
			    wr);
		shell_print(shell, "EVT:{\"t\":\"provdone\",\"ok\":false,\"err\":\"version write failed\"}");
		return -EIO;
	}

	shell_print(shell, "provisioning finalized, reboot required to run the PSA key "
			   "migration and take effect");
	shell_print(shell, "EVT:{\"t\":\"provdone\",\"ok\":true,\"err\":\"\"}");

	prov_written_mask = 0;
	return 0;
}

static int cmd_prov_reboot(const struct shell *shell, int32_t argc, const char **argv)
{
	ARG_UNUSED(argv);
	if (argc != CMD_PROV_REBOOT_ARG_REQUIRED) {
		return -EINVAL;
	}

	shell_print(shell, "rebooting to apply provisioning changes");
	LOG_INF("prov reboot requested");
	LOG_PANIC();
	k_sleep(K_MSEC(PROV_REBOOT_FLUSH_DELAY_MS));
	sys_reboot(SYS_REBOOT_WARM);

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_prov,
	SHELL_CMD_ARG(status, NULL, CMD_PROV_STATUS_DESCRIPTION, cmd_prov_status,
		      CMD_PROV_STATUS_ARG_REQUIRED, CMD_PROV_STATUS_ARG_OPTIONAL),
	SHELL_CMD_ARG(erase, NULL, CMD_PROV_ERASE_DESCRIPTION, cmd_prov_erase,
		      CMD_PROV_ERASE_ARG_REQUIRED, CMD_PROV_ERASE_ARG_OPTIONAL),
	SHELL_CMD_ARG(set, NULL, CMD_PROV_SET_DESCRIPTION, cmd_prov_set, CMD_PROV_SET_ARG_REQUIRED,
		      CMD_PROV_SET_ARG_OPTIONAL),
	SHELL_CMD_ARG(finalize, NULL, CMD_PROV_FINALIZE_DESCRIPTION, cmd_prov_finalize,
		      CMD_PROV_FINALIZE_ARG_REQUIRED, CMD_PROV_FINALIZE_ARG_OPTIONAL),
	SHELL_CMD_ARG(reboot, NULL, CMD_PROV_REBOOT_DESCRIPTION, cmd_prov_reboot,
		      CMD_PROV_REBOOT_ARG_REQUIRED, CMD_PROV_REBOOT_ARG_OPTIONAL),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(prov, &sub_prov, "Sidewalk provisioning", NULL);
#endif /* CONFIG_SHELL */
