/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <memfault/app_memfault.h>

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

#if defined(CONFIG_SHELL)
#include <zephyr/shell/shell.h>
#include <zephyr/sys/base64.h>
#endif
#if defined(CONFIG_SID_END_DEVICE_NUS_SHELL)
#include <zephyr/shell/shell_backend.h>
#endif

#include <memfault/core/data_packetizer.h>
#include <memfault/panics/assert.h>
#include <memfault/core/platform/device_info.h>
#include <memfault/demo/cli.h>
#include <memfault/metrics/metrics.h>
#include <memfault/metrics/platform/battery.h>
#include <memfault_ncs.h>

#include <sid_api.h>
#include <sid_hal_memory_ifc.h>
#include <sid_pal_mfg_store_ifc.h>
#include <sidewalk.h>
#include <sensor_monitoring/app_tx.h>

LOG_MODULE_REGISTER(app_memfault, CONFIG_SIDEWALK_LOG_LEVEL);

/* Wire format for a Memfault chunk uplink, fixed contract with the backend:
 *   byte 0      : 0xC0 tag identifying a Memfault chunk uplink
 *   byte 1      : uint8 sequence, monotonic, wraps at 256
 *   bytes 2..N  : raw bytes from memfault_packetizer_get_chunk(), unmodified
 */
#define APP_MEMFAULT_CHUNK_TAG 0xC0
#define APP_MEMFAULT_CHUNK_HDR_SIZE 2
#define APP_MEMFAULT_CHUNK_PAYLOAD_MAX 96
/*
 * Used when sid_get_mtu() has not answered yet. 19 bytes is the smallest
 * Sidewalk BLE payload, so this is safe on any link and still leaves 17 bytes
 * for chunk data after the 2 byte header, above MEMFAULT_PACKETIZER_MIN_BUF_LEN.
 */
#define APP_MEMFAULT_CHUNK_FALLBACK_PAYLOAD 19
#define APP_MEMFAULT_CHUNK_BUF_SIZE (APP_MEMFAULT_CHUNK_HDR_SIZE + APP_MEMFAULT_CHUNK_PAYLOAD_MAX)

#define APP_MEMFAULT_SID_MSG_TTL_S 60
#define APP_MEMFAULT_SID_MSG_RETRIES 1

/* Single static chunk buffer. All packetizer calls (and therefore all use of
 * this buffer) happen on the app TX thread, see app_memfault_drain().
 */
static uint8_t s_chunk_buf[APP_MEMFAULT_CHUNK_BUF_SIZE];
static uint8_t s_chunk_seq;

static atomic_t s_chunk_pending;
static atomic_t s_cached_mtu;

struct app_memfault_battery_snapshot {
	bool valid;
	uint32_t soc_pct;
	bool discharging;
};

static struct app_memfault_battery_snapshot s_battery_snapshot;
static struct k_spinlock s_battery_lock;

void app_memfault_init(void)
{
	/* Coredumps are never drained over the radio; only small Event data
	 * (heartbeats, reboot info, trace events) is worth the airtime.
	 */
	memfault_packetizer_set_active_sources(kMfltDataSourceMask_Event);

	uint16_t smsn_size = sid_pal_mfg_store_get_length_for_value(SID_PAL_MFG_STORE_SMSN);
	uint8_t smsn[SID_PAL_MFG_STORE_SMSN_SIZE];

	if (smsn_size != sizeof(smsn)) {
		LOG_WRN("Sidewalk SMSN has invalid size %u, Memfault device id left as default",
			smsn_size);
		return;
	}

	sid_pal_mfg_store_read(SID_PAL_MFG_STORE_SMSN, smsn, sizeof(smsn));

	char smsn_hex[(SID_PAL_MFG_STORE_SMSN_SIZE * 2) + 1];

	for (size_t i = 0; i < sizeof(smsn); i++) {
		snprintk(&smsn_hex[i * 2], 3, "%02X", smsn[i]);
	}

	int err = memfault_ncs_device_id_set(smsn_hex, strlen(smsn_hex));

	if (err) {
		LOG_ERR("Failed to set Memfault device id from SMSN (err %d)", err);
	}

	LOG_INF("Memfault integration ready");
}

static void app_memfault_chunk_ctx_free(void *ctx)
{
	sidewalk_msg_t *msg = (sidewalk_msg_t *)ctx;

	if (msg == NULL) {
		return;
	}
	if (msg->msg.data) {
		sid_hal_free(msg->msg.data);
	}
	sid_hal_free(msg);
}

static int app_memfault_chunk_msg_send(const uint8_t *payload, size_t payload_size)
{
	sidewalk_msg_t *msg = sid_hal_malloc(sizeof(*msg));

	if (msg == NULL) {
		LOG_ERR("Failed to alloc memfault chunk msg context");
		return -ENOMEM;
	}

	memset(msg, 0, sizeof(*msg));
	msg->msg.size = payload_size;
	msg->msg.data = sid_hal_malloc(payload_size);
	if (msg->msg.data == NULL) {
		sid_hal_free(msg);
		LOG_ERR("Failed to alloc memfault chunk payload");
		return -ENOMEM;
	}
	memcpy(msg->msg.data, payload, payload_size);

	msg->desc = (struct sid_msg_desc){
		.link_type = SID_LINK_TYPE_ANY,
		.type = SID_MSG_TYPE_NOTIFY,
		.link_mode = SID_LINK_MODE_CLOUD,
		.msg_desc_attr.tx_attr.request_ack = false,
		.msg_desc_attr.tx_attr.num_retries = APP_MEMFAULT_SID_MSG_RETRIES,
		.msg_desc_attr.tx_attr.ttl_in_seconds = APP_MEMFAULT_SID_MSG_TTL_S,
	};

	int err = sidewalk_event_send(sidewalk_event_send_msg, msg, app_memfault_chunk_ctx_free);

	if (err) {
		app_memfault_chunk_ctx_free(msg);
		LOG_ERR("Memfault chunk event send err %d", err);
		return -EIO;
	}

	return 0;
}

#if defined(CONFIG_SID_END_DEVICE_NUS_SHELL)
static void app_memfault_report_chunk_event(uint8_t seq, size_t len, bool pending)
{
	const struct shell *sh = shell_backend_get_by_name("shell_bt_nus");

	if (sh == NULL) {
		return;
	}

	shell_print(sh, "EVT:{\"t\":\"mflt\",\"seq\":%u,\"len\":%u,\"pending\":%s}",
		    (unsigned int)seq, (unsigned int)len, pending ? "true" : "false");
}
#endif

/*
 * Base64 of the largest chunk we ask for, plus the NUL that base64_encode
 * writes. Kept static because the app TX thread stack is 4096 bytes.
 */
#define APP_MEMFAULT_EXPORT_B64_LEN (((APP_MEMFAULT_CHUNK_PAYLOAD_MAX + 2) / 3) * 4 + 1)
static uint8_t s_export_chunk[APP_MEMFAULT_CHUNK_PAYLOAD_MAX];
static char s_export_b64[APP_MEMFAULT_EXPORT_B64_LEN];

void app_memfault_export_chunk(void)
{
	if (!memfault_packetizer_data_available()) {
		LOG_INF("MFLT_CHUNK: none");
		return;
	}

	size_t chunk_len = sizeof(s_export_chunk);

	if (!memfault_packetizer_get_chunk(s_export_chunk, &chunk_len) || chunk_len == 0) {
		LOG_WRN("Memfault export found no chunk to read");
		return;
	}

	size_t written = 0;
	int err = base64_encode(s_export_b64, sizeof(s_export_b64), &written, s_export_chunk,
				chunk_len);

	if (err) {
		LOG_ERR("Memfault chunk base64 encode failed %d (chunk %u bytes)", err,
			(unsigned int)chunk_len);
		return;
	}

	/*
	 * Print on the log backend (UART) and, when a client is attached, the NUS
	 * shell, so the chunk can be collected over either transport. The prefix
	 * is what a host script greps for.
	 */
	LOG_INF("MFLT_CHUNK:%s", s_export_b64);

#if defined(CONFIG_SID_END_DEVICE_NUS_SHELL)
	const struct shell *sh = shell_backend_get_by_name("shell_bt_nus");

	if (sh != NULL) {
		shell_print(sh, "MFLT_CHUNK:%s", s_export_b64);
	}
#endif
}

void app_memfault_drain(void)
{
	uint32_t mtu = (uint32_t)atomic_get(&s_cached_mtu);
	uint32_t budget;

	if (mtu > APP_MEMFAULT_CHUNK_HDR_SIZE) {
		budget = MIN(mtu - APP_MEMFAULT_CHUNK_HDR_SIZE,
			     (uint32_t)APP_MEMFAULT_CHUNK_PAYLOAD_MAX);
	} else {
		/*
		 * No MTU cached yet. Do not refuse to send: a smaller chunk still
		 * carries data and the packetizer splits records to fit whatever it
		 * is given. Falling back to the Sidewalk BLE minimum keeps device
		 * health flowing on a link whose MTU could not be read, which is
		 * exactly when that data is worth having.
		 */
		budget = APP_MEMFAULT_CHUNK_FALLBACK_PAYLOAD;
		LOG_DBG("Memfault chunk MTU unknown, using %u byte fallback budget", budget);
	}

	if (budget < (MEMFAULT_PACKETIZER_MIN_BUF_LEN + APP_MEMFAULT_CHUNK_HDR_SIZE)) {
		LOG_WRN("Memfault chunk budget %u too small (mtu %u), skipping drain", budget,
			mtu);
		return;
	}

	for (int sent = 0; sent < CONFIG_SID_END_DEVICE_MEMFAULT_DRAIN_MAX_CHUNKS; sent++) {
		if (!memfault_packetizer_data_available()) {
			atomic_set(&s_chunk_pending, 0);
			break;
		}

		size_t buf_len = budget;
		bool ok = memfault_packetizer_get_chunk(&s_chunk_buf[APP_MEMFAULT_CHUNK_HDR_SIZE],
							&buf_len);

		if (!ok || buf_len == 0) {
			atomic_set(&s_chunk_pending, 0);
			break;
		}

		s_chunk_buf[0] = APP_MEMFAULT_CHUNK_TAG;
		s_chunk_buf[1] = s_chunk_seq++;

		bool pending = memfault_packetizer_data_available();

		atomic_set(&s_chunk_pending, pending ? 1 : 0);

		int err = app_memfault_chunk_msg_send(s_chunk_buf,
						      APP_MEMFAULT_CHUNK_HDR_SIZE + buf_len);
		if (err) {
			LOG_ERR("Memfault chunk send failed %d", err);
			break;
		}

#if defined(CONFIG_SID_END_DEVICE_NUS_SHELL)
		app_memfault_report_chunk_event(s_chunk_buf[1], buf_len, pending);
#endif

		if (!pending) {
			break;
		}
	}
}

void app_memfault_battery_sample_update(bool pmic_valid, int32_t battery_millivolts,
					int16_t battery_level_percent, bool vbus_present,
					int charger_status, bool temperature_valid,
					int32_t temperature_millicelsius)
{
	if (pmic_valid) {
		int32_t pct = CLAMP((int32_t)battery_level_percent, 0, 100);

		k_spinlock_key_t key = k_spin_lock(&s_battery_lock);
		s_battery_snapshot.valid = true;
		s_battery_snapshot.soc_pct = (uint32_t)pct;
		/* Not discharging while external power is present or the
		 * charger reports an active charging state.
		 */
		s_battery_snapshot.discharging = !(vbus_present || (charger_status != 0));
		k_spin_unlock(&s_battery_lock, key);

		uint32_t mv = (battery_millivolts > 0) ? (uint32_t)battery_millivolts : 0;

		MEMFAULT_METRIC_SET_UNSIGNED(sid_battery_mv, mv);
	}

	if (temperature_valid) {
		MEMFAULT_METRIC_SET_SIGNED(sid_temperature_millic, temperature_millicelsius);
	}
}

#if defined(CONFIG_MEMFAULT_METRICS_BATTERY_ENABLE)
int memfault_platform_get_stateofcharge(sMfltPlatformBatterySoc *soc)
{
	if (soc == NULL) {
		return -1;
	}

	k_spinlock_key_t key = k_spin_lock(&s_battery_lock);
	bool valid = s_battery_snapshot.valid;
	uint32_t pct = s_battery_snapshot.soc_pct;
	bool discharging = s_battery_snapshot.discharging;
	k_spin_unlock(&s_battery_lock, key);

	if (!valid) {
		return -1;
	}

	soc->soc = pct;
	soc->discharging = discharging;
	return 0;
}
#endif /* CONFIG_MEMFAULT_METRICS_BATTERY_ENABLE */

void app_memfault_metric_uplink_sent(void)
{
	MEMFAULT_METRIC_ADD(sid_uplink_count, 1);
}

void app_memfault_metric_send_error(void)
{
	MEMFAULT_METRIC_ADD(sid_send_error_count, 1);
}

void app_memfault_metric_link_status_changed(void)
{
	MEMFAULT_METRIC_ADD(sid_link_status_change_count, 1);
}

bool app_memfault_chunk_pending(void)
{
	/*
	 * Ask the packetizer, not the cached flag. s_chunk_pending only gets
	 * written inside the drain loop, so reporting it here reads "no" whenever
	 * a drain has not run yet, which is exactly when you want to know whether
	 * data is queued.
	 */
	return memfault_packetizer_data_available();
}

void sidewalk_event_mflt_mtu_query(sidewalk_ctx_t *sid, void *ctx)
{
	ARG_UNUSED(ctx);

	if (sid == NULL || sid->handle == NULL) {
		return;
	}

	/*
	 * sid_get_mtu() wants one concrete link, not SID_LINK_TYPE_ANY, which it
	 * rejects. Ask each link in turn and keep the first answer. Logged at
	 * warning level because a silent failure here left the drain with a zero
	 * budget and no data ever left the device.
	 */
	static const uint32_t links[] = { SID_LINK_TYPE_1, SID_LINK_TYPE_2, SID_LINK_TYPE_3 };

	for (size_t i = 0; i < ARRAY_SIZE(links); i++) {
		size_t mtu = 0;
		sid_error_t e = sid_get_mtu(sid->handle, links[i], &mtu);

		if (e == SID_ERROR_NONE && mtu > 0) {
			atomic_set(&s_cached_mtu, (atomic_val_t)mtu);
			LOG_INF("Memfault chunk MTU %u from link type %u", (unsigned int)mtu,
				(unsigned int)links[i]);
			return;
		}
	}

	LOG_WRN("Memfault could not read an MTU from any link; drain will use its fallback");
}

/* Crash and reboot triggers that do not need a shell.
 *
 * Reachable from a Sidewalk downlink (see app_rx.c) so a crash can be requested
 * remotely: the device faults, the Memfault fault handler records the reason,
 * and the reboot event is drained over Sidewalk on the next boot. Note that no
 * coredump is uploaded -- app_memfault_init() restricts the packetizer to event
 * data -- so what shows up in Memfault is a reboot with its reason, not a
 * symbolicated stack trace.
 */
int app_memfault_trigger_crash(int crash_type)
{
	switch (crash_type) {
	case APP_MEMFAULT_CRASH_ASSERT:
		LOG_WRN("Memfault crash requested: MEMFAULT_ASSERT(0)");
		MEMFAULT_ASSERT(0);
		return 0;
	case APP_MEMFAULT_CRASH_HARDFAULT: {
		LOG_WRN("Memfault crash requested: HardFault");
		/* Call through an address that cannot be executed. The fault
		 * handler runs before the reset, so the reason is recorded.
		 */
		void (*bad_func)(void) = (void (*)(void))0xEEEEDEAD;

		bad_func();
		return 0;
	}
	default:
		LOG_ERR("Unknown Memfault crash type %d", crash_type);
		return -EINVAL;
	}
}

int app_memfault_trigger_reboot(void)
{
	LOG_INF("Memfault reboot requested");
	return memfault_demo_cli_cmd_system_reboot(0, NULL);
}

#if defined(CONFIG_SHELL)

int app_memfault_shell_info(const struct shell *shell)
{
	sMemfaultDeviceInfo info = { 0 };

	memfault_platform_get_device_info(&info);

	shell_print(shell, "Memfault device serial  : %s", info.device_serial);
	shell_print(shell, "Memfault software type  : %s", info.software_type);
	shell_print(shell, "Memfault software ver   : %s", info.software_version);
	shell_print(shell, "Memfault hardware ver   : %s", info.hardware_version);
	shell_print(shell, "Memfault chunk pending  : %s",
		    app_memfault_chunk_pending() ? "yes" : "no");

	return 0;
}

int app_memfault_shell_export(const struct shell *shell)
{
	int err = app_tx_event_send(APP_EVENT_MFLT_EXPORT);

	if (err) {
		shell_error(shell, "Failed to queue a Memfault chunk export (err %d)", err);
		return err;
	}

	shell_print(shell, "Memfault chunk export queued");
	return 0;
}

int app_memfault_shell_drain(const struct shell *shell)
{
	int err = app_tx_event_send(APP_EVENT_MFLT_DRAIN);

	if (err) {
		shell_error(shell, "Failed to queue drain cycle (err %d)", err);
		return err;
	}

	shell_print(shell, "Memfault drain cycle queued");
	return 0;
}

int app_memfault_shell_heartbeat(const struct shell *shell)
{
	memfault_metrics_heartbeat_debug_trigger();
	shell_print(shell, "Memfault heartbeat collected");
	return 0;
}

int app_memfault_shell_crash(const struct shell *shell, int crash_type)
{
	switch (crash_type) {
	case APP_MEMFAULT_CRASH_ASSERT:
		shell_print(shell, "Triggering MEMFAULT_ASSERT(0)");
		return app_memfault_trigger_crash(APP_MEMFAULT_CRASH_ASSERT);
	case APP_MEMFAULT_CRASH_HARDFAULT:
		shell_print(shell, "Triggering a HardFault");
		return app_memfault_trigger_crash(APP_MEMFAULT_CRASH_HARDFAULT);
	default:
		shell_error(shell, "Usage: \"mflt crash <n>\" where n is 0 (assert) or 1 (hardfault)");
		return -EINVAL;
	}
}

int app_memfault_shell_reboot(const struct shell *shell)
{
	shell_print(shell, "Rebooting");
	return app_memfault_trigger_reboot();
}

#endif /* CONFIG_SHELL */
