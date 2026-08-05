/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdbool.h>
#include <errno.h>
#include <limits.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include <app_subGHz_config.h>
#include <semtech_radio_ifc.h>
#include <sid_pal_radio_ifc.h>
#include <sx126x_config.h>
#include <sx126x_radio.h>

LOG_MODULE_REGISTER(fsk_probe, CONFIG_SIDEWALK_LOG_LEVEL);

#define FSK_PROBE_RX_TIMEOUT_US ((uint32_t)CONFIG_FSK_PROBE_RX_TIMEOUT_MS * 1000U)
#define FSK_PROBE_TX_TIMEOUT_US SID_PAL_RADIO_FSK_DEFAULT_TX_TIMEOUT
#define FSK_PROBE_PSDU_MAX 64U
#define FSK_PROBE_PSDU_LEN 18U
#define FSK_PROBE_TX_BUFFER_LEN (SID_MAX_CUSTOM_PHYHDR_SZ + FSK_PROBE_PSDU_MAX + sizeof(uint32_t))

#define SX126X_FSK_PHY_HEADER_LENGTH 2U
#define FSK_PROBE_FRAME_LEN (SX126X_FSK_PHY_HEADER_LENGTH + FSK_PROBE_PSDU_LEN + sizeof(uint16_t))
#define FSK_PROBE_TRIM_MAX 0x2FU

static K_SEM_DEFINE(radio_irq_sem, 0, 1);
K_MSGQ_DEFINE(radio_event_msgq, sizeof(sid_pal_radio_events_t), 8, 4);

static sid_pal_radio_rx_packet_t rx_packet;
static sid_pal_radio_fsk_modulation_params_t fsk_mod_params;
static sid_pal_radio_fsk_packet_params_t fsk_packet_params;
static sid_pal_radio_fsk_phy_hdr_t fsk_phy_hdr;
static uint8_t fsk_sync_word[SID_PAL_RADIO_FSK_SYNC_WORD_LENGTH];

struct rssi_stats {
	int16_t min;
	int16_t max;
	int16_t last;
	int32_t sum;
	uint32_t count;
};

static const char *event_name(sid_pal_radio_events_t event)
{
	switch (event) {
	case SID_PAL_RADIO_EVENT_TX_DONE:
		return "TX_DONE";
	case SID_PAL_RADIO_EVENT_RX_DONE:
		return "RX_DONE";
	case SID_PAL_RADIO_EVENT_RX_ERROR:
		return "RX_ERROR";
	case SID_PAL_RADIO_EVENT_TX_TIMEOUT:
		return "TX_TIMEOUT";
	case SID_PAL_RADIO_EVENT_RX_TIMEOUT:
		return "RX_TIMEOUT";
	case SID_PAL_RADIO_EVENT_HEADER_ERROR:
		return "HEADER_ERROR";
	case SID_PAL_RADIO_EVENT_SYNC_DET:
		return "SYNC_DET";
	default:
		return "OTHER";
	}
}

static int radio_call(const char *name, int err)
{
	if (err != RADIO_ERROR_NONE) {
		LOG_ERR("%s failed: %d", name, err);
	}

	return err;
}

static void radio_irq_handler(void)
{
	k_sem_give(&radio_irq_sem);
}

static void radio_event_notify(sid_pal_radio_events_t event)
{
	(void)k_msgq_put(&radio_event_msgq, &event, K_NO_WAIT);
}

static void apply_sync_override(void)
{
#if defined(CONFIG_FSK_PROBE_SHORT_SYNC)
	fsk_sync_word[0] = 0x90;
	fsk_sync_word[1] = 0x4e;
	fsk_packet_params.sync_word_length = 2U;
#elif defined(CONFIG_FSK_PROBE_LORAMAC_SYNC)
	fsk_sync_word[0] = 0xc1;
	fsk_sync_word[1] = 0x94;
	fsk_sync_word[2] = 0xc1;
	fsk_packet_params.sync_word_length = 3U;
#endif
}

static void rssi_stats_reset(struct rssi_stats *stats)
{
	stats->min = INT16_MAX;
	stats->max = INT16_MIN;
	stats->last = INT16_MAX;
	stats->sum = 0;
	stats->count = 0;
}

static void rssi_stats_sample(struct rssi_stats *stats)
{
	int16_t rssi = sid_pal_radio_rssi();

	if (rssi == INT16_MAX) {
		return;
	}

	if (rssi < stats->min) {
		stats->min = rssi;
	}
	if (rssi > stats->max) {
		stats->max = rssi;
	}

	stats->last = rssi;
	stats->sum += rssi;
	stats->count++;
}

static void log_rssi_stats(uint32_t window, const struct rssi_stats *stats)
{
	if (stats->count == 0U) {
		LOG_WRN("rx rssi window=%u samples=0", window);
		return;
	}

	LOG_INF("rx rssi window=%u samples=%u min=%d max=%d avg=%d last=%d",
		window, stats->count, stats->min, stats->max,
		(int)(stats->sum / (int32_t)stats->count), stats->last);
}

static int wait_radio_event(sid_pal_radio_events_t *event, int64_t timeout_ms,
			    struct rssi_stats *rssi_stats)
{
	const int64_t deadline = k_uptime_get() + timeout_ms;
	int64_t next_rssi_sample = k_uptime_get();
	int err;

	while (true) {
		err = k_msgq_get(&radio_event_msgq, event, K_NO_WAIT);
		if (err == 0) {
			return 0;
		}

		if (k_uptime_get() >= deadline) {
			return -ETIMEDOUT;
		}

		if (rssi_stats != NULL && CONFIG_FSK_PROBE_RX_RSSI_SAMPLE_MS > 0 &&
		    k_uptime_get() >= next_rssi_sample) {
			rssi_stats_sample(rssi_stats);
			next_rssi_sample = k_uptime_get() + CONFIG_FSK_PROBE_RX_RSSI_SAMPLE_MS;
		}

		(void)k_sem_take(&radio_irq_sem, K_MSEC(100));
		err = sid_pal_radio_irq_process();
		if (err != RADIO_ERROR_NONE) {
			LOG_ERR("irq_process failed: %d", err);
			return err;
		}
	}
}

static void log_raw_radio_status(uint32_t window)
{
	const halo_drv_semtech_ctx_t *ctx = sx126x_get_drv_ctx();
	sx126x_irq_mask_t irq_status = 0;
	uint8_t dio1 = 0xff;
	uint8_t busy = 0xff;
	sx126x_status_t irq_err;
	int32_t dio1_err;
	int32_t busy_err;

	irq_err = sx126x_get_irq_status(ctx, &irq_status);
	dio1_err = sid_pal_gpio_read(ctx->config->gpio_int1, &dio1);
	busy_err = sid_pal_gpio_read(ctx->config->gpio_radio_busy, &busy);

	LOG_WRN("rx raw window=%u irq_err=%d irq=0x%04x dio1_err=%d dio1=%u busy_err=%d busy=%u",
		window, irq_err, irq_status, dio1_err, dio1, busy_err, busy);
}

static int configure_common_radio(void)
{
	set_radio_sx126x_device_config(get_radio_cfg());

	if (radio_call("sid_pal_radio_init",
		       sid_pal_radio_init(radio_event_notify, radio_irq_handler, &rx_packet))) {
		return -EIO;
	}

	if (radio_call("sid_pal_radio_standby", sid_pal_radio_standby())) {
		return -EIO;
	}

	if (radio_call("sid_pal_radio_set_modem_mode",
		       sid_pal_radio_set_modem_mode(SID_PAL_RADIO_MODEM_MODE_FSK))) {
		return -EIO;
	}

	if (radio_call("sid_pal_radio_set_frequency",
		       sid_pal_radio_set_frequency(CONFIG_FSK_PROBE_FREQ_HZ))) {
		return -EIO;
	}

	if (radio_call("sid_pal_radio_set_tx_power",
		       sid_pal_radio_set_tx_power(CONFIG_FSK_PROBE_TX_POWER_DBM))) {
		return -EIO;
	}

	fsk_mod_params.bit_rate = CONFIG_FSK_PROBE_BIT_RATE_BPS;
	fsk_mod_params.freq_dev = CONFIG_FSK_PROBE_FREQ_DEV_HZ;
	fsk_mod_params.bandwidth = (uint8_t)CONFIG_FSK_PROBE_RX_BW_DSB_PARAM;
	fsk_mod_params.mod_shaping = (uint8_t)CONFIG_FSK_PROBE_MOD_SHAPING;
	fsk_mod_params.header_type = SID_PAL_RADIO_FSK_SIDEWALK_HEADER;

	if (radio_call("sid_pal_radio_set_fsk_modulation_params",
		       sid_pal_radio_set_fsk_modulation_params(&fsk_mod_params))) {
		return -EIO;
	}

	fsk_phy_hdr.fcs_type = RADIO_FSK_FCS_TYPE_1;
	fsk_phy_hdr.is_data_whitening_enabled = false;
	fsk_phy_hdr.is_fec_enabled = false;
	fsk_packet_params.preamble_length = CONFIG_FSK_PROBE_PREAMBLE_LEN_BYTES;

	LOG_INF("configured %u Hz FSK br=%u fdev=%u bw=0x%02x shape=0x%02x preamble=%u power %d dBm",
		(unsigned int)CONFIG_FSK_PROBE_FREQ_HZ,
		(unsigned int)fsk_mod_params.bit_rate,
		(unsigned int)fsk_mod_params.freq_dev,
		(unsigned int)fsk_mod_params.bandwidth,
		(unsigned int)fsk_mod_params.mod_shaping,
		(unsigned int)CONFIG_FSK_PROBE_PREAMBLE_LEN_BYTES,
		CONFIG_FSK_PROBE_TX_POWER_DBM);

	return 0;
}

static int configure_rx_packet(void)
{
	sid_pal_radio_fsk_pkt_cfg_t cfg = {
		.phy_hdr = &fsk_phy_hdr,
		.packet_params = &fsk_packet_params,
		.sync_word = fsk_sync_word,
	};

	fsk_packet_params.preamble_length = CONFIG_FSK_PROBE_PREAMBLE_LEN_BYTES;

	if (radio_call("sid_pal_radio_prepare_fsk_for_rx", sid_pal_radio_prepare_fsk_for_rx(&cfg))) {
		return -EIO;
	}

	fsk_packet_params.preamble_min_detect = (uint8_t)CONFIG_FSK_PROBE_PREAMBLE_MIN_DETECT;
	apply_sync_override();

	fsk_packet_params.payload_length = FSK_PROBE_FRAME_LEN;

	if (radio_call("sid_pal_radio_set_fsk_packet_params",
		       sid_pal_radio_set_fsk_packet_params(&fsk_packet_params))) {
		return -EIO;
	}

	if (radio_call("sid_pal_radio_set_fsk_sync_word",
		       sid_pal_radio_set_fsk_sync_word(fsk_sync_word,
						       fsk_packet_params.sync_word_length))) {
		return -EIO;
	}

	LOG_INF("rx packet sync=%02x %02x %02x sync_len=%u pbl_det=0x%02x payload_len=%u",
		fsk_sync_word[0], fsk_sync_word[1], fsk_sync_word[2],
		fsk_packet_params.sync_word_length,
		fsk_packet_params.preamble_min_detect,
		fsk_packet_params.payload_length);

	return 0;
}

static int run_rx(void)
{
	sid_pal_radio_events_t event;
	struct rssi_stats rssi_stats;
	uint32_t windows = 0;

	if (configure_common_radio() || configure_rx_packet()) {
		return -EIO;
	}

	while (true) {
		memset(&rx_packet, 0, sizeof(rx_packet));
		rssi_stats_reset(&rssi_stats);
		windows++;

#if defined(CONFIG_FSK_PROBE_SWEEP_FREQ)
		int32_t freq_span = CONFIG_FSK_PROBE_FREQ_SWEEP_STOP_HZ -
				    CONFIG_FSK_PROBE_FREQ_SWEEP_START_HZ;
		uint32_t freq_steps = (uint32_t)(freq_span / CONFIG_FSK_PROBE_FREQ_SWEEP_STEP_HZ) + 1U;
		int32_t freq_offset = CONFIG_FSK_PROBE_FREQ_SWEEP_START_HZ +
				      (int32_t)(((windows - 1U) % freq_steps) *
						CONFIG_FSK_PROBE_FREQ_SWEEP_STEP_HZ);
		uint32_t freq_hz = (uint32_t)((int32_t)CONFIG_FSK_PROBE_FREQ_HZ + freq_offset);
		int32_t freq_err = sid_pal_radio_set_frequency(freq_hz);

		LOG_INF("rx freq window=%u freq=%u offset=%d err=%d",
			windows, freq_hz, (int)freq_offset, (int)freq_err);
#endif

#if defined(CONFIG_FSK_PROBE_SWEEP_TRIM)
		uint16_t trim_step = (uint16_t)((windows - 1U) % (FSK_PROBE_TRIM_MAX + 1U));
		uint16_t trim = (uint16_t)((trim_step << 8) | trim_step);
		int32_t trim_err = semtech_radio_set_trim_cap_val(trim);

		LOG_INF("rx trim window=%u trim=0x%04x err=%d",
			windows, trim, (int)trim_err);
#endif

		if (radio_call("sid_pal_radio_start_rx", sid_pal_radio_start_rx(FSK_PROBE_RX_TIMEOUT_US))) {
			k_sleep(K_SECONDS(1));
			continue;
		}

		if (wait_radio_event(&event, CONFIG_FSK_PROBE_RX_TIMEOUT_MS + 500,
				     &rssi_stats) != 0) {
			log_rssi_stats(windows, &rssi_stats);
			log_raw_radio_status(windows);
			LOG_WRN("rx wait timeout window=%u status=%u", windows,
				(unsigned int)sid_pal_radio_get_status());
			(void)sid_pal_radio_standby();
			continue;
		}

		log_rssi_stats(windows, &rssi_stats);
		LOG_INF("rx event %s(%d) window=%u len=%u rssi_avg=%d rssi_sync=%d",
			event_name(event), event, windows, rx_packet.payload_len,
			rx_packet.fsk_rx_packet_status.rssi_avg,
			rx_packet.fsk_rx_packet_status.rssi_sync);

		if (event == SID_PAL_RADIO_EVENT_RX_DONE && rx_packet.payload_len > 0) {
			LOG_HEXDUMP_INF(rx_packet.rcv_payload, rx_packet.payload_len, "rx payload");
		}

		(void)sid_pal_radio_standby();
	}
}

static int configure_tx_packet(uint8_t *tx_buffer, uint8_t psdu_len)
{
	sid_pal_radio_fsk_pkt_cfg_t cfg = {
		.phy_hdr = &fsk_phy_hdr,
		.packet_params = &fsk_packet_params,
		.sync_word = fsk_sync_word,
		.payload = tx_buffer,
	};

	fsk_packet_params.preamble_length = CONFIG_FSK_PROBE_PREAMBLE_LEN_BYTES;
	fsk_packet_params.payload_length = psdu_len;

	if (radio_call("sid_pal_radio_prepare_fsk_for_tx", sid_pal_radio_prepare_fsk_for_tx(&cfg))) {
		return -EIO;
	}

	apply_sync_override();

	if (radio_call("sid_pal_radio_set_fsk_packet_params",
		       sid_pal_radio_set_fsk_packet_params(&fsk_packet_params))) {
		return -EIO;
	}

	if (radio_call("sid_pal_radio_set_fsk_sync_word",
		       sid_pal_radio_set_fsk_sync_word(fsk_sync_word,
						       fsk_packet_params.sync_word_length))) {
		return -EIO;
	}

	if (radio_call("sid_pal_radio_set_tx_payload",
		       sid_pal_radio_set_tx_payload(tx_buffer, fsk_packet_params.payload_length))) {
		return -EIO;
	}

	return 0;
}

static int run_tx(void)
{
	uint8_t tx_buffer[FSK_PROBE_TX_BUFFER_LEN];
	uint32_t seq = 0;

	if (configure_common_radio()) {
		return -EIO;
	}

#if defined(CONFIG_FSK_PROBE_TX_CONTINUOUS_PREAMBLE)
	if (radio_call("sid_pal_radio_set_tx_continuous_preamble",
		       sid_pal_radio_set_tx_continuous_preamble(CONFIG_FSK_PROBE_FREQ_HZ,
								CONFIG_FSK_PROBE_TX_POWER_DBM))) {
		return -EIO;
	}

	LOG_INF("tx continuous preamble active freq=%u power=%d",
		(unsigned int)CONFIG_FSK_PROBE_FREQ_HZ, CONFIG_FSK_PROBE_TX_POWER_DBM);

	while (true) {
		k_sleep(K_SECONDS(60));
	}
#endif

#if defined(CONFIG_FSK_PROBE_TX_CONTINUOUS_WAVE)
	if (radio_call("sid_pal_radio_set_tx_continuous_wave",
		       sid_pal_radio_set_tx_continuous_wave(CONFIG_FSK_PROBE_FREQ_HZ,
							   CONFIG_FSK_PROBE_TX_POWER_DBM))) {
		return -EIO;
	}

	LOG_INF("tx continuous wave active freq=%u power=%d",
		(unsigned int)CONFIG_FSK_PROBE_FREQ_HZ, CONFIG_FSK_PROBE_TX_POWER_DBM);

	while (true) {
		k_sleep(K_SECONDS(60));
	}
#endif

	while (true) {
		sid_pal_radio_events_t event;
		int len;

		memset(tx_buffer, 0, sizeof(tx_buffer));
		len = snprintk(tx_buffer, FSK_PROBE_PSDU_MAX, "fsk-probe %08u", seq++);
		if (len != FSK_PROBE_PSDU_LEN) {
			LOG_ERR("payload format failed len=%d", len);
			k_sleep(K_SECONDS(1));
			continue;
		}

		if (configure_tx_packet(tx_buffer, (uint8_t)len) ||
		    radio_call("sid_pal_radio_start_tx", sid_pal_radio_start_tx(FSK_PROBE_TX_TIMEOUT_US))) {
			(void)sid_pal_radio_standby();
			k_sleep(K_SECONDS(1));
			continue;
		}

		if (wait_radio_event(&event, 6000, NULL) != 0) {
			LOG_WRN("tx wait timeout seq=%u status=%u", seq - 1,
				(unsigned int)sid_pal_radio_get_status());
		} else {
			LOG_INF("tx event %s(%d) seq=%u", event_name(event), event, seq - 1);
		}

		(void)sid_pal_radio_standby();
		k_sleep(K_MSEC(CONFIG_FSK_PROBE_INTERVAL_MS));
	}
}

int main(void)
{
	int err;

	for (int i = 0; i < 10; i++) {
		printk("fsk_probe boot wait %d\n", i);
		k_sleep(K_SECONDS(1));
	}

	LOG_INF("FSK probe boot role=%s freq=%u power=%d",
#if defined(CONFIG_FSK_PROBE_ROLE_TX)
		"tx",
#else
		"rx",
#endif
		(unsigned int)CONFIG_FSK_PROBE_FREQ_HZ, CONFIG_FSK_PROBE_TX_POWER_DBM);

#if defined(CONFIG_FSK_PROBE_ROLE_TX)
	err = run_tx();
#else
	err = run_rx();
#endif

	while (true) {
		LOG_ERR("FSK probe stopped err=%d", err);
		printk("fsk_probe stopped err=%d\n", err);
		k_sleep(K_SECONDS(5));
	}
}
