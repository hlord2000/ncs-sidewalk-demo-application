/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <prop_radio.h>

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/shell/shell.h>

static const char *radio_state_to_str(uint8_t state)
{
	switch (state) {
	case SID_PAL_RADIO_STANDBY:
		return "standby";
	case SID_PAL_RADIO_SLEEP:
		return "sleep";
	case SID_PAL_RADIO_RX:
		return "rx";
	case SID_PAL_RADIO_TX:
		return "tx";
	case SID_PAL_RADIO_CAD:
		return "cad";
	case SID_PAL_RADIO_STANDBY_XOSC:
		return "standby_xosc";
	case SID_PAL_RADIO_RX_DC:
		return "rx_duty_cycle";
	case SID_PAL_RADIO_BUSY:
		return "busy";
	default:
		return "unknown";
	}
}

static long bandwidth_to_khz(uint8_t bandwidth)
{
	switch (bandwidth) {
	case SID_PAL_RADIO_LORA_BW_125KHZ:
		return 125;
	case SID_PAL_RADIO_LORA_BW_250KHZ:
		return 250;
	case SID_PAL_RADIO_LORA_BW_500KHZ:
		return 500;
	default:
		return -1;
	}
}

static int parse_bandwidth_khz(long bw_khz, uint8_t *bandwidth)
{
	switch (bw_khz) {
	case 125:
		*bandwidth = SID_PAL_RADIO_LORA_BW_125KHZ;
		return 0;
	case 250:
		*bandwidth = SID_PAL_RADIO_LORA_BW_250KHZ;
		return 0;
	case 500:
		*bandwidth = SID_PAL_RADIO_LORA_BW_500KHZ;
		return 0;
	default:
		return -EINVAL;
	}
}

static int run_request(const struct shell *sh, struct prop_radio_request *req, const char *action)
{
	int err = prop_radio_request_submit(req);

	if (err != 0) {
		shell_error(sh, "%s failed: result=%d sid=%d radio=%d", action, err, req->sid_error,
			    req->radio_error);
		return err;
	}

	return 0;
}

static int cmd_claim(const struct shell *sh, size_t argc, char **argv)
{
	struct prop_radio_request req = {
		.op = PROP_RADIO_REQ_CLAIM,
	};

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (run_request(sh, &req, "claim") != 0) {
		return -EIO;
	}

	shell_print(sh, "proprietary LoRa claimed");
	return 0;
}

static int cmd_release(const struct shell *sh, size_t argc, char **argv)
{
	struct prop_radio_request req = {
		.op = PROP_RADIO_REQ_RELEASE,
	};

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (run_request(sh, &req, "release") != 0) {
		return -EIO;
	}

	shell_print(sh, "proprietary LoRa released");
	return 0;
}

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	struct prop_radio_status_snapshot status = { 0 };

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	prop_radio_get_status_snapshot(&status);
	shell_print(sh,
		    "ready=%d claimed=%d rx_enabled=%d rx_running=%d sidewalk_subghz_paused=%d state=%s freq=%u sf=%u bw=%ld power=%d sync=0x%04x",
		    status.module_ready, status.claimed, status.rx_enabled, status.rx_running,
		    status.sidewalk_subghz_paused, radio_state_to_str(status.radio_state),
		    status.config.frequency_hz, status.config.spreading_factor,
		    bandwidth_to_khz(status.config.bandwidth), status.config.tx_power_dbm,
		    status.config.sync_word);
	return 0;
}

static int cmd_config(const struct shell *sh, size_t argc, char **argv)
{
	struct prop_radio_request req = {
		.op = PROP_RADIO_REQ_CONFIG,
	};
	unsigned long freq_hz;
	long sf;
	long bw_khz;
	long power_dbm;
	uint8_t bandwidth;
	char *endptr = NULL;

	freq_hz = strtoul(argv[1], &endptr, 0);
	if ((endptr == argv[1]) || (*endptr != '\0')) {
		return -EINVAL;
	}

	sf = strtol(argv[2], &endptr, 0);
	if ((endptr == argv[2]) || (*endptr != '\0') ||
	    (sf < SID_PAL_RADIO_LORA_SF5) || (sf > SID_PAL_RADIO_LORA_SF12)) {
		return -EINVAL;
	}

	bw_khz = strtol(argv[3], &endptr, 0);
	if ((endptr == argv[3]) || (*endptr != '\0') ||
	    parse_bandwidth_khz(bw_khz, &bandwidth) != 0) {
		shell_error(sh, "bw must be 125, 250, or 500");
		return -EINVAL;
	}

	power_dbm = strtol(argv[4], &endptr, 0);
	if ((endptr == argv[4]) || (*endptr != '\0')) {
		return -EINVAL;
	}

	req.params.config = (struct prop_radio_config){
		.frequency_hz = (uint32_t)freq_hz,
		.tx_power_dbm = (int8_t)power_dbm,
		.spreading_factor = (uint8_t)sf,
		.bandwidth = bandwidth,
		.coding_rate = SID_PAL_RADIO_LORA_CODING_RATE_4_5,
		.preamble_length = 8,
		.sync_word = SID_PAL_RADIO_LORA_PRIVATE_NETWORK_SYNC_WORD,
		.invert_iq = false,
		.crc_on = true,
	};

	if (run_request(sh, &req, "config") != 0) {
		return -EIO;
	}

	shell_print(sh, "updated freq=%u sf=%u bw=%ld power=%ld",
		    req.params.config.frequency_hz, req.params.config.spreading_factor, bw_khz,
		    power_dbm);
	return 0;
}

static int cmd_tx(const struct shell *sh, size_t argc, char **argv)
{
	struct prop_radio_request req = {
		.op = PROP_RADIO_REQ_TX,
	};
	size_t len = strlen(argv[1]);

	ARG_UNUSED(argc);

	if (len == 0U || len > PROP_RADIO_MAX_PAYLOAD_LEN) {
		shell_error(sh, "payload must be 1..%u bytes", PROP_RADIO_MAX_PAYLOAD_LEN);
		return -EINVAL;
	}

	req.params.tx.len = len;
	memcpy(req.params.tx.payload, argv[1], len);

	if (run_request(sh, &req, "tx") != 0) {
		return -EIO;
	}

	shell_print(sh, "tx started");
	return 0;
}

static int cmd_rx_start(const struct shell *sh, size_t argc, char **argv)
{
	struct prop_radio_request req = {
		.op = PROP_RADIO_REQ_RX_START,
	};

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (run_request(sh, &req, "rx start") != 0) {
		return -EIO;
	}

	shell_print(sh, "continuous rx started");
	return 0;
}

static int cmd_rx_stop(const struct shell *sh, size_t argc, char **argv)
{
	struct prop_radio_request req = {
		.op = PROP_RADIO_REQ_RX_STOP,
	};

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (run_request(sh, &req, "rx stop") != 0) {
		return -EIO;
	}

	shell_print(sh, "radio in standby");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	sid_radio_pal_rx_cmds,
	SHELL_CMD(start, NULL, "Start continuous RX and pause the Sidewalk sub-GHz links",
		  cmd_rx_start),
	SHELL_CMD(stop, NULL, "Stop proprietary RX and enter standby", cmd_rx_stop),
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sid_radio_pal_cmds,
	SHELL_CMD(claim, NULL, "Pause Sidewalk sub-GHz links and claim the SX1262 for proprietary LoRa",
		  cmd_claim),
	SHELL_CMD(config, NULL, "config <freq_hz> <sf> <bw_khz> <power_dbm>", cmd_config),
	SHELL_CMD(release, NULL, "Release proprietary LoRa and resume the Sidewalk sub-GHz links",
		  cmd_release),
	SHELL_CMD(rx, &sid_radio_pal_rx_cmds, "RX controls", NULL),
	SHELL_CMD(status, NULL, "Print proprietary radio status", cmd_status),
	SHELL_CMD(tx, NULL, "tx <ascii-payload>", cmd_tx),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(sid_radio_pal, &sid_radio_pal_cmds,
		   "Raw LoRa controls sharing the Sidewalk SX1262 radio", NULL);
