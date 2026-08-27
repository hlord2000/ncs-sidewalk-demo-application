/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <sensor_monitoring/app_sensor.h>
#include <errno.h>
#include <string.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/npm1300_charger.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#if IS_ENABLED(CONFIG_SID_END_DEVICE_FUEL_GAUGE)
#include <nrf_fuel_gauge.h>
#endif

LOG_MODULE_REGISTER(app_sensor, CONFIG_SIDEWALK_LOG_LEVEL);

#define SHT4X_NODE DT_ALIAS(humidity0)
#define ADXL367_NODE DT_ALIAS(accel0)
#define NPM1300_CHARGER_NODE DT_NODELABEL(npm1300_charger)

#define HAS_SHT4X DT_NODE_HAS_STATUS(SHT4X_NODE, okay)
#define HAS_ADXL367 DT_NODE_HAS_STATUS(ADXL367_NODE, okay)
#define HAS_NPM1300_CHARGER DT_NODE_HAS_STATUS(NPM1300_CHARGER_NODE, okay)

#if HAS_SHT4X
static const struct device *const sht4x_dev = DEVICE_DT_GET(SHT4X_NODE);
#endif

#if HAS_ADXL367
static const struct device *const accel_dev = DEVICE_DT_GET(ADXL367_NODE);
#endif

#if HAS_NPM1300_CHARGER
static const struct device *const pmic_charger_dev = DEVICE_DT_GET(NPM1300_CHARGER_NODE);
#endif

#if IS_ENABLED(CONFIG_SID_END_DEVICE_FUEL_GAUGE) && HAS_NPM1300_CHARGER

/* nPM1300 CHARGER.BCHGCHARGESTATUS register bitmasks */
#define NPM1300_CHG_STATUS_COMPLETE_MASK BIT(1)
#define NPM1300_CHG_STATUS_TRICKLE_MASK BIT(2)
#define NPM1300_CHG_STATUS_CC_MASK BIT(3)
#define NPM1300_CHG_STATUS_CV_MASK BIT(4)

static const struct battery_model battery_model = {
#include "battery_model.inc"
};

static int32_t app_sensor_last_chg_status = -1;
static int64_t app_sensor_fuel_gauge_ref_time;
static bool app_sensor_fuel_gauge_initialized;

static int app_sensor_fuel_gauge_charge_state_update(int32_t chg_status)
{
	union nrf_fuel_gauge_ext_state_info_data state_info;

	if (chg_status < 0) {
		return 0;
	}

	if (chg_status & NPM1300_CHG_STATUS_COMPLETE_MASK) {
		state_info.charge_state = NRF_FUEL_GAUGE_CHARGE_STATE_COMPLETE;
	} else if (chg_status & NPM1300_CHG_STATUS_TRICKLE_MASK) {
		state_info.charge_state = NRF_FUEL_GAUGE_CHARGE_STATE_TRICKLE;
	} else if (chg_status & NPM1300_CHG_STATUS_CC_MASK) {
		state_info.charge_state = NRF_FUEL_GAUGE_CHARGE_STATE_CC;
	} else if (chg_status & NPM1300_CHG_STATUS_CV_MASK) {
		state_info.charge_state = NRF_FUEL_GAUGE_CHARGE_STATE_CV;
	} else {
		state_info.charge_state = NRF_FUEL_GAUGE_CHARGE_STATE_IDLE;
	}

	return nrf_fuel_gauge_ext_state_update(
		NRF_FUEL_GAUGE_EXT_STATE_INFO_CHARGE_STATE_CHANGE,
		&state_info);
}

static int app_sensor_fuel_gauge_init(float voltage, float current, float temp, int32_t chg_status,
				     bool vbus_connected)
{
	struct sensor_value value = { 0, 0 };
	struct nrf_fuel_gauge_init_parameters parameters = {
		.v0 = voltage,
		.i0 = current,
		.t0 = temp,
		.model = &battery_model,
		.opt_params = NULL,
		.state = NULL,
	};
	float max_charge_current;
	float term_charge_current;
	int ret;

	ret = nrf_fuel_gauge_init(&parameters, NULL);
	if (ret < 0) {
		LOG_WRN("nRF Fuel Gauge init failed %d", ret);
		return ret;
	}

	ret = app_sensor_fuel_gauge_charge_state_update(chg_status);
	if (ret < 0) {
		LOG_WRN("Fuel gauge charge-state update failed %d", ret);
		return ret;
	}

	ret = nrf_fuel_gauge_ext_state_update(vbus_connected ?
					      NRF_FUEL_GAUGE_EXT_STATE_INFO_VBUS_CONNECTED :
					      NRF_FUEL_GAUGE_EXT_STATE_INFO_VBUS_DISCONNECTED,
					      NULL);
	if (ret < 0) {
		LOG_WRN("Fuel gauge VBUS state update failed %d", ret);
		return ret;
	}

	ret = sensor_channel_get(pmic_charger_dev, SENSOR_CHAN_GAUGE_DESIRED_CHARGING_CURRENT, &value);
	if (!ret) {
		max_charge_current = (float)value.val1 + ((float)value.val2 / 1000000.0f);
		term_charge_current = max_charge_current / 10.0f;

		ret = nrf_fuel_gauge_ext_state_update(
			NRF_FUEL_GAUGE_EXT_STATE_INFO_CHARGE_CURRENT_LIMIT,
			&(union nrf_fuel_gauge_ext_state_info_data){
				.charge_current_limit = max_charge_current
			});
		if (ret < 0) {
			LOG_WRN("Fuel gauge charge current limit update failed %d", ret);
			return ret;
		}

		ret = nrf_fuel_gauge_ext_state_update(
			NRF_FUEL_GAUGE_EXT_STATE_INFO_TERM_CURRENT,
			&(union nrf_fuel_gauge_ext_state_info_data){
				.charge_term_current = term_charge_current
			});
		if (ret < 0) {
			LOG_WRN("Fuel gauge term current update failed %d", ret);
			return ret;
		}
	}

	app_sensor_last_chg_status = chg_status;
	app_sensor_fuel_gauge_ref_time = k_uptime_get();
	app_sensor_fuel_gauge_initialized = true;

	return 0;
}

static int app_sensor_fuel_gauge_update(float voltage, float current, float temp, int32_t chg_status,
				       bool vbus_connected, int16_t *soc_percent)
{
	float battery_soc;
	float t_delta;
	int ret;

	if (!soc_percent) {
		return -EINVAL;
	}

	ret = nrf_fuel_gauge_ext_state_update(vbus_connected ?
					      NRF_FUEL_GAUGE_EXT_STATE_INFO_VBUS_CONNECTED :
					      NRF_FUEL_GAUGE_EXT_STATE_INFO_VBUS_DISCONNECTED,
					      NULL);
	if (ret < 0) {
		return ret;
	}

	if ((chg_status >= 0) && (chg_status != app_sensor_last_chg_status)) {
		ret = app_sensor_fuel_gauge_charge_state_update(chg_status);
		if (ret < 0) {
			return ret;
		}
		app_sensor_last_chg_status = chg_status;
	}

	t_delta = (float)k_uptime_delta(&app_sensor_fuel_gauge_ref_time) / 1000.0f;
	battery_soc = nrf_fuel_gauge_process(voltage, current, temp, t_delta, NULL);

	if (battery_soc < 0.0f) {
		*soc_percent = 0;
	} else if (battery_soc > 100.0f) {
		*soc_percent = 100;
	} else {
		*soc_percent = (int16_t)(battery_soc + 0.5f);
	}

	return 0;
}

#endif

static int32_t app_sensor_value_to_milli(const struct sensor_value *value)
{
	return (value->val1 * 1000) + (value->val2 / 1000);
}

static int32_t app_sensor_value_to_micro(const struct sensor_value *value)
{
	return (value->val1 * 1000000) + value->val2;
}

#ifdef CONFIG_BOARD_THINGY53_NRF5340_CPUAPP
#define HAS_FALLBACK_TEMPERATURE 1
#define TEMP_CHANNEL SENSOR_CHAN_AMBIENT_TEMP
static const struct device *temp_dev = DEVICE_DT_GET_ONE(bosch_bme680);
#elif defined(CONFIG_TEMP_NRF5)
#define HAS_FALLBACK_TEMPERATURE 1
#define TEMP_CHANNEL SENSOR_CHAN_DIE_TEMP
static const struct device *temp_dev = DEVICE_DT_GET_ONE(nordic_nrf_temp);
#else
#define HAS_FALLBACK_TEMPERATURE 0
#endif

static int fallback_temperature_millicelsius_get(int32_t *temp_mc)
{
	if (!temp_mc) {
		return -EINVAL;
	}

#if HAS_FALLBACK_TEMPERATURE
	if (!device_is_ready(temp_dev)) {
		return -EIO;
	}

	int err = sensor_sample_fetch(temp_dev);

	if (err) {
		return -EIO;
	}

	struct sensor_value sensor = { 0, 0 };

	err = sensor_channel_get(temp_dev, TEMP_CHANNEL, &sensor);
	if (err) {
		return -EFAULT;
	}

	/* The die sensor resolves finer than a whole degree, and val2 carries that
	 * fraction in micro-degrees. Keep it: dropping it turns the demo's
	 * temperature chart into a 1 degree staircase on boards that have no
	 * external sensor. Zephyr signs val1 and val2 alike, so this stays correct
	 * below zero.
	 */
	*temp_mc = (int32_t)sensor.val1 * 1000 + sensor.val2 / 1000;
	return 0;
#else
	return -ENODEV;
#endif
}

static int fallback_temperature_get(int16_t *temp)
{
	if (!temp) {
		return -EINVAL;
	}

	int32_t temp_mc;
	int err = fallback_temperature_millicelsius_get(&temp_mc);

	if (err) {
		return err;
	}

	*temp = (int16_t)(temp_mc / 1000);
	return 0;
}

static atomic_t accel_wake_pending;
static app_sensor_wake_handler_t sensor_wake_handler;
static uint8_t app_sensor_caps;

#if HAS_ADXL367 && IS_ENABLED(CONFIG_ADXL367_TRIGGER)
static void accel_trigger_handler(const struct device *dev, const struct sensor_trigger *trigger)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(trigger);

	atomic_set(&accel_wake_pending, 1);

	if (sensor_wake_handler) {
		sensor_wake_handler();
	}
}
#endif

int app_sensor_init(app_sensor_wake_handler_t wake_handler)
{
	sensor_wake_handler = wake_handler;

	/* Capabilities are derived from what device_is_ready() actually finds on
	 * this build, not from a per-board table, so a different shield changes
	 * the mask without touching this file.
	 */
	app_sensor_caps = 0;
#if HAS_SHT4X
	if (device_is_ready(sht4x_dev)) {
		app_sensor_caps |= APP_SENSOR_CAP_TEMPERATURE | APP_SENSOR_CAP_HUMIDITY;
	}
#endif
#if HAS_ADXL367
	if (device_is_ready(accel_dev)) {
		app_sensor_caps |= APP_SENSOR_CAP_ACCEL;
	}
#endif
#if HAS_FALLBACK_TEMPERATURE
	if (!(app_sensor_caps & APP_SENSOR_CAP_TEMPERATURE) && device_is_ready(temp_dev)) {
		app_sensor_caps |= APP_SENSOR_CAP_TEMPERATURE;
	}
#endif
#if HAS_NPM1300_CHARGER
	if (device_is_ready(pmic_charger_dev)) {
		app_sensor_caps |= APP_SENSOR_CAP_BATTERY;
	}
#endif

#if HAS_ADXL367 && IS_ENABLED(CONFIG_ADXL367_TRIGGER)
	if (!device_is_ready(accel_dev)) {
		LOG_WRN("ADXL367 is not ready");
		return -ENODEV;
	}

	const struct sensor_trigger trigger = {
		.type = SENSOR_TRIG_THRESHOLD,
		.chan = SENSOR_CHAN_ACCEL_XYZ,
	};

	int err = sensor_trigger_set(accel_dev, &trigger, accel_trigger_handler);

	if (err) {
		LOG_WRN("ADXL367 trigger setup failed %d", err);
		return err;
	}

	LOG_INF("ADXL367 activity wake trigger armed");
#endif

	return 0;
}

int app_sensor_sample_get(struct app_sensor_sample *sample)
{
	int err;
	int successes = 0;

	if (!sample) {
		return -EINVAL;
	}

	memset(sample, 0, sizeof(*sample));
	sample->battery_level_percent = -1;

#if HAS_SHT4X
	if (device_is_ready(sht4x_dev)) {
		struct sensor_value temperature = { 0, 0 };
		struct sensor_value humidity = { 0, 0 };

		err = sensor_sample_fetch(sht4x_dev);
		if (!err) {
			if (!sensor_channel_get(sht4x_dev, SENSOR_CHAN_AMBIENT_TEMP,
						&temperature)) {
				sample->temperature_millicelsius =
					app_sensor_value_to_milli(&temperature);
				sample->temperature_valid = true;
				successes++;
			}

			if (!sensor_channel_get(sht4x_dev, SENSOR_CHAN_HUMIDITY, &humidity)) {
				sample->humidity_millipercent =
					(uint32_t)app_sensor_value_to_milli(&humidity);
				sample->humidity_valid = true;
				successes++;
			}
		} else {
			LOG_WRN("SHT4x fetch failed %d", err);
		}
	} else {
		LOG_WRN("SHT4x is not ready");
	}
#endif

#if HAS_FALLBACK_TEMPERATURE
	/* Boards with no external temperature sensor (or a failed SHT4x read)
	 * still have the SoC die temperature. Feed it into the same sample so
	 * app_sensor_sample_get() does not return -ENODEV on a board that can
	 * report at least this much.
	 */
	if (!sample->temperature_valid) {
		int32_t fallback_temp_mc;

		err = fallback_temperature_millicelsius_get(&fallback_temp_mc);
		if (!err) {
			sample->temperature_millicelsius = fallback_temp_mc;
			sample->temperature_valid = true;
			successes++;
		}
	}
#endif

#if HAS_ADXL367
	if (device_is_ready(accel_dev)) {
		struct sensor_value accel[3] = { { 0, 0 }, { 0, 0 }, { 0, 0 } };

		err = sensor_sample_fetch(accel_dev);
		if (!err && !sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, accel)) {
			for (size_t i = 0; i < ARRAY_SIZE(sample->accel_milli_ms2); i++) {
				sample->accel_milli_ms2[i] =
					app_sensor_value_to_milli(&accel[i]);
			}
			sample->accel_valid = true;
			successes++;
		} else {
			LOG_WRN("ADXL367 read failed %d", err);
		}
	} else {
		LOG_WRN("ADXL367 is not ready");
	}
#endif

#if HAS_NPM1300_CHARGER
#if IS_ENABLED(CONFIG_SID_END_DEVICE_FUEL_GAUGE)
	float battery_voltage_v = 0.0f;
	float battery_current_a = 0.0f;
	float battery_temp_c = 0.0f;
	int32_t chg_status = -1;
	bool gauge_temp_ready = false;
	bool gauge_voltage_ready = false;
	bool gauge_current_ready = false;
	bool vbus_connected = false;
	int gauge_err;
#endif

	if (device_is_ready(pmic_charger_dev)) {
		struct sensor_value value = { 0, 0 };

		err = sensor_sample_fetch(pmic_charger_dev);
		if (!err) {
			sample->pmic_valid = true;
			successes++;

#if IS_ENABLED(CONFIG_SID_END_DEVICE_FUEL_GAUGE)
			gauge_temp_ready = false;
			gauge_voltage_ready = false;
			gauge_current_ready = false;
			vbus_connected = false;
#endif

			if (!sensor_channel_get(pmic_charger_dev, SENSOR_CHAN_GAUGE_VOLTAGE,
						&value)) {
				sample->battery_millivolts = app_sensor_value_to_milli(&value);
#if IS_ENABLED(CONFIG_SID_END_DEVICE_FUEL_GAUGE)
				battery_voltage_v = sample->battery_millivolts / 1000.0f;
				gauge_voltage_ready = true;
#endif
			}

			if (!sensor_channel_get(pmic_charger_dev, SENSOR_CHAN_GAUGE_AVG_CURRENT,
						&value)) {
				sample->battery_current_microamps =
					app_sensor_value_to_micro(&value);
#if IS_ENABLED(CONFIG_SID_END_DEVICE_FUEL_GAUGE)
				battery_current_a = sample->battery_current_microamps / 1000000.0f;
				gauge_current_ready = true;
#endif
			}

			if (!sensor_channel_get(pmic_charger_dev,
						SENSOR_CHAN_NPM1300_CHARGER_STATUS, &value)) {
				sample->charger_status = value.val1;
#if IS_ENABLED(CONFIG_SID_END_DEVICE_FUEL_GAUGE)
				chg_status = value.val1;
#endif
			}

			if (!sensor_channel_get(pmic_charger_dev,
						SENSOR_CHAN_NPM1300_CHARGER_ERROR, &value)) {
				sample->charger_error = value.val1;
			}

			if (!sensor_channel_get(pmic_charger_dev,
						SENSOR_CHAN_NPM1300_CHARGER_VBUS_STATUS, &value)) {
				sample->vbus_status = value.val1;
			}

			if (!sensor_attr_get(pmic_charger_dev,
					     SENSOR_CHAN_NPM1300_CHARGER_VBUS_STATUS,
					     SENSOR_ATTR_NPM1300_CHARGER_VBUS_PRESENT,
					     &value)) {
				sample->vbus_present = value.val1 != 0;
#if IS_ENABLED(CONFIG_SID_END_DEVICE_FUEL_GAUGE)
				vbus_connected = sample->vbus_present;
#endif
			}

#if IS_ENABLED(CONFIG_SID_END_DEVICE_FUEL_GAUGE)
			if (!sensor_channel_get(pmic_charger_dev, SENSOR_CHAN_GAUGE_TEMP, &value)) {
				battery_temp_c = app_sensor_value_to_milli(&value) / 1000.0f;
				gauge_temp_ready = true;
			} else if (!sensor_channel_get(pmic_charger_dev, SENSOR_CHAN_DIE_TEMP,
						      &value)) {
				battery_temp_c = app_sensor_value_to_milli(&value) / 1000.0f;
				gauge_temp_ready = true;
			}

			if (gauge_voltage_ready && gauge_current_ready && gauge_temp_ready) {
				if (!app_sensor_fuel_gauge_initialized) {
					gauge_err = app_sensor_fuel_gauge_init(
						battery_voltage_v, battery_current_a,
						battery_temp_c, chg_status,
						vbus_connected);
					if (gauge_err) {
						LOG_WRN("Fuel gauge init failed %d", gauge_err);
						app_sensor_fuel_gauge_initialized = false;
					}
				}

				if (app_sensor_fuel_gauge_initialized) {
					int16_t soc_percent;

					gauge_err = app_sensor_fuel_gauge_update(
						battery_voltage_v, battery_current_a,
						battery_temp_c, chg_status, vbus_connected,
						&soc_percent);
					if (!gauge_err) {
						sample->battery_level_percent = soc_percent;
					} else {
						LOG_WRN("Fuel gauge update failed %d", gauge_err);
					}
				}
			}
#endif
		} else {
			LOG_WRN("nPM1300 charger fetch failed %d", err);
		}
	} else {
		LOG_WRN("nPM1300 charger is not ready");
	}
#endif

	sample->accel_wake = atomic_cas(&accel_wake_pending, 1, 0);

	return successes > 0 ? 0 : -ENODEV;
}

int app_sensor_temperature_get(int16_t *temp)
{
	struct app_sensor_sample sample;

	if (!temp) {
		return -EINVAL;
	};

	if (!app_sensor_sample_get(&sample) && sample.temperature_valid) {
		*temp = (int16_t)(sample.temperature_millicelsius / 1000);
		return 0;
	}

	return fallback_temperature_get(temp);
}

uint8_t app_sensor_caps_get(void)
{
	return app_sensor_caps;
}
