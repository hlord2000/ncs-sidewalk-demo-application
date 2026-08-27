/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef APP_SENSOR_H
#define APP_SENSOR_H

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/sys/util.h>

/**
 * @brief Sensor capability bits, one per sensor class app_sensor_sample_get() can fill.
 *
 * Derived once at app_sensor_init() from device_is_ready() on the aliased devicetree
 * nodes, so the mask reflects whatever shield or SoC peripheral is actually present
 * on this build rather than a hardcoded per-board table.
 */
enum app_sensor_cap {
	APP_SENSOR_CAP_TEMPERATURE = BIT(0),
	APP_SENSOR_CAP_HUMIDITY = BIT(1),
	APP_SENSOR_CAP_ACCEL = BIT(2),
	APP_SENSOR_CAP_BATTERY = BIT(3),
};

struct app_sensor_sample {
	bool temperature_valid;
	int32_t temperature_millicelsius;
	bool humidity_valid;
	uint32_t humidity_millipercent;
	bool accel_valid;
	int32_t accel_milli_ms2[3];
	bool accel_wake;
	bool pmic_valid;
	bool vbus_present;
	int32_t battery_millivolts;
	int32_t battery_current_microamps;
	int16_t battery_level_percent;
	int charger_status;
	int charger_error;
	int vbus_status;
};

typedef void (*app_sensor_wake_handler_t)(void);

/**
 * @brief Initialize board sensors and motion wake notification.
 *
 * @param wake_handler callback invoked from the sensor trigger thread when ADXL367 activity fires.
 * @return 0 if all present sensors initialized, otherwise a negative errno code.
 */
int app_sensor_init(app_sensor_wake_handler_t wake_handler);

/**
 * @brief Read available board sensors.
 *
 * @param[out] sample sensor sample. Validity flags indicate which fields were updated.
 * @return 0 if at least one sensor was read successfully, otherwise a negative errno code.
 */
int app_sensor_sample_get(struct app_sensor_sample *sample);

/**
 * @brief Get current temperature from device sensor.
 *
 * @param[out] temp - sensor temperature in Celsius degrees.
 * @return 0 if successful, negative errno code if failure.
 */
int app_sensor_temperature_get(int16_t *temp);

/**
 * @brief Get the sensor capability bitmask detected for this board.
 *
 * Valid once app_sensor_init() has run; before that it reads as 0.
 *
 * @return Bitwise OR of APP_SENSOR_CAP_* for every sensor found ready.
 */
uint8_t app_sensor_caps_get(void);

#endif /* APP_SENSOR_H */
