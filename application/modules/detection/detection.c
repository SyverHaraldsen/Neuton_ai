/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>
#include <nrf_edgeai/nrf_edgeai.h>
#include "nrf_edgeai_generated/nrf_edgeai_user_model.h"
#include <math.h>

#include "detection.h"
#include "../sampling/sampling.h"

LOG_MODULE_REGISTER(app_detection, CONFIG_APP_DETECTION_LOG_LEVEL);

/* Zbus channel for publishing detection results */
ZBUS_CHAN_DEFINE(detection_result_chan,
		 struct detection_result,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(0));

/* EdgeAI model handle */
static nrf_edgeai_t *p_model = NULL;

/* Track last published class to avoid spam */
static uint16_t last_published_class = UINT16_MAX;  /* Start with invalid value */

/**
 * @brief Calculate acceleration magnitude from 3-axis accelerometer data
 * @param accel_x X-axis acceleration (m/s^2)
 * @param accel_y Y-axis acceleration (m/s^2)
 * @param accel_z Z-axis acceleration (m/s^2)
 * @return Acceleration magnitude in milli-g
 */
static float calculate_accel_magnitude(double accel_x, double accel_y, double accel_z)
{
	/* Calculate magnitude: sqrt(x^2 + y^2 + z^2) */
	double magnitude = sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);

	/* Convert from m/s^2 to milli-g (1g = 9.80665 m/s^2) */
	float magnitude_mg = (float)(magnitude / 9.80665 * 1000.0);

	return magnitude_mg;
}

/**
 * @brief Zbus listener callback for IMU data
 * This function is called every time new IMU data is published on imu_data_chan
 */
static void imu_data_listener_cb(const struct zbus_channel *chan)
{
	const struct imu_sample *sample = zbus_chan_const_msg(chan);
	nrf_edgeai_err_t res;

	/* Calculate acceleration magnitude (model expects single feature) */
	float accel_magnitude = calculate_accel_magnitude(
		sample->accel_x,
		sample->accel_y,
		sample->accel_z
	);

	/* Feed the sample to the EdgeAI model */
	res = nrf_edgeai_feed_inputs(p_model, &accel_magnitude, 1);

	if (res == NRF_EDGEAI_ERR_SUCCESS) {
		/* Window is full - run inference */
		res = nrf_edgeai_run_inference(p_model);

		if (res == NRF_EDGEAI_ERR_SUCCESS) {
			/* Extract results from model output */
			uint16_t predicted_class =
				p_model->decoded_output.classif.predicted_class;
			const float *p_probabilities =
				p_model->decoded_output.classif.probabilities.p_f32;
			float confidence = p_probabilities[predicted_class];

			/* Only publish to Zbus if class has changed (avoid spam) */
			if (predicted_class != last_published_class) {
				/* Prepare detection result */
				struct detection_result result = {
					.predicted_class = predicted_class,
					.confidence = confidence,
					.timestamp = k_uptime_get_32(),
				};

				/* Publish result to Zbus */
				int ret = zbus_chan_pub(&detection_result_chan, &result, K_NO_WAIT);
				if (ret) {
					LOG_WRN("Failed to publish detection result: %d", ret);
				} else {
					/* Update last published class */
					last_published_class = predicted_class;
				}
			}
		} else {
			LOG_ERR("Inference failed: %d", res);
		}
	} else if (res != NRF_EDGEAI_ERR_INPROGRESS) {
		/* Log errors, but not when window is still filling up */
		LOG_ERR("Failed to feed input: %d", res);
	}
}

/* Zbus listener for IMU data channel */
ZBUS_LISTENER_DEFINE(imu_data_listener, imu_data_listener_cb);

/* Subscribe the listener to the IMU data channel */
ZBUS_CHAN_ADD_OBS(imu_data_chan, imu_data_listener, 0);

int detection_init(void)
{
	nrf_edgeai_err_t res;

	LOG_INF("Initializing detection module");

	/* Get the user-generated model */
	p_model = nrf_edgeai_user_model();
	if (!p_model) {
		LOG_ERR("Failed to get EdgeAI model");
		return -ENOMEM;
	}

	/* Initialize the EdgeAI runtime */
	res = nrf_edgeai_init(p_model);
	if (res != NRF_EDGEAI_ERR_SUCCESS) {
		LOG_ERR("Failed to initialize EdgeAI: %d", res);
		return -EIO;
	}

	/* Reset detection state */
	last_published_class = UINT16_MAX;

	LOG_INF("EdgeAI model initialized:");
	LOG_INF("  Window size: %u samples", nrf_edgeai_input_window_size(p_model));
	LOG_INF("  Input features: %u", nrf_edgeai_uniq_inputs_num(p_model));
	LOG_INF("  Output classes: %u", nrf_edgeai_model_outputs_num(p_model));

	return 0;
}

void detection_reset_state(void)
{
	last_published_class = UINT16_MAX;
	LOG_DBG("Detection state reset - next detection will be published");
}
