/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/zbus/zbus.h>
#include <errno.h>
#include "sampling.h"

LOG_MODULE_REGISTER(app_sampling, CONFIG_APP_SAMPLING_LOG_LEVEL);

/* Zbus channel for IMU data */
ZBUS_CHAN_DEFINE(imu_data_chan,
		 struct imu_sample,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(0));

static const struct device *imu_dev;
static bool sampling_active = false;
static bool print_enabled = false;
static struct k_timer sampling_timer;
static K_SEM_DEFINE(sampling_sem, 0, 1);

/* Sampling thread */
#define SAMPLING_STACK_SIZE 2048
#define SAMPLING_PRIORITY 5

static void sampling_thread_fn(void *arg1, void *arg2, void *arg3);
K_THREAD_DEFINE(sampling_thread, SAMPLING_STACK_SIZE,
		sampling_thread_fn, NULL, NULL, NULL,
		SAMPLING_PRIORITY, 0, 0);

/* Timer handler */
static void sampling_timer_handler(struct k_timer *timer)
{
	k_sem_give(&sampling_sem);
}

int sampling_init(void)
{
	imu_dev = DEVICE_DT_GET(DT_ALIAS(imu0));
	if (!device_is_ready(imu_dev)) {
		LOG_ERR("IMU device not ready");
		return -ENODEV;
	}

	/* Initialize timer */
	k_timer_init(&sampling_timer, sampling_timer_handler, NULL);

	LOG_INF("IMU initialized: %s", imu_dev->name);
	return 0;
}

int sampling_set_frequency(int frequency_hz)
{
	int ret;
	struct sensor_value odr;

	if (!imu_dev) {
		LOG_ERR("IMU not initialized");
		return -ENODEV;
	}

	odr.val1 = frequency_hz;
	odr.val2 = 0;

	/* Set accelerometer ODR */
	ret = sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ,
			      SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
	if (ret) {
		LOG_ERR("Failed to set accel ODR: %d", ret);
		return ret;
	}

	/* Set gyroscope ODR */
	ret = sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ,
			      SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
	if (ret) {
		LOG_ERR("Failed to set gyro ODR: %d", ret);
		return ret;
	}

	LOG_INF("Sampling frequency set to %d Hz", frequency_hz);
	return 0;
}

int sampling_get_sample(struct imu_sample *sample)
{
	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	int ret;

	if (!imu_dev) {
		LOG_ERR("IMU not initialized");
		return -ENODEV;
	}

	if (!sample) {
		return -EINVAL;
	}

	/* Fetch sensor data */
	ret = sensor_sample_fetch(imu_dev);
	if (ret) {
		LOG_ERR("Failed to fetch sensor data: %d", ret);
		return ret;
	}

	/* Get accelerometer data */
	ret = sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
	if (ret) {
		LOG_ERR("Failed to get accel data: %d", ret);
		return ret;
	}

	/* Get gyroscope data */
	ret = sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_XYZ, gyro);
	if (ret) {
		LOG_ERR("Failed to get gyro data: %d", ret);
		return ret;
	}

	/* Convert to double */
	sample->accel_x = sensor_value_to_double(&accel[0]);
	sample->accel_y = sensor_value_to_double(&accel[1]);
	sample->accel_z = sensor_value_to_double(&accel[2]);
	sample->gyro_x = sensor_value_to_double(&gyro[0]);
	sample->gyro_y = sensor_value_to_double(&gyro[1]);
	sample->gyro_z = sensor_value_to_double(&gyro[2]);

	return 0;
}

static void sampling_thread_fn(void *arg1, void *arg2, void *arg3)
{
	struct imu_sample sample;
	int ret;

	LOG_INF("Sampling thread started");

	while (1) {
		/* Wait for trigger */
		k_sem_take(&sampling_sem, K_FOREVER);

		if (!sampling_active) {
			continue;
		}

		/* Get sample */
		ret = sampling_get_sample(&sample);
		if (ret) {
			LOG_ERR("Failed to get sample: %d", ret);
			continue;
		}

		/* Publish to zbus */
		ret = zbus_chan_pub(&imu_data_chan, &sample, K_NO_WAIT);
		if (ret) {
			LOG_WRN("Failed to publish: %d", ret);
		}

		/* Only print if enabled (for raw sampling mode) */
		if (print_enabled) {
			printk("%f,%f,%f,%f,%f,%f\n",
				sample.accel_x, sample.accel_y, sample.accel_z,
				sample.gyro_x, sample.gyro_y, sample.gyro_z);
		}
	}
}

int sampling_start(void)
{
	if (sampling_active) {
		LOG_WRN("Sampling already active");
		return -EALREADY;
	}

	LOG_INF("Starting continuous sampling at %d Hz", CONFIG_APP_SAMPLING_FREQUENCY_HZ);
	sampling_active = true;

	/* Start timer at CONFIG_APP_SAMPLING_FREQUENCY_HZ Hz (1000 / CONFIG_APP_SAMPLING_FREQUENCY_HZ ms period) */
	k_timer_start(&sampling_timer, K_MSEC(1000 / CONFIG_APP_SAMPLING_FREQUENCY_HZ), K_MSEC(1000 / CONFIG_APP_SAMPLING_FREQUENCY_HZ));

	return 0;
}

int sampling_stop(void)
{
	if (!sampling_active) {
		LOG_WRN("Sampling not active");
		return -EALREADY;
	}

	LOG_INF("Stopping continuous sampling");
	sampling_active = false;
	k_timer_stop(&sampling_timer);

	return 0;
}

void sampling_set_print_enabled(bool enabled)
{
	print_enabled = enabled;
	LOG_DBG("Sample printing %s", enabled ? "enabled" : "disabled");
}
