/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _SAMPLING_H_
#define _SAMPLING_H_

#include <zephyr/zbus/zbus.h>

struct imu_sample {
	double accel_x;
	double accel_y;
	double accel_z;
	double gyro_x;
	double gyro_y;
	double gyro_z;
};

/* Zbus channel declaration */
ZBUS_CHAN_DECLARE(imu_data_chan);

int sampling_init(void);
int sampling_set_frequency(int frequency_hz);
int sampling_get_sample(struct imu_sample *sample);
int sampling_start(void);
int sampling_stop(void);
void sampling_set_print_enabled(bool enabled);

#endif /* _SAMPLING_H_ */
