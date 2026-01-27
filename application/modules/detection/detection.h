/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _DETECTION_H_
#define _DETECTION_H_

#include <zephyr/zbus/zbus.h>
#include <stdint.h>

/**
 * @brief Detection result structure published on Zbus
 */
struct detection_result {
	uint16_t predicted_class;  /* Predicted class (0-6) */
	float confidence;          /* Confidence score (0.0-1.0) */
	uint32_t timestamp;        /* Timestamp of detection */
};

/* Zbus channel declaration for detection results */
ZBUS_CHAN_DECLARE(detection_result_chan);

/**
 * @brief Initialize the detection module
 * @return 0 on success, negative error code on failure
 */
int detection_init(void);

/**
 * @brief Reset detection state (clears last published class)
 * Call this when starting a new detection session to ensure
 * the first detection result is always published.
 */
void detection_reset_state(void);

#endif /* _DETECTION_H_ */
