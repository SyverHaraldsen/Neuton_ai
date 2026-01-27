/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _BUTTON_H_
#define _BUTTON_H_

#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>

/* Button Event Types */
enum button_event {
	BUTTON_EVENT_SINGLE_PRESS,
	BUTTON_EVENT_DOUBLE_PRESS,
	BUTTON_EVENT_LONG_PRESS,
};

/* Button Event Message Type */
struct button_event_msg {
	enum button_event event;
};

/* Zbus Channels */
ZBUS_CHAN_DECLARE(BUTTON_CHAN);

/* Button Initialization */
int button_init(void);

#endif /* _BUTTON_H_ */
