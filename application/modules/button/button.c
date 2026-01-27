/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>
#include <dk_buttons_and_leds.h>

#include "button.h"

LOG_MODULE_REGISTER(app_button, CONFIG_APP_BUTTON_LOG_LEVEL);

/* Zbus Channel Definitions */
ZBUS_CHAN_DEFINE(BUTTON_CHAN,
		 struct button_event_msg,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(0)
)

/* Button state tracking */
static struct {
	uint32_t button_press_count;
	uint64_t last_button_press_time;
	uint32_t pressed_buttons;
	struct k_work_delayable button_work;
	struct k_work_delayable long_press_work;
} button_state;

/* Button pressed callback */
static void button_pressed_callback(uint32_t button_state_mask, uint32_t has_changed)
{
	if (has_changed == 0) {
		return;
	}

	/* Handle button 1 press/release */
	if (!(has_changed & DK_BTN1_MSK)) {
		return;
	}

	/* Button pressed */
	if (button_state_mask & DK_BTN1_MSK) {
		int64_t now = k_uptime_get();

		if (now - button_state.last_button_press_time < CONFIG_APP_BUTTON_DEBOUNCE_TIME_MS) {
			return;
		}

		button_state.pressed_buttons |= DK_BTN1_MSK;
		button_state.last_button_press_time = now;
		button_state.button_press_count++;

		/* Start long press timer */
		k_work_schedule(&button_state.long_press_work,
				K_MSEC(CONFIG_APP_BUTTON_LONG_PRESS_TIMEOUT_MS));

		LOG_DBG("Button pressed, count: %d", button_state.button_press_count);
	}
	/* Button released */
	else {
		button_state.pressed_buttons &= ~DK_BTN1_MSK;

		/* Cancel long press timer if it was running */
		if (k_work_delayable_is_pending(&button_state.long_press_work)) {
			k_work_cancel_delayable(&button_state.long_press_work);

			/* Schedule short/double press detection */
			k_work_reschedule(&button_state.button_work,
					  K_MSEC(CONFIG_APP_BUTTON_DOUBLE_PRESS_TIMEOUT_MS));

			LOG_DBG("Button released");
		}
	}
}

/* Long press work handler */
static void long_press_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	struct button_event_msg msg;

	/* Check if button is still pressed */
	if (button_state.pressed_buttons & DK_BTN1_MSK) {
		LOG_INF("Button long press detected");

		msg.event = BUTTON_EVENT_LONG_PRESS;
		zbus_chan_pub(&BUTTON_CHAN, &msg, K_NO_WAIT);

		/* Reset button press count and state */
		button_state.button_press_count = 0;
		button_state.last_button_press_time = 0;
	}
}

/* Button work handler */
static void button_work_handler(struct k_work *work)
{
	struct button_event_msg msg;

	if (button_state.button_press_count == 1) {
		msg.event = BUTTON_EVENT_SINGLE_PRESS;
	} else if (button_state.button_press_count == 2) {
		msg.event = BUTTON_EVENT_DOUBLE_PRESS;
	} else {
		LOG_ERR("Invalid button press count: %d", button_state.button_press_count);
		button_state.button_press_count = 0;
		button_state.last_button_press_time = 0;
		return;
	}

	zbus_chan_pub(&BUTTON_CHAN, &msg, K_NO_WAIT);

	button_state.button_press_count = 0;
	button_state.last_button_press_time = 0;
}

int button_init(void)
{
	int err;

	button_state.button_press_count = 0;
	button_state.last_button_press_time = 0;
	button_state.pressed_buttons = 0;

	k_work_init_delayable(&button_state.button_work, button_work_handler);
	k_work_init_delayable(&button_state.long_press_work, long_press_work_handler);

	err = dk_buttons_init(button_pressed_callback);
	if (err) {
		LOG_ERR("Failed to initialize buttons: %d", err);
		return err;
	}

	return 0;
}
