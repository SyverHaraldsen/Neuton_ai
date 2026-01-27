/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/smf.h>

#include "../modules/button/button.h"
#include "../modules/sampling/sampling.h"
#include "../modules/detection/detection.h"

LOG_MODULE_REGISTER(app_main, LOG_LEVEL_DBG);

enum app_states {
	STATE_IDLE,
	STATE_DETECTING,
	STATE_SENSOR_SAMPLING
};

struct app_context {
	struct smf_ctx ctx;
};

static struct app_context app_ctx;

static const char *DETECTION_CLASS_NAMES[] = {
	"Idle", "Shaking", "Impact", "Free Fall", "Carrying", "in Car", "Placed",
};

enum {
	DETECTION_CLASS_IDLE,
	DETECTION_CLASS_SHAKING,
	DETECTION_CLASS_IMPACT,
	DETECTION_CLASS_FREE_FALL,
	DETECTION_CLASS_CARRYING,
	DETECTION_CLASS_IN_CAR,
	DETECTION_CLASS_PLACED,
};

static void idle_entry(void *obj);
static enum smf_state_result idle_run(void *obj);

static void detecting_entry(void *obj);
static enum smf_state_result detecting_run(void *obj);
static void detecting_exit(void *obj);

static void sensor_sampling_entry(void *obj);
static enum smf_state_result sensor_sampling_run(void *obj);
static void sensor_sampling_exit(void *obj);

static const struct smf_state states[] = {
	[STATE_IDLE] = SMF_CREATE_STATE(
		idle_entry,
		idle_run,
		NULL,
		NULL,
		NULL
	),
	[STATE_DETECTING] = SMF_CREATE_STATE(
		detecting_entry,
		detecting_run,
		detecting_exit,
		NULL,
		NULL
	),
	[STATE_SENSOR_SAMPLING] = SMF_CREATE_STATE(
		sensor_sampling_entry,
		sensor_sampling_run,
		sensor_sampling_exit,
		NULL,
		NULL
	),
};

static void idle_entry(void *obj)
{
	ARG_UNUSED(obj);
	LOG_INF("Idle");
}

static enum smf_state_result idle_run(void *obj)
{
	k_sleep(K_MSEC(1000));
	return SMF_EVENT_HANDLED;
}

static void detecting_entry(void *obj)
{
	ARG_UNUSED(obj);
	LOG_INF("Detection started");

	detection_reset_state();
	sampling_set_print_enabled(false);

	int err = sampling_start();
	if (err) {
		LOG_ERR("sampling start failed: %d", err);
		smf_set_state(SMF_CTX(&app_ctx), &states[STATE_IDLE]);
	}
}

static enum smf_state_result detecting_run(void *obj)
{
	ARG_UNUSED(obj);
	k_sleep(K_MSEC(100));
	return SMF_EVENT_HANDLED;
}

static void detecting_exit(void *obj)
{
	ARG_UNUSED(obj);
	LOG_INF("Detection stopped");

	int err = sampling_stop();
	if (err) {
		LOG_ERR("sampling stop failed: %d", err);
	}
}

static void sensor_sampling_entry(void *obj)
{
	ARG_UNUSED(obj);
	LOG_INF("Raw sampling started");

	sampling_set_print_enabled(true);

	int err = sampling_start();
	if (err) {
		LOG_ERR("sampling start failed: %d", err);
		smf_set_state(SMF_CTX(&app_ctx), &states[STATE_IDLE]);
	}
}

static enum smf_state_result sensor_sampling_run(void *obj)
{
	ARG_UNUSED(obj);
	k_sleep(K_MSEC(100));
	return SMF_EVENT_HANDLED;
}

static void sensor_sampling_exit(void *obj)
{
	ARG_UNUSED(obj);
	LOG_INF("Raw sampling stopped");

	int err = sampling_stop();
	if (err) {
		LOG_ERR("sampling stop failed: %d", err);
	}
}

static void button_listener_callback(const struct zbus_channel *chan)
{
	const struct button_event_msg *button_msg = zbus_chan_const_msg(chan);

	switch (button_msg->event) {
		case BUTTON_EVENT_SINGLE_PRESS:
			smf_set_state(SMF_CTX(&app_ctx), &states[STATE_DETECTING]);
			break;
		case BUTTON_EVENT_DOUBLE_PRESS:
			smf_set_state(SMF_CTX(&app_ctx), &states[STATE_SENSOR_SAMPLING]);
			break;
		case BUTTON_EVENT_LONG_PRESS:
			smf_set_state(SMF_CTX(&app_ctx), &states[STATE_IDLE]);
			break;
	}
}

ZBUS_LISTENER_DEFINE(button_listener, button_listener_callback);

static void detection_result_listener_callback(const struct zbus_channel *chan)
{
	const struct detection_result *result = zbus_chan_const_msg(chan);

	LOG_INF("%s (%u%%)",
		DETECTION_CLASS_NAMES[result->predicted_class],
		(uint32_t)(result->confidence * 100.0f));
}

ZBUS_LISTENER_DEFINE(detection_result_listener, detection_result_listener_callback);

int main(void)
{
	int err;

	err = sampling_init();
	if (err) {
		LOG_ERR("sampling_init: %d", err);
		return err;
	}

	err = sampling_set_frequency(100);
	if (err) {
		LOG_ERR("sampling_set_frequency: %d", err);
		return err;
	}

	err = detection_init();
	if (err) {
		LOG_ERR("detection_init: %d", err);
		return err;
	}

	err = button_init();
	if (err) {
		LOG_ERR("button_init: %d", err);
		return err;
	}

	err = zbus_chan_add_obs(&BUTTON_CHAN, &button_listener, K_MSEC(100));
	if (err) {
		LOG_ERR("zbus button subscribe: %d", err);
		return err;
	}

	err = zbus_chan_add_obs(&detection_result_chan, &detection_result_listener, K_MSEC(100));
	if (err) {
		LOG_ERR("zbus detection subscribe: %d", err);
		return err;
	}

	smf_set_initial(SMF_CTX(&app_ctx), &states[STATE_IDLE]);

	while (1) {
		err = smf_run_state(SMF_CTX(&app_ctx));
		if (err) {
			LOG_ERR("smf_run_state: %d", err);
			return err;
		}
		k_yield();
	}

	return 0;
}
