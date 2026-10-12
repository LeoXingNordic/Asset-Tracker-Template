/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/smf.h>
#include <zephyr/sys/timeutil.h>
#include <zephyr/sys/util.h>
#include <zephyr/task_wdt/task_wdt.h>
#include <zephyr/zbus/zbus.h>
#include <date_time.h>
#include <hyperpulse_lib.h>
#include <nrf_modem_at.h>
#include <nrf_modem_gnss.h>

#if defined(CONFIG_APP_NTN_SENML_PAYLOAD)
#include <zcbor_encode.h>
#endif

#include "app_common.h"
#include "button.h"
#include "ntn.h"
#if defined(CONFIG_APP_POWER)
#include "power.h"
#endif /* CONFIG_APP_POWER */
#if defined(CONFIG_APP_LED)
#include "led.h"
#endif /* CONFIG_APP_LED */

LOG_MODULE_REGISTER(ntn_module, CONFIG_APP_NTN_LOG_LEVEL);

#define HYPERPULSE_AT_RESPONSE_LEN 550
#define HYPERPULSE_UPLINK_MAX_LEN    250
#define HYPERPULSE_ICCID_LEN         20

/* Define channels provided by this module */
ZBUS_CHAN_DEFINE(NTN_CHAN,
		 struct ntn_msg,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(0)
);

ZBUS_MSG_SUBSCRIBER_DEFINE(ntn);
ZBUS_CHAN_ADD_OBS(NTN_CHAN, ntn, 0);
ZBUS_CHAN_ADD_OBS(BUTTON_CHAN, ntn, 0);
#if defined(CONFIG_APP_POWER)
ZBUS_CHAN_ADD_OBS(POWER_CHAN, ntn, 0);
#endif /* CONFIG_APP_POWER */

#if defined(CONFIG_APP_POWER)
#define MAX_MSG_SIZE MAX(sizeof(struct ntn_msg), sizeof(struct power_msg))
#else
#define MAX_MSG_SIZE sizeof(struct ntn_msg)
#endif

enum ntn_module_state {
	STATE_RUNNING,
	STATE_GNSS,
	STATE_NTN,
	STATE_IDLE,
};

struct ntn_state_object {
	struct smf_ctx ctx;
	const struct zbus_channel *chan;
	uint8_t msg_buf[MAX_MSG_SIZE];
	struct k_timer ntn_trigger_timer;
	struct k_timer gnss_timeout_timer;
	struct k_timer ntn_send_timeout_timer;
	struct nrf_modem_gnss_pvt_data_frame last_pvt;
	uint64_t location_validity_end_time;
	bool location_valid;
	/* Battery level in tenths of a percent, negative until the first sample arrives.
	 * Written by the NTN thread, read by the send work.
	 */
	atomic_t battery_tenths;
};

static struct k_work ntn_trigger_timer_work;
static struct k_work gnss_timeout_work;
static struct k_work ntn_send_timeout_work;
static struct k_work ntn_send_work;

static struct ntn_state_object *active_state;

static void state_running_entry(void *obj);
static enum smf_state_result state_running_run(void *obj);
static void state_gnss_entry(void *obj);
static enum smf_state_result state_gnss_run(void *obj);
static void state_ntn_entry(void *obj);
static enum smf_state_result state_ntn_run(void *obj);
static void state_idle_entry(void *obj);
static enum smf_state_result state_idle_run(void *obj);

static const struct smf_state states[] = {
	[STATE_RUNNING] = SMF_CREATE_STATE(state_running_entry, state_running_run, NULL,
					   NULL, &states[STATE_GNSS]),
	[STATE_GNSS] = SMF_CREATE_STATE(state_gnss_entry, state_gnss_run, NULL,
					&states[STATE_RUNNING], NULL),
	[STATE_NTN] = SMF_CREATE_STATE(state_ntn_entry, state_ntn_run, NULL,
				       &states[STATE_RUNNING], NULL),
	[STATE_IDLE] = SMF_CREATE_STATE(state_idle_entry, state_idle_run, NULL,
					&states[STATE_RUNNING], NULL),
};

/* Status LED and battery helpers, no-ops when the modules are disabled */

static void led_status_set(uint8_t red, uint8_t green, uint8_t blue,
			   uint32_t on_msec, uint32_t off_msec, int repetitions)
{
#if defined(CONFIG_APP_LED)
	struct led_msg led_msg = {
		.type = LED_RGB_SET,
		.red = red,
		.green = green,
		.blue = blue,
		.duration_on_msec = on_msec,
		.duration_off_msec = off_msec,
		.repetitions = repetitions,
	};
	int err = zbus_chan_pub(&LED_CHAN, &led_msg, K_SECONDS(1));

	if (err) {
		LOG_WRN("Failed to publish LED pattern, error: %d", err);
	}
#else
	ARG_UNUSED(red);
	ARG_UNUSED(green);
	ARG_UNUSED(blue);
	ARG_UNUSED(on_msec);
	ARG_UNUSED(off_msec);
	ARG_UNUSED(repetitions);
#endif /* CONFIG_APP_LED */
}

static void battery_sample_request(void)
{
#if defined(CONFIG_APP_POWER)
	struct power_msg power_msg = {
		.type = POWER_BATTERY_PERCENTAGE_SAMPLE_REQUEST,
	};
	int err = zbus_chan_pub(&POWER_CHAN, &power_msg, K_SECONDS(1));

	if (err) {
		LOG_WRN("Failed to request battery sample, error: %d", err);
	}
#endif /* CONFIG_APP_POWER */
}

static int hyperpulse_send_at_check_ok(const char *cmd)
{
	char response[HYPERPULSE_AT_RESPONSE_LEN] = {0};

	if (hyperpulse_lib_send_at_command(cmd, response, sizeof(response)) != 0) {
		return -EIO;
	}

	if (strstr(response, "OK") == NULL) {
		LOG_ERR("AT command failed: %s, response: %s", cmd, response);
		return -EIO;
	}

	return 0;
}

static void ntn_msg_publish(enum ntn_msg_type type)
{
	int err;
	struct ntn_msg msg = {
		.type = type,
	};

	err = zbus_chan_pub(&NTN_CHAN, &msg, K_SECONDS(1));
	if (err) {
		LOG_ERR("Failed to publish NTN message, error: %d", err);
	}
}

static void publish_last_pvt(const struct nrf_modem_gnss_pvt_data_frame *pvt)
{
	int err;
	struct ntn_msg msg = {
		.type = NTN_LOCATION_SEARCH_DONE,
		.pvt = *pvt,
	};

	err = zbus_chan_pub(&NTN_CHAN, &msg, K_SECONDS(1));
	if (err) {
		LOG_ERR("Failed to publish last PVT message, error: %d", err);
	}
}

#if defined(CONFIG_APP_NTN_THINGY_ROCKS_ENDPOINT)
/* Link statistics for the thingy.rocks payload.
 *
 * The HyperPulse library owns the modem AT notification handler, so the handler registration is
 * wrapped at link time (-Wl,--wrap=nrf_modem_at_notif_handler_set, see CMakeLists.txt) to tap
 * notifications before they are passed on to the library unchanged. The library enables
 * AT%CESQ=1 and AT+CEREG=5 itself.
 *
 * AT#MSEND only queues the uplink, which is sent on the next RRC connection, so a payload always
 * carries the statistics of the previous connection.
 */
#define LINK_STAT_UNKNOWN		-1
#define CESQ_RSRP_UNKNOWN		255
#define CEREG_STAT_HOME			1
#define CEREG_STAT_SEARCHING		2
#define CEREG_STAT_ROAMING		5

/* RSRP index as reported by %CESQ: dBm = index - 140 */
static atomic_t link_rsrp_index = ATOMIC_INIT(LINK_STAT_UNKNOWN);
/* Time from "UE trying to attach" (+CEREG: 2) to registered (+CEREG: 1/5) */
static atomic_t link_attach_latency_ms = ATOMIC_INIT(LINK_STAT_UNKNOWN);
static atomic_t link_attach_start_ms;

static nrf_modem_at_notif_handler_t lib_notif_handler;

#if defined(CONFIG_APP_NTN_THINGY_ROCKS_REFRESH_ON_FIRST_ATTACH)
/* Delay after registration so the first %CESQ of the connection can arrive */
#define LINK_STATS_REFRESH_DELAY_MS	2000

static void link_stats_refresh_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(link_stats_refresh_work, link_stats_refresh_work_fn);
static atomic_t link_stats_refresh_done;
#endif

int __real_nrf_modem_at_notif_handler_set(nrf_modem_at_notif_handler_t callback);

/* Runs in ISR context, keep it short */
static void link_stats_notif_tap(const char *notif)
{
	if (strncmp(notif, "%CESQ: ", 7) == 0) {
		long rsrp = strtol(notif + 7, NULL, 10);

		if (rsrp >= 0 && rsrp < CESQ_RSRP_UNKNOWN) {
			atomic_set(&link_rsrp_index, rsrp);
		}
	} else if (strncmp(notif, "+CEREG: ", 8) == 0) {
		long stat = strtol(notif + 8, NULL, 10);
		uint32_t now = k_uptime_get_32();

		if (stat == CEREG_STAT_SEARCHING) {
			/* Keep the first "searching" report of an attach attempt */
			atomic_cas(&link_attach_start_ms, 0, now ? now : 1);
		} else if (stat == CEREG_STAT_HOME || stat == CEREG_STAT_ROAMING) {
			uint32_t start = atomic_set(&link_attach_start_ms, 0);

			if (start != 0) {
				atomic_set(&link_attach_latency_ms, now - start);
#if defined(CONFIG_APP_NTN_THINGY_ROCKS_REFRESH_ON_FIRST_ATTACH)
				if (atomic_cas(&link_stats_refresh_done, 0, 1)) {
					k_work_schedule(&link_stats_refresh_work,
							K_MSEC(LINK_STATS_REFRESH_DELAY_MS));
				}
#endif
			}
		} else {
			atomic_set(&link_attach_start_ms, 0);
		}
	}

	if (lib_notif_handler) {
		lib_notif_handler(notif);
	}
}

int __wrap_nrf_modem_at_notif_handler_set(nrf_modem_at_notif_handler_t callback)
{
	lib_notif_handler = callback;

	return __real_nrf_modem_at_notif_handler_set(callback ? link_stats_notif_tap : NULL);
}
#endif /* CONFIG_APP_NTN_THINGY_ROCKS_ENDPOINT */

static void apply_gnss_time(const struct nrf_modem_gnss_pvt_data_frame *pvt_data)
{
	int err;
	struct tm gnss_time = {
		.tm_year = pvt_data->datetime.year - 1900,
		.tm_mon = pvt_data->datetime.month - 1,
		.tm_mday = pvt_data->datetime.day,
		.tm_hour = pvt_data->datetime.hour,
		.tm_min = pvt_data->datetime.minute,
		.tm_sec = pvt_data->datetime.seconds,
	};

	err = date_time_set(&gnss_time);
	if (err) {
		LOG_ERR("Failed to apply GNSS time, error: %d", err);
	}
}

static int parse_datetime_string(const char *time_str, struct tm *out)
{
	if (sscanf(time_str, "%d-%d-%d-%d:%d:%d",
		   &out->tm_year, &out->tm_mon, &out->tm_mday,
		   &out->tm_hour, &out->tm_min, &out->tm_sec) != 6) {
		return -EINVAL;
	}

	out->tm_year -= 1900;
	out->tm_mon -= 1;

	return 0;
}

static int apply_manual_datetime(const char *datetime_str)
{
	struct tm manual_time = {0};
	int err;

	err = parse_datetime_string(datetime_str, &manual_time);
	if (err) {
		LOG_ERR("Failed to parse manual date time");
		return err;
	}

	err = date_time_set(&manual_time);
	if (err) {
		LOG_ERR("Failed to set manual date time: %d", err);
		return err;
	}

	LOG_INF("Applied manual date time: %s", datetime_str);

	return 0;
}

static int myriota_get_imei(char *imei, size_t imei_len)
{
	char response[HYPERPULSE_AT_RESPONSE_LEN] = {0};

	if (hyperpulse_lib_send_at_command("AT+CGSN=1", response, sizeof(response)) != 0) {
		return -EIO;
	}

	/* Modem returns quoted IMEI: +CGSN: "359404235474245" — avoid capturing the quote. */
	if (sscanf(response, "+CGSN: \"%15[^\"]\"", imei) != 1 &&
	    sscanf(response, "+CGSN: %15[0-9]", imei) != 1) {
		return -EINVAL;
	}

	imei[imei_len - 1] = '\0';

	return 0;
}

static int myriota_get_temperature(char *temp, size_t temp_len)
{
	char response[HYPERPULSE_AT_RESPONSE_LEN] = {0};
	int8_t temperature_celsius;

	if (hyperpulse_lib_send_at_command("AT%XTEMP?", response, sizeof(response)) != 0) {
		return -EIO;
	}

	if (strstr(response, "OK") == NULL) {
		return -EIO;
	}

	if (sscanf(response, "%%XTEMP: %hhd", &temperature_celsius) != 1) {
		return -EINVAL;
	}

	snprintk(temp, temp_len, "%d", temperature_celsius);

	return 0;
}

static int myriota_print_iccid(void)
{
	char response[HYPERPULSE_AT_RESPONSE_LEN] = {0};
	char iccid[HYPERPULSE_ICCID_LEN + 1] = {0};

	if (hyperpulse_lib_send_at_command("AT%XICCID", response, sizeof(response)) != 0) {
		LOG_ERR("Failed to send ICCID AT command");
		return -EIO;
	}

	if (sscanf(response, "%%XICCID: %20s", iccid) != 1) {
		return -EINVAL;
	}

	LOG_INF("ICCID: %s", iccid);

	return 0;
}

static int myriota_enable_downlink_notifications(void)
{
	return hyperpulse_send_at_check_ok("AT#MRECV=1");
}

static int myriota_request_gnss_location(void)
{
	char command[HYPERPULSE_AT_RESPONSE_LEN];

	snprintk(command, sizeof(command), "AT#MLOCATION?%u,%u,%u,%u",
		 0, 0, 0, CONFIG_APP_NTN_MYRIOTA_GNSS_TIMEOUT_SECONDS);

	return hyperpulse_send_at_check_ok(command);
}

static int myriota_parse_location_response(const char *response,
					   struct nrf_modem_gnss_pvt_data_frame *pvt)
{
	int method, lat, lon, elev;
	time_t timestamp;
	struct nrf_modem_gnss_pvt_data_frame modem_pvt;

	if (!response || !pvt || strncmp(response, "#MLOCATION:", 11) != 0) {
		return -EINVAL;
	}

	/* #MLOCATION: <method>,<lat>,<lon>,<elev>,<timestamp_s>, carries no accuracy */
	if (sscanf(response, "#MLOCATION: %d,%d,%d,%d,%jd,", &method, &lat, &lon, &elev,
		   &timestamp) != 5) {
		return -EINVAL;
	}

	memset(pvt, 0, sizeof(*pvt));
	pvt->latitude = (double)lat / 1e7;
	pvt->longitude = (double)lon / 1e7;
	pvt->altitude = (float)elev / 1000.0f;
	pvt->flags |= NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID;

	/* Take the accuracy from the modem's latest PVT frame, if it is the same fix */
	if (nrf_modem_gnss_read(&modem_pvt, sizeof(modem_pvt), NRF_MODEM_GNSS_DATA_PVT) == 0 &&
	    (modem_pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID) &&
	    fabs(modem_pvt.latitude - pvt->latitude) < 1e-4 &&
	    fabs(modem_pvt.longitude - pvt->longitude) < 1e-4) {
		pvt->accuracy = modem_pvt.accuracy;
	} else {
		LOG_WRN("GNSS accuracy not available");
	}

	if (timestamp > 0) {
		struct tm gnss_time;

		gmtime_r(&timestamp, &gnss_time);
		pvt->datetime.year = gnss_time.tm_year + 1900;
		pvt->datetime.month = gnss_time.tm_mon + 1;
		pvt->datetime.day = gnss_time.tm_mday;
		pvt->datetime.hour = gnss_time.tm_hour;
		pvt->datetime.minute = gnss_time.tm_min;
		pvt->datetime.seconds = gnss_time.tm_sec;
	}

	LOG_INF("GNSS fix: lat=%.6f lon=%.6f alt=%.1f acc=%.1f method=%d",
		pvt->latitude, pvt->longitude, (double)pvt->altitude,
		(double)pvt->accuracy, method);

	return 0;
}

#if defined(CONFIG_APP_NTN_SENML_PAYLOAD)
/* SenML CBOR labels (RFC 8428 section 6), encoded as text strings because
 * world.thingy.rocks decodes integer-keyed maps into empty records.
 */
#define SENML_LABEL_BASE_NAME	"-2"
#define SENML_LABEL_BASE_TIME	"-3"
#define SENML_LABEL_NAME	"0"
#define SENML_LABEL_VALUE	"2"
#define SENML_LABEL_STRING_VALUE "3"

/* Max number of records (one map per resource) in the SenML pack */
#define SENML_MAX_RECORDS	8

static bool senml_record_float(zcbor_state_t *zse, const char *base_name, const int64_t *base_time,
			       const char *name, double value, bool full_precision)
{
	size_t entries = 2 + (base_name ? 1 : 0) + (base_time ? 1 : 0);
	bool ok = zcbor_map_start_encode(zse, entries);

	if (base_name) {
		ok = ok && zcbor_tstr_put_lit(zse, SENML_LABEL_BASE_NAME) &&
		     zcbor_tstr_put_term(zse, base_name, SIZE_MAX);
	}

	if (base_time) {
		ok = ok && zcbor_tstr_put_lit(zse, SENML_LABEL_BASE_TIME) &&
		     zcbor_int64_put(zse, *base_time);
	}

	ok = ok && zcbor_tstr_put_lit(zse, SENML_LABEL_NAME) &&
	     zcbor_tstr_put_term(zse, name, SIZE_MAX) &&
	     zcbor_tstr_put_lit(zse, SENML_LABEL_VALUE);

	if (full_precision) {
		ok = ok && zcbor_float64_put(zse, value);
	} else {
		ok = ok && zcbor_float32_put(zse, (float)value);
	}

	return ok && zcbor_map_end_encode(zse, entries);
}

/* [IMEI, [SenML records]] in CBOR, as expected by world.thingy.rocks (UDP port 6667 format).
 * The records carry LwM2M objects, see https://github.com/hello-nrfcloud/proto-map
 */
static int myriota_format_senml_payload(struct ntn_state_object *state, uint8_t *payload,
					size_t payload_size, size_t *payload_len)
{
	bool ok;
	int64_t timestamp;
	char imei[16] = {0};
	char temp[16] = {0};
	struct tm fix_time = {
		.tm_year = state->last_pvt.datetime.year - 1900,
		.tm_mon = state->last_pvt.datetime.month - 1,
		.tm_mday = state->last_pvt.datetime.day,
		.tm_hour = state->last_pvt.datetime.hour,
		.tm_min = state->last_pvt.datetime.minute,
		.tm_sec = state->last_pvt.datetime.seconds,
	};

	ZCBOR_STATE_E(zse, 3, payload, payload_size, 1);

	/* world.thingy.rocks identifies the device by its full IMEI */
	if (myriota_get_imei(imei, sizeof(imei)) < 0) {
		LOG_WRN("No IMEI for SenML payload");
		return -ENODATA;
	}

	/* Base time is mandatory. Prefer the GNSS fix time, fall back to system time. */
	if (state->last_pvt.datetime.year >= 2000) {
		timestamp = timeutil_timegm64(&fix_time);
	} else if (date_time_now(&timestamp) == 0) {
		timestamp /= MSEC_PER_SEC;
	} else {
		LOG_WRN("No valid time for SenML payload");
		return -ENODATA;
	}

	ok = zcbor_list_start_encode(zse, 2) && zcbor_tstr_put_term(zse, imei, sizeof(imei)) &&
	     zcbor_list_start_encode(zse, SENML_MAX_RECORDS);

	/* Geolocation (14201) */
	ok = ok && senml_record_float(zse, "14201/0/", &timestamp, "0",
				      state->last_pvt.latitude, true);
	ok = ok && senml_record_float(zse, NULL, NULL, "1", state->last_pvt.longitude, true);
	ok = ok && senml_record_float(zse, NULL, NULL, "2",
				      (double)state->last_pvt.altitude, false);
	ok = ok && senml_record_float(zse, NULL, NULL, "3",
				      (double)state->last_pvt.accuracy, false);
	ok = ok && zcbor_map_start_encode(zse, 2) &&
	     zcbor_tstr_put_lit(zse, SENML_LABEL_NAME) && zcbor_tstr_put_lit(zse, "6") &&
	     zcbor_tstr_put_lit(zse, SENML_LABEL_STRING_VALUE) && zcbor_tstr_put_lit(zse, "GNSS") &&
	     zcbor_map_end_encode(zse, 2);

	/* Environment (14205), reuses the base time set above */
	if (myriota_get_temperature(temp, sizeof(temp)) == 0) {
		ok = ok && senml_record_float(zse, "14205/0/", NULL, "0", atoi(temp), false);
	}

	ok = ok && zcbor_list_end_encode(zse, SENML_MAX_RECORDS) && zcbor_list_end_encode(zse, 2);
	if (!ok) {
		LOG_ERR("SenML CBOR encoding failed: %d", zcbor_peek_error(zse));
		return -ENOBUFS;
	}

	*payload_len = zse->payload - payload;

	return 0;
}
#endif /* CONFIG_APP_NTN_SENML_PAYLOAD */

static int myriota_format_payload(struct ntn_state_object *state, uint8_t *payload,
				  size_t payload_size, size_t *payload_len)
{
#if defined(CONFIG_APP_NTN_SENML_PAYLOAD)
	return myriota_format_senml_payload(state, payload, payload_size, payload_len);
#else
	char *message = (char *)payload;
	size_t message_len = payload_size;
	int err;
	char temp[16] = {0};
	char imei[16] = {0};
	char imei_suffix[5];

	err = myriota_get_imei(imei, sizeof(imei));
	if (err < 0) {
		snprintk(imei, sizeof(imei), "N/A");
	}

	err = myriota_get_temperature(temp, sizeof(temp));
	if (err < 0) {
		snprintk(temp, sizeof(temp), "N/A");
	}

	size_t imei_len = strnlen(imei, sizeof(imei));

	if (imei_len > 4) {
		snprintk(imei_suffix, sizeof(imei_suffix), "%s", imei + (imei_len - 4));
	} else {
		snprintk(imei_suffix, sizeof(imei_suffix), "N/A");
	}

#if defined(CONFIG_APP_NTN_THINGY_ROCKS_ENDPOINT)
	/* world.thingy.rocks UDP format (port 6666):
	 * imei,ts,ping,rsrp,band,ue_mode,operator,lat,lon,accuracy,battery,temp,pressure,humidity
	 * ts is left empty (receive time is used). Unknown ping/rsrp are left empty.
	 * rsrp is the %CESQ index (dBm + 140). Band 255 is the NTN band locked by HyperPulse.
	 */
	char ping[12] = "";
	char rsrp[12] = "";
	atomic_val_t latency_ms = atomic_get(&link_attach_latency_ms);
	atomic_val_t rsrp_index = atomic_get(&link_rsrp_index);

	if (latency_ms != LINK_STAT_UNKNOWN) {
		snprintk(ping, sizeof(ping), "%ld", (long)latency_ms);
	}

	if (rsrp_index != LINK_STAT_UNKNOWN) {
		snprintk(rsrp, sizeof(rsrp), "%ld", (long)rsrp_index);
	}

	char battery[16] = "99.9";

#if defined(CONFIG_APP_POWER)
	/* No sample yet: leave the field empty rather than reporting a made-up level */
	atomic_val_t battery_x10 = atomic_get(&state->battery_tenths);

	if (battery_x10 >= 0) {
		snprintk(battery, sizeof(battery), "%d.%d",
			 (int)(battery_x10 / 10), (int)(battery_x10 % 10));
	} else {
		battery[0] = '\0';
	}
#endif /* CONFIG_APP_POWER */

	err = snprintk(message, message_len,
		       "%s,,%s,%s,%s,%s,%s,%.6f,%.6f,%d,%s,%s,%s,%s",
		       imei,
		       ping,
		       rsrp,
		       "255",
		       "0",
		       "90129",
		       state->last_pvt.latitude,
		       state->last_pvt.longitude,
		       (int)state->last_pvt.accuracy,
		       battery, temp, "999.99", "99.99");
#elif defined(CONFIG_APP_NTN_SEND_GNSS_DATA)
	err = snprintk(message, message_len,
		       "Device: *%s, temp: %s, lat=%.2f, lon=%.2f, alt=%.2f, "
		       "time=%04d-%02d-%02d %02d:%02d:%02d",
		       imei_suffix, temp,
		       state->last_pvt.latitude, state->last_pvt.longitude,
		       (double)state->last_pvt.altitude,
		       state->last_pvt.datetime.year, state->last_pvt.datetime.month,
		       state->last_pvt.datetime.day, state->last_pvt.datetime.hour,
		       state->last_pvt.datetime.minute, state->last_pvt.datetime.seconds);
#else
	err = snprintk(message, message_len, "Device: *%s, temp: %s", imei_suffix, temp);
#endif

	if (err < 0 || err >= (int)message_len) {
		return -EINVAL;
	}

	*payload_len = err;

	return 0;
#endif /* CONFIG_APP_NTN_SENML_PAYLOAD */
}

static int myriota_send_payload(struct ntn_state_object *state)
{
	uint8_t message[HYPERPULSE_UPLINK_MAX_LEN + 1];
	char hex_string[(HYPERPULSE_UPLINK_MAX_LEN * 2) + 1];
	char command[HYPERPULSE_AT_RESPONSE_LEN];
	size_t msg_len;
	int err;

	if (!state->location_valid) {
		LOG_WRN("No valid GNSS data to send");
		return -EINVAL;
	}

	err = myriota_format_payload(state, message, sizeof(message), &msg_len);
	if (err) {
		return err;
	}

	if (msg_len > HYPERPULSE_UPLINK_MAX_LEN) {
		return -ENOBUFS;
	}

	err = bin2hex(message, msg_len, hex_string, sizeof(hex_string));
	ARG_UNUSED(err);

	snprintk(command, sizeof(command), "AT#MSEND=%zu,\"%s\"", msg_len, hex_string);

#if defined(CONFIG_APP_NTN_SENML_PAYLOAD)
	LOG_INF("Scheduling SenML CBOR uplink (%zu bytes): %s", msg_len, hex_string);
#else
	LOG_INF("Scheduling uplink: %.*s", (int)msg_len, (char *)message);
#endif

	return hyperpulse_send_at_check_ok(command);
}

static void ntn_trigger_timer_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	k_work_submit(&ntn_trigger_timer_work);
}

static void gnss_timeout_timer_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	k_work_submit(&gnss_timeout_work);
}

static void ntn_send_timeout_timer_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	k_work_submit(&ntn_send_timeout_work);
}

static void ntn_trigger_timer_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	ntn_msg_publish(NTN_TRIGGER);
}

static void gnss_timeout_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	LOG_WRN("GNSS search timed out");
	ntn_msg_publish(GNSS_TIMEOUT);
}

static void ntn_send_timeout_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	LOG_WRN("Myriota uplink timed out");
	ntn_msg_publish(NTN_SEND_FAILED);
}

#if defined(CONFIG_APP_NTN_THINGY_ROCKS_REFRESH_ON_FIRST_ATTACH)
/* The uplink queued before the first attach has no link statistics. Queue one more right after
 * the first registration, so it can go out in the same connection with the fresh values.
 */
static void link_stats_refresh_work_fn(struct k_work *work)
{
	int err;

	ARG_UNUSED(work);

	if (!active_state) {
		return;
	}

	LOG_INF("First attach done, queuing uplink with link statistics");

	err = myriota_send_payload(active_state);
	if (err) {
		LOG_WRN("Failed to queue uplink with link statistics, error: %d", err);
	}
}
#endif

static void ntn_send_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!active_state) {
		ntn_msg_publish(NTN_SEND_FAILED);
		return;
	}

	if (myriota_send_payload(active_state) != 0) {
		ntn_msg_publish(NTN_SEND_FAILED);
	} else {
		ntn_msg_publish(NTN_SEND_ACK);
	}
}

void hyperpulse_lib_unsolicited_at_response_app_handler(const char *const response)
{
	if (response == NULL) {
		return;
	}

	if (strncmp(response, "#MLOCATION:", 11) == 0) {
		struct nrf_modem_gnss_pvt_data_frame pvt;

		if (myriota_parse_location_response(response, &pvt) == 0) {
			if (active_state != NULL) {
				k_timer_stop(&active_state->gnss_timeout_timer);
			}
			apply_gnss_time(&pvt);
			publish_last_pvt(&pvt);
		}
		return;
	}

	if (strncmp(response, "#MRECV:", 7) == 0) {
		LOG_INF("Downlink received: %s", response);
		return;
	}

	LOG_DBG("Unhandled HyperPulse URC: %s", response);
}

static void ntn_wdt_callback(int channel_id, void *user_data)
{
	LOG_ERR("NTN watchdog expired, Channel: %d, Thread: %s",
		channel_id, k_thread_name_get((k_tid_t)user_data));
}

static void state_running_entry(void *obj)
{
	struct ntn_state_object *state = obj;

	LOG_DBG("%s", __func__);

	active_state = state;
	atomic_set(&state->battery_tenths, -1);
	battery_sample_request();

	k_work_init(&ntn_trigger_timer_work, ntn_trigger_timer_work_fn);
	k_work_init(&gnss_timeout_work, gnss_timeout_work_fn);
	k_work_init(&ntn_send_timeout_work, ntn_send_timeout_work_fn);
	k_work_init(&ntn_send_work, ntn_send_work_fn);

	k_timer_init(&state->ntn_trigger_timer, ntn_trigger_timer_handler, NULL);
	k_timer_init(&state->gnss_timeout_timer, gnss_timeout_timer_handler, NULL);
	k_timer_init(&state->ntn_send_timeout_timer, ntn_send_timeout_timer_handler, NULL);

	if (!hyperpulse_lib_is_initialised()) {
		LOG_ERR("HyperPulse library failed to initialise");
	}

	myriota_print_iccid();
	myriota_enable_downlink_notifications();

	k_timer_start(&state->ntn_trigger_timer,
		      K_MINUTES(CONFIG_APP_NTN_TIMER_TIMEOUT_MINUTES), K_NO_WAIT);
}

static enum smf_state_result state_running_run(void *obj)
{
	struct ntn_state_object *state = obj;

	if (state->chan == &NTN_CHAN) {
		struct ntn_msg *msg = (struct ntn_msg *)state->msg_buf;

		switch (msg->type) {
		case GNSS_TRIGGER:
			smf_set_state(SMF_CTX(state), &states[STATE_GNSS]);
			return SMF_EVENT_HANDLED;
		case IDLE_TRIGGER:
			smf_set_state(SMF_CTX(state), &states[STATE_IDLE]);
			return SMF_EVENT_HANDLED;
		case NTN_TRIGGER:
			k_timer_start(&state->ntn_trigger_timer,
				      K_MINUTES(CONFIG_APP_NTN_TIMER_TIMEOUT_MINUTES), K_NO_WAIT);
			if (!state->location_valid) {
				smf_set_state(SMF_CTX(state), &states[STATE_GNSS]);
			} else {
				smf_set_state(SMF_CTX(state), &states[STATE_NTN]);
			}
			return SMF_EVENT_HANDLED;
		case NTN_SHELL_SET_GNSS_LOCATION: {
			struct nrf_modem_gnss_pvt_data_frame pvt = msg->pvt;
			int64_t now_ms;

			if (date_time_now(&now_ms) == 0) {
				time_t now = now_ms / 1000;
				struct tm tm_now;

				gmtime_r(&now, &tm_now);
				pvt.datetime.year = tm_now.tm_year + 1900;
				pvt.datetime.month = tm_now.tm_mon + 1;
				pvt.datetime.day = tm_now.tm_mday;
				pvt.datetime.hour = tm_now.tm_hour;
				pvt.datetime.minute = tm_now.tm_min;
				pvt.datetime.seconds = tm_now.tm_sec;
			}

			pvt.flags |= NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID;
			if (pvt.accuracy <= 0.0f) {
				pvt.accuracy = 5.0f;
			}

			memcpy(&state->last_pvt, &pvt, sizeof(state->last_pvt));
			state->location_valid = true;
			if (msg->location_validity_seconds == 0) {
				state->location_validity_end_time = 0;
			} else {
				state->location_validity_end_time =
					k_uptime_get() +
					msg->location_validity_seconds * MSEC_PER_SEC;
			}

			LOG_INF("Stored manual GNSS location: lat=%.6f lon=%.6f alt=%.2f",
				state->last_pvt.latitude, state->last_pvt.longitude,
				(double)state->last_pvt.altitude);

			return SMF_EVENT_HANDLED;
		}
		case NTN_SHELL_SET_DATETIME:
			apply_manual_datetime(msg->datetime);
			return SMF_EVENT_HANDLED;
		case NTN_LOCATION_REQUEST: {
			uint64_t current_time = k_uptime_get();

			if (state->location_validity_end_time != 0 &&
			    current_time < state->location_validity_end_time) {
				LOG_DBG("Location still valid, skipping GNSS request");
				return SMF_EVENT_HANDLED;
			}

			smf_set_state(SMF_CTX(state), &states[STATE_GNSS]);
			return SMF_EVENT_HANDLED;
		}
		default:
			break;
		}
#if defined(CONFIG_APP_POWER)
	} else if (state->chan == &POWER_CHAN) {
		const struct power_msg *msg = (const struct power_msg *)state->msg_buf;

		if (msg->type == POWER_BATTERY_PERCENTAGE_SAMPLE_RESPONSE) {
			atomic_set(&state->battery_tenths, (atomic_val_t)(msg->percentage * 10.0));
			LOG_DBG("Battery: %d%% (%.2f V)", (int)msg->percentage, msg->voltage);

			return SMF_EVENT_HANDLED;
		}
#endif /* CONFIG_APP_POWER */
	} else if (state->chan == &BUTTON_CHAN) {
		k_timer_start(&state->ntn_trigger_timer,
			      K_MINUTES(CONFIG_APP_NTN_TIMER_TIMEOUT_MINUTES), K_NO_WAIT);
		if (!state->location_valid) {
			smf_set_state(SMF_CTX(state), &states[STATE_GNSS]);
		} else {
			smf_set_state(SMF_CTX(state), &states[STATE_NTN]);
		}
		return SMF_EVENT_HANDLED;
	}

	return SMF_EVENT_PROPAGATE;
}

static void state_gnss_entry(void *obj)
{
	struct ntn_state_object *state = obj;
	int err;

	LOG_DBG("%s", __func__);

	/* Blue slow blink: searching for GNSS fix */
	led_status_set(0, 0, 55, 250, 1000, -1);
	battery_sample_request();

	err = myriota_request_gnss_location();
	if (err) {
		LOG_ERR("Failed to request GNSS location: %d", err);
		ntn_msg_publish(GNSS_TIMEOUT);
		return;
	}

	k_timer_start(&state->gnss_timeout_timer,
		      K_SECONDS(CONFIG_APP_NTN_MYRIOTA_GNSS_TIMEOUT_SECONDS), K_NO_WAIT);
}

static enum smf_state_result state_gnss_run(void *obj)
{
	struct ntn_state_object *state = obj;

	if (state->chan != &NTN_CHAN) {
		return SMF_EVENT_PROPAGATE;
	}

	struct ntn_msg *msg = (struct ntn_msg *)state->msg_buf;

	switch (msg->type) {
	case NTN_LOCATION_SEARCH_DONE:
		memcpy(&state->last_pvt, &msg->pvt, sizeof(state->last_pvt));
		state->location_valid = true;
		state->location_validity_end_time =
			k_uptime_get() +
			CONFIG_APP_NTN_LOCATION_VALIDITY_TIME_SECONDS * MSEC_PER_SEC;
		k_timer_stop(&state->gnss_timeout_timer);
		smf_set_state(SMF_CTX(state), &states[STATE_NTN]);
		return SMF_EVENT_HANDLED;
	case GNSS_TIMEOUT:
		k_timer_stop(&state->gnss_timeout_timer);
		smf_set_state(SMF_CTX(state), &states[STATE_IDLE]);
		return SMF_EVENT_HANDLED;
	default:
		break;
	}

	return SMF_EVENT_PROPAGATE;
}

static void state_ntn_entry(void *obj)
{
	struct ntn_state_object *state = obj;

	LOG_DBG("%s", __func__);

	if (!state->location_valid) {
		LOG_WRN("No valid location for uplink");
		smf_set_state(SMF_CTX(state), &states[STATE_IDLE]);
		return;
	}

	/* Yellow blink: sending uplink */
	led_status_set(55, 30, 0, 250, 750, -1);

	k_timer_start(&state->ntn_send_timeout_timer,
		      K_SECONDS(CONFIG_APP_NTN_MYRIOTA_SEND_TIMEOUT_SECONDS), K_NO_WAIT);
	k_work_submit(&ntn_send_work);
}

static enum smf_state_result state_ntn_run(void *obj)
{
	struct ntn_state_object *state = obj;

	if (state->chan != &NTN_CHAN) {
		return SMF_EVENT_PROPAGATE;
	}

	struct ntn_msg *msg = (struct ntn_msg *)state->msg_buf;

	switch (msg->type) {
	case NTN_SEND_ACK:
		k_timer_stop(&state->ntn_send_timeout_timer);
		smf_set_state(SMF_CTX(state), &states[STATE_IDLE]);
		/* Green blinks: uplink acknowledged (after idle entry turned the LED off) */
		led_status_set(0, 55, 0, 250, 250, 6);
		return SMF_EVENT_HANDLED;
	case NTN_SEND_FAILED:
		k_timer_stop(&state->ntn_send_timeout_timer);
		smf_set_state(SMF_CTX(state), &states[STATE_IDLE]);
		/* Red blinks: uplink failed */
		led_status_set(55, 0, 0, 250, 250, 6);
		return SMF_EVENT_HANDLED;
	default:
		break;
	}

	return SMF_EVENT_PROPAGATE;
}

static void state_idle_entry(void *obj)
{
	ARG_UNUSED(obj);

	LOG_DBG("%s", __func__);

	/* LED off while idle to save power */
	led_status_set(0, 0, 0, 0, 0, 0);

	/* Fresh battery level for the next uplink */
	battery_sample_request();
}

static enum smf_state_result state_idle_run(void *obj)
{
	ARG_UNUSED(obj);

	return SMF_EVENT_PROPAGATE;
}

static void ntn_module_thread(void)
{
	int err;
	int task_wdt_id;
	const uint32_t wdt_timeout_ms = CONFIG_APP_NTN_WATCHDOG_TIMEOUT_SECONDS * MSEC_PER_SEC;
	const uint32_t execution_time_ms =
		CONFIG_APP_NTN_MSG_PROCESSING_TIMEOUT_SECONDS * MSEC_PER_SEC;
	const k_timeout_t zbus_wait_ms = K_MSEC(wdt_timeout_ms - execution_time_ms);
	struct ntn_state_object ntn_state = {0};

	task_wdt_id = task_wdt_add(wdt_timeout_ms, ntn_wdt_callback, (void *)k_current_get());
	if (task_wdt_id < 0) {
		LOG_ERR("Failed to add task to watchdog: %d", task_wdt_id);
		SEND_FATAL_ERROR();
		return;
	}

	smf_set_initial(SMF_CTX(&ntn_state), &states[STATE_RUNNING]);

	while (true) {
		err = task_wdt_feed(task_wdt_id);
		if (err) {
			LOG_ERR("task_wdt_feed, error: %d", err);
			SEND_FATAL_ERROR();
			return;
		}

		err = zbus_sub_wait_msg(&ntn, &ntn_state.chan, ntn_state.msg_buf, zbus_wait_ms);
		if (err == -ENOMSG) {
			continue;
		} else if (err) {
			LOG_ERR("zbus_sub_wait_msg, error: %d", err);
			SEND_FATAL_ERROR();
			return;
		}

		err = smf_run_state(SMF_CTX(&ntn_state));
		if (err) {
			LOG_ERR("Failed to run state machine, error: %d", err);
		}
	}
}

K_THREAD_DEFINE(ntn_module_thread_id,
		CONFIG_APP_NTN_THREAD_STACK_SIZE,
		ntn_module_thread, NULL, NULL, NULL,
		K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);
