/*
 * airspyhf_stream_break_test
 *
 * Isolated diagnostic tool (NOT wired into the production driver). Opens a
 * device, starts streaming, and mid-stream fires the same kind of control
 * transfer(s) SDR# issues when the user drags the frequency or sample-rate
 * widget while the radio is running -- then watches the sample callback to
 * see whether the stream keeps flowing or the hardware wedges.
 *
 * This exists because the wedge reported on some Discovery units only
 * reproduces while a real bulk-IN stream is in flight; testing set_freq()/
 * set_samplerate() against an idle device does not reproduce it.
 *
 * Usage:
 *   airspyhf_stream_break_test [-s serial] [-f start_MHz] [-a start_samplerate]
 *                              [-F new_MHz] [-A new_samplerate]
 *                              [-T trigger_after_s] [-W watch_after_s]
 *                              [-x] [-d]
 *
 *   -F / -A select which control transfer(s) to fire mid-stream. Give either,
 *   both (in that order: freq then samplerate), or neither (baseline run,
 *   useful to confirm the stream is healthy before adding a stimulus).
 *   -x additionally brackets the change with stop()/start(), mimicking a
 *   client that pauses streaming before reconfiguring.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <signal.h>

#include <airspyhf.h>

#ifdef _WIN32
#include <windows.h>
#ifdef _MSC_VER
typedef int64_t ssize_t_compat;
#define strtoull _strtoui64
int gettimeofday(struct timeval *tv, void *ignored)
{
	FILETIME ft;
	unsigned __int64 tmp = 0;
	if (tv) {
		GetSystemTimeAsFileTime(&ft);
		tmp |= ft.dwHighDateTime; tmp <<= 32; tmp |= ft.dwLowDateTime;
		tmp /= 10; tmp -= 11644473600000000Ui64;
		tv->tv_sec = (long)(tmp / 1000000UL);
		tv->tv_usec = (long)(tmp % 1000000UL);
	}
	return 0;
}
#endif
#define sleep_ms(a) Sleep(a)
#else
#include <unistd.h>
#include <sys/time.h>
#define sleep_ms(a) usleep((a) * 1000)
#endif

static double now_s(void)
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (double)tv.tv_sec + 1e-6 * (double)tv.tv_usec;
}

/* ------------------------------------------------------------------ */
/* shared state between the callback thread and main                  */
/* ------------------------------------------------------------------ */

static volatile int      do_exit = 0;
static volatile uint64_t total_callbacks = 0;
static volatile uint64_t total_samples = 0;
static volatile double   last_callback_at = 0.0;
static volatile uint64_t total_dropped = 0;

static int rx_callback(airspyhf_transfer_t *transfer)
{
	total_callbacks++;
	total_samples += (uint64_t)transfer->sample_count;
	total_dropped += transfer->dropped_samples;
	last_callback_at = now_s();
	return 0;
}

static void on_sigint(int signum)
{
	(void)signum;
	do_exit = 1;
}

static int parse_u64(const char *s, uint64_t *value)
{
	char *end;
	uint64_t v = strtoull(s, &end, (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) ? 16 : 10);
	if (end == s || *end != 0) return -1;
	*value = v;
	return 0;
}

static void usage(void)
{
	fprintf(stderr,
		"airspyhf_stream_break_test - break a live stream mid-flight and watch what happens\n\n"
		"  -s <serial>        open device with this 64-bit serial (default: first found)\n"
		"  -f <MHz>           starting frequency (default 10.0 MHz)\n"
		"  -a <Hz>            starting sample rate (default: first supported rate)\n"
		"  -F <MHz>           frequency to switch to mid-stream (triggers set_freq)\n"
		"  -A <Hz>            sample rate to switch to mid-stream (triggers set_samplerate)\n"
		"  -T <seconds>       stream this long before firing the change (default 5)\n"
		"  -W <seconds>       keep watching this long after the change (default 15)\n"
		"  -x                 bracket the change with stop()/start() (mimic a client that\n"
		"                     pauses streaming before reconfiguring)\n"
		"  -d                 verbose\n");
}

int main(int argc, char **argv)
{
	int opt;
	int have_serial = 0;
	uint64_t serial_val = 0;
	double start_mhz = 10.0;
	uint32_t start_sr = 0;
	int have_new_freq = 0;
	double new_mhz = 0.0;
	int have_new_sr = 0;
	uint32_t new_sr = 0;
	double trigger_after = 5.0;
	double watch_after = 15.0;
	int bracket_stop_start = 0;
	int verbose = 0;

	while ((opt = getopt(argc, argv, "s:f:a:F:A:T:W:xdh")) != -1) {
		switch (opt) {
		case 's': have_serial = 1; if (parse_u64(optarg, &serial_val)) { usage(); return EXIT_FAILURE; } break;
		case 'f': start_mhz = atof(optarg); break;
		case 'a': start_sr = (uint32_t)strtoul(optarg, NULL, 10); break;
		case 'F': have_new_freq = 1; new_mhz = atof(optarg); break;
		case 'A': have_new_sr = 1; new_sr = (uint32_t)strtoul(optarg, NULL, 10); break;
		case 'T': trigger_after = atof(optarg); break;
		case 'W': watch_after = atof(optarg); break;
		case 'x': bracket_stop_start = 1; break;
		case 'd': verbose = 1; break;
		case 'h':
		default: usage(); return opt == 'h' ? EXIT_SUCCESS : EXIT_FAILURE;
		}
	}

	struct airspyhf_device *device = NULL;
	int ret;

	if (have_serial) {
		ret = airspyhf_open_sn(&device, serial_val);
	} else {
		ret = airspyhf_open(&device);
	}
	if (ret != AIRSPYHF_SUCCESS || !device) {
		fprintf(stderr, "error: could not open device (ret=%d)\n", ret);
		return EXIT_FAILURE;
	}

	{
		airspyhf_read_partid_serialno_t id;
		memset(&id, 0, sizeof(id));
		if (airspyhf_board_partid_serialno_read(device, &id) == AIRSPYHF_SUCCESS) {
			fprintf(stderr, "device: part_id=0x%08X serial=0x%08X%08X\n",
				id.part_id, id.serial_no[2], id.serial_no[3]);
		}
	}

	uint32_t nsrates = 0;
	airspyhf_get_samplerates(device, &nsrates, 0);
	if (nsrates == 0) {
		fprintf(stderr, "error: device reports no supported sample rates\n");
		airspyhf_close(device);
		return EXIT_FAILURE;
	}
	uint32_t *rates = (uint32_t *)calloc(nsrates, sizeof(uint32_t));
	airspyhf_get_samplerates(device, rates, nsrates);

	if (start_sr == 0) start_sr = rates[0];
	int start_sr_ok = 0;
	for (uint32_t i = 0; i < nsrates; i++) if (rates[i] == start_sr) start_sr_ok = 1;
	if (have_new_sr) {
		int new_sr_ok = 0;
		for (uint32_t i = 0; i < nsrates; i++) if (rates[i] == new_sr) new_sr_ok = 1;
		if (!new_sr_ok) {
			fprintf(stderr, "warning: -A %u is not in the device's supported rate list, sending it anyway\n", new_sr);
		}
	}
	free(rates);

	if (!start_sr_ok) {
		fprintf(stderr, "warning: -a %u is not in the device's supported rate list, sending it anyway\n", start_sr);
	}

	if ((ret = airspyhf_set_samplerate(device, start_sr)) != AIRSPYHF_SUCCESS) {
		fprintf(stderr, "error: airspyhf_set_samplerate(%u) failed: %d\n", start_sr, ret);
		airspyhf_close(device);
		return EXIT_FAILURE;
	}
	fprintf(stderr, "starting sample rate: %u Hz\n", start_sr);

#ifdef _MSC_VER
	SetConsoleCtrlHandler(NULL, TRUE);
#else
	signal(SIGINT, on_sigint);
#endif

	if ((ret = airspyhf_start(device, rx_callback, NULL)) != AIRSPYHF_SUCCESS) {
		fprintf(stderr, "error: airspyhf_start() failed: %d\n", ret);
		airspyhf_close(device);
		return EXIT_FAILURE;
	}

	uint32_t start_hz = (uint32_t)(start_mhz * 1000000.0);
	if ((ret = airspyhf_set_freq(device, start_hz)) != AIRSPYHF_SUCCESS) {
		fprintf(stderr, "error: airspyhf_set_freq(%u) failed: %d\n", start_hz, ret);
	}
	fprintf(stderr, "starting frequency: %.6f MHz\n", start_mhz);

	double t_start = now_s();
	double t_trigger_target = t_start + trigger_after;
	int triggered = 0;
	double t_triggered_at = 0.0;
	uint64_t callbacks_before_trigger = 0;

	fprintf(stderr, "streaming... will trigger change at t=+%.1fs, then watch for %.1fs\n",
		trigger_after, watch_after);

	double last_report = 0.0;
	uint64_t last_report_callbacks = 0;

	while (!do_exit) {
		double t = now_s();
		double elapsed = t - t_start;

		if (!triggered && t >= t_trigger_target) {
			callbacks_before_trigger = total_callbacks;
			fprintf(stderr, "\n[t=%.2fs] TRIGGER: %llu callbacks so far, %llu samples, last callback %.3fs ago\n",
				elapsed, (unsigned long long)total_callbacks,
				(unsigned long long)total_samples, t - last_callback_at);

			if (bracket_stop_start) {
				double c0 = now_s();
				int r = airspyhf_stop(device);
				fprintf(stderr, "[t=%.2fs] airspyhf_stop() -> %d (%.1fms)\n", now_s() - t_start, r, (now_s() - c0) * 1000.0);
			}
			if (have_new_freq) {
				uint32_t hz = (uint32_t)(new_mhz * 1000000.0);
				double c0 = now_s();
				int r = airspyhf_set_freq(device, hz);
				fprintf(stderr, "[t=%.2fs] airspyhf_set_freq(%u) -> %d (%.1fms)\n",
					now_s() - t_start, hz, r, (now_s() - c0) * 1000.0);
			}
			if (have_new_sr) {
				double c0 = now_s();
				int r = airspyhf_set_samplerate(device, new_sr);
				fprintf(stderr, "[t=%.2fs] airspyhf_set_samplerate(%u) -> %d (%.1fms)\n",
					now_s() - t_start, new_sr, r, (now_s() - c0) * 1000.0);
			}
			if (bracket_stop_start) {
				double c0 = now_s();
				int r = airspyhf_start(device, rx_callback, NULL);
				fprintf(stderr, "[t=%.2fs] airspyhf_start() -> %d (%.1fms)\n", now_s() - t_start, r, (now_s() - c0) * 1000.0);
			}

			triggered = 1;
			t_triggered_at = now_s();
			t_trigger_target = t_triggered_at + watch_after; /* reuse as end-of-watch marker */
		}

		if (triggered && t >= t_trigger_target) {
			break;
		}

		if (t - last_report >= 1.0) {
			uint64_t cb = total_callbacks;
			double gap = t - last_callback_at;
			fprintf(stderr, "[t=%6.2fs] callbacks=%llu (+%llu/s) samples=%llu dropped=%llu last_cb=%.3fs ago is_streaming=%d%s\n",
				elapsed, (unsigned long long)cb, (unsigned long long)(cb - last_report_callbacks),
				(unsigned long long)total_samples, (unsigned long long)total_dropped,
				gap, airspyhf_is_streaming(device),
				(gap > 1.0) ? "  <-- STALLED" : "");
			last_report = t;
			last_report_callbacks = cb;
		}

		sleep_ms(50);
	}

	double t_end = now_s();
	int is_streaming_final = airspyhf_is_streaming(device);
	double final_gap = t_end - last_callback_at;

	fprintf(stderr, "\n===== summary =====\n");
	fprintf(stderr, "total runtime:            %.2fs\n", t_end - t_start);
	fprintf(stderr, "total callbacks:          %llu\n", (unsigned long long)total_callbacks);
	fprintf(stderr, "total samples:            %llu\n", (unsigned long long)total_samples);
	fprintf(stderr, "total dropped:            %llu\n", (unsigned long long)total_dropped);
	if (triggered) {
		uint64_t after = total_callbacks - callbacks_before_trigger;
		fprintf(stderr, "callbacks after trigger:  %llu (over %.2fs watch window)\n", (unsigned long long)after, watch_after);
		if (after == 0) {
			fprintf(stderr, "RESULT: WEDGED -- zero callbacks fired after the mid-stream change.\n");
		} else {
			fprintf(stderr, "RESULT: stream continued after the mid-stream change.\n");
		}
	} else {
		fprintf(stderr, "RESULT: baseline run, no change was triggered (-F/-A not given).\n");
	}
	fprintf(stderr, "airspyhf_is_streaming() at exit: %d\n", is_streaming_final);
	fprintf(stderr, "time since last callback at exit: %.3fs\n", final_gap);
	if (verbose) {
		fprintf(stderr, "(note: is_streaming() reflects the driver's internal state, not whether\n"
			" the hardware is actually delivering data -- a wedge that the driver can't\n"
			" detect will still report is_streaming()=1 with no callbacks arriving.)\n");
	}

	airspyhf_stop(device);
	airspyhf_close(device);
	return (triggered && (total_callbacks - callbacks_before_trigger) == 0) ? 2 : EXIT_SUCCESS;
}
