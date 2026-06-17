/*
 * Active-LED marker generator for the evk4_sdk_advanced `led_tracking` pipeline.
 *
 * Blinks an LED on a Raspberry Pi GPIO with the Metavision modulated-light
 * encoding, so the EVK4 + led_tracking can decode its ID and track it. Lets you
 * validate led_tracking with nothing but an LED + resistor (no commercial marker).
 *
 * ENCODING (matches ModulatedLightDetectorAlgorithm): each blink is an LED rising
 * edge; the gap between consecutive rising edges, in multiples of a base period p,
 * encodes a symbol:  2p = bit 0,  3p = bit 1,  4p = start/sync.  An ID is `num_bits`
 * bits sent LSB-first, framed by start symbols. OFF edges and sub-period noise are
 * ignored by the detector.
 *
 * TIMING: a real marker uses p = 200 us. Linux is not real-time, so for a
 * Pi-driven LED use a LARGER base (e.g. 5000 us) and set the node's
 * `base_period_us` to match -- the encoding re-syncs on every start symbol, so the
 * occasional scheduler hiccup just drops one word. This loop pins itself to a core
 * and busy-waits; run it on a Pi 5 (RP1 = /dev/gpiochip4).
 *
 *   gcc -O2 -o led_marker led_marker.c
 *   ./led_marker [id=146] [base_us=5000] [gpio_line=17]
 *
 * Wiring: GPIO17 (header pin 11) -> 220-330 ohm -> LED(+) ; LED(-) -> GND (pin 9).
 * Access: the SSH user must reach /dev/gpiochip4 (e.g. `sudo usermod -aG dialout $USER`).
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <sched.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <linux/gpio.h>

static volatile int run = 1;
static void onsig(int s) { (void)s; run = 0; }
static inline long nowns(void) {
  struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
  return t.tv_sec * 1000000000L + t.tv_nsec;
}

int main(int argc, char** argv) {
  const int id        = (argc > 1) ? atoi(argv[1]) : 146;
  const long base_us  = (argc > 2) ? atol(argv[2]) : 5000;
  const unsigned line_off = (argc > 3) ? (unsigned)atoi(argv[3]) : 17;
  const int num_bits  = 8;
  const long base_ns  = base_us * 1000L;
  const long flash_ns = base_ns / 4;  // LED on-time per blink (< smallest gap)

  signal(SIGINT, onsig); signal(SIGTERM, onsig);
  cpu_set_t set; CPU_ZERO(&set); CPU_SET(3, &set); sched_setaffinity(0, sizeof(set), &set);

  int gaps[9];  // 8 data symbols (LSB-first) + 1 start delimiter
  for (int b = 0; b < num_bits; ++b) gaps[b] = ((id >> b) & 1) ? 3 : 2;
  gaps[8] = 4;

  int chip = open("/dev/gpiochip4", O_RDWR);
  if (chip < 0) { perror("open /dev/gpiochip4"); return 1; }
  struct gpio_v2_line_request req; memset(&req, 0, sizeof(req));
  req.num_lines = 1; req.offsets[0] = line_off;
  req.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
  strncpy(req.consumer, "evk4-led-marker", sizeof(req.consumer) - 1);
  if (ioctl(chip, GPIO_V2_GET_LINE_IOCTL, &req) < 0) { perror("GPIO_V2_GET_LINE"); return 1; }
  int led = req.fd;
  struct gpio_v2_line_values hi = {.bits = 1, .mask = 1}, lo = {.bits = 0, .mask = 1};

  printf("marker: GPIO%u id=%d base=%ldus (LSB-first, 2p=0/3p=1/4p=start)\n", line_off, id, base_us);
  fflush(stdout);
  long t = nowns();
  while (run) {
    for (int s = 0; s < 9 && run; ++s) {
      ioctl(led, GPIO_V2_LINE_SET_VALUES_IOCTL, &hi);          // rising edge = a blink
      long off = t + flash_ns; while (nowns() < off) {}
      ioctl(led, GPIO_V2_LINE_SET_VALUES_IOCTL, &lo);
      t += (long)gaps[s] * base_ns; while (nowns() < t) {}     // gap to the next rising edge
    }
  }
  ioctl(led, GPIO_V2_LINE_SET_VALUES_IOCTL, &lo);
  return 0;
}
