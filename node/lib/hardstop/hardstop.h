/*
 * NightShift watchman node: hard-stop rules.  SAFETY-RELEVANT: changes need plan mode
 * and human review (see CLAUDE.md).
 *
 * Portable C99, no dynamic allocation, float32 only. Shared by the ESP32-S3 firmware
 * and the host parity tests. The Python reference (cell/watchman/hardstop.py) must make
 * IDENTICAL decisions on the same inputs: change both together and keep the parity
 * suite (tests/hardstop/) green.
 *
 * Build with: -std=c99 -ffp-contract=off  (no fused multiply-add: it changes rounding).
 * Time is uint32 milliseconds; elapsed time uses unsigned subtraction, so the ~49.7-day
 * counter wraparound is handled.
 */
#ifndef NIGHTSHIFT_HARDSTOP_H
#define NIGHTSHIFT_HARDSTOP_H

#include <stdint.h>

/* Event bits returned by hs_step(). OVERLOAD, TOOL_BREAK and SENSOR_FAULT are levels:
 * set on every step while the confirmed condition holds. CUT_START / CUT_END are edges. */
#define HS_CUT_START    (1u << 0)
#define HS_CUT_END      (1u << 1)
#define HS_OVERLOAD     (1u << 2)
#define HS_TOOL_BREAK   (1u << 3)
#define HS_SENSOR_FAULT (1u << 4)

typedef struct {
    float cut_on_a;              /* cut starts: current > this for cut_on_ms */
    float cut_off_a;             /* cut ends: current < this for cut_off_ms (< cut_on_a) */
    uint32_t cut_on_ms;
    uint32_t cut_off_ms;
    uint32_t settle_ms;          /* ignore tool break this long after a cut starts */
    float break_ratio;           /* tool break: current < ratio x settled cut mean ... */
    uint32_t break_confirm_ms;   /* ... held this long */
    uint32_t min_settled;        /* settled samples needed before tool break is judged */
    float overload_a;            /* overload: current > this or vib > vib_max_g ... */
    float vib_max_g;
    uint32_t overload_confirm_ms; /* ... held this long, while cutting */
    float vib_floor_g;           /* sensor fault: vib < floor while cutting, or ADC */
    uint32_t sensor_fault_ms;    /* clipped, held this long */
} hs_config_t;

typedef struct {
    hs_config_t cfg;
    uint8_t cutting;
    uint8_t on_pending, off_pending, break_pending, over_pending, fault_pending;
    uint32_t on_since, off_since, break_since, over_since, fault_since;
    uint32_t cut_start;
    float settled_sum;
    uint32_t settled_n;
} hs_state_t;

void hs_init(hs_state_t *st, const hs_config_t *cfg);

/*
 * One step (the node calls this every 100 ms with fresh features).
 *   current_a, vib_g : RMS features; NaN, inf or negative -> HS_SENSOR_FAULT at once,
 *                      and no other rule runs on that step.
 *   clipped          : nonzero if the current ADC saturated in this window.
 *   cutting_hint     : -1 = detect cutting from current (standalone node, no CNC I/O);
 *                      0 / 1 = the machine says whether it is cutting.
 */
uint32_t hs_step(hs_state_t *st, uint32_t t_ms, float current_a, float vib_g,
                 int clipped, int cutting_hint);

/* Mean of this cut's settled current (0 if none yet), for alert text. */
float hs_cut_mean(const hs_state_t *st);

/*
 * RMS of raw ADC samples with the DC offset removed, times `scale` (units per count).
 * Sequential float32 sums (the Python reference uses the same order). *clipped is set
 * to the number of samples at the int16 rails.
 */
float hs_rms(const int16_t *samples, uint32_t n, float scale, uint32_t *clipped);

/* sizeof(hs_state_t), for the host test harness. */
uint32_t hs_state_size(void);

#endif
