/* See hardstop.h. Keep line-for-line in step with cell/watchman/hardstop.py. */
#include "hardstop.h"

#include <math.h>

static uint32_t elapsed(uint32_t now, uint32_t since) { return now - since; }

void hs_init(hs_state_t *st, const hs_config_t *cfg) {
    static const hs_state_t zero; /* static storage: all fields zero */
    *st = zero;
    st->cfg = *cfg;
}

uint32_t hs_state_size(void) { return (uint32_t)sizeof(hs_state_t); }

float hs_cut_mean(const hs_state_t *st) {
    if (st->settled_n == 0) return 0.0f;
    return st->settled_sum / (float)st->settled_n;
}

static void new_cut(hs_state_t *st, uint32_t t_ms) {
    st->cut_start = t_ms;
    st->settled_sum = 0.0f;
    st->settled_n = 0;
    st->break_pending = 0;
    st->over_pending = 0;
}

/* Condition held for at least `window` ms? Starts the timer on the first true step. */
static int held(int cond, uint8_t *pending, uint32_t *since, uint32_t t_ms, uint32_t window) {
    if (!cond) {
        *pending = 0;
        return 0;
    }
    if (!*pending) {
        *pending = 1;
        *since = t_ms;
    }
    return elapsed(t_ms, *since) >= window;
}

uint32_t hs_step(hs_state_t *st, uint32_t t_ms, float current_a, float vib_g,
                 int clipped, int cutting_hint) {
    const hs_config_t *c = &st->cfg;
    uint32_t ev = 0;

    /* Invalid input never passes silently. */
    if (!isfinite(current_a) || !isfinite(vib_g) || current_a < 0.0f || vib_g < 0.0f) {
        return HS_SENSOR_FAULT;
    }

    /* --- cut detection --- */
    uint8_t was = st->cutting;
    uint8_t now_cut;
    if (cutting_hint >= 0) {
        now_cut = cutting_hint ? 1 : 0;
        st->on_pending = 0;
        st->off_pending = 0;
    } else if (!was) {
        st->off_pending = 0;
        now_cut = (uint8_t)held(current_a > c->cut_on_a, &st->on_pending, &st->on_since,
                                t_ms, c->cut_on_ms);
    } else {
        st->on_pending = 0;
        now_cut = (uint8_t)!held(current_a < c->cut_off_a, &st->off_pending, &st->off_since,
                                 t_ms, c->cut_off_ms);
    }
    if (now_cut && !was) {
        ev |= HS_CUT_START;
        new_cut(st, t_ms);
    }
    if (!now_cut && was) ev |= HS_CUT_END;
    st->cutting = now_cut;

    /* --- sensor fault (ADC clipping any time; vibration flat while cutting) --- */
    int fault = clipped != 0 || (now_cut && vib_g < c->vib_floor_g);
    if (held(fault, &st->fault_pending, &st->fault_since, t_ms, c->sensor_fault_ms)) {
        ev |= HS_SENSOR_FAULT;
    }

    if (!now_cut) {
        st->over_pending = 0;
        st->break_pending = 0;
        return ev;
    }

    /* --- overload: current or vibration above limit, held --- */
    int over = current_a > c->overload_a || vib_g > c->vib_max_g;
    if (held(over, &st->over_pending, &st->over_since, t_ms, c->overload_confirm_ms)) {
        ev |= HS_OVERLOAD;
    }

    /* Standalone (no CNC cut signal): a load collapse alone is NOT a tool break: a
     * normal retract looks identical in the current. The hub judges collapses against
     * the program + tool reference instead. */
    if (cutting_hint < 0) return ev;

    if (elapsed(t_ms, st->cut_start) < c->settle_ms) return ev;

    /* --- tool break: current collapses vs. this cut's settled mean, held --- */
    int below = 0;
    if (st->settled_n >= c->min_settled) {
        float mean = st->settled_sum / (float)st->settled_n;
        float limit = c->break_ratio * mean;
        below = current_a < limit;
    }
    if (held(below, &st->break_pending, &st->break_since, t_ms, c->break_confirm_ms)) {
        ev |= HS_TOOL_BREAK;
    }
    if (!below && !over) {
        st->settled_sum = st->settled_sum + current_a;
        st->settled_n = st->settled_n + 1;
    }
    return ev;
}

float hs_rms(const int16_t *samples, uint32_t n, float scale, uint32_t *clipped) {
    uint32_t i;
    float sum = 0.0f;
    float acc = 0.0f;
    uint32_t clip = 0;
    if (n == 0) {
        if (clipped) *clipped = 0;
        return 0.0f;
    }
    for (i = 0; i < n; i++) {
        sum = sum + (float)samples[i];
        if (samples[i] == INT16_MAX || samples[i] == INT16_MIN) clip++;
    }
    float mean = sum / (float)n;
    for (i = 0; i < n; i++) {
        float d = (float)samples[i] - mean;
        acc = acc + d * d;
    }
    if (clipped) *clipped = clip;
    return sqrtf(acc / (float)n) * scale;
}
