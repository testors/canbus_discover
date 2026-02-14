/**
 * @file can_discover.c
 * @brief PT-CAN Signal Auto-Discovery — Analysis Logic (Pure C)
 */

#include "can_discover.h"
#include <string.h>
#include <stdlib.h>

/*******************************************************************************
 * Signal Names
 ******************************************************************************/

static const char *s_signal_names[DISC_SIG_COUNT] = {
    "steering", "rpm", "throttle", "brake", "gear", "wheel_speed"
};

const char *disc_signal_name(disc_signal_t sig) {
    if (sig < 0 || sig >= DISC_SIG_COUNT) return "unknown";
    return s_signal_names[sig];
}

/*******************************************************************************
 * Phase Capture
 ******************************************************************************/

void disc_phase_init(disc_phase_t *phase) {
    memset(phase, 0, sizeof(*phase));
}

/**
 * Find or create an ID slot in the phase.
 * Returns index, or -1 if full.
 */
static int find_or_create_id(disc_phase_t *phase, uint32_t can_id, uint8_t dlc) {
    for (int i = 0; i < phase->id_count; i++) {
        if (phase->ids[i].can_id == can_id) return i;
    }
    if (phase->id_count >= DISC_MAX_IDS) return -1;

    int idx = phase->id_count++;
    disc_id_t *id = &phase->ids[idx];
    id->can_id = can_id;
    id->dlc = dlc;
    id->frame_count = 0;
    for (int b = 0; b < DISC_MAX_DLC; b++) {
        id->bytes[b].min_val = 0xFF;
        id->bytes[b].max_val = 0x00;
        id->bytes[b].unique_count = 0;
        memset(id->bytes[b].seen, 0, sizeof(id->bytes[b].seen));
    }
    return idx;
}

void disc_phase_add_frame(disc_phase_t *phase, uint32_t can_id,
                           const uint8_t *data, uint8_t dlc) {
    if (dlc > DISC_MAX_DLC) dlc = DISC_MAX_DLC;

    int idx = find_or_create_id(phase, can_id, dlc);
    if (idx < 0) return;

    disc_id_t *id = &phase->ids[idx];
    id->frame_count++;
    phase->total_frames++;

    for (int b = 0; b < dlc; b++) {
        disc_byte_t *bs = &id->bytes[b];
        uint8_t val = data[b];
        if (val < bs->min_val) bs->min_val = val;
        if (val > bs->max_val) bs->max_val = val;
        if (!bs->seen[val]) {
            bs->seen[val] = true;
            bs->unique_count++;
        }
    }
}

void disc_phase_finalize(disc_phase_t *phase, double duration_s) {
    phase->duration_s = duration_s;
}

/*******************************************************************************
 * Analysis Internals
 ******************************************************************************/

/** Per-byte comparison result */
typedef struct {
    int     byte_idx;
    double  score;              /* change score for this byte */
    bool    is_counter;         /* both phases have unique > 15 */
    bool    is_constant;        /* both phases have range == 0 */
    int     active_range;
    int     baseline_range;
    int     active_unique;
    int     baseline_unique;
} byte_cmp_t;

/** Per-CAN-ID comparison result */
typedef struct {
    uint32_t    can_id;
    uint8_t     dlc;
    double      hz;             /* frames / duration in active phase */
    double      total_score;    /* sum of non-ignored byte scores */
    int         changed_bytes;  /* count of bytes with score > 0 */
    byte_cmp_t  bytes[DISC_MAX_DLC];
    int         byte_count;
    int         best_byte;      /* byte index with highest score */
    int         second_byte;    /* adjacent byte with next-highest score */
} id_cmp_t;

/**
 * Compare one CAN ID between baseline and active phases.
 * Returns false if ID not found in both phases.
 */
static bool compare_id(const disc_phase_t *baseline, const disc_phase_t *active,
                         uint32_t can_id, id_cmp_t *out) {
    /* Find ID in both phases */
    const disc_id_t *base_id = NULL;
    const disc_id_t *act_id = NULL;

    for (int i = 0; i < baseline->id_count; i++) {
        if (baseline->ids[i].can_id == can_id) { base_id = &baseline->ids[i]; break; }
    }
    for (int i = 0; i < active->id_count; i++) {
        if (active->ids[i].can_id == can_id) { act_id = &active->ids[i]; break; }
    }

    if (!act_id) return false;

    memset(out, 0, sizeof(*out));
    out->can_id = can_id;
    out->dlc = act_id->dlc;
    out->hz = (active->duration_s > 0) ?
              (double)act_id->frame_count / active->duration_s : 0;

    int n = act_id->dlc;
    if (n > DISC_MAX_DLC) n = DISC_MAX_DLC;
    out->byte_count = n;

    double best_score = -1;
    double second_score = -1;
    out->best_byte = -1;
    out->second_byte = -1;

    for (int b = 0; b < n; b++) {
        byte_cmp_t *bc = &out->bytes[b];
        bc->byte_idx = b;

        const disc_byte_t *a = &act_id->bytes[b];
        int a_range = a->max_val - a->min_val;
        int a_unique = a->unique_count;

        int b_range = 0;
        int b_unique = 0;
        if (base_id) {
            const disc_byte_t *bl = &base_id->bytes[b];
            b_range = bl->max_val - bl->min_val;
            b_unique = bl->unique_count;
        }

        bc->active_range = a_range;
        bc->baseline_range = b_range;
        bc->active_unique = a_unique;
        bc->baseline_unique = b_unique;

        /* Filter: counter/CRC — both have many unique values */
        if (a_unique > 15 && b_unique > 15) {
            bc->is_counter = true;
            bc->score = 0;
            continue;
        }

        /* Filter: constant — both have range 0 */
        if (a_range == 0 && b_range == 0) {
            bc->is_constant = true;
            bc->score = 0;
            continue;
        }

        /* Change score */
        double range_delta = (a_range > b_range) ? (a_range - b_range) : 0;
        double unique_delta = (a_unique > b_unique) ? (a_unique - b_unique) * 0.5 : 0;
        bc->score = range_delta + unique_delta;

        if (bc->score > 0) {
            out->changed_bytes++;
            out->total_score += bc->score;

            if (bc->score > best_score) {
                second_score = best_score;
                out->second_byte = out->best_byte;
                best_score = bc->score;
                out->best_byte = b;
            } else if (bc->score > second_score) {
                second_score = bc->score;
                out->second_byte = b;
            }
        }
    }

    return true;
}

/**
 * Compare all CAN IDs between baseline and active phases.
 * Returns number of comparable IDs.
 */
static int compare_phases(const disc_phase_t *baseline, const disc_phase_t *active,
                            id_cmp_t *results, int max_results) {
    int count = 0;

    /* Use active phase IDs as the reference set */
    for (int i = 0; i < active->id_count && count < max_results; i++) {
        id_cmp_t cmp;
        if (compare_id(baseline, active, active->ids[i].can_id, &cmp)) {
            if (cmp.total_score > 0) {
                results[count++] = cmp;
            }
        }
    }

    /* Sort by total_score descending */
    for (int i = 0; i < count - 1; i++) {
        for (int j = i + 1; j < count; j++) {
            if (results[j].total_score > results[i].total_score) {
                id_cmp_t tmp = results[i];
                results[i] = results[j];
                results[j] = tmp;
            }
        }
    }

    return count;
}

/** Check if a CAN ID is in the exclusion list */
static bool is_excluded(uint32_t can_id, const uint32_t *excl, int excl_count) {
    for (int i = 0; i < excl_count; i++) {
        if (excl[i] == can_id) return true;
    }
    return false;
}

/** Fill a candidate from an id_cmp result */
static void fill_candidate(disc_candidate_t *cand, const id_cmp_t *cmp) {
    cand->can_id = cmp->can_id;
    cand->dlc = cmp->dlc;
    cand->hz = cmp->hz;
    cand->byte_idx = cmp->best_byte;
    cand->byte2_idx = cmp->second_byte;
    cand->score = cmp->total_score;
}

/**
 * Find the best non-excluded CAN ID with optional preference criteria.
 * Returns index in results[] or -1.
 */
static int find_best(const id_cmp_t *results, int count,
                       const uint32_t *excl, int excl_count) {
    for (int i = 0; i < count; i++) {
        if (!is_excluded(results[i].can_id, excl, excl_count)) {
            return i;
        }
    }
    return -1;
}

/*******************************************************************************
 * Signal Classification
 ******************************************************************************/

/**
 * Classify steering: highest score in steering phase.
 * Prefer IDs with 2 adjacent changing bytes (16-bit angle).
 */
static bool classify_steering(const id_cmp_t *results, int count,
                                const uint32_t *excl, int excl_count,
                                disc_candidate_t *out) {
    /* First pass: look for 2 adjacent changed bytes (16-bit signal) */
    for (int i = 0; i < count; i++) {
        if (is_excluded(results[i].can_id, excl, excl_count)) continue;

        const id_cmp_t *r = &results[i];
        if (r->best_byte >= 0 && r->second_byte >= 0 &&
            abs(r->best_byte - r->second_byte) == 1) {
            fill_candidate(out, r);
            return true;
        }
    }

    /* Second pass: just take highest score */
    int idx = find_best(results, count, excl, excl_count);
    if (idx >= 0) {
        fill_candidate(out, &results[idx]);
        return true;
    }
    return false;
}

/**
 * Classify throttle + RPM from throttle phase:
 *   - Throttle: fewest changed_bytes or smallest DLC (pedal = simple)
 *   - RPM: highest Hz + multi-byte change (engine = high-frequency)
 */
static void classify_throttle_rpm(const id_cmp_t *results, int count,
                                    const uint32_t *excl, int excl_count,
                                    disc_candidate_t *throttle_out, bool *throttle_found,
                                    disc_candidate_t *rpm_out, bool *rpm_found) {
    *throttle_found = false;
    *rpm_found = false;

    /* Collect top non-excluded candidates */
    int top_idx[8];
    int top_count = 0;
    for (int i = 0; i < count && top_count < 8; i++) {
        if (!is_excluded(results[i].can_id, excl, excl_count)) {
            top_idx[top_count++] = i;
        }
    }

    if (top_count == 0) return;

    if (top_count == 1) {
        /* Only one candidate — assign to RPM (more valuable for motorsport) */
        fill_candidate(rpm_out, &results[top_idx[0]]);
        *rpm_found = true;
        return;
    }

    /* Strategy: separate RPM (high Hz, multi-byte) from throttle (simple) */
    int rpm_idx = -1;
    int throttle_idx = -1;

    /* RPM: prefer highest Hz among candidates with multi-byte change */
    double best_hz = -1;
    for (int i = 0; i < top_count; i++) {
        const id_cmp_t *r = &results[top_idx[i]];
        if (r->changed_bytes >= 2 && r->hz > best_hz) {
            best_hz = r->hz;
            rpm_idx = i;
        }
    }

    /* If no multi-byte, just pick highest Hz */
    if (rpm_idx < 0) {
        best_hz = -1;
        for (int i = 0; i < top_count; i++) {
            if (results[top_idx[i]].hz > best_hz) {
                best_hz = results[top_idx[i]].hz;
                rpm_idx = i;
            }
        }
    }

    /* Throttle: among remaining, pick smallest DLC or fewest changed_bytes */
    int best_simplicity = 9999;
    for (int i = 0; i < top_count; i++) {
        if (i == rpm_idx) continue;
        const id_cmp_t *r = &results[top_idx[i]];
        int simplicity = r->changed_bytes * 10 + r->dlc;
        if (simplicity < best_simplicity) {
            best_simplicity = simplicity;
            throttle_idx = i;
        }
    }

    if (rpm_idx >= 0) {
        fill_candidate(rpm_out, &results[top_idx[rpm_idx]]);
        *rpm_found = true;
    }
    if (throttle_idx >= 0) {
        fill_candidate(throttle_out, &results[top_idx[throttle_idx]]);
        *throttle_found = true;
    }
}

/**
 * Classify brake: highest score in brake phase (excluding already-found IDs).
 */
static bool classify_brake(const id_cmp_t *results, int count,
                             const uint32_t *excl, int excl_count,
                             disc_candidate_t *out) {
    int idx = find_best(results, count, excl, excl_count);
    if (idx >= 0) {
        fill_candidate(out, &results[idx]);
        return true;
    }
    return false;
}

/**
 * Classify gear: discrete enum pattern in gear phase.
 *   - Active unique values 3-8 (P/R/N/D/S/M...)
 *   - Baseline unique ≤ 2
 * Falls back to highest score if no enum pattern found.
 */
static bool classify_gear(const id_cmp_t *results, int count,
                            const uint32_t *excl, int excl_count,
                            disc_candidate_t *out) {
    /* First pass: look for discrete enum pattern */
    for (int i = 0; i < count; i++) {
        if (is_excluded(results[i].can_id, excl, excl_count)) continue;

        const id_cmp_t *r = &results[i];
        /* Check each byte for enum-like pattern */
        for (int b = 0; b < r->byte_count; b++) {
            const byte_cmp_t *bc = &r->bytes[b];
            if (bc->is_counter || bc->is_constant) continue;
            if (bc->active_unique >= 3 && bc->active_unique <= 8 &&
                bc->baseline_unique <= 2 && bc->score > 0) {
                fill_candidate(out, r);
                out->byte_idx = b;
                out->byte2_idx = -1;
                return true;
            }
        }
    }

    /* Fallback: highest score */
    int idx = find_best(results, count, excl, excl_count);
    if (idx >= 0) {
        fill_candidate(out, &results[idx]);
        return true;
    }
    return false;
}

/**
 * Classify wheel speed: requires driving phase (stationary baseline = constant).
 *   - 4+ changing bytes at high Hz → 4-wheel speed in one message
 *   - 2+ changing bytes at high Hz → 2-wheel per message
 *   - Falls back to highest score
 */
static bool classify_wheel_speed(const id_cmp_t *results, int count,
                                   const uint32_t *excl, int excl_count,
                                   disc_candidate_t *out) {
    /* 1st pass: 4+ bytes changing + high Hz (4-wheel pattern) */
    for (int i = 0; i < count; i++) {
        if (is_excluded(results[i].can_id, excl, excl_count)) continue;
        const id_cmp_t *r = &results[i];
        if (r->changed_bytes >= 4 && r->hz >= 30.0) {
            fill_candidate(out, r);
            return true;
        }
    }

    /* 2nd pass: 2+ bytes changing + high Hz (2-wheel per message) */
    for (int i = 0; i < count; i++) {
        if (is_excluded(results[i].can_id, excl, excl_count)) continue;
        const id_cmp_t *r = &results[i];
        if (r->changed_bytes >= 2 && r->hz >= 30.0) {
            fill_candidate(out, r);
            return true;
        }
    }

    /* Fallback: highest score */
    int idx = find_best(results, count, excl, excl_count);
    if (idx >= 0) {
        fill_candidate(out, &results[idx]);
        return true;
    }
    return false;
}

/*******************************************************************************
 * Main Analysis
 ******************************************************************************/

void disc_analyze(const disc_phase_t phases[DISC_NUM_PHASES],
                   disc_result_t *result) {
    memset(result, 0, sizeof(*result));
    for (int i = 0; i < DISC_SIG_COUNT; i++) {
        result->signals[i].byte_idx = -1;
        result->signals[i].byte2_idx = -1;
    }

    const disc_phase_t *baseline = &phases[DISC_PHASE_BASELINE];

    /* Exclusion list — grows as signals are identified */
    uint32_t excl[DISC_SIG_COUNT];
    int excl_count = 0;

    id_cmp_t cmp_buf[DISC_MAX_IDS];

    /* 1. Steering (from steering phase) */
    {
        int n = compare_phases(baseline, &phases[DISC_PHASE_STEERING],
                                cmp_buf, DISC_MAX_IDS);
        if (classify_steering(cmp_buf, n, excl, excl_count,
                               &result->signals[DISC_SIG_STEERING])) {
            result->found[DISC_SIG_STEERING] = true;
            excl[excl_count++] = result->signals[DISC_SIG_STEERING].can_id;
        }
    }

    /* 2. Throttle + RPM (from throttle phase) */
    {
        int n = compare_phases(baseline, &phases[DISC_PHASE_THROTTLE],
                                cmp_buf, DISC_MAX_IDS);
        classify_throttle_rpm(cmp_buf, n, excl, excl_count,
                               &result->signals[DISC_SIG_THROTTLE],
                               &result->found[DISC_SIG_THROTTLE],
                               &result->signals[DISC_SIG_RPM],
                               &result->found[DISC_SIG_RPM]);

        if (result->found[DISC_SIG_RPM])
            excl[excl_count++] = result->signals[DISC_SIG_RPM].can_id;
        if (result->found[DISC_SIG_THROTTLE])
            excl[excl_count++] = result->signals[DISC_SIG_THROTTLE].can_id;
    }

    /* 3. Brake (from brake phase, excluding already-found) */
    {
        int n = compare_phases(baseline, &phases[DISC_PHASE_BRAKE],
                                cmp_buf, DISC_MAX_IDS);
        if (classify_brake(cmp_buf, n, excl, excl_count,
                            &result->signals[DISC_SIG_BRAKE])) {
            result->found[DISC_SIG_BRAKE] = true;
            excl[excl_count++] = result->signals[DISC_SIG_BRAKE].can_id;
        }
    }

    /* 4. Gear (from gear phase, excluding already-found) */
    {
        int n = compare_phases(baseline, &phases[DISC_PHASE_GEAR],
                                cmp_buf, DISC_MAX_IDS);
        if (classify_gear(cmp_buf, n, excl, excl_count,
                           &result->signals[DISC_SIG_GEAR])) {
            result->found[DISC_SIG_GEAR] = true;
            excl[excl_count++] = result->signals[DISC_SIG_GEAR].can_id;
        }
    }

    /* 5. Wheel speed (from wheel_speed phase, excluding already-found) */
    {
        int n = compare_phases(baseline, &phases[DISC_PHASE_WHEEL_SPEED],
                                cmp_buf, DISC_MAX_IDS);
        if (classify_wheel_speed(cmp_buf, n, excl, excl_count,
                                  &result->signals[DISC_SIG_WHEEL_SPEED])) {
            result->found[DISC_SIG_WHEEL_SPEED] = true;
        }
    }
}
