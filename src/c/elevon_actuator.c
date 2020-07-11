#include "elevon_actuator.h"
#include <math.h>

void elevon_actuator_init(
    elevon_actuator_state_t *act,
    double max_slew_deg_s,
    double time_constant_s,
    double min_pos_deg,
    double max_pos_deg
) {
    if (!act) return;
    act->current_pos_deg = 0.0;
    act->target_pos_deg = 0.0;
    act->max_slew_deg_s = max_slew_deg_s;
    act->time_constant_s = time_constant_s;
    act->min_pos_deg = min_pos_deg;
    act->max_pos_deg = max_pos_deg;
    act->stalled = false;
    act->tripped_ofd = false;
}

double elevon_actuator_step(
    elevon_actuator_state_t *act,
    double target_deg,
    double hydraulic_press_psi,
    double dt_sec
) {
    if (!act) return 0.0;

    if (target_deg > act->max_pos_deg) target_deg = act->max_pos_deg;
    if (target_deg < act->min_pos_deg) target_deg = act->min_pos_deg;
    act->target_pos_deg = target_deg;

    if (hydraulic_press_psi < 1500.0 || act->stalled || act->tripped_ofd) {
        return act->current_pos_deg;
    }

    double pressure_scale = (hydraulic_press_psi >= 3000.0) ? 1.0 : (hydraulic_press_psi / 3000.0);
    double effective_slew = act->max_slew_deg_s * pressure_scale;

    double error = act->target_pos_deg - act->current_pos_deg;
    double tc = (act->time_constant_s > 0.001) ? act->time_constant_s : 0.04;
    double rate = error / tc;

    if (rate > effective_slew) rate = effective_slew;
    if (rate < -effective_slew) rate = -effective_slew;

    act->current_pos_deg += rate * dt_sec;

    if (act->current_pos_deg > act->max_pos_deg) act->current_pos_deg = act->max_pos_deg;
    if (act->current_pos_deg < act->min_pos_deg) act->current_pos_deg = act->min_pos_deg;

    return act->current_pos_deg;
}

void elevon_hydraulic_step(
    elevon_hydraulic_status_t *hyd,
    bool engines_running,
    bool rat_deployed,
    double dt_sec
) {
    if (!hyd) return;

    if (engines_running) {
        // Normal 3000 psi system pressure
        hyd->green_press_psi = 3000.0;
        hyd->blue_press_psi = 3000.0;
        hyd->yellow_press_psi = 3000.0;
    } else {
        // Accumulator pressure bleed down
        double decay = 150.0 * dt_sec;
        if (hyd->green_press_psi > 0.0) hyd->green_press_psi -= decay;
        if (hyd->green_press_psi < 0.0) hyd->green_press_psi = 0.0;

        if (hyd->yellow_press_psi > 0.0) hyd->yellow_press_psi -= decay;
        if (hyd->yellow_press_psi < 0.0) hyd->yellow_press_psi = 0.0;

        if (rat_deployed) {
            // Ram Air Turbine pressurizes Blue circuit to 2500 psi
            hyd->blue_press_psi = 2500.0;
        } else {
            if (hyd->blue_press_psi > 0.0) hyd->blue_press_psi -= decay;
            if (hyd->blue_press_psi < 0.0) hyd->blue_press_psi = 0.0;
        }
    }

    hyd->green_avail = (hyd->green_press_psi >= 2200.0);
    hyd->blue_avail = (hyd->blue_press_psi >= 2200.0);
    hyd->yellow_avail = (hyd->yellow_press_psi >= 2200.0);
}

bool elevon_ofd_check(
    double current_pos_deg,
    uint64_t timestamp_us,
    double *prev_pos,
    uint64_t *last_crossing_us,
    int *osc_count,
    double *peak_pos,
    double *peak_neg
) {
    if (!prev_pos || !last_crossing_us || !osc_count || !peak_pos || !peak_neg) {
        return false;
    }

    if (current_pos_deg > *peak_pos) *peak_pos = current_pos_deg;
    if (current_pos_deg < *peak_neg) *peak_neg = current_pos_deg;

    bool crossing = ((*prev_pos) * current_pos_deg) < 0.0;
    if (crossing) {
        uint64_t half_period = timestamp_us - (*last_crossing_us);
        *last_crossing_us = timestamp_us;

        // Band: 2.5 Hz to 12.5 Hz
        if (half_period >= 40000 && half_period <= 200000) {
            double p2p = (*peak_pos) - (*peak_neg);
            if (p2p >= 1.75) {
                (*osc_count)++;
                if (*osc_count >= 3) {
                    return true;
                }
            }
        } else {
            if (*osc_count > 0) (*osc_count)--;
        }

        *peak_pos = 0.0;
        *peak_neg = 0.0;
    }

    *prev_pos = current_pos_deg;
    return false;
}
