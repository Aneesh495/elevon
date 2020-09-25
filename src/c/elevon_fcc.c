#include "elevon.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

struct elevon_fcc {
    uint64_t frame_index;
    uint64_t current_time_us;

    bool lane_healthy[3];
    int consecutive_missed_frames[3];

    elevon_air_data_t air_data;
    elevon_imu_data_t imu_data;
    elevon_surfaces_t surfaces;
    elevon_law_mode_t law_mode;
    uint32_t prot_flags;

    // Longitudinal C* law state
    double c_star_integrator;
    double auto_trim_ths_deg;
    bool trim_frozen;

    // Actuator models
    elevon_actuator_state_t act_elev_in_l;
    elevon_actuator_state_t act_elev_in_r;
    elevon_actuator_state_t act_elev_out_l;
    elevon_actuator_state_t act_elev_out_r;
    elevon_actuator_state_t act_ail_l;
    elevon_actuator_state_t act_ail_r;
    elevon_actuator_state_t act_rudder;

    // AFDX state
    elevon_afdx_vl_t vl_actuators;
    elevon_afdx_redundancy_t red_fcc;

    // Fault injection
    int faulted_probe;
    double probe_fault_pct;
    bool faulted_lane[3];
};

elevon_fcc_t *elevon_fcc_create(void) {
    elevon_fcc_t *fcc = (elevon_fcc_t *)calloc(1, sizeof(elevon_fcc_t));
    if (!fcc) return NULL;

    fcc->lane_healthy[0] = true;
    fcc->lane_healthy[1] = true;
    fcc->lane_healthy[2] = true;
    fcc->law_mode = ELEVON_LAW_NORMAL;
    fcc->prot_flags = ELEVON_PROT_NONE;

    fcc->imu_data.quat = elevon_quat_identity();
    fcc->imu_data.load_factor_nz = 1.0;

    elevon_actuator_init(&fcc->act_elev_in_l, 40.0, 0.04, -30.0, 25.0);
    elevon_actuator_init(&fcc->act_elev_in_r, 40.0, 0.04, -30.0, 25.0);
    elevon_actuator_init(&fcc->act_elev_out_l, 40.0, 0.04, -30.0, 25.0);
    elevon_actuator_init(&fcc->act_elev_out_r, 40.0, 0.04, -30.0, 25.0);
    elevon_actuator_init(&fcc->act_ail_l, 40.0, 0.04, -25.0, 25.0);
    elevon_actuator_init(&fcc->act_ail_r, 40.0, 0.04, -25.0, 25.0);
    elevon_actuator_init(&fcc->act_rudder, 60.0, 0.04, -30.0, 30.0);

    elevon_afdx_vl_init(&fcc->vl_actuators, 101, 10, 256, 400);
    elevon_afdx_redundancy_init(&fcc->red_fcc, 101);

    fcc->faulted_probe = -1;

    return fcc;
}

void elevon_fcc_destroy(elevon_fcc_t *fcc) {
    if (fcc) {
        free(fcc);
    }
}

elevon_status_t elevon_fcc_step(
    elevon_fcc_t *fcc,
    uint64_t actual_time_us,
    const elevon_inceptor_t *inceptor,
    double p_static_pa,
    double p_total_pa,
    double tat_k,
    elevon_vec3_t gyro_rad_s,
    elevon_vec3_t accel_m_s2,
    double dt_sec
) {
    if (!fcc) return ELEVON_ERR_NULL_PTR;

    fcc->current_time_us = actual_time_us;
    fcc->frame_index++;

    // 1. Process Air Data (with fault injection if applicable)
    double effective_pt = p_total_pa;
    if (fcc->faulted_probe == 1) {
        effective_pt *= (1.0 - (fcc->probe_fault_pct * 0.01));
    }
    elevon_adc_process(p_static_pa, effective_pt, tat_k, &fcc->air_data);

    // 2. Process Inertial Measurements
    elevon_imu_step(&fcc->imu_data, gyro_rad_s, accel_m_s2, dt_sec);

    // 3. Evaluate TMR lane faults
    int healthy_lanes = 0;
    for (int i = 0; i < 3; i++) {
        if (!fcc->faulted_lane[i] && fcc->lane_healthy[i]) {
            healthy_lanes++;
        }
    }

    if (healthy_lanes >= 2) {
        fcc->law_mode = ELEVON_LAW_NORMAL;
    } else if (healthy_lanes == 1) {
        fcc->law_mode = ELEVON_LAW_ALTERNATE;
    } else {
        fcc->law_mode = ELEVON_LAW_DIRECT;
    }

    // 4. Longitudinal C* Flight Law
    double pitch_stick = inceptor ? inceptor->pitch_stick_norm : 0.0;
    double roll_stick = inceptor ? inceptor->roll_stick_norm : 0.0;
    double rudder_pedal = inceptor ? inceptor->rudder_pedal_norm : 0.0;

    double c_star_dem = (pitch_stick >= 0.0) ? (pitch_stick * 1.5) : (pitch_stick * 2.0);
    double v_co_mps = 210.0 * 0.51444444;
    double c_star_fb = (fcc->imu_data.load_factor_nz - 1.0) + ((v_co_mps / ELEVON_GRAVITY_G0) * gyro_rad_s.y);
    // Nose-down is positive, nose-up is negative
    double c_star_err = c_star_fb - c_star_dem;

    fcc->c_star_integrator += 3.2 * c_star_err * dt_sec;
    if (fcc->c_star_integrator > 30.0) fcc->c_star_integrator = 30.0;
    if (fcc->c_star_integrator < -30.0) fcc->c_star_integrator = -30.0;

    double elev_cmd = (8.5 * c_star_err) + fcc->c_star_integrator;

    // 5. Envelope Protections
    fcc->prot_flags = ELEVON_PROT_NONE;

    // High-AOA Alpha Protection
    double alpha_deg = 2.5; // Nominal clean AOA
    if (gyro_rad_s.y > 0.1) {
        alpha_deg += gyro_rad_s.y * 10.0;
    }
    if (alpha_deg >= 11.2) {
        fcc->prot_flags |= ELEVON_PROT_ALPHA_ACTIVE;
        fcc->trim_frozen = true;
        double exceed = alpha_deg - 11.2;
        elev_cmd += 4.5 * exceed; // Strong nose-down elevator
    } else {
        fcc->trim_frozen = false;
    }

    // High-Speed Protection
    double cas_knots = fcc->air_data.v_cas_mps * 1.94384449;
    if (cas_knots > 346.0) {
        fcc->prot_flags |= ELEVON_PROT_HIGH_SPEED;
        fcc->trim_frozen = true;
        elev_cmd -= 0.85 * (cas_knots - 346.0); // Nose-up override
    }

    // Attitude pitch limit (-15 to +30)
    if (fcc->imu_data.pitch_deg >= 30.0 && elev_cmd < 0.0) {
        elev_cmd = 2.0 * (fcc->imu_data.pitch_deg - 30.0);
    } else if (fcc->imu_data.pitch_deg <= -15.0 && elev_cmd > 0.0) {
        elev_cmd = -2.0 * (-15.0 - fcc->imu_data.pitch_deg);
    }

    // Structural g-load limiter (+2.5g / -1.0g)
    if (fcc->imu_data.load_factor_nz > 2.5) {
        fcc->prot_flags |= ELEVON_PROT_G_LIMIT_POS;
        elev_cmd += 6.0 * (fcc->imu_data.load_factor_nz - 2.5);
    } else if (fcc->imu_data.load_factor_nz < -1.0) {
        fcc->prot_flags |= ELEVON_PROT_G_LIMIT_NEG;
        elev_cmd -= 6.0 * (-1.0 - fcc->imu_data.load_factor_nz);
    }

    // Clamps
    if (elev_cmd > 30.0) elev_cmd = 30.0;
    if (elev_cmd < -30.0) elev_cmd = -30.0;

    // Auto-trim THS
    if (!fcc->trim_frozen) {
        if (elev_cmd > 0.75) {
            fcc->auto_trim_ths_deg -= 0.5 * dt_sec;
        } else if (elev_cmd < -0.75) {
            fcc->auto_trim_ths_deg += 0.5 * dt_sec;
        }
        if (fcc->auto_trim_ths_deg > 14.0) fcc->auto_trim_ths_deg = 14.0;
        if (fcc->auto_trim_ths_deg < -3.0) fcc->auto_trim_ths_deg = -3.0;
    }

    // Lateral roll-rate law
    double ail_cmd = roll_stick * 25.0;
    if (fcc->imu_data.roll_deg >= 67.0 && ail_cmd > 0.0) {
        ail_cmd = 0.0;
        fcc->prot_flags |= ELEVON_PROT_BANK_LIMIT;
    } else if (fcc->imu_data.roll_deg <= -67.0 && ail_cmd < 0.0) {
        ail_cmd = 0.0;
        fcc->prot_flags |= ELEVON_PROT_BANK_LIMIT;
    }

    // Directional yaw law
    double rud_cmd = rudder_pedal * 30.0;

    // 6. Step physical actuators
    elevon_actuator_step(&fcc->act_elev_in_l, elev_cmd, 3000.0, dt_sec);
    elevon_actuator_step(&fcc->act_elev_in_r, elev_cmd, 3000.0, dt_sec);
    elevon_actuator_step(&fcc->act_elev_out_l, elev_cmd, 3000.0, dt_sec);
    elevon_actuator_step(&fcc->act_elev_out_r, elev_cmd, 3000.0, dt_sec);
    elevon_actuator_step(&fcc->act_ail_l, ail_cmd, 3000.0, dt_sec);
    elevon_actuator_step(&fcc->act_ail_r, -ail_cmd, 3000.0, dt_sec);
    elevon_actuator_step(&fcc->act_rudder, rud_cmd, 3000.0, dt_sec);

    // Record voted surface positions
    fcc->surfaces.elev_inboard_left_deg = fcc->act_elev_in_l.current_pos_deg;
    fcc->surfaces.elev_inboard_right_deg = fcc->act_elev_in_r.current_pos_deg;
    fcc->surfaces.elev_outboard_left_deg = fcc->act_elev_out_l.current_pos_deg;
    fcc->surfaces.elev_outboard_right_deg = fcc->act_elev_out_r.current_pos_deg;
    fcc->surfaces.aileron_left_deg = fcc->act_ail_l.current_pos_deg;
    fcc->surfaces.aileron_right_deg = fcc->act_ail_r.current_pos_deg;
    fcc->surfaces.rudder_upper_deg = fcc->act_rudder.current_pos_deg;
    fcc->surfaces.rudder_lower_deg = fcc->act_rudder.current_pos_deg;
    fcc->surfaces.ths_deg = fcc->auto_trim_ths_deg;

    // 7. AFDX VL transmission bookkeeping
    elevon_afdx_vl_record_tx(&fcc->vl_actuators, actual_time_us);

    return ELEVON_OK;
}

elevon_surfaces_t elevon_fcc_get_surfaces(const elevon_fcc_t *fcc) {
    if (!fcc) {
        elevon_surfaces_t s;
        memset(&s, 0, sizeof(s));
        return s;
    }
    return fcc->surfaces;
}

elevon_air_data_t elevon_fcc_get_air_data(const elevon_fcc_t *fcc) {
    if (!fcc) {
        elevon_air_data_t a;
        memset(&a, 0, sizeof(a));
        return a;
    }
    return fcc->air_data;
}

elevon_imu_data_t elevon_fcc_get_imu_data(const elevon_fcc_t *fcc) {
    if (!fcc) {
        elevon_imu_data_t i;
        memset(&i, 0, sizeof(i));
        return i;
    }
    return fcc->imu_data;
}

elevon_law_mode_t elevon_fcc_get_law_mode(const elevon_fcc_t *fcc) {
    return fcc ? fcc->law_mode : ELEVON_LAW_MECHANICAL;
}

uint32_t elevon_fcc_get_protection_flags(const elevon_fcc_t *fcc) {
    return fcc ? fcc->prot_flags : 0;
}

void elevon_fcc_get_lane_health(const elevon_fcc_t *fcc, bool *out_a, bool *out_b, bool *out_c) {
    if (!fcc) return;
    if (out_a) *out_a = fcc->lane_healthy[0] && !fcc->faulted_lane[0];
    if (out_b) *out_b = fcc->lane_healthy[1] && !fcc->faulted_lane[1];
    if (out_c) *out_c = fcc->lane_healthy[2] && !fcc->faulted_lane[2];
}

void elevon_fcc_inject_pitot_fault(elevon_fcc_t *fcc, int probe_index, double pressure_drop_pct) {
    if (!fcc) return;
    fcc->faulted_probe = probe_index;
    fcc->probe_fault_pct = pressure_drop_pct;
}

void elevon_fcc_inject_lane_fault(elevon_fcc_t *fcc, elevon_lane_id_t lane) {
    if (!fcc || lane >= ELEVON_LANE_COUNT) return;
    fcc->faulted_lane[lane] = true;
    fcc->lane_healthy[lane] = false;
}

void elevon_fcc_inject_actuator_stall(elevon_fcc_t *fcc, const char *surface_name) {
    if (!fcc || !surface_name) return;
    if (strcmp(surface_name, "ELEV_IN_L") == 0) {
        fcc->act_elev_in_l.stalled = true;
    } else if (strcmp(surface_name, "ELEV_IN_R") == 0) {
        fcc->act_elev_in_r.stalled = true;
    } else if (strcmp(surface_name, "AIL_L") == 0) {
        fcc->act_ail_l.stalled = true;
    }
}
