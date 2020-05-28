#include "elevon_adirs.h"
#include "elevon_math.h"
#include <math.h>

#define P0_PA 101325.0
#define T0_K 288.15
#define RHO0_KG_M3 1.225
#define G0_M_S2 9.80665
#define R_SPECIFIC 287.05287
#define GAMMA 1.4
#define LAPSE_RATE -0.0065
#define TROPOPAUSE_ALT_M 11000.0
#define T_TROPOPAUSE_K 216.65
#define P_TROPOPAUSE_PA 22632.1
#define A0_MPS 340.29437

double elevon_isa_temperature(double alt_m) {
    if (alt_m <= TROPOPAUSE_ALT_M) {
        return T0_K + (LAPSE_RATE * alt_m);
    }
    return T_TROPOPAUSE_K;
}

double elevon_isa_pressure(double alt_m) {
    if (alt_m <= TROPOPAUSE_ALT_M) {
        double t = elevon_isa_temperature(alt_m);
        double exponent = -G0_M_S2 / (LAPSE_RATE * R_SPECIFIC);
        return P0_PA * pow(t / T0_K, exponent);
    } else {
        double delta_h = alt_m - TROPOPAUSE_ALT_M;
        double exponent = -G0_M_S2 * delta_h / (R_SPECIFIC * T_TROPOPAUSE_K);
        return P_TROPOPAUSE_PA * exp(exponent);
    }
}

double elevon_isa_density(double alt_m) {
    double p = elevon_isa_pressure(alt_m);
    double t = elevon_isa_temperature(alt_m);
    return p / (R_SPECIFIC * t);
}

double elevon_isa_speed_of_sound(double alt_m) {
    double t = elevon_isa_temperature(alt_m);
    return sqrt(GAMMA * R_SPECIFIC * t);
}

double elevon_isa_altitude_from_pressure(double p_static_pa) {
    if (p_static_pa <= 0.0) {
        return 30000.0;
    }
    if (p_static_pa >= P_TROPOPAUSE_PA) {
        double exponent = -LAPSE_RATE * R_SPECIFIC / G0_M_S2;
        double ratio = pow(p_static_pa / P0_PA, exponent);
        return (T0_K / -LAPSE_RATE) * (1.0 - ratio);
    } else {
        double log_ratio = log(p_static_pa / P_TROPOPAUSE_PA);
        double delta_h = -R_SPECIFIC * T_TROPOPAUSE_K * log_ratio / G0_M_S2;
        return TROPOPAUSE_ALT_M + delta_h;
    }
}

double elevon_adirs_compute_cas(double impact_pressure_pa) {
    if (impact_pressure_pa <= 0.0) {
        return 0.0;
    }
    double ratio = (impact_pressure_pa / P0_PA) + 1.0;
    double term = pow(ratio, 2.0 / 7.0) - 1.0;
    if (term <= 0.0) {
        return 0.0;
    }
    return A0_MPS * sqrt(5.0 * term);
}

double elevon_adirs_compute_mach(double p_static_pa, double p_total_pa) {
    if (p_static_pa <= 0.0 || p_total_pa <= p_static_pa) {
        return 0.0;
    }
    double pressure_ratio = p_total_pa / p_static_pa;
    double term = pow(pressure_ratio, 2.0 / 7.0) - 1.0;
    if (term <= 0.0) {
        return 0.0;
    }
    return sqrt(5.0 * term);
}

double elevon_adirs_compute_tas(double mach, double tat_k) {
    double temp_divisor = 1.0 + (0.98 * 0.2 * mach * mach);
    double sat_k = (tat_k > 100.0) ? (tat_k / temp_divisor) : 288.15;
    double a = sqrt(GAMMA * R_SPECIFIC * sat_k);
    return mach * a;
}

elevon_status_t elevon_adc_process(
    double p_static_pa,
    double p_total_pa,
    double tat_k,
    elevon_air_data_t *out_data
) {
    if (!out_data) {
        return ELEVON_ERR_NULL_PTR;
    }
    if (p_static_pa < 5000.0 || p_static_pa > 115000.0 || p_total_pa < (p_static_pa - 200.0)) {
        out_data->valid = false;
        return ELEVON_ERR_OUT_OF_BOUNDS;
    }

    double qc = (p_total_pa >= p_static_pa) ? (p_total_pa - p_static_pa) : 0.0;
    out_data->p_static_pa = p_static_pa;
    out_data->p_total_pa = p_total_pa;
    out_data->tat_kelvin = tat_k;
    out_data->altitude_m = elevon_isa_altitude_from_pressure(p_static_pa);
    out_data->v_cas_mps = elevon_adirs_compute_cas(qc);
    out_data->mach = elevon_adirs_compute_mach(p_static_pa, p_total_pa);
    out_data->v_tas_mps = elevon_adirs_compute_tas(out_data->mach, tat_k);
    out_data->valid = true;

    return ELEVON_OK;
}

void elevon_imu_step(
    elevon_imu_data_t *imu,
    elevon_vec3_t gyro_rad_s,
    elevon_vec3_t accel_m_s2,
    double dt_sec
) {
    if (!imu) {
        return;
    }
    imu->omega_rad_s = gyro_rad_s;
    imu->accel_m_s2 = accel_m_s2;
    imu->quat = elevon_quat_integrate(imu->quat, gyro_rad_s, dt_sec);

    double roll_rad, pitch_rad, yaw_rad;
    elevon_quat_to_euler(imu->quat, &roll_rad, &pitch_rad, &yaw_rad);
    imu->roll_deg = roll_rad * ELEVON_RAD_TO_DEG;
    imu->pitch_deg = pitch_rad * ELEVON_RAD_TO_DEG;
    imu->heading_deg = yaw_rad * ELEVON_RAD_TO_DEG;
    imu->load_factor_nz = -accel_m_s2.z / ELEVON_GRAVITY_G0;
    imu->valid = true;
}
