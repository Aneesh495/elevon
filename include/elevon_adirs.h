#ifndef ELEVON_ADIRS_H
#define ELEVON_ADIRS_H

#include "elevon_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ISA Atmosphere lookups */
double elevon_isa_temperature(double alt_m);
double elevon_isa_pressure(double alt_m);
double elevon_isa_density(double alt_m);
double elevon_isa_speed_of_sound(double alt_m);
double elevon_isa_altitude_from_pressure(double p_static_pa);

/* Aerodynamic airspeed and Mach calculations */
double elevon_adirs_compute_cas(double impact_pressure_pa);
double elevon_adirs_compute_mach(double p_static_pa, double p_total_pa);
double elevon_adirs_compute_tas(double mach, double tat_k);

/* Full ADC pipeline processing */
elevon_status_t elevon_adc_process(
    double p_static_pa,
    double p_total_pa,
    double tat_k,
    elevon_air_data_t *out_data
);

/* IMU strapdown attitude integration step */
void elevon_imu_step(
    elevon_imu_data_t *imu,
    elevon_vec3_t gyro_rad_s,
    elevon_vec3_t accel_m_s2,
    double dt_sec
);

#ifdef __cplusplus
}
#endif

#endif /* ELEVON_ADIRS_H */
