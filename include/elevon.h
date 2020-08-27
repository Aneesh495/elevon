#ifndef ELEVON_H
#define ELEVON_H

#include "elevon_types.h"
#include "elevon_crc32.h"
#include "elevon_math.h"
#include "elevon_voter.h"
#include "elevon_adirs.h"
#include "elevon_actuator.h"
#include "elevon_afdx.h"
#include "elevon_fdr.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct elevon_fcc elevon_fcc_t;

/* Creates and initializes a master Flight Control Computer instance */
elevon_fcc_t *elevon_fcc_create(void);

/* Frees an FCC instance */
void elevon_fcc_destroy(elevon_fcc_t *fcc);

/* Executes one 10 ms minor cycle of the flight control computer */
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
);

/* Retrieves current voted control surface deflections */
elevon_surfaces_t elevon_fcc_get_surfaces(const elevon_fcc_t *fcc);

/* Retrieves current air data and inertial navigation state */
elevon_air_data_t elevon_fcc_get_air_data(const elevon_fcc_t *fcc);
elevon_imu_data_t elevon_fcc_get_imu_data(const elevon_fcc_t *fcc);

/* Retrieves active flight control law mode and protection flags */
elevon_law_mode_t elevon_fcc_get_law_mode(const elevon_fcc_t *fcc);
uint32_t elevon_fcc_get_protection_flags(const elevon_fcc_t *fcc);

/* Checks health status of TMR computational lanes A, B, and C */
void elevon_fcc_get_lane_health(const elevon_fcc_t *fcc, bool *out_a, bool *out_b, bool *out_c);

/* Injects hardware or aerodynamic faults for safety drills */
void elevon_fcc_inject_pitot_fault(elevon_fcc_t *fcc, int probe_index, double pressure_drop_pct);
void elevon_fcc_inject_lane_fault(elevon_fcc_t *fcc, elevon_lane_id_t lane);
void elevon_fcc_inject_actuator_stall(elevon_fcc_t *fcc, const char *surface_name);

#ifdef __cplusplus
}
#endif

#endif /* ELEVON_H */
