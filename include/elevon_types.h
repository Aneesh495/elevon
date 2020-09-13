#ifndef ELEVON_TYPES_H
#define ELEVON_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Standard error return codes */
typedef enum {
    ELEVON_OK                     = 0,
    ELEVON_ERR_NULL_PTR           = -1,
    ELEVON_ERR_OUT_OF_BOUNDS      = -2,
    ELEVON_ERR_BYZANTINE_FAULT    = -3,
    ELEVON_ERR_BAG_VIOLATION      = -4,
    ELEVON_ERR_FDR_CORRUPT        = -5,
    ELEVON_ERR_CRC_MISMATCH       = -6,
    ELEVON_ERR_HYDRAULIC_LOSS     = -7,
    ELEVON_ERR_ACTUATOR_STALL     = -8,
    ELEVON_ERR_CCDL_TIMEOUT       = -9,
    ELEVON_ERR_AIR_DATA_DISAGREE  = -10,
    ELEVON_ERR_INVALID_STATE      = -11
} elevon_status_t;

/* Triple-modular redundant lane identities */
typedef enum {
    ELEVON_LANE_A = 0,
    ELEVON_LANE_B = 1,
    ELEVON_LANE_C = 2,
    ELEVON_LANE_COUNT = 3
} elevon_lane_id_t;

/* Flight Control Law operational modes */
typedef enum {
    ELEVON_LAW_NORMAL     = 0,
    ELEVON_LAW_ALTERNATE  = 1,
    ELEVON_LAW_DIRECT     = 2,
    ELEVON_LAW_MECHANICAL = 3
} elevon_law_mode_t;

/* Envelope protection state flags */
typedef enum {
    ELEVON_PROT_NONE          = 0,
    ELEVON_PROT_ALPHA_ACTIVE  = (1 << 0),
    ELEVON_PROT_ALPHA_FLOOR   = (1 << 1),
    ELEVON_PROT_HIGH_SPEED    = (1 << 2),
    ELEVON_PROT_BANK_LIMIT    = (1 << 3),
    ELEVON_PROT_G_LIMIT_POS   = (1 << 4),
    ELEVON_PROT_G_LIMIT_NEG   = (1 << 5)
} elevon_prot_flags_t;

/* 3-Dimensional vector in body or navigation frames */
typedef struct {
    double x;
    double y;
    double z;
} elevon_vec3_t;

/* 3x3 Matrix for coordinate transformations */
typedef struct {
    double m[3][3];
} elevon_mat3_t;

/* Quaternion representing attitude rotation (w + xi + yj + zk) */
typedef struct {
    double w;
    double x;
    double y;
    double z;
} elevon_quat_t;

/* Raw and calibrated air data parameters */
typedef struct {
    double p_static_pa;       /* Static ambient pressure in Pascals */
    double p_total_pa;        /* Total / Pitot pressure in Pascals */
    double tat_kelvin;        /* Total Air Temperature in Kelvin */
    double v_cas_mps;         /* Calibrated Airspeed in meters/second */
    double v_tas_mps;         /* True Airspeed in meters/second */
    double mach;              /* Mach number */
    double altitude_m;        /* Pressure altitude in meters */
    double alpha_deg;         /* Angle of Attack in degrees */
    double beta_deg;          /* Angle of Sideslip in degrees */
    bool valid;               /* Sensor validity flag */
} elevon_air_data_t;

/* Inertial Measurement Unit dynamic state */
typedef struct {
    elevon_vec3_t omega_rad_s; /* Body angular rates (p, q, r) in rad/sec */
    elevon_vec3_t accel_m_s2;  /* Body linear accelerations (ax, ay, az) in m/s^2 */
    elevon_quat_t quat;        /* Attitude quaternion */
    double pitch_deg;          /* Euler pitch angle (theta) */
    double roll_deg;           /* Euler roll angle (phi) */
    double heading_deg;        /* Euler yaw / heading angle (psi) */
    double load_factor_nz;     /* Normal load factor Nz in g units */
    bool valid;                /* Sensor validity flag */
} elevon_imu_data_t;

/* Pilot inceptor (sidestick, rudder pedals, speedbrake lever) commands */
typedef struct {
    double pitch_stick_norm;   /* Longitudinal stick demand (-1.0 to +1.0) */
    double roll_stick_norm;    /* Lateral stick demand (-1.0 to +1.0) */
    double rudder_pedal_norm;  /* Directional pedal demand (-1.0 to +1.0) */
    double speedbrake_lever;   /* Speedbrake command (0.0 to 1.0) */
    bool ap_disconnect;        /* Autopilot instinctive disconnect switch */
    bool take_over_priority;   /* Priority takeover pushbutton */
} elevon_inceptor_t;

/* Aircraft primary and secondary flight control surface positions (degrees) */
typedef struct {
    double elev_inboard_left_deg;   /* Left inboard elevator */
    double elev_inboard_right_deg;  /* Right inboard elevator */
    double elev_outboard_left_deg;  /* Left outboard elevator */
    double elev_outboard_right_deg; /* Right outboard elevator */
    double aileron_left_deg;        /* Left aileron */
    double aileron_right_deg;       /* Right aileron */
    double spoilers_left_deg[5];    /* Left wing spoilers 1 to 5 */
    double spoilers_right_deg[5];   /* Right wing spoilers 1 to 5 */
    double rudder_upper_deg;        /* Upper rudder segment */
    double rudder_lower_deg;        /* Lower rudder segment */
    double ths_deg;                 /* Trimmable Horizontal Stabilizer */
} elevon_surfaces_t;

/* Hydraulic power distribution status */
typedef struct {
    double green_press_psi;
    double blue_press_psi;
    double yellow_press_psi;
    bool green_avail;
    bool blue_avail;
    bool yellow_avail;
} elevon_hydraulic_status_t;

/* Utility conversion and helper declarations */
const char *elevon_status_str(elevon_status_t status);
const char *elevon_law_mode_str(elevon_law_mode_t mode);
void elevon_surfaces_zero(elevon_surfaces_t *surf);
void elevon_inceptor_zero(elevon_inceptor_t *inc);

#ifdef __cplusplus
}
#endif

#endif /* ELEVON_TYPES_H */
