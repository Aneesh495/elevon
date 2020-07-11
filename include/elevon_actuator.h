#ifndef ELEVON_ACTUATOR_H
#define ELEVON_ACTUATOR_H

#include "elevon_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double current_pos_deg;
    double target_pos_deg;
    double max_slew_deg_s;
    double time_constant_s;
    double min_pos_deg;
    double max_pos_deg;
    bool stalled;
    bool tripped_ofd;
} elevon_actuator_state_t;

/* Initializes actuator model with physical constraints */
void elevon_actuator_init(
    elevon_actuator_state_t *act,
    double max_slew_deg_s,
    double time_constant_s,
    double min_pos_deg,
    double max_pos_deg
);

/* Advances actuator servo step forward in time */
double elevon_actuator_step(
    elevon_actuator_state_t *act,
    double target_deg,
    double hydraulic_press_psi,
    double dt_sec
);

/* Updates hydraulic circuit pressures over dt */
void elevon_hydraulic_step(
    elevon_hydraulic_status_t *hyd,
    bool engines_running,
    bool rat_deployed,
    double dt_sec
);

/* High-speed OFD check on position time-series */
bool elevon_ofd_check(
    double current_pos_deg,
    uint64_t timestamp_us,
    double *prev_pos,
    uint64_t *last_crossing_us,
    int *osc_count,
    double *peak_pos,
    double *peak_neg
);

#ifdef __cplusplus
}
#endif

#endif /* ELEVON_ACTUATOR_H */
