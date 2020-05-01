#ifndef ELEVON_VOTER_H
#define ELEVON_VOTER_H

#include "elevon_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double voted_val;
    int healthy_count;
    bool diverged[3];
    bool byzantine;
} elevon_mvs_c_result_t;

/*
 * Evaluates Mid-Value Selection across three scalar inputs with validation flags.
 */
elevon_mvs_c_result_t elevon_mvs_vote_scalar(
    double a, bool valid_a,
    double b, bool valid_b,
    double c, bool valid_c,
    double threshold_triplex,
    double threshold_duplex
);

/*
 * Discrete 2-out-of-3 majority voting across boolean flags.
 */
bool elevon_discrete_vote_2oo3(
    bool a, bool valid_a,
    bool b, bool valid_b,
    bool c, bool valid_c,
    bool default_val
);

/*
 * Full surface array MVS voting fastpath.
 */
void elevon_vote_surfaces(
    const elevon_surfaces_t *in_a, bool valid_a,
    const elevon_surfaces_t *in_b, bool valid_b,
    const elevon_surfaces_t *in_c, bool valid_c,
    elevon_surfaces_t *out_voted,
    double surf_threshold_deg
);

#ifdef __cplusplus
}
#endif

#endif /* ELEVON_VOTER_H */
