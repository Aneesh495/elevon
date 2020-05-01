#include "elevon_voter.h"
#include <math.h>

elevon_mvs_c_result_t elevon_mvs_vote_scalar(
    double a, bool valid_a,
    double b, bool valid_b,
    double c, bool valid_c,
    double threshold_triplex,
    double threshold_duplex
) {
    elevon_mvs_c_result_t res;
    res.diverged[0] = false;
    res.diverged[1] = false;
    res.diverged[2] = false;
    res.byzantine = false;

    // Case 1: All 3 channels valid
    if (valid_a && valid_b && valid_c) {
        res.healthy_count = 3;
        double diff_ab = fabs(a - b);
        double diff_bc = fabs(b - c);
        double diff_ca = fabs(c - a);

        if (diff_ab > threshold_triplex && diff_ca > threshold_triplex && diff_bc <= threshold_triplex) {
            res.diverged[0] = true;
        }
        if (diff_ab > threshold_triplex && diff_bc > threshold_triplex && diff_ca <= threshold_triplex) {
            res.diverged[1] = true;
        }
        if (diff_bc > threshold_triplex && diff_ca > threshold_triplex && diff_ab <= threshold_triplex) {
            res.diverged[2] = true;
        }

        if (diff_ab > threshold_triplex && diff_bc > threshold_triplex && diff_ca > threshold_triplex) {
            res.byzantine = true;
        }

        // Mid-value selection
        if ((a <= b && b <= c) || (c <= b && b <= a)) {
            res.voted_val = b;
        } else if ((b <= a && a <= c) || (c <= a && a <= b)) {
            res.voted_val = a;
        } else {
            res.voted_val = c;
        }
        return res;
    }

    // Case 2: Duplex
    if (valid_a && valid_b && !valid_c) {
        res.healthy_count = 2;
        res.diverged[2] = true;
        res.byzantine = (fabs(a - b) > threshold_duplex);
        res.voted_val = (a + b) * 0.5;
        return res;
    }
    if (valid_b && valid_c && !valid_a) {
        res.healthy_count = 2;
        res.diverged[0] = true;
        res.byzantine = (fabs(b - c) > threshold_duplex);
        res.voted_val = (b + c) * 0.5;
        return res;
    }
    if (valid_c && valid_a && !valid_b) {
        res.healthy_count = 2;
        res.diverged[1] = true;
        res.byzantine = (fabs(c - a) > threshold_duplex);
        res.voted_val = (c + a) * 0.5;
        return res;
    }

    // Case 3: Simplex
    if (valid_a) {
        res.healthy_count = 1;
        res.diverged[1] = true;
        res.diverged[2] = true;
        res.voted_val = a;
        return res;
    }
    if (valid_b) {
        res.healthy_count = 1;
        res.diverged[0] = true;
        res.diverged[2] = true;
        res.voted_val = b;
        return res;
    }
    if (valid_c) {
        res.healthy_count = 1;
        res.diverged[0] = true;
        res.diverged[1] = true;
        res.voted_val = c;
        return res;
    }

    // Case 4: Total loss
    res.healthy_count = 0;
    res.diverged[0] = true;
    res.diverged[1] = true;
    res.diverged[2] = true;
    res.byzantine = true;
    res.voted_val = 0.0;
    return res;
}

bool elevon_discrete_vote_2oo3(
    bool a, bool valid_a,
    bool b, bool valid_b,
    bool c, bool valid_c,
    bool default_val
) {
    if (valid_a && valid_b && valid_c) {
        return (a && b) || (b && c) || (c && a);
    }
    if (valid_a && valid_b) { return (a == b) ? a : default_val; }
    if (valid_b && valid_c) { return (b == c) ? b : default_val; }
    if (valid_c && valid_a) { return (c == a) ? c : default_val; }
    if (valid_a) { return a; }
    if (valid_b) { return b; }
    if (valid_c) { return c; }
    return default_val;
}

void elevon_vote_surfaces(
    const elevon_surfaces_t *in_a, bool valid_a,
    const elevon_surfaces_t *in_b, bool valid_b,
    const elevon_surfaces_t *in_c, bool valid_c,
    elevon_surfaces_t *out_voted,
    double surf_threshold_deg
) {
    if (!out_voted) {
        return;
    }
    double th_dup = surf_threshold_deg * 1.5;

    #define VOTE_ELEM(elem) \
        out_voted->elem = elevon_mvs_vote_scalar( \
            in_a ? in_a->elem : 0.0, in_a && valid_a, \
            in_b ? in_b->elem : 0.0, in_b && valid_b, \
            in_c ? in_c->elem : 0.0, in_c && valid_c, \
            surf_threshold_deg, th_dup \
        ).voted_val

    VOTE_ELEM(elev_inboard_left_deg);
    VOTE_ELEM(elev_inboard_right_deg);
    VOTE_ELEM(elev_outboard_left_deg);
    VOTE_ELEM(elev_outboard_right_deg);
    VOTE_ELEM(aileron_left_deg);
    VOTE_ELEM(aileron_right_deg);

    for (int i = 0; i < 5; i++) {
        VOTE_ELEM(spoilers_left_deg[i]);
        VOTE_ELEM(spoilers_right_deg[i]);
    }

    VOTE_ELEM(rudder_upper_deg);
    VOTE_ELEM(rudder_lower_deg);
    VOTE_ELEM(ths_deg);

    #undef VOTE_ELEM
}
