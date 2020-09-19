#include "elevon_voter.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>

static void test_mvs_triplex_identical(void) {
    elevon_mvs_c_result_t res = elevon_mvs_vote_scalar(10.0, true, 10.0, true, 10.0, true, 1.5, 2.5);
    assert(res.healthy_count == 3);
    assert(fabs(res.voted_val - 10.0) < 1e-6);
    assert(!res.diverged[0] && !res.diverged[1] && !res.diverged[2]);
    assert(!res.byzantine);
    printf("test_mvs_triplex_identical: PASSED\n");
}

static void test_mvs_triplex_ordered(void) {
    elevon_mvs_c_result_t res = elevon_mvs_vote_scalar(5.0, true, 10.0, true, 15.0, true, 20.0, 25.0);
    assert(res.healthy_count == 3);
    assert(fabs(res.voted_val - 10.0) < 1e-6);
    printf("test_mvs_triplex_ordered: PASSED\n");
}

static void test_mvs_single_outlier(void) {
    // Lane C diverges wildly (95.0 vs 10.0, 10.05)
    elevon_mvs_c_result_t res = elevon_mvs_vote_scalar(10.0, true, 10.05, true, 95.0, true, 1.5, 2.5);
    assert(res.healthy_count == 3);
    assert(fabs(res.voted_val - 10.05) < 1e-6);
    assert(!res.diverged[0] && !res.diverged[1] && res.diverged[2]);
    assert(!res.byzantine);
    printf("test_mvs_single_outlier: PASSED\n");
}

static void test_mvs_byzantine_fault(void) {
    // All 3 channels disagree widely
    elevon_mvs_c_result_t res = elevon_mvs_vote_scalar(10.0, true, 40.0, true, 80.0, true, 5.0, 10.0);
    assert(res.byzantine);
    printf("test_mvs_byzantine_fault: PASSED\n");
}

static void test_mvs_duplex_average(void) {
    // Lane C failed
    elevon_mvs_c_result_t res = elevon_mvs_vote_scalar(12.0, true, 12.4, true, 0.0, false, 1.5, 2.5);
    assert(res.healthy_count == 2);
    assert(fabs(res.voted_val - 12.2) < 1e-6);
    assert(res.diverged[2]);
    assert(!res.byzantine);
    printf("test_mvs_duplex_average: PASSED\n");
}

static void test_discrete_majority_2oo3(void) {
    assert(elevon_discrete_vote_2oo3(true, true, true, true, false, true, false) == true);
    assert(elevon_discrete_vote_2oo3(false, true, true, true, false, true, false) == false);
    assert(elevon_discrete_vote_2oo3(true, true, false, true, true, true, false) == true);
    // Duplex agreement
    assert(elevon_discrete_vote_2oo3(true, true, true, true, false, false, false) == true);
    // Duplex disagreement falls back to default
    assert(elevon_discrete_vote_2oo3(true, true, false, true, false, false, false) == false);
    printf("test_discrete_majority_2oo3: PASSED\n");
}

int main(void) {
    printf("=== Starting Voter Unit Tests ===\n");
    test_mvs_triplex_identical();
    test_mvs_triplex_ordered();
    test_mvs_single_outlier();
    test_mvs_byzantine_fault();
    test_mvs_duplex_average();
    test_discrete_majority_2oo3();
    printf("All voter tests passed successfully.\n");
    return 0;
}
