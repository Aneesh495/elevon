#include "elevon.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>

static void test_c_star_nominal(void) {
    elevon_fcc_t *fcc = elevon_fcc_create();
    assert(fcc != NULL);

    elevon_inceptor_t inc = {0};
    inc.pitch_stick_norm = 0.5; // Pilot pulls aft stick

    elevon_vec3_t gyro = {0.0, 0.05, 0.0};
    elevon_vec3_t accel = {0.0, 0.0, -9.80665 * 1.5}; // 1.5g pull

    elevon_fcc_step(fcc, 10000, &inc, 30000.0, 42000.0, 240.0, gyro, accel, 0.01);

    elevon_surfaces_t s = elevon_fcc_get_surfaces(fcc);
    // Commanded elevator must respond to pilot demand
    assert(fabs(s.elev_inboard_left_deg) > 0.1);

    elevon_fcc_destroy(fcc);
    printf("test_c_star_nominal: PASSED\n");
}

static void test_high_speed_protection(void) {
    elevon_fcc_t *fcc = elevon_fcc_create();
    assert(fcc != NULL);

    elevon_inceptor_t inc = {0}; // Neutral stick

    // Extreme high total pressure -> CAS > 350 knots
    elevon_fcc_step(
        fcc, 10000, &inc,
        70000.0, 130000.0, 280.0,
        (elevon_vec3_t){0}, (elevon_vec3_t){0, 0, -9.80665}, 0.01
    );

    uint32_t flags = elevon_fcc_get_protection_flags(fcc);
    assert(flags & ELEVON_PROT_HIGH_SPEED);

    elevon_surfaces_t s = elevon_fcc_get_surfaces(fcc);
    // Negative elevator deflection corresponds to nose-up command
    assert(s.elev_inboard_left_deg < 0.0);

    elevon_fcc_destroy(fcc);
    printf("test_high_speed_protection: PASSED\n");
}

static void test_alpha_protection(void) {
    elevon_fcc_t *fcc = elevon_fcc_create();
    assert(fcc != NULL);

    elevon_inceptor_t inc = {0};
    // Force pitch rate high to simulate sudden high AOA pull
    elevon_vec3_t gyro = {0.0, 0.95, 0.0};
    elevon_vec3_t accel = {0.0, 0.0, -9.80665 * 1.8};

    elevon_fcc_step(
        fcc, 10000, &inc,
        90000.0, 105000.0, 288.0,
        gyro, accel, 0.01
    );

    uint32_t flags = elevon_fcc_get_protection_flags(fcc);
    assert(flags & ELEVON_PROT_ALPHA_ACTIVE);

    elevon_surfaces_t s = elevon_fcc_get_surfaces(fcc);
    // Positive elevator deflection pushes nose down
    assert(s.elev_inboard_left_deg > 0.0);

    elevon_fcc_destroy(fcc);
    printf("test_alpha_protection: PASSED\n");
}

static void test_bank_angle_limit(void) {
    elevon_fcc_t *fcc = elevon_fcc_create();
    assert(fcc != NULL);

    elevon_inceptor_t inc = {0};
    inc.roll_stick_norm = 1.0; // Demand right roll

    // Integrate roll angle up to 70 deg
    elevon_vec3_t gyro = {0.5, 0.0, 0.0};
    elevon_vec3_t accel = {0.0, 0.0, -9.80665};

    for (int i = 0; i < 250; i++) {
        elevon_fcc_step(fcc, (i + 1) * 10000, &inc, 50000.0, 60000.0, 260.0, gyro, accel, 0.01);
    }

    uint32_t flags = elevon_fcc_get_protection_flags(fcc);
    assert(flags & ELEVON_PROT_BANK_LIMIT);

    elevon_fcc_destroy(fcc);
    printf("test_bank_angle_limit: PASSED\n");
}

int main(void) {
    printf("=== Starting Flight Control Law Unit Tests ===\n");
    test_c_star_nominal();
    test_high_speed_protection();
    test_alpha_protection();
    test_bank_angle_limit();
    printf("All FCL tests passed successfully.\n");
    return 0;
}
