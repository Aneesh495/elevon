#include "elevon.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>

static void test_10000_cycle_flight(void) {
    elevon_fcc_t *fcc = elevon_fcc_create();
    assert(fcc != NULL);

    elevon_inceptor_t inceptor;
    elevon_inceptor_zero(&inceptor);

    // Initial state: sea level
    double p_static = 101325.0;
    double p_total = 101325.0;
    double tat_k = 288.15;
    double sim_altitude_m = 0.0;
    double sim_cas_mps = 0.0;

    const int TOTAL_FRAMES = 10000; // 100 seconds at 100 Hz

    for (int f = 0; f < TOTAL_FRAMES; f++) {
        uint64_t now_us = (uint64_t)f * 10000ULL;
        double t_sec = (double)f * 0.01;

        // Phase 1: Takeoff roll (0 to 15 s)
        if (t_sec < 15.0) {
            sim_cas_mps += 5.5 * 0.01; // Accelerate to ~82 m/s (160 kt)
            if (t_sec > 10.0) {
                inceptor.pitch_stick_norm = 0.35; // Rotate nose up
            }
        }
        // Phase 2: Climb out (15 to 40 s)
        else if (t_sec < 40.0) {
            inceptor.pitch_stick_norm = 0.10;
            sim_altitude_m += 15.0 * 0.01 * 100.0; // Climb rate
            sim_cas_mps = 128.0; // 250 knots
        }
        // Phase 3: Cruise and bank turn (40 to 70 s)
        else if (t_sec < 70.0) {
            inceptor.pitch_stick_norm = 0.0;
            inceptor.roll_stick_norm = (t_sec >= 45.0 && t_sec <= 60.0) ? 0.30 : 0.0;
            sim_altitude_m = 10000.0; // 33,000 ft
            sim_cas_mps = 135.0;
        }
        // Phase 4: Turbulence encounter (70 to 85 s)
        else if (t_sec < 85.0) {
            inceptor.roll_stick_norm = 0.0;
            inceptor.pitch_stick_norm = 0.0;
            sim_altitude_m = 10000.0;
            sim_cas_mps = 140.0;
        }
        // Phase 5: Descent (85 to 100 s)
        else {
            inceptor.pitch_stick_norm = -0.10;
            sim_altitude_m -= 12.0 * 0.01 * 100.0;
            sim_cas_mps = 110.0;
        }

        // Compute ambient pressure at simulated altitude
        p_static = elevon_isa_pressure(sim_altitude_m);
        tat_k = elevon_isa_temperature(sim_altitude_m);
        // Dynamic impact pressure: qc = 0.5 * rho0 * V_cas^2
        double qc = 0.5 * 1.225 * sim_cas_mps * sim_cas_mps;
        p_total = p_static + qc;

        // Gust injection during Phase 4
        double gust_accel = (t_sec >= 70.0 && t_sec < 85.0) ? (sin(t_sec * 5.0) * 4.5) : 0.0;
        elevon_vec3_t gyro = {
            inceptor.roll_stick_norm * 0.15,
            inceptor.pitch_stick_norm * 0.08,
            0.0
        };
        elevon_vec3_t accel = {0.0, 0.0, -9.80665 + gust_accel};

        elevon_status_t status = elevon_fcc_step(
            fcc, now_us, &inceptor, p_static, p_total, tat_k, gyro, accel, 0.01
        );
        assert(status == ELEVON_OK);

        // Invariant checks
        elevon_surfaces_t s = elevon_fcc_get_surfaces(fcc);
        assert(!isnan(s.elev_inboard_left_deg) && !isinf(s.elev_inboard_left_deg));
        assert(!isnan(s.aileron_left_deg) && !isinf(s.aileron_left_deg));
        assert(!isnan(s.rudder_upper_deg) && !isinf(s.rudder_upper_deg));

        // Physical deflection boundary assertions
        assert(s.elev_inboard_left_deg >= -30.01 && s.elev_inboard_left_deg <= 25.01);
        assert(s.aileron_left_deg >= -25.01 && s.aileron_left_deg <= 25.01);
        assert(s.rudder_upper_deg >= -30.01 && s.rudder_upper_deg <= 30.01);
        assert(s.ths_deg >= -3.01 && s.ths_deg <= 14.01);

        // System must remain healthy in Normal Law throughout full flight
        elevon_law_mode_t law = elevon_fcc_get_law_mode(fcc);
        assert(law == ELEVON_LAW_NORMAL);
    }

    elevon_fcc_destroy(fcc);
    printf("test_10000_cycle_flight: PASSED (10,000 cycles without violation)\n");
}

int main(void) {
    printf("=== Starting End-to-End Simulation Tests ===\n");
    test_10000_cycle_flight();
    printf("All simulation integration tests passed successfully.\n");
    return 0;
}
