#include "elevon_adirs.h"
#include "elevon_math.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>

static void test_isa_sea_level(void) {
    double t = elevon_isa_temperature(0.0);
    double p = elevon_isa_pressure(0.0);
    double rho = elevon_isa_density(0.0);
    double a = elevon_isa_speed_of_sound(0.0);

    assert(fabs(t - 288.15) < 1e-3);
    assert(fabs(p - 101325.0) < 1.0);
    assert(fabs(rho - 1.225) < 1e-2);
    assert(fabs(a - 340.29) < 0.5);
    printf("test_isa_sea_level: PASSED\n");
}

static void test_isa_tropopause_and_inversion(void) {
    double t = elevon_isa_temperature(11000.0);
    double p = elevon_isa_pressure(11000.0);
    assert(fabs(t - 216.65) < 1e-2);
    assert(fabs(p - 22632.1) < 5.0);

    // Test reverse altitude lookup
    double alt_recovered = elevon_isa_altitude_from_pressure(p);
    assert(fabs(alt_recovered - 11000.0) < 10.0);
    printf("test_isa_tropopause_and_inversion: PASSED\n");
}

static void test_compressible_cas_and_mach(void) {
    // Sea level, 8800 Pa impact pressure produces approx 229.5 knots CAS
    double cas_mps = elevon_adirs_compute_cas(8800.0);
    double cas_knots = cas_mps * 1.94384449;
    assert(cas_knots > 225.0 && cas_knots < 235.0);

    // Cruise at 33,000 ft: Ps = 26,000 Pa, Pt = 34,500 Pa
    double mach = elevon_adirs_compute_mach(26000.0, 34500.0);
    assert(mach > 0.60 && mach < 0.70);
    printf("test_compressible_cas_and_mach: PASSED\n");
}

static void test_adc_pipeline(void) {
    elevon_air_data_t ad;
    elevon_status_t status = elevon_adc_process(26000.0, 34500.0, 235.0, &ad);
    assert(status == ELEVON_OK);
    assert(ad.valid);
    assert(ad.altitude_m > 9500.0 && ad.altitude_m < 10500.0);
    assert(ad.v_tas_mps > 180.0 && ad.v_tas_mps < 210.0);
    printf("test_adc_pipeline: PASSED\n");
}

static void test_imu_strapdown(void) {
    elevon_imu_data_t imu;
    imu.quat = elevon_quat_identity();
    imu.load_factor_nz = 1.0;

    // Pitch rate of 0.1 rad/sec for 1.0 second
    elevon_vec3_t gyro = {0.0, 0.1, 0.0};
    elevon_vec3_t accel = {0.0, 0.0, -9.80665};

    for (int i = 0; i < 100; i++) {
        elevon_imu_step(&imu, gyro, accel, 0.01);
    }

    // 0.1 rad = 5.73 degrees pitch
    assert(fabs(imu.pitch_deg - 5.73) < 0.3);
    assert(fabs(imu.roll_deg) < 0.01);
    assert(fabs(imu.load_factor_nz - 1.0) < 0.01);
    printf("test_imu_strapdown: PASSED\n");
}

int main(void) {
    printf("=== Starting ADIRS Unit Tests ===\n");
    test_isa_sea_level();
    test_isa_tropopause_and_inversion();
    test_compressible_cas_and_mach();
    test_adc_pipeline();
    test_imu_strapdown();
    printf("All ADIRS tests passed successfully.\n");
    return 0;
}
