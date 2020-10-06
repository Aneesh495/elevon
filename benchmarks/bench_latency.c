#include "elevon.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>

static int cmp_double(const void *a, const void *b) {
    double da = *(const double *)a;
    double db = *(const double *)b;
    if (da < db) return -1;
    if (da > db) return 1;
    return 0;
}

int main(void) {
    printf("================================================================================\n");
    printf(" ELEVON FLIGHT CONTROL COMPUTER: HARD REAL-TIME LATENCY BENCHMARK\n");
    printf("================================================================================\n\n");

    elevon_fcc_t *fcc = elevon_fcc_create();
    if (!fcc) {
        fprintf(stderr, "Failed to create FCC instance\n");
        return 1;
    }

    const int SAMPLES = 20000;
    double *latencies_us = (double *)malloc(SAMPLES * sizeof(double));
    if (!latencies_us) {
        elevon_fcc_destroy(fcc);
        return 1;
    }

    elevon_inceptor_t inceptor = {0};
    inceptor.pitch_stick_norm = 0.15;
    inceptor.roll_stick_norm = 0.20;

    elevon_vec3_t gyro = {0.05, 0.02, 0.001};
    elevon_vec3_t accel = {0.0, 0.0, -9.80665 * 1.1};

    // Warm-up
    for (int i = 0; i < 500; i++) {
        elevon_fcc_step(fcc, i * 10000, &inceptor, 30000, 42000, 240, gyro, accel, 0.01);
    }

    // Benchmark loop
    for (int i = 0; i < SAMPLES; i++) {
        clock_t t0 = clock();
        elevon_fcc_step(fcc, (i + 500) * 10000, &inceptor, 30000, 42000, 240, gyro, accel, 0.01);
        clock_t t1 = clock();
        double elapsed_us = ((double)(t1 - t0) / CLOCKS_PER_SEC) * 1e6;
        latencies_us[i] = elapsed_us;
    }

    qsort(latencies_us, SAMPLES, sizeof(double), cmp_double);

    double p50 = latencies_us[(int)(SAMPLES * 0.50)];
    double p90 = latencies_us[(int)(SAMPLES * 0.90)];
    double p99 = latencies_us[(int)(SAMPLES * 0.99)];
    double p999 = latencies_us[(int)(SAMPLES * 0.999)];
    double max_lat = latencies_us[SAMPLES - 1];

    printf("Minor Cycle Execution Latency Statistics (%d runs, 10,000 us deadline):\n", SAMPLES);
    printf("  P50   (Median):   %6.2f us\n", p50);
    printf("  P90:              %6.2f us\n", p90);
    printf("  P99:              %6.2f us\n", p99);
    printf("  P99.9:            %6.2f us\n", p999);
    printf("  Max (Worst-Case): %6.2f us\n", max_lat);
    printf("  Hard Deadline:    10,000.00 us\n");
    printf("  Timing Margin:    %6.2f %%\n\n", (1.0 - (max_lat / 10000.0)) * 100.0);

    free(latencies_us);
    elevon_fcc_destroy(fcc);
    printf("Benchmark completed successfully.\n");
    return 0;
}
