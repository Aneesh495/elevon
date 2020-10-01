#include "elevon_fdr.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <unistd.h>

static void test_fdr_init_and_write(void) {
    const char *test_path = "/tmp/elevon_test_journal.fdr";
    unlink(test_path);

    elevon_status_t status = elevon_fdr_init_file(test_path, 100);
    assert(status == ELEVON_OK);

    // Write 50 records
    elevon_fdr_raw_record_t rec;
    memset(&rec, 0, sizeof(rec));

    for (int i = 0; i < 50; i++) {
        rec.timestamp_us = 1000000ULL + (uint64_t)i * 10000ULL;
        rec.frame_index = i;
        rec.cas_knots = 250.0f;
        rec.altitude_m = 3000.0f;
        rec.load_factor_nz = 1.0f;
        status = elevon_fdr_write_record(test_path, i, &rec);
        assert(status == ELEVON_OK);
    }

    uint64_t valid = 0, corrupt = 0;
    status = elevon_fdr_recover(test_path, &valid, &corrupt);
    assert(status == ELEVON_OK);
    assert(valid == 50);
    assert(corrupt == 0);

    unlink(test_path);
    printf("test_fdr_init_and_write: PASSED\n");
}

static void test_fdr_circular_rollover(void) {
    const char *test_path = "/tmp/elevon_rollover.fdr";
    unlink(test_path);

    // Capacity 50 records
    elevon_status_t status = elevon_fdr_init_file(test_path, 50);
    assert(status == ELEVON_OK);

    elevon_fdr_raw_record_t rec;
    memset(&rec, 0, sizeof(rec));

    // Write 120 records (rolls over twice)
    for (int i = 0; i < 120; i++) {
        rec.timestamp_us = 2000000ULL + (uint64_t)i * 10000ULL;
        rec.frame_index = i;
        rec.cas_knots = 240.0f + (float)(i % 20);
        status = elevon_fdr_write_record(test_path, i, &rec);
        assert(status == ELEVON_OK);
    }

    uint64_t valid = 0, corrupt = 0;
    status = elevon_fdr_recover(test_path, &valid, &corrupt);
    assert(status == ELEVON_OK);
    assert(valid == 50); // Exactly capacity
    assert(corrupt == 0);

    unlink(test_path);
    printf("test_fdr_circular_rollover: PASSED\n");
}

int main(void) {
    printf("=== Starting FDR Unit Tests ===\n");
    test_fdr_init_and_write();
    test_fdr_circular_rollover();
    printf("All FDR tests passed successfully.\n");
    return 0;
}
