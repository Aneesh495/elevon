#include "elevon_afdx.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>

static void test_vl_bag_and_seq(void) {
    elevon_afdx_vl_t vl;
    elevon_afdx_vl_init(&vl, 101, 10, 256, 400); // 10 ms BAG = 10,000 us

    // First frame at t=0
    assert(elevon_afdx_vl_can_tx(&vl, 0));
    uint8_t seq0 = elevon_afdx_vl_record_tx(&vl, 0);
    assert(seq0 == 0);
    assert(vl.bag_violations == 0);

    // Frame attempted 4 ms later (too early: BAG violation)
    assert(!elevon_afdx_vl_can_tx(&vl, 4000));
    uint8_t seq1 = elevon_afdx_vl_record_tx(&vl, 4000);
    assert(seq1 == 1);
    assert(vl.bag_violations == 1);

    // Frame at t=14,000 us (10 ms after t=4000: compliant)
    assert(elevon_afdx_vl_can_tx(&vl, 14000));
    uint8_t seq2 = elevon_afdx_vl_record_tx(&vl, 14000);
    assert(seq2 == 2);
    assert(vl.bag_violations == 1);

    printf("test_vl_bag_and_seq: PASSED\n");
}

static void test_frame_encoding_and_crc(void) {
    char buf[512];
    const char *payload = "AVIONICS_PAYLOAD_TEST_DATA";
    size_t payload_len = strlen(payload);

    size_t wire_len = elevon_afdx_encode_frame(
        201, 42, 50201, payload, payload_len, buf, sizeof(buf)
    );
    assert(wire_len >= ELEVON_AFDX_MIN_FRAME_LEN);

    uint16_t out_vl;
    uint8_t out_seq;
    uint16_t out_port;
    const void *out_payload = NULL;
    size_t out_len = 0;

    elevon_status_t dec_res = elevon_afdx_decode_frame(
        buf, wire_len, &out_vl, &out_seq, &out_port, &out_payload, &out_len
    );
    assert(dec_res == ELEVON_OK);
    assert(out_vl == 201);
    assert(out_seq == 42);
    assert(out_port == 50201);
    assert(out_len == payload_len);
    assert(memcmp(out_payload, payload, payload_len) == 0);

    // Corrupt one bit in payload
    buf[45] ^= 0x01;
    dec_res = elevon_afdx_decode_frame(
        buf, wire_len, &out_vl, &out_seq, &out_port, &out_payload, &out_len
    );
    assert(dec_res == ELEVON_ERR_CRC_MISMATCH);

    printf("test_frame_encoding_and_crc: PASSED\n");
}

static void test_redundancy_first_valid(void) {
    elevon_afdx_redundancy_t red;
    elevon_afdx_redundancy_init(&red, 101);

    // Sequence 0: arrives on Network A first at t=100
    bool rx_a0 = elevon_afdx_process_rx(&red, 0, true, 100);
    assert(rx_a0); // Accepted as first valid

    // Duplicate arrives on Network B at t=160
    bool rx_b0 = elevon_afdx_process_rx(&red, 0, false, 160);
    assert(!rx_b0); // Redundant duplicate dropped
    assert(red.redundant_discards == 1);
    assert(red.max_skew_us == 60);

    // Sequence 1: Network A drops out (cable cut); arrives ONLY on Network B
    bool rx_b1 = elevon_afdx_process_rx(&red, 1, false, 10150);
    assert(rx_b1); // Seamlessly accepted over Network B
    assert(red.accepted_frames == 2);

    printf("test_redundancy_first_valid: PASSED\n");
}

int main(void) {
    printf("=== Starting AFDX Unit Tests ===\n");
    test_vl_bag_and_seq();
    test_frame_encoding_and_crc();
    test_redundancy_first_valid();
    printf("All AFDX tests passed successfully.\n");
    return 0;
}
