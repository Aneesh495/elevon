#ifndef ELEVON_AFDX_H
#define ELEVON_AFDX_H

#include "elevon_types.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ELEVON_AFDX_MIN_FRAME_LEN 64
#define ELEVON_AFDX_MAX_FRAME_LEN 1518

typedef struct {
    uint16_t vl_id;
    uint32_t bag_ms;
    uint32_t l_max_bytes;
    uint32_t max_jitter_us;
    uint64_t last_tx_us;
    bool has_transmitted;
    uint8_t seq_num;
    uint64_t tx_count;
    uint64_t bag_violations;
} elevon_afdx_vl_t;

typedef struct {
    uint16_t vl_id;
    uint8_t expected_seq;
    bool first_frame_rx;
    uint64_t accepted_frames;
    uint64_t integrity_errors;
    uint64_t dropped_frames;
    uint64_t redundant_discards;
    uint64_t max_skew_us;
    uint64_t last_arrival_a_us;
    uint64_t last_arrival_b_us;
} elevon_afdx_redundancy_t;

/* Initializes a Virtual Link shaper */
void elevon_afdx_vl_init(
    elevon_afdx_vl_t *vl,
    uint16_t vl_id,
    uint32_t bag_ms,
    uint32_t l_max_bytes,
    uint32_t max_jitter_us
);

/* Checks if transmission is BAG-compliant */
bool elevon_afdx_vl_can_tx(const elevon_afdx_vl_t *vl, uint64_t now_us);

/* Records transmission and increments 8-bit sequence number */
uint8_t elevon_afdx_vl_record_tx(elevon_afdx_vl_t *vl, uint64_t now_us);

/* Initializes AFDX Redundancy Manager */
void elevon_afdx_redundancy_init(elevon_afdx_redundancy_t *red, uint16_t vl_id);

/* Processes frame arrival across Network A or B (is_net_a == true for A, false for B) */
bool elevon_afdx_process_rx(
    elevon_afdx_redundancy_t *red,
    uint8_t seq_num,
    bool is_net_a,
    uint64_t arrival_us
);

/* Encodes AFDX binary frame into buffer */
size_t elevon_afdx_encode_frame(
    uint16_t vl_id,
    uint8_t seq_num,
    uint16_t udp_dest_port,
    const void *payload,
    size_t payload_len,
    void *out_buf,
    size_t out_buf_capacity
);

/* Decodes AFDX binary frame from buffer */
elevon_status_t elevon_afdx_decode_frame(
    const void *buf,
    size_t buf_len,
    uint16_t *out_vl_id,
    uint8_t *out_seq_num,
    uint16_t *out_dest_port,
    const void **out_payload,
    size_t *out_payload_len
);

#ifdef __cplusplus
}
#endif

#endif /* ELEVON_AFDX_H */
