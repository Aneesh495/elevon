#include "elevon_afdx.h"
#include "elevon_crc32.h"
#include <string.h>

void elevon_afdx_vl_init(
    elevon_afdx_vl_t *vl,
    uint16_t vl_id,
    uint32_t bag_ms,
    uint32_t l_max_bytes,
    uint32_t max_jitter_us
) {
    if (!vl) return;
    vl->vl_id = vl_id;
    vl->bag_ms = bag_ms;
    vl->l_max_bytes = l_max_bytes;
    vl->max_jitter_us = max_jitter_us;
    vl->last_tx_us = 0;
    vl->seq_num = 0;
    vl->tx_count = 0;
    vl->bag_violations = 0;
}

bool elevon_afdx_vl_can_tx(const elevon_afdx_vl_t *vl, uint64_t now_us) {
    if (!vl || vl->last_tx_us == 0) return true;
    uint64_t bag_us = (uint64_t)vl->bag_ms * 1000ULL;
    return (now_us - vl->last_tx_us) >= bag_us;
}

uint8_t elevon_afdx_vl_record_tx(elevon_afdx_vl_t *vl, uint64_t now_us) {
    if (!vl) return 0;
    if (!elevon_afdx_vl_can_tx(vl, now_us)) {
        vl->bag_violations++;
    }
    vl->last_tx_us = now_us;
    vl->tx_count++;
    uint8_t seq = vl->seq_num;
    vl->seq_num = (vl->seq_num == 255) ? 0 : (vl->seq_num + 1);
    return seq;
}

void elevon_afdx_redundancy_init(elevon_afdx_redundancy_t *red, uint16_t vl_id) {
    if (!red) return;
    memset(red, 0, sizeof(*red));
    red->vl_id = vl_id;
}

bool elevon_afdx_process_rx(
    elevon_afdx_redundancy_t *red,
    uint8_t seq_num,
    bool is_net_a,
    uint64_t arrival_us
) {
    if (!red) return false;

    if (is_net_a) {
        red->last_arrival_a_us = arrival_us;
    } else {
        red->last_arrival_b_us = arrival_us;
    }

    if (red->last_arrival_a_us > 0 && red->last_arrival_b_us > 0) {
        uint64_t skew = (red->last_arrival_a_us >= red->last_arrival_b_us)
            ? (red->last_arrival_a_us - red->last_arrival_b_us)
            : (red->last_arrival_b_us - red->last_arrival_a_us);
        if (skew > red->max_skew_us) {
            red->max_skew_us = skew;
        }
    }

    if (!red->first_frame_rx) {
        red->expected_seq = (seq_num == 255) ? 0 : (seq_num + 1);
        red->first_frame_rx = true;
        red->accepted_frames++;
        return true;
    }

    if (seq_num == red->expected_seq) {
        red->expected_seq = (red->expected_seq == 255) ? 0 : (red->expected_seq + 1);
        red->accepted_frames++;
        return true;
    }

    int diff = ((int)seq_num - (int)red->expected_seq + 256) % 256;
    if (diff >= 1 && diff <= 2) {
        red->dropped_frames += (uint64_t)diff;
        red->expected_seq = (seq_num == 255) ? 0 : (seq_num + 1);
        red->accepted_frames++;
        return true;
    }

    red->redundant_discards++;
    red->integrity_errors++;
    return false;
}

size_t elevon_afdx_encode_frame(
    uint16_t vl_id,
    uint8_t seq_num,
    uint16_t udp_dest_port,
    const void *payload,
    size_t payload_len,
    void *out_buf,
    size_t out_buf_capacity
) {
    // Layout:
    // Dest MAC (6 bytes): 03:00:00:00:VL_H:VL_L
    // Src MAC (6 bytes): 02:00:00:00:01:01
    // EtherType (2 bytes): 0x0800 (IPv4)
    // IP Multicast (20 bytes): 10.0.VL_H.VL_L
    // UDP (8 bytes): SrcPort 50000, DestPort
    // Payload (payload_len bytes)
    // Sequence Number (1 byte)
    // CRC-32C FCS (4 bytes)
    size_t total_len = 14 + 20 + 8 + payload_len + 1 + 4;
    if (total_len < ELEVON_AFDX_MIN_FRAME_LEN) {
        total_len = ELEVON_AFDX_MIN_FRAME_LEN;
    }
    if (total_len > out_buf_capacity || total_len > ELEVON_AFDX_MAX_FRAME_LEN) {
        return 0;
    }

    uint8_t *p = (uint8_t *)out_buf;
    memset(p, 0, total_len);

    // Dest MAC
    p[0] = 0x03; p[1] = 0x00; p[2] = 0x00; p[3] = 0x00;
    p[4] = (uint8_t)((vl_id >> 8) & 0xFF);
    p[5] = (uint8_t)(vl_id & 0xFF);

    // Src MAC
    p[6] = 0x02; p[7] = 0x00; p[8] = 0x00; p[9] = 0x00; p[10] = 0x01; p[11] = 0x01;

    // EtherType IPv4
    p[12] = 0x08; p[13] = 0x00;

    // IPv4 Header (20 bytes)
    p[14] = 0x45; // Version 4, IHL 5
    p[15] = 0x00; // DSCP
    uint16_t ip_len = (uint16_t)(20 + 8 + payload_len + 1);
    p[16] = (uint8_t)((ip_len >> 8) & 0xFF);
    p[17] = (uint8_t)(ip_len & 0xFF);
    p[23] = 17; // Protocol UDP
    // IP Dest: 10.0.VL_H.VL_L
    p[30] = 10; p[31] = 0; p[32] = p[4]; p[33] = p[5];

    // UDP Header
    p[34] = 0xC3; p[35] = 0x50; // SrcPort 50000
    p[36] = (uint8_t)((udp_dest_port >> 8) & 0xFF);
    p[37] = (uint8_t)(udp_dest_port & 0xFF);
    uint16_t udp_len = (uint16_t)(8 + payload_len + 1);
    p[38] = (uint8_t)((udp_len >> 8) & 0xFF);
    p[39] = (uint8_t)(udp_len & 0xFF);

    // Payload
    if (payload && payload_len > 0) {
        memcpy(&p[42], payload, payload_len);
    }

    // Sequence number placed right after payload
    p[42 + payload_len] = seq_num;

    // CRC-32C FCS across all preceding bytes
    uint32_t fcs = elevon_crc32c(p, total_len - 4);
    p[total_len - 4] = (uint8_t)(fcs & 0xFF);
    p[total_len - 3] = (uint8_t)((fcs >> 8) & 0xFF);
    p[total_len - 2] = (uint8_t)((fcs >> 16) & 0xFF);
    p[total_len - 1] = (uint8_t)((fcs >> 24) & 0xFF);

    return total_len;
}

elevon_status_t elevon_afdx_decode_frame(
    const void *buf,
    size_t buf_len,
    uint16_t *out_vl_id,
    uint8_t *out_seq_num,
    uint16_t *out_dest_port,
    const void **out_payload,
    size_t *out_payload_len
) {
    if (!buf || buf_len < ELEVON_AFDX_MIN_FRAME_LEN || buf_len > ELEVON_AFDX_MAX_FRAME_LEN) {
        return ELEVON_ERR_OUT_OF_BOUNDS;
    }

    const uint8_t *p = (const uint8_t *)buf;

    // Verify CRC32C
    uint32_t calc_crc = elevon_crc32c(p, buf_len - 4);
    uint32_t stored_crc = (uint32_t)p[buf_len - 4] |
                          ((uint32_t)p[buf_len - 3] << 8) |
                          ((uint32_t)p[buf_len - 2] << 16) |
                          ((uint32_t)p[buf_len - 1] << 24);
    if (calc_crc != stored_crc) {
        return ELEVON_ERR_CRC_MISMATCH;
    }

    uint16_t vl_id = ((uint16_t)p[4] << 8) | (uint16_t)p[5];
    uint16_t dest_port = ((uint16_t)p[36] << 8) | (uint16_t)p[37];
    uint16_t udp_len = ((uint16_t)p[38] << 8) | (uint16_t)p[39];

    if (udp_len < 9) {
        return ELEVON_ERR_INVALID_STATE;
    }
    size_t payload_len = udp_len - 9;
    uint8_t seq = p[42 + payload_len];

    if (out_vl_id) *out_vl_id = vl_id;
    if (out_seq_num) *out_seq_num = seq;
    if (out_dest_port) *out_dest_port = dest_port;
    if (out_payload) *out_payload = (payload_len > 0) ? &p[42] : NULL;
    if (out_payload_len) *out_payload_len = payload_len;

    return ELEVON_OK;
}
