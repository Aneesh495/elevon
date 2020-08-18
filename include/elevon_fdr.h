#ifndef ELEVON_FDR_H
#define ELEVON_FDR_H

#include "elevon_types.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ELEVON_FDR_MAGIC 0x454C4644U // "ELFD"
#define ELEVON_FDR_VERSION 1

#pragma pack(push, 1)
typedef struct {
    uint32_t magic;
    uint16_t version;
    uint32_t record_size;
    uint32_t capacity_records;
    uint64_t created_time_us;
    uint32_t header_crc32c;
} elevon_fdr_superblock_t;

typedef struct {
    uint64_t timestamp_us;
    uint64_t frame_index;
    float cas_knots;
    float mach;
    float altitude_m;
    float pitch_deg;
    float roll_deg;
    float heading_deg;
    float pitch_rate_deg_s;
    float roll_rate_deg_s;
    float yaw_rate_deg_s;
    float load_factor_nz;
    float elev_left_deg;
    float elev_right_deg;
    float aileron_left_deg;
    float aileron_right_deg;
    float rudder_deg;
    float ths_deg;
    float pitch_stick_norm;
    float roll_stick_norm;
    float rudder_pedal_norm;
    uint8_t active_law_mode;
    uint8_t prot_flags;
    uint8_t lane_health_bits;
    uint8_t reserved;
    uint32_t crc32c;
} elevon_fdr_raw_record_t;
#pragma pack(pop)

/* Formats and initializes an FDR journal file on disk */
elevon_status_t elevon_fdr_init_file(const char *filepath, uint32_t capacity_records);

/* Appends a raw record to the circular file journal */
elevon_status_t elevon_fdr_write_record(
    const char *filepath,
    uint64_t record_index,
    const elevon_fdr_raw_record_t *record
);

/* Scans and recovers records from an FDR journal file */
elevon_status_t elevon_fdr_recover(
    const char *filepath,
    uint64_t *out_valid_records,
    uint64_t *out_corrupt_records
);

#ifdef __cplusplus
}
#endif

#endif /* ELEVON_FDR_H */
