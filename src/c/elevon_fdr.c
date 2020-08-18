#include "elevon_fdr.h"
#include "elevon_crc32.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>

elevon_status_t elevon_fdr_init_file(const char *filepath, uint32_t capacity_records) {
    if (!filepath || capacity_records == 0) {
        return ELEVON_ERR_NULL_PTR;
    }

    int fd = open(filepath, O_RDWR | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) {
        return ELEVON_ERR_INVALID_STATE;
    }

    elevon_fdr_superblock_t sb;
    sb.magic = ELEVON_FDR_MAGIC;
    sb.version = ELEVON_FDR_VERSION;
    sb.record_size = (uint32_t)sizeof(elevon_fdr_raw_record_t);
    sb.capacity_records = capacity_records;
    sb.created_time_us = 1000000ULL;
    sb.header_crc32c = elevon_crc32c(&sb, sizeof(sb) - 4);

    if (write(fd, &sb, sizeof(sb)) != (ssize_t)sizeof(sb)) {
        close(fd);
        return ELEVON_ERR_INVALID_STATE;
    }

    // Zero-fill file for circular writes
    elevon_fdr_raw_record_t zero_rec;
    memset(&zero_rec, 0, sizeof(zero_rec));
    for (uint32_t i = 0; i < capacity_records; i++) {
        if (write(fd, &zero_rec, sizeof(zero_rec)) != (ssize_t)sizeof(zero_rec)) {
            close(fd);
            return ELEVON_ERR_INVALID_STATE;
        }
    }

    close(fd);
    return ELEVON_OK;
}

elevon_status_t elevon_fdr_write_record(
    const char *filepath,
    uint64_t record_index,
    const elevon_fdr_raw_record_t *record
) {
    if (!filepath || !record) {
        return ELEVON_ERR_NULL_PTR;
    }

    int fd = open(filepath, O_RDWR);
    if (fd < 0) {
        return ELEVON_ERR_INVALID_STATE;
    }

    elevon_fdr_superblock_t sb;
    if (read(fd, &sb, sizeof(sb)) != (ssize_t)sizeof(sb)) {
        close(fd);
        return ELEVON_ERR_FDR_CORRUPT;
    }

    if (sb.magic != ELEVON_FDR_MAGIC) {
        close(fd);
        return ELEVON_ERR_FDR_CORRUPT;
    }

    uint32_t slot = (uint32_t)(record_index % sb.capacity_records);
    off_t offset = (off_t)sizeof(sb) + (off_t)(slot * sizeof(elevon_fdr_raw_record_t));

    elevon_fdr_raw_record_t rec_to_write = *record;
    rec_to_write.crc32c = elevon_crc32c(&rec_to_write, sizeof(rec_to_write) - 4);

    if (pwrite(fd, &rec_to_write, sizeof(rec_to_write), offset) != (ssize_t)sizeof(rec_to_write)) {
        close(fd);
        return ELEVON_ERR_INVALID_STATE;
    }

    close(fd);
    return ELEVON_OK;
}

elevon_status_t elevon_fdr_recover(
    const char *filepath,
    uint64_t *out_valid_records,
    uint64_t *out_corrupt_records
) {
    if (!filepath || !out_valid_records || !out_corrupt_records) {
        return ELEVON_ERR_NULL_PTR;
    }

    *out_valid_records = 0;
    *out_corrupt_records = 0;

    int fd = open(filepath, O_RDONLY);
    if (fd < 0) {
        return ELEVON_ERR_INVALID_STATE;
    }

    elevon_fdr_superblock_t sb;
    if (read(fd, &sb, sizeof(sb)) != (ssize_t)sizeof(sb)) {
        close(fd);
        return ELEVON_ERR_FDR_CORRUPT;
    }

    if (sb.magic != ELEVON_FDR_MAGIC) {
        close(fd);
        return ELEVON_ERR_FDR_CORRUPT;
    }

    elevon_fdr_raw_record_t rec;
    for (uint32_t i = 0; i < sb.capacity_records; i++) {
        if (read(fd, &rec, sizeof(rec)) != (ssize_t)sizeof(rec)) {
            break;
        }

        // Check if non-zero record
        if (rec.timestamp_us > 0) {
            uint32_t calc_crc = elevon_crc32c(&rec, sizeof(rec) - 4);
            if (calc_crc == rec.crc32c) {
                (*out_valid_records)++;
            } else {
                (*out_corrupt_records)++;
            }
        }
    }

    close(fd);
    return ELEVON_OK;
}
