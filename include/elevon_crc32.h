#ifndef ELEVON_CRC32_H
#define ELEVON_CRC32_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Castagnoli CRC-32C polynomial (0x1EDC6F41).
 * Used for ARINC 664 packet framing and Flight Data Recorder integrity checks.
 */
#define ELEVON_CRC32C_POLYNOMIAL 0x82F63B78U

/*
 * Initializes the static 256-entry CRC32C lookup table.
 * Idempotent: can be called multiple times safely.
 */
void elevon_crc32c_init(void);

/*
 * Computes the 32-bit CRC32C checksum of a byte buffer.
 * If initial_crc is 0, standard Castagnoli initialization (0xFFFFFFFF) is used.
 */
uint32_t elevon_crc32c_update(uint32_t current_crc, const void *data, size_t length);

/*
 * Computes a standalone CRC32C for a buffer from start to finish.
 */
uint32_t elevon_crc32c(const void *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* ELEVON_CRC32_H */
