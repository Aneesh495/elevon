#include "elevon_crc32.h"
#include <stdbool.h>

static uint32_t s_crc32c_table[256];
static bool s_table_initialized = false;

void elevon_crc32c_init(void) {
    if (s_table_initialized) {
        return;
    }
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t crc = i;
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ ELEVON_CRC32C_POLYNOMIAL;
            } else {
                crc >>= 1;
            }
        }
        s_crc32c_table[i] = crc;
    }
    s_table_initialized = true;
}

uint32_t elevon_crc32c_update(uint32_t current_crc, const void *data, size_t length) {
    if (!s_table_initialized) {
        elevon_crc32c_init();
    }
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = ~current_crc;

    for (size_t i = 0; i < length; i++) {
        uint8_t table_index = (uint8_t)((crc ^ p[i]) & 0xFF);
        crc = (crc >> 8) ^ s_crc32c_table[table_index];
    }

    return ~crc;
}

uint32_t elevon_crc32c(const void *data, size_t length) {
    return elevon_crc32c_update(0, data, length);
}
