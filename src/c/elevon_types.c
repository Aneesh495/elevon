#include "elevon_types.h"
#include <string.h>

const char *elevon_status_str(elevon_status_t status) {
    switch (status) {
        case ELEVON_OK:                    return "ELEVON_OK";
        case ELEVON_ERR_NULL_PTR:          return "ELEVON_ERR_NULL_PTR";
        case ELEVON_ERR_OUT_OF_BOUNDS:     return "ELEVON_ERR_OUT_OF_BOUNDS";
        case ELEVON_ERR_BYZANTINE_FAULT:   return "ELEVON_ERR_BYZANTINE_FAULT";
        case ELEVON_ERR_BAG_VIOLATION:     return "ELEVON_ERR_BAG_VIOLATION";
        case ELEVON_ERR_FDR_CORRUPT:       return "ELEVON_ERR_FDR_CORRUPT";
        case ELEVON_ERR_CRC_MISMATCH:      return "ELEVON_ERR_CRC_MISMATCH";
        case ELEVON_ERR_HYDRAULIC_LOSS:    return "ELEVON_ERR_HYDRAULIC_LOSS";
        case ELEVON_ERR_ACTUATOR_STALL:    return "ELEVON_ERR_ACTUATOR_STALL";
        case ELEVON_ERR_CCDL_TIMEOUT:      return "ELEVON_ERR_CCDL_TIMEOUT";
        case ELEVON_ERR_AIR_DATA_DISAGREE: return "ELEVON_ERR_AIR_DATA_DISAGREE";
        case ELEVON_ERR_INVALID_STATE:     return "ELEVON_ERR_INVALID_STATE";
        default:                           return "ELEVON_ERR_UNKNOWN";
    }
}

const char *elevon_law_mode_str(elevon_law_mode_t mode) {
    switch (mode) {
        case ELEVON_LAW_NORMAL:     return "NORMAL_LAW";
        case ELEVON_LAW_ALTERNATE:  return "ALTERNATE_LAW";
        case ELEVON_LAW_DIRECT:     return "DIRECT_LAW";
        case ELEVON_LAW_MECHANICAL: return "MECHANICAL_BACKUP";
        default:                    return "UNKNOWN_LAW";
    }
}

void elevon_surfaces_zero(elevon_surfaces_t *surf) {
    if (surf) {
        memset(surf, 0, sizeof(*surf));
    }
}

void elevon_inceptor_zero(elevon_inceptor_t *inc) {
    if (inc) {
        memset(inc, 0, sizeof(*inc));
    }
}
