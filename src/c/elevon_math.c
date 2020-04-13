#include "elevon_math.h"

elevon_vec3_t elevon_vec3_add(elevon_vec3_t a, elevon_vec3_t b) {
    return (elevon_vec3_t){a.x + b.x, a.y + b.y, a.z + b.z};
}

elevon_vec3_t elevon_vec3_sub(elevon_vec3_t a, elevon_vec3_t b) {
    return (elevon_vec3_t){a.x - b.x, a.y - b.y, a.z - b.z};
}

elevon_vec3_t elevon_vec3_scale(elevon_vec3_t v, double scale) {
    return (elevon_vec3_t){v.x * scale, v.y * scale, v.z * scale};
}

double elevon_vec3_dot(elevon_vec3_t a, elevon_vec3_t b) {
    return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

elevon_vec3_t elevon_vec3_cross(elevon_vec3_t a, elevon_vec3_t b) {
    return (elevon_vec3_t){
        (a.y * b.z) - (a.z * b.y),
        (a.z * b.x) - (a.x * b.z),
        (a.x * b.y) - (a.y * b.x)
    };
}

double elevon_vec3_norm(elevon_vec3_t v) {
    return sqrt((v.x * v.x) + (v.y * v.y) + (v.z * v.z));
}

elevon_vec3_t elevon_vec3_normalize(elevon_vec3_t v) {
    double n = elevon_vec3_norm(v);
    if (n < 1e-12) {
        return (elevon_vec3_t){0.0, 0.0, 0.0};
    }
    double inv = 1.0 / n;
    return (elevon_vec3_t){v.x * inv, v.y * inv, v.z * inv};
}

elevon_mat3_t elevon_mat3_identity(void) {
    elevon_mat3_t m = {{{0}}};
    m.m[0][0] = 1.0;
    m.m[1][1] = 1.0;
    m.m[2][2] = 1.0;
    return m;
}

elevon_vec3_t elevon_mat3_mult_vec(elevon_mat3_t m, elevon_vec3_t v) {
    return (elevon_vec3_t){
        (m.m[0][0] * v.x) + (m.m[0][1] * v.y) + (m.m[0][2] * v.z),
        (m.m[1][0] * v.x) + (m.m[1][1] * v.y) + (m.m[1][2] * v.z),
        (m.m[2][0] * v.x) + (m.m[2][1] * v.y) + (m.m[2][2] * v.z)
    };
}

elevon_mat3_t elevon_mat3_mult(elevon_mat3_t a, elevon_mat3_t b) {
    elevon_mat3_t res = {{{0}}};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            res.m[i][j] = (a.m[i][0] * b.m[0][j]) +
                          (a.m[i][1] * b.m[1][j]) +
                          (a.m[i][2] * b.m[2][j]);
        }
    }
    return res;
}

elevon_mat3_t elevon_mat3_transpose(elevon_mat3_t m) {
    elevon_mat3_t res;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            res.m[i][j] = m.m[j][i];
        }
    }
    return res;
}

double elevon_mat3_det(elevon_mat3_t m) {
    return (m.m[0][0] * ((m.m[1][1] * m.m[2][2]) - (m.m[1][2] * m.m[2][1]))) -
           (m.m[0][1] * ((m.m[1][0] * m.m[2][2]) - (m.m[1][2] * m.m[2][0]))) +
           (m.m[0][2] * ((m.m[1][0] * m.m[2][1]) - (m.m[1][1] * m.m[2][0])));
}

elevon_quat_t elevon_quat_identity(void) {
    return (elevon_quat_t){1.0, 0.0, 0.0, 0.0};
}

elevon_quat_t elevon_quat_normalize(elevon_quat_t q) {
    double n = sqrt((q.w * q.w) + (q.x * q.x) + (q.y * q.y) + (q.z * q.z));
    if (n < 1e-12) {
        return elevon_quat_identity();
    }
    double inv = 1.0 / n;
    return (elevon_quat_t){q.w * inv, q.x * inv, q.y * inv, q.z * inv};
}

elevon_quat_t elevon_quat_conjugate(elevon_quat_t q) {
    return (elevon_quat_t){q.w, -q.x, -q.y, -q.z};
}

elevon_quat_t elevon_quat_mult(elevon_quat_t a, elevon_quat_t b) {
    return (elevon_quat_t){
        (a.w * b.w) - (a.x * b.x) - (a.y * b.y) - (a.z * b.z),
        (a.w * b.x) + (a.x * b.w) + (a.y * b.z) - (a.z * b.y),
        (a.w * b.y) - (a.x * b.z) + (a.y * b.w) + (a.z * b.x),
        (a.w * b.z) + (a.x * b.y) - (a.y * b.x) + (a.z * b.w)
    };
}

elevon_vec3_t elevon_quat_rotate(elevon_quat_t q, elevon_vec3_t v) {
    elevon_quat_t p = {0.0, v.x, v.y, v.z};
    elevon_quat_t q_norm = elevon_quat_normalize(q);
    elevon_quat_t q_conj = elevon_quat_conjugate(q_norm);
    elevon_quat_t rotated = elevon_quat_mult(elevon_quat_mult(q_norm, p), q_conj);
    return (elevon_vec3_t){rotated.x, rotated.y, rotated.z};
}

void elevon_quat_to_euler(elevon_quat_t q, double *roll_rad, double *pitch_rad, double *yaw_rad) {
    double sinr_cosp = 2.0 * ((q.w * q.x) + (q.y * q.z));
    double cosr_cosp = 1.0 - (2.0 * ((q.x * q.x) + (q.y * q.y)));
    if (roll_rad) {
        *roll_rad = atan2(sinr_cosp, cosr_cosp);
    }

    double sinp = 2.0 * ((q.w * q.y) - (q.z * q.x));
    if (pitch_rad) {
        if (fabs(sinp) >= 1.0) {
            *pitch_rad = copysign(ELEVON_PI / 2.0, sinp);
        } else {
            *pitch_rad = asin(sinp);
        }
    }

    double siny_cosp = 2.0 * ((q.w * q.z) + (q.x * q.y));
    double cosy_cosp = 1.0 - (2.0 * ((q.y * q.y) + (q.z * q.z)));
    if (yaw_rad) {
        *yaw_rad = atan2(siny_cosp, cosy_cosp);
    }
}

elevon_quat_t elevon_quat_from_euler(double roll_rad, double pitch_rad, double yaw_rad) {
    double cy = cos(yaw_rad * 0.5);
    double sy = sin(yaw_rad * 0.5);
    double cp = cos(pitch_rad * 0.5);
    double sp = sin(pitch_rad * 0.5);
    double cr = cos(roll_rad * 0.5);
    double sr = sin(roll_rad * 0.5);

    return (elevon_quat_t){
        (cr * cp * cy) + (sr * sp * sy),
        (sr * cp * cy) - (cr * sp * sy),
        (cr * sp * cy) + (sr * cp * sy),
        (cr * cp * sy) - (sr * sp * cy)
    };
}

elevon_quat_t elevon_quat_integrate(elevon_quat_t q, elevon_vec3_t omega_rad_s, double dt_sec) {
    elevon_quat_t omega_q = {0.0, omega_rad_s.x, omega_rad_s.y, omega_rad_s.z};
    elevon_quat_t q_dot = elevon_quat_mult(q, omega_q);

    double half_dt = 0.5 * dt_sec;
    elevon_quat_t q_new = {
        q.w + (q_dot.w * half_dt),
        q.x + (q_dot.x * half_dt),
        q.y + (q_dot.y * half_dt),
        q.z + (q_dot.z * half_dt)
    };
    return elevon_quat_normalize(q_new);
}
