#ifndef ELEVON_MATH_H
#define ELEVON_MATH_H

#include "elevon_types.h"
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ELEVON_PI 3.14159265358979323846
#define ELEVON_DEG_TO_RAD (ELEVON_PI / 180.0)
#define ELEVON_RAD_TO_DEG (180.0 / ELEVON_PI)
#define ELEVON_GRAVITY_G0 9.80665

/* Vector operations */
elevon_vec3_t elevon_vec3_add(elevon_vec3_t a, elevon_vec3_t b);
elevon_vec3_t elevon_vec3_sub(elevon_vec3_t a, elevon_vec3_t b);
elevon_vec3_t elevon_vec3_scale(elevon_vec3_t v, double scale);
double elevon_vec3_dot(elevon_vec3_t a, elevon_vec3_t b);
elevon_vec3_t elevon_vec3_cross(elevon_vec3_t a, elevon_vec3_t b);
double elevon_vec3_norm(elevon_vec3_t v);
elevon_vec3_t elevon_vec3_normalize(elevon_vec3_t v);

/* Matrix operations */
elevon_mat3_t elevon_mat3_identity(void);
elevon_vec3_t elevon_mat3_mult_vec(elevon_mat3_t m, elevon_vec3_t v);
elevon_mat3_t elevon_mat3_mult(elevon_mat3_t a, elevon_mat3_t b);
elevon_mat3_t elevon_mat3_transpose(elevon_mat3_t m);
double elevon_mat3_det(elevon_mat3_t m);

/* Quaternion operations */
elevon_quat_t elevon_quat_identity(void);
elevon_quat_t elevon_quat_normalize(elevon_quat_t q);
elevon_quat_t elevon_quat_conjugate(elevon_quat_t q);
elevon_quat_t elevon_quat_mult(elevon_quat_t a, elevon_quat_t b);
elevon_vec3_t elevon_quat_rotate(elevon_quat_t q, elevon_vec3_t v);
void elevon_quat_to_euler(elevon_quat_t q, double *roll_rad, double *pitch_rad, double *yaw_rad);
elevon_quat_t elevon_quat_from_euler(double roll_rad, double pitch_rad, double yaw_rad);

/* Dynamic integration: updates attitude quaternion using body rates (p, q, r) over dt */
elevon_quat_t elevon_quat_integrate(elevon_quat_t q, elevon_vec3_t omega_rad_s, double dt_sec);

#ifdef __cplusplus
}
#endif

#endif /* ELEVON_MATH_H */
