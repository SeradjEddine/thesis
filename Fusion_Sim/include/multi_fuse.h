#ifndef MULTI_FUSE_H
#define MULTI_FUSE_H

#include "sensors.h"

/*
 * Simple fused sample type for output; separate accel/gyro covariances.
 */
struct fused_imu {
    double t;
    double ax, ay, az;
    double gx, gy, gz;
    double P_acc[9];   // row-major 3x3
    double P_gyro[9];  // row-major 3x3
    int n_used;        // number of contributing sensors used
    struct fused_imu *next;
};

struct fused_gps {
    double t;
    double lat, lon, alt;
    double vn, ve, vu;
    double P_pos[9];   // 3x3 ENU or geodetic cov (user decides)
    double P_vel[9];
    int n_used;
    struct fused_gps *next;
};

/*
 * Fuse N IMU linked lists into a single fused linked list.
 * - imu_heads: array of N heads (each is struct imu_node*).
 * - N: number of sensors
 * - max_dt: matching tolerance (seconds) to align timestamps
 * - default_R_acc_diag: pointer to 3 doubles (diag terms) used if per-sensor covariance not provided (can be NULL -> uses identity)
 * - default_R_gyro_diag: similar for gyro
 * Returns pointer to head of fused linked list (caller must free). Returns NULL on error or empty input.
 */
struct fused_imu *fuse_imus(struct imu_node **imu_heads, int N, double max_dt,
                            const double (*sensor_R_acc_diag)[3],
                            const double (*sensor_R_gyro_diag)[3]);

/*
 * Fuse N GPS linked lists into a single fused linked list.
 * Similar semantics as fuse_imus.
 */
struct fused_gps *fuse_gps(struct gps_node **gps_heads, int N, double max_dt,
                           const double (*sensor_R_pos_diag)[3],
                           const double (*sensor_R_vel_diag)[3]);

/* Free functions for fused lists */
void free_fused_imu_list(struct fused_imu *h);
void free_fused_gps_list(struct fused_gps *h);

#endif // MULTI_FUSE_H
