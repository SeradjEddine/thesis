#ifndef MULTI_FUSE_H
#define MULTI_FUSE_H

#include "sensors.h"


struct fused_imu
{
    double t;
    double ax, ay, az;
    double gx, gy, gz;
    double P_acc[9];
    double P_gyro[9];
    int n_used;
    struct fused_imu *next;
};

struct fused_gps
{
    double t;
    double lat, lon, alt;
    double vn, ve, vu;
    double P_pos[9];
    double P_vel[9];
    int n_used;
    struct fused_gps *next;
};

struct fused_imu *fuse_imus(struct imu_node **imu_heads, int N, double max_dt,
                            const double (*sensor_R_acc_diag)[3],
                            const double (*sensor_R_gyro_diag)[3]);

struct fused_gps *fuse_gps(struct gps_node **gps_heads, int N, double max_dt,
                           const double (*sensor_R_pos_diag)[3],
                           const double (*sensor_R_vel_diag)[3]);

struct fused_gps *append_fused_gps(struct fused_gps **head, struct fused_gps **tail, struct fused_gps *node);

void free_fused_imu_list(struct fused_imu *h);

void free_fused_gps_list(struct fused_gps *h);

#endif
