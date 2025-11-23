#ifndef MULTI_FUSE_H
#define MULTI_FUSE_H

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

#include "../src/mathlib/mathlib.h"
#include "sensors.h"

#define MAX_SENSORS 32
#define MAX_DT_IMU 0.002
#define MAX_DT_GPS 0.01
#define K_ROBUST 3.0

struct fused_imu
{
    double t;
    double ax, ay, az;
    double wx, wy, wz;
    double P_acc[9];
    double P_gyro[9];
    int n_used;
};

struct fused_gps
{
    double t;
    double lat, lon, alt;
    double vn, ve, vu;
    double P_pos[9];
    double P_vel[9];
    int n_used;
};

void component_median(double vals[][3], int n, double out[3]);
void component_mad(double vals[][3], int n, const double med[3], double out[3]);
void accumulate_weighted(const double *W, const double *v, double *sumW, double *sumWv);
void build_diag3(const double diag[3], double out[9]);
double compute_alpha(const double z[3], const double med[3], const double mad[3], double k);

int fuse_imus(struct imu_sample **imu_arrays, int N, int samples_per_sensor, struct fused_imu *out);
int fuse_gps(struct gps_sample **gps_arrays, int N, int samples_per_sensor, struct fused_gps *out);

#endif
