#include "ekf.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

void test_quat_norm() {
    struct ekf_state s;
    double P[P_DIM];
    struct imu_sample imu = {0,0, 0,0,0, 0,0,0}; // no rotation, no accel
    ekf_init(&s, P, 0.0);
    for (int i=0;i<1000;++i) {
        ekf_propagate(&s, P, &imu, 0.01);
        double norm = sqrt(s.q[0]*s.q[0]+s.q[1]*s.q[1]+s.q[2]*s.q[2]+s.q[3]*s.q[3]);
        if (fabs(norm-1.0) > 1e-9) {
            printf("Quaternion not normalized at step %d: norm=%.12f\n", i, norm);
            exit(1);
        }
    }
    printf("Quaternion normalization test passed.\n");
}

void test_stationary() {
    struct ekf_state s;
    double P[P_DIM];
    struct imu_sample imu = {0,0, 0,0,0, 0,0,9.81}; // should cancel gravity
    ekf_init(&s, P, 0.0);
    for (int i=0;i<100;++i) {
        ekf_propagate(&s, P, &imu, 0.01);
    }
    printf("Final pos: %.6f %.6f %.6f\n", s.p[0], s.p[1], s.p[2]);
    printf("Final vel: %.6f %.6f %.6f\n", s.v[0], s.v[1], s.v[2]);
}

void test_constant_rotation() {
    struct ekf_state s;
    double P[P_DIM];
    ekf_init(&s, P, 0.0);
    struct imu_sample imu = {0,0, 0,0,0, 0,0,M_PI/2}; // 90 deg/s yaw
    for (int i=0;i<2;++i) {
        ekf_propagate(&s, P, &imu, 1.0); // 2 seconds
    }
    // after 2s at 90 deg/s, yaw ~180°
    // Extract yaw from quaternion
    double yaw = atan2(2*(s.q[0]*s.q[3]+s.q[1]*s.q[2]),
                       1-2*(s.q[2]*s.q[2]+s.q[3]*s.q[3]));
    printf("Yaw after 2s: %.3f rad (expected ~%.3f rad)\n", yaw, M_PI);
}


int main() {
    struct imu_sample imus[10000];
    struct gps_sample gps[10000]; // not used yet
    int n = read_oxts_csv("data/oxts.csv", imus, 10000, gps, 10000);

    struct ekf_state state;
    double P[P_DIM];
    ekf_init(&state, P, imus[0].t);

    FILE *f = fopen("imu_prop.csv","w");
    fprintf(f,"t,px,py,pz,vx,vy,vz,qw,qx,qy,qz\n");

    for (int i=1;i<n;++i) {
        double dt = imus[i].t - imus[i-1].t;
        ekf_propagate(&state, P, &imus[i], dt);

        fprintf(f,"%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                state.t, state.p[0],state.p[1],state.p[2],
                state.v[0],state.v[1],state.v[2],
                state.q[0],state.q[1],state.q[2],state.q[3]);
    }
    fclose(f);
    return 0;
}

