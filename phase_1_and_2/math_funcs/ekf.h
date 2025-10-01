#ifndef EKF_H
#define EKF_H

#include <stddef.h>

#define STATE_DIM 15
#define P_DIM (STATE_DIM*STATE_DIM)

#ifndef IMU_SAMPLE
#define IMU_SAMPLE

 struct imu_sample {
    double t;    // seconds
    double ax, ay, az; // raw accel (body)
    double gx, gy, gz; // raw gyro (body)
 };

#endif

struct ekf_state {
    double t;       // timestamp

    double p[3];    // position in world (m)
    double v[3];    // velocity in world (m/s)
    double q[4];    // quaternion world <- body (w,x,y,z)

    double bg[3];   // gyro bias (rad/s)
    double ba[3];   // accel bias (m/s^2)
};

void ekf_init(struct ekf_state *x, double P[P_DIM], double t0);
int ekf_propagate(struct ekf_state *x, double P[P_DIM], const struct imu_sample *imu, double dt);

// Helper: fill process noise Qd (STATE_DIM x STATE_DIM) from noise params
void ekf_build_process_noise(double Qd[P_DIM], double dt,
                             double sigma_g, double sigma_a, double sigma_bg_rw, double sigma_ba_rw);

#endif // EKF_H

