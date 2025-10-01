#ifndef EKF_H
#define EKF_H

#include <stddef.h>

#define STATE_DIM 15
#define P_DIM (STATE_DIM * STATE_DIM)
#define IDX(r,c) ((r)*STATE_DIM + (c))
struct imu_sample; // defined in sensors.h

struct ekf_state {
    double t;       // timestamp (s)
    double p[3];    // position (m)
    double v[3];    // velocity (m/s)
    double q[4];    // quaternion (w,x,y,z) world <- body
    double bg[3];   // gyro bias (rad/s)
    double ba[3];   // accel bias (m/s^2)
};

/* Initialize EKF state and covariance P (row-major 15x15) */
void ekf_init(struct ekf_state *x, double P[P_DIM], double t0);

/*
 * Propagate EKF with a single IMU sample over dt seconds.
 * imu: bias/measurement fields expected in sensors.h
 * dt: positive time difference in seconds
 * Returns 0 on success, non-zero on failure (e.g., dt <= 0)
 */
int ekf_propagate(struct ekf_state *x, double P[P_DIM],
                  const struct imu_sample *imu, double dt);

/* Build discrete process noise Qd for time step dt using tunable params */
void ekf_build_process_noise(double Qd[P_DIM], double dt,
                             double sigma_g, double sigma_a,
                             double sigma_bg_rw, double sigma_ba_rw);

#endif // EKF_H

