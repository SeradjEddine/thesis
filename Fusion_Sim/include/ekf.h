#ifndef EKF_H
#define EKF_H

#include <stddef.h>
#include "sensors.h"

#define STATE_DIM 15
#define P_DIM (STATE_DIM * STATE_DIM)
#define IDX(r,c) ((r)*STATE_DIM + (c))
#define MAHALANOBIS_THRESH_POS 16.27 
#define MAHALANOBIS_THRESH_VEL 11.34

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct ekf_state
{
    double t;       // timestamp (s)
    double p[3];    // position (m)
    double v[3];    // velocity (m/s)
    double q[4];    // quaternion (w,x,y,z)
    double bg[3];   // gyro bias (rad/s)
    double ba[3];   // accel bias (m/s^2)
};

void ekf_init(struct ekf_state *x, double P[P_DIM], double t0,
              const double *init_pos_enu, const double *init_vel_enu);

int ekf_propagate(struct ekf_state *x, double P[P_DIM],
                  const struct imu_sample *imu, double dt);

void ekf_build_process_noise(double Qd[P_DIM], double dt,
                             double sigma_g, double sigma_a,
                             double sigma_bg_rw, double sigma_ba_rw);

int ekf_update_gps(struct ekf_state *x, double P[P_DIM],
                   const double gps_pos_enu[3], const double gps_vel_enu[3],
                   const double Rpos[9], const double Rvel[9],
                   double *out_mahalanobis_pos, int *out_accepted_pos,
                   double *out_mahalanobis_vel, int *out_accepted_vel);

void ekf_init_gps_ref(double lat0_deg, double lon0_deg, double alt0_m);

void latlon_to_enu(double lat_deg, double lon_deg, double alt_m, double enu_out[3]);

#endif

