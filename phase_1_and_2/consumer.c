#include "consumer.h"
#include "ekf.h"
#include "logger.h"
#include "sensors.h"
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <time.h>

/* Default GPS measurement noise (standard deviation) */
static const double GPS_POS_STD = 1.0; // meters
static const double GPS_VEL_STD = 0.2; // m/s

void *consumer_thread(void *arg)
{
    struct consumer_args *cargs = (struct consumer_args *)arg;
    struct ekf_state state;
    double P[P_DIM];

    /* Wait for first IMU sample */
    struct imu_sample first_imu;
    while (rb_is_empty(cargs->imu_rb)) { usleep(1000); }
    rb_peek(cargs->imu_rb, &first_imu);
    ekf_init(&state, P, first_imu.t);

    open_state_log("imu_prop.csv");
    open_gps_log("gps_updates.csv");

    double last_time = state.t;

    size_t processed = 0;
    while (1)
    {
        /* Prefer IMU pops: process available IMU samples quickly */
        struct imu_sample imu;
        if (rb_pop(cargs->imu_rb, &imu) == 0) {
            double dt = imu.t - last_time;
            if (dt <= 0.0) dt = 0.01;
            ekf_propagate(&state, P, &imu, dt);
            last_time = state.t;
            log_state(&state, P);
            processed++;
            continue; // go back to pop more IMU
        }

        /* If no IMU, check GPS buffer */
        struct gps_sample g;
        if (rb_pop(cargs->gps_rb, &g) == 0) {
            /* Before applying GPS update, propagate IMU up to gps.t (if available) */
            struct imu_sample imu2;
            while (!rb_is_empty(cargs->imu_rb)) {
                /* peek to see next imu timestamp */
                rb_peek(cargs->imu_rb, &imu2);
                if (imu2.t <= g.t)
                {
                    rb_pop(cargs->imu_rb, &imu2);
                    double dt2 = imu2.t - last_time;
                    if (dt2 <= 0.0) dt2 = 0.01;
                    ekf_propagate(&state, P, &imu2, dt2);
                    last_time = state.t;
                    log_state(&state, P);
                }
                else
                    break;
            }

            /* Convert GPS lat/lon/alt -> ENU using ref (assumed initialized) */
            double pos_enu[3];
            latlon_to_enu(g.lat, g.lon, g.alt, pos_enu);

            /* Convert NED velocities (vn,ve,vu) to ENU: N->y, E->x, U->z? 
               KITTI/your struct: vn = north, ve = east, vu = up
               ENU coordinates: x = east, y = north, z = up. So mapping:
               v_east = ve -> x
               v_north = vn -> y
               v_up = vu -> z
            */
            double vel_enu[3];
            vel_enu[0] = g.ve; // east -> x
            vel_enu[1] = g.vn; // north -> y
            vel_enu[2] = g.vu; // up -> z

            /* Build Rpos and Rvel 3x3 diagonal covariances */
            double Rpos[9] = { GPS_POS_STD*GPS_POS_STD, 0,0, 0, GPS_POS_STD*GPS_POS_STD, 0, 0,0, GPS_POS_STD*GPS_POS_STD};
            double Rvel[9] = { GPS_VEL_STD*GPS_VEL_STD, 0,0, 0, GPS_VEL_STD*GPS_VEL_STD, 0, 0,0, GPS_VEL_STD*GPS_VEL_STD};

            double maha_pos = 0.0; int accepted_pos = 0;
            double maha_vel = 0.0; int accepted_vel = 0;

            ekf_update_gps(&state, P, pos_enu, vel_enu, Rpos, Rvel, &maha_pos, &accepted_pos, &maha_vel, &accepted_vel);

            /* Compute innovation vectors for logging: innovation = z - h(x) (we computed them inside ekf_update_gps earlier; recompute here) */
            double innov_pos[3] = { pos_enu[0] - state.p[0], pos_enu[1] - state.p[1], pos_enu[2] - state.p[2] };
            double innov_vel[3] = { vel_enu[0] - state.v[0], vel_enu[1] - state.v[1], vel_enu[2] - state.v[2] };

            log_gps_update(g.t, pos_enu, innov_pos, maha_pos, accepted_pos, vel_enu, innov_vel, maha_vel, accepted_vel);

            continue;
        }

        /* If nothing available, check termination condition */
        if (*(cargs->producers_done) >= 2 && rb_is_empty(cargs->imu_rb) && rb_is_empty(cargs->gps_rb)) {
            break;
        }
        usleep(1000);
    }

    close_state_log();
    close_gps_log();
    printf("Consumer finished, processed %zu IMU samples\n", processed);
    return NULL;
}

