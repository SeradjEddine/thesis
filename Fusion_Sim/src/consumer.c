#include "../include/fuzzy_supervisor.h"
#include "../include/consumer.h"
#include "../include/ekf.h"
#include "../include/logger.h"
#include "../include/sensors.h"
#include "../src/mathlib/mathlib.h"

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <string.h>
#include <errno.h>

#define PATH_MAX 100

// Default GPS measurement noise (standard deviation)
static const double GPS_POS_STD = 1.0; // meters
static const double GPS_VEL_STD = 0.2; // m/s

unsigned int interval_ms = 10;
struct timespec ts;

// Create all parent directories recursively
static void create_directory_recursive(const char *path)
{
    char tmp[PATH_MAX];
    snprintf(tmp, sizeof(tmp), "%s", path);

    for (char *p = tmp + 1; *p; p++)
    {
        if (*p == '/')
        {
            *p = '\0';
            mkdir(tmp, 0777);  // ignore EEXIST
            *p = '/';
        }
    }
    mkdir(tmp, 0777);  // create final directory
}
void *consumer_thread(void *arg)
{
    struct consumer_args *cargs = (struct consumer_args *)arg;
    struct ekf_state state;
    double P[P_DIM];

    double maha_pos = 0.0;
    double maha_vel = 0.0;
    double last_time;
    double dt;
    size_t processed;

    // Wait for first IMU sample
    struct imu_sample first_imu;
    while (rb_is_empty(cargs->imu_rb))
        usleep(1000);

    rb_peek(cargs->imu_rb, &first_imu);
    ekf_init(&state, P, first_imu.t);

    char imu_path[PATH_MAX];
    char gps_path[PATH_MAX];
    char fuzzy_path[PATH_MAX];

    create_directory_recursive(cargs->output_dir);
    snprintf(imu_path, sizeof(imu_path), "%s/imu_prop.csv", cargs->output_dir);
    snprintf(gps_path, sizeof(gps_path), "%s/gps_updates.csv", cargs->output_dir);
    snprintf(fuzzy_path, sizeof(fuzzy_path), "%s/fuzzy.csv", cargs->output_dir);

    open_state_log(imu_path);
    open_gps_log(gps_path);
    open_fuzzy_log(fuzzy_path);

    fuzzy_params_t fparams = 
    {
        .min_scale_R = 0.6,
        .max_scale_R = 3.0,
        .min_scale_Q = 0.7,
        .max_scale_Q = 2.0,
        .min_scale_gate = 0.85,
        .max_scale_gate = 1.5,
        .smoothing_alpha = 0.85
    };

    fuzzy_init(&fparams);
    last_time = state.t;
    processed = 0;

    while (1)
    {
        struct imu_sample imu;

        if (rb_pop(cargs->imu_rb, &imu) == 0)
        {
            dt = imu.t - last_time;
            if (dt <= 0.0)
                dt = 0.01;
            ekf_propagate(&state, P, &imu, dt);
            last_time = state.t;
            log_state(&state, P);
            processed++;
            continue; // go back to pop more IMU
        }

        struct gps_sample g;
        if (rb_pop(cargs->gps_rb, &g) == 0) 
        {
            struct imu_sample imu2; // Before GPS update, propagate IMU up to gps.t (if available)
            while (!rb_is_empty(cargs->imu_rb))
            {
                rb_peek(cargs->imu_rb, &imu2); // peek to see next imu timestamp
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

            double pos_enu[3]; // Convert GPS lat/lon/alt -> ENU using ref (assumed initialized)
            latlon_to_enu(g.lat, g.lon, g.alt, pos_enu);

            double vel_enu[3];             // Convert NED velocities (vn,ve,vu) to ENU
            vel_enu[0] = g.ve; // east -> x
            vel_enu[1] = g.vn; // north -> y
            vel_enu[2] = g.vu; // up -> z

            // Build Rpos and Rvel 3x3 diagonal covariances
            double Rpos[9] = {  GPS_POS_STD*GPS_POS_STD, 0, 0, 
                                0, GPS_POS_STD*GPS_POS_STD, 0, 
                                0,0, GPS_POS_STD*GPS_POS_STD  };

            double Rvel[9] = {  GPS_VEL_STD*GPS_VEL_STD, 0, 0,
                                0, GPS_VEL_STD*GPS_VEL_STD, 0,
                                0,0, GPS_VEL_STD*GPS_VEL_STD  };

            //---------- fuzzy supervisor -------------------//
            fuzzy_inputs_t fin;
            fin.mahalanobis_pos = maha_pos;   // from previous update
            fin.mahalanobis_vel = maha_vel;
            fin.cov_trace = P[0] + P[7] + P[14];  // trace of P (assuming row-major 3×3 block for pos)
            fin.accel_norm = 0.0; // placeholder, can fill from last IMU magnitude later

            fuzzy_outputs_t fout;
            fuzzy_update(&fin, &fout);

            double Rpos_scaled[9]; // Scale R matrices according to fuzzy output
            double Rvel_scaled[9]; 
            mat_scale(3, 3, fout.scale_R_gps, Rpos, Rpos_scaled);
            mat_scale(3, 3, fout.scale_R_gps, Rvel, Rvel_scaled);

            int accepted_pos = 0;
            int accepted_vel = 0;

            ekf_update_gps(&state, P, pos_enu, vel_enu, Rpos_scaled, Rvel_scaled, // Use scaled R in EKF
                                        &maha_pos, &accepted_pos, &maha_vel, &accepted_vel);  

            double innov_pos[3] = {
                pos_enu[0] - state.p[0],
                pos_enu[1] - state.p[1],
                pos_enu[2] - state.p[2]
            };
            double innov_vel[3] = {
                vel_enu[0] - state.v[0],
                vel_enu[1] - state.v[1],
                vel_enu[2] - state.v[2]
            };

            log_gps_update(g.t, pos_enu, innov_pos, maha_pos, accepted_pos, 
                                                    vel_enu, innov_vel, maha_vel, accepted_vel);

            log_fuzzy(g.t, fout.scale_R_gps, fout.scale_Q, fout.scale_gate); // log fuzzy scalers

            continue;
        }

        if (*(cargs->producers_done) >= 2 && rb_is_empty(cargs->imu_rb) && rb_is_empty(cargs->gps_rb))
            break;         // If nothing available, check termination condition
        usleep(1000);
    }

    close_state_log();
    close_gps_log();
    printf("Consumer finished, processed %zu IMU samples\n", processed);
    return NULL;
}

