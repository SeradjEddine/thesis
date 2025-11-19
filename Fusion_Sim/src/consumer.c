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
    size_t processed = 0;

    /* ==========================================
       🔸 1. Wait for first IMU and GPS samples
       ========================================== */
    struct imu_sample first_imu;
    struct gps_sample first_gps;

    while (rb_is_empty(cargs->imu_rb) || rb_is_empty(cargs->gps_rb))
        usleep(100);

    rb_peek(cargs->imu_rb, &first_imu);
    rb_peek(cargs->gps_rb, &first_gps);

    /* ==========================================
       🔸 2. Initialize ENU reference and EKF state
       ========================================== */
    // Initialize GPS reference for ENU conversion
    ekf_init_gps_ref(first_gps.lat, first_gps.lon, first_gps.alt);

    // Convert GPS position to ENU for initial position
    double pos0[3];
    latlon_to_enu(first_gps.lat, first_gps.lon, first_gps.alt, pos0);
    
    // Convert NED velocities (vn, ve, vu) to ENU
    double vel0[3];
    vel0[0] = first_gps.ve; // East → x
    vel0[1] = first_gps.vn; // North → y
    vel0[2] = first_gps.vu; // Up → z

    // Initialize EKF with proper starting state
    ekf_init(&state, P, first_imu.t, pos0, vel0);
    last_time = state.t;

    printf("EKF initialized at t=%.3f ENU pos=[%.2f %.2f %.2f] vel=[%.2f %.2f %.2f]\n",
           first_imu.t, pos0[0], pos0[1], pos0[2], vel0[0], vel0[1], vel0[2]);

    /* ==========================================
       🔸 3. Prepare log file outputs
       ========================================== */
    char imu_path[PATH_MAX], gps_path[PATH_MAX], fuzzy_path[PATH_MAX];
    create_directory_recursive(cargs->output_dir);

    

    snprintf(imu_path, sizeof(imu_path), "%s/imu_prop.csv", cargs->output_dir);
    snprintf(gps_path, sizeof(gps_path), "%s/gps_updates.csv", cargs->output_dir);
    snprintf(fuzzy_path, sizeof(fuzzy_path), "%s/fuzzy.csv", cargs->output_dir);

    open_state_log(imu_path);
    open_gps_log(gps_path);
    open_fuzzy_log(fuzzy_path);

    /* ==========================================
       🔸 4. Initialize fuzzy supervisor
       ========================================== */
    fuzzy_params_t fparams = {
        .min_scale_R = 0.6,
        .max_scale_R = 3.0,
        .min_scale_Q = 0.7,
        .max_scale_Q = 2.0,
        .min_scale_gate = 0.85,
        .max_scale_gate = 1.5,
        .smoothing_alpha = 0.85
    };
    fuzzy_init(&fparams);

    /* ==========================================
       🔸 5. Main consumer loop
       ========================================== */
    while (1)
    {
        struct imu_sample imu;

        /* --- IMU Propagation Step --- */
        if (rb_pop(cargs->imu_rb, &imu) == 0)
        {
            dt = imu.t - last_time;
            if (dt <= 0.0) dt = 0.01;
            ekf_propagate(&state, P, &imu, dt);
            last_time = state.t;
            log_state(&state, P);
            processed++;
            continue;
        }

        /* --- GPS Update Step --- */
        struct gps_sample g;
        if (rb_pop(cargs->gps_rb, &g) == 0)
        {
            /* Propagate IMU samples up to GPS timestamp */
            struct imu_sample imu2;
            while (!rb_is_empty(cargs->imu_rb))
            {
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
                else break;
            }

            /* --- Convert GPS to ENU --- */
            double pos_enu[3];
            latlon_to_enu(g.lat, g.lon, g.alt, pos_enu);
            double vel_enu[3] = { g.ve, g.vn, g.vu };

            /* --- Build measurement noise matrices --- */
            double Rpos[9] = {
                GPS_POS_STD*GPS_POS_STD, 0, 0,
                0, GPS_POS_STD*GPS_POS_STD, 0,
                0, 0, GPS_POS_STD*GPS_POS_STD
            };

            double Rvel[9] = {
                GPS_VEL_STD*GPS_VEL_STD, 0, 0,
                0, GPS_VEL_STD*GPS_VEL_STD, 0,
                0, 0, GPS_VEL_STD*GPS_VEL_STD
            };

            /* --- Fuzzy Supervisor Update --- */
            fuzzy_inputs_t fin;
            fin.mahalanobis_pos = maha_pos;
            fin.mahalanobis_vel = maha_vel;
            fin.cov_trace = P[IDX(0,0)] + P[IDX(1,1)] + P[IDX(2,2)];
            fin.accel_norm = 0.0;

            fuzzy_outputs_t fout;
            fuzzy_update(&fin, &fout);

            double Rpos_scaled[9], Rvel_scaled[9];
            mat_scale(3, 3, fout.scale_R_gps, Rpos, Rpos_scaled);
            mat_scale(3, 3, fout.scale_R_gps, Rvel, Rvel_scaled);

            /* --- EKF GPS Update --- */
            int accepted_pos = 0, accepted_vel = 0;
            ekf_update_gps(&state, P, pos_enu, vel_enu,
                           Rpos_scaled, Rvel_scaled,
                           &maha_pos, &accepted_pos,
                           &maha_vel, &accepted_vel);

            /* --- Logging --- */
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
            log_fuzzy(g.t, fout.scale_R_gps, fout.scale_Q, fout.scale_gate);
            continue;
        }

        /* --- Exit condition --- */
        if (*(cargs->producers_done) >= 2 &&
            rb_is_empty(cargs->imu_rb) &&
            rb_is_empty(cargs->gps_rb))
            break;

        usleep(100);
    }

    /* ==========================================
       🔸 6. Cleanup
       ========================================== */
    close_state_log();
    close_gps_log();
    close_fuzzy_log();

    printf("Consumer finished, processed %zu IMU samples\n", processed);
    return NULL;
}
