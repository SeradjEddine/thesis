#include "consumer.h"
#include "ekf.h"
#include "logger.h"
#include "sensors.h"
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <time.h>

void *consumer_thread(void *arg)
{
    struct consumer_args *cargs = (struct consumer_args *)arg;
    struct ekf_state state;
    double P[P_DIM];

    /* Initialize EKF using first IMU timestamp if available in buffer, otherwise wait */
    struct imu_sample first_imu;
    /* Wait until buffer has at least one sample or producers done */
    while (rb_is_empty(cargs->imu_rb)) {
        if (*(cargs->producers_done) > 0) { usleep(1000); } else usleep(1000);
    }
    rb_peek(cargs->imu_rb, &first_imu);
    ekf_init(&state, P, first_imu.t);

    /* Open log */
    if (open_state_log("imu_prop.csv") != 0) {
        fprintf(stderr, "Failed to open log file\n");
    }

    double last_time = state.t;

    size_t processed = 0;
    while (1) {
        /* Check IMU buffer */
        struct imu_sample imu;
        int ok = rb_pop(cargs->imu_rb, &imu);
        if (ok == 0) {
            double dt = imu.t - last_time;
            if (dt <= 0.0) dt = 0.01; /* fallback */
            ekf_propagate(&state, P, &imu, dt);
            last_time = state.t;
            log_state(&state, P);
            processed++;
        } else {
            /* no imu sample available */
            /* check termination: if producers are done and buffers empty, break */
            if (*(cargs->producers_done) >= 2 && rb_is_empty(cargs->imu_rb)) {
                break;
            }
            /* sleep briefly to avoid busy loop */
            usleep(1000); /* 1 ms */
        }
    }

    close_state_log();
    printf("Consumer finished, processed %zu IMU samples\n", processed);
    return NULL;
}

