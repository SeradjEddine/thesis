#include "consumer.h"
#include "ekf.h"
#include "logger.h"
#include "sensors.h"
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <time.h>

void consumer_init ()
{


}

void *consumer_thread(void *arg)
{
    struct consumer_args *cargs;
    struct ekf_state state;
    double P[P_DIM];
    struct imu_sample first_imu;
    double last_time;
    size_t processed;

    cargs = (struct consumer_args *)arg;
    /* Initialize EKF using first IMU timestamp if available in buffer,
    otherwise Wait til buffer has at least one sample or producers done */

    while (rb_is_empty(cargs->imu_rb))
    {
        if (*(cargs->producers_done) > 0)
        {
            usleep(1000);
        }
        else usleep(1000);
    }

    rb_peek(cargs->imu_rb, &first_imu);
    ekf_init(&state, P, first_imu.t);

    /* Open log file*/
    if (open_state_log("imu_prop.csv") != 0) {
        fprintf(stderr, "Failed to open log file\n");
    }

    last_time = state.t;
    processed = 0;
    
    double dt;
    struct imu_sample imu;

    while (1)
    {
        //int ok = rb_pop(cargs->imu_rb, &imu);
        if (rb_pop(cargs->imu_rb, &imu))
        {
            dt = imu.t - last_time;
            if (dt <= 0.0)
                dt = 0.01; /* fallback */
            ekf_propagate(&state, P, &imu, dt);
            last_time = state.t;
            log_state(&state, P);
            processed ++;
        } 
        else  /* no sample available */
        {
            /* term check: if producers done and buffers empty, break */
            if (*(cargs->producers_done) >= 2 && rb_is_empty(cargs->imu_rb))
                break;
            usleep(1000); /* breaf sleep to avoid busy loop 1 ms */
        }
    }

    close_state_log();
    printf("Consumer finished, processed %zu IMU samples\n", processed);
    return NULL;
}

