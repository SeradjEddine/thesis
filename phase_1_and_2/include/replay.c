#include "replay.h"
#include "ringbuffer.h"
#include <stdio.h>
#include <stdint.h>  // for int64_t

/**
 * Replay IMU and GPS samples according to their relative timestamps.
 * Uses ring buffers to feed consumer callbacks at the correct "time".
 *
 * @param imus       array of IMU samples
 * @param n_imus     number of IMU samples
 * @param gps        array of GPS samples
 * @param n_gps      number of GPS samples
 * @param t_end_ns   simulation end time (nanoseconds)
 * @param dt_ns      simulation step size (nanoseconds)
 * @param imu_cb     callback for IMU samples
 * @param gps_cb     callback for GPS samples
 */
void replay_sensors(const struct imu_sample *imus, int n_imus,
                    const struct gps_sample *gps, int n_gps,
                    int64_t t_end_ns, int64_t dt_ns,
                    void (*imu_cb)(const struct imu_sample*),
                    void (*gps_cb)(const struct gps_sample*))
{
    // Static storage (no malloc) for ring buffers
    struct imu_sample imu_storage[200];
    struct gps_sample gps_storage[50];

    struct ringbuffer rb_imu, rb_gps;
    rb_init(&rb_imu, imu_storage, 200, sizeof(struct imu_sample));
    rb_init(&rb_gps, gps_storage, 50, sizeof(struct gps_sample));

    int imu_idx = 0;
    int gps_idx = 0;

    // Convert all relative times to nanoseconds (int64_t)
    int64_t imu_times[n_imus];
    int64_t gps_times[n_gps];

    for (int i = 0; i < n_imus; i++)
        imu_times[i] = (int64_t)(imus[i].t * 1e9);

    for (int i = 0; i < n_gps; i++)
        gps_times[i] = (int64_t)(gps[i].t * 1e9);

    // Simulation loop in nanoseconds
    for (int64_t t_ns = 0; t_ns <= t_end_ns; t_ns += dt_ns) {

        // Feed IMU samples whose timestamp <= current simulation time
        while (imu_idx < n_imus && imu_times[imu_idx] <= t_ns) {
            rb_push(&rb_imu, &imus[imu_idx]);
            imu_idx++;
        }

        // Feed GPS samples whose timestamp <= current simulation time
        while (gps_idx < n_gps && gps_times[gps_idx] <= t_ns) {
            rb_push(&rb_gps, &gps[gps_idx]);
            gps_idx++;
        }

        // Deliver IMU samples to consumer
        struct imu_sample s_imu;
        while (!rb_is_empty(&rb_imu)) {
            rb_pop(&rb_imu, &s_imu);
            if (imu_cb) imu_cb(&s_imu);
        }

        // Deliver GPS samples to consumer
        struct gps_sample s_gps;
        while (!rb_is_empty(&rb_gps)) {
            rb_pop(&rb_gps, &s_gps);
            if (gps_cb) gps_cb(&s_gps);
        }
    }
}

