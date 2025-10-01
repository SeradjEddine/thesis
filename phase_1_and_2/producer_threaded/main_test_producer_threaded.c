#include <pthread.h>
#include "ringbuffer.h"
#include "producer.h"
#include "sensors.h" // your function to fill arrays
#include <unistd.h>
#define IMU_CAPACITY 2000
#define GPS_CAPACITY 2000

int main() {
    // Load dataset into arrays
    struct imu_sample imus[10000];
    struct gps_sample gps[10000];
    int n_imu, n_gps;
    n_imu = read_oxts_csv("oxts.csv", imus, 10000, gps, 10000);
    n_gps = n_imu;

    // Initialize ring buffers
    struct imu_sample imu_storage[IMU_CAPACITY];
    struct gps_sample gps_storage[GPS_CAPACITY];
    struct ringbuffer imu_rb, gps_rb;

    rb_init(&imu_rb, imu_storage, IMU_CAPACITY, sizeof(struct imu_sample));
    rb_init(&gps_rb, gps_storage, GPS_CAPACITY, sizeof(struct gps_sample));

    // Prepare producer threads
    pthread_t imu_thread, gps_thread;
    struct producer_args imu_args = {
        .rb = &imu_rb,
        .data_array = imus,
        .elem_size = sizeof(struct imu_sample),
        .count = n_imu,
        .interval_ms = 10   // 100 Hz = 10 ms period
    };
    struct producer_args gps_args = {
        .rb = &gps_rb,
        .data_array = gps,
        .elem_size = sizeof(struct gps_sample),
        .count = n_gps,
        .interval_ms = 10   // also 100 Hz in your assumption
    };

    // Start producers
    pthread_create(&imu_thread, NULL, producer_thread, &imu_args);
    pthread_create(&gps_thread, NULL, producer_thread, &gps_args);

    // EKF consumer loop (simplified)
    while (1) {
        struct imu_sample imu;
        if (!rb_is_empty(&imu_rb)) {
            rb_pop(&imu_rb, &imu);
            // call ekf_propagate(&state, P, &imu, dt);
        }

        struct gps_sample g;
        if (!rb_is_empty(&gps_rb)) {
            rb_pop(&gps_rb, &g);
            // call ekf_update_gps(&state, P, &g);
        }

        usleep(1000); // 1 ms tick
    }

    return 0;
}

