#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include "../include/ringbuffer.h"
#include "../include/producer.h"
#include "../include/consumer.h"
#include "../include/sensors.h"
#include "../include/ekf.h"

#define MAX_IMU 10000
#define MAX_GPS 2000
#define IMU_CAPACITY 2048
#define GPS_CAPACITY 512

int main(void)
{
    struct imu_sample *imus = malloc(MAX_IMU * sizeof(struct imu_sample));
    struct gps_sample *gpss = malloc(MAX_GPS * sizeof(struct gps_sample));

    if (!imus || !gpss) {
        fprintf(stderr, "Memory allocation failed\n");
        return 1;
    }

    /* ---- 1. Load combined dataset ---- */
    int imu_count = 0, gps_count = 0;
    int n = read_oxts_csv("../Data/oxts_csv/oxts.csv", imus, MAX_IMU, gpss, MAX_GPS);
    if (n < 0) {
        fprintf(stderr, "Error: failed to load OXTS data file.\n");
        return 1;
    }

    imu_count = n; // IMU and GPS are synced count-wise in this dataset
    gps_count = n;
    printf("Loaded %d combined OXTS samples\n", n);

    /* ---- 2. Initialize GPS ENU reference ---- */
    ekf_init_gps_ref(gpss[0].lat, gpss[0].lon, gpss[0].alt);
    printf("GPS ref: lat=%.6f lon=%.6f alt=%.2f\n",
           gpss[0].lat, gpss[0].lon, gpss[0].alt);

    /* ---- 3. Allocate ring buffers ---- */
    static uint8_t imu_storage[IMU_CAPACITY * sizeof(struct imu_sample)];
    static uint8_t gps_storage[GPS_CAPACITY * sizeof(struct gps_sample)];

    struct ringbuffer imu_rb, gps_rb;
    rb_init(&imu_rb, imu_storage, IMU_CAPACITY, sizeof(struct imu_sample));
    rb_init(&gps_rb, gps_storage, GPS_CAPACITY, sizeof(struct gps_sample));

    int producers_done = 0;

    /* ---- 4. Producer args ---- */
    struct producer_args imu_args = {
        .rb = &imu_rb,
        .data_array = imus,
        .elem_size = sizeof(struct imu_sample),
        .count = imu_count,
        .interval_ms = 10, // 100 Hz IMU
    };

    struct producer_args gps_args = {
        .rb = &gps_rb,
        .data_array = gpss,
        .elem_size = sizeof(struct gps_sample),
        .count = gps_count,
        .interval_ms = 10, // 100 Hz GPS
    };

    /* ---- 5. Consumer args ---- */
    struct consumer_args cons_args = {
        .imu_rb = &imu_rb,
        .gps_rb = &gps_rb,
        .producers_done = &producers_done
    };

    /* ---- 6. Launch threads ---- */
    pthread_t imu_thread, gps_thread, cons_thread;
    pthread_create(&imu_thread, NULL, producer_thread, &imu_args);
    pthread_create(&gps_thread, NULL, producer_thread, &gps_args);
    pthread_create(&cons_thread, NULL, consumer_thread, &cons_args);

    printf ("waiting here\n");
    /* ---- 7. Wait ---- */
    pthread_join(imu_thread, NULL);
    *(cons_args.producers_done) += 1;
    pthread_join(gps_thread, NULL);
    *(cons_args.producers_done) += 1;
    pthread_join(cons_thread, NULL);

    printf("Simulation completed successfully.\n");

    free(imus);
    free(gpss);
    return 0;
}
