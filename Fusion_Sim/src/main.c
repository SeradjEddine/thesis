#include "../include/ringbuffer.h"
#include "../include/producer.h"
#include "../include/consumer.h"
#include "../include/sensors.h"
#include "../include/ekf.h"
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

#define MAX_IMU 5000
#define MAX_GPS 5000
#define IMU_CAPACITY 2048
#define GPS_CAPACITY 2048

int main(void)
{
    struct imu_sample *imus = malloc(MAX_IMU * sizeof(struct imu_sample));
    struct gps_sample *gpss = malloc(MAX_GPS * sizeof(struct gps_sample));
    int imu_count = 0;
    int gps_count = 0;
    int n;

    if (!imus || !gpss)
        return (fprintf(stderr, "Memory allocation failed\n"), 1);

    n = read_oxts_csv("../Data/oxts_csv/oxts.csv", imus, MAX_IMU, gpss, MAX_GPS); // Loads combined dataset
    if (n < 0)
        return (fprintf(stderr, "Error: failed to load OXTS data file.\n"), 1);

    imu_count = n;
    gps_count = n;
    printf("Loaded %d combined OXTS samples\n", n);

    ekf_init_gps_ref(gpss[0].lat, gpss[0].lon, gpss[0].alt); //Initialize GPS ENU reference
    printf("GPS ref: lat=%.6f lon=%.6f alt=%.2f\n", gpss[0].lat, gpss[0].lon, gpss[0].alt);

   //  Allocate ring buffers 
    struct ringbuffer imu_rb;
    struct ringbuffer gps_rb;

    static uint8_t imu_storage[IMU_CAPACITY * sizeof(struct imu_sample)];
    static uint8_t gps_storage[GPS_CAPACITY * sizeof(struct gps_sample)];
    
    rb_init(&imu_rb, imu_storage, IMU_CAPACITY, sizeof(struct imu_sample));
    rb_init(&gps_rb, gps_storage, GPS_CAPACITY, sizeof(struct gps_sample));

    // Producer args
    int producers_done = 0;
    struct producer_args imu_args =
    {
        .rb = &imu_rb,
        .data_array = imus,
        .elem_size = sizeof(struct imu_sample),
        .count = imu_count,
        .interval_ms = 10, // 100 Hz IMU
    };

    struct producer_args gps_args = 
    {
        .rb = &gps_rb,
        .data_array = gpss,
        .elem_size = sizeof(struct gps_sample),
        .count = gps_count,
        .interval_ms = 10, // 100 Hz GPS
    };

    // Consumer args
    struct consumer_args cons_args = 
    {
        .imu_rb = &imu_rb,
        .gps_rb = &gps_rb,
        .producers_done = &producers_done
    };

    // Launch threads
    pthread_t imu_thread;
    pthread_t gps_thread;
    pthread_t cons_thread;

    pthread_create(&imu_thread, NULL, producer_thread, &imu_args);
    pthread_create(&gps_thread, NULL, producer_thread, &gps_args);
    pthread_create(&cons_thread, NULL, consumer_thread, &cons_args);

    // Wait for the threads to finish
    pthread_join(imu_thread, NULL);
    *(cons_args.producers_done) += 1;
    pthread_join(gps_thread, NULL);
    *(cons_args.producers_done) += 1;
    pthread_join(cons_thread, NULL);

    printf("Simulation completed successfully.\n");

    free(imus);
    free(gpss);
    return (0);
}
