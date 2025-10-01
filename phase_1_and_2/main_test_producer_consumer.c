#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include "ringbuffer.h"
#include "producer.h"
#include "consumer.h"
#include "sensors.h"

#define MAX_IMU 20000
#define MAX_GPS 20000
#define IMU_CAPACITY 2048
#define GPS_CAPACITY 2048

int main(int argc, char **argv)
{
    (void)argc; (void)argv;
    /* 1) load data using your reader (assumed available) */
    struct imu_sample *imus = malloc(sizeof(struct imu_sample) * MAX_IMU);
    struct gps_sample *gpss = malloc(sizeof(struct gps_sample) * MAX_GPS);
    if (!imus || !gpss) { fprintf(stderr,"OOM\n"); return -1; }

    int n_imus = read_oxts_csv("oxts.csv", imus, 10000, gpss, 10000);
    int n_gpss = n_imus;
    printf("Loaded %d IMU and %d GPS samples\n", n_imus, n_gpss);

    /* 2) init ring buffers with external storage arrays */
    struct imu_sample *imu_storage = malloc(sizeof(struct imu_sample) * IMU_CAPACITY);
    struct gps_sample *gps_storage = malloc(sizeof(struct gps_sample) * GPS_CAPACITY);
    struct ringbuffer imu_rb, gps_rb;
    rb_init(&imu_rb, imu_storage, IMU_CAPACITY, sizeof(struct imu_sample));
    rb_init(&gps_rb, gps_storage, GPS_CAPACITY, sizeof(struct gps_sample));

    /* 3) prepare producers */
    pthread_t imu_thread, gps_thread;
    struct producer_args imu_args = {
        .rb = &imu_rb,
        .data_array = imus,
        .elem_size = sizeof(struct imu_sample),
        .count = (size_t)n_imus,
        .interval_ms = 10
    };
    struct producer_args gps_args = {
        .rb = &gps_rb,
        .data_array = gpss,
        .elem_size = sizeof(struct gps_sample),
        .count = (size_t)n_gpss,
        .interval_ms = 10
    };

    /* producers_done counter */
    int producers_done = 0;

    /* create producers */
    pthread_create(&imu_thread, NULL, producer_thread, &imu_args);
    pthread_create(&gps_thread, NULL, producer_thread, &gps_args);

    /* 4) start consumer */
    pthread_t consumer;
    struct consumer_args carg = {
        .imu_rb = &imu_rb,
        .gps_rb = &gps_rb,
        .imu_total = (size_t)n_imus,
        .producers_done = &producers_done
    };
    pthread_create(&consumer, NULL, consumer_thread, &carg);

    /* 5) wait for producers to finish and increment producers_done accordingly */
    pthread_join(imu_thread, NULL);
    /* mark one producer finished */
    producers_done++;
    pthread_join(gps_thread, NULL);
    producers_done++;

    /* wait for consumer to finish */
    pthread_join(consumer, NULL);

    printf("All done. Exiting.\n");

    /* cleanup */
    free(imus); free(gpss); free(imu_storage); free(gps_storage);
    return 0;
}

