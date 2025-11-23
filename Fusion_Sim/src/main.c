#include "../include/ringbuffer.h"
#include "../include/producer.h"
#include "../include/consumer.h"
#include "../include/sensors.h"
#include "../include/ekf.h"
#include "../include/multi_fuse.h"
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define MAX_IMU 5000
#define MAX_GPS 5000
#define IMU_CAPACITY 2048
#define GPS_CAPACITY 2048

int main(int argc, char **argv)
{
    if (argc < 4)
        return (fprintf(stderr, "Usage: %s <num_sensors> <input_file_prefix> <output_dir>\n", argv[0]), 1);
    int N = atoi(argv[1]);
    const char *prefix = argv[2];
    if (N <= 0)
        return (fprintf(stderr, "Invalid sensor count: %d\n", N), 1);

    struct  imu_sample **imu_arrays = calloc(N, sizeof(struct imu_sample *));
    struct  gps_sample **gps_arrays = calloc(N, sizeof(struct gps_sample *));
    int     count;

    if (!imu_arrays || !gps_arrays)
        return (perror("calloc"), 1);

    for (int i = 0; i < N; ++i)
    {
        char    fname[64];
        struct  imu_sample *imus = malloc(MAX_IMU * sizeof(struct imu_sample));
        struct  gps_sample *gpss = malloc(MAX_GPS * sizeof(struct gps_sample));

        if (!imus || !gpss)
            return ( perror("malloc"),1);

        snprintf(fname, sizeof(fname), "%s%d.csv", prefix, i);
        count = sensor_reader(fname, imus, MAX_IMU, gpss, MAX_GPS);
        if (count < 0)
            return(free(imus), free(gpss), write(1, "sensor_reader failed\n", 22));
        printf("Loaded %d samples from sensor: '%s' \n", count, fname);
        imu_arrays[i] = imus;
        gps_arrays[i] = gpss;

    }

    struct fused_imu *imus_fused = malloc(N * MAX_IMU * sizeof(struct fused_imu));
    struct fused_gps *gpss_fused = malloc(N * MAX_GPS * sizeof(struct fused_gps));
    int imu_count = 0;
    int gps_count = 0;

    if (!imus_fused || !gpss_fused)
        return ( perror("malloc"), 1);

    imu_count = fuse_imus(imu_arrays, N, count, imus_fused);
    gps_count = fuse_gps(gps_arrays,  N, count, gpss_fused);

    if (imu_count <= 0 || gps_count <= 0)
        return (fprintf(stderr, "Fusion produced no samples\n"),1);

    printf("Pre_fusion done: fused_imu=%d fused_gps=%d\n", imu_count, gps_count);
  
    
    if (gpss_fused && gps_count > 0)
    {
            ekf_init_gps_ref(gpss_fused);
            printf("GPS ref initialized : lat=%.6f lon=%.6f alt=%.2f\n",
                    gpss_fused[0].lat, gpss_fused[0].lon, gpss_fused[0].alt);
    }

    struct ringbuffer imu_rb;
    struct ringbuffer gps_rb;
    static uint8_t imu_storage[IMU_CAPACITY * sizeof(struct fused_imu)];
    static uint8_t gps_storage[GPS_CAPACITY * sizeof(struct fused_gps)];
    
    rb_init(&imu_rb, imu_storage, IMU_CAPACITY, sizeof(struct fused_imu));
    rb_init(&gps_rb, gps_storage, GPS_CAPACITY, sizeof(struct fused_gps));

    int producers_done = 0;

    struct producer_args imu_args = {
        .rb = &imu_rb,
        .data_array = imus_fused,
        .elem_size = sizeof(struct fused_imu),
        .count = imu_count,
        .interval_ms = 10

    };

    struct producer_args gps_args = {
        .rb = &gps_rb,
        .data_array = gpss_fused,
        .elem_size = sizeof(struct fused_gps),
        .count = gps_count,
        .interval_ms = 10
    };

    struct consumer_args cons_args = {
        .imu_rb = &imu_rb,
        .gps_rb = &gps_rb,
        .producers_done = &producers_done,
        .output_dir = argv[3]
    };

    pthread_t imu_thread;
    pthread_t gps_thread;
    pthread_t cons_thread;
    pthread_create(&imu_thread,  NULL, producer_thread, &imu_args);
    pthread_create(&gps_thread,  NULL, producer_thread, &gps_args);
    pthread_create(&cons_thread, NULL, consumer_thread, &cons_args);

    pthread_join(imu_thread, NULL);
    (*cons_args.producers_done)++;
    pthread_join(gps_thread, NULL);
    (*cons_args.producers_done)++;
    pthread_join(cons_thread, NULL);

    printf("Simulation completed successfully.\n");

    /* cleanup */
    for (int i = 0; i < N; ++i)
    {
        free(imu_arrays[i]);
        free(gps_arrays[i]);
    }
    free(imu_arrays);
    free(gps_arrays);
    free(imus_fused);
    free(gpss_fused);
    return 0;
}
