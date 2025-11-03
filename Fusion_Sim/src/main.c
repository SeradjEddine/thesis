/* main.c - updated to read N separate oxts files: PREFIX0.csv .. PREFIX(N-1).csv
 *
 * Usage:
 *    ./fusion_sim <num_sensors> <file_prefix>
 *
 * Example:
 *    ./fusion_sim 3 ../Data/oxts_csv/oxts
 *    -> reads ../Data/oxts_csv/oxts0.csv, oxts1.csv, oxts2.csv
 */

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

#define MAX_IMU 5000
#define MAX_GPS 5000
#define IMU_CAPACITY 2048
#define GPS_CAPACITY 2048

/* --- Helpers: convert arrays -> linked lists --- */
static struct imu_node *array_to_imu_list(struct imu_sample *arr, int count)
{
    struct imu_node *head = NULL;
    struct imu_node *tail = NULL;

    for (int i = 0; i < count; ++i)
    {
        struct imu_node *n = malloc(sizeof(*n));

        if (!n) 
        {
            perror("malloc"); 
            return head; 
        }
        n->s = arr[i];
        n->next = NULL;
        if (!tail) 
            head = tail = n;
        else
        {
            tail->next = n; 
            tail = n; 
        }
    }
    return head;
}

static struct gps_node *array_to_gps_list(struct gps_sample *arr, int count)
{
    struct gps_node *head = NULL;
    struct gps_node *tail = NULL;

    for (int i = 0; i < count; ++i)
    {
        struct gps_node *n = malloc(sizeof(*n));
        if (!n)
        { 
            perror("malloc"); 
            return head;
        }
        n->s = arr[i];
        n->next = NULL;
        if (!tail)
            head = tail = n;
        else
        {
            tail->next = n;
            tail = n;
        }
    }
    return head;
}

/* NOTE: sensor_reader() signature (unchanged):
 * int sensor_reader(const char *filename,
 *                   struct imu_sample *imus, int max_imus,
 *                   struct gps_sample *gps, int max_gps)
 */

int main(int argc, char **argv)
{
    if (argc < 4)
    {
        fprintf(stderr, "Usage: %s <num_sensors> <input_file_prefix> <output_dir>\n", argv[0]);
        fprintf(stderr, "Example: %s 3 ../Data/oxts_csv/oxts\n", argv[0]);
        return 1;
    }

    int N = atoi(argv[1]);
    const char *prefix = argv[2];
    if (N <= 0)
    {
        fprintf(stderr, "Invalid sensor count: %d\n", N);
        return 1;
    }

    printf("Starting fusion_sim with %d sensors, prefix='%s'\n", N, prefix);

    /* Arrays to hold pointers to linked lists (one list per sensor) */
    struct imu_node **imu_heads = calloc(N, sizeof(struct imu_node*));
    struct gps_node **gps_heads = calloc(N, sizeof(struct gps_node*));

    if (!imu_heads || !gps_heads)
    {
        perror("calloc");
        return 1;
    }

    /* For each sensor index, build filename and call sensor_reader */
    for (int i = 0; i < N; ++i)
    {
        char fname[1024];
        /* Build filename: prefix + index + .csv
         * If your files use a different pattern, change this line.
         * e.g. sprintf(fname, "%s_%d.csv", prefix, i); */
        snprintf(fname, sizeof(fname), "%s%d.csv", prefix, i);

        struct imu_sample *imus = malloc(MAX_IMU * sizeof(struct imu_sample));
        struct gps_sample *gpss = malloc(MAX_GPS * sizeof(struct gps_sample));

        if (!imus || !gpss)
        {
            perror("malloc");
            return 1;
        }

        int n = sensor_reader(fname, imus, MAX_IMU, gpss, MAX_GPS);
        if (n < 0)
        {
            fprintf(stderr, "sensor_reader failed for file '%s'\n", fname);
            free(imus); free(gpss);
            // cleanup previously allocated lists
            for (int k = 0; k < i; ++k)
            {
                // might add a free linked lists function call here for imu_node/gps_node
                // For safety. must check this one later. if needed they will be called from here.
            }
            free(imu_heads); 
            free(gps_heads);
            return 1;
        }

        printf("Loaded sensor %d file '%s' (%d samples)\n", i, fname, n);

        imu_heads[i] = array_to_imu_list(imus, n);
        gps_heads[i] = array_to_gps_list(gpss, n);

        /* Use first sensor's first GPS sample to init ENU ref */
        if (i == 0 && gpss && n > 0)
        {
            ekf_init_gps_ref(gpss[0].lat, gpss[0].lon, gpss[0].alt);
            printf("GPS ref initialized from %s : lat=%.6f lon=%.6f alt=%.2f\n",
                   fname, gpss[0].lat, gpss[0].lon, gpss[0].alt);
        }

        free(imus);
        free(gpss);
    }

    /* --- Perform pre-EKF fusion ---
     * We fuse IMU and GPS lists separately (same fuse functions handle both types)
     * max_dt thresholds can be tuned depending on sensor sync quality.
     */
    double imu_max_dt = 0.002;  // 2 ms tolerance for IMU alignment
    double gps_max_dt = 0.01;   // 10 ms tolerance for GPS alignment
    struct fused_imu *fused_imu = fuse_imus(imu_heads, N, imu_max_dt, NULL, NULL);
    struct fused_gps *fused_gps = fuse_gps(gps_heads, N, gps_max_dt, NULL, NULL);

    /* Count fused samples */
    int imu_count = 0;
    int gps_count = 0;

    for (struct fused_imu *p = fused_imu; p; p = p->next) 
        ++imu_count;
    for (struct fused_gps *p = fused_gps; p; p = p->next)
        ++gps_count;

    if (imu_count == 0 || gps_count == 0)
    {
        fprintf(stderr, "Fusion produced no samples (imu_count=%d gps_count=%d)\n", imu_count, gps_count);
        // cleanup: free lists and arrays
        free(imu_heads); free(gps_heads);
        free_fused_imu_list(fused_imu);
        free_fused_gps_list(fused_gps);
        return 1;
    }

    /* Convert fused linked lists back into arrays so existing producers can reuse them */
    struct imu_sample *imus_fused = malloc(imu_count * sizeof(struct imu_sample));
    struct gps_sample *gpss_fused = malloc(gps_count * sizeof(struct gps_sample));

    if (!imus_fused || !gpss_fused)
    {
        perror("malloc");
        return 1;
    }

    int idx = 0;
    for (struct fused_imu *p = fused_imu; p; p = p->next, ++idx)
    {
        imus_fused[idx].t  = p->t;
        imus_fused[idx].ax = p->ax;
        imus_fused[idx].ay = p->ay;
        imus_fused[idx].az = p->az;
        imus_fused[idx].wx = p->gx;
        imus_fused[idx].wy = p->gy;
        imus_fused[idx].wz = p->gz;
    }
    idx = 0;
    for (struct fused_gps *p = fused_gps; p; p = p->next, ++idx)
    {
        gpss_fused[idx].t   = p->t;
        gpss_fused[idx].lat = p->lat;
        gpss_fused[idx].lon = p->lon;
        gpss_fused[idx].alt = p->alt;
        gpss_fused[idx].vn  = p->vn;
        gpss_fused[idx].ve  = p->ve;
        gpss_fused[idx].vu  = p->vu;
    }

    printf("Fusion complete: fused_imu_samples=%d fused_gps_samples=%d\n", imu_count, gps_count);

    /* --- Set up ring buffers and producers (unchanged workflow) --- */
    struct ringbuffer imu_rb, gps_rb;
    static uint8_t imu_storage[IMU_CAPACITY * sizeof(struct imu_sample)];
    static uint8_t gps_storage[GPS_CAPACITY * sizeof(struct gps_sample)];
    rb_init(&imu_rb, imu_storage, IMU_CAPACITY, sizeof(struct imu_sample));
    rb_init(&gps_rb, gps_storage, GPS_CAPACITY, sizeof(struct gps_sample));

    int producers_done = 0;
    struct producer_args imu_args = {
        .rb = &imu_rb,
        .data_array = imus_fused,
        .elem_size = sizeof(struct imu_sample),
        .count = imu_count,
        .interval_ms = 10 // adjust if fused rate differs
    };

    struct producer_args gps_args = {
        .rb = &gps_rb,
        .data_array = gpss_fused,
        .elem_size = sizeof(struct gps_sample),
        .count = gps_count,
        .interval_ms = 10
    };

    struct consumer_args cons_args = {
        .imu_rb = &imu_rb,
        .gps_rb = &gps_rb,
        .producers_done = &producers_done,
        .output_dir = argv[3]
    };

    pthread_t imu_thread, gps_thread, cons_thread;
    pthread_create(&imu_thread, NULL, producer_thread, &imu_args);
    pthread_create(&gps_thread, NULL, producer_thread, &gps_args);
    pthread_create(&cons_thread, NULL, consumer_thread, &cons_args);

    pthread_join(imu_thread, NULL);
    (*cons_args.producers_done)++;
    pthread_join(gps_thread, NULL);
    (*cons_args.producers_done)++;
    pthread_join(cons_thread, NULL);

    printf("Simulation completed successfully.\n");

    /* cleanup */
    free(imus_fused);
    free(gpss_fused);
    free_fused_imu_list(fused_imu);
    free_fused_gps_list(fused_gps);

    /* free original raw lists if they were allocated by array_to_* (they were) */
    for (int i = 0; i < N; ++i)
    {
        struct imu_node *in = imu_heads[i];
        while (in)
        {
            struct imu_node *nx = in->next;
            free(in);
            in = nx;
        }
        struct gps_node *gn = gps_heads[i];
        while (gn) 
        {
            struct gps_node *gx = gn->next;
            free(gn);
            gn = gx;
        }
    }

    free(imu_heads);
    free(gps_heads);

    return 0;
}
