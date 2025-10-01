#include <stdio.h>
#include <stdlib.h>
#include "sensors.h"

// Forward declaration of the CSV reader
int read_oxts_csv(const char *filename,
                  struct imu_sample *imus, int max_imus,
                  struct gps_sample *gps, int max_gps);

#define MAX_SAMPLES 10000

int main(void) {
    struct imu_sample imus[MAX_SAMPLES];
    struct gps_sample gps[MAX_SAMPLES];

    const char *filename = "oxts_clean.csv";

    int n = read_oxts_csv(filename, imus, MAX_SAMPLES, gps, MAX_SAMPLES);
    if (n <= 0) {
        fprintf(stderr, "Failed to read samples from %s\n", filename);
        return 1;
    }

    printf("Read %d samples from %s\n\n", n, filename);

    // --- Time delta check ---
    printf("Time check (first 5 deltas):\n");
    for (int i = 1; i < 6 && i < n; i++) {
        double dt = gps[i].t - gps[i-1].t;
        printf("[%d -> %d] Δt = %.9f s\n", i-1, i, dt);
    }
    printf("\n");

    // --- First 5 GPS samples ---
    printf("GPS samples (first 5):\n");
    for (int i = 0; i < 5 && i < n; i++) {
        printf("[%d] t=%.9f s lat=%.7f lon=%.7f alt=%.3f vn=%.3f ve=%.3f vu=%.3f\n",
               i,
               gps[i].t,
               gps[i].lat, gps[i].lon, gps[i].alt,
               gps[i].vn, gps[i].ve, gps[i].vu);
    }
    printf("\n");

    // --- First 5 IMU samples ---
    printf("IMU samples (first 5):\n");
    for (int i = 0; i < 5 && i < n; i++) {
        printf("[%d] t=%.9f s ax=%.3f ay=%.3f az=%.3f wx=%.3f wy=%.3f wz=%.3f\n",
               i,
               imus[i].t,
               imus[i].ax, imus[i].ay, imus[i].az,
               imus[i].wx, imus[i].wy, imus[i].wz);
    }

    // --- Optional: print summary statistics ---
    if (n > 1) {
        double total_time = gps[n-1].t - gps[0].t;
        printf("\nTotal relative time: %.9f s\n", total_time);
        printf("Average dt: %.9f s\n", total_time / (n-1));
    }

    return 0;
}

