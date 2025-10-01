#include <stdio.h>
#include "sensors.h"
#include "replay.h"

// === Consumer callbacks ===
void imu_consumer(const struct imu_sample *s) {
    static int count = 0;
    if (count < 5) { // print only first few
        printf("[%.9f] IMU: ax=%.6f ay=%.6f az=%.6f wx=%.6f wy=%.6f wz=%.6f\n",
               s->t, s->ax, s->ay, s->az, s->wx, s->wy, s->wz);
    }
    count++;
}

void gps_consumer(const struct gps_sample *s) {
    static int count = 0;
    if (count < 5) { // print only first few
        printf("[%.9f] GPS: lat=%.7f lon=%.7f alt=%.3f vn=%.6f ve=%.6f vu=%.6f\n",
               s->t, s->lat, s->lon, s->alt, s->vn, s->ve, s->vu);
    }
    count++;
}

int main(void) {
    // === Step 1: Load CSV into arrays ===
    struct imu_sample imu_array[50000];  // adjust to dataset size
    struct gps_sample gps_array[50000];

    int n = read_oxts_csv("oxts_clean.csv",
                          imu_array, 50000,
                          gps_array, 50000);
    if (n <= 0) {
        fprintf(stderr, "Failed to read oxts_clean.csv\n");
        return 1;
    }

    printf("Loaded %d samples from oxts_clean.csv\n", n);

    // === Step 2: Replay simulation in nanoseconds ===
    int64_t sim_time_ns = (int64_t)(20.0 * 1e9);  // simulate first 20 seconds
    int64_t dt_ns = (int64_t)(0.01 * 1e9);        // 10 ms per tick

    replay_sensors(imu_array, n,
                   gps_array, n,
                   sim_time_ns, dt_ns,
                   imu_consumer,
                   gps_consumer);

    return 0;
}

