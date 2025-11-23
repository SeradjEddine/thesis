#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>


/* GPS-only observation */
struct gps_sample {
    double t;     // timestamp in seconds
    double lat;   // latitude (deg)
    double lon;   // longitude (deg)
    double alt;   // altitude (m)

    // Velocities in NED frame
    double vn;    // north velocity (m/s)
    double ve;    // east velocity (m/s)
    double vu;    // up velocity (m/s)
};

/* IMU-only observation */
struct imu_sample {
    double t;     // timestamp in seconds

    // Body-frame accelerations
    double ax;    // forward (X) accel [m/s^2]
    double ay;    // left (Y) accel [m/s^2]
    double az;    // up (Z) accel   [m/s^2]

    // Body-frame angular rates
    double wx;    // roll rate  [rad/s]
    double wy;    // pitch rate [rad/s]
    double wz;    // yaw rate   [rad/s]
};

int sensor_reader(const char *filename,
                  struct imu_sample *imus, int max_imus,
                  struct gps_sample *gps, int max_gps);

#endif

