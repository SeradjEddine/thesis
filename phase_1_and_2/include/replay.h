#ifndef REPLAY_H
#define REPLAY_H

#include "sensors.h"

#include <stdint.h>  // make sure this is included

void replay_sensors(const struct imu_sample *imus, int n_imus,
                    const struct gps_sample *gps, int n_gps,
                    int64_t t_end_ns, int64_t dt_ns,
                    void (*imu_cb)(const struct imu_sample *),
                    void (*gps_cb)(const struct gps_sample *));

#endif // REPLAY_H

