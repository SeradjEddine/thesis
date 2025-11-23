#ifndef CONSUMER_H
#define CONSUMER_H

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <pthread.h>

#include "fuzzy_supervisor.h"
#include "ekf.h"
#include "logger.h"
#include "sensors.h"
#include "multi_fuse.h"
#include "ringbuffer.h"
#include "../src/mathlib/mathlib.h"

#define PATH_MAX 256


struct consumer_args
{
    struct ringbuffer *imu_rb;
    struct ringbuffer *gps_rb;
    size_t imu_total;
    int *producers_done;
    const char *output_dir;
};

void fused_imu_to_imu_sample(const struct fused_imu *f, struct imu_sample *out);
void fused_pos_to_R(const double fusedP_pos[9], double R_out[9]);
void fused_vel_to_R(const double fusedP_vel[9], double R_out[9]);
void create_directory_recursive(const char *path);
void build_default_Rpos(double R_out[9]);
void build_default_Rvel(double R_out[9]);

void *consumer_thread(void *arg);




#endif

