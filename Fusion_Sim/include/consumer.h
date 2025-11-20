#ifndef CONSUMER_H
#define CONSUMER_H
#include "ringbuffer.h"
#include <pthread.h>

struct consumer_args
{
    struct ringbuffer *imu_rb;
    struct ringbuffer *gps_rb;
    size_t imu_total;
    int *producers_done;
    const char *output_dir;
};

void *consumer_thread(void *arg);
#endif

