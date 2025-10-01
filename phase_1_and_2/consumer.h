#ifndef CONSUMER_H
#define CONSUMER_H
#include "ringbuffer.h"
#include <pthread.h>

struct consumer_args {
    struct ringbuffer *imu_rb;
    struct ringbuffer *gps_rb; // keep if you want to extend
    size_t imu_total;          // total imu samples to expect (for termination)
    int *producers_done;       // pointer to int counter; when equals 2 producers finished -> exit condition
};

void *consumer_thread(void *arg);
#endif

