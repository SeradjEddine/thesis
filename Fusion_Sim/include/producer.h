#ifndef PRODUCER_H
#define PRODUCER_H

#include <stddef.h>
#include "ringbuffer.h"

struct producer_args {
    struct ringbuffer *rb;   // target ring buffer
    void *data_array;        // pointer to dataset array
    size_t elem_size;        // sizeof(struct imu_sample) or gps_sample
    int count;            // number of elements in dataset
    unsigned interval_ms;    // time between pushes (sensor period)
};

void *producer_thread(void *arg);

#endif // PRODUCER_H

