#include "ringbuffer.h"
#include "sensors.h"
#include <stdio.h>

#define BUFFER_CAPACITY 16

int main(void)
{
    struct imu_sample storage[BUFFER_CAPACITY];
    struct ringbuffer rb;

    rb_init(&rb, storage, BUFFER_CAPACITY, sizeof(struct imu_sample));

    // Push
    struct imu_sample imu1 = {0.01, 0.0, 0.0, 9.8, 0.01, 0.02, 0.03};
    rb_push(&rb, &imu1);

    // Pop
    struct imu_sample out;
    if (rb_pop(&rb, &out) == 0) {
        printf("t=%.3f ax=%.2f ay=%.2f az=%.2f\n",
               out.t, out.ax, out.ay, out.az);
    }

    return 0;
}

