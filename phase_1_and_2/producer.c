#include "producer.h"
#include <unistd.h>   // usleep
#include <string.h>   // memcpy
#include <stdio.h>

void *producer_thread(void *arg) {
    struct producer_args *pargs = (struct producer_args *)arg;

    uint8_t *base = (uint8_t *)pargs->data_array;

    for (size_t i = 0; i < pargs->count; ++i) {
        // pointer to ith element
        void *elem = base + i * pargs->elem_size;

        // push into buffer (wait if full)
        while (rb_push(pargs->rb, elem) != 0) {
            // buffer full, wait a bit and retry
            usleep(1000); // 1 ms
        }

        // simulate real sensor rate
        usleep(pargs->interval_ms * 1000);
    }

    printf("Producer finished: pushed %zu elements\n", pargs->count);
    return NULL;
}

