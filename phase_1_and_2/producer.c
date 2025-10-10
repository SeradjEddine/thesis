#include "producer.h"
#include <unistd.h>   // usleep
#include <string.h>   // memcpy
#include <stdio.h>

void *producer_thread(void *arg)
{
    struct producer_args *pargs;
    uint8_t *base;
    void *elem;
    
    base = (uint8_t *)pargs->data_array;
    pargs = (struct producer_args *)arg;
    
    for (size_t i = 0; i < pargs->count; ++i)
    {
        elem = base + (i * pargs->elem_size);

        while (rb_push(pargs->rb, elem) != 0) {

            // if buffer full, wait a bit and retry
            usleep(1000); // 1 ms
        }
        // simulate real sensor rate
        usleep(pargs->interval_ms * 1000);
    }

    printf("Producer finished: pushed %zu elements\n", pargs->count);
    return NULL;
}

