#include "../include/producer.h"
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <time.h>

void *producer_thread(void *arg)
{
    struct producer_args *pargs = (struct producer_args *)arg;  
    uint8_t *base = (uint8_t *)pargs->data_array;
    void    *elem;
    size_t     i = 0;

    while ( i < pargs->count)
    {
        elem = base + (i * pargs->elem_size);

        while (rb_push(pargs->rb, elem) != 0)
            usleep(100); // buffer full, wait and retry1 (ms)

        usleep(pargs->interval_ms * 100);  // simulates sensor update rate
        i ++;
    }
    printf("Producer finished: pushed %zu elements\n", pargs->count);
    return (NULL);
}


