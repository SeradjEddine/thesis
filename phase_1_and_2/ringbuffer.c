#include "ringbuffer.h"
#include <string.h>  // for memcpy

void rb_init(struct ringbuffer *rb, void *storage,
             size_t capacity, size_t elem_size)
{
    rb->buffer = (uint8_t *)storage;
    rb->capacity = capacity;
    rb->elem_size = elem_size;
    rb->head = rb->tail = rb->count = 0;
}

void rb_reset(struct ringbuffer *rb)
{
    rb->head = rb->tail = rb->count = 0;
}

int rb_push(struct ringbuffer *rb, const void *elem)
{
    if (rb->count == rb->capacity) {
        return -1; // full
    }

    memcpy(&rb->buffer[rb->head * rb->elem_size], elem, rb->elem_size);
    rb->head = (rb->head + 1) % rb->capacity;
    rb->count++;
    return 0;
}

int rb_pop(struct ringbuffer *rb, void *elem)
{
    if (rb->count == 0) {
        return -1; // empty
    }

    memcpy(elem, &rb->buffer[rb->tail * rb->elem_size], rb->elem_size);
    rb->tail = (rb->tail + 1) % rb->capacity;
    rb->count--;
    return 0;
}

int rb_peek(const struct ringbuffer *rb, void *elem)
{
    if (rb->count == 0) {
        return -1; // empty
    }

    memcpy(elem, &rb->buffer[rb->tail * rb->elem_size], rb->elem_size);
    return 0;
}

int rb_is_empty(const struct ringbuffer *rb)
{
    return rb->count == 0;
}

int rb_is_full(const struct ringbuffer *rb)
{
    return rb->count == rb->capacity;
}

size_t rb_size(const struct ringbuffer *rb)
{
    return rb->count;
}

