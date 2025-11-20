#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <stddef.h>
#include <stdint.h>

struct ringbuffer
{
    uint8_t *buffer;     // pointer to storage
    size_t capacity;     // max number of elements
    size_t elem_size;    // size of each element in bytes
    size_t head;         // write index
    size_t tail;         // read index
    size_t count;        // number of elements currently stored
};

// Initialize ring buffer with external storage
void rb_init(struct ringbuffer *rb, void *storage,
             size_t capacity, size_t elem_size);

// Reset buffer (clear contents)
void rb_reset(struct ringbuffer *rb);

// Push element (copy into buffer). Returns 0 on success, -1 if full.
int rb_push(struct ringbuffer *rb, const void *elem);

// Pop element (copy out of buffer). Returns 0 on success, -1 if empty.
int rb_pop(struct ringbuffer *rb, void *elem);

// Peek at oldest element without removing it. Returns 0 on success, -1 if empty.
int rb_peek(const struct ringbuffer *rb, void *elem);

// Utility checks
int rb_is_empty(const struct ringbuffer *rb);
int rb_is_full(const struct ringbuffer *rb);
size_t rb_size(const struct ringbuffer *rb);

#endif

