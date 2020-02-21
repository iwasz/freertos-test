#include <FreeRTOS.h>
#include <errno.h>
#include <stdio.h>

/**
 * Compatibility with libc heap allocator
 */
extern int errno;

caddr_t _sbrk (int incr)
{
        errno = ENOMEM;
        return (caddr_t)-1;
}

void *malloc (size_t size) { return pvPortMalloc (size); }

void *calloc (size_t num, size_t size)
{
        (void)num;
        (void)size;
        return NULL;
}

void *realloc (void *ptr, size_t size)
{
        (void)ptr;
        (void)size;
        return NULL;
}

void free (void *ptr) { vPortFree (ptr); }
