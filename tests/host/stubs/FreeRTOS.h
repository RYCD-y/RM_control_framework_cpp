// host 替身：把固件的 pvPortMalloc 映射到 malloc
#ifndef FREERTOS_HOST_STUB_H
#define FREERTOS_HOST_STUB_H
#include <stdlib.h>
static inline void *pvPortMalloc(size_t n) { return malloc(n); }
static inline void  vPortFree(void *p) { free(p); }
#endif
