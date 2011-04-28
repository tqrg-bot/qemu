/*
 * malloc-like functions for system emulation.
 *
 * Copyright (c) 2006 Fabrice Bellard
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "qemu-common.h"
#include "trace.h"
#include <stdlib.h>

#ifdef CONFIG_MALLOC_STATS
struct QEMUMallocStatsData {
    void *base;
    QEMUMallocStatsLocus *locus;
    size_t size;
};

typedef struct QEMUMallocStatsData QEMUMallocStatsData;

#define STATS_DATA_SIZE sizeof(QEMUMallocStatsData)
#else
#define STATS_DATA_SIZE 0
#endif

static void *qemu_oom_check(void *ptr)
{
    if (ptr == NULL) {
        fprintf(stderr, "Failed to allocate memory: %s\n", strerror(errno));
        abort();
    }
    return ptr;
}

static int allow_zero_malloc(void)
{
#if defined(CONFIG_ZERO_MALLOC)
    return 1;
#else
    return 0;
#endif
}

#ifdef CONFIG_MALLOC_STATS
static inline QEMUMallocStatsData *fetch_mem_data(void *ptr)
{
    return ((QEMUMallocStatsData *)ptr) - 1;
}
#endif

static inline void *store_mem_data(char *ptr, int extra,
				   QEMUMallocStatsLocus *locus, size_t size)
{
#ifdef CONFIG_MALLOC_STATS
    QEMUMallocStatsData *data;
    data = (QEMUMallocStatsData *) (ptr + extra - STATS_DATA_SIZE);
    data->base = ptr;
    data->locus = locus;
    data->size = size;
    return &data[1];
#else
    return ptr;
#endif
}

#ifdef CONFIG_MALLOC_STATS
static QEMUMallocStatsLocus *malloc_loci_head;
#endif

static inline void update_mem_stats(QEMUMallocStatsLocus *locus,
                                    ptrdiff_t size, int dir)
{
#ifdef CONFIG_MALLOC_STATS
    size_t alloc, alloc_max, alloc_tot;

    __sync_add_and_fetch(&locus->live, dir);
    alloc = __sync_add_and_fetch(&locus->alloc, size);
    if (dir < 0) {
        return;
    }

    __sync_add_and_fetch(&locus->count, dir);
    do {
        alloc_max = locus->alloc_max;
    } while (alloc > alloc_max &&
             !__sync_bool_compare_and_swap(&locus->alloc_max, alloc_max, alloc));

    if (size < 0) {
        return;
    }

    /* On the first allocation, put the struct into the list.  */
    alloc_tot = __sync_fetch_and_add(&locus->alloc_tot, size);
    if (alloc_tot == 0) {
        do {
            locus->next = malloc_loci_head;
        } while (!__sync_bool_compare_and_swap(&malloc_loci_head,
                                               locus->next, locus));
    }
#endif
}

void qemu_free(void *ptr)
{
    trace_qemu_free(ptr);
#ifdef CONFIG_MALLOC_STATS
    if (ptr) {
        QEMUMallocStatsData *data = fetch_mem_data(ptr);
        update_mem_stats(data->locus, -data->size, -1);
        ptr = data->base;
    }
#endif
    free(ptr);
}

#undef qemu_malloc
void *qemu_malloc(size_t size, QEMUMallocStatsLocus *locus)
{
    char *ptr;
    size_t real_size = size;
    assert(size <= LONG_MAX);
    if (!size && !allow_zero_malloc()) {
        abort();
    }
    size += STATS_DATA_SIZE;
    ptr = qemu_oom_check(malloc(size ? size : 1));
    ptr = store_mem_data(ptr, STATS_DATA_SIZE, locus, real_size);
    trace_qemu_malloc(real_size, ptr);
    update_mem_stats(locus, real_size, 1);
    return ptr;
}

#undef qemu_realloc
void *qemu_realloc(void *ptr, size_t size, QEMUMallocStatsLocus *locus)
{
    void *oldptr, *newptr;
    size_t old_size;
    size_t real_size = size;
    assert(size <= LONG_MAX);
    oldptr = ptr;
    old_size = 0;

#ifdef CONFIG_MALLOC_STATS
    if (ptr) {
        QEMUMallocStatsData *data = fetch_mem_data(ptr);
        locus = data->locus;
        old_size = data->size;
        oldptr = data->base;
    }
#endif
    if (!size && !allow_zero_malloc()) {
        abort();
    }
    size += STATS_DATA_SIZE;
    newptr = qemu_oom_check(realloc(oldptr, size ? size : 1));
    newptr = store_mem_data(newptr, STATS_DATA_SIZE, locus, real_size);
    trace_qemu_realloc(ptr, real_size, newptr);
    update_mem_stats(locus, real_size - old_size, ptr ? 0 : 1);
    return newptr;
}

#undef qemu_mallocz
void *qemu_mallocz(size_t size, QEMUMallocStatsLocus *locus)
{
    void *ptr;
    size_t real_size = size;
    assert(size <= LONG_MAX);
    if (!size && !allow_zero_malloc()) {
        abort();
    }
    size += STATS_DATA_SIZE;
    ptr = qemu_oom_check(calloc(1, size ? size : 1));
    ptr = store_mem_data(ptr, STATS_DATA_SIZE, locus, real_size);
    trace_qemu_malloc(real_size, ptr);
    update_mem_stats(locus, real_size, 1);
    return ptr;
}

#undef qemu_strdup
char *qemu_strdup(const char *str, QEMUMallocStatsLocus *locus)
{
    char *ptr;
    size_t len = strlen(str);
    ptr = (qemu_malloc)(len + 1, locus);
    memcpy(ptr, str, len + 1);
    return ptr;
}

#undef qemu_strndup
char *qemu_strndup(const char *str, size_t size, QEMUMallocStatsLocus *locus)
{
    const char *end = memchr(str, 0, size);
    char *new;

    if (end) {
        size = end - str;
    }

    new = (qemu_malloc)(size + 1, locus);
    new[size] = 0;

    return memcpy(new, str, size);
}

#undef qemu_memalign
void *qemu_memalign(size_t alignment, size_t size, QEMUMallocStatsLocus *locus)
{
    char *ptr;
    size_t real_size = size;
    size_t extra;
    assert(size <= LONG_MAX);
    if (!size && !allow_zero_malloc()) {
        abort();
    }
    extra = (STATS_DATA_SIZE + alignment - 1) & -alignment;
    size += extra;
    ptr = qemu_oom_check(os_memalign(alignment, size ? size : 1));
    ptr = store_mem_data(ptr, extra, locus, real_size);
    trace_qemu_memalign(alignment, real_size, ptr);
    update_mem_stats(locus, real_size, 1);
    return ptr;
}

/* alloc shared memory pages */
#undef qemu_vmalloc
void *qemu_vmalloc(size_t size, QEMUMallocStatsLocus *locus)
{
    return qemu_memalign(getpagesize(), size, locus);
}

void qemu_vfree(void *ptr)
{
    trace_qemu_vfree(ptr);
#ifdef CONFIG_MALLOC_STATS
    if (ptr) {
        QEMUMallocStatsData *data = fetch_mem_data(ptr);
        update_mem_stats(data->locus, -data->size, -1);
        ptr = data->base;
    }
#endif
    os_vfree(ptr);
}

#ifdef CONFIG_MALLOC_STATS
int dump_malloc_stats(const char *file_name)
{
    FILE *file = fopen(file_name, "w");
    QEMUMallocStatsLocus *p = malloc_loci_head;

    const char *this_file = __FILE__;
    const char *base = strrchr(this_file, '/');
    int skip = base ? base + 1 - this_file : 0;
    int rc = 0;

    fprintf(file,
            "FILE:LINE (FUNCTION)                      Live  "
            "Count   Curr bytes    Max bytes        Tot bytes\n"
            "------------------------------------------------"
            "------------------------------------------------\n");

    while (p && !ferror(file)) {
        const char *locus_file, *prefix = "";
        char s[40];
        if (skip && !memcmp(p->file, this_file, skip)) {
            locus_file = p->file + skip;
        } else {
            locus_file = strrchr(p->file, '/');
            if (locus_file) {
                prefix = "...";
            } else {
                locus_file = p->file;
            }
        }
        snprintf(s, sizeof(s), "%s%s:%d (%s)",
                 prefix, locus_file, p->line, p->func);
        fprintf(file,
                "%-40s %5"PRIu64 " %6"PRIu64 " %12"PRIu64" %12"PRIu64
                " %16"PRIu64"\n",
                s, p->live, p->count, p->alloc, p->alloc_max, p->alloc_tot);
        p = p->next;
    }

    if (ferror(file)) {
        rc = -errno;
    }
    if (fflush(file) == EOF && rc == 0) {
        rc = -errno;
    }
    if (fclose(file) == EOF && rc == 0) {
        rc = -errno;
    }
    return rc;
}
#endif
