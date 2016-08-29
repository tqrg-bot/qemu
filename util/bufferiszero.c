/*
 * Simple C functions to supplement the C library
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
#include "qemu/osdep.h"
#include "qemu-common.h"
#include "qemu/cutils.h"
#include "qemu/bswap.h"


/* vector definitions */

typedef bool (*accel_zero_fn)(const void *, size_t);

static bool
buffer_zero_int(const void *buf, size_t len)
{
    if (unlikely(len < 8)) {
        /* For a very small buffer, simply accumulate all the bytes.  */
        const unsigned char *p = buf;
        const unsigned char *e = buf + len;
        unsigned char t = 0;

        do {
            t |= *p++;
        } while (p < e);

        return t == 0;
    } else {
        /* Otherwise, use the unaligned memory access functions to
           handle the beginning and end of the buffer, with a couple
           of loops handling the middle aligned section.  */
        uint64_t t = ldq_he_p(buf);
        const uint64_t *p = (uint64_t *)(((uintptr_t)buf + 8) & -8);
        const uint64_t *e = (uint64_t *)(((uintptr_t)buf + len) & -8);

        for (; p + 8 <= e; p += 8) {
            __builtin_prefetch(p + 8);
            if (t) {
                return false;
            }
            t = p[0] | p[1] | p[2] | p[3] | p[4] | p[5] | p[6] | p[7];
        }
        while (p < e) {
            t |= *p++;
        }
        t |= ldq_he_p(buf + len - 8);

        return t == 0;
    }
}

#if defined(CONFIG_AVX2_OPT) || defined(__SSE2__)
#include <cpuid.h>
#include <x86intrin.h>

/* Note that we're going to check for LEN >= 64 for all of these.  */

#ifdef CONFIG_AVX2_OPT
#pragma GCC push_options
#pragma GCC target("avx2")

static bool
buffer_zero_avx2(const void *buf, size_t len)
{
    /* Begin with an unaligned head of 32 bytes.  */
    __m256i t = _mm256_loadu_si256(buf);
    __m256i *p = (__m256i *)(((uintptr_t)buf + 5 * 32) & -32);
    __m256i *e = (__m256i *)(((uintptr_t)buf + len) & -32);

    if (likely(p <= e)) {
        /* Loop over 32-byte aligned blocks of 128.  */
        do {
            __builtin_prefetch(p);
            if (unlikely(!_mm256_testz_si256(t, t))) {
                return false;
            }
            t = p[-4] | p[-3] | p[-2] | p[-1];
            p += 4;
        } while (p <= e);
    } else {
        t |= _mm256_loadu_si256(buf + 32);
        if (len <= 128) {
            goto last2;
        }
    }

    /* Finish the last block of 128 unaligned.  */
    t |= _mm256_loadu_si256(buf + len - 4 * 32);
    t |= _mm256_loadu_si256(buf + len - 3 * 32);
 last2:
    t |= _mm256_loadu_si256(buf + len - 2 * 32);
    t |= _mm256_loadu_si256(buf + len - 1 * 32);

    return _mm256_testz_si256(t, t);
}

#pragma GCC pop_options
#pragma GCC push_options
#pragma GCC target("avx")

static bool
buffer_zero_avx(const void *buf, size_t len)
{
    __m128i t = _mm_loadu_si128(buf);
    __m128i *p = (__m128i *)(((uintptr_t)buf + 5 * 16) & -16);
    __m128i *e = (__m128i *)(((uintptr_t)buf + len) & -16);

    /* Loop over 16-byte aligned blocks of 64.  */
    while (likely(p <= e)) {
        __builtin_prefetch(p);
        if (unlikely(!_mm_testz_si128(t, t))) {
            return false;
        }
        t = p[-4] | p[-3] | p[-2] | p[-1];
        p += 4;
    }

    /* Finish the last block of 64 unaligned.  */
    t |= _mm_loadu_si128(buf + len - 4 * 16);
    t |= _mm_loadu_si128(buf + len - 3 * 16);
    t |= _mm_loadu_si128(buf + len - 2 * 16);
    t |= _mm_loadu_si128(buf + len - 1 * 16);

    return _mm_testz_si128(t, t);
}

#pragma GCC pop_options
#pragma GCC push_options
#pragma GCC target("sse4")

static bool
buffer_zero_sse4(const void *buf, size_t len)
{
    __m128i t = _mm_loadu_si128(buf);
    __m128i *p = (__m128i *)(((uintptr_t)buf + 5 * 16) & -16);
    __m128i *e = (__m128i *)(((uintptr_t)buf + len) & -16);

    /* Loop over 16-byte aligned blocks of 64.  */
    while (likely(p <= e)) {
        __builtin_prefetch(p);
        if (unlikely(!_mm_testz_si128(t, t))) {
            return false;
        }
        t = p[-4] | p[-3] | p[-2] | p[-1];
        p += 4;
    }

    /* Finish the aligned tail.  */
    t |= e[-3];
    t |= e[-2];
    t |= e[-1];

    /* Finish the unaligned tail.  */
    t |= _mm_loadu_si128(buf + len - 16);

    return _mm_testz_si128(t, t);
}

#pragma GCC pop_options
#pragma GCC push_options
#pragma GCC target("sse2")
#endif /* CONFIG_AVX2_OPT */

static bool
buffer_zero_sse2(const void *buf, size_t len)
{
    __m128i t = _mm_loadu_si128(buf);
    __m128i *p = (__m128i *)(((uintptr_t)buf + 5 * 16) & -16);
    __m128i *e = (__m128i *)(((uintptr_t)buf + len) & -16);
    __m128i zero = _mm_setzero_si128();

    /* Loop over 16-byte aligned blocks of 64.  */
    while (likely(p <= e)) {
        __builtin_prefetch(p);
        t = _mm_cmpeq_epi8(t, zero);
        if (unlikely(_mm_movemask_epi8(t) != 0xFFFF)) {
            return false;
        }
        t = p[-4] | p[-3] | p[-2] | p[-1];
        p += 4;
    }

    /* Finish the aligned tail.  */
    t |= e[-3];
    t |= e[-2];
    t |= e[-1];

    /* Finish the unaligned tail.  */
    t |= _mm_loadu_si128(buf + len - 16);

    return _mm_movemask_epi8(_mm_cmpeq_epi8(t, zero)) == 0xFFFF;
}

#ifdef CONFIG_AVX2_OPT
#pragma GCC pop_options

/* These values must be most preferable alternative first.
   See test_buffer_is_zero_next_accel.  */
#define CACHE_AVX2    1
#define CACHE_AVX1    2
#define CACHE_SSE4    4
#define CACHE_SSE2    8

static unsigned cpuid_cache;
static accel_zero_fn buffer_accel;

static void init_accel(unsigned cache)
{
    accel_zero_fn fn;
    if (cache & CACHE_AVX2) {
        fn = buffer_zero_avx2;
    } else if (cache & CACHE_AVX1) {
        fn = buffer_zero_avx;
    } else if (cache & CACHE_SSE4) {
        fn = buffer_zero_sse4;
    } else if (cache & CACHE_SSE2) {
        fn = buffer_zero_sse2;
    } else {
        fn = buffer_zero_int;
    }
    buffer_accel = fn;
}

static void __attribute__((constructor)) init_cpuid_cache(void)
{
    int max = __get_cpuid_max(0, NULL);
    int a, b, c, d;
    unsigned cache = 0;

    if (max >= 1) {
        __cpuid(1, a, b, c, d);
        if (d & bit_SSE2) {
            cache |= CACHE_SSE2;
        }
        if (c & bit_SSE4_1) {
            cache |= CACHE_SSE4;
        }

        /* We must check that AVX is not just available, but usable.  */
        if ((c & bit_OSXSAVE) && (c & bit_AVX)) {
            int bv;
            __asm("xgetbv" : "=a"(bv), "=d"(d) : "c"(0));
            if ((bv & 6) == 6) {
                cache |= CACHE_AVX1;
                if (max >= 7) {
                    __cpuid_count(7, 0, a, b, c, d);
                    if (b & bit_AVX2) {
                        cache |= CACHE_AVX2;
                    }
                }
            }
        }
    }
    cpuid_cache = cache;
    init_accel(cache);
}

#define HAVE_NEXT_ACCEL
bool test_buffer_is_zero_next_accel(void)
{
    /* If no bits set, we just tested buffer_zero_int, and there
       are no more acceleration options to test.  */
    if (cpuid_cache == 0) {
        return false;
    }
    /* Disable the accelerator we used before and select a new one.  */
    cpuid_cache &= cpuid_cache - 1;
    init_accel(cpuid_cache);
    return true;
}
#endif /* CONFIG_AVX2_OPT */

static bool select_accel_fn(const void *buf, size_t len)
{
    if (likely(len >= 64)) {
#ifdef CONFIG_AVX2_OPT
        return buffer_accel(buf, len);
#else
        return buffer_zero_sse2(buf, len);
#endif
    }
    return buffer_zero_int(buf, len);
}

#else
#define select_accel_fn  buffer_zero_int
#endif

#ifndef HAVE_NEXT_ACCEL
bool test_buffer_is_zero_next_accel(void)
{
    return false;
}
#endif

/*
 * Checks if a buffer is all zeroes
 */
bool buffer_is_zero(const void *buf, size_t len)
{
    if (unlikely(len == 0)) {
        return true;
    }

    /* Fetch the beginning of the buffer while we select the accelerator.  */
    __builtin_prefetch(buf);

    /* Use an optimized zero check if possible.  Note that this also
       includes a check for an unrolled loop over longs, as well as
       the unsized, unaligned fallback to buffer_zero_base.  */
    return select_accel_fn(buf, len);
}
