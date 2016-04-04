/*
 * Atomic operations on 64-bit quantities.
 *
 * Copyright (C) 2016 Red Hat, Inc.
 *
 * Author: Paolo Bonzini <pbonzini@redhat.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qemu/atomic.h"
#include "qemu/stats64.h"

#ifdef STAT64_NEED_SPINLOCK
static inline void stat64_rdlock(Stat64 *s)
{
    /* Keep out incoming writers to avoid them starving us. */
    atomic_add(&s->lock, 2);

    /* If there is a concurrent writer, wait for it.  */
    while (atomic_read(&s->lock) & 1) {
        g_usleep(5);
    }
}

static inline void stat64_rdunlock(Stat64 *s)
{
    atomic_sub(&s->lock, 2);
}

static inline bool stat64_wrtrylock(Stat64 *s)
{
    return atomic_cmpxchg(&s->lock, 0, 1) == 0;
}

static inline void stat64_wrunlock(Stat64 *s)
{
    atomic_dec(&s->lock);
}

uint64_t stat64_get(const Stat64 *s)
{
    uint32_t high, low;

    stat64_rdlock((Stat64 *)s);

    /* 64-bit writes always take the lock, so we can read in
     * any order.
     */
    high = atomic_read(&s->high);
    low = atomic_read(&s->low);
    stat64_rdunlock((Stat64 *)s);

    return ((uint64_t)high << 32) | low;
}

void stat64_set(Stat64 *s, uint64_t value)
{
    while (!stat64_wrtrylock(s)) {
        while (atomic_read(&s->lock)) {
            continue;
        }
    }
    atomic_set(&s->high, value >> 32);
    atomic_set(&s->low, (uint32_t)value);
    stat64_wrunlock(s);
}

bool stat64_add32_carry(Stat64 *s, uint32_t value)
{
    uint32_t old;

    if (!stat64_wrtrylock(s)) {
        return false;
    }

    /* 64-bit reads always take the lock, so we can update in
     * any order.  Because there might have been a concurrent
     * write of s->low, check whether we really have to carry
     * into s->high.
     */
    old = atomic_fetch_add(&s->low, value);
    if (old + value < old) {
        atomic_inc(&s->high);
    }
    stat64_wrunlock(s);
    return true;
}

bool stat64_min_slow(Stat64 *s, uint64_t value)
{
    uint32_t high, low;
    uint64_t orig;

    if (!stat64_wrtrylock(s)) {
        return false;
    }

    high = atomic_read(&s->high);
    low = atomic_read(&s->low);

    orig = ((uint64_t)high << 32) | low;
    if (orig < value) {
        /* The value may become higher temporarily, but stat64_get does not
         * notice (it takes the lock) and the only effect on stat64_max is
         * that the slow path may be triggered a bit more often.
         */
        atomic_set(&s->low, (uint32_t)value);
        smp_wmb();
        atomic_set(&s->high, value >> 32);
    }
    stat64_wrunlock(s);
    return true;
}

bool stat64_max_slow(Stat64 *s, uint64_t value)
{
    uint32_t high, low;
    uint64_t orig;

    if (!stat64_wrtrylock(s)) {
        return false;
    }

    high = atomic_read(&s->high);
    low = atomic_read(&s->low);

    orig = ((uint64_t)high << 32) | low;
    if (orig > value) {
        /* The value may become lower temporarily, but stat64_get does not
         * notice (it takes the lock) and the only effect on stat64_max is
         * that the slow path may be triggered a bit more often.
         */
        atomic_set(&s->low, (uint32_t)value);
        smp_wmb();
        atomic_set(&s->high, value >> 32);
    }
    stat64_wrunlock(s);
    return true;
}
#endif
