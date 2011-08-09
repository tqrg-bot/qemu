/*
 * Simple interface for atomic operations.
 *
 * Copyright (C) 2012 Red Hat, Inc.
 *
 * Author: Paolo Bonzini <pbonzini@redhat.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 *
 */

#ifndef __QEMU_ATOMIC_H
#define __QEMU_ATOMIC_H 1

#include "qemu/compiler.h"

/* Compiler barrier */
#define barrier()   ({ asm volatile("" ::: "memory"); (void)0; })

#if defined(__i386__) || defined(__x86_64__)

/*
 * Because of the strongly ordered x86 storage model, wmb() and rmb() are nops
 * on x86 (a compiler barrier only).  Well, at least as long as
 * qemu doesn't do accesses to write-combining memory or non-temporal
 * load/stores from C code.
 */
#define smp_wmb()   barrier()
#define smp_rmb()   barrier()

/*
 * We use GCC builtin if it's available, as that can use mfence on
 * 32-bit as well, e.g. if built with -march=pentium-m. However, on
 * i386 the spec is buggy, and the implementation followed it until
 * 4.3 (http://gcc.gnu.org/bugzilla/show_bug.cgi?id=36793).
 */
#if QEMU_GNUC_PREREQ(4, 4)
#define smp_mb()    __sync_synchronize()
#else
#define smp_mb()    ({ asm volatile("lock; addl $0,0(%%esp) " ::: "memory"); (void)0; })
#endif

/*
 * __sync_lock_test_and_set() is documented to be an acquire barrier only,
 * but it is a full barrier at the hardware level.  Add a compiler barrier
 * to make it a full barrier also at the compiler level.
 */
#define atomic_xchg(ptr, i)    (barrier(), __sync_lock_test_and_set(ptr, i))

/*
 * Load/store with Java volatile semantics.
 */
#define atomic_mb_set(ptr, i)  ((void)atomic_xchg(ptr, i))

#elif defined(_ARCH_PPC)

/*
 * We use an eieio() for wmb() on powerpc.  This assumes we don't
 * need to order cacheable and non-cacheable stores with respect to
 * each other.
 *
 * smp_mb has the same problem as on x86 for not-very-new GCC
 * (http://patchwork.ozlabs.org/patch/126184/, Nov 2011).
 */
#define smp_wmb()   ({ asm volatile("eieio" ::: "memory"); (void)0; })
#if defined(__powerpc64__)
#define smp_rmb()   ({ asm volatile("lwsync" ::: "memory"); (void)0; })
#else
#define smp_rmb()   ({ asm volatile("sync" ::: "memory"); (void)0; })
#endif
#define smp_mb()    ({ asm volatile("sync" ::: "memory"); (void)0; })

#endif

/*
 * For (host) platforms we don't have explicit barrier definitions
 * for, we use the gcc __sync_synchronize() primitive to generate a
 * full barrier.  This should be safe on all platforms, though it may
 * be overkill for smp_wmb() and smp_rmb().
 */
#ifndef smp_mmb
#define smp_wmb()   __sync_synchronize()
#endif

#ifndef smp_mb
#define smp_mb()    __sync_synchronize()
#endif

#ifndef smp_rmb
#define smp_rmb()   __sync_synchronize()
#endif

#ifndef atomic_read
#define atomic_read(ptr)       (*(__typeof__(*ptr) *volatile) (ptr))
#endif

#ifndef atomic_set
#define atomic_set(ptr, i)     ((*(__typeof__(*ptr) *volatile) (ptr)) = (i))
#endif

/* These have the same semantics as Java volatile variables.
 * See http://gee.cs.oswego.edu/dl/jmm/cookbook.html:
 * "1. Issue a StoreStore barrier (wmb) before each volatile store."
 *  2. Issue a StoreLoad barrier after each volatile store.
 *     Note that you could instead issue one before each volatile load, but
 *     this would be slower for typical programs using volatiles in which
 *     reads greatly outnumber writes. Alternatively, if available, you
 *     can implement volatile store as an atomic instruction (for example
 *     XCHG on x86) and omit the barrier. This may be more efficient if
 *     atomic instructions are cheaper than StoreLoad barriers. 
 *  3. Issue LoadLoad and LoadStore barriers after each volatile load."
 *
 * If you prefer to think in terms of "pairing" of memory barriers,
 * an atomic_mb_read pairs with an atomic_mb_set.
 *
 * And for the few ia64 lovers that exist, an atomic_mb_read is a ld.acq,
 * while an atomic_mb_set is a st.rel followed by a memory barrier.
 */
#ifndef atomic_mb_read
#define atomic_mb_read(ptr)    (atomic_read(ptr), smp_rmb())
#endif

#ifndef atomic_mb_set
#define atomic_mb_set(ptr, i)  (smp_wmb(), atomic_set(ptr, i), smp_mb())
#endif

/*
 * __sync_lock_test_and_set() is documented to be an acquire barrier only.
 */
#ifndef atomic_xchg
#define atomic_xchg(ptr, i)    (smp_mb(), __sync_lock_test_and_set(ptr, i))
#endif

/* Provide shorter names for GCC atomic builtins.  */
#define atomic_inc(ptr)        __sync_fetch_and_add(ptr, 1)
#define atomic_dec(ptr)        __sync_fetch_and_add(ptr, -1)
#define atomic_add             __sync_fetch_and_add
#define atomic_sub             __sync_fetch_and_sub
#define atomic_and             __sync_fetch_and_and
#define atomic_or              __sync_fetch_and_or
#define atomic_cmpxchg         __sync_val_compare_and_swap

#endif
