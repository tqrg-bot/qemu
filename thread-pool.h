/*
 * QEMU block layer thread pool
 *
 * Copyright IBM, Corp. 2008
 * Copyright Red Hat, Inc. 2012
 *
 * Authors:
 *  Anthony Liguori   <aliguori@us.ibm.com>
 *  Paolo Bonzini     <pbonzini@redhat.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 *
 * Contributions after 2012-01-13 are licensed under the terms of the
 * GNU GPL, version 2 or (at your option) any later version.
 */

#ifndef QEMU_THREAD_POOL_H
#define QEMU_THREAD_POOL_H 1

#include "qemu-common.h"
#include "qemu-queue.h"
#include "qemu-thread.h"
#include "qemu-coroutine.h"
#include "block_int.h"

typedef struct ThreadPoolElement ThreadPoolElement;
typedef int ThreadPoolFunc(void *opaque);

ThreadPoolElement *thread_pool_submit(ThreadPoolFunc *func, void *arg);
BlockDriverAIOCB *thread_pool_submit_aio(ThreadPoolFunc *func, void *arg,
     BlockDriverCompletionFunc *cb, void *opaque);

ThreadPoolElement *thread_self(void);
bool qemu_in_worker(void);

#endif
