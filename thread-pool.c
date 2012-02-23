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
#include "qemu-common.h"
#include "qemu-queue.h"
#include "qemu-thread.h"
#include "osdep.h"
#include "qemu-common.h"
#include "trace.h"
#include "block_int.h"
#include "event_notifier.h"
#include "thread-pool.h"

static void do_spawn_thread(void);

typedef struct ThreadPoolElement ThreadPoolElement;

enum ThreadState {
    THREAD_QUEUED,
    THREAD_ACTIVE,
    THREAD_DONE,
    THREAD_CANCELED,
};

typedef struct ThreadStaticData ThreadStaticData;

struct ThreadStaticData {
    ThreadPoolElement *elem;
    QSLIST_ENTRY(ThreadStaticData) next;
};

struct ThreadPoolElement {
    BlockDriverAIOCB common;
    ThreadPoolFunc *func;
    void *arg;
    enum ThreadState state;
    int ret;

    struct ThreadStaticData *tdata;

    QTAILQ_ENTRY(ThreadPoolElement) reqs;
    QLIST_ENTRY(ThreadPoolElement) all;
};

static EventNotifier notifier;
static QemuMutex lock;
static QemuSemaphore sem;
static int max_threads = 64;
static int cur_threads = 0;
static int idle_threads = 0;
static int new_threads = 0;     /* backlog of threads we need to create */
static int pending_threads = 0; /* threads created but not running yet */
static QEMUBH *new_thread_bh;
static QLIST_HEAD(, ThreadPoolElement) head;
static QTAILQ_HEAD(, ThreadPoolElement) request_list;

static QSLIST_HEAD(, ThreadStaticData) data_pool;
static QemuMutex data_lock;

static __thread ThreadStaticData *tdata;

static void free_thread_data(ThreadStaticData *data)
{
    qemu_mutex_lock(&data_lock);
    QSLIST_INSERT_HEAD(&data_pool, data, next);
    qemu_mutex_unlock(&data_lock);
}

static ThreadStaticData *new_thread_data(void)
{
    ThreadStaticData *ret = g_malloc0(sizeof (ThreadStaticData));

    return ret;
}

static ThreadStaticData *get_thread_data(void)
{
    qemu_mutex_lock(&data_lock);
    if (QSLIST_EMPTY(&data_pool)) {
        qemu_mutex_unlock(&data_lock);
        return new_thread_data();
    } else {
        ThreadStaticData *ret = QSLIST_FIRST(&data_pool);
        QSLIST_REMOVE_HEAD(&data_pool, next);
        qemu_mutex_unlock(&data_lock);
        return ret;
    }
}

ThreadPoolElement *thread_self(void)
{
    return tdata ? tdata->elem : NULL;
}

bool qemu_in_worker(void)
{
    return !!tdata;
}

static void *worker_thread(void *unused)
{
    tdata = get_thread_data();

    qemu_mutex_lock(&lock);
    pending_threads--;
    qemu_mutex_unlock(&lock);
    do_spawn_thread();

    while (1) {
        ThreadPoolElement *req;
        int ret;

        qemu_mutex_lock(&lock);
        idle_threads++;
        qemu_mutex_unlock(&lock);
        ret = qemu_sem_timedwait(&sem, 10000);
        qemu_mutex_lock(&lock);
        idle_threads--;
        if (ret == -1) {
            if (QTAILQ_EMPTY(&request_list)) {
                break;
            }
            qemu_mutex_unlock(&lock);
            continue;
        }

        req = QTAILQ_FIRST(&request_list);
        QTAILQ_REMOVE(&request_list, req, reqs);
        req->tdata = tdata;
        tdata->elem = req;
        req->state = THREAD_ACTIVE;
        qemu_mutex_unlock(&lock);

        ret = req->func(req->arg);

        qemu_mutex_lock(&lock);
        req->state = THREAD_DONE;
        req->ret = ret;
        qemu_mutex_unlock(&lock);

        event_notifier_set(&notifier);
    }

    cur_threads--;
    qemu_mutex_unlock(&lock);

    free_thread_data(tdata);
    return NULL;
}

static void do_spawn_thread(void)
{
    QemuThread t;

    qemu_mutex_lock(&lock);
    if (!new_threads) {
        qemu_mutex_unlock(&lock);
        return;
    }

    new_threads--;
    pending_threads++;

    qemu_mutex_unlock(&lock);

    qemu_thread_create(&t, worker_thread, NULL, QEMU_THREAD_DETACHED);
}

static void spawn_thread_bh_fn(void *opaque)
{
    do_spawn_thread();
}

static void spawn_thread(void)
{
    cur_threads++;
    new_threads++;
    /* If there are threads being created, they will spawn new workers, so
     * we don't spend time creating many threads in a loop holding a mutex or
     * starving the current vcpu.
     *
     * If there are no idle threads, ask the main thread to create one, so we
     * inherit the correct affinity instead of the vcpu affinity.
     */
    if (!pending_threads) {
        qemu_bh_schedule(new_thread_bh);
    }
}

static void event_notifier_ready(void *opaque)
{
    ThreadPoolElement *elem, *next;

    event_notifier_test_and_clear(&notifier);
restart:
    QLIST_FOREACH_SAFE(elem, &head, all, next) {
        if (elem->state != THREAD_CANCELED && elem->state != THREAD_DONE) {
            continue;
        }
        if (elem->state == THREAD_DONE) {
            trace_thread_pool_complete(elem, elem->common.opaque, elem->ret);
        }
        if (elem->state == THREAD_DONE && elem->common.cb) {
            QLIST_REMOVE(elem, all);
            elem->common.cb(elem->common.opaque, elem->ret);
            qemu_aio_release(elem);
            goto restart;
        } else {
            /* remove the request */
            QLIST_REMOVE(elem, all);
            qemu_aio_release(elem);
	}
    }
}

static int thread_pool_active(void *opaque)
{
    return !QLIST_EMPTY(&head);
}

static void thread_pool_cancel(BlockDriverAIOCB *acb)
{
    ThreadPoolElement *elem = (ThreadPoolElement *)acb;

    trace_thread_pool_cancel(elem, elem->common.opaque);

    qemu_mutex_lock(&lock);
    if (elem->state == THREAD_QUEUED) {
        if (qemu_sem_timedwait(&sem, 0) == 0) {
            QTAILQ_REMOVE(&request_list, elem, reqs);
            elem->state = THREAD_CANCELED;
            event_notifier_set(&notifier);
        }
    }
    qemu_mutex_unlock(&lock);

    while (elem->state != THREAD_CANCELED && elem->state != THREAD_DONE) {
        /* TODO: remove busy wait */
    }
}

static AIOPool thread_pool_cb_pool = {
    .aiocb_size         = sizeof(ThreadPoolElement),
    .cancel             = thread_pool_cancel,
};

BlockDriverAIOCB *thread_pool_submit_aio(ThreadPoolFunc *func, void *arg,
        BlockDriverCompletionFunc *cb, void *opaque)
{
    ThreadPoolElement *req;

    req = qemu_aio_get(&thread_pool_cb_pool, NULL, cb, opaque);
    req->func = func;
    req->arg = arg;
    req->state = THREAD_QUEUED;

    QLIST_INSERT_HEAD(&head, req, all);

    trace_thread_pool_submit(req, arg);

    qemu_mutex_lock(&lock);
    if (idle_threads == 0 && cur_threads < max_threads) {
        spawn_thread();
    }
    QTAILQ_INSERT_TAIL(&request_list, req, reqs);
    qemu_mutex_unlock(&lock);
    qemu_sem_post(&sem);
    return &req->common;
}

ThreadPoolElement *thread_pool_submit(ThreadPoolFunc *func, void *arg)
{
    return (ThreadPoolElement *) thread_pool_submit_aio(func, arg, NULL, NULL);
}

static void thread_pool_init(void)
{
    QLIST_INIT(&head);
    event_notifier_init(&notifier, false);
    qemu_mutex_init(&lock);
    qemu_mutex_init(&data_lock);
    qemu_sem_init(&sem, 0);
    qemu_aio_set_fd_handler(event_notifier_get_fd(&notifier),
                            event_notifier_ready, NULL,
                            thread_pool_active, NULL);

    QTAILQ_INIT(&request_list);
    new_thread_bh = qemu_bh_new(spawn_thread_bh_fn, NULL);
}

block_init(thread_pool_init)
