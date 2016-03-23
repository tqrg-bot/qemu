/*
 * QEMU coroutine implementation
 *
 * Copyright IBM, Corp. 2011
 *
 * Authors:
 *  Stefan Hajnoczi    <stefanha@linux.vnet.ibm.com>
 *  Kevin Wolf         <kwolf@redhat.com>
 *
 * This work is licensed under the terms of the GNU LGPL, version 2 or later.
 * See the COPYING.LIB file in the top-level directory.
 *
 */

#ifndef QEMU_COROUTINE_H
#define QEMU_COROUTINE_H

#include "qemu/typedefs.h"
#include "qemu/queue.h"
#include "qemu/timer.h"

/**
 * Coroutines are a mechanism for stack switching and can be used for
 * cooperative userspace threading.  These functions provide a simple but
 * useful flavor of coroutines that is suitable for writing sequential code,
 * rather than callbacks, for operations that need to give up control while
 * waiting for events to complete.
 *
 * These functions are re-entrant and may be used outside the global mutex.
 */

/**
 * Mark a function that executes in coroutine context
 *
 * Functions that execute in coroutine context cannot be called directly from
 * normal functions.  In the future it would be nice to enable compiler or
 * static checker support for catching such errors.  This annotation might make
 * it possible and in the meantime it serves as documentation.
 *
 * For example:
 *
 *   static void coroutine_fn foo(void) {
 *       ....
 *   }
 */
#define coroutine_fn

typedef struct Coroutine Coroutine;

/**
 * Coroutine entry point
 *
 * When the coroutine is entered for the first time, opaque is passed in as an
 * argument.
 *
 * When this function returns, the coroutine is destroyed automatically and
 * execution continues in the caller who last entered the coroutine.
 */
typedef void coroutine_fn CoroutineEntry(void *opaque);

/**
 * Create a new coroutine
 *
 * Use qemu_coroutine_enter() to actually transfer control to the coroutine.
 */
Coroutine *qemu_coroutine_create(CoroutineEntry *entry);

/**
 * Transfer control to a coroutine
 *
 * The opaque argument is passed as the argument to the entry point when
 * entering the coroutine for the first time.  It is subsequently ignored.
 */
void qemu_coroutine_enter(Coroutine *coroutine, void *opaque);

/**
 * Transfer control back to a coroutine's caller
 *
 * This function does not return until the coroutine is re-entered using
 * qemu_coroutine_enter().
 */
void coroutine_fn qemu_coroutine_yield(void);

/**
 * Get the currently executing coroutine
 */
Coroutine *coroutine_fn qemu_coroutine_self(void);

/**
 * Return whether or not currently inside a coroutine
 *
 * This can be used to write functions that work both when in coroutine context
 * and when not in coroutine context.  Note that such functions cannot use the
 * coroutine_fn annotation since they work outside coroutine context.
 */
bool qemu_in_coroutine(void);



/**
 * CoQueues are a mechanism to queue coroutines in order to continue executing
 * them later. They provide the fundamental primitives on which coroutine locks
 * are built.
 */
typedef struct CoQueue {
    QTAILQ_HEAD(, Coroutine) entries;
} CoQueue;

/**
 * Initialise a CoQueue. This must be called before any other operation is used
 * on the CoQueue.
 */
void qemu_co_queue_init(CoQueue *queue);

/**
 * Adds the current coroutine to the CoQueue and transfers control to the
 * caller of the coroutine.
 */
void coroutine_fn qemu_co_queue_wait(CoQueue *queue);

/**
 * Adds the current coroutine to the CoQueue and transfers control to the
 * caller of the coroutine.  The coroutine is reentered immediately if
 * canceled.
 */
void coroutine_fn qemu_co_queue_cancelable_wait(CoQueue *queue);

/**
 * Remove @next from the queue and restart it as soon as the current
 * coroutine yields.
 */
void qemu_co_queue_restart(CoQueue *queue, Coroutine *next);

/**
 * Remove the next coroutine in the CoQueue, restarting it
 * as soon as the current coroutine yields.
 *
 * Returns true if a coroutine was restarted, false if the queue is empty.
 */
bool coroutine_fn qemu_co_queue_next(CoQueue *queue);

/**
 * Restarts all coroutines in the CoQueue and leaves the queue empty.
 * The first coroutine to be restarted will execute as soon as the
 * current coroutine yields.
 */
void coroutine_fn qemu_co_queue_restart_all(CoQueue *queue);

/**
 * Remove @next from the queue and enter it
 */
void qemu_co_queue_enter(CoQueue *queue, Coroutine *next);

/**
 * Enter the next coroutine in the queue
 */
bool qemu_co_enter_next(CoQueue *queue);

/**
 * Checks if the CoQueue is empty.
 */
bool qemu_co_queue_empty(CoQueue *queue);


/**
 * Provides a mutex that can be used to synchronise coroutines
 */
typedef struct CoMutex {
    bool locked;
    CoQueue queue;
} CoMutex;

/**
 * Initialises a CoMutex. This must be called before any other operation is used
 * on the CoMutex.
 */
void qemu_co_mutex_init(CoMutex *mutex);

/**
 * Locks the mutex. If the lock cannot be taken immediately, control is
 * transferred to the caller of the current coroutine.
 */
void coroutine_fn qemu_co_mutex_lock(CoMutex *mutex);

/**
 * Unlocks the mutex and schedules the next coroutine that was waiting for this
 * lock to be run.
 */
void coroutine_fn qemu_co_mutex_unlock(CoMutex *mutex);

typedef struct CoRwlock {
    bool writer;
    int reader;
    CoQueue queue;
} CoRwlock;

/**
 * Initialises a CoRwlock. This must be called before any other operation
 * is used on the CoRwlock
 */
void qemu_co_rwlock_init(CoRwlock *lock);

/**
 * Read locks the CoRwlock. If the lock cannot be taken immediately because
 * of a parallel writer, control is transferred to the caller of the current
 * coroutine.
 */
void qemu_co_rwlock_rdlock(CoRwlock *lock);

/**
 * Write Locks the mutex. If the lock cannot be taken immediately because
 * of a parallel reader, control is transferred to the caller of the current
 * coroutine.
 */
void qemu_co_rwlock_wrlock(CoRwlock *lock);

/**
 * Unlocks the read/write lock and schedules the next coroutine that was
 * waiting for this lock to be run.
 */
void qemu_co_rwlock_unlock(CoRwlock *lock);

/**
 * Yield the coroutine for a given duration
 *
 * Behaves similarly to co_sleep_ns(), but the sleeping coroutine will be
 * resumed when using aio_poll().
 */
void coroutine_fn co_aio_sleep_ns(AioContext *ctx, QEMUClockType type,
                                  int64_t ns);

/**
 * Yield until a file descriptor becomes readable
 *
 * Note that this function clobbers the handlers for the file descriptor.
 */
void coroutine_fn yield_until_fd_readable(int fd);

/*
 * Return whether qemu_coroutine_cancel has been called on this coroutine.
 */
bool qemu_coroutine_canceled(void);

/*
 * Mark @co as cancelled and call cancellation notifiers.
 */
void qemu_coroutine_cancel(Coroutine *co);

/*
 * Register @notify to be called on cancellation of the current coroutine.
 * The notifier will be called in the context of the caller of
 * qemu_coroutine_cancel---which may not be coroutine context at all.
 */
void qemu_coroutine_add_cancel_notifier(Notifier *notify);

/*
 * Do not call the notifier anymore if the coroutine is cancelled.
 */
void qemu_coroutine_remove_cancel_notifier(Notifier *notify);

#endif /* QEMU_COROUTINE_H */
