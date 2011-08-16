/*
 * Wrappers around mutex/cond/thread functions
 *
 * Copyright Red Hat, Inc. 2009
 *
 * Author:
 *  Marcelo Tosatti <mtosatti@redhat.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 *
 */
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <time.h>
#include <signal.h>
#include <stdint.h>
#include <string.h>
#include "qemu-thread.h"
#include "qemu-common.h"
#include "qemu-tls.h"
#include "qemu-barrier.h"

static void error_exit(int err, const char *msg)
{
    fprintf(stderr, "qemu: %s: %s\n", msg, strerror(err));
    abort();
}

void qemu_mutex_init(QemuMutex *mutex)
{
    int err;
    pthread_mutexattr_t mutexattr;

    pthread_mutexattr_init(&mutexattr);
    pthread_mutexattr_settype(&mutexattr, PTHREAD_MUTEX_ERRORCHECK);
    err = pthread_mutex_init(&mutex->lock, &mutexattr);
    pthread_mutexattr_destroy(&mutexattr);
    if (err)
        error_exit(err, __func__);
}

void qemu_mutex_destroy(QemuMutex *mutex)
{
    int err;

    err = pthread_mutex_destroy(&mutex->lock);
    if (err)
        error_exit(err, __func__);
}

void qemu_mutex_lock(QemuMutex *mutex)
{
    int err;

    err = pthread_mutex_lock(&mutex->lock);
    if (err)
        error_exit(err, __func__);
}

int qemu_mutex_trylock(QemuMutex *mutex)
{
    return pthread_mutex_trylock(&mutex->lock);
}

void qemu_mutex_unlock(QemuMutex *mutex)
{
    int err;

    err = pthread_mutex_unlock(&mutex->lock);
    if (err)
        error_exit(err, __func__);
}

void qemu_cond_init(QemuCond *cond)
{
    int err;

    err = pthread_cond_init(&cond->cond, NULL);
    if (err)
        error_exit(err, __func__);
}

void qemu_cond_destroy(QemuCond *cond)
{
    int err;

    err = pthread_cond_destroy(&cond->cond);
    if (err)
        error_exit(err, __func__);
}

void qemu_cond_signal(QemuCond *cond)
{
    int err;

    err = pthread_cond_signal(&cond->cond);
    if (err)
        error_exit(err, __func__);
}

void qemu_cond_broadcast(QemuCond *cond)
{
    int err;

    err = pthread_cond_broadcast(&cond->cond);
    if (err)
        error_exit(err, __func__);
}

void qemu_cond_wait(QemuCond *cond, QemuMutex *mutex)
{
    int err;

    err = pthread_cond_wait(&cond->cond, &mutex->lock);
    if (err)
        error_exit(err, __func__);
}

size_t tls_size;
pthread_key_t tls_key;

void qemu_tls_init(void)
{
    /* It's easier to always create the key, even if using GCC tls.  */
    pthread_key_create(&tls_key, g_free);
    tls_init_thread();
}

typedef struct QemuThreadData {
    void *(*start_routine)(void *);
    void *arg;
} QemuThreadData;

static void *start_routine_wrapper(void *arg)
{
    QemuThreadData args = *(QemuThreadData *) arg;
    g_free(arg);
    tls_init_thread();
    return args.start_routine(args.arg);
}

void qemu_thread_create(QemuThread *thread,
                       void *(*start_routine)(void*),
                       void *arg, int mode)
{
    sigset_t set, oldset;
    QemuThreadData *args = g_malloc(sizeof(QemuThreadData));
    int err;
    pthread_attr_t attr;

    err = pthread_attr_init(&attr);
    if (err) {
        error_exit(err, __func__);
    }
    if (mode == QEMU_THREAD_DETACHED) {
        err = pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
        if (err) {
            error_exit(err, __func__);
        }
    }

    args->start_routine = start_routine;
    args->arg = arg;

    /* Leave signal handling to the iothread.  */
    sigfillset(&set);
    pthread_sigmask(SIG_SETMASK, &set, &oldset);
    err = pthread_create(&thread->thread, &attr, start_routine_wrapper, args);
    if (err)
        error_exit(err, __func__);

    pthread_sigmask(SIG_SETMASK, &oldset, NULL);

    pthread_attr_destroy(&attr);
}

void qemu_thread_get_self(QemuThread *thread)
{
    thread->thread = pthread_self();
}

int qemu_thread_is_self(QemuThread *thread)
{
   return pthread_equal(pthread_self(), thread->thread);
}

void qemu_thread_exit(void *retval)
{
    pthread_exit(retval);
}

void *qemu_thread_join(QemuThread *thread)
{
    int err;
    void *ret;

    err = pthread_join(thread->thread, &ret);
    if (err) {
        error_exit(err, __func__);
    }
    return ret;
}
