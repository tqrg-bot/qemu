/*
 * ucontext coroutine initialization code
 *
 * Copyright (C) 2006  Anthony Liguori <anthony@codemonkey.ws>
 * Copyright (C) 2011  Kevin Wolf <kwolf@redhat.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.0 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

/* XXX Is there a nicer way to disable glibc's stack check for longjmp? */
#ifdef _FORTIFY_SOURCE
#undef _FORTIFY_SOURCE
#endif
#include <stdlib.h>
#include <setjmp.h>
#include <stdint.h>
#include <pthread.h>
#include <ucontext.h>
#include "qemu-common.h"
#include "qemu-coroutine-int.h"

#ifndef SS_ONSTACK
#define SS_ONSTACK SA_ONSTACK
#endif

/* Pick the most obscure signal available. :)  */
#ifdef SIGSYS
#define SIGCOROUTINE SIGSYS
#elif defined SIGEMT
#define SIGCOROUTINE SIGEMT
#endif

enum {
    /* Maximum free pool size prevents holding too many freed coroutines */
    POOL_MAX_SIZE = 64,
};

/** Free list to speed up creation */
static QLIST_HEAD(, Coroutine) pool = QLIST_HEAD_INITIALIZER(pool);
static unsigned int pool_size;

typedef struct {
    Coroutine base;
    void *stack;
    jmp_buf env;
} CoroutineUContext;

/**
 * Per-thread coroutine bookkeeping
 */
typedef struct {
    /** Currently executing coroutine */
    Coroutine *current;

    /** The default coroutine */
    CoroutineUContext leader;
} CoroutineThreadState;

static pthread_key_t thread_state_key;

static CoroutineThreadState *coroutine_get_thread_state(void)
{
    CoroutineThreadState *s = pthread_getspecific(thread_state_key);

    if (!s) {
        s = g_malloc0(sizeof(*s));
        s->current = &s->leader.base;
        pthread_setspecific(thread_state_key, s);
    }
    return s;
}

static void qemu_coroutine_thread_cleanup(void *opaque)
{
    CoroutineThreadState *s = opaque;

    g_free(s);
}

static void __attribute__((destructor)) coroutine_cleanup(void)
{
    Coroutine *co;
    Coroutine *tmp;

    QLIST_FOREACH_SAFE(co, &pool, pool_next, tmp) {
        g_free(DO_UPCAST(CoroutineUContext, base, co)->stack);
        g_free(co);
    }
}

static void coroutine_trampoline(void)
{
    CoroutineThreadState *s = coroutine_get_thread_state();
    Coroutine *co = s->current;
    CoroutineUContext *self = DO_UPCAST(CoroutineUContext, base, co);

    /* Initialize longjmp environment and switch back the caller */
    if (!setjmp(self->env)) {
        siglongjmp(*(jmp_buf *)co->entry_arg, 1);
    }

    while (true) {
        co->entry(co->entry_arg);
        qemu_coroutine_switch(co, co->caller, COROUTINE_TERMINATE);
    }
}

static void __attribute__((constructor)) coroutine_init(void)
{
    int ret;
    struct sigaction sa;
    sigset_t set;

    ret = pthread_key_create(&thread_state_key, qemu_coroutine_thread_cleanup);
    if (ret != 0) {
        fprintf(stderr, "unable to create leader key: %s\n", strerror(errno));
        abort();
    }

    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = (void (*)(int)) coroutine_trampoline;
    sa.sa_flags = SA_ONSTACK;
    sigaction(SIGCOROUTINE, &sa, NULL);

    sigemptyset(&set);
    sigaddset(&set, SIGCOROUTINE);
    pthread_sigmask(SIG_BLOCK, &set, NULL);
}

static Coroutine *coroutine_new(void)
{
    const size_t stack_size = 1 << 20;
    CoroutineThreadState *s = coroutine_get_thread_state();
    Coroutine *current;
    CoroutineUContext *co;
    sigjmp_buf old_env;

    stack_t ss, oss;
    sigset_t set;

    /* The ucontext functions preserve signal masks which incurs a system call
     * overhead, and makecontext() is not portable anyway.  setjmp()/longjmp()
     * does not preserve signal masks and are fast, but only works on the
     * current stack.  So we use sigaltstack to create and switch to a new
     * stack, and setjmp()/longjmp() for everything else.
     *
     * Note: we use sigsetjmp()/siglongjmp() to get out of the just-created
     * alternate stack, because some IRIX and Solaris apparently believe
     * that you are still on the alternate stack if you longjmp out of it.
     * (getcontext/setcontext is an alternative that would also also work
     * for them, but this way we just avoid the ucontext functions).
     */

    co = g_malloc0(sizeof(*co));
    co->stack = g_malloc(stack_size);
    co->base.entry_arg = &old_env; /* stash away our jmp_buf */

    ss.ss_sp = co->stack;
    ss.ss_size = stack_size;
    ss.ss_flags = 0;
    sigaltstack(&ss, &oss);

    /* enter by signal, siglongjmp() back out. siglongjmp() will also
     * block SIGCOROUTINE again.  */
    current = s->current;
    s->current = &co->base;
    if (!sigsetjmp(old_env, 1)) {
        /* Queue the signal so that coroutine_trampoline is only entered
         * once.
         */
        pthread_kill(pthread_self(), SIGCOROUTINE);

        sigemptyset(&set);
        sigaddset(&set, SIGCOROUTINE);
        pthread_sigmask(SIG_UNBLOCK, &set, NULL);
    }
    s->current = current;

    sigaltstack(&oss, NULL);
    return &co->base;
}

Coroutine *qemu_coroutine_new(void)
{
    Coroutine *co;

    co = QLIST_FIRST(&pool);
    if (co) {
        QLIST_REMOVE(co, pool_next);
        pool_size--;
    } else {
        co = coroutine_new();
    }
    return co;
}

void qemu_coroutine_delete(Coroutine *co_)
{
    CoroutineUContext *co = DO_UPCAST(CoroutineUContext, base, co_);

    if (pool_size < POOL_MAX_SIZE) {
        QLIST_INSERT_HEAD(&pool, &co->base, pool_next);
        co->base.caller = NULL;
        pool_size++;
        return;
    }

    g_free(co->stack);
    g_free(co);
}

CoroutineAction qemu_coroutine_switch(Coroutine *from_, Coroutine *to_,
                                      CoroutineAction action)
{
    CoroutineUContext *from = DO_UPCAST(CoroutineUContext, base, from_);
    CoroutineUContext *to = DO_UPCAST(CoroutineUContext, base, to_);
    CoroutineThreadState *s = coroutine_get_thread_state();
    int ret;

    s->current = to_;

    ret = setjmp(from->env);
    if (ret == 0) {
        longjmp(to->env, action);
    }
    return ret;
}

Coroutine *qemu_coroutine_self(void)
{
    CoroutineThreadState *s = coroutine_get_thread_state();

    return s->current;
}

bool qemu_in_coroutine(void)
{
    CoroutineThreadState *s = pthread_getspecific(thread_state_key);

    return s && s->current->caller;
}
