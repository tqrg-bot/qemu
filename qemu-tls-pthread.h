/*
 * TLS with pthread_getspecific
 *
 * Copyright Red Hat, Inc. 2011
 *
 * Authors:
 *  Paolo Bonzini   <pbonzini@us.ibm.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 *
 */

#ifndef QEMU_TLS_PTHREAD_H
#define QEMU_TLS_PTHREAD_H

#include <pthread.h>
#include <glib.h>

#define DECLARE_TLS(type, x)                                     \
  extern size_t tls_offset__##x;                                 \
  extern __typeof__(type) *tls_dummy__##x(void)

#define DEFINE_TLS(type, x)                                      \
  size_t tls_offset__##x;                                        \
  static void __attribute__((constructor)) tls_init__##x(void)   \
  {                                                              \
    tls_offset__##x = tls_init(sizeof(type), __alignof__(type)); \
  }                                                              \
  extern inline __attribute__((__gnu_inline__)) __typeof__(type) *tls_dummy__##x(void) { \
    return NULL;                                                 \
  }                                                              \
  extern size_t tls_swallow_semicolon__##x

extern size_t tls_size;
extern pthread_key_t tls_key;

static inline size_t tls_init(size_t size, size_t alignment)
{
  size_t tls_offset = (tls_size + alignment - 1) & -alignment;
  tls_size = tls_offset + size;
  return tls_offset;
}

static inline void tls_init_thread(void)
{
  void *mem = tls_size == 0 ? NULL : g_malloc0(tls_size);
  pthread_setspecific(tls_key, mem);
}

static inline __attribute__((__const__)) void *_tls_var(size_t offset)
{
  char *base = pthread_getspecific(tls_key);
  return &base[offset];
}

#define tls_var(x) \
  (*(__typeof__(tls_dummy__##x())) _tls_var(tls_offset__##x))

#endif
