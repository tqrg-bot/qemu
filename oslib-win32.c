/*
 * os-win32.c
 *
 * Copyright (c) 2003-2008 Fabrice Bellard
 * Copyright (c) 2010 Red Hat, Inc.
 *
 * QEMU library functions for win32 which are shared between QEMU and
 * the QEMU tools.
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
#include <windows.h>
#include "config-host.h"
#include "sysemu.h"
#include "main-loop.h"
#include "trace.h"
#include "qemu_socket.h"

void *qemu_oom_check(void *ptr)
{
    if (ptr == NULL) {
        fprintf(stderr, "Failed to allocate memory: %lu\n", GetLastError());
        abort();
    }
    return ptr;
}

void *qemu_memalign(size_t alignment, size_t size)
{
    void *ptr;

    if (!size) {
        abort();
    }
    ptr = qemu_oom_check(VirtualAlloc(NULL, size, MEM_COMMIT, PAGE_READWRITE));
    trace_qemu_memalign(alignment, size, ptr);
    return ptr;
}

void *qemu_vmalloc(size_t size)
{
    void *ptr;

    /* FIXME: this is not exactly optimal solution since VirtualAlloc
       has 64Kb granularity, but at least it guarantees us that the
       memory is page aligned. */
    if (!size) {
        abort();
    }
    ptr = qemu_oom_check(VirtualAlloc(NULL, size, MEM_COMMIT, PAGE_READWRITE));
    trace_qemu_vmalloc(size, ptr);
    return ptr;
}

void qemu_vfree(void *ptr)
{
    trace_qemu_vfree(ptr);
    VirtualFree(ptr, 0, MEM_RELEASE);
}

void socket_set_block(int fd)
{
    unsigned long opt = 0;
    WSAEventSelect(_get_osfhandle(fd), NULL, 0);
    ioctlsocket(_get_osfhandle(fd), FIONBIO, &opt);
}

void socket_set_nonblock(int fd)
{
    unsigned long opt = 1;
    ioctlsocket(_get_osfhandle(fd), FIONBIO, &opt);
    qemu_fd_register(fd);
}

int inet_aton(const char *cp, struct in_addr *ia)
{
    uint32_t addr = inet_addr(cp);
    if (addr == 0xffffffff) {
	return 0;
    }
    ia->s_addr = addr;
    return 1;
}

void qemu_set_cloexec(int fd)
{
}

/* Offset between 1/1/1601 and 1/1/1970 in 100 nanosec units */
#define _W32_FT_OFFSET (116444736000000000ULL)

int qemu_gettimeofday(qemu_timeval *tp)
{
  union {
    unsigned long long ns100; /*time since 1 Jan 1601 in 100ns units */
    FILETIME ft;
  }  _now;

  if(tp) {
      GetSystemTimeAsFileTime (&_now.ft);
      tp->tv_usec=(long)((_now.ns100 / 10ULL) % 1000000ULL );
      tp->tv_sec= (long)((_now.ns100 - _W32_FT_OFFSET) / 10000000ULL);
  }
  /* Always return 0 as per Open Group Base Specifications Issue 6.
     Do not set errno on error.  */
  return 0;
}

int qemu_get_thread_id(void)
{
    return GetCurrentThreadId();
}

int qemu_close_socket(int fd)
{
    SOCKET s = _get_osfhandle(fd);
    int rc = closesocket(s);
    if (rc < 0) {
        rc = -socket_error();
    }
    close(fd);
    return rc;
}

int qemu_listen(int fd, int backlog)
{
    int rc = listen(_get_osfhandle(fd), backlog);
    if (rc < 0) {
        rc = -socket_error();
    }
    return rc;
}

int qemu_bind(int fd, const struct sockaddr *addr, socklen_t addrlen)
{
    int rc = bind(_get_osfhandle(fd), addr, addrlen);
    if (rc < 0) {
        rc = -socket_error();
    }
    return rc;
}

int qemu_connect(int fd, const struct sockaddr *addr, socklen_t addrlen)
{
    int rc = connect(_get_osfhandle(fd), addr, addrlen);
    if (rc < 0) {
        rc = -socket_error();
    }
    return rc;
}

int qemu_getsockopt(int fd, int level, int opt, void *val, socklen_t *len)
{
    int rc = getsockopt(_get_osfhandle(fd), level, opt, val, len);
    if (rc < 0) {
        rc = -socket_error();
    }
    return rc;
}

int qemu_accept(int s, struct sockaddr *addr, socklen_t *addrlen)
{
    int fd = accept(_get_osfhandle(s), addr, addrlen);
    if (fd < 0) {
        return -socket_error();
    }
    return _open_osfhandle(fd, O_RDWR | O_BINARY);
}

int qemu_setsockopt(int fd, int level, int opt, const void *val, socklen_t len)
{
    int rc = setsockopt(_get_osfhandle(fd), level, opt, val, len);
    if (rc < 0) {
        rc = -socket_error();
    }
    return rc;
}

/*
 * Opens a socket with FD_CLOEXEC set.  Allow using ReadFile/WriteFile on it.
 */
int qemu_socket(int domain, int type, int protocol)
{
    /* WSASocket() creates a non-overlapped IO socket, which can be
     * used with ReadFile/WriteFile.  */
    int fd = WSASocket(domain, type, protocol, NULL, 0, 0);
    if (fd < 0) {
        return -socket_error();
    }
    return _open_osfhandle(fd, O_RDWR | O_BINARY);
}

static void fdset_to_sdset(fd_set *fdset, fd_set *sdset1, fd_set *sdset2)
{
    int i;
    sdset1->fd_count = sdset1->fd_count;
    sdset2->fd_count = sdset2->fd_count;
    for (i = 0; i < fdset->fd_count; i++) {
        SOCKET sd = _get_osfhandle(fdset->fd_array[i]);
        sdset1->fd_array[i] = sdset2->fd_array[i] = sd;
    }
}

static void sdset_to_fdset(fd_set *fdset, fd_set *sdset, fd_set *orig_sdset)
{
    int i, j;
    for (i = j = 0; i < orig_sdset->fd_count; i++) {
        if (orig_sdset->fd_array[i] == sdset->fd_array[j]) {
            fdset->fd_array[j++] = fdset->fd_array[i];
        }
    }
    fdset->fd_count = j;
}

int qemu_select(int nfds, fd_set *readfds, fd_set *writefds,
                fd_set *exceptfds, struct timeval *timeout)
{
    fd_set readsds, writesds, exceptsds;
    fd_set orig_readsds, orig_writesds, orig_exceptsds;
    fd_set *p_readsds = NULL, *p_writesds = NULL, *p_exceptsds = NULL;
    int rc;
    if (readfds) {
        p_readsds = &readsds;
        fdset_to_sdset(readfds, &readsds, &orig_readsds);
    }
    if (writefds) {
        p_writesds = &writesds;
        fdset_to_sdset(writefds, &writesds, &orig_writesds);
    }
    if (exceptfds) {
        p_exceptsds = &exceptsds;
        fdset_to_sdset(exceptfds, &exceptsds, &orig_exceptsds);
    }
    rc = select(nfds, p_readsds, p_writesds, p_exceptsds, timeout);
    if (rc < 0) {
        return -socket_error();
    }
    if (readfds) {
        sdset_to_fdset(readfds, &readsds, &orig_readsds);
    }
    if (writefds) {
        sdset_to_fdset(writefds, &writesds, &orig_writesds);
    }
    if (exceptfds) {
        sdset_to_fdset(exceptfds, &exceptsds, &orig_exceptsds);
    }
    return rc;
}
