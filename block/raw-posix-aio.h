/*
 * QEMU Posix block I/O backend AIO support
 *
 * Copyright IBM, Corp. 2008
 *
 * Authors:
 *  Anthony Liguori   <aliguori@us.ibm.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 *
 * Contributions after 2012-01-13 are licensed under the terms of the
 * GNU GPL, version 2 or (at your option) any later version.
 */
#ifndef QEMU_RAW_POSIX_AIO_H
#define QEMU_RAW_POSIX_AIO_H

/* AIO flags */
#define QEMU_AIO_MISALIGNED   0x1000


/* posix-aio-compat.c - thread pool based implementation */
int paio_init(void);
BlockDriverAIOCB *paio_submit(BlockDriverState *bs, int fd,
        int64_t sector_num, QEMUIOVector *qiov, int nb_sectors,
        BlockDriverCompletionFunc *cb, void *opaque, int type);

/* linux-aio.c - Linux native implementation */
void *laio_init(void);
BlockDriverAIOCB *laio_submit(BlockDriverState *bs, void *aio_ctx, int fd,
        int64_t sector_num, QEMUIOVector *qiov, int nb_sectors,
        BlockDriverCompletionFunc *cb, void *opaque, int type);

#endif /* QEMU_RAW_POSIX_AIO_H */
