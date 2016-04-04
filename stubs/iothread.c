#include "qemu/osdep.h"
#include "block/aio.h"
#include "qemu/main-loop.h"

AioContext *qemu_get_current_aio_context(void)
{
    return qemu_get_aio_context();
}

bool aio_context_in_iothread(AioContext *ctx)
{
    return ctx == qemu_get_aio_context();
}
