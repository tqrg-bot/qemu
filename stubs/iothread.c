#include "qemu/osdep.h"
#include "block/aio.h"
#include "qemu/main-loop.h"

bool aio_context_in_iothread(AioContext *ctx)
{
    return ctx == qemu_get_aio_context();
}
