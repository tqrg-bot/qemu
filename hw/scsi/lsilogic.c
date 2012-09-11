/*
 * QEMU LSILOGIC LSI53C1030 SCSI and SAS1068 Host Bus Adapter emulation
 * Based on the QEMU Megaraid emulator and the VirtualBox LsiLogic
 * LSI53c1030 SCSI controller
 *
 * Copyright (c) 2009-2012 Hannes Reinecke, SUSE Labs
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

/* Id: DevLsiLogicSCSI.cpp 40642 2012-03-26 13:14:08Z vboxsync $ */
/** @file
 * VBox storage devices: LsiLogic LSI53c1030 SCSI controller.
 */

/*
 * Copyright (C) 2006-2009 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 */


#include "hw.h"
#include "pci.h"
#include "dma.h"
#include "msix.h"
#include "iov.h"
#include "scsi.h"
#include "scsi-defs.h"
#include "block_int.h"
#include "trace.h"

#include "lsilogic.h"

#define RT_ELEMENTS(aArray)        (sizeof(aArray) / sizeof((aArray)[0]))

#define LSILOGIC_MAX_FRAMES 2048     /* Firmware limit at 65535 */

#define NAA_LOCALLY_ASSIGNED_ID 0x3ULL
#define IEEE_COMPANY_LOCALLY_ASSIGNED 0x525400

#define LSILOGIC_FLAG_USE_MSIX      0
#define LSILOGIC_MASK_USE_MSIX      (1 << LSILOGIC_FLAG_USE_MSIX)
#define LSILOGIC_FLAG_USE_QUEUE64   1
#define LSILOGIC_MASK_USE_QUEUE64   (1 << LSILOGIC_FLAG_USE_QUEUE64)
#define LSILOGIC_CMD_BUSY   (1 << 0)

typedef struct LsilogicCmd {
    uint32_t index;
    uint16_t flags;
    uint16_t count;
    uint64_t context;

    target_phys_addr_t host_msg_frame_pa;
    MptRequestUnion request;
    MptReplyUnion reply;
    SCSIRequest *req;
    QEMUSGList qsg;
    uint32_t sge_cnt;
    void *iov_buf;
    size_t iov_size;
    size_t iov_offset;
    struct LsilogicState *state;
} LsilogicCmd;

typedef struct Lsilogic_device {
    struct LsilogicState *pLsiLogic;

    uint32_t iLUN;
    uint32_t cOutstandingRequests;
    uint32_t *pDrvBase;
} Lsilogic_device;

typedef struct LsilogicState {
    PCIDevice dev;
    MemoryRegion mmio_io;
    MemoryRegion port_io;
    MemoryRegion diag_io;

    MptConfigurationPagesSupported *config_pages;

    LSILOGICCTRLTYPE ctrl_type;
    LSILOGICSTATE state;
    LSILOGICWHOINIT who_init;
    uint16_t next_handle;
    uint32_t ports;
    uint32_t flags;
    uint32_t intr_mask;
    uint32_t intr_status;
    uint32_t doorbell;
    uint32_t busy;
    bool     event_notification_enabled;
    bool     diagnostic_enabled;
    uint32   diagnostic_access_idx;
    /** Maximum number of devices the driver reported he can handle. */
    uint16_t max_devices;
    /** Maximum number of buses the driver reported he can handle. */
    uint16_t max_buses;

    uint64_t sas_addr;

     /* Buffer for messages which are passed through the doorbell
      * using the handshake method.
      */
    uint32_t drbl_message[(sizeof(MptRequestUnion)+sizeof(uint32_t)-1)/
                                sizeof(uint32_t)];
    uint16_t drbl_message_index;
    uint16_t drbl_message_size; /** Size of the message in dwords. */

    MptReplyUnion reply_buffer;
    uint16_t next_reply_entry_read;
    uint16_t reply_size;        /* in 16bit words. */

    uint16_t IOC_fault_code;    /* if we are in the fault state. */
    /** Current size of reply message frames in the guest. */
    uint16_t reply_frame_size;
    /** Upper 32 bits of the message frame address to
        locate requests in guest memory. */
    uint32_t host_mfa_high_addr;
    /** Upper 32 bits of the sense buffer address. */
    uint32_t sense_buffer_high_addr;

    uint32_t reply_queue_entries;
    uint32_t request_queue_entries;

    uint32_t *reply_post_queue;
    uint32_t *reply_free_queue;
    uint32_t *request_queue;
    uint32_t reply_free_queue_next_entry_free_write;
    uint32_t reply_free_queue_next_address_read;

    uint32_t reply_post_queue_next_entry_free_write;
    uint32_t reply_post_queue_next_address_read;

    uint32_t request_queue_next_entry_free_write;
    uint32_t request_queue_next_address_read;

    uint32_t next_frame;
    LsilogicCmd * frames[LSILOGIC_MAX_FRAMES];

    SCSIBus bus;
} LsilogicState;

#define LSILOGIC_INTR_DISABLED_MASK 0xFFFFFFFF

static bool lsilogic_use_msix(LsilogicState *s)
{
    return s->flags & LSILOGIC_MASK_USE_MSIX;
}

static bool lsilogic_is_sas(LsilogicState *s)
{
    return true;
}

static uint16_t lsilogicGetHandle(LsilogicState *s)
{
    uint16_t u16Handle = s->next_handle++;
    return u16Handle;
}

static void lsilogic_soft_reset(LsilogicState *s);

static void lsilogic_update_interrupt(LsilogicState *s)
{
    uint32_t uIntSts;

    uIntSts = (s->intr_status & ~LSILOGIC_REG_HOST_INTR_STATUS_DOORBELL_STS);
    uIntSts &= ~(s->intr_mask & ~LSILOGIC_REG_HOST_INTR_MASK_IRQ_ROUTING);

    if (uIntSts) {
        if (msix_enabled(&s->dev)) {
            trace_lsilogic_msix_raise(0);
            msix_notify(&s->dev, 0);
        } else {
            trace_lsilogic_irq_raise();
            qemu_irq_raise(s->dev.irq[0]);
        }
    } else if (!msix_enabled(&s->dev)) {
        trace_lsilogic_irq_lower();
        qemu_irq_lower(s->dev.irq[0]);
    }
}

static void lsilogic_finish_address_reply(LsilogicState *s,
        MptReplyUnion *reply, bool fForceReplyFifo)
{
    /*
     * If we are in a doorbell function we set the reply size now and
     * set the system doorbell status interrupt to notify the guest that
     * we are ready to send the reply.
     */
    if (s->doorbell && !fForceReplyFifo) {
        /* Set size of the reply in 16bit words.
           The size in the reply is in 32bit dwords. */
        s->reply_size = reply->Header.u8MessageLength * 2;
        s->next_reply_entry_read = 0;
        s->intr_status |= LSILOGIC_REG_HOST_INTR_STATUS_SYSTEM_DOORBELL;
        lsilogic_update_interrupt(s);
    } else {
        /* Grab a free reply message from the queue. */

        /* Check for a free reply frame and room on the post queue. */
        if ((s->reply_free_queue_next_address_read ==
                s->reply_free_queue_next_entry_free_write)) {
            s->IOC_fault_code = LSILOGIC_IOCSTATUS_INSUFFICIENT_RESOURCES;
            s->state = LSILOGICSTATE_FAULT;
            return;
        }
        uint32_t reply_frame_address_low =
                s->reply_free_queue[s->reply_free_queue_next_address_read];

        uint32_t next_addr = (s->reply_free_queue_next_address_read + 1) %
                s->reply_queue_entries;
        if (next_addr != s->reply_free_queue_next_entry_free_write) {
            s->reply_free_queue_next_address_read = next_addr;
        }

        uint64_t reply_message_pa = ((uint64_t)s->host_mfa_high_addr << 32) |
                reply_frame_address_low;
        int reply_copied = (s->reply_frame_size < sizeof(MptReplyUnion)) ?
                s->reply_frame_size : sizeof(MptReplyUnion);

        cpu_physical_memory_write((target_phys_addr_t)reply_message_pa,
                (uint8_t *)reply, reply_copied);

        /* Write low 32bits of reply frame into post reply queue. */

        /* We have a address reply. Set the 31th bit to indicate that. */
        s->reply_post_queue[s->reply_post_queue_next_entry_free_write++] =
                (1<<31) | (reply_frame_address_low >> 1);
        s->reply_post_queue_next_entry_free_write %= s->reply_queue_entries;

        if (fForceReplyFifo) {
            s->doorbell = false;
            s->intr_status |= LSILOGIC_REG_HOST_INTR_STATUS_SYSTEM_DOORBELL;
        }

        /* Set interrupt. */
        s->intr_status |= LSILOGIC_REG_HOST_INTR_STATUS_REPLY_INTR;
        lsilogic_update_interrupt(s);
    }
}

static void lsilogic_abort_command(LsilogicCmd *cmd)
{
    if (cmd->req) {
        scsi_req_cancel(cmd->req);
        cmd->req = NULL;
    }
}


static QEMUSGList *lsilogic_get_sg_list(SCSIRequest *req)
{
    LsilogicCmd *cmd = req->hba_private;

    if (cmd->sge_cnt == 0) {
        return NULL;
    } else {
        return &cmd->qsg;
    }
}

static void lsilogic_xfer_complete(SCSIRequest *req, uint32_t len)
{
    LsilogicCmd *cmd = req->hba_private;

    trace_lsilogic_io_complete(cmd->index, len);
    if (cmd->sge_cnt != 0) {
        scsi_req_continue(req);
        return;
    }
}

static void lsilogic_finish_context_reply(LsilogicState *s,
        uint32_t u32MessageContext)
{
    assert(!s->doorbell);

    /* Write message context ID into reply post queue. */
    s->reply_post_queue[s->reply_post_queue_next_entry_free_write++] =
        u32MessageContext;
    s->reply_post_queue_next_entry_free_write %= s->reply_queue_entries;

    s->intr_status |= LSILOGIC_REG_HOST_INTR_STATUS_REPLY_INTR;
    lsilogic_update_interrupt(s);
}

static void lsilogic_command_complete(SCSIRequest *req,
        uint32_t status, size_t resid)
{
    LsilogicCmd *cmd = req->hba_private;
    uint8_t sense_buf[SCSI_SENSE_BUF_SIZE];
    uint8_t sense_len;

    target_phys_addr_t sense_buffer_pa =
        cmd->request.SCSIIO.u32SenseBufferLowAddress |
            ((uint64_t)cmd->state->sense_buffer_high_addr << 32);

    trace_lsilogic_command_complete(cmd->index, status, resid);

    if (cmd->sge_cnt) {
        qemu_sglist_destroy(&cmd->qsg);
    }

    sense_len = scsi_req_get_sense(cmd->req, sense_buf,
        SCSI_SENSE_BUF_SIZE);
    req->status = status;
    trace_lsilogic_scsi_complete(cmd->index, req->status,
        cmd->iov_size, req->cmd.xfer);

    if (sense_len > 0) {
        cpu_physical_memory_write(sense_buffer_pa, sense_buf,
                MIN(cmd->request.SCSIIO.u8SenseBufferLength, sense_len));
    }

    if (req->status != GOOD) {
        /* The SCSI target encountered an error during processing.
                Post a reply. */
        memset(&cmd->reply, 0, sizeof(MptReplyUnion));
        cmd->reply.SCSIIOError.u8TargetID          =
                cmd->request.SCSIIO.u8TargetID;
        cmd->reply.SCSIIOError.u8Bus               =
                cmd->request.SCSIIO.u8Bus;
        cmd->reply.SCSIIOError.u8MessageLength     = 8;
        cmd->reply.SCSIIOError.u8Function          =
                cmd->request.SCSIIO.u8Function;
        cmd->reply.SCSIIOError.u8CDBLength         =
                cmd->request.SCSIIO.u8CDBLength;
        cmd->reply.SCSIIOError.u8SenseBufferLength =
                cmd->request.SCSIIO.u8SenseBufferLength;
        cmd->reply.SCSIIOError.u8MessageFlags      =
                cmd->request.SCSIIO.u8MessageFlags;
        cmd->reply.SCSIIOError.u32MessageContext   =
                cmd->request.SCSIIO.u32MessageContext;
        cmd->reply.SCSIIOError.u8SCSIStatus        = req->status;
        cmd->reply.SCSIIOError.u8SCSIState         =
                MPT_SCSI_IO_ERROR_SCSI_STATE_AUTOSENSE_VALID;
        cmd->reply.SCSIIOError.u16IOCStatus        = 0;
        cmd->reply.SCSIIOError.u32IOCLogInfo       = 0;
        cmd->reply.SCSIIOError.u32TransferCount    = 0;
        cmd->reply.SCSIIOError.u32SenseCount       = sense_len;
        cmd->reply.SCSIIOError.u32ResponseInfo     = 0;

        lsilogic_finish_address_reply(cmd->state, &cmd->reply, true);
    } else {
        lsilogic_finish_context_reply(cmd->state,
                cmd->request.SCSIIO.u32MessageContext);
    }

    scsi_req_unref(cmd->req);
    cmd->req = NULL;
    g_free(cmd);
}

static void lsilogic_command_cancel(SCSIRequest *req)
{
    LsilogicCmd *cmd = req->hba_private;

    if (cmd) {
        lsilogic_abort_command(cmd);
    } else {
        scsi_req_unref(req);
    }
}

static void lsilogic_map_sgl(LsilogicState *s, LsilogicCmd *cmd,
        target_phys_addr_t sgl_pa, uint32_t chain_offset)
{
    uint32_t iov_count = 0;
    bool do_mapping = false;
    uint32_t pass;

    for (pass = 0; pass < 2; pass++) {
        bool end_of_list = false;
        target_phys_addr_t next_sge_pa = sgl_pa;
        target_phys_addr_t seg_start_pa = sgl_pa;
        uint32_t next_chain_offset = chain_offset;

        if (do_mapping) {
            cmd->sge_cnt = iov_count;
            qemu_sglist_init(&cmd->qsg, iov_count, pci_dma_context(&s->dev));
        }
        while (end_of_list == false) {
            bool end_of_seg = false;

            while (end_of_seg == false) {
                MptSGEntryUnion sge;
                cpu_physical_memory_read(next_sge_pa, &sge,
                        sizeof(MptSGEntryUnion));
                assert(sge.Simple32.u2ElementType == MPTSGENTRYTYPE_SIMPLE);
                if (sge.Simple32.u24Length == 0 && sge.Simple32.fEndOfList &&
                                sge.Simple32.fEndOfBuffer) {
                    cmd->sge_cnt = 0;
                    return;
                }
                if (sge.Simple32.f64BitAddress) {
                    next_sge_pa += sizeof(MptSGEntrySimple64);
                } else {
                    next_sge_pa += sizeof(MptSGEntrySimple32);
                }
                if (do_mapping) {
                    dma_addr_t iov_pa = sge.Simple32.u32DataBufferAddressLow;
                    dma_addr_t iov_size = sge.Simple32.u24Length;

                    if (sge.Simple32.f64BitAddress) {
                        iov_pa |= ((uint64_t)sge.Simple64.
                                u32DataBufferAddressHigh) << 32;
                    }

                    qemu_sglist_add(&cmd->qsg, iov_pa, iov_size);
                }
                iov_count++;
                if (sge.Simple32.fEndOfList) {
                    end_of_seg = true;
                    end_of_list = true;
                } else if (sge.Simple32.fLastElement) {
                    end_of_seg = true;
                }
            }
            if (next_chain_offset) {
                MptSGEntryChain sgec;
                cpu_physical_memory_read(seg_start_pa + next_chain_offset,
                        &sgec, sizeof(MptSGEntryChain));
                assert(sgec.u2ElementType == MPTSGENTRYTYPE_CHAIN);
                next_sge_pa = sgec.u32SegmentAddressLow;
                if (sgec.f64BitAddress) {
                    next_sge_pa |=
                        ((uint64_t)sgec.u32SegmentAddressHigh) << 32;
                }
                seg_start_pa = next_sge_pa;
                next_chain_offset = sgec.u8NextChainOffset * sizeof(uint32_t);
            }
        }
        do_mapping = true;
    }
}

static int lsilogic_process_SCSIIO_Request(LsilogicState *s, LsilogicCmd *cmd)
{
    struct SCSIDevice *sdev = NULL;

    if (cmd->request.SCSIIO.u8TargetID < s->max_devices &&
                cmd->request.SCSIIO.u8Bus == 0) {
        sdev = scsi_device_find(&s->bus, 0, cmd->request.SCSIIO.u8TargetID,
                                cmd->request.SCSIIO.au8LUN[1]);
        cmd->iov_size = le32_to_cpu(cmd->request.SCSIIO.u32DataLength);
        trace_lsilogic_handle_scsi("SCSI IO", 0,
                cmd->request.SCSIIO.u8TargetID,
                cmd->request.SCSIIO.au8LUN[1], sdev, cmd->iov_size);
        if (sdev) {
            uint32_t chain_offset = cmd->request.SCSIIO.u8ChainOffset;
            int32_t len;
            bool is_write;

            if (chain_offset) {
                chain_offset = chain_offset * sizeof(uint32_t) -
                        sizeof(MptSCSIIORequest);
            }

            lsilogic_map_sgl(s, cmd, cmd->host_msg_frame_pa +
                        sizeof(MptSCSIIORequest), chain_offset);
            is_write = MPT_SCSIIO_REQUEST_CONTROL_TXDIR_GET(
                        cmd->request.SCSIIO.u32Control) ==
                                MPT_SCSIIO_REQUEST_CONTROL_TXDIR_WRITE ?
                        true : false;
            cmd->state = s;
            cmd->req = scsi_req_new(sdev, cmd->index++,
                            cmd->request.SCSIIO.au8LUN[1],
                                cmd->request.SCSIIO.au8CDB, cmd);
            len = scsi_req_enqueue(cmd->req);
            if (len < 0) {
                len = -len;
            }
            if (len > 0) {
                if (len > cmd->iov_size) {
                    if (is_write) {
                        trace_lsilogic_iov_write_overflow(cmd->index, len,
                                cmd->iov_size);
                    } else {
                        trace_lsilogic_iov_read_overflow(cmd->index, len,
                                cmd->iov_size);
                    }
                }
                if (len < cmd->iov_size) {
                    if (is_write) {
                        trace_lsilogic_iov_write_underflow(cmd->index, len,
                                cmd->iov_size);
                    } else {
                        trace_lsilogic_iov_read_underflow(cmd->index, len,
                                cmd->iov_size);
                    }
                    cmd->iov_size = len;
                }
                scsi_req_continue(cmd->req);
            }
            if (len > 0) {
                if (is_write) {
                    trace_lsilogic_scsi_write_start(cmd->index, len);
                } else {
                    trace_lsilogic_scsi_read_start(cmd->index, len);
                }
            } else {
                trace_lsilogic_scsi_nodata(cmd->index);
            }
            return 0;
        } else {
            cmd->reply.SCSIIOError.u16IOCStatus =
                MPT_SCSI_IO_ERROR_IOCSTATUS_DEVICE_NOT_THERE;
        }
    } else {
        if (cmd->request.SCSIIO.u8Bus != 0) {
            cmd->reply.SCSIIOError.u16IOCStatus =
                MPT_SCSI_IO_ERROR_IOCSTATUS_INVALID_BUS;
        } else {
            cmd->reply.SCSIIOError.u16IOCStatus =
                MPT_SCSI_IO_ERROR_IOCSTATUS_INVALID_TARGETID;
        }
    }
    cmd->reply.SCSIIOError.u8TargetID          =
        cmd->request.SCSIIO.u8TargetID;
    cmd->reply.SCSIIOError.u8Bus               =
        cmd->request.SCSIIO.u8Bus;
    cmd->reply.SCSIIOError.u8MessageLength     =
        sizeof(MptSCSIIOErrorReply) / 4;
    cmd->reply.SCSIIOError.u8Function          =
        cmd->request.SCSIIO.u8Function;
    cmd->reply.SCSIIOError.u8CDBLength         =
        cmd->request.SCSIIO.u8CDBLength;
    cmd->reply.SCSIIOError.u8SenseBufferLength =
        cmd->request.SCSIIO.u8SenseBufferLength;
    cmd->reply.SCSIIOError.u32MessageContext   =
        cmd->request.SCSIIO.u32MessageContext;
    cmd->reply.SCSIIOError.u8SCSIStatus        = GOOD;
    cmd->reply.SCSIIOError.u8SCSIState         =
        MPT_SCSI_IO_ERROR_SCSI_STATE_TERMINATED;
    cmd->reply.SCSIIOError.u32IOCLogInfo       = 0;
    cmd->reply.SCSIIOError.u32TransferCount    = 0;
    cmd->reply.SCSIIOError.u32SenseCount       = 0;
    cmd->reply.SCSIIOError.u32ResponseInfo     = 0;

    lsilogic_finish_address_reply(s, &cmd->reply, false);
    g_free(cmd);

    return 0;
}

static void lsilogic_process_message(LsilogicState *s, MptMessageHdr *msg,
        MptReplyUnion *reply);

static bool lsilogic_queue_consumer(LsilogicState *s)
{
    /* Only process request which arrived before we
        received the notification. */
    uint32_t uRequestQueueNextEntryWrite =
        s->request_queue_next_entry_free_write;

    /* Go through the messages now and process them. */
    while ((s->state == LSILOGICSTATE_OPERATIONAL)
           && (s->request_queue_next_address_read !=
                uRequestQueueNextEntryWrite)) {
        uint32_t u32RequestMessageFrameDesc =
                s->request_queue[s->request_queue_next_address_read];
        MptRequestUnion request;
        target_phys_addr_t host_msg_frame_pa;


        host_msg_frame_pa = ((uint64_t)s->host_mfa_high_addr) << 32 |
                (u32RequestMessageFrameDesc & ~0x03);

        /* Read the message header from the guest first. */
        cpu_physical_memory_read(host_msg_frame_pa, &request.Header,
                sizeof(MptMessageHdr));

        /* Determine the size of the request. */
        uint32_t cbRequest = 0;

        switch (request.Header.u8Function) {
        case MPT_MESSAGE_HDR_FUNCTION_SCSI_IO_REQUEST:
            cbRequest = sizeof(MptSCSIIORequest);
            break;
        case MPT_MESSAGE_HDR_FUNCTION_SCSI_TASK_MGMT:
            cbRequest = sizeof(MptSCSITaskManagementRequest);
            break;
        case MPT_MESSAGE_HDR_FUNCTION_IOC_INIT:
            cbRequest = sizeof(MptIOCInitRequest);
            break;
        case MPT_MESSAGE_HDR_FUNCTION_IOC_FACTS:
            cbRequest = sizeof(MptIOCFactsRequest);
            break;
        case MPT_MESSAGE_HDR_FUNCTION_CONFIG:
            cbRequest = sizeof(MptConfigurationRequest);
            break;
        case MPT_MESSAGE_HDR_FUNCTION_PORT_FACTS:
            cbRequest = sizeof(MptPortFactsRequest);
            break;
        case MPT_MESSAGE_HDR_FUNCTION_PORT_ENABLE:
            cbRequest = sizeof(MptPortEnableRequest);
            break;
        case MPT_MESSAGE_HDR_FUNCTION_EVENT_NOTIFICATION:
            cbRequest = sizeof(MptEventNotificationRequest);
            break;
        case MPT_MESSAGE_HDR_FUNCTION_FW_DOWNLOAD:
            cbRequest = sizeof(MptFWDownloadRequest);
            break;
        case MPT_MESSAGE_HDR_FUNCTION_FW_UPLOAD:
            cbRequest = sizeof(MptFWUploadRequest);
            break;
        case MPT_MESSAGE_HDR_FUNCTION_EVENT_ACK:
        default:
            if (s->state != LSILOGICSTATE_FAULT) {
                s->IOC_fault_code = LSILOGIC_IOCSTATUS_INVALID_FUNCTION;
                s->state = LSILOGICSTATE_FAULT;
            }
        }

        if (cbRequest != 0) {
            /* Handle SCSI I/O requests seperately. */
            if (request.Header.u8Function ==
                        MPT_MESSAGE_HDR_FUNCTION_SCSI_IO_REQUEST) {
                LsilogicCmd *cmd = g_malloc0(sizeof(LsilogicCmd));
                cpu_physical_memory_read(host_msg_frame_pa,
                        &cmd->request.Header, cbRequest);
                cmd->host_msg_frame_pa = host_msg_frame_pa;
                lsilogic_process_SCSIIO_Request(s, cmd);
            } else {
                MptReplyUnion Reply;
                cpu_physical_memory_read(host_msg_frame_pa, &request.Header,
                        cbRequest);
                lsilogic_process_message(s, &request.Header, &Reply);
            }

        }
        s->request_queue_next_address_read++;
        s->request_queue_next_address_read %= s->request_queue_entries;
    }

    return true;
}


static int lsilogic_hard_reset(LsilogicState *s);

static int lsilogic_config_unit_page(LsilogicState *pLsiLogic,
                            PMptConfigurationPagesSupported pPages,
                            uint8_t u8PageNumber,
                            PMptConfigurationPageHeader *ppPageHeader,
                            uint8_t **ppbPageData, size_t *pcbPage)
{
    int rc = 0;

    switch (u8PageNumber) {
    case 0:
        *ppPageHeader = &pPages->IOUnitPage0.u.fields.Header;
        *ppbPageData  =  pPages->IOUnitPage0.u.abPageData;
        *pcbPage      = sizeof(pPages->IOUnitPage0);
        break;
    case 1:
        *ppPageHeader = &pPages->IOUnitPage1.u.fields.Header;
        *ppbPageData  =  pPages->IOUnitPage1.u.abPageData;
        *pcbPage      = sizeof(pPages->IOUnitPage1);
        break;
    case 2:
        *ppPageHeader = &pPages->IOUnitPage2.u.fields.Header;
        *ppbPageData  =  pPages->IOUnitPage2.u.abPageData;
        *pcbPage      = sizeof(pPages->IOUnitPage2);
        break;
    case 3:
        *ppPageHeader = &pPages->IOUnitPage3.u.fields.Header;
        *ppbPageData  =  pPages->IOUnitPage3.u.abPageData;
        *pcbPage      = sizeof(pPages->IOUnitPage3);
        break;
    case 4:
        *ppPageHeader = &pPages->IOUnitPage4.u.fields.Header;
        *ppbPageData  =  pPages->IOUnitPage4.u.abPageData;
        *pcbPage      = sizeof(pPages->IOUnitPage4);
        break;
    default:
        rc = -1;
    }

    return rc;
}

static int lsilogic_config_ioc_page(LsilogicState *pLsiLogic,
                         PMptConfigurationPagesSupported pPages,
                         uint8_t u8PageNumber,
                         PMptConfigurationPageHeader *ppPageHeader,
                         uint8_t **ppbPageData, size_t *pcbPage)
{
    int rc = 0;

    switch (u8PageNumber) {
    case 0:
        *ppPageHeader = &pPages->IOCPage0.u.fields.Header;
        *ppbPageData  =  pPages->IOCPage0.u.abPageData;
        *pcbPage      = sizeof(pPages->IOCPage0);
        break;
    case 1:
        *ppPageHeader = &pPages->IOCPage1.u.fields.Header;
        *ppbPageData  =  pPages->IOCPage1.u.abPageData;
        *pcbPage      = sizeof(pPages->IOCPage1);
        break;
    case 2:
        *ppPageHeader = &pPages->IOCPage2.u.fields.Header;
        *ppbPageData  =  pPages->IOCPage2.u.abPageData;
        *pcbPage      = sizeof(pPages->IOCPage2);
        break;
    case 3:
        *ppPageHeader = &pPages->IOCPage3.u.fields.Header;
        *ppbPageData  =  pPages->IOCPage3.u.abPageData;
        *pcbPage      = sizeof(pPages->IOCPage3);
        break;
    case 4:
        *ppPageHeader = &pPages->IOCPage4.u.fields.Header;
        *ppbPageData  =  pPages->IOCPage4.u.abPageData;
        *pcbPage      = sizeof(pPages->IOCPage4);
        break;
    case 6:
        *ppPageHeader = &pPages->IOCPage6.u.fields.Header;
        *ppbPageData  =  pPages->IOCPage6.u.abPageData;
        *pcbPage      = sizeof(pPages->IOCPage6);
        break;
    default:
        rc = -1;
    }

    return rc;
}

static int lsilogic_config_manufacturing_page(LsilogicState *pLsiLogic,
                               PMptConfigurationPagesSupported pPages,
                               uint8_t u8PageNumber,
                               PMptConfigurationPageHeader *ppPageHeader,
                               uint8_t **ppbPageData, size_t *pcbPage)
{
    int rc = 0;

    switch (u8PageNumber) {
    case 0:
        *ppPageHeader = &pPages->ManufacturingPage0.u.fields.Header;
        *ppbPageData  =  pPages->ManufacturingPage0.u.abPageData;
        *pcbPage      = sizeof(pPages->ManufacturingPage0);
        break;
    case 1:
        *ppPageHeader = &pPages->ManufacturingPage1.u.fields.Header;
        *ppbPageData  =  pPages->ManufacturingPage1.u.abPageData;
        *pcbPage      = sizeof(pPages->ManufacturingPage1);
        break;
    case 2:
        *ppPageHeader = &pPages->ManufacturingPage2.u.fields.Header;
        *ppbPageData  =  pPages->ManufacturingPage2.u.abPageData;
        *pcbPage      = sizeof(pPages->ManufacturingPage2);
        break;
    case 3:
        *ppPageHeader = &pPages->ManufacturingPage3.u.fields.Header;
        *ppbPageData  =  pPages->ManufacturingPage3.u.abPageData;
        *pcbPage      = sizeof(pPages->ManufacturingPage3);
        break;
    case 4:
        *ppPageHeader = &pPages->ManufacturingPage4.u.fields.Header;
        *ppbPageData  =  pPages->ManufacturingPage4.u.abPageData;
        *pcbPage      = sizeof(pPages->ManufacturingPage4);
        break;
    case 5:
        *ppPageHeader = &pPages->ManufacturingPage5.u.fields.Header;
        *ppbPageData  =  pPages->ManufacturingPage5.u.abPageData;
        *pcbPage      = sizeof(pPages->ManufacturingPage5);
        break;
    case 6:
        *ppPageHeader = &pPages->ManufacturingPage6.u.fields.Header;
        *ppbPageData  =  pPages->ManufacturingPage6.u.abPageData;
        *pcbPage      = sizeof(pPages->ManufacturingPage6);
        break;
    case 7:
        if (pLsiLogic->ctrl_type == LSILOGICCTRLTYPE_SCSI_SAS) {
            *ppPageHeader = &pPages->u.SasPages.pManufacturingPage7->
                u.fields.Header;
            *ppbPageData  =  pPages->u.SasPages.pManufacturingPage7->
                u.abPageData;
            *pcbPage      = pPages->u.SasPages.cbManufacturingPage7;
        } else {
            rc = -1;
        }
        break;
    case 8:
        *ppPageHeader = &pPages->ManufacturingPage8.u.fields.Header;
        *ppbPageData  =  pPages->ManufacturingPage8.u.abPageData;
        *pcbPage      = sizeof(pPages->ManufacturingPage8);
        break;
    case 9:
        *ppPageHeader = &pPages->ManufacturingPage9.u.fields.Header;
        *ppbPageData  =  pPages->ManufacturingPage9.u.abPageData;
        *pcbPage      = sizeof(pPages->ManufacturingPage9);
        break;
    case 10:
        *ppPageHeader = &pPages->ManufacturingPage10.u.fields.Header;
        *ppbPageData  =  pPages->ManufacturingPage10.u.abPageData;
        *pcbPage      = sizeof(pPages->ManufacturingPage10);
        break;
    default:
        rc = -1;
    }

    return rc;
}

static int lsilogic_config_bios_page(LsilogicState *pLsiLogic,
                              PMptConfigurationPagesSupported pPages,
                              uint8_t u8PageNumber,
                              PMptConfigurationPageHeader *ppPageHeader,
                              uint8_t **ppbPageData, size_t *pcbPage)
{
    int rc = 0;

    switch (u8PageNumber) {
    case 1:
        *ppPageHeader = &pPages->BIOSPage1.u.fields.Header;
        *ppbPageData  =  pPages->BIOSPage1.u.abPageData;
        *pcbPage      = sizeof(pPages->BIOSPage1);
        break;
    case 2:
        *ppPageHeader = &pPages->BIOSPage2.u.fields.Header;
        *ppbPageData  =  pPages->BIOSPage2.u.abPageData;
        *pcbPage      = sizeof(pPages->BIOSPage2);
        break;
    case 4:
        *ppPageHeader = &pPages->BIOSPage4.u.fields.Header;
        *ppbPageData  =  pPages->BIOSPage4.u.abPageData;
        *pcbPage      = sizeof(pPages->BIOSPage4);
        break;
    default:
        rc = -1;
    }

    return rc;
}

static int lsilogic_config_scsi_spi_port_page(LsilogicState *pLsiLogic,
                             PMptConfigurationPagesSupported pPages,
                             uint8_t u8Port,
                             uint8_t u8PageNumber,
                             PMptConfigurationPageHeader *ppPageHeader,
                             uint8_t **ppbPageData, size_t *pcbPage)
{
    int rc = 0;

    if (u8Port >= RT_ELEMENTS(pPages->u.SpiPages.aPortPages)) {
        return -1;
    }

    switch (u8PageNumber) {
    case 0:
        *ppPageHeader = &pPages->u.SpiPages.aPortPages[u8Port].
            SCSISPIPortPage0.u.fields.Header;
        *ppbPageData  =  pPages->u.SpiPages.aPortPages[u8Port].
            SCSISPIPortPage0.u.abPageData;
        *pcbPage      = sizeof(pPages->u.SpiPages.aPortPages[u8Port].
            SCSISPIPortPage0);
        break;
    case 1:
        *ppPageHeader = &pPages->u.SpiPages.aPortPages[u8Port].
            SCSISPIPortPage1.u.fields.Header;
        *ppbPageData  =  pPages->u.SpiPages.aPortPages[u8Port].
            SCSISPIPortPage1.u.abPageData;
        *pcbPage      = sizeof(pPages->u.SpiPages.aPortPages[u8Port].
            SCSISPIPortPage1);
        break;
    case 2:
        *ppPageHeader = &pPages->u.SpiPages.aPortPages[u8Port].
            SCSISPIPortPage2.u.fields.Header;
        *ppbPageData  =  pPages->u.SpiPages.aPortPages[u8Port].
            SCSISPIPortPage2.u.abPageData;
        *pcbPage      = sizeof(pPages->u.SpiPages.aPortPages[u8Port]
            .SCSISPIPortPage2);
        break;
    default:
        rc = -1;
    }

    return rc;
}

static int lsilogic_config_scsi_spi_device_page(LsilogicState *pLsiLogic,
                           PMptConfigurationPagesSupported pPages,
                           uint8_t u8Bus,
                           uint8_t u8TargetID, uint8_t u8PageNumber,
                           PMptConfigurationPageHeader *ppPageHeader,
                           uint8_t **ppbPageData, size_t *pcbPage)
{
    int rc = 0;

    if (u8Bus >= RT_ELEMENTS(pPages->u.SpiPages.aBuses)) {
        return -1;
    }

    if (u8TargetID >=
        RT_ELEMENTS(pPages->u.SpiPages.aBuses[u8Bus].aDevicePages)) {
        return -1;
    }

    switch (u8PageNumber) {
    case 0:
        *ppPageHeader = &pPages->u.SpiPages.aBuses[u8Bus].
                aDevicePages[u8TargetID].SCSISPIDevicePage0.u.fields.Header;
        *ppbPageData  =  pPages->u.SpiPages.aBuses[u8Bus].
                aDevicePages[u8TargetID].SCSISPIDevicePage0.u.abPageData;
        *pcbPage      = sizeof(pPages->u.SpiPages.aBuses[u8Bus].
                aDevicePages[u8TargetID].SCSISPIDevicePage0);
        break;
    case 1:
        *ppPageHeader = &pPages->u.SpiPages.aBuses[u8Bus].
                aDevicePages[u8TargetID].SCSISPIDevicePage1.u.fields.Header;
        *ppbPageData  =  pPages->u.SpiPages.aBuses[u8Bus].
                aDevicePages[u8TargetID].SCSISPIDevicePage1.u.abPageData;
        *pcbPage      = sizeof(pPages->u.SpiPages.aBuses[u8Bus].
                aDevicePages[u8TargetID].SCSISPIDevicePage1);
        break;
    case 2:
        *ppPageHeader = &pPages->u.SpiPages.aBuses[u8Bus].
                aDevicePages[u8TargetID].SCSISPIDevicePage2.u.fields.Header;
        *ppbPageData  =  pPages->u.SpiPages.aBuses[u8Bus].
                aDevicePages[u8TargetID].SCSISPIDevicePage2.u.abPageData;
        *pcbPage      = sizeof(pPages->u.SpiPages.aBuses[u8Bus].
                aDevicePages[u8TargetID].SCSISPIDevicePage2);
        break;
    case 3:
        *ppPageHeader = &pPages->u.SpiPages.aBuses[u8Bus].
                aDevicePages[u8TargetID].SCSISPIDevicePage3.u.fields.Header;
        *ppbPageData  =  pPages->u.SpiPages.aBuses[u8Bus].
                aDevicePages[u8TargetID].SCSISPIDevicePage3.u.abPageData;
        *pcbPage      = sizeof(pPages->u.SpiPages.aBuses[u8Bus].
                aDevicePages[u8TargetID].SCSISPIDevicePage3);
        break;
    default:
        rc = -1;
    }

    return rc;
}

static int lsilogic_config_sas_unit(LsilogicState *pLsiLogic,
                       PMptConfigurationPagesSupported pPages,
                       uint8_t u8PageNumber,
                       PMptExtendedConfigurationPageHeader *ppPageHeader,
                       uint8_t **ppbPageData, size_t *pcbPage)
{
    int rc = 0;

    switch (u8PageNumber) {
    case 0:
        *ppPageHeader = &pPages->u.SasPages.pSASIOUnitPage0->u.fields.
                ExtHeader;
        *ppbPageData  = pPages->u.SasPages.pSASIOUnitPage0->u.abPageData;
        *pcbPage      = pPages->u.SasPages.cbSASIOUnitPage0;
        break;
    case 1:
        *ppPageHeader = &pPages->u.SasPages.pSASIOUnitPage1->u.fields.
                ExtHeader;
        *ppbPageData  = pPages->u.SasPages.pSASIOUnitPage1->u.abPageData;
        *pcbPage      = pPages->u.SasPages.cbSASIOUnitPage1;
        break;
    case 2:
        *ppPageHeader = &pPages->u.SasPages.SASIOUnitPage2.u.fields.ExtHeader;
        *ppbPageData  =  pPages->u.SasPages.SASIOUnitPage2.u.abPageData;
        *pcbPage      = sizeof(pPages->u.SasPages.SASIOUnitPage2);
        break;
    case 3:
        *ppPageHeader = &pPages->u.SasPages.SASIOUnitPage3.u.fields.ExtHeader;
        *ppbPageData  =  pPages->u.SasPages.SASIOUnitPage3.u.abPageData;
        *pcbPage      = sizeof(pPages->u.SasPages.SASIOUnitPage3);
        break;
    default:
        rc = -1;
    }

    return rc;
}

static int lsilogic_config_sas_phy(LsilogicState *pLsiLogic,
                PMptConfigurationPagesSupported pPages,
                uint8_t u8PageNumber,
                MptConfigurationPageAddress PageAddress,
                PMptExtendedConfigurationPageHeader *ppPageHeader,
                uint8_t **ppbPageData, size_t *pcbPage)
{
    int rc = 0;
    uint8_t uAddressForm =
        MPT_CONFIGURATION_PAGE_ADDRESS_GET_SAS_FORM(PageAddress);
    PMptConfigurationPagesSas pPagesSas = &pPages->u.SasPages;
    PMptPHY pPHYPages = NULL;


    if (uAddressForm == 0) { /* PHY number */
        uint8_t u8PhyNumber = PageAddress.SASPHY.Form0.u8PhyNumber;

        if (u8PhyNumber >= pPagesSas->cPHYs) {
            return -1;
        }

        pPHYPages = &pPagesSas->paPHYs[u8PhyNumber];
    } else if (uAddressForm == 1) { /* Index form */
        uint16_t u16Index = PageAddress.SASPHY.Form1.u16Index;

        if (u16Index >= pPagesSas->cPHYs) {
            return -1;
        }

        pPHYPages = &pPagesSas->paPHYs[u16Index];
    } else {
        rc = -1; /* Correct? */
    }

    if (pPHYPages) {
        switch (u8PageNumber) {
        case 0:
            *ppPageHeader = &pPHYPages->SASPHYPage0.u.fields.ExtHeader;
            *ppbPageData  = pPHYPages->SASPHYPage0.u.abPageData;
            *pcbPage      = sizeof(pPHYPages->SASPHYPage0);
            break;
        case 1:
            *ppPageHeader = &pPHYPages->SASPHYPage1.u.fields.ExtHeader;
            *ppbPageData  =  pPHYPages->SASPHYPage1.u.abPageData;
            *pcbPage      = sizeof(pPHYPages->SASPHYPage1);
            break;
        default:
            rc = -1;
        }
    } else {
        rc = -1;
    }

    return rc;
}

static int lsilogic_config_sas_device(LsilogicState *pLsiLogic,
                   PMptConfigurationPagesSupported pPages,
                   uint8_t u8PageNumber,
                   MptConfigurationPageAddress PageAddress,
                   PMptExtendedConfigurationPageHeader *ppPageHeader,
                   uint8_t **ppbPageData, size_t *pcbPage)
{
    int rc = 0;
    uint8_t uAddressForm =
        MPT_CONFIGURATION_PAGE_ADDRESS_GET_SAS_FORM(PageAddress);
    PMptConfigurationPagesSas pPagesSas = &pPages->u.SasPages;
    PMptSASDevice pSASDevice = NULL;

    if (uAddressForm == 0) {
        uint16_t u16Handle = PageAddress.SASDevice.Form0And2.u16Handle;

        pSASDevice = pPagesSas->pSASDeviceHead;

        /* Get the first device? */
        if (u16Handle != 0xffff) {
            /* No, search for the right one. */

            while (pSASDevice
                   && pSASDevice->SASDevicePage0.u.fields.u16DevHandle !=
                        u16Handle)
                pSASDevice = pSASDevice->pNext;

            if (pSASDevice) {
                pSASDevice = pSASDevice->pNext;
            }
        }
    } else if (uAddressForm == 1) {
        uint8_t u8TargetID = PageAddress.SASDevice.Form1.u8TargetID;
        uint8_t u8Bus      = PageAddress.SASDevice.Form1.u8Bus;

        pSASDevice = pPagesSas->pSASDeviceHead;

        while (pSASDevice
               && (pSASDevice->SASDevicePage0.u.fields.u8TargetID !=
                        u8TargetID
                   || pSASDevice->SASDevicePage0.u.fields.u8Bus != u8Bus))
            pSASDevice = pSASDevice->pNext;
    } else if (uAddressForm == 2) {
        uint16_t u16Handle = PageAddress.SASDevice.Form0And2.u16Handle;

        pSASDevice = pPagesSas->pSASDeviceHead;

        while (pSASDevice
               && pSASDevice->SASDevicePage0.u.fields.u16DevHandle !=
                u16Handle) {
            pSASDevice = pSASDevice->pNext;
        }
    }

    if (pSASDevice) {
        switch (u8PageNumber) {
        case 0:
            *ppPageHeader = &pSASDevice->SASDevicePage0.u.fields.ExtHeader;
            *ppbPageData  =  pSASDevice->SASDevicePage0.u.abPageData;
            *pcbPage      = sizeof(pSASDevice->SASDevicePage0);
            break;
        case 1:
            *ppPageHeader = &pSASDevice->SASDevicePage1.u.fields.ExtHeader;
            *ppbPageData  =  pSASDevice->SASDevicePage1.u.abPageData;
            *pcbPage      = sizeof(pSASDevice->SASDevicePage1);
            break;
        case 2:
            *ppPageHeader = &pSASDevice->SASDevicePage2.u.fields.ExtHeader;
            *ppbPageData  =  pSASDevice->SASDevicePage2.u.abPageData;
            *pcbPage      = sizeof(pSASDevice->SASDevicePage2);
            break;
        default:
            rc = -1;
        }
    } else {
        rc = -1;
    }

    return rc;
}

static int lsilogic_config_page_get_extended(LsilogicState *pLsiLogic,
                PMptConfigurationRequest pConfigurationReq,
                PMptExtendedConfigurationPageHeader *ppPageHeader,
                uint8_t **ppbPageData, size_t *pcbPage)
{
    int rc = 0;

    switch (pConfigurationReq->u8ExtPageType) {
    case MPT_CONFIGURATION_PAGE_TYPE_EXTENDED_SASIOUNIT:
    {
        rc = lsilogic_config_sas_unit(pLsiLogic,
                                     pLsiLogic->config_pages,
                                     pConfigurationReq->u8PageNumber,
                                     ppPageHeader, ppbPageData, pcbPage);
        break;
    }
    case MPT_CONFIGURATION_PAGE_TYPE_EXTENDED_SASPHYS:
    {
        rc = lsilogic_config_sas_phy(pLsiLogic,
                                      pLsiLogic->config_pages,
                                      pConfigurationReq->u8PageNumber,
                                      pConfigurationReq->PageAddress,
                                      ppPageHeader, ppbPageData, pcbPage);
        break;
    }
    case MPT_CONFIGURATION_PAGE_TYPE_EXTENDED_SASDEVICE:
    {
        rc = lsilogic_config_sas_device(pLsiLogic,
                                     pLsiLogic->config_pages,
                                     pConfigurationReq->u8PageNumber,
                                     pConfigurationReq->PageAddress,
                                     ppPageHeader, ppbPageData, pcbPage);
        break;
    }
    case MPT_CONFIGURATION_PAGE_TYPE_EXTENDED_SASEXPANDER:
        /* No expanders supported */
    case MPT_CONFIGURATION_PAGE_TYPE_EXTENDED_ENCLOSURE:
        /* No enclosures supported */
    default:
        rc = -1;
    }

    return rc;
}


static int lsilogic_process_config_req(LsilogicState *s,
        MptConfigurationRequest *config_req, MptConfigurationReply *reply)
{
    int rc = 0;
    uint8_t                            *pbPageData     = NULL;
    PMptConfigurationPageHeader         pPageHeader    = NULL;
    PMptExtendedConfigurationPageHeader pExtPageHeader = NULL;
    size_t                              cbPage = 0;


    /* Copy common bits from the request into the reply. */
    reply->u8MessageLength   = 6; /* 6 32bit D-Words. */
    reply->u8Action          = config_req->u8Action;
    reply->u8Function        = config_req->u8Function;
    reply->u32MessageContext = config_req->u32MessageContext;

    switch (MPT_CONFIGURATION_PAGE_TYPE_GET(config_req->u8PageType)) {
    case MPT_CONFIGURATION_PAGE_TYPE_IO_UNIT:
    {
        rc = lsilogic_config_unit_page(s, s->config_pages,
                  config_req->u8PageNumber,
                  &pPageHeader, &pbPageData, &cbPage);
        break;
    }
    case MPT_CONFIGURATION_PAGE_TYPE_IOC:
    {
        /* Get the page data. */
        rc = lsilogic_config_ioc_page(s, s->config_pages,
                  config_req->u8PageNumber,
                  &pPageHeader, &pbPageData, &cbPage);
        break;
    }
    case MPT_CONFIGURATION_PAGE_TYPE_MANUFACTURING:
    {
        rc = lsilogic_config_manufacturing_page(s, s->config_pages,
                 config_req->u8PageNumber,
                 &pPageHeader, &pbPageData, &cbPage);
        break;
    }
    case MPT_CONFIGURATION_PAGE_TYPE_SCSI_SPI_PORT:
    {
        rc = lsilogic_config_scsi_spi_port_page(s, s->config_pages,
                 config_req->PageAddress.MPIPortNumber.u8PortNumber,
                 config_req->u8PageNumber,
                 &pPageHeader, &pbPageData, &cbPage);
        break;
    }
    case MPT_CONFIGURATION_PAGE_TYPE_SCSI_SPI_DEVICE:
    {
        rc = lsilogic_config_scsi_spi_device_page(s, s->config_pages,
                 config_req->PageAddress.BusAndTargetId.u8Bus,
                 config_req->PageAddress.BusAndTargetId.u8TargetID,
                 config_req->u8PageNumber,
                 &pPageHeader, &pbPageData, &cbPage);
        break;
    }
    case MPT_CONFIGURATION_PAGE_TYPE_BIOS:
    {
        rc = lsilogic_config_bios_page(s, s->config_pages,
                            config_req->u8PageNumber,
                            &pPageHeader, &pbPageData, &cbPage);
        break;
    }
    case MPT_CONFIGURATION_PAGE_TYPE_EXTENDED:
    {
        rc = lsilogic_config_page_get_extended(s, config_req,
            &pExtPageHeader, &pbPageData, &cbPage);
        break;
    }
    default:
        rc = -1;
    }

    if (rc == -1) {
        reply->u8PageType    = config_req->u8PageType;
        reply->u8PageNumber  = config_req->u8PageNumber;
        reply->u8PageLength  = config_req->u8PageLength;
        reply->u8PageVersion = config_req->u8PageVersion;
        reply->u16IOCStatus  = MPT_IOCSTATUS_CONFIG_INVALID_PAGE;
        return 0;
    }

    if (MPT_CONFIGURATION_PAGE_TYPE_GET(config_req->u8PageType) ==
                MPT_CONFIGURATION_PAGE_TYPE_EXTENDED) {
        reply->u8PageType       = pExtPageHeader->u8PageType;
        reply->u8PageNumber     = pExtPageHeader->u8PageNumber;
        reply->u8PageVersion    = pExtPageHeader->u8PageVersion;
        reply->u8ExtPageType    = pExtPageHeader->u8ExtPageType;
        reply->u16ExtPageLength = pExtPageHeader->u16ExtPageLength;
    } else {
        reply->u8PageType    = pPageHeader->u8PageType;
        reply->u8PageNumber  = pPageHeader->u8PageNumber;
        reply->u8PageLength  = pPageHeader->u8PageLength;
        reply->u8PageVersion = pPageHeader->u8PageVersion;
    }

    /*
     * Don't use the scatter gather handling code as the configuration
     * request always have only one simple element.
     */
    switch (config_req->u8Action) {
    case MPT_CONFIGURATION_REQUEST_ACTION_DEFAULT:
        /* Nothing to do. We are always using the defaults. */
    case MPT_CONFIGURATION_REQUEST_ACTION_HEADER:
    {
        /* Already copied above nothing to do. */
        break;
    }
    case MPT_CONFIGURATION_REQUEST_ACTION_READ_NVRAM:
    case MPT_CONFIGURATION_REQUEST_ACTION_READ_CURRENT:
    case MPT_CONFIGURATION_REQUEST_ACTION_READ_DEFAULT:
    {
        uint32_t cbBuffer = config_req->SimpleSGElement.u24Length;
        if (cbBuffer != 0) {
            uint64_t page_buffer_pa = config_req->SimpleSGElement.
                u32DataBufferAddressLow;
            if (config_req->SimpleSGElement.f64BitAddress) {
                page_buffer_pa |= (uint64_t)config_req->SimpleSGElement.
                        u32DataBufferAddressHigh << 32;
            }

            cpu_physical_memory_write(page_buffer_pa, pbPageData, MIN(cbBuffer,
                cbPage));
        }
        break;
    }
    case MPT_CONFIGURATION_REQUEST_ACTION_WRITE_CURRENT:
    case MPT_CONFIGURATION_REQUEST_ACTION_WRITE_NVRAM:
    {
        uint32_t cbBuffer = config_req->SimpleSGElement.u24Length;
        if (cbBuffer != 0) {
            uint64_t page_buffer_pa = config_req->SimpleSGElement.
                u32DataBufferAddressLow;
            if (config_req->SimpleSGElement.f64BitAddress) {
                page_buffer_pa |= (uint64_t)config_req->SimpleSGElement.
                        u32DataBufferAddressHigh << 32;
            }
            cpu_physical_memory_read(page_buffer_pa, pbPageData, MIN(cbBuffer,
                cbPage));
        }
        break;
    }
    default:
        break;
    }

    return 0;
}

static const char *lsilogic_msg_desc[] = {
        "SCSI_IO_REQUEST",
        "SCSI_TASK_MGMT",
        "IOC_INIT",
        "IOC_FACTS",
        "CONFIG",
        "PORT_FACTS",
        "PORT_ENABLE",
        "EVENT_NOTIFICATION",
        "EVENT_ACK",
        "FW_DOWNLOAD",
        "TARGET_CMD_BUFFER_POST",
        "TARGET_ASSIST",
        "TARGET_STATUS_SEND",
        "TARGET_MODE_ABORT",
        "UNDEFINED",
        "UNDEFINED",
        "UNDEFINED",
        "UNDEFINED",
        "FW_UPLOAD"
};

static void lsilogic_process_message(LsilogicState *s, MptMessageHdr *msg,
        MptReplyUnion *reply)
{
    bool fForceReplyPostFifo = false;

    memset(reply, 0, sizeof(MptReplyUnion));

    trace_lsilogic_process_message(lsilogic_msg_desc[msg->u8Function]);
    switch (msg->u8Function) {
    case MPT_MESSAGE_HDR_FUNCTION_SCSI_TASK_MGMT:
    {
        PMptSCSITaskManagementRequest pTaskMgmtReq =
            (PMptSCSITaskManagementRequest)msg;

        reply->SCSITaskManagement.u8MessageLength     = 6;
        reply->SCSITaskManagement.u8TaskType          =
            pTaskMgmtReq->u8TaskType;
        reply->SCSITaskManagement.u32TerminationCount = 0;
        fForceReplyPostFifo = true;
        break;
    }

    case MPT_MESSAGE_HDR_FUNCTION_IOC_INIT:
    {
        /* This request sets the I/O contr to the operational state. */
        PMptIOCInitRequest pIOCInitReq = (PMptIOCInitRequest)msg;

        /* Update configuration values. */
        s->who_init             = (LSILOGICWHOINIT)pIOCInitReq->u8WhoInit;
        s->reply_frame_size     = pIOCInitReq->u16ReplyFrameSize;
        s->max_buses            = pIOCInitReq->u8MaxBuses;
        s->max_devices          = pIOCInitReq->u8MaxDevices;
        s->host_mfa_high_addr   = pIOCInitReq->u32HostMfaHighAddr;
        s->sense_buffer_high_addr = pIOCInitReq->u32SenseBufferHighAddr;

        if (s->state == LSILOGICSTATE_READY) {
            s->state = LSILOGICSTATE_OPERATIONAL;
        }

        /* Return reply. */
        reply->IOCInit.u8MessageLength = 5;
        reply->IOCInit.u8WhoInit       = s->who_init;
        reply->IOCInit.u8MaxDevices    = s->max_devices;
        reply->IOCInit.u8MaxBuses      = s->max_buses;
        break;
    }
    case MPT_MESSAGE_HDR_FUNCTION_IOC_FACTS:
    {
        reply->IOCFacts.u8MessageLength      = 15; /* 15 32bit dwords. */

        if (s->ctrl_type == LSILOGICCTRLTYPE_SCSI_SPI) {
            /* Version from the specification. */
            reply->IOCFacts.u16MessageVersion    = 0x0102;
         } else if (s->ctrl_type == LSILOGICCTRLTYPE_SCSI_SAS) {
            /* Version from the specification. */
            reply->IOCFacts.u16MessageVersion    = 0x0105;
         }

        reply->IOCFacts.u8NumberOfPorts      = s->ports;
        /* PCI function number. */
        reply->IOCFacts.u8IOCNumber          = 0;
        reply->IOCFacts.u16IOCExceptions     = 0;
        reply->IOCFacts.u8MaxChainDepth = LSILOGICSCSI_MAXIMUM_CHAIN_DEPTH;
        reply->IOCFacts.u8WhoInit            = s->who_init;
        /* Block size in 32bit dwords. This is the largest request
           we can get (SCSI I/O). */
        reply->IOCFacts.u8BlockSize          = 12;
        /* Bit 0 is set if the guest must upload the FW prior to using
            the controller. Obviously not needed here. */
        reply->IOCFacts.u8Flags              = 0;
        /* One entry is always free. */
        reply->IOCFacts.u16ReplyQueueDepth   = s->reply_queue_entries - 1;
        reply->IOCFacts.u16RequestFrameSize  = 128;
        /* Our own product ID :) */
        reply->IOCFacts.u16ProductID         = 0x2704;
        reply->IOCFacts.u32CurrentHostMFAHighAddr = s->host_mfa_high_addr;
        /* One entry is always free. */
        reply->IOCFacts.u16GlobalCredits = s->request_queue_entries - 1;

            /* Event notifications not enabled. */
        reply->IOCFacts.u8EventState         = 0;
        reply->IOCFacts.u32CurrentSenseBufferHighAddr =
            s->sense_buffer_high_addr;
        reply->IOCFacts.u16CurReplyFrameSize = s->reply_frame_size;
        reply->IOCFacts.u8MaxDevices         = s->max_devices;
        reply->IOCFacts.u8MaxBuses           = s->max_buses;
        reply->IOCFacts.u32FwImageSize       = 0;
        reply->IOCFacts.u32FWVersion         = 0x1329200;
        break;
    }
    case MPT_MESSAGE_HDR_FUNCTION_PORT_FACTS:
    {
        PMptPortFactsRequest pPortFactsReq = (PMptPortFactsRequest)msg;

        reply->PortFacts.u8MessageLength = 10;
        reply->PortFacts.u8PortNumber    = pPortFactsReq->u8PortNumber;

        if (s->ctrl_type == LSILOGICCTRLTYPE_SCSI_SPI) {
            /* This controller only supports one bus with bus number 0. */
            if (pPortFactsReq->u8PortNumber >= s->ports) {
                reply->PortFacts.u8PortType = 0; /* Not existant. */
            } else {
                reply->PortFacts.u8PortType = 0x01; /* SCSI Port. */
                reply->PortFacts.u16MaxDevices          =
                    LSILOGICSCSI_PCI_SPI_DEVICES_PER_BUS_MAX;
                /* SCSI initiator and LUN supported. */
                reply->PortFacts.u16ProtocolFlags = (1 << 3) | (1 << 0);
                reply->PortFacts.u16PortSCSIID          = 7; /* Default */
                reply->PortFacts.u16MaxPersistentIDs    = 0;
                /* Only applies for target mode which we dont support. */
                reply->PortFacts.u16MaxPostedCmdBuffers = 0;
                /* Only for the LAN controller. */
                reply->PortFacts.u16MaxLANBuckets       = 0;
            }
        } else if (s->ctrl_type == LSILOGICCTRLTYPE_SCSI_SAS) {
            if (pPortFactsReq->u8PortNumber >= s->ports) {
                reply->PortFacts.u8PortType = 0; /* Not existant. */
            } else {
                reply->PortFacts.u8PortType = 0x30; /* SAS Port. */
                reply->PortFacts.u16MaxDevices          = s->ports;
                /* SCSI initiator and LUN supported. */
                reply->PortFacts.u16ProtocolFlags = (1 << 3) | (1 << 0);
                reply->PortFacts.u16PortSCSIID          = s->ports;
                reply->PortFacts.u16MaxPersistentIDs    = 0;
                /* Only applies for target mode which we dont support. */
                reply->PortFacts.u16MaxPostedCmdBuffers = 0;
                /* Only for the LAN controller. */
                reply->PortFacts.u16MaxLANBuckets       = 0;
            }
        }
        break;
    }
    case MPT_MESSAGE_HDR_FUNCTION_PORT_ENABLE:
    {
        /*
         * The port enable request notifies the IOC to make the port
         * available and perform appropriate discovery on the associated
         * link.
         */
        PMptPortEnableRequest pPortEnableReq = (PMptPortEnableRequest)msg;

        reply->PortEnable.u8MessageLength = 5;
        reply->PortEnable.u8PortNumber    = pPortEnableReq->u8PortNumber;
        break;
    }
    case MPT_MESSAGE_HDR_FUNCTION_EVENT_NOTIFICATION:
    {
        PMptEventNotificationRequest pEventNotificationReq =
            (PMptEventNotificationRequest)msg;

        if (pEventNotificationReq->u8Switch) {
            s->event_notification_enabled = true;
        } else {
            s->event_notification_enabled = false;
        }

        reply->EventNotification.u16EventDataLength = 1; /* 32bit Word. */
        reply->EventNotification.u8MessageLength    = 8;
        reply->EventNotification.u8MessageFlags     = (1 << 7);
        reply->EventNotification.u8AckRequired      = 0;
        reply->EventNotification.u32Event = MPT_EVENT_EVENT_CHANGE;
        reply->EventNotification.u32EventContext    = 0;
        reply->EventNotification.u32EventData       =
            s->event_notification_enabled ? 1 : 0;

        break;
    }
    case MPT_MESSAGE_HDR_FUNCTION_EVENT_ACK:
    {
        break;
    }
    case MPT_MESSAGE_HDR_FUNCTION_CONFIG:
    {
        PMptConfigurationRequest config_req =
            (PMptConfigurationRequest)msg;

        lsilogic_process_config_req(s, config_req, &reply->Configuration);
        break;
    }
    case MPT_MESSAGE_HDR_FUNCTION_FW_UPLOAD:
    {
        PMptFWUploadRequest pFWUploadReq = (PMptFWUploadRequest)msg;
        target_phys_addr_t iov_pa = pFWUploadReq->sge.u32DataBufferAddressLow;
        void *ptr;

        reply->FWUpload.u8ImageType        = pFWUploadReq->u8ImageType;
        reply->FWUpload.u8MessageLength    = 6;
        assert(pFWUploadReq->u8ImageType == MPI_FW_UPLOAD_ITYPE_BIOS_FLASH);
        assert(pFWUploadReq->sge.u2ElementType == MPTSGENTRYTYPE_SIMPLE);
        assert(pFWUploadReq->sge.f64BitAddress == 0);
        assert(pFWUploadReq->sge.fEndOfList);
        assert(pFWUploadReq->sge.fLastElement);
        reply->FWUpload.u32ActualImageSize = memory_region_size(&s->dev.rom);
        assert(reply->FWUpload.u32ActualImageSize >=
            pFWUploadReq->TCSge.ImageOffset + pFWUploadReq->sge.u24Length);
        ptr = memory_region_get_ram_ptr(&s->dev.rom);
        cpu_physical_memory_write(iov_pa, (uint8_t *)ptr +
            pFWUploadReq->TCSge.ImageOffset, pFWUploadReq->sge.u24Length);
        qemu_put_ram_ptr(ptr);
        reply->FWUpload.u32ActualImageSize = memory_region_size(&s->dev.rom);
        break;
    }
    case MPT_MESSAGE_HDR_FUNCTION_FW_DOWNLOAD:
    {

        reply->FWDownload.u8MessageLength    = 5;
        break;
    }
    case MPT_MESSAGE_HDR_FUNCTION_SCSI_IO_REQUEST:
        /* Should be handled already. */
    default:
        trace_lsilogic_unhandled_cmd(msg->u8Function, 0);
    }

    /* Copy common bits from request message frame to reply. */
    reply->Header.u8Function        = msg->u8Function;
    reply->Header.u32MessageContext = msg->u32MessageContext;

    lsilogic_finish_address_reply(s, reply, fForceReplyPostFifo);
}

static uint64_t lsilogic_mmio_read(void *opaque, target_phys_addr_t addr,
                                  unsigned size)
{
    LsilogicState *s = opaque;
    uint32_t retval = 0;

    switch (addr & ~3) {
    case LSILOGIC_REG_DOORBELL:
        retval = LSILOGIC_REG_DOORBELL_SET_STATE(s->state) |
                 LSILOGIC_REG_DOORBELL_SET_USED(s->doorbell) |
                 LSILOGIC_REG_DOORBELL_SET_WHOINIT(s->who_init);
        /*
         * If there is a doorbell function in progress we pass the
         * return value instead of the status code. We transfer 16bits
         * of the reply during one read.
         */
        if (s->doorbell) {
            retval |= s->reply_buffer.au16Reply[s->next_reply_entry_read++];
        } else {
            retval |= s->IOC_fault_code;
        }
        break;

    case LSILOGIC_REG_REPLY_QUEUE:
        if (s->reply_post_queue_next_entry_free_write !=
                s->reply_post_queue_next_address_read) {
            retval = s->reply_post_queue[
                s->reply_post_queue_next_address_read++];
            s->reply_post_queue_next_address_read %=
                s->reply_queue_entries;
        } else {
            /* The reply post queue is empty. Reset interrupt. */
            retval = 0xffffffff;
            s->intr_status &= ~LSILOGIC_REG_HOST_INTR_STATUS_REPLY_INTR;
            lsilogic_update_interrupt(s);
        }
        break;

    case LSILOGIC_REG_HOST_INTR_STATUS:
        retval = s->intr_status;
        break;

    case LSILOGIC_REG_HOST_INTR_MASK:
        retval = s->intr_mask;
        break;

    case LSILOGIC_REG_HOST_DIAGNOSTIC:
        if (s->diagnostic_enabled) {
            retval = LSILOGIC_REG_HOST_DIAGNOSTIC_DRWE;
        } else {
            retval = 0;
        }
        break;

    case LSILOGIC_REG_TEST_BASE_ADDRESS:
    case LSILOGIC_REG_DIAG_RW_DATA:
    case LSILOGIC_REG_DIAG_RW_ADDRESS:
    default:
        trace_lsilogic_mmio_invalid_readl(addr);
        break;
    }
    trace_lsilogic_mmio_readl(addr, retval);
    return retval;
}

static void lsilogic_mmio_write(void *opaque, target_phys_addr_t addr,
                               uint64_t val, unsigned size)
{
    static const uint8_t DiagnosticAccess[] = {0x04, 0x0b, 0x02, 0x07, 0x0d};

    LsilogicState *s = opaque;

    trace_lsilogic_mmio_writel(addr, val);
    switch (addr) {
    case LSILOGIC_REG_REPLY_QUEUE:
        s->reply_free_queue[s->reply_free_queue_next_entry_free_write++] = val;
        s->reply_free_queue_next_entry_free_write %= s->reply_queue_entries;
        break;

    case LSILOGIC_REG_REQUEST_QUEUE:
        s->request_queue[s->request_queue_next_entry_free_write++] = val;
        s->request_queue_next_entry_free_write %= s->request_queue_entries;
        lsilogic_queue_consumer(s);
        break;

    case LSILOGIC_REG_DOORBELL:
        if (!s->doorbell) {
            uint32_t uFunction = LSILOGIC_REG_DOORBELL_GET_FUNCTION(val);

            switch (uFunction) {
            case LSILOGIC_DOORBELL_FUNCTION_IOC_MSG_UNIT_RESET:
                lsilogic_soft_reset(s);
                break;
            case LSILOGIC_DOORBELL_FUNCTION_IO_UNIT_RESET:
                break;
            case LSILOGIC_DOORBELL_FUNCTION_HANDSHAKE:
            {
                s->drbl_message_size = LSILOGIC_REG_DOORBELL_GET_SIZE(val);
                s->drbl_message_index = 0;
                s->doorbell = true;
                /* Update the interrupt status to notify the guest that
                   a doorbell function was started. */
                s->intr_status |=
                    LSILOGIC_REG_HOST_INTR_STATUS_SYSTEM_DOORBELL;
                lsilogic_update_interrupt(s);
            }
                break;
            case LSILOGIC_DOORBELL_FUNCTION_REPLY_FRAME_REMOVAL:
            default:
                trace_lsilogic_mmio_invalid_writel(addr, val);
                break;
            }
        } else {
            /*
            * We are already performing a doorbell function.
            * Get the remaining parameters.
            */
            s->drbl_message[s->drbl_message_index++] = val;
            if (s->drbl_message_index == s->drbl_message_size) {
                lsilogic_process_message(s, (MptMessageHdr *)s->drbl_message,
                        &s->reply_buffer);
            }
        }
        break;

    case LSILOGIC_REG_HOST_INTR_STATUS:
        s->intr_status &= ~LSILOGIC_REG_HOST_INTR_STATUS_SYSTEM_DOORBELL;
        if (s->doorbell && s->drbl_message_size == s->drbl_message_index) {
            if (s->next_reply_entry_read == s->reply_size) {
                s->doorbell = false;
            }
            s->intr_status |= LSILOGIC_REG_HOST_INTR_STATUS_SYSTEM_DOORBELL;
        }
        lsilogic_update_interrupt(s);
        break;

    case LSILOGIC_REG_HOST_INTR_MASK:
        s->intr_mask = val & LSILOGIC_REG_HOST_INTR_MASK_W_MASK;
        lsilogic_update_interrupt(s);
        break;

    case LSILOGIC_REG_WRITE_SEQUENCE:
        /* Any value will cause a reset and disabling access. */
        if (s->diagnostic_enabled) {
            s->diagnostic_enabled = false;
            s->diagnostic_access_idx = 0;
        } else if ((val & 0xf) == DiagnosticAccess[s->diagnostic_access_idx]) {
            s->diagnostic_access_idx++;
            if (s->diagnostic_access_idx == sizeof(DiagnosticAccess)) {
                /*
                * Key sequence successfully written. Enable access to
                * diagnostic memory and register.
                */
                s->diagnostic_enabled = true;
            }
        } else { /* Wrong value written - reset to beginning. */
            s->diagnostic_access_idx = 0;
        }
        break;

        break;

    case LSILOGIC_REG_HOST_DIAGNOSTIC:
        if (val & LSILOGIC_REG_HOST_DIAGNOSTIC_RESET_ADAPTER) {
            lsilogic_hard_reset(s);
        }
        break;
    default:
        trace_lsilogic_mmio_invalid_writel(addr, val);
        break;
    }
}

static const MemoryRegionOps lsilogic_mmio_ops = {
    .read = lsilogic_mmio_read,
    .write = lsilogic_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 8,
        .max_access_size = 8,
    }
};

static uint64_t lsilogic_port_read(void *opaque, target_phys_addr_t addr,
                                  unsigned size)
{
    return lsilogic_mmio_read(opaque, addr & 0xff, size);
}

static void lsilogic_port_write(void *opaque, target_phys_addr_t addr,
                               uint64_t val, unsigned size)
{
    lsilogic_mmio_write(opaque, addr & 0xff, val, size);
}

static const MemoryRegionOps lsilogic_port_ops = {
    .read = lsilogic_port_read,
    .write = lsilogic_port_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    }
};

static uint64_t lsilogic_diag_read(void *opaque, target_phys_addr_t addr,
                                   unsigned size)
{
    trace_lsilogic_diag_readl(addr, 0);
    return 0;
}

static void lsilogic_diag_write(void *opaque, target_phys_addr_t addr,
                               uint64_t val, unsigned size)
{
    trace_lsilogic_diag_writel(addr, val);
}

static const MemoryRegionOps lsilogic_diag_ops = {
    .read = lsilogic_diag_read,
    .write = lsilogic_diag_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 8,
        .max_access_size = 8,
    }
};

static void lsilogic_soft_reset(LsilogicState *s)
{
    int i;
    trace_lsilogic_reset();
    s->state = LSILOGICSTATE_RESET;

    s->intr_status = 0;
    lsilogic_update_interrupt(s);

    /* Reset the queues. */
    s->reply_free_queue_next_entry_free_write = 0;
    s->reply_free_queue_next_address_read = 0;
    s->reply_post_queue_next_entry_free_write = 0;
    s->reply_post_queue_next_address_read = 0;
    s->request_queue_next_entry_free_write = 0;
    s->request_queue_next_address_read = 0;
    for (i = 0; i < LSILOGIC_MAX_FRAMES; i++) {
        LsilogicCmd *cmd = s->frames[i];
        if (cmd) {
            lsilogic_abort_command(cmd);
            cmd->flags = 0;
        }
    }
    s->next_frame = 0;
    s->state = LSILOGICSTATE_READY;
}

static void lsilogic_config_pages_free(LsilogicState *s)
{

    if (s->config_pages) {
        /* Destroy device list if we emulate a SAS controller. */
        if (s->ctrl_type == LSILOGICCTRLTYPE_SCSI_SAS) {
            PMptConfigurationPagesSas pSasPages = &s->config_pages->u.SasPages;
            PMptSASDevice pSASDeviceCurr = pSasPages->pSASDeviceHead;

            while (pSASDeviceCurr) {
                PMptSASDevice pFree = pSASDeviceCurr;

                pSASDeviceCurr = pSASDeviceCurr->pNext;
                g_free(pFree);
            }
            if (pSasPages->paPHYs) {
                g_free(pSasPages->paPHYs);
            }
            if (pSasPages->pManufacturingPage7) {
                g_free(pSasPages->pManufacturingPage7);
            }
            if (pSasPages->pSASIOUnitPage0) {
                g_free(pSasPages->pSASIOUnitPage0);
            }
            if (pSasPages->pSASIOUnitPage1) {
                g_free(pSasPages->pSASIOUnitPage1);
            }
        }

        g_free(s->config_pages);
    }
}

static void lsilogic_init_config_pages_spi(LsilogicState *s)
{
    unsigned i;
    PMptConfigurationPagesSpi pPages = &s->config_pages->u.SpiPages;

    /* Clear everything first. */
    memset(pPages, 0, sizeof(PMptConfigurationPagesSpi));

    for (i = 0; i < RT_ELEMENTS(pPages->aPortPages); i++) {
        /* SCSI-SPI port page 0. */
        pPages->aPortPages[i].SCSISPIPortPage0.u.fields.Header.u8PageType   =
                MPT_CONFIGURATION_PAGE_ATTRIBUTE_READONLY
                  | MPT_CONFIGURATION_PAGE_TYPE_SCSI_SPI_PORT;
        pPages->aPortPages[i].SCSISPIPortPage0.u.fields.Header.u8PageNumber =
                0;
        pPages->aPortPages[i].SCSISPIPortPage0.u.fields.Header.u8PageLength =
                sizeof(MptConfigurationPageSCSISPIPort0) / 4;
        pPages->aPortPages[i].SCSISPIPortPage0.u.fields.
                fInformationUnitTransfersCapable = true;
        pPages->aPortPages[i].SCSISPIPortPage0.u.fields.fDTCapable  = true;
        pPages->aPortPages[i].SCSISPIPortPage0.u.fields.fQASCapable = true;
        pPages->aPortPages[i].SCSISPIPortPage0.u.fields.
                u8MinimumSynchronousTransferPeriod =  0;
        pPages->aPortPages[i].SCSISPIPortPage0.u.fields.
                u8MaximumSynchronousOffset         = 0xff;
        pPages->aPortPages[i].SCSISPIPortPage0.u.fields.fWide = true;
        pPages->aPortPages[i].SCSISPIPortPage0.u.fields.fAIPCapable = true;
        pPages->aPortPages[i].SCSISPIPortPage0.u.fields.u2SignalingType =
                0x3; /* Single Ended. */

        /* SCSI-SPI port page 1. */
        pPages->aPortPages[i].SCSISPIPortPage1.u.fields.Header.u8PageType   =
                MPT_CONFIGURATION_PAGE_ATTRIBUTE_CHANGEABLE
                  | MPT_CONFIGURATION_PAGE_TYPE_SCSI_SPI_PORT;
        pPages->aPortPages[i].SCSISPIPortPage1.u.fields.Header.u8PageNumber = 1;
        pPages->aPortPages[i].SCSISPIPortPage1.u.fields.Header.u8PageLength =
                sizeof(MptConfigurationPageSCSISPIPort1) / 4;
        pPages->aPortPages[i].SCSISPIPortPage1.u.fields.u8SCSIID = 7;
        pPages->aPortPages[i].SCSISPIPortPage1.u.fields.
                u16PortResponseIDsBitmask = (1 << 7);
        pPages->aPortPages[i].SCSISPIPortPage1.u.fields.u32OnBusTimerValue = 0;

        /* SCSI-SPI port page 2. */
        pPages->aPortPages[i].SCSISPIPortPage2.u.fields.Header.u8PageType   =
                MPT_CONFIGURATION_PAGE_ATTRIBUTE_CHANGEABLE
                  | MPT_CONFIGURATION_PAGE_TYPE_SCSI_SPI_PORT;
        pPages->aPortPages[i].SCSISPIPortPage2.u.fields.
                Header.u8PageNumber = 2;
        pPages->aPortPages[i].SCSISPIPortPage2.u.fields.Header.
                u8PageLength = sizeof(MptConfigurationPageSCSISPIPort2) / 4;
        pPages->aPortPages[i].SCSISPIPortPage2.u.fields.
                u4HostSCSIID           = 7;
        pPages->aPortPages[i].SCSISPIPortPage2.u.fields.
                u2InitializeHBA        = 0x3;
        pPages->aPortPages[i].SCSISPIPortPage2.u.fields.
                fTerminationDisabled   = true;
        unsigned iDevice;

        for (iDevice = 0; iDevice < RT_ELEMENTS(pPages->aPortPages[i].
                SCSISPIPortPage2.u.fields.aDeviceSettings); iDevice++) {
            pPages->aPortPages[i].SCSISPIPortPage2.u.fields.
                aDeviceSettings[iDevice].fBootChoice   = true;
        }
        /* Everything else 0 for now. */
    }

    unsigned uBusCurr;
    for (uBusCurr = 0; uBusCurr < RT_ELEMENTS(pPages->aBuses); uBusCurr++) {
        unsigned uDeviceCurr;
        for (uDeviceCurr = 0; uDeviceCurr <
                RT_ELEMENTS(pPages->aBuses[uBusCurr].aDevicePages);
                        uDeviceCurr++) {
            /* SCSI-SPI device page 0. */
            pPages->aBuses[uBusCurr].aDevicePages[uDeviceCurr].
                SCSISPIDevicePage0.u.fields.Header.u8PageType   =
                        MPT_CONFIGURATION_PAGE_ATTRIBUTE_READONLY
                         | MPT_CONFIGURATION_PAGE_TYPE_SCSI_SPI_DEVICE;
            pPages->aBuses[uBusCurr].aDevicePages[uDeviceCurr].
                SCSISPIDevicePage0.u.fields.Header.u8PageNumber = 0;
            pPages->aBuses[uBusCurr].aDevicePages[uDeviceCurr].
                SCSISPIDevicePage0.u.fields.Header.u8PageLength =
                        sizeof(MptConfigurationPageSCSISPIDevice0) / 4;
            /* Everything else 0 for now. */

            /* SCSI-SPI device page 1. */
            pPages->aBuses[uBusCurr].aDevicePages[uDeviceCurr].
                SCSISPIDevicePage1.u.fields.Header.u8PageType   =
                        MPT_CONFIGURATION_PAGE_ATTRIBUTE_CHANGEABLE
                         | MPT_CONFIGURATION_PAGE_TYPE_SCSI_SPI_DEVICE;
            pPages->aBuses[uBusCurr].aDevicePages[uDeviceCurr].
                SCSISPIDevicePage1.u.fields.Header.u8PageNumber = 1;
            pPages->aBuses[uBusCurr].aDevicePages[uDeviceCurr].
                SCSISPIDevicePage1.u.fields.Header.u8PageLength =
                        sizeof(MptConfigurationPageSCSISPIDevice1) / 4;
            /* Everything else 0 for now. */

            /* SCSI-SPI device page 2. */
            pPages->aBuses[uBusCurr].aDevicePages[uDeviceCurr].
                SCSISPIDevicePage2.u.fields.Header.u8PageType   =
                        MPT_CONFIGURATION_PAGE_ATTRIBUTE_CHANGEABLE
                         | MPT_CONFIGURATION_PAGE_TYPE_SCSI_SPI_DEVICE;
            pPages->aBuses[uBusCurr].aDevicePages[uDeviceCurr].
                SCSISPIDevicePage2.u.fields.Header.u8PageNumber = 2;
            pPages->aBuses[uBusCurr].aDevicePages[uDeviceCurr].
                SCSISPIDevicePage2.u.fields.Header.u8PageLength =
                        sizeof(MptConfigurationPageSCSISPIDevice2) / 4;
            /* Everything else 0 for now. */

            pPages->aBuses[uBusCurr].aDevicePages[uDeviceCurr].
                SCSISPIDevicePage3.u.fields.Header.u8PageType   =
                        MPT_CONFIGURATION_PAGE_ATTRIBUTE_READONLY
                         | MPT_CONFIGURATION_PAGE_TYPE_SCSI_SPI_DEVICE;
            pPages->aBuses[uBusCurr].aDevicePages[uDeviceCurr].
                SCSISPIDevicePage3.u.fields.Header.u8PageNumber = 3;
            pPages->aBuses[uBusCurr].aDevicePages[uDeviceCurr].
                SCSISPIDevicePage3.u.fields.Header.u8PageLength =
                        sizeof(MptConfigurationPageSCSISPIDevice3) / 4;
            /* Everything else 0 for now. */
        }
    }
}

static void lsilogic_init_config_pages_sas(LsilogicState *s)
{
    PMptConfigurationPagesSas pPages = &s->config_pages->u.SasPages;

    /* Manufacturing Page 7 - Connector settings. */
    pPages->cbManufacturingPage7 =
        LSILOGICSCSI_MANUFACTURING7_GET_SIZE(s->ports);
    PMptConfigurationPageManufacturing7 pManufacturingPage7 =
        (PMptConfigurationPageManufacturing7)g_malloc0(
            pPages->cbManufacturingPage7);
    MPT_CONFIG_PAGE_HEADER_INIT_MANUFACTURING(pManufacturingPage7, 0, 7,
                      MPT_CONFIGURATION_PAGE_ATTRIBUTE_PERSISTENT_READONLY);
    /* Set size manually. */
    if (pPages->cbManufacturingPage7 / 4 > 255) {
        pManufacturingPage7->u.fields.Header.u8PageLength = 255;
    } else {
        pManufacturingPage7->u.fields.Header.u8PageLength =
                pPages->cbManufacturingPage7 / 4;
    }
    pManufacturingPage7->u.fields.u8NumPhys = s->ports;
    pPages->pManufacturingPage7 = pManufacturingPage7;

    /* SAS I/O unit page 0 - Port specific information. */
    pPages->cbSASIOUnitPage0 = LSILOGICSCSI_SASIOUNIT0_GET_SIZE(s->ports);
    PMptConfigurationPageSASIOUnit0 pSASPage0 =
        (PMptConfigurationPageSASIOUnit0)g_malloc0(pPages->cbSASIOUnitPage0);

    MPT_CONFIG_EXTENDED_PAGE_HEADER_INIT(pSASPage0, pPages->cbSASIOUnitPage0,
                             0, MPT_CONFIGURATION_PAGE_ATTRIBUTE_READONLY,
                             MPT_CONFIGURATION_PAGE_TYPE_EXTENDED_SASIOUNIT);
    pSASPage0->u.fields.u8NumPhys = s->ports;
    pPages->pSASIOUnitPage0 = pSASPage0;

    /* SAS I/O unit page 1 - Port specific settings. */
    pPages->cbSASIOUnitPage1 = LSILOGICSCSI_SASIOUNIT1_GET_SIZE(s->ports);
    PMptConfigurationPageSASIOUnit1 pSASPage1 =
        (PMptConfigurationPageSASIOUnit1)g_malloc0(pPages->cbSASIOUnitPage1);

    MPT_CONFIG_EXTENDED_PAGE_HEADER_INIT(pSASPage1, pPages->cbSASIOUnitPage1,
                             1, MPT_CONFIGURATION_PAGE_ATTRIBUTE_CHANGEABLE,
                             MPT_CONFIGURATION_PAGE_TYPE_EXTENDED_SASIOUNIT);
    pSASPage1->u.fields.u8NumPhys = pSASPage0->u.fields.u8NumPhys;
    pSASPage1->u.fields.u16ControlFlags = 0;
    pSASPage1->u.fields.u16AdditionalControlFlags = 0;
    pPages->pSASIOUnitPage1 = pSASPage1;

    /* SAS I/O unit page 2 - Port specific information. */
    pPages->SASIOUnitPage2.u.fields.ExtHeader.u8PageType       =
        MPT_CONFIGURATION_PAGE_ATTRIBUTE_READONLY
         | MPT_CONFIGURATION_PAGE_TYPE_EXTENDED;
    pPages->SASIOUnitPage2.u.fields.ExtHeader.u8PageNumber     = 2;
    pPages->SASIOUnitPage2.u.fields.ExtHeader.u8ExtPageType    =
        MPT_CONFIGURATION_PAGE_TYPE_EXTENDED_SASIOUNIT;
    pPages->SASIOUnitPage2.u.fields.ExtHeader.u16ExtPageLength =
        sizeof(MptConfigurationPageSASIOUnit2) / 4;

    /* SAS I/O unit page 3 - Port specific information. */
    pPages->SASIOUnitPage3.u.fields.ExtHeader.u8PageType       =
        MPT_CONFIGURATION_PAGE_ATTRIBUTE_READONLY
         | MPT_CONFIGURATION_PAGE_TYPE_EXTENDED;
    pPages->SASIOUnitPage3.u.fields.ExtHeader.u8PageNumber     = 3;
    pPages->SASIOUnitPage3.u.fields.ExtHeader.u8ExtPageType    =
        MPT_CONFIGURATION_PAGE_TYPE_EXTENDED_SASIOUNIT;
    pPages->SASIOUnitPage3.u.fields.ExtHeader.u16ExtPageLength =
        sizeof(MptConfigurationPageSASIOUnit3) / 4;

    pPages->cPHYs  = s->ports;
    pPages->paPHYs = (PMptPHY)g_malloc0(pPages->cPHYs * sizeof(MptPHY));

    /* Initialize the PHY configuration */
    unsigned i;
    for (i = 0; i < s->ports; i++) {
        PMptPHY pPHYPages = &pPages->paPHYs[i];
        uint16_t u16ControllerHandle = lsilogicGetHandle(s);

        pManufacturingPage7->u.fields.aPHY[i].u8Location =
                LSILOGICSCSI_MANUFACTURING7_LOCATION_AUTO;

        pSASPage0->u.fields.aPHY[i].u8Port      = i;
        pSASPage0->u.fields.aPHY[i].u8PortFlags = 0;
        pSASPage0->u.fields.aPHY[i].u8PhyFlags  = 0;
        pSASPage0->u.fields.aPHY[i].u8NegotiatedLinkRate =
                LSILOGICSCSI_SASIOUNIT0_NEGOTIATED_RATE_FAILED;
        pSASPage0->u.fields.aPHY[i].u32ControllerPhyDeviceInfo =
                LSILOGICSCSI_SASIOUNIT0_DEVICE_TYPE_SET(
                    LSILOGICSCSI_SASIOUNIT0_DEVICE_TYPE_NO);
        pSASPage0->u.fields.aPHY[i].u16ControllerDevHandle =
                u16ControllerHandle;
        pSASPage0->u.fields.aPHY[i].u16AttachedDevHandle = 0;
        pSASPage0->u.fields.aPHY[i].u32DiscoveryStatus = 0;

        pSASPage1->u.fields.aPHY[i].u8Port           = i;
        pSASPage1->u.fields.aPHY[i].u8PortFlags      = 0;
        pSASPage1->u.fields.aPHY[i].u8PhyFlags       = 0;
        pSASPage1->u.fields.aPHY[i].u8MaxMinLinkRate =
                LSILOGICSCSI_SASIOUNIT1_LINK_RATE_MIN_SET(
                    LSILOGICSCSI_SASIOUNIT1_LINK_RATE_15GB)
                   | LSILOGICSCSI_SASIOUNIT1_LINK_RATE_MAX_SET(
                    LSILOGICSCSI_SASIOUNIT1_LINK_RATE_30GB);
        pSASPage1->u.fields.aPHY[i].u32ControllerPhyDeviceInfo =
                LSILOGICSCSI_SASIOUNIT0_DEVICE_TYPE_SET(
                    LSILOGICSCSI_SASIOUNIT0_DEVICE_TYPE_NO);

        /* SAS PHY page 0. */
        pPHYPages->SASPHYPage0.u.fields.ExtHeader.u8PageType       =
                MPT_CONFIGURATION_PAGE_ATTRIBUTE_READONLY
                  | MPT_CONFIGURATION_PAGE_TYPE_EXTENDED;
        pPHYPages->SASPHYPage0.u.fields.ExtHeader.u8PageNumber     = 0;
        pPHYPages->SASPHYPage0.u.fields.ExtHeader.u8ExtPageType    =
                MPT_CONFIGURATION_PAGE_TYPE_EXTENDED_SASPHYS;
        pPHYPages->SASPHYPage0.u.fields.ExtHeader.u16ExtPageLength =
                sizeof(MptConfigurationPageSASPHY0) / 4;
        pPHYPages->SASPHYPage0.u.fields.u8AttachedPhyIdentifier    = i;
        pPHYPages->SASPHYPage0.u.fields.u32AttachedDeviceInfo      =
                LSILOGICSCSI_SASPHY0_DEV_INFO_DEVICE_TYPE_SET(
                    LSILOGICSCSI_SASPHY0_DEV_INFO_DEVICE_TYPE_NO);
        pPHYPages->SASPHYPage0.u.fields.u8ProgrammedLinkRate       =
                LSILOGICSCSI_SASIOUNIT1_LINK_RATE_MIN_SET(
                    LSILOGICSCSI_SASIOUNIT1_LINK_RATE_15GB)
                 | LSILOGICSCSI_SASIOUNIT1_LINK_RATE_MAX_SET(
                    LSILOGICSCSI_SASIOUNIT1_LINK_RATE_30GB);
        pPHYPages->SASPHYPage0.u.fields.u8HwLinkRate               =
                LSILOGICSCSI_SASIOUNIT1_LINK_RATE_MIN_SET(
                    LSILOGICSCSI_SASIOUNIT1_LINK_RATE_15GB)
                 | LSILOGICSCSI_SASIOUNIT1_LINK_RATE_MAX_SET(
                    LSILOGICSCSI_SASIOUNIT1_LINK_RATE_30GB);

        /* SAS PHY page 1. */
        pPHYPages->SASPHYPage1.u.fields.ExtHeader.u8PageType       =
                MPT_CONFIGURATION_PAGE_ATTRIBUTE_READONLY
                 | MPT_CONFIGURATION_PAGE_TYPE_EXTENDED;
        pPHYPages->SASPHYPage1.u.fields.ExtHeader.u8PageNumber     = 1;
        pPHYPages->SASPHYPage1.u.fields.ExtHeader.u8ExtPageType    =
                MPT_CONFIGURATION_PAGE_TYPE_EXTENDED_SASPHYS;
        pPHYPages->SASPHYPage1.u.fields.ExtHeader.u16ExtPageLength =
                sizeof(MptConfigurationPageSASPHY1) / 4;

        /* Settings for present devices. */
        if (scsi_device_find(&s->bus, 0, i, 0)) {
            uint16_t u16DeviceHandle = lsilogicGetHandle(s);
            SASADDRESS SASAddress;
            PMptSASDevice pSASDevice =
                (PMptSASDevice)g_malloc0(sizeof(MptSASDevice));

            memset(&SASAddress, 0, sizeof(SASADDRESS));
            SASAddress.u64Address = s->sas_addr;

            pSASPage0->u.fields.aPHY[i].u8NegotiatedLinkRate       =
                LSILOGICSCSI_SASIOUNIT0_NEGOTIATED_RATE_SET(
                    LSILOGICSCSI_SASIOUNIT0_NEGOTIATED_RATE_30GB);
            pSASPage0->u.fields.aPHY[i].u32ControllerPhyDeviceInfo =
                LSILOGICSCSI_SASIOUNIT0_DEVICE_TYPE_SET(
                    LSILOGICSCSI_SASIOUNIT0_DEVICE_TYPE_END)
                 | LSILOGICSCSI_SASIOUNIT0_DEVICE_SSP_TARGET;
            pSASPage0->u.fields.aPHY[i].u16AttachedDevHandle       =
                u16DeviceHandle;
            pSASPage1->u.fields.aPHY[i].u32ControllerPhyDeviceInfo =
                LSILOGICSCSI_SASIOUNIT0_DEVICE_TYPE_SET(
                    LSILOGICSCSI_SASIOUNIT0_DEVICE_TYPE_END)
                 | LSILOGICSCSI_SASIOUNIT0_DEVICE_SSP_TARGET;
            pSASPage0->u.fields.aPHY[i].u16ControllerDevHandle     =
                u16DeviceHandle;

            pPHYPages->SASPHYPage0.u.fields.u32AttachedDeviceInfo  =
                LSILOGICSCSI_SASPHY0_DEV_INFO_DEVICE_TYPE_SET(
                    LSILOGICSCSI_SASPHY0_DEV_INFO_DEVICE_TYPE_END);
            pPHYPages->SASPHYPage0.u.fields.SASAddress             =
                SASAddress;
            pPHYPages->SASPHYPage0.u.fields.u16OwnerDevHandle      =
                u16DeviceHandle;
            pPHYPages->SASPHYPage0.u.fields.u16AttachedDevHandle   =
                u16DeviceHandle;

            /* SAS device page 0. */
            pSASDevice->SASDevicePage0.u.fields.ExtHeader.u8PageType       =
                MPT_CONFIGURATION_PAGE_ATTRIBUTE_READONLY
                 | MPT_CONFIGURATION_PAGE_TYPE_EXTENDED;
            pSASDevice->SASDevicePage0.u.fields.ExtHeader.u8PageNumber     = 0;
            pSASDevice->SASDevicePage0.u.fields.ExtHeader.u8ExtPageType    =
                MPT_CONFIGURATION_PAGE_TYPE_EXTENDED_SASDEVICE;
            pSASDevice->SASDevicePage0.u.fields.ExtHeader.u16ExtPageLength =
                sizeof(MptConfigurationPageSASDevice0) / 4;
            pSASDevice->SASDevicePage0.u.fields.SASAddress                 =
                SASAddress;
            pSASDevice->SASDevicePage0.u.fields.u16ParentDevHandle         =
                u16ControllerHandle;
            pSASDevice->SASDevicePage0.u.fields.u8PhyNum                   = i;
            pSASDevice->SASDevicePage0.u.fields.u8AccessStatus =
                LSILOGICSCSI_SASDEVICE0_STATUS_NO_ERRORS;
            pSASDevice->SASDevicePage0.u.fields.u16DevHandle = u16DeviceHandle;
            pSASDevice->SASDevicePage0.u.fields.u8TargetID                 = i;
            pSASDevice->SASDevicePage0.u.fields.u8Bus                      = 0;
            pSASDevice->SASDevicePage0.u.fields.u32DeviceInfo              =
                LSILOGICSCSI_SASPHY0_DEV_INFO_DEVICE_TYPE_SET(
                    LSILOGICSCSI_SASPHY0_DEV_INFO_DEVICE_TYPE_END)
                     | LSILOGICSCSI_SASIOUNIT0_DEVICE_SSP_TARGET;
            pSASDevice->SASDevicePage0.u.fields.u16Flags                   =
             LSILOGICSCSI_SASDEVICE0_FLAGS_DEVICE_PRESENT
             | LSILOGICSCSI_SASDEVICE0_FLAGS_DEVICE_MAPPED_TO_BUS_AND_TARGET_ID
             | LSILOGICSCSI_SASDEVICE0_FLAGS_DEVICE_MAPPING_PERSISTENT;
            pSASDevice->SASDevicePage0.u.fields.u8PhysicalPort             = i;

            /* SAS device page 1. */
            pSASDevice->SASDevicePage1.u.fields.ExtHeader.u8PageType       =
                MPT_CONFIGURATION_PAGE_ATTRIBUTE_READONLY
                     | MPT_CONFIGURATION_PAGE_TYPE_EXTENDED;
            pSASDevice->SASDevicePage1.u.fields.ExtHeader.u8PageNumber     = 1;
            pSASDevice->SASDevicePage1.u.fields.ExtHeader.u8ExtPageType    =
                MPT_CONFIGURATION_PAGE_TYPE_EXTENDED_SASDEVICE;
            pSASDevice->SASDevicePage1.u.fields.ExtHeader.u16ExtPageLength =
                sizeof(MptConfigurationPageSASDevice1) / 4;
            pSASDevice->SASDevicePage1.u.fields.SASAddress = SASAddress;
            pSASDevice->SASDevicePage1.u.fields.u16DevHandle = u16DeviceHandle;
            pSASDevice->SASDevicePage1.u.fields.u8TargetID                 = i;
            pSASDevice->SASDevicePage1.u.fields.u8Bus                      = 0;

            /* SAS device page 2. */
            pSASDevice->SASDevicePage2.u.fields.ExtHeader.u8PageType       =
                        MPT_CONFIGURATION_PAGE_ATTRIBUTE_READONLY
                          | MPT_CONFIGURATION_PAGE_TYPE_EXTENDED;
            pSASDevice->SASDevicePage2.u.fields.ExtHeader.u8PageNumber     = 2;
            pSASDevice->SASDevicePage2.u.fields.ExtHeader.u8ExtPageType    =
                        MPT_CONFIGURATION_PAGE_TYPE_EXTENDED_SASDEVICE;
            pSASDevice->SASDevicePage2.u.fields.ExtHeader.u16ExtPageLength =
                        sizeof(MptConfigurationPageSASDevice2) / 4;
            pSASDevice->SASDevicePage2.u.fields.SASAddress                 =
                        SASAddress;

            /* Link into device list. */
            if (!pPages->cDevices) {
                pPages->pSASDeviceHead = pSASDevice;
                pPages->pSASDeviceTail = pSASDevice;
                pPages->cDevices = 1;
            } else {
                pSASDevice->pPrev = pPages->pSASDeviceTail;
                pPages->pSASDeviceTail->pNext = pSASDevice;
                pPages->pSASDeviceTail = pSASDevice;
                pPages->cDevices++;
            }
        }
    }
}

static void lsilogic_init_config_pages(LsilogicState *s)
{
    /* Initialize the common pages. */
    PMptConfigurationPagesSupported pPages =
        (PMptConfigurationPagesSupported)g_malloc0(
                sizeof(MptConfigurationPagesSupported));

    s->config_pages = pPages;

    /* Clear everything first. */
    memset(pPages, 0, sizeof(MptConfigurationPagesSupported));

    /* Manufacturing Page 0. */
    MPT_CONFIG_PAGE_HEADER_INIT_MANUFACTURING(&pPages->ManufacturingPage0,
                      MptConfigurationPageManufacturing0, 0,
                      MPT_CONFIGURATION_PAGE_ATTRIBUTE_PERSISTENT_READONLY);
    strncpy((char *)pPages->ManufacturingPage0.u.fields.abChipName,
                                                    "QEMU MPT Fusion", 16);
    strncpy((char *)pPages->ManufacturingPage0.u.fields.abChipRevision,
                                                    "1.0", 8);
    strncpy((char *)pPages->ManufacturingPage0.u.fields.abBoardName,
                                                    "QEMU MPT Fusion", 16);
    strncpy((char *)pPages->ManufacturingPage0.u.fields.abBoardAssembly,
                                                    "Verizon", 8);
    strncpy((char *)pPages->ManufacturingPage0.u.fields.abBoardTracerNumber,
                                                    "DEADBEEFDEADBEEF", 16);

    /* Manufacturing Page 1 - Leave it 0 for now. */
    MPT_CONFIG_PAGE_HEADER_INIT_MANUFACTURING(&pPages->ManufacturingPage1,
                      MptConfigurationPageManufacturing1, 1,
                      MPT_CONFIGURATION_PAGE_ATTRIBUTE_PERSISTENT_READONLY);

    /* Manufacturing Page 2. */
    MPT_CONFIG_PAGE_HEADER_INIT_MANUFACTURING(&pPages->ManufacturingPage2,
                      MptConfigurationPageManufacturing2, 2,
                      MPT_CONFIGURATION_PAGE_ATTRIBUTE_PERSISTENT_READONLY);

    if (s->ctrl_type == LSILOGICCTRLTYPE_SCSI_SPI) {
        pPages->ManufacturingPage2.u.fields.u16PCIDeviceID =
                LSILOGICSCSI_PCI_SPI_DEVICE_ID;
        pPages->ManufacturingPage2.u.fields.u8PCIRevisionID =
                LSILOGICSCSI_PCI_SPI_REVISION_ID;
    } else if (s->ctrl_type == LSILOGICCTRLTYPE_SCSI_SAS) {
        pPages->ManufacturingPage2.u.fields.u16PCIDeviceID =
                LSILOGICSCSI_PCI_SAS_DEVICE_ID;
        pPages->ManufacturingPage2.u.fields.u8PCIRevisionID =
                LSILOGICSCSI_PCI_SAS_REVISION_ID;
    }

    /* Manufacturing Page 3. */
    MPT_CONFIG_PAGE_HEADER_INIT_MANUFACTURING(&pPages->ManufacturingPage3,
                      MptConfigurationPageManufacturing3, 3,
                      MPT_CONFIGURATION_PAGE_ATTRIBUTE_PERSISTENT_READONLY);

    if (s->ctrl_type == LSILOGICCTRLTYPE_SCSI_SPI) {
        pPages->ManufacturingPage3.u.fields.u16PCIDeviceID =
                LSILOGICSCSI_PCI_SPI_DEVICE_ID;
        pPages->ManufacturingPage3.u.fields.u8PCIRevisionID =
                LSILOGICSCSI_PCI_SPI_REVISION_ID;
    } else if (s->ctrl_type == LSILOGICCTRLTYPE_SCSI_SAS) {
        pPages->ManufacturingPage3.u.fields.u16PCIDeviceID =
                LSILOGICSCSI_PCI_SAS_DEVICE_ID;
        pPages->ManufacturingPage3.u.fields.u8PCIRevisionID =
                LSILOGICSCSI_PCI_SAS_REVISION_ID;
    }

    /* Manufacturing Page 4 - Leave it 0 for now. */
    MPT_CONFIG_PAGE_HEADER_INIT_MANUFACTURING(&pPages->ManufacturingPage4,
                      MptConfigurationPageManufacturing4, 4,
                      MPT_CONFIGURATION_PAGE_ATTRIBUTE_PERSISTENT_READONLY);

    /* Manufacturing Page 5 - WWID settings. */
    MPT_CONFIG_PAGE_HEADER_INIT_MANUFACTURING(&pPages->ManufacturingPage5,
                      MptConfigurationPageManufacturing5, 5,
                      MPT_CONFIGURATION_PAGE_ATTRIBUTE_PERSISTENT_READONLY);

    /* Manufacturing Page 6 - Product specific settings. */
    MPT_CONFIG_PAGE_HEADER_INIT_MANUFACTURING(&pPages->ManufacturingPage6,
                                  MptConfigurationPageManufacturing6, 6,
                                  MPT_CONFIGURATION_PAGE_ATTRIBUTE_CHANGEABLE);

    /* Manufacturing Page 8 -  Product specific settings. */
    MPT_CONFIG_PAGE_HEADER_INIT_MANUFACTURING(&pPages->ManufacturingPage8,
                                  MptConfigurationPageManufacturing8, 8,
                                  MPT_CONFIGURATION_PAGE_ATTRIBUTE_CHANGEABLE);

    /* Manufacturing Page 9 -  Product specific settings. */
    MPT_CONFIG_PAGE_HEADER_INIT_MANUFACTURING(&pPages->ManufacturingPage9,
                                  MptConfigurationPageManufacturing9, 9,
                                  MPT_CONFIGURATION_PAGE_ATTRIBUTE_CHANGEABLE);

    /* Manufacturing Page 10 -  Product specific settings. */
    MPT_CONFIG_PAGE_HEADER_INIT_MANUFACTURING(&pPages->ManufacturingPage10,
                                  MptConfigurationPageManufacturing10, 10,
                                  MPT_CONFIGURATION_PAGE_ATTRIBUTE_CHANGEABLE);

    /* I/O Unit page 0. */
    MPT_CONFIG_PAGE_HEADER_INIT_IO_UNIT(&pPages->IOUnitPage0,
                                MptConfigurationPageIOUnit0, 0,
                                MPT_CONFIGURATION_PAGE_ATTRIBUTE_READONLY);
    pPages->IOUnitPage0.u.fields.u64UniqueIdentifier = 0xcafe;

    /* I/O Unit page 1. */
    MPT_CONFIG_PAGE_HEADER_INIT_IO_UNIT(&pPages->IOUnitPage1,
                                MptConfigurationPageIOUnit1, 1,
                                MPT_CONFIGURATION_PAGE_ATTRIBUTE_READONLY);
    pPages->IOUnitPage1.u.fields.fSingleFunction         = true;
    pPages->IOUnitPage1.u.fields.fAllPathsMapped         = false;
    pPages->IOUnitPage1.u.fields.fIntegratedRAIDDisabled = true;
    pPages->IOUnitPage1.u.fields.f32BitAccessForced      = false;

    /* I/O Unit page 2. */
    MPT_CONFIG_PAGE_HEADER_INIT_IO_UNIT(&pPages->IOUnitPage2,
                                MptConfigurationPageIOUnit2, 2,
                                MPT_CONFIGURATION_PAGE_ATTRIBUTE_PERSISTENT);
    pPages->IOUnitPage2.u.fields.fPauseOnError       = false;
    pPages->IOUnitPage2.u.fields.fVerboseModeEnabled = false;
    pPages->IOUnitPage2.u.fields.fDisableColorVideo  = false;
    pPages->IOUnitPage2.u.fields.fNotHookInt40h      = false;
    pPages->IOUnitPage2.u.fields.u32BIOSVersion      = 0xdeadbeef;
    pPages->IOUnitPage2.u.fields.aAdapterOrder[0].fAdapterEnabled = true;
    pPages->IOUnitPage2.u.fields.aAdapterOrder[0].fAdapterEmbedded = true;
    pPages->IOUnitPage2.u.fields.aAdapterOrder[0].u8PCIBusNumber = 0;
    pPages->IOUnitPage2.u.fields.aAdapterOrder[0].u8PCIDevFn = s->dev.devfn;

    /* I/O Unit page 3. */
    MPT_CONFIG_PAGE_HEADER_INIT_IO_UNIT(&pPages->IOUnitPage3,
                                MptConfigurationPageIOUnit3, 3,
                                MPT_CONFIGURATION_PAGE_ATTRIBUTE_CHANGEABLE);
    pPages->IOUnitPage3.u.fields.u8GPIOCount = 0;

    /* I/O Unit page 4. */
    MPT_CONFIG_PAGE_HEADER_INIT_IO_UNIT(&pPages->IOUnitPage4,
                                MptConfigurationPageIOUnit4, 4,
                                MPT_CONFIGURATION_PAGE_ATTRIBUTE_CHANGEABLE);

    /* IOC page 0. */
    MPT_CONFIG_PAGE_HEADER_INIT_IOC(&pPages->IOCPage0,
                                MptConfigurationPageIOC0, 0,
                                MPT_CONFIGURATION_PAGE_ATTRIBUTE_READONLY);
    pPages->IOCPage0.u.fields.u32TotalNVStore      = 0;
    pPages->IOCPage0.u.fields.u32FreeNVStore       = 0;

    if (s->ctrl_type == LSILOGICCTRLTYPE_SCSI_SPI) {
        pPages->IOCPage0.u.fields.u16VendorId          =
                LSILOGICSCSI_PCI_VENDOR_ID;
        pPages->IOCPage0.u.fields.u16DeviceId          =
                LSILOGICSCSI_PCI_SPI_DEVICE_ID;
        pPages->IOCPage0.u.fields.u8RevisionId         =
                LSILOGICSCSI_PCI_SPI_REVISION_ID;
        pPages->IOCPage0.u.fields.u32ClassCode         =
                LSILOGICSCSI_PCI_SPI_CLASS_CODE;
        pPages->IOCPage0.u.fields.u16SubsystemVendorId =
                LSILOGICSCSI_PCI_SPI_SUBSYSTEM_VENDOR_ID;
        pPages->IOCPage0.u.fields.u16SubsystemId       =
                LSILOGICSCSI_PCI_SPI_SUBSYSTEM_ID;
    } else if (s->ctrl_type == LSILOGICCTRLTYPE_SCSI_SAS) {
        pPages->IOCPage0.u.fields.u16VendorId          =
                LSILOGICSCSI_PCI_VENDOR_ID;
        pPages->IOCPage0.u.fields.u16DeviceId          =
                LSILOGICSCSI_PCI_SAS_DEVICE_ID;
        pPages->IOCPage0.u.fields.u8RevisionId         =
                LSILOGICSCSI_PCI_SAS_REVISION_ID;
        pPages->IOCPage0.u.fields.u32ClassCode         =
                LSILOGICSCSI_PCI_SAS_CLASS_CODE;
        pPages->IOCPage0.u.fields.u16SubsystemVendorId =
                LSILOGICSCSI_PCI_SAS_SUBSYSTEM_VENDOR_ID;
        pPages->IOCPage0.u.fields.u16SubsystemId       =
                LSILOGICSCSI_PCI_SAS_SUBSYSTEM_ID;
    }

    /* IOC page 1. */
    MPT_CONFIG_PAGE_HEADER_INIT_IOC(&pPages->IOCPage1,
                            MptConfigurationPageIOC1, 1,
                            MPT_CONFIGURATION_PAGE_ATTRIBUTE_CHANGEABLE);
    pPages->IOCPage1.u.fields.fReplyCoalescingEnabled = false;
    pPages->IOCPage1.u.fields.u32CoalescingTimeout    = 0;
    pPages->IOCPage1.u.fields.u8CoalescingDepth       = 0;

    /* IOC page 2. */
    MPT_CONFIG_PAGE_HEADER_INIT_IOC(&pPages->IOCPage2,
                                MptConfigurationPageIOC2, 2,
                                MPT_CONFIGURATION_PAGE_ATTRIBUTE_READONLY);
    /* Everything else here is 0. */

    /* IOC page 3. */
    MPT_CONFIG_PAGE_HEADER_INIT_IOC(&pPages->IOCPage3,
                                MptConfigurationPageIOC3, 3,
                                MPT_CONFIGURATION_PAGE_ATTRIBUTE_READONLY);
    /* Everything else here is 0. */

    /* IOC page 4. */
    MPT_CONFIG_PAGE_HEADER_INIT_IOC(&pPages->IOCPage4,
                                MptConfigurationPageIOC4, 4,
                                MPT_CONFIGURATION_PAGE_ATTRIBUTE_READONLY);
    /* Everything else here is 0. */

    /* IOC page 6. */
    MPT_CONFIG_PAGE_HEADER_INIT_IOC(&pPages->IOCPage6,
                                MptConfigurationPageIOC6, 6,
                                MPT_CONFIGURATION_PAGE_ATTRIBUTE_READONLY);
    /* Everything else here is 0. */

    /* BIOS page 1. */
    MPT_CONFIG_PAGE_HEADER_INIT_BIOS(&pPages->BIOSPage1,
                                 MptConfigurationPageBIOS1, 1,
                                 MPT_CONFIGURATION_PAGE_ATTRIBUTE_CHANGEABLE);

    /* BIOS page 2. */
    MPT_CONFIG_PAGE_HEADER_INIT_BIOS(&pPages->BIOSPage2,
                                 MptConfigurationPageBIOS2, 2,
                                 MPT_CONFIGURATION_PAGE_ATTRIBUTE_CHANGEABLE);

    /* BIOS page 4. */
    MPT_CONFIG_PAGE_HEADER_INIT_BIOS(&pPages->BIOSPage4,
                                 MptConfigurationPageBIOS4, 4,
                                 MPT_CONFIGURATION_PAGE_ATTRIBUTE_CHANGEABLE);

    if (s->ctrl_type == LSILOGICCTRLTYPE_SCSI_SPI) {
        lsilogic_init_config_pages_spi(s);
    } else if (s->ctrl_type == LSILOGICCTRLTYPE_SCSI_SAS) {
        lsilogic_init_config_pages_sas(s);
    }
}


static int lsilogic_hard_reset(LsilogicState *s)
{

    s->intr_mask |= LSILOGIC_REG_HOST_INTR_MASK_DOORBELL |
                             LSILOGIC_REG_HOST_INTR_MASK_REPLY;
    lsilogic_soft_reset(s);

    /* Set default values. */
    if (s->ctrl_type == LSILOGICCTRLTYPE_SCSI_SPI) {
        s->max_devices = LSILOGICSCSI_PCI_SPI_PORTS_MAX *
                         LSILOGICSCSI_PCI_SPI_DEVICES_PER_BUS_MAX;
    } else if (s->ctrl_type == LSILOGICCTRLTYPE_SCSI_SAS) {
        s->max_devices = LSILOGICSCSI_PCI_SAS_PORTS_MAX *
                         LSILOGICSCSI_PCI_SAS_DEVICES_PER_PORT_MAX;
    }
    s->max_buses     = 1;
    s->reply_frame_size  = 128; /* @todo Figure out where it is needed. */
    s->next_handle = 1;

    lsilogic_config_pages_free(s);
    lsilogic_init_config_pages(s);

    /* Mark that we finished performing the reset. */
    s->state = LSILOGICSTATE_READY;
    return 0;
}

static void lsilogic_scsi_reset(DeviceState *dev)
{
    LsilogicState *s = DO_UPCAST(LsilogicState, dev.qdev, dev);

    lsilogic_hard_reset(s);
}

static const VMStateDescription vmstate_lsilogic = {
    .name = "lsilogic",
    .version_id = 0,
    .minimum_version_id = 0,
    .minimum_version_id_old = 0,
    .fields      = (VMStateField[]) {
        VMSTATE_PCI_DEVICE(dev, LsilogicState),

        VMSTATE_UINT32(intr_mask, LsilogicState),
        VMSTATE_UINT32(doorbell, LsilogicState),
        VMSTATE_END_OF_LIST()
    }
};

static void lsilogic_queues_free(LsilogicState *s)
{
    assert(s->reply_free_queue);

    g_free(s->reply_free_queue);

    s->reply_free_queue = NULL;
    s->reply_post_queue = NULL;
    s->request_queue = NULL;
}

static void lsilogic_scsi_uninit(PCIDevice *d)
{
    LsilogicState *s = DO_UPCAST(LsilogicState, dev, d);

    lsilogic_queues_free(s);
#ifdef USE_MSIX
    msix_uninit(&s->dev, &s->mmio_io);
#endif
    memory_region_destroy(&s->mmio_io);
    memory_region_destroy(&s->port_io);
    memory_region_destroy(&s->diag_io);
}

static const struct SCSIBusInfo lsilogic_scsi_info = {
    .tcq = true,
    .max_target = LSILOGICSCSI_PCI_SAS_PORTS_MAX,
    .max_lun = 1,

    .transfer_data = lsilogic_xfer_complete,
    .get_sg_list = lsilogic_get_sg_list,
    .complete = lsilogic_command_complete,
    .cancel = lsilogic_command_cancel,
};

static int lsilogic_queues_alloc(LsilogicState *s)
{
    uint32_t cbQueues;

    assert(!s->reply_free_queue);

    cbQueues  = 2*s->reply_queue_entries * sizeof(uint32_t);
    cbQueues += s->request_queue_entries * sizeof(uint32_t);

    s->reply_free_queue = g_malloc0(cbQueues);

    s->reply_post_queue = s->reply_free_queue + s->reply_queue_entries;

    s->request_queue   = s->reply_post_queue + s->reply_queue_entries;

    return 0;
}

static int lsilogic_scsi_init(PCIDevice *dev, LSILOGICCTRLTYPE ctrl_type)
{
    LsilogicState *s = DO_UPCAST(LsilogicState, dev, dev);
    uint8_t *pci_conf;

    s->ctrl_type = ctrl_type;

    pci_conf = s->dev.config;

    /* PCI latency timer = 0 */
    pci_conf[PCI_LATENCY_TIMER] = 0;
    /* Interrupt pin 1 */
    pci_conf[PCI_INTERRUPT_PIN] = 0x01;

    if (s->ctrl_type == LSILOGICCTRLTYPE_SCSI_SPI) {
        memory_region_init_io(&s->mmio_io, &lsilogic_mmio_ops, s,
                              "lsilogic-mmio", 0x4000);
        memory_region_init_io(&s->port_io, &lsilogic_port_ops, s,
                              "lsilogic-io", 256);
        memory_region_init_io(&s->diag_io, &lsilogic_diag_ops, s,
                              "lsilogic-diag", 0x10000);
    } else if (s->ctrl_type == LSILOGICCTRLTYPE_SCSI_SAS) {
        memory_region_init_io(&s->mmio_io, &lsilogic_mmio_ops, s,
                              "lsilogic-mmio", 0x4000);
        memory_region_init_io(&s->port_io, &lsilogic_port_ops, s,
                              "lsilogic-io", 256);
        memory_region_init_io(&s->diag_io, &lsilogic_diag_ops, s,
                              "lsilogic-diag", 0x10000);
    }

#ifdef USE_MSIX
    /* MSI-X support is currently broken */
    if (lsilogic_use_msix(s) &&
        msix_init(&s->dev, 15, &s->mmio_io, 0, 0x2000)) {
        s->flags &= ~LSILOGIC_MASK_USE_MSIX;
    }
#else
    s->flags &= ~LSILOGIC_MASK_USE_MSIX;
#endif

    pci_register_bar(&s->dev, 0, PCI_BASE_ADDRESS_SPACE_IO, &s->port_io);
    pci_register_bar(&s->dev, 1, PCI_BASE_ADDRESS_SPACE_MEMORY |
                                 PCI_BASE_ADDRESS_MEM_TYPE_32, &s->mmio_io);
    pci_register_bar(&s->dev, 2, PCI_BASE_ADDRESS_SPACE_MEMORY |
                                 PCI_BASE_ADDRESS_MEM_TYPE_32, &s->diag_io);

    if (lsilogic_use_msix(s)) {
        msix_vector_use(&s->dev, 0);
    }

    if (!s->sas_addr) {
        s->sas_addr = ((NAA_LOCALLY_ASSIGNED_ID << 24) |
                       IEEE_COMPANY_LOCALLY_ASSIGNED) << 36;
        s->sas_addr |= (pci_bus_num(dev->bus) << 16);
        s->sas_addr |= (PCI_SLOT(dev->devfn) << 8);
        s->sas_addr |= PCI_FUNC(dev->devfn);
    }
    s->reply_queue_entries = LSILOGICSCSI_REPLY_QUEUE_DEPTH_DEFAULT + 1;
    s->request_queue_entries = LSILOGICSCSI_REQUEST_QUEUE_DEPTH_DEFAULT + 1;
    lsilogic_queues_alloc(s);

    trace_lsilogic_init(0, 0,
                       lsilogic_use_msix(s) ? "MSI-X" : "INTx",
                       lsilogic_is_sas(s) ? "sas" : "scsi");

    if (s->ctrl_type == LSILOGICCTRLTYPE_SCSI_SPI) {
        s->ports = LSILOGICSCSI_PCI_SPI_PORTS_MAX;
        s->max_devices = s->ports * LSILOGICSCSI_PCI_SPI_DEVICES_PER_BUS_MAX;
    } else if (s->ctrl_type == LSILOGICCTRLTYPE_SCSI_SAS) {
        s->max_devices = s->ports * LSILOGICSCSI_PCI_SAS_DEVICES_PER_PORT_MAX;
    }

    scsi_bus_new(&s->bus, &dev->qdev, &lsilogic_scsi_info);
    scsi_bus_legacy_handle_cmdline(&s->bus);
    return 0;
}

static int lsilogic_scsi_spi_init(PCIDevice *dev)
{
    return lsilogic_scsi_init(dev, LSILOGICCTRLTYPE_SCSI_SPI);
}

static int lsilogic_scsi_sas_init(PCIDevice *dev)
{
    return lsilogic_scsi_init(dev, LSILOGICCTRLTYPE_SCSI_SAS);
}

static Property lsilogicscsi_properties[] = {
#ifdef USE_MSIX
    DEFINE_PROP_BIT("use_msix", LsilogicState, flags,
                    LSILOGIC_FLAG_USE_MSIX, false),
#endif
    DEFINE_PROP_END_OF_LIST(),
};

static Property lsilogicsas_properties[] = {
    DEFINE_PROP_UINT32("ports", LsilogicState, ports,
                       LSILOGICSCSI_PCI_SAS_PORTS_DEFAULT),
    DEFINE_PROP_HEX64("sas_address", LsilogicState, sas_addr, 0),
#ifdef USE_MSIX
    DEFINE_PROP_BIT("use_msix", LsilogicState, flags,
                    LSILOGIC_FLAG_USE_MSIX, false),
#endif
    DEFINE_PROP_END_OF_LIST(),
};

static void lsilogicscsi_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PCIDeviceClass *pc = PCI_DEVICE_CLASS(oc);

    pc->init = lsilogic_scsi_spi_init;
    pc->exit = lsilogic_scsi_uninit;
    pc->vendor_id = PCI_VENDOR_ID_LSI_LOGIC;
    pc->device_id = PCI_DEVICE_ID_LSI_53C1030;
    pc->subsystem_vendor_id = PCI_VENDOR_ID_LSI_LOGIC;
    pc->subsystem_id = 0x8000;
    pc->class_id = PCI_CLASS_STORAGE_SCSI;
    dc->props = lsilogicscsi_properties;
    dc->reset = lsilogic_scsi_reset;
    dc->vmsd = &vmstate_lsilogic;
    dc->desc = "LSI SCSI 53C1030";
}

static void lsilogicsas_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PCIDeviceClass *pc = PCI_DEVICE_CLASS(oc);

    pc->init = lsilogic_scsi_sas_init;
    pc->exit = lsilogic_scsi_uninit;
    pc->romfile = 0;
    pc->vendor_id = PCI_VENDOR_ID_LSI_LOGIC;
    pc->device_id = PCI_DEVICE_ID_LSI_SAS1068;
    pc->subsystem_vendor_id = PCI_VENDOR_ID_LSI_LOGIC;
    pc->subsystem_id = 0x8000;
    pc->class_id = PCI_CLASS_STORAGE_SCSI;
    dc->props = lsilogicsas_properties;
    dc->reset = lsilogic_scsi_reset;
    dc->vmsd = &vmstate_lsilogic;
    dc->desc = "LSI SAS 1068";
}

static void lsilogicsase_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PCIDeviceClass *pc = PCI_DEVICE_CLASS(oc);

    pc->init = lsilogic_scsi_sas_init;
    pc->exit = lsilogic_scsi_uninit;
    pc->romfile = 0;
    pc->vendor_id = PCI_VENDOR_ID_LSI_LOGIC;
    pc->device_id = PCI_DEVICE_ID_LSI_SAS1068E;
    pc->subsystem_vendor_id = PCI_VENDOR_ID_LSI_LOGIC;
    pc->subsystem_id = 0x8000;
    pc->is_express = 1;
    pc->class_id = PCI_CLASS_STORAGE_SCSI;
    dc->props = lsilogicsas_properties;
    dc->reset = lsilogic_scsi_reset;
    dc->vmsd = &vmstate_lsilogic;
    dc->desc = "LSI SAS 1068E";
}

static const TypeInfo lsilogic_info[] = {
    {
        .name  = "lsi53c1030",
        .parent = TYPE_PCI_DEVICE,
        .instance_size = sizeof(LsilogicState),
        .class_init = lsilogicscsi_class_init,
    }, {
        .name  = "sas1068",
        .parent = TYPE_PCI_DEVICE,
        .instance_size = sizeof(LsilogicState),
        .class_init = lsilogicsas_class_init,
    }, {
        .name  = "sas1068e",
        .parent = TYPE_PCI_DEVICE,
        .instance_size = sizeof(LsilogicState),
        .class_init = lsilogicsase_class_init,
    }
};

static void lsilogic_register_types(void)
{
    unsigned i;
    for (i = 0; i < ARRAY_SIZE(lsilogic_info); i++) {
        type_register(&lsilogic_info[i]);
    }
}

type_init(lsilogic_register_types)
