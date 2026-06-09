/*
 * Copyright (c) 2023 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "dev/amdgpu/amdgpu_nbio.hh"

#include "debug/AMDGPUDevice.hh"
#include "dev/amdgpu/amdgpu_device.hh"
#include "mem/packet_access.hh"

namespace gem5 {

AMDGPUNbio::AMDGPUNbio()
{
    // All read-before-write MMIOs go here
    triggered_reads[AMDGPU_MP0_SMN_C2PMSG_64] = 0x80000000;
}

void
AMDGPUNbio::setGPUDevice(AMDGPUDevice* gpu_device)
{
    gpuDevice = gpu_device;
}

void
AMDGPUNbio::readMMIO(PacketPtr pkt, Addr offset)
{
    // For Vega10 we rely on the golden values in an MMIO trace. Return
    // immediately as to not clobber those values.
    if (gpuDevice->getGfxVersion() == GfxVersion::gfx900) {
        if (offset == AMDGPU_PCIE_DATA || offset == AMDGPU_PCIE_DATA2) {
            return;
        }
    }

    DPRINTF(AMDGPUDevice, "NBIO Read MMIO offset %#x\n", offset);
    switch (offset) {
        // PCIE_DATA, PCIE_DATA2, PCIE_INDEX, and PCIE_INDEX2 handle "indirect
        // "register reads/writes from the driver. This provides a way to read
        // any register by providing a 32-bit address to one of the two INDEX
        // registers and then reading the corresponding DATA register. See:
        // https://github.com/ROCm/ROCK-Kernel-Driver/blob/roc-6.0.x/drivers/
        //     gpu/drm/amd/amdgpu/amdgpu_device.c#L459
        case AMDGPU_PCIE_DATA: {
            uint32_t value = gpuDevice->getRegVal(pcie_index_reg);
            DPRINTF(AMDGPUDevice, "Read PCIe index %lx data %x\n",
                    pcie_index_reg, value);
            pkt->setLE<uint32_t>(value);
        } break;
        case AMDGPU_PCIE_DATA2: {
            uint32_t value = gpuDevice->getRegVal(pcie_index2_reg);
            DPRINTF(AMDGPUDevice, "Read PCIe index2 %lx data2 %x\n",
                    pcie_index2_reg, value);
            pkt->setLE<uint32_t>(value);
        } break;
        case AMDGPU_PCIE_INDEX:
            pkt->setLE<uint32_t>(pcie_index_reg);
            break;
        case AMDGPU_PCIE_INDEX2:
            pkt->setLE<uint32_t>(pcie_index2_reg);
            break;
        case AMDGPU_MM_DATA:
            pkt->setLE<uint32_t>(gpuDevice->getRegVal(mm_index_reg));
            break;
        case VEGA10_INV_ENG17_ACK1:
        case VEGA10_INV_ENG17_ACK2:
        case MI100_INV_ENG17_ACK2:
        case MI100_INV_ENG17_ACK3:
        case MI200_INV_ENG17_ACK2:
        case MI300X_INV_ENG17_ACK1:
        case MI300X_INV_ENG17_ACK2:
        case MI300X_INV_ENG17_ACK3:
        case MI300X_INV_ENG17_ACK4:
        case MI300X_INV_ENG17_ACK5:
        case MI300X_INV_ENG17_ACK6:
        case MI300X_INV_ENG17_ACK7:
        case MI300X_INV_ENG17_ACK8:
        case MI300X_INV_ENG17_ACK9:
        case MI300X_INV_ENG17_ACK10:
        case MI300X_INV_ENG17_ACK11:
            pkt->setLE<uint32_t>(0x10001);
            break;
        case VEGA10_INV_ENG17_SEM1:
        case VEGA10_INV_ENG17_SEM2:
        case MI100_INV_ENG17_SEM2:
        case MI100_INV_ENG17_SEM3:
        case MI200_INV_ENG17_SEM2:
            pkt->setLE<uint32_t>(0x1);
            break;
        // PSP responds with bit 31 set when ready
        case AMDGPU_MP0_SMN_C2PMSG_35:
            pkt->setLE<uint32_t>(0x80000000);
            break;
        case AMDGPU_MP1_SMN_C2PMSG_90:
            pkt->setLE<uint32_t>(0x1);
            break;
        case MI300X_EPF0_STRAP0:
            // This contains a revision ID for the chip. It is required for
            // MI300X to see the GFX target as gfx942 instead of gfx941.
            if (gpuDevice->getGfxVersion() == GfxVersion::gfx942) {
                pkt->setLE<uint32_t>(2 << 24);
            } else {
                pkt->setLE<uint32_t>(0);
            }
            break;
        case MI200_BIOS_SCRATCH_7:
            pkt->setLE<uint32_t>(0x200); // ATOM_S7_ASIC_INIT_COMPLETE_MASK
            break;
        default:
            if (triggered_reads.count(offset)) {
                DPRINTF(AMDGPUDevice, "Found triggered read for %#x\n",
                        offset);
                pkt->setLE<uint32_t>(triggered_reads[offset]);
            } else if (regs.count(offset)) {
                DPRINTF(AMDGPUDevice,
                        "Returning value of unknown MMIO offset "
                        "%x: %x\n",
                        offset, regs[offset]);
                pkt->setLE<uint32_t>(regs[offset]);
            } else {
                DPRINTF(AMDGPUDevice, "NBIO Unknown MMIO %#x (%#x)\n", offset,
                        pkt->getAddr());
            }
            break;
    }
}

void
AMDGPUNbio::writeMMIO(PacketPtr pkt, Addr offset)
{
    if (offset == AMDGPU_MM_INDEX) {
        assert(pkt->getSize() == 4);
        mm_index_reg = insertBits(mm_index_reg, 31, 0, pkt->getLE<uint32_t>());
    } else if (offset == AMDGPU_MM_INDEX_HI) {
        assert(pkt->getSize() == 4);
        mm_index_reg =
            insertBits(mm_index_reg, 63, 32, pkt->getLE<uint32_t>());
    } else if (offset == AMDGPU_MM_DATA) {
        DPRINTF(AMDGPUDevice, "MM write to reg %#lx data %#lx\n", mm_index_reg,
                pkt->getLE<uint32_t>());
        gpuDevice->setRegVal(AMDGPU_MM_DATA, pkt->getLE<uint32_t>());
        // PCIE_DATA, PCIE_DATA2, PCIE_INDEX, and PCIE_INDEX2 handle "indirect
        // "register reads/writes from the driver. This provides a way to read
        // any register by providing a 32-bit address to one of the two INDEX
        // registers and then reading the corresponding DATA register. See:
        // https://github.com/ROCm/ROCK-Kernel-Driver/blob/roc-6.0.x/drivers/
        //     gpu/drm/amd/amdgpu/amdgpu_device.c#L459
    } else if (offset == AMDGPU_PCIE_INDEX) {
        assert(pkt->getSize() == 4);
        pcie_index_reg = pkt->getLE<uint32_t>();
    } else if (offset == AMDGPU_PCIE_DATA) {
        assert(pkt->getSize() == 4);
        gpuDevice->setRegVal(pcie_index_reg, pkt->getLE<uint32_t>());
    } else if (offset == AMDGPU_PCIE_INDEX2) {
        assert(pkt->getSize() == 4);
        pcie_index2_reg = pkt->getLE<uint32_t>();
    } else if (offset == AMDGPU_PCIE_DATA2) {
        assert(pkt->getSize() == 4);
        gpuDevice->setRegVal(pcie_index2_reg, pkt->getLE<uint32_t>());
    } else if (offset == AMDGPU_MP0_SMN_C2PMSG_35) {
        // See psp_v3_1_bootloader_load_sos in amdgpu driver code.
        if (pkt->getLE<uint32_t>() == 0x10000) {
            triggered_reads[AMDGPU_MP0_SMN_C2PMSG_81] = 0xdf40b31;
        }
    } else if (offset == AMDGPU_MP0_SMN_C2PMSG_64) {
        triggered_reads[AMDGPU_MP0_SMN_C2PMSG_64] =
            0x80000000 + pkt->getLE<uint32_t>();
    } else if (offset == AMDGPU_MP0_SMN_C2PMSG_69) {
        // PSP ring low addr
        psp_ring = insertBits(psp_ring, 31, 0, pkt->getLE<uint32_t>());
        psp_ring_listen_addr =
            psp_ring - gpuDevice->getVM().getSysAddrRangeLow() + 0xc;
    } else if (offset == AMDGPU_MP0_SMN_C2PMSG_70) {
        // PSP ring high addr
        psp_ring = insertBits(psp_ring, 63, 32, pkt->getLE<uint32_t>());
        psp_ring_listen_addr =
            psp_ring - gpuDevice->getVM().getSysAddrRangeLow() + 0xc;
    } else if (offset == AMDGPU_MP0_SMN_C2PMSG_71) {
        // PSP ring size
        psp_ring_size = pkt->getLE<uint32_t>();

    } else if (offset == AMDGPU_MP0_SMN_C2PMSG_67) {
        // AMDGPU_MP0_SMN_C2PMSG_67: PSP wptr update
        wptr = pkt->getLE<uint32_t>();
        // print message
        DPRINTF(AMDGPUDevice, "PSP: Received wptr update to %u\n", wptr);
        // Call function to read packet and process commands
        processPspCommand(wptr);
    } else if (offset == NBIO_PARTITION_COMPUTE_STATUS) {
        DPRINTF(AMDGPUDevice,
                "Writing NBIO_PARTITION_COMPUTE_STATUS "
                "value %x\n",
                pkt->getLE<uint32_t>());
        regs[offset] = pkt->getLE<uint32_t>();
    } else if (is_MI200_regBM_PAGE_TABLE_BASE_ADDR(offset)) {
        uint16_t context_id =
            get_context_from_MI200_regBM_PAGE_TABLE_BASE_ADDR(offset);
        regs[offset] = pkt->getLE<uint32_t>();
        if ((offset % 8) == 0) {
            // The register write is to ptBaseH
            gpuDevice->getVM().setPageTableBaseH(context_id,
                                                 pkt->getLE<uint32_t>());
        } else {
            // The register write is to ptBaseL
            gpuDevice->getVM().setPageTableBaseL(context_id,
                                                 pkt->getLE<uint32_t>());
        }
    } else if (is_MI200_regBM_PAGE_TABLE_START_ADDR(offset)) {
        uint16_t context_id =
            get_context_from_MI200_regBM_PAGE_TABLE_START_ADDR(offset);
        regs[offset] = pkt->getLE<uint32_t>();
        if ((offset % 8) == 0) {
            // The register write is to ptBaseH
            gpuDevice->getVM().setPageTableStartH(context_id,
                                                  pkt->getLE<uint32_t>());
        } else {
            // The register write is to ptBaseL
            gpuDevice->getVM().setPageTableStartL(context_id,
                                                  pkt->getLE<uint32_t>());
        }
    } else if (is_MI200_regBM_PAGE_TABLE_END_ADDR(offset)) {
        uint16_t context_id =
            get_context_from_MI200_regBM_PAGE_TABLE_END_ADDR(offset);
        regs[offset] = pkt->getLE<uint32_t>();
        // MI200 page table addresses are 64 bits long. There are
        // separate registers to handle the lower 32 bits and upper 32
        // bits. Use the MMIO offset to figure out which part of the
        // address is being written to
        if ((offset % 8) == 0) {
            // The register write is to ptBaseH
            gpuDevice->getVM().setPageTableEndH(context_id,
                                                pkt->getLE<uint32_t>());
        } else {
            // The register write is to ptBaseL
            gpuDevice->getVM().setPageTableEndL(context_id,
                                                pkt->getLE<uint32_t>());
        }
    } else {
        // Fallback to a map of register values. This was previously in the
        // AMDGPUDevice, however that short-circuited some reads from other
        // IP blocks. Since this is an end point IP block it is safer to use
        // here.
        regs[offset] = pkt->getLE<uint32_t>();
    }
}

bool
AMDGPUNbio::readFrame(PacketPtr pkt, Addr offset)
{
    if (offset == psp_ring_dev_addr) {
        psp_ring_value++;
        pkt->setUintX(psp_ring_value, ByteOrder::little);

        return true;
    }

    return false;
}

void
AMDGPUNbio::writeFrame(PacketPtr pkt, Addr offset)
{
    if (offset == psp_ring_listen_addr) {
        DPRINTF(AMDGPUDevice,
                "Saw psp_ring_listen_addr with size %ld value "
                "%ld\n",
                pkt->getSize(), pkt->getUintX(ByteOrder::little));

        /*
         * In ROCm versions 4.x this packet is a 4 byte value. In ROCm 5.x
         * the packet is 8 bytes and mapped as a system address which needs
         * to be subtracted out to get the framebuffer address.
         */
        if (pkt->getSize() == 4) {
            psp_ring_dev_addr = pkt->getLE<uint32_t>();
        } else if (pkt->getSize() == 8) {
            psp_ring_dev_addr = pkt->getUintX(ByteOrder::little) -
                                gpuDevice->getVM().getSysAddrRangeLow();
        } else {
            panic("Invalid write size to psp_ring_listen_addr\n");
        }

        DPRINTF(AMDGPUDevice, "Setting PSP ring device address to %#lx\n",
                psp_ring_dev_addr);
    }
}

// new function to process PSP commands
void
AMDGPUNbio::processPspCommand(uint32_t new_wptr)
{
    // The wptr written is the *new* wptr, pointing to the
    // next available frame.
    // The command was submitted to the frame corresponding
    // to the *previous* wptr.
    uint32_t rb_frame_size_dw = PSP_RB_FRAME_SIZE_DWORDS;
    uint32_t ring_size_dw = psp_ring_size / 4;

    uint32_t submitted_wptr_dw;

    if (new_wptr == 0) {
        submitted_wptr_dw = ring_size_dw - rb_frame_size_dw;
    } else {
        submitted_wptr_dw = (new_wptr - rb_frame_size_dw) % ring_size_dw;
    }

    // Calculate the physical address of the submitted frame
    Addr frame_addr = psp_ring + (Addr)submitted_wptr_dw * 4;
    DPRINTF(AMDGPUDevice,
            "PSP: Processing command at frame address before the Addr "
            "translation %#lx\n ",
            frame_addr);

    bool check_addr = gpuDevice->getVM().inMMHUB(frame_addr);
    if (check_addr) {
        frame_addr = frame_addr - gpuDevice->getVM().getMMHUBBase();
    } else {
        DPRINTF(AMDGPUDevice,
                "PSP: Frame address %#lx not in MMHUB aperture.Cannot process "
                "command.\n",
                frame_addr);
        return;
    }

    // Allocate the context object to pass state asynchronously
    PspCommandContext *ctx = new PspCommandContext;
    ctx->name = name();

    // 1. Read the Ring Buffer Frame (64 bytes)
    DPRINTF(AMDGPUDevice,
            "PSP: Reading 64B ring frame from physical address %#lx\n",
            frame_addr);

// Extract key addresses immediately after read

    // Schedule the next stage: reading the command buffer, simulating latency
    auto cb = new EventFunctionWrapper(
        [=]{
        readCmdBufferAndProcess(ctx); }, ctx->name);

    gpuDevice->getMemMgr()->readRequest(frame_addr,
                                        (uint8_t*)&ctx->frame,
                                        sizeof(PspGfxRbFrame),
                                        0,
                                        cb);
}

void
AMDGPUNbio::readCmdBufferAndProcess(PspCommandContext *ctx)
{
    // 2. Functional Read of the entire Command Buffer (Max 1KB size)

    ctx->cmd_buf_addr =
        (Addr)ctx->frame.cmd_buf_addr_hi << 32 | ctx->frame.cmd_buf_addr_lo;
    ctx->fence_addr =
        (Addr)ctx->frame.fence_addr_hi << 32 | ctx->frame.fence_addr_lo;
    ctx->fence_value = ctx->frame.fence_value;

    DPRINTF(AMDGPUDevice,
            "PSP: CmdBufAddr=%#lx, FenceAddr=%#lx,FenceValue = % u\n "
            "cmd_buf_size = % u\n ",
            ctx->cmd_buf_addr, ctx->fence_addr, ctx->fence_value,
            ctx->frame.cmd_buf_size);

    bool check_addr = gpuDevice->getVM().inMMHUB(ctx->cmd_buf_addr);
    DPRINTF(AMDGPUDevice,
            "PSP: Command Buffer address %#lx in MMHUB aperture: % d\n MMHUB "
            "Base = % #lx\n ",
            ctx->cmd_buf_addr, check_addr, gpuDevice->getVM().getMMHUBBase());
    if (!check_addr) {
        DPRINTF(AMDGPUDevice,
                "PSP: Command Buffer address %#lx not in MMHUB "
                "aperture.Cannot process command  .\n ",
                ctx->cmd_buf_addr);
    } else {
        ctx->cmd_buf_addr =
            ctx->cmd_buf_addr - gpuDevice->getVM().getMMHUBBase();
        DPRINTF(AMDGPUDevice,
                "PSP: Translated Command Buffer address to %#lx\n ",
                ctx->cmd_buf_addr);
    }

    auto cb = new EventFunctionWrapper([=] { CmdBufferAndProcessDone(ctx); },
                                       ctx->name);

    gpuDevice->getMemMgr()->readRequest(ctx->cmd_buf_addr, ctx->cmd_buffer,
                                        PSP_CMD_BUFFER_MAX_SIZE, 0,
                                        cb); // Blocking functional read
}
void
AMDGPUNbio::CmdBufferAndProcessDone(PspCommandContext *ctx)
{
    // 3. Process Command
    DPRINTF(AMDGPUDevice, "PSP: Processing command buffer\n");
    DPRINTF(AMDGPUDevice, "PSP: Command buffer contents (first 64 bytes):\n");
    for (int i = 0; i < 64; i += 4) {
        uint32_t val = *(uint32_t *)(ctx->cmd_buffer + i);
        DPRINTF(AMDGPUDevice, "  Offset %#x: %#x\n", i, val);
    }

    uint32_t cmd_id = ((PspGfxCmdResp*)ctx->cmd_buffer)->cmd_id;
    DPRINTF(AMDGPUDevice, "PSP: Processing Command ID %#x\n", cmd_id);

    switch (cmd_id) {
        case GFX_CMD_ID_LOAD_TA: {
            PspGfxCmdLoadTa *load_ta =
                (PspGfxCmdLoadTa *)(ctx->cmd_buffer + PSP_CMD_PAYLOAD_OFFSET);

            ta_cmd_buf_addr = (Addr)load_ta->cmd_buf_phys_addr_hi << 32 |
                              load_ta->cmd_buf_phys_addr_lo;

            DPRINTF(
                AMDGPUDevice,
                "PSP: GFX_CMD_ID_LOAD_TA processed. TA Cmd Buf Addr: % #lx\n ",
                ta_cmd_buf_addr);
        } break;

        case GFX_CMD_ID_SRIOV_SPATIAL_PART: {
            PspGfxCmdSriovSpatialPart *spatial_part =
                (PspGfxCmdSriovSpatialPart *)(ctx->cmd_buffer +
                                              PSP_CMD_PAYLOAD_OFFSET);

            ctx->sriov_spatial_mode = spatial_part->mode;

            DPRINTF(
                AMDGPUDevice,
                "PSP: GFX_CMD_ID_SRIOV_SPATIAL_PART processed.Mode: % #x\n ",
                ctx->sriov_spatial_mode);

            // Update the register value that the driver reads back
            // NOTE: This assumes gpuDevice->setRegVal manages the
            // NBIO_PARTITION_COMPUTE_STATUS value
            // need a mapping here of the values as driver compute the
            // values slightly in a different approach
            // so if value is 8 , then modify the value as 4 :
            // similarly if value is 4,
            // then modify the value as 3
            // similary for 2, return 1 and for 1 return 0
            if (ctx->sriov_spatial_mode == 8) {
                ctx->sriov_spatial_mode = 4;
            } else if (ctx->sriov_spatial_mode == 4) {
                ctx->sriov_spatial_mode = 3;
            } else if (ctx->sriov_spatial_mode == 2) {
                ctx->sriov_spatial_mode = 1;
            } else if (ctx->sriov_spatial_mode == 1) {
                ctx->sriov_spatial_mode = 0;
            }
            gpuDevice->setRegVal(NBIO_PARTITION_COMPUTE_STATUS,
                                 (ctx->sriov_spatial_mode << 0x4));
        } break;

        default:
            DPRINTF(AMDGPUDevice,
                    "PSP: Unknown command ID %#x. Doing nothing.\n", cmd_id);
            break;
    }

    // again we have to write the physical fence address
    bool check_addr = gpuDevice->getVM().inMMHUB(ctx->fence_addr);
    if (check_addr) {
        ctx->fence_addr = ctx->fence_addr - gpuDevice->getVM().getMMHUBBase();
    } else {
        DPRINTF(AMDGPUDevice,
                "PSP: Fence address %#lx not in MMHUB aperture. Cannot write "
                "fence.\n",
                ctx->fence_addr);
        return;
    }
    // 4. Write Fence Value (Start the final asynchronous step)
    DPRINTF(AMDGPUDevice, "PSP: Writing fence value %u to address %#lx\n",
            ctx->fence_value, ctx->fence_addr);

    // We must use the value stored in the context struct

    // Schedule the completion callback after a sufficient latency to
    //simulate command execution time
    auto cb = new EventFunctionWrapper(
        [=]{
        fenceWriteDone(ctx); }, ctx->name);

    gpuDevice->getMemMgr()->writeRequest(ctx->fence_addr,
                                         (uint8_t*)&ctx->fence_value,
                                         sizeof(ctx->fence_value),
                                         0,
                                         cb); // Blocking functional write
}

void
AMDGPUNbio::fenceWriteDone(PspCommandContext *ctx)
{
    // The fence write is complete, the driver can now proceed.
    DPRINTF(AMDGPUDevice,
            "PSP: Command processing complete. Fence address %#lx signaled.\n",
            ctx->fence_addr);

    // Cleanup the dynamically allocated context object
    delete ctx;
}

} // namespace gem5
