/*
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
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

#include "gpu-compute/blit_emulation.hh"

#include "arch/amdgpu/vega/pagetable_walker.hh"
#include "dev/amdgpu/amdgpu_device.hh"
#include "gpu-compute/gpu_command_processor.hh"
#include "gpu-compute/hsa_queue_entry.hh"
#include "mem/abstract_mem.hh"

namespace gem5
{

void
GPUCommandProcessor::readBlitKernargs(HSAQueueEntry *task)
{
    /**
     * In full system mode, the page table entry may point to a system
     * page or a device page. System pages use the proxy as normal, but
     * a device page needs to be read from device memory. Check what type
     * it is here.
     */
    bool is_system_page = true;
    Addr phys_addr = task->kernargAddr();

    /**
     * Full system currently only supports running on single VMID (one
     * virtual memory space), i.e., one application running on GPU at a
     * time. Because of this, for now we know the VMID is always 1. Later
     * the VMID would have to be passed on to the command processor.
     */
    int vmid = 1;
    unsigned tmp_bytes;
    walker->startFunctional(gpuDevice->getVM().getPageTableBase(vmid),
                            phys_addr, tmp_bytes, BaseMMU::Mode::Read,
                            is_system_page);

    DPRINTF(GPUCommandProc, "Blit kernarg data is in %s memory\n",
            is_system_page ? "host" : "device");

    /* Copy the max amount -- Unused kernargs *should* be zeros. */
    unsigned kernarg_size = maxBlitKernargs * sizeof(uint32_t);
    uint32_t *blit_kernargs = new uint32_t[kernarg_size];

    if (is_system_page) {
        auto cb = new DmaVirtCallback<uint32_t>([=](const uint32_t &) {
            processBlitKernargs(task, blit_kernargs);
        });

        dmaReadVirt(task->kernargAddr(), kernarg_size, cb, blit_kernargs);
    } else {
        // Read from GPU memory manager one cache line at a time to prevent
        // rare cases where the preload data spans two memory pages.
        constexpr unsigned alignment_granularity = 64;
        ChunkGenerator gen(task->kernargAddr(), kernarg_size,
                           alignment_granularity);

        for (; !gen.done(); gen.next()) {
            Addr chunk_addr = gen.addr();
            int vmid = 1;
            unsigned dummy;
            walker->startFunctional(gpuDevice->getVM().getPageTableBase(vmid),
                                    chunk_addr, dummy, BaseMMU::Mode::Read,
                                    is_system_page);

            Request::Flags flags = Request::PHYSICAL;
            RequestPtr request =
                std::make_shared<Request>(chunk_addr, alignment_granularity,
                                          flags, walker->getDevRequestor());

            PacketPtr readPkt = new Packet(request, MemCmd::ReadReq);
            readPkt->dataStatic(blit_kernargs + gen.complete());

            assert(system()->getDeviceMemory(readPkt) != nullptr);
            system()->getDeviceMemory(readPkt)->access(readPkt);
            delete readPkt;
        }

        processBlitKernargs(task, blit_kernargs);
    }
}

void
GPUCommandProcessor::processBlitKernargs(HSAQueueEntry *task, uint32_t *args)
{
    if (debug::GPUCommandProc) {
        for (int i = 0; i < maxBlitKernargs; ++i) {
            DPRINTF(GPUCommandProc, "Blit kernarg[%d] = %#x\n", i, args[i]);
        }
    }

    // There are 3 types of blit kernels: Copy Aligned, Copy Misaligned, and
    // Fill. Typically we see Copy Aligned so implement that first. If we
    // see the other types then we should run the blit kernel. Copy Misaligned
    // has 13 SGPRs and Fill has 8 SGPRs. We use the first kernarg that is
    // zero to detect the type.
    if (args[8] == 0) {
        DPRINTF(GPUCommandProc, "Fill blit detected -- Skipping emulation\n");

        delete args;
        initABI(task);

        return;
    } else if (args[13] == 0) {
        DPRINTF(GPUCommandProc, "Copy Misaligned blit detected -- Skipping "
                                "emulation\n");

        delete args;
        initABI(task);

        return;
    } else {
        DPRINTF(GPUCommandProc,
                "Emulating Copy Aligned blit kernel %i "
                "(Task ID: %i)\n",
                dynamic_task_id - non_blit_kernel_id, dynamic_task_id);

        copyAligned(task, args);
    }
}

void
GPUCommandProcessor::completeBlit(HSAQueueEntry *task)
{
    // Notify the HSA PP that this kernel is complete
    hsaPacketProc().finishPkt(task->dispPktPtr(), task->queueId());
    assert(task->completionSignal());

    DPRINTF(GPUCommandProc,
            "Blit emulation complete with completion "
            "signal! Addr: %d\n",
            task->completionSignal());

    sendCompletionSignal(task->completionSignal());
}

void
GPUCommandProcessor::blitRead(BlitCopyDesc *desc, HSAQueueEntry *task)
{
    DPRINTF(GPUCommandProc, "Read src data for blit phase %d\n", desc->phase);

    if (desc->size[desc->phase] == 0) {
        desc->phase++;
        if (desc->phase == desc->numPhases) {
            DPRINTF(GPUCommandProc, "No blit copy data, completing\n");
            delete desc;
            completeBlit(task);
        } else {
            DPRINTF(GPUCommandProc, "No blit copy data, next phase\n");
            blitRead(desc, task);
        }

        return;
    }

    auto buf_size = desc->size[desc->phase];
    uint8_t *buf = new uint8_t[buf_size];

    auto cb = new DmaVirtCallback<uint32_t>(
        [=](const uint32_t &) { blitWrite(desc, task, buf); });
    dmaReadVirt(desc->srcStart[desc->phase], buf_size, cb, buf);
}

void
GPUCommandProcessor::blitWrite(BlitCopyDesc *desc, HSAQueueEntry *task,
                               uint8_t *buf)
{
    DPRINTF(GPUCommandProc, "Write dest data for blit phase %d\n",
            desc->phase);

    auto buf_size = desc->size[desc->phase];

    bool is_system_page = false;
    Addr base_addr = desc->dstStart[desc->phase];
    int vmid = 1;
    unsigned tmp_bytes;

    // Since this is functional anyways, it is unclear if writing pages at a
    // time is more efficient. Run with this now as it's easier to implement.
    for (int i = 0; i < buf_size; ++i) {
        Addr phys_addr = base_addr + i;
        walker->startFunctional(gpuDevice->getVM().getPageTableBase(vmid),
                                phys_addr, tmp_bytes, BaseMMU::Mode::Read,
                                is_system_page);
        assert(!is_system_page);

        Request::Flags flags = Request::PHYSICAL;
        RequestPtr request = std::make_shared<Request>(
            phys_addr, 1, flags, walker->getDevRequestor());

        PacketPtr writePkt = new Packet(request, MemCmd::WriteReq);
        writePkt->dataStatic(buf + i);

        assert(system()->getDeviceMemory(writePkt) != nullptr);
        system()->getDeviceMemory(writePkt)->access(writePkt);
        delete writePkt;
    }

    blitAck(desc, task, buf);
}

void
GPUCommandProcessor::blitAck(BlitCopyDesc *desc, HSAQueueEntry *task,
                             uint8_t *buf)
{
    DPRINTF(GPUCommandProc, "Write complete for blit phase %d\n", desc->phase);

    delete buf;

    desc->phase++;
    if (desc->phase == desc->numPhases) {
        delete desc;
        completeBlit(task);
    } else {
        blitRead(desc, task);
    }
}

void
GPUCommandProcessor::copyAligned(HSAQueueEntry *task, uint32_t *args)
{
    // Comment from https://github.com/ROCm/ROCR-Runtime/blob/rocm-6.3.3/
    //     runtime/hsa-runtime/core/runtime/amd_blit_kernel.cpp
    //
    // Kernel argument buffer:
    //   [DW  0, 1]  Phase 1 src start address
    //   [DW  2, 3]  Phase 1 dst start address
    //   [DW  4, 5]  Phase 2 src start address
    //   [DW  6, 7]  Phase 2 dst start address
    //   [DW  8, 9]  Phase 3 src start address
    //   [DW 10,11]  Phase 3 dst start address
    //   [DW 12,13]  Phase 4 src start address
    //   [DW 14,15]  Phase 4 dst start address
    //   [DW 16,17]  Phase 4 src end address
    //   [DW 18,19]  Phase 4 dst end address
    //   [DW 20   ]  Total number of workitems
    Addr *as_addr = reinterpret_cast<Addr *>(args);

    Addr p1_src_start = as_addr[0];
    Addr p1_dst_start = as_addr[1];
    Addr p2_src_start = as_addr[2];
    Addr p2_dst_start = as_addr[3];
    Addr p3_src_start = as_addr[4];
    Addr p3_dst_start = as_addr[5];
    Addr p4_src_start = as_addr[6];
    Addr p4_dst_start = as_addr[7];
    Addr p4_src_end = as_addr[8];

    assert(p2_src_start >= p1_src_start);
    assert(p3_src_start >= p2_src_start);
    assert(p4_src_start >= p3_src_start);
    assert(p4_src_end >= p4_src_start);

    BlitCopyDesc *copyDesc = new BlitCopyDesc();

    // Insert backwards so `phase` variable can be used to end blit emulation.
    copyDesc->dstStart[0] = p1_dst_start;
    copyDesc->srcStart[0] = p1_src_start;
    copyDesc->dstStart[1] = p2_dst_start;
    copyDesc->srcStart[1] = p2_src_start;
    copyDesc->dstStart[2] = p3_dst_start;
    copyDesc->srcStart[2] = p3_src_start;
    copyDesc->dstStart[3] = p4_dst_start;
    copyDesc->srcStart[3] = p4_src_start;

    copyDesc->size[0] = p2_src_start - p1_src_start;
    copyDesc->size[1] = p3_src_start - p2_src_start;
    copyDesc->size[2] = p4_src_start - p3_src_start;
    copyDesc->size[3] = p4_src_end - p4_src_start;

    copyDesc->phase = 0;
    copyDesc->numPhases = 4;

    blitRead(copyDesc, task);
}

} // namespace gem5
