/*
 * Copyright (c) 2021 Advanced Micro Devices, Inc.
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
 *
 */

#include "dev/amdgpu/pm4_packet_processor.hh"

#include "debug/PM4PacketProcessor.hh"
#include "dev/amdgpu/amdgpu_device.hh"
#include "dev/amdgpu/hwreg_defines.hh"
#include "dev/amdgpu/interrupt_handler.hh"
#include "dev/amdgpu/pm4_mmio.hh"
#include "dev/amdgpu/sdma_engine.hh"
#include "dev/hsa/hw_scheduler.hh"
#include "enums/GfxVersion.hh"
#include "gpu-compute/gpu_command_processor.hh"
#include "gpu-compute/shader.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"

namespace gem5
{

PM4PacketProcessor::PM4PacketProcessor(const PM4PacketProcessorParams &p)
    : DmaVirtDevice(p)
{
    memset(&kiq, 0, sizeof(QueueDesc));
    memset(&pq, 0, sizeof(QueueDesc));
}

/**
 * AMDGPUDevice will perform DMA operations on VAs, and because
 * page faults are not currently supported for Vega 10, we
 * must be able to find the pages mapped for the process.
 */
TranslationGenPtr
PM4PacketProcessor::translate(Addr vaddr, Addr size)
{
    if (gpuDevice->getVM().inAGP(vaddr)) {
        // Use AGP translation gen
        return TranslationGenPtr(
            new AMDGPUVM::AGPTranslationGen(&gpuDevice->getVM(), vaddr, size));
    }

    // Assume GART otherwise as this is the only other translation aperture
    // available to the PM4 packet processor.
    return TranslationGenPtr(
        new AMDGPUVM::GARTTranslationGen(&gpuDevice->getVM(), vaddr, size));
}

AddrRangeList
PM4PacketProcessor::getAddrRanges() const
{
    AddrRangeList ranges;
    return ranges;
}

void
PM4PacketProcessor::setGPUDevice(AMDGPUDevice *gpu_device)
{
    gpuDevice = gpu_device;
}

Addr
PM4PacketProcessor::getGARTAddr(Addr addr) const
{
    if (!gpuDevice->getVM().inAGP(addr)) {
        Addr low_bits = bits(addr, 11, 0);
        addr = (((addr >> 12) << 3) << 12) | low_bits;
    }
    return addr;
}

PM4Queue *
PM4PacketProcessor::getQueue(Addr offset, bool gfx)
{
    auto result = queuesMap.find(offset);
    if (result == queuesMap.end()) {
        if (gfx)
            mapPq(offset);
        else
            mapKiq(offset);
        return queuesMap[offset];
    }
    return result->second;
}

void
PM4PacketProcessor::mapKiq(Addr offset)
{
    DPRINTF(PM4PacketProcessor, "Mapping KIQ\n");
    newQueue((QueueDesc *)&kiq, offset, &kiq_pkt);
}

void
PM4PacketProcessor::mapPq(Addr offset)
{
    DPRINTF(PM4PacketProcessor, "Mapping PQ\n");
    newQueue((QueueDesc *)&pq, offset, &pq_pkt);
}

void
PM4PacketProcessor::newQueue(QueueDesc *mqd, Addr offset,
                             PM4MapQueues *pkt, int id)
{
    if (id == -1)
        id = queues.size();

    /* 256 bytes aligned address */
    mqd->base <<= 8;
    PM4Queue *q = new PM4Queue(id, mqd, offset, pkt);

    queuesMap[offset] = q;
    queues[id] = q;

    /* we are assumming only compute queues can be map from MQDs */
    QueueType qt;
    qt = mqd->aql ? QueueType::ComputeAQL
                  : QueueType::Compute;
    gpuDevice->setDoorbellType(offset, qt);

    DPRINTF(PM4PacketProcessor, "New PM4 queue %d, base: %p offset: %p, me: "
            "%d, pipe %d queue: %d size: %d\n", id, q->base(), q->offset(),
            q->me(), q->pipe(), q->queue(), q->size());
}

void
PM4PacketProcessor::process(PM4Queue *q, Addr wptrOffset)
{
    q->wptr(wptrOffset * sizeof(uint32_t));

    if (!q->processing()) {
        q->processing(true);
        decodeNext(q);
    }
}

void
PM4PacketProcessor::decodeNext(PM4Queue *q)
{
    DPRINTF(PM4PacketProcessor, "PM4 decode queue %d rptr %p, wptr %p\n",
            q->id(), q->rptr(), q->wptr());

    if (q->rptr() != q->wptr()) {
        /* Additional braces here are needed due to a clang compilation bug
           falsely throwing a "suggest braces around initialization of
           subject" error. More info on this bug is available here:
           https://stackoverflow.com/questions/31555584
         */
        PM4Header h{{{0, 0, 0, 0, 0, 0}}};
        auto cb = new DmaVirtCallback<PM4Header>(
            [ = ] (PM4Header header)
                { decodeHeader(q, header); }, h);
        dmaReadVirt(getGARTAddr(q->rptr()), sizeof(uint32_t), cb,
                    &cb->dmaBuffer);
    } else {
        // Reached the end of processable data in the queue. Switch out of IB
        // if this is an indirect buffer.
        assert(q->rptr() == q->wptr());
        q->processing(false);
        if (q->ib()) {
            q->ib(false);
            decodeNext(q);
        }

        // Write back rptr when the queue is empty. For static queues which
        // are not unmapped, this is how the driver knows there is enough
        // space in the queue to continue writing packets to the ring buffer.
        if (q->getMQD()->aqlRptr) {
            Addr addr = getGARTAddr(q->getMQD()->aqlRptr);
            uint32_t *data = new uint32_t;
            // gem5 stores rptr as a bytes offset while the driver expects
            // a dword offset. Convert the offset to dword count.
            *data = q->getRptr() >> 2;
            auto cb = new DmaVirtCallback<uint32_t>(
                [data](const uint32_t &) { delete data; });
            dmaWriteVirt(addr, sizeof(uint32_t), cb, data);
        }
    }
}

void
PM4PacketProcessor::decodeHeader(PM4Queue *q, PM4Header header)
{
    DPRINTF(PM4PacketProcessor, "PM4 packet %p\n", header.opcode);

    q->incRptr(sizeof(PM4Header));

    DmaVirtCallback<uint64_t> *cb = nullptr;
    void *dmaBuffer = nullptr;

    switch(header.opcode) {
      case IT_NOP: {
        DPRINTF(PM4PacketProcessor, "PM4 nop, count %p\n", header.count);
        DPRINTF(PM4PacketProcessor, "rptr %p wptr %p\n", q->rptr(), q->wptr());
        if (header.count != 0x3fff) {
            q->incRptr((header.count + 1) * sizeof(uint32_t));
        }
        decodeNext(q);
        } break;
      case IT_WRITE_DATA: {
        dmaBuffer = new PM4WriteData();
        cb = new DmaVirtCallback<uint64_t>(
            [ = ] (const uint64_t &)
                { writeData(q, (PM4WriteData *)dmaBuffer); });
        dmaReadVirt(getGARTAddr(q->rptr()), sizeof(PM4WriteData), cb,
                    dmaBuffer);
        } break;

      case IT_MAP_QUEUES: {
        dmaBuffer = new PM4MapQueues();
        cb = new DmaVirtCallback<uint64_t>(
            [ = ] (const uint64_t &)
                { mapQueues(q, (PM4MapQueues *)dmaBuffer); });
        dmaReadVirt(getGARTAddr(q->rptr()), sizeof(PM4MapQueues), cb,
                    dmaBuffer);
        } break;

      case IT_RELEASE_MEM: {
        dmaBuffer = new PM4ReleaseMem();
        cb = new DmaVirtCallback<uint64_t>(
            [ = ] (const uint64_t &)
                { releaseMem(q, (PM4ReleaseMem *)dmaBuffer); });
        dmaReadVirt(getGARTAddr(q->rptr()), sizeof(PM4ReleaseMem), cb,
                    dmaBuffer);
        } break;

      case IT_INDIRECT_BUFFER: {
        dmaBuffer = new PM4IndirectBuf();
        cb = new DmaVirtCallback<uint64_t>(
            [ = ] (const uint64_t &)
                { indirectBuffer(q, (PM4IndirectBuf *)dmaBuffer); });
        dmaReadVirt(getGARTAddr(q->rptr()), sizeof(PM4IndirectBuf), cb,
                    dmaBuffer);
        } break;

      case IT_SWITCH_BUFFER: {
        dmaBuffer = new PM4SwitchBuf();
        cb = new DmaVirtCallback<uint64_t>(
            [ = ] (const uint64_t &)
                { switchBuffer(q, (PM4SwitchBuf *)dmaBuffer); });
        dmaReadVirt(getGARTAddr(q->rptr()), sizeof(PM4SwitchBuf), cb,
                    dmaBuffer);
        } break;

      case IT_SET_UCONFIG_REG: {
        dmaBuffer = new PM4SetUconfigReg();
        cb = new DmaVirtCallback<uint64_t>(
            [ = ] (const uint64_t &)
                { setUconfigReg(q, (PM4SetUconfigReg *)dmaBuffer); });
        dmaReadVirt(getGARTAddr(q->rptr()), sizeof(PM4SetUconfigReg), cb,
                    dmaBuffer);
        } break;

      case IT_WAIT_REG_MEM: {
        dmaBuffer = new PM4WaitRegMem();
        cb = new DmaVirtCallback<uint64_t>(
            [ = ] (const uint64_t &)
                { waitRegMem(q, (PM4WaitRegMem *)dmaBuffer); });
        dmaReadVirt(getGARTAddr(q->rptr()), sizeof(PM4WaitRegMem), cb,
                    dmaBuffer);
        } break;
      case IT_MAP_PROCESS: {
        if (gpuDevice->getGfxVersion() == GfxVersion::gfx90a) {
            dmaBuffer = new PM4MapProcessMI200();
            cb = new DmaVirtCallback<uint64_t>(
                [ = ] (const uint64_t &)
                    { mapProcessGfx90a(q, (PM4MapProcessMI200 *)dmaBuffer); });
            dmaReadVirt(getGARTAddr(q->rptr()), sizeof(PM4MapProcessMI200),
                        cb, dmaBuffer);
        } else {
            dmaBuffer = new PM4MapProcess();
            cb = new DmaVirtCallback<uint64_t>(
                [ = ] (const uint64_t &)
                    { mapProcessGfx9(q, (PM4MapProcess *)dmaBuffer); });
            dmaReadVirt(getGARTAddr(q->rptr()), sizeof(PM4MapProcess), cb,
                        dmaBuffer);
        }
        } break;

      case IT_UNMAP_QUEUES: {
        dmaBuffer = new PM4UnmapQueues();
        cb = new DmaVirtCallback<uint64_t>(
            [ = ] (const uint64_t &)
                { unmapQueues(q, (PM4UnmapQueues *)dmaBuffer); });
        dmaReadVirt(getGARTAddr(q->rptr()), sizeof(PM4UnmapQueues), cb,
                    dmaBuffer);
        } break;

      case IT_RUN_LIST: {
        dmaBuffer = new PM4RunList();
        cb = new DmaVirtCallback<uint64_t>(
            [ = ] (const uint64_t &)
                { runList(q, (PM4RunList *)dmaBuffer); });
        dmaReadVirt(getGARTAddr(q->rptr()), sizeof(PM4RunList), cb,
                    dmaBuffer);
        } break;

      case IT_QUERY_STATUS: {
        dmaBuffer = new PM4QueryStatus();
        cb = new DmaVirtCallback<uint64_t>(
            [ = ] (const uint64_t &)
                { queryStatus(q, (PM4QueryStatus *)dmaBuffer); });
        dmaReadVirt(getGARTAddr(q->rptr()), sizeof(PM4QueryStatus), cb,
                    dmaBuffer);
        } break;

      case IT_INVALIDATE_TLBS: {
        DPRINTF(PM4PacketProcessor, "Functionaly invalidating all TLBs\n");
        gpuDevice->getVM().invalidateTLBs();
        q->incRptr((header.count + 1) * sizeof(uint32_t));
        decodeNext(q);
        } break;

      default: {
        warn("PM4 packet opcode 0x%x not supported.\n", header.opcode);
        DPRINTF(PM4PacketProcessor, "PM4 packet opcode 0x%x not supported.\n",
                header.opcode);
        q->incRptr((header.count + 1) * sizeof(uint32_t));
        decodeNext(q);
        } break;
    }
}

void
PM4PacketProcessor::writeData(PM4Queue *q, PM4WriteData *pkt)
{
    q->incRptr(sizeof(PM4WriteData));

    Addr addr = getGARTAddr(pkt->destAddr);
    DPRINTF(PM4PacketProcessor, "PM4 write addr: %p data: %p.\n", addr,
            pkt->data);
    auto cb = new DmaVirtCallback<uint32_t>(
        [ = ](const uint32_t &) { writeDataDone(q, pkt, addr); });
    //TODO: the specs indicate that pkt->data holds the number of dword that
    //need to be written.
    dmaWriteVirt(addr, sizeof(uint32_t), cb, &pkt->data);

    if (!pkt->writeConfirm)
        decodeNext(q);
}

void
PM4PacketProcessor::writeDataDone(PM4Queue *q, PM4WriteData *pkt, Addr addr)
{
    DPRINTF(PM4PacketProcessor, "PM4 write completed to %p, %p.\n", addr,
            pkt->data);

    if (pkt->writeConfirm)
        decodeNext(q);

    delete pkt;
}

void
PM4PacketProcessor::mapQueues(PM4Queue *q, PM4MapQueues *pkt)
{
    q->incRptr(sizeof(PM4MapQueues));

    DPRINTF(PM4PacketProcessor, "MAPQueues queueSel: %d, vmid: %d, me: %d, "
            "pipe: %d, queueSlot: %d, queueType: %d, allocFormat: %d, "
            "engineSel: %d, numQueues: %d, checkDisable: %d, doorbellOffset:"
            " %d, mqdAddr: %lx, wptrAddr: %lx\n", pkt->queueSel, pkt->vmid,
            pkt->me, pkt->pipe, pkt->queueSlot, pkt->queueType,
            pkt->allocFormat, pkt->engineSel, pkt->numQueues,
            pkt->checkDisable, pkt->doorbellOffset, pkt->mqdAddr,
            pkt->wptrAddr);

    // Partially reading the mqd with an offset of 96 dwords
    if (pkt->engineSel == 0 || pkt->engineSel == 1 || pkt->engineSel == 4) {
        Addr addr = getGARTAddr(pkt->mqdAddr + 96 * sizeof(uint32_t));

        DPRINTF(PM4PacketProcessor,
                "Mapping mqd from %p %p (vmid %d - last vmid %d).\n",
                addr, pkt->mqdAddr, pkt->vmid, gpuDevice->lastVMID());

        // The doorbellOffset is a dword address. We shift by two / multiply
        // by four to get the byte address to match doorbell addresses in
        // the GPU device.
        gpuDevice->mapDoorbellToVMID(pkt->doorbellOffset << 2,
                                     gpuDevice->lastVMID());

        QueueDesc *mqd = new QueueDesc();
        memset(mqd, 0, sizeof(QueueDesc));
        auto cb = new DmaVirtCallback<uint32_t>(
            [ = ] (const uint32_t &) {
                processMQD(pkt, q, addr, mqd, gpuDevice->lastVMID()); });
        dmaReadVirt(addr, sizeof(QueueDesc), cb, mqd);
    } else if (pkt->engineSel == 2 || pkt->engineSel == 3) {
        SDMAQueueDesc *sdmaMQD = new SDMAQueueDesc();
        memset(sdmaMQD, 0, sizeof(SDMAQueueDesc));

        // For SDMA we read the full MQD, so there is no offset calculation.
        Addr addr = getGARTAddr(pkt->mqdAddr);

        auto cb = new DmaVirtCallback<uint32_t>(
            [ = ] (const uint32_t &) {
                processSDMAMQD(pkt, q, addr, sdmaMQD,
                               gpuDevice->lastVMID()); });
        dmaReadVirt(addr, sizeof(SDMAQueueDesc), cb, sdmaMQD);
    } else {
        panic("Unknown engine for MQD: %d\n", pkt->engineSel);
    }

    decodeNext(q);
}

void
PM4PacketProcessor::processMQD(PM4MapQueues *pkt, PM4Queue *q, Addr addr,
    QueueDesc *mqd, uint16_t vmid)
{
    DPRINTF(PM4PacketProcessor, "MQDbase: %lx, active: %d, vmid: %d, base: "
            "%lx, rptr: %x aqlPtr: %lx\n", mqd->mqdBase, mqd->hqd_active,
            mqd->hqd_vmid, mqd->base, mqd->rptr, mqd->aqlRptr);

    Addr offset = mqd->doorbell & 0x1ffffffc;
    newQueue(mqd, offset, pkt);
    PM4Queue *new_q = queuesMap[offset];
    gpuDevice->insertQId(vmid, new_q->id());

    if (mqd->aql) {
        // The queue size is encoded in the cp_hqd_pq_control field in the
        // kernel driver in the 6 lowest bits as log2(queue_size / 4) - 1
        // number of dwords.
        //
        //      https://github.com/RadeonOpenCompute/ROCK-Kernel-Driver/blob/
        //          roc-4.3.x/drivers/gpu/drm/amd/amdgpu/gfx_v9_0.c#L3561
        //
        // Queue size is then 2^(cp_hqd_pq_control[5:0] + 1) dword. Multiply
        // by 4 to get the number of bytes as HSAPP expects.
        int mqd_size = (1 << ((mqd->hqd_pq_control & 0x3f) + 1)) * 4;
        auto &hsa_pp = gpuDevice->CP()->hsaPacketProc();
        hsa_pp.setDeviceQueueDesc(mqd->aqlRptr, mqd->base, new_q->id(),
                                  mqd_size, 8, GfxVersion::gfx900, offset,
                                  mqd->mqdReadIndex);
    }

    DPRINTF(PM4PacketProcessor, "PM4 mqd read completed, base %p, mqd %p, "
            "hqdAQL %d.\n", mqd->base, mqd->mqdBase, mqd->aql);

    gpuDevice->processPendingDoorbells(offset);
}

void
PM4PacketProcessor::processSDMAMQD(PM4MapQueues *pkt, PM4Queue *q, Addr addr,
    SDMAQueueDesc *mqd, uint16_t vmid)
{
    uint32_t rlc_size = 4UL << bits(mqd->sdmax_rlcx_rb_cntl, 6, 1);
    Addr rptr_wb_addr = mqd->sdmax_rlcx_rb_rptr_addr_hi;
    rptr_wb_addr <<= 32;
    rptr_wb_addr |= mqd->sdmax_rlcx_rb_rptr_addr_lo;

    DPRINTF(PM4PacketProcessor, "SDMAMQD: rb base: %#lx rptr: %#x/%#x wptr: "
            "%#x/%#x ib: %#x/%#x size: %d ctrl: %#x rptr wb addr: %#lx\n",
            mqd->rb_base, mqd->sdmax_rlcx_rb_rptr, mqd->sdmax_rlcx_rb_rptr_hi,
            mqd->sdmax_rlcx_rb_wptr, mqd->sdmax_rlcx_rb_wptr_hi,
            mqd->sdmax_rlcx_ib_base_lo, mqd->sdmax_rlcx_ib_base_hi,
            rlc_size, mqd->sdmax_rlcx_rb_cntl, rptr_wb_addr);

    // Engine 2 points to SDMA0 while engine 3 points to SDMA1
    assert(pkt->engineSel == 2 || pkt->engineSel == 3);
    SDMAEngine *sdma_eng = gpuDevice->getSDMAById(pkt->engineSel - 2);

    // Register RLC queue with SDMA
    sdma_eng->registerRLCQueue(pkt->doorbellOffset << 2, addr, mqd);

    // Register doorbell with GPU device
    gpuDevice->setSDMAEngine(pkt->doorbellOffset << 2, sdma_eng);
    gpuDevice->setDoorbellType(pkt->doorbellOffset << 2, RLC);

    gpuDevice->processPendingDoorbells(pkt->doorbellOffset << 2);
}

void
PM4PacketProcessor::releaseMem(PM4Queue *q, PM4ReleaseMem *pkt)
{
    q->incRptr(sizeof(PM4ReleaseMem));

    Addr addr = getGARTAddr(pkt->addr);
    DPRINTF(PM4PacketProcessor, "PM4 release_mem event %d eventIdx %d intSel "
            "%d destSel %d dataSel %d, address %p data %p, intCtx %p\n",
            pkt->event, pkt->eventIdx, pkt->intSelect, pkt->destSelect,
            pkt->dataSelect, addr, pkt->dataLo, pkt->intCtxId);

    DPRINTF(PM4PacketProcessor,
            "PM4 release_mem destSel 0 bypasses caches to MC.\n");

    if (pkt->dataSelect == 1) {
        auto cb = new DmaVirtCallback<uint32_t>(
            [ = ](const uint32_t &) { releaseMemDone(q, pkt, addr); },
            pkt->dataLo);
        dmaWriteVirt(addr, sizeof(uint32_t), cb, &cb->dmaBuffer);
    } else {
        panic("Unimplemented PM4ReleaseMem.dataSelect");
    }
}

void
PM4PacketProcessor::releaseMemDone(PM4Queue *q, PM4ReleaseMem *pkt, Addr addr)
{
    DPRINTF(PM4PacketProcessor, "PM4 release_mem wrote %d to %p\n",
            pkt->dataLo, addr);
    if (pkt->intSelect == 2) {
        DPRINTF(PM4PacketProcessor, "PM4 interrupt, id: %d ctx: %d, me: %d, "
                "pipe: %d, queueSlot:%d\n", q->id(), pkt->intCtxId, q->me(),
                q->pipe(), q->queue());

        uint8_t ringId = 0;
        if (q->id() != 0) {
            ringId = (q->queue() << 4) | (q->me() << 2) | q->pipe();
        }
        gpuDevice->getIH()->prepareInterruptCookie(pkt->intCtxId, ringId,
                                            SOC15_IH_CLIENTID_GRBM_CP, CP_EOP);
        gpuDevice->getIH()->submitInterruptCookie();
    }

    delete pkt;
    decodeNext(q);
}

void
PM4PacketProcessor::updateReadIndex(Addr offset, uint64_t rd_idx)
{
    assert(queuesMap.count(offset));
    queuesMap[offset]->getMQD()->mqdReadIndex = rd_idx;
}

void
PM4PacketProcessor::unmapQueues(PM4Queue *q, PM4UnmapQueues *pkt)
{
    q->incRptr(sizeof(PM4UnmapQueues));

    DPRINTF(PM4PacketProcessor, "PM4 unmap_queues queueSel: %d numQueues: %d "
            "pasid: %p doorbellOffset0 %p \n",
            pkt->queueSel, pkt->numQueues, pkt->pasid, pkt->doorbellOffset0);

    switch (pkt->queueSel) {
      case 0:
        switch (pkt->numQueues) {
          case 1:
            gpuDevice->deallocateVmid(
                    gpuDevice->getVMID(pkt->doorbellOffset0));
            gpuDevice->deallocateVmid(
                    gpuDevice->getVMID(pkt->doorbellOffset1));
            gpuDevice->deallocateVmid(
                    gpuDevice->getVMID(pkt->doorbellOffset2));
            gpuDevice->deallocateVmid(
                    gpuDevice->getVMID(pkt->doorbellOffset3));
            break;
          case 2:
            gpuDevice->deallocateVmid(
                    gpuDevice->getVMID(pkt->doorbellOffset1));
            gpuDevice->deallocateVmid(
                    gpuDevice->getVMID(pkt->doorbellOffset2));
            gpuDevice->deallocateVmid(
                    gpuDevice->getVMID(pkt->doorbellOffset3));
            break;
          case 3:
            gpuDevice->deallocateVmid(
                    gpuDevice->getVMID(pkt->doorbellOffset2));
            gpuDevice->deallocateVmid(
                    gpuDevice->getVMID(pkt->doorbellOffset3));
            break;
          case 4:
            gpuDevice->deallocateVmid(
                    gpuDevice->getVMID(pkt->doorbellOffset3));
            break;
          default:
            panic("Unrecognized number of queues %d\n", pkt->numQueues);
        }
        break;
      case 1:
        gpuDevice->deallocatePasid(pkt->pasid);
        break;
      case 2:
        panic("Unmapping queue selection 2 unimplemented\n");
        break;
      case 3: {
        auto &hsa_pp = gpuDevice->CP()->hsaPacketProc();
        for (auto iter : gpuDevice->getUsedVMIDs()) {
            for (auto id : iter.second) {
                assert(queues.count(id));

                // Do not unmap KMD queues
                if (queues[id]->privileged()) {
                    continue;
                }
                QueueDesc *mqd = queues[id]->getMQD();
                DPRINTF(PM4PacketProcessor, "Unmapping queue %d with read "
                        "index %ld\n", id, mqd->mqdReadIndex);
                // Partially writing the mqd with an offset of 96 dwords
                Addr addr = getGARTAddr(queues[id]->mqdBase() +
                                        96 * sizeof(uint32_t));
                Addr mqd_base = queues[id]->mqdBase();
                auto cb = new DmaVirtCallback<uint32_t>(
                    [ = ] (const uint32_t &) {
                        doneMQDWrite(mqd_base, addr);
                    });
                mqd->base >>= 8;
                dmaWriteVirt(addr, sizeof(QueueDesc), cb, mqd);
                queues.erase(id);
                hsa_pp.unsetDeviceQueueDesc(id, 8);
            }
        }
        gpuDevice->deallocateAllQueues();
      } break;
      default:
        panic("Unrecognized options\n");
        break;
    }

    delete pkt;
    decodeNext(q);
}

void
PM4PacketProcessor::doneMQDWrite(Addr mqdAddr, Addr addr) {
    DPRINTF(PM4PacketProcessor, "PM4 unmap_queues MQD %p wrote to addr %p\n",
            mqdAddr, addr);
}

void
PM4PacketProcessor::mapProcess(uint32_t pasid, uint64_t ptBase,
                               uint32_t shMemBases)
{
    uint16_t vmid = gpuDevice->allocateVMID(pasid);

    gpuDevice->getVM().setPageTableBase(vmid, ptBase);
    gpuDevice->CP()->shader()->setHwReg(HW_REG_SH_MEM_BASES, shMemBases);

    // Setup the apertures that gem5 uses. These values are bits [63:48].
    Addr lds_base = (Addr)bits(shMemBases, 31, 16) << 48;
    Addr scratch_base = (Addr)bits(shMemBases, 15, 0) << 48;

    // There does not seem to be any register for the limit, but the driver
    // assumes scratch and LDS have a 4GB aperture, so use that.
    gpuDevice->CP()->shader()->setLdsApe(lds_base, lds_base + 0xFFFFFFFF);
    gpuDevice->CP()->shader()->setScratchApe(scratch_base,
                                             scratch_base + 0xFFFFFFFF);
}

void
PM4PacketProcessor::mapProcessGfx9(PM4Queue *q, PM4MapProcess *pkt)
{
    q->incRptr(sizeof(PM4MapProcess));

    DPRINTF(PM4PacketProcessor, "PM4 map_process pasid: %p quantum: "
            "%d pt: %p signal: %p\n", pkt->pasid, pkt->processQuantum,
            pkt->ptBase, pkt->completionSignal);

    mapProcess(pkt->pasid, pkt->ptBase, pkt->shMemBases);

    delete pkt;
    decodeNext(q);
}

void
PM4PacketProcessor::mapProcessGfx90a(PM4Queue *q, PM4MapProcessMI200 *pkt)
{
    q->incRptr(sizeof(PM4MapProcessMI200));

    DPRINTF(PM4PacketProcessor, "PM4 map_process pasid: %p quantum: "
            "%d pt: %p signal: %p\n", pkt->pasid, pkt->processQuantum,
            pkt->ptBase, pkt->completionSignal);

    mapProcess(pkt->pasid, pkt->ptBase, pkt->shMemBases);

    delete pkt;
    decodeNext(q);
}

void
PM4PacketProcessor::runList(PM4Queue *q, PM4RunList *pkt)
{
    DPRINTF(PM4PacketProcessor, "PM4 run_list base: %p size: %d\n",
            pkt->ibBase, pkt->ibSize);

    q->incRptr(sizeof(PM4RunList));

    q->ib(true);
    q->ibBase(pkt->ibBase);
    q->rptr(0);
    q->wptr(pkt->ibSize * sizeof(uint32_t));

    delete pkt;
    decodeNext(q);
}

void
PM4PacketProcessor::indirectBuffer(PM4Queue *q, PM4IndirectBuf *pkt)
{
    DPRINTF(PM4PacketProcessor, "PM4 indirect buffer, base: %p.\n",
            pkt->ibBase);

    q->incRptr(sizeof(PM4IndirectBuf));

    q->ib(true);
    q->ibBase(pkt->ibBase);
    q->wptr(pkt->ibSize * sizeof(uint32_t));

    decodeNext(q);
}

void
PM4PacketProcessor::switchBuffer(PM4Queue *q, PM4SwitchBuf *pkt)
{
    q->incRptr(sizeof(PM4SwitchBuf));

    q->ib(true);
    DPRINTF(PM4PacketProcessor, "PM4 switching buffer, rptr: %p.\n",
            q->wptr());

    decodeNext(q);
}

void
PM4PacketProcessor::setUconfigReg(PM4Queue *q, PM4SetUconfigReg *pkt)
{
    q->incRptr(sizeof(PM4SetUconfigReg));

    // SET_UCONFIG_REG_START and pkt->offset are dword addresses
    uint32_t reg_addr = (PACKET3_SET_UCONFIG_REG_START + pkt->offset) * 4;

    gpuDevice->setRegVal(reg_addr, pkt->data);

    decodeNext(q);
}

void
PM4PacketProcessor::waitRegMem(PM4Queue *q, PM4WaitRegMem *pkt)
{
    q->incRptr(sizeof(PM4WaitRegMem));

    DPRINTF(PM4PacketProcessor, "PM4 WAIT_REG_MEM\nfunc: %d memSpace: %d op: "
            "%d\n", pkt->function, pkt->memSpace, pkt->operation);
    DPRINTF(PM4PacketProcessor, "    AddrLo/Reg1: %lx\n", pkt->memAddrLo);
    DPRINTF(PM4PacketProcessor, "    AddrHi/Reg2: %lx\n", pkt->memAddrHi);
    DPRINTF(PM4PacketProcessor, "    Reference: %lx\n", pkt->reference);
    DPRINTF(PM4PacketProcessor, "    Mask: %lx\n", pkt->mask);
    DPRINTF(PM4PacketProcessor, "    Poll Interval: %lx\n", pkt->pollInterval);

    decodeNext(q);
}

void
PM4PacketProcessor::queryStatus(PM4Queue *q, PM4QueryStatus *pkt)
{
    q->incRptr(sizeof(PM4QueryStatus));

    DPRINTF(PM4PacketProcessor, "PM4 query status contextId: %d, interruptSel:"
            " %d command: %d, pasid: %d, doorbellOffset: %d, engineSel: %d "
            "addr: %lx, data: %lx\n", pkt->contextId, pkt->interruptSel,
            pkt->command, pkt->pasid, pkt->doorbellOffset, pkt->engineSel,
            pkt->addr, pkt->data);

    if (pkt->interruptSel == 0 && pkt->command == 2) {
        // Write data value to fence address
        Addr addr = getGARTAddr(pkt->addr);
        DPRINTF(PM4PacketProcessor, "Using GART addr %lx\n", addr);
        auto cb = new DmaVirtCallback<uint64_t>(
            [ = ] (const uint64_t &) { queryStatusDone(q, pkt); }, pkt->data);
        dmaWriteVirt(addr, sizeof(uint64_t), cb, &cb->dmaBuffer);
    } else {
        // No other combinations used in amdkfd v9
        panic("query_status with interruptSel %d command %d not supported",
              pkt->interruptSel, pkt->command);
    }
}

void
PM4PacketProcessor::queryStatusDone(PM4Queue *q, PM4QueryStatus *pkt)
{
    DPRINTF(PM4PacketProcessor, "PM4 query status complete\n");

    delete pkt;
    decodeNext(q);
}

void
PM4PacketProcessor::writeMMIO(PacketPtr pkt, Addr mmio_offset)
{
    switch (mmio_offset) {
      /* Hardware queue descriptor (HQD) registers */
      case mmCP_HQD_VMID:
        setHqdVmid(pkt->getLE<uint32_t>());
        break;
      case mmCP_HQD_ACTIVE:
        setHqdActive(pkt->getLE<uint32_t>());
        break;
      case mmCP_HQD_PQ_BASE:
        setHqdPqBase(pkt->getLE<uint32_t>());
        break;
      case mmCP_HQD_PQ_BASE_HI:
        setHqdPqBaseHi(pkt->getLE<uint32_t>());
        break;
      case mmCP_HQD_PQ_DOORBELL_CONTROL:
        setHqdPqDoorbellCtrl(pkt->getLE<uint32_t>());
        gpuDevice->setDoorbellType(getKiqDoorbellOffset(), Compute);
        break;
      case mmCP_HQD_PQ_RPTR:
        setHqdPqPtr(pkt->getLE<uint32_t>());
        break;
      case mmCP_HQD_PQ_WPTR_LO:
        setHqdPqWptrLo(pkt->getLE<uint32_t>());
        break;
      case mmCP_HQD_PQ_WPTR_HI:
        setHqdPqWptrHi(pkt->getLE<uint32_t>());
        break;
      case mmCP_HQD_PQ_RPTR_REPORT_ADDR:
        setHqdPqRptrReportAddr(pkt->getLE<uint32_t>());
        break;
      case mmCP_HQD_PQ_RPTR_REPORT_ADDR_HI:
        setHqdPqRptrReportAddrHi(pkt->getLE<uint32_t>());
        break;
      case mmCP_HQD_PQ_WPTR_POLL_ADDR:
        setHqdPqWptrPollAddr(pkt->getLE<uint32_t>());
        break;
      case mmCP_HQD_PQ_WPTR_POLL_ADDR_HI:
        setHqdPqWptrPollAddrHi(pkt->getLE<uint32_t>());
        break;
      case mmCP_HQD_PQ_CONTROL:
        setHqdPqControl(pkt->getLE<uint32_t>());
        break;
      case mmCP_HQD_IB_CONTROL:
        setHqdIbCtrl(pkt->getLE<uint32_t>());
        break;
      /* Ring buffer registers */
      case mmCP_RB_VMID:
        setRbVmid(pkt->getLE<uint32_t>());
        break;
      case mmCP_RB0_CNTL:
        setRbCntl(pkt->getLE<uint32_t>());
        break;
      case mmCP_RB0_WPTR:
        setRbWptrLo(pkt->getLE<uint32_t>());
        break;
      case mmCP_RB0_WPTR_HI:
        setRbWptrHi(pkt->getLE<uint32_t>());
        break;
      case mmCP_RB0_RPTR_ADDR:
        setRbRptrAddrLo(pkt->getLE<uint32_t>());
        break;
      case mmCP_RB0_RPTR_ADDR_HI:
        setRbRptrAddrHi(pkt->getLE<uint32_t>());
        break;
      case mmCP_RB_WPTR_POLL_ADDR_LO:
        setRbWptrPollAddrLo(pkt->getLE<uint32_t>());
        break;
      case mmCP_RB_WPTR_POLL_ADDR_HI:
        setRbWptrPollAddrHi(pkt->getLE<uint32_t>());
        break;
      case mmCP_RB0_BASE:
        setRbBaseLo(pkt->getLE<uint32_t>());
        break;
      case mmCP_RB0_BASE_HI:
        setRbBaseHi(pkt->getLE<uint32_t>());
        break;
      case mmCP_RB_DOORBELL_CONTROL:
        setRbDoorbellCntrl(pkt->getLE<uint32_t>());
        gpuDevice->setDoorbellType(getPqDoorbellOffset(), Gfx);
        break;
      case mmCP_RB_DOORBELL_RANGE_LOWER:
        setRbDoorbellRangeLo(pkt->getLE<uint32_t>());
        break;
      case mmCP_RB_DOORBELL_RANGE_UPPER:
        setRbDoorbellRangeHi(pkt->getLE<uint32_t>());
        break;
      default:
        break;
    }
}

void
PM4PacketProcessor::setHqdVmid(uint32_t data)
{
    kiq.hqd_vmid = data;
}

void
PM4PacketProcessor::setHqdActive(uint32_t data)
{
    kiq.hqd_active = data;
}

void
PM4PacketProcessor::setHqdPqBase(uint32_t data)
{
    kiq.hqd_pq_base_lo = data;
}

void
PM4PacketProcessor::setHqdPqBaseHi(uint32_t data)
{
    kiq.hqd_pq_base_hi = data;
}

void
PM4PacketProcessor::setHqdPqDoorbellCtrl(uint32_t data)
{
    kiq.hqd_pq_doorbell_control = data;
}

void
PM4PacketProcessor::setHqdPqPtr(uint32_t data)
{
    kiq.rptr = data;
}

void
PM4PacketProcessor::setHqdPqWptrLo(uint32_t data)
{
    /* Write pointer communicated through doorbell value. */
}

void
PM4PacketProcessor::setHqdPqWptrHi(uint32_t data)
{
    /* Write pointer communicated through doorbell value. */
}

void
PM4PacketProcessor::setHqdPqRptrReportAddr(uint32_t data)
{
    kiq.hqd_pq_rptr_report_addr_lo = data;
}

void
PM4PacketProcessor::setHqdPqRptrReportAddrHi(uint32_t data)
{
    kiq.hqd_pq_rptr_report_addr_hi = data;
}

void
PM4PacketProcessor::setHqdPqWptrPollAddr(uint32_t data)
{
    kiq.hqd_pq_wptr_poll_addr_lo = data;
}

void
PM4PacketProcessor::setHqdPqWptrPollAddrHi(uint32_t data)
{
    kiq.hqd_pq_wptr_poll_addr_hi = data;
}

void
PM4PacketProcessor::setHqdPqControl(uint32_t data)
{
    kiq.hqd_pq_control = data;
}

void
PM4PacketProcessor::setHqdIbCtrl(uint32_t data)
{
    kiq.hqd_ib_control = data;
}

void
PM4PacketProcessor::setRbVmid(uint32_t data)
{
    pq.hqd_vmid = data;
}

void
PM4PacketProcessor::setRbCntl(uint32_t data)
{
    pq.hqd_pq_control = data;
}

void
PM4PacketProcessor::setRbWptrLo(uint32_t data)
{
    pq.queueWptrLo = data;
}

void
PM4PacketProcessor::setRbWptrHi(uint32_t data)
{
    pq.queueWptrHi = data;
}

void
PM4PacketProcessor::setRbRptrAddrLo(uint32_t data)
{
    pq.queueRptrAddrLo = data;
}

void
PM4PacketProcessor::setRbRptrAddrHi(uint32_t data)
{
    pq.queueRptrAddrHi = data;
}

void
PM4PacketProcessor::setRbWptrPollAddrLo(uint32_t data)
{
    pq.hqd_pq_wptr_poll_addr_lo = data;
}

void
PM4PacketProcessor::setRbWptrPollAddrHi(uint32_t data)
{
    pq.hqd_pq_wptr_poll_addr_hi = data;
}

void
PM4PacketProcessor::setRbBaseLo(uint32_t data)
{
    pq.hqd_pq_base_lo = data;
}

void
PM4PacketProcessor::setRbBaseHi(uint32_t data)
{
    pq.hqd_pq_base_hi = data;
}

void
PM4PacketProcessor::setRbDoorbellCntrl(uint32_t data)
{
    pq.hqd_pq_doorbell_control = data;
    pq.doorbellOffset = data & 0x1ffffffc;
}

void
PM4PacketProcessor::setRbDoorbellRangeLo(uint32_t data)
{
    pq.doorbellRangeLo = data;
}

void
PM4PacketProcessor::setRbDoorbellRangeHi(uint32_t data)
{
    pq.doorbellRangeHi = data;
}

void
PM4PacketProcessor::serialize(CheckpointOut &cp) const
{
    // Serialize the DmaVirtDevice base class
    DmaVirtDevice::serialize(cp);

    int num_queues = queues.size();
    Addr id[num_queues];
    Addr mqd_base[num_queues];
    uint64_t mqd_read_index[num_queues];
    Addr base[num_queues];
    Addr rptr[num_queues];
    Addr wptr[num_queues];
    Addr ib_base[num_queues];
    Addr ib_rptr[num_queues];
    Addr ib_wptr[num_queues];
    Addr offset[num_queues];
    bool processing[num_queues];
    bool ib[num_queues];
    uint32_t me[num_queues];
    uint32_t pipe[num_queues];
    uint32_t queue[num_queues];
    bool privileged[num_queues];
    uint32_t hqd_active[num_queues];
    uint32_t hqd_vmid[num_queues];
    Addr aql_rptr[num_queues];
    uint32_t aql[num_queues];
    uint32_t doorbell[num_queues];
    uint32_t hqd_pq_control[num_queues];

    int i = 0;
    for (auto iter : queues) {
        PM4Queue *q = iter.second;
        id[i] = q->id();
        mqd_base[i] = q->mqdBase();
        mqd_read_index[i] = q->getMQD()->mqdReadIndex;
        bool cur_state = q->ib();
        q->ib(false);
        base[i] = q->base();
        rptr[i] = q->getRptr();
        wptr[i] = q->getWptr();
        q->ib(true);
        ib_base[i] = q->ibBase();
        ib_rptr[i] = q->getRptr();
        ib_wptr[i] = q->getWptr();
        q->ib(cur_state);
        offset[i] = q->offset();
        processing[i] = q->processing();
        ib[i] = q->ib();
        me[i] = q->me();
        pipe[i] = q->pipe();
        queue[i] = q->queue();
        privileged[i] = q->privileged();
        hqd_active[i] = q->getMQD()->hqd_active;
        hqd_vmid[i] = q->getMQD()->hqd_vmid;
        aql_rptr[i] = q->getMQD()->aqlRptr;
        aql[i] = q->getMQD()->aql;
        doorbell[i] = q->getMQD()->doorbell;
        hqd_pq_control[i] = q->getMQD()->hqd_pq_control;
        i++;
    }

    SERIALIZE_SCALAR(num_queues);
    SERIALIZE_ARRAY(id, num_queues);
    SERIALIZE_ARRAY(mqd_base, num_queues);
    SERIALIZE_ARRAY(mqd_read_index, num_queues);
    SERIALIZE_ARRAY(base, num_queues);
    SERIALIZE_ARRAY(rptr, num_queues);
    SERIALIZE_ARRAY(wptr, num_queues);
    SERIALIZE_ARRAY(ib_base, num_queues);
    SERIALIZE_ARRAY(ib_rptr, num_queues);
    SERIALIZE_ARRAY(ib_wptr, num_queues);
    SERIALIZE_ARRAY(offset, num_queues);
    SERIALIZE_ARRAY(processing, num_queues);
    SERIALIZE_ARRAY(ib, num_queues);
    SERIALIZE_ARRAY(me, num_queues);
    SERIALIZE_ARRAY(pipe, num_queues);
    SERIALIZE_ARRAY(queue, num_queues);
    SERIALIZE_ARRAY(privileged, num_queues);
    SERIALIZE_ARRAY(hqd_active, num_queues);
    SERIALIZE_ARRAY(hqd_vmid, num_queues);
    SERIALIZE_ARRAY(aql_rptr, num_queues);
    SERIALIZE_ARRAY(aql, num_queues);
    SERIALIZE_ARRAY(doorbell, num_queues);
    SERIALIZE_ARRAY(hqd_pq_control, num_queues);
}

void
PM4PacketProcessor::unserialize(CheckpointIn &cp)
{
    // Serialize the DmaVirtDevice base class
    DmaVirtDevice::unserialize(cp);

    int num_queues = 0;
    UNSERIALIZE_SCALAR(num_queues);

    Addr id[num_queues];
    Addr mqd_base[num_queues];
    uint64_t mqd_read_index[num_queues];
    Addr base[num_queues];
    Addr rptr[num_queues];
    Addr wptr[num_queues];
    Addr ib_base[num_queues];
    Addr ib_rptr[num_queues];
    Addr ib_wptr[num_queues];
    Addr offset[num_queues];
    bool processing[num_queues];
    bool ib[num_queues];
    uint32_t me[num_queues];
    uint32_t pipe[num_queues];
    uint32_t queue[num_queues];
    bool privileged[num_queues];
    uint32_t hqd_active[num_queues];
    uint32_t hqd_vmid[num_queues];
    Addr aql_rptr[num_queues];
    uint32_t aql[num_queues];
    uint32_t doorbell[num_queues];
    uint32_t hqd_pq_control[num_queues];

    UNSERIALIZE_ARRAY(id, num_queues);
    UNSERIALIZE_ARRAY(mqd_base, num_queues);
    UNSERIALIZE_ARRAY(mqd_read_index, num_queues);
    UNSERIALIZE_ARRAY(base, num_queues);
    UNSERIALIZE_ARRAY(rptr, num_queues);
    UNSERIALIZE_ARRAY(wptr, num_queues);
    UNSERIALIZE_ARRAY(ib_base, num_queues);
    UNSERIALIZE_ARRAY(ib_rptr, num_queues);
    UNSERIALIZE_ARRAY(ib_wptr, num_queues);
    UNSERIALIZE_ARRAY(offset, num_queues);
    UNSERIALIZE_ARRAY(processing, num_queues);
    UNSERIALIZE_ARRAY(ib, num_queues);
    UNSERIALIZE_ARRAY(me, num_queues);
    UNSERIALIZE_ARRAY(pipe, num_queues);
    UNSERIALIZE_ARRAY(queue, num_queues);
    UNSERIALIZE_ARRAY(privileged, num_queues);
    UNSERIALIZE_ARRAY(hqd_active, num_queues);
    UNSERIALIZE_ARRAY(hqd_vmid, num_queues);
    UNSERIALIZE_ARRAY(aql_rptr, num_queues);
    UNSERIALIZE_ARRAY(aql, num_queues);
    UNSERIALIZE_ARRAY(doorbell, num_queues);
    UNSERIALIZE_ARRAY(hqd_pq_control, num_queues);

    for (int i = 0; i < num_queues; i++) {
        QueueDesc *mqd = new QueueDesc();
        memset(mqd, 0, sizeof(QueueDesc));

        mqd->mqdBase = mqd_base[i] >> 8;
        mqd->mqdReadIndex = mqd_read_index[i];
        mqd->base = base[i] >> 8;
        mqd->aql = aql[i];

        PM4MapQueues* pkt = new PM4MapQueues;
        memset(pkt, 0, sizeof(PM4MapQueues));
        newQueue(mqd, offset[i], pkt, id[i]);

        if (ib[i]) {
            queues[id[i]]->wptr(ib_wptr[i]);
            queues[id[i]]->rptr(ib_rptr[i]);
        } else {
            queues[id[i]]->rptr(rptr[i]);
            queues[id[i]]->wptr(wptr[i]);
        }
        queues[id[i]]->ib(ib[i]);
        queues[id[i]]->offset(offset[i]);
        queues[id[i]]->processing(processing[i]);
        queues[id[i]]->setPkt(me[i], pipe[i], queue[i], privileged[i]);
        queues[id[i]]->getMQD()->hqd_active = hqd_active[i];
        queues[id[i]]->getMQD()->hqd_vmid = hqd_vmid[i];
        queues[id[i]]->getMQD()->aqlRptr = aql_rptr[i];
        queues[id[i]]->getMQD()->doorbell = doorbell[i];
        queues[id[i]]->getMQD()->hqd_pq_control = hqd_pq_control[i];

        if (mqd->aql) {
            int mqd_size = (1 << ((hqd_pq_control[i] & 0x3f) + 1)) * 4;
            auto &hsa_pp = gpuDevice->CP()->hsaPacketProc();
            hsa_pp.setDeviceQueueDesc(aql_rptr[i], base[i], id[i],
                                  mqd_size, 8, GfxVersion::gfx900, offset[i],
                                  mqd_read_index[i]);
        }

        DPRINTF(PM4PacketProcessor, "PM4 queue %d, rptr: %p wptr: %p\n",
                queues[id[i]]->id(), queues[id[i]]->rptr(),
                queues[id[i]]->wptr());
    }
}

} // namespace gem5
