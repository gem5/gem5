/*
 * Copyright (c) 2014-2017 Advanced Micro Devices, Inc.
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

#include "gpu-compute/fetch_unit.hh"

#include "arch/amdgpu/common/gpu_translation_state.hh"
#include "arch/amdgpu/common/tlb.hh"
#include "base/bitfield.hh"
#include "debug/GPUFetch.hh"
#include "debug/GPUPort.hh"
#include "debug/GPUTLB.hh"
#include "gpu-compute/compute_unit.hh"
#include "gpu-compute/gpu_dyn_inst.hh"
#include "gpu-compute/gpu_static_inst.hh"
#include "gpu-compute/shader.hh"
#include "gpu-compute/wavefront.hh"
#include "mem/ruby/system/RubySystem.hh"

namespace gem5
{

uint32_t FetchUnit::globalFetchUnitID;

FetchUnit::FetchUnit(const ComputeUnitParams &p, ComputeUnit &cu)
    : timingSim(true), computeUnit(cu), fetchScheduler(p),
      waveList(nullptr), fetchDepth(p.fetch_depth)
{
}

FetchUnit::~FetchUnit()
{
    fetchQueue.clear();
    fetchStatusQueue.clear();
}

void
FetchUnit::init()
{
    timingSim = computeUnit.shader->timingSim;
    fetchQueue.clear();
    fetchStatusQueue.resize(computeUnit.shader->n_wf);
    fetchBuf.resize(computeUnit.shader->n_wf, FetchBufDesc());

    for (int i = 0; i < computeUnit.shader->n_wf; ++i) {
        Wavefront *wf = waveList->at(i);
        assert(wf->wfSlotId == i);
        fetchStatusQueue[i] = std::make_pair(wf, false);
        fetchBuf[i].allocateBuf(fetchDepth, computeUnit.cacheLineSize(), wf);
        fetchBuf[i].decoder(&decoder);
    }

    fetchScheduler.bindList(&fetchQueue);
}

void
FetchUnit::exec()
{
    /**
     * now we check if any of the fetch buffers have
     * buffered instruction data that can be decoded
     * and sent to its wavefront's instruction buffer.
     * then we check if any of the fetch buffer entries
     * can be released. we only check if we can
     * release a buffer
     */
    for (auto &fetch_buf : fetchBuf) {
        if (!fetch_buf.hasFreeSpace()) {
            fetch_buf.checkWaveReleaseBuf();
        }
        if (fetch_buf.hasFetchDataToProcess()) {
            fetch_buf.decodeInsts();
        }
    }

    // re-evaluate waves which are marked as not ready for fetch
    for (int j = 0; j < computeUnit.shader->n_wf; ++j) {
        // Following code assumes 64-bit opertaion and all insts are
        // represented by 64-bit pointers to inst objects.
        Wavefront *curWave = fetchStatusQueue[j].first;
        assert (curWave);

        // The wavefront has to be active, the IB occupancy has to be
        // 4 or less instructions and it can not have any branches to
        // prevent speculative instruction fetches
        if (!fetchStatusQueue[j].second) {
            if ((curWave->getStatus() == Wavefront::S_RUNNING ||
                curWave->getStatus() == Wavefront::S_WAITCNT) &&
                fetchBuf[j].hasFreeSpace() &&
                !curWave->stopFetch() &&
                !curWave->pendingFetch) {
                fetchQueue.push_back(curWave);
                fetchStatusQueue[j].second = true;
            }
        }
    }

    // Fetch only if there is some wave ready to be fetched
    // An empty fetchQueue will cause the schedular to panic
    if (fetchQueue.size()) {
        Wavefront *waveToBeFetched = fetchScheduler.chooseWave();
        waveToBeFetched->pendingFetch = true;
        fetchStatusQueue[waveToBeFetched->wfSlotId].second = false;
        initiateFetch(waveToBeFetched);
    }
}

void
FetchUnit::initiateFetch(Wavefront *wavefront)
{
    assert(fetchBuf.at(wavefront->wfSlotId).hasFreeSpace());

    /**
     * calculate the virtual address to fetch from the SQC. the fetch
     * buffer holds a configurable number of cache lines. we start
     * fetching at the address of the cache line immediately following
     * the buffered line(s).
     */
    Addr vaddr = fetchBuf.at(wavefront->wfSlotId).nextFetchAddr();

    // this should already be aligned to a cache line
    assert(vaddr == ruby::makeLineAddress(vaddr,
           computeUnit.getCacheLineBits()));

    // shouldn't be fetching a line that is already buffered
    assert(!fetchBuf.at(wavefront->wfSlotId).pcBuffered(vaddr));

    fetchBuf.at(wavefront->wfSlotId).reserveBuf(vaddr);

    DPRINTF(GPUFetch, "CU%d: WF[%d][%d]: Id%d: Initiate fetch "
            "from pc: %d %#x\n", computeUnit.cu_id, wavefront->simdId,
            wavefront->wfSlotId, wavefront->wfDynId, wavefront->pc(), vaddr);

    DPRINTF(GPUTLB, "CU%d: WF[%d][%d]: Initiating fetch translation: %#x\n",
            computeUnit.cu_id, wavefront->simdId, wavefront->wfSlotId, vaddr);

    // set up virtual request
    RequestPtr req = std::make_shared<Request>(
        vaddr, computeUnit.cacheLineSize(), Request::INST_FETCH,
        computeUnit.requestorId(), 0, 0, nullptr);

    PacketPtr pkt = new Packet(req, MemCmd::ReadReq);

    if (timingSim) {
        // SenderState needed on Return
        pkt->senderState = new ComputeUnit::ITLBPort::SenderState(wavefront);

        // Sender State needed by TLB hierarchy
        pkt->senderState =
            new GpuTranslationState(BaseMMU::Execute,
                                                 computeUnit.shader->gpuTc,
                                                 false, pkt->senderState);

        if (computeUnit.sqcTLBPort.isStalled()) {
            assert(computeUnit.sqcTLBPort.retries.size() > 0);

            DPRINTF(GPUTLB, "Failed to send TLB req for FETCH addr %#x\n",
                    vaddr);

            computeUnit.sqcTLBPort.retries.push_back(pkt);
        } else if (!computeUnit.sqcTLBPort.sendTimingReq(pkt)) {
            // Stall the data port;
            // No more packet is issued till
            // ruby indicates resources are freed by
            // a recvReqRetry() call back on this port.
            computeUnit.sqcTLBPort.stallPort();

            DPRINTF(GPUTLB, "Failed to send TLB req for FETCH addr %#x\n",
                    vaddr);

            computeUnit.sqcTLBPort.retries.push_back(pkt);
        } else {
            DPRINTF(GPUTLB, "sent FETCH translation request for %#x\n", vaddr);
        }
    } else {
        pkt->senderState =
            new GpuTranslationState(BaseMMU::Execute,
                                                 computeUnit.shader->gpuTc);

        computeUnit.sqcTLBPort.sendFunctional(pkt);

        /**
         * For full system, if this is a device request we need to set the
         * requestor ID of the packet to the GPU memory manager so it is routed
         * through Ruby as a memory request and not a PIO request.
         */
        if (!pkt->req->systemReq()) {
            pkt->req->requestorId(computeUnit.vramRequestorId());
        }

        GpuTranslationState *sender_state =
             safe_cast<GpuTranslationState*>(pkt->senderState);

        delete sender_state->tlbEntry;
        delete sender_state;
        // fetch the instructions from the SQC when we operate in
        // functional mode only
        fetch(pkt, wavefront);
    }
}

void
FetchUnit::fetch(PacketPtr pkt, Wavefront *wavefront)
{
    assert(pkt->req->hasPaddr());
    assert(pkt->req->hasSize());

    DPRINTF(GPUFetch, "CU%d: WF[%d][%d]: Fetch Access: %#x\n",
            computeUnit.cu_id, wavefront->simdId, wavefront->wfSlotId,
            pkt->req->getPaddr());

    /**
     * this is necessary because the GPU TLB receives packets instead of
     * requests. when the translation is complete, all relevent fields in
     * the request will be populated, but not in the packet. here we create
     * the new packet so we can set the size, addr, and proper flags.
     */
    PacketPtr oldPkt = pkt;
    pkt = new Packet(oldPkt->req, oldPkt->cmd);
    delete oldPkt;

    /**
     * if we have not reserved an entry in the fetch buffer,
     * stop fetching. this can happen due to a branch instruction
     * flushing the fetch buffer while an ITLB or I-cache request is still
     * pending, in the same cycle another instruction is trying to fetch.
     */
    if (!fetchBuf.at(wavefront->wfSlotId).isReserved(pkt->req->getVaddr())) {
        wavefront->dropFetch = false;
        wavefront->pendingFetch = false;
        return;
    }

    /**
     * For full system, if this is a device request we need to set the
     * requestor ID of the packet to the GPU memory manager so it is routed
     * through Ruby as a memory request and not a PIO request.
     */
    if (!pkt->req->systemReq()) {
        pkt->req->requestorId(computeUnit.vramRequestorId());
    }

    /**
     * we should have reserved an entry in the fetch buffer
     * for this cache line. here we get the pointer to the
     * entry used to buffer this request's line data.
     */
    pkt->dataStatic(fetchBuf.at(wavefront->wfSlotId)
                    .reservedBuf(pkt->req->getVaddr()));

    // New SenderState for the memory access
    pkt->senderState = new ComputeUnit::SQCPort::SenderState(wavefront);

    if (timingSim) {
        // translation is done. Send the appropriate timing memory request.

        if (pkt->req->systemReq()) {
            SystemHubEvent *resp_event = new SystemHubEvent(pkt, this);
            assert(computeUnit.shader->systemHub);
            computeUnit.shader->systemHub->sendRequest(pkt, resp_event);
        } else if (!computeUnit.sqcPort.sendTimingReq(pkt)) {
            computeUnit.sqcPort.retries.push_back(std::make_pair(pkt,
                                                                   wavefront));

            DPRINTF(GPUPort, "CU%d: WF[%d][%d]: Fetch addr %#x failed!\n",
                    computeUnit.cu_id, wavefront->simdId, wavefront->wfSlotId,
                    pkt->req->getPaddr());
        } else {
            DPRINTF(GPUPort, "CU%d: WF[%d][%d]: Fetch addr %#x sent!\n",
                    computeUnit.cu_id, wavefront->simdId, wavefront->wfSlotId,
                    pkt->req->getPaddr());
        }
    } else {
        computeUnit.sqcPort.sendFunctional(pkt);
        processFetchReturn(pkt);
    }
}

void
FetchUnit::processFetchReturn(PacketPtr pkt)
{
    ComputeUnit::SQCPort::SenderState *sender_state =
        safe_cast<ComputeUnit::SQCPort::SenderState*>(pkt->senderState);

    Wavefront *wavefront = sender_state->wavefront;

    DPRINTF(GPUFetch, "CU%d: WF[%d][%d]: Fetch addr %#x returned "
            "%d bytes!\n", computeUnit.cu_id, wavefront->simdId,
            wavefront->wfSlotId, pkt->req->getPaddr(), pkt->req->getSize());

    if (wavefront->dropFetch) {
        assert(wavefront->instructionBuffer.empty());
        assert(!fetchBuf.at(wavefront->wfSlotId).hasFetchDataToProcess());
        wavefront->dropFetch = false;
    } else {
        fetchBuf.at(wavefront->wfSlotId).fetchDone(pkt->req->getVaddr());
    }

    wavefront->pendingFetch = false;

    delete pkt->senderState;
    delete pkt;
}

void
FetchUnit::flushBuf(int wfSlotId)
{
    fetchBuf.at(wfSlotId).flushBuf();
}

void
FetchUnit::bindWaveList(std::vector<Wavefront*> *wave_list)
{
    waveList = wave_list;
}

/** FetchBufDesc */
void
FetchUnit::FetchBufDesc::allocateBuf(int fetch_depth, int cache_line_size,
                                     Wavefront *wf)
{
    wavefront = wf;
    fetchDepth = fetch_depth;
    maxIbSize = wavefront->maxIbSize;
    cacheLineSize = cache_line_size;
    maxFbSize = cacheLineSize * fetchDepth;

    // Calculate the number of bits to address a cache line
    panic_if(!isPowerOf2(cacheLineSize),
        "Cache line size should be a power of two.");
    cacheLineBits = floorLog2(cacheLineSize);

    bufStart = new uint8_t[maxFbSize];
    readPtr = bufStart;
    bufEnd = bufStart + maxFbSize;

    for (int i = 0; i < fetchDepth; ++i) {
        freeList.emplace_back(readPtr + i * cacheLineSize);
    }
}

void
FetchUnit::FetchBufDesc::flushBuf()
{
    restartFromBranch = true;
    /**
     * free list may have some entries
     * so we clear it here to avoid duplicates
     */
    freeList.clear();
    bufferedPCs.clear();
    reservedPCs.clear();
    readPtr = bufStart;

    for (int i = 0; i < fetchDepth; ++i) {
        freeList.push_back(bufStart + i * cacheLineSize);
    }

    DPRINTF(GPUFetch, "WF[%d][%d]: Id%d Fetch dropped, flushing fetch "
            "buffer\n", wavefront->simdId, wavefront->wfSlotId,
            wavefront->wfDynId);
}

void
FetchUnit::FetchBufDesc::invBuf()
{
    restartFromBranch = false;
    /**
     * free list may have some entries
     * so we clear it here to avoid duplicates
     */
    freeList.clear();
    bufferedPCs.clear();
    reservedPCs.clear();
    readPtr = bufStart;

    for (int i = 0; i < fetchDepth; ++i) {
        freeList.push_back(bufStart + i * cacheLineSize);
    }

    DPRINTF(GPUFetch, "WF[%d][%d]: Id%d Fetch dropped, flushing fetch "
            "buffer\n", wavefront->simdId, wavefront->wfSlotId,
            wavefront->wfDynId);

}

Addr
FetchUnit::FetchBufDesc::nextFetchAddr()
{
    Addr next_line = 0;

    if (bufferedAndReservedLines()) {
        Addr last_line_fetched = 0;
        if (!reservedLines()) {
            /**
             * get the PC of the most recently fetched cache line,
             * then return the address of the next line.
             */
            last_line_fetched = bufferedPCs.rbegin()->first;
        } else {
            last_line_fetched = reservedPCs.rbegin()->first;
        }

        next_line = last_line_fetched + cacheLineSize;

        /**
         * should not be trying to fetch a line that has already
         * been fetched.
         */
        assert(bufferedPCs.find(next_line) == bufferedPCs.end());
        assert(reservedPCs.find(next_line) == reservedPCs.end());
    } else {
        /**
         * we do not have any buffered cache lines yet, so we
         * assume this is the initial fetch, or the first fetch
         * after a branch, and get the PC directly from the WF.
         * in the case of a branch, we may not start at the
         * beginning of a cache line, so we adjust the readPtr by
         * the current PC's offset from the start of the line.
         */
        next_line = ruby::makeLineAddress(wavefront->pc(), cacheLineBits);
        readPtr = bufStart;

        /**
         * if we are here we have no buffered lines. in the case we flushed
         * the buffer due to a branch, we may need to start fetching from
         * some offset from the start of the fetch buffer, so we adjust for
         * that here.
         */
        if (restartFromBranch) {
            restartFromBranch = false;
            int byte_offset
                = wavefront->pc() - ruby::makeLineAddress(wavefront->pc(),
                                    cacheLineBits);
            readPtr += byte_offset;
        }
    }

    return next_line;
}

void
FetchUnit::FetchBufDesc::reserveBuf(Addr vaddr)
{
    // we should have free buffer space, and the line
    // at vaddr should not already be cached.
    assert(hasFreeSpace());
    assert(bufferedPCs.find(vaddr) == bufferedPCs.end());
    assert(reservedPCs.find(vaddr) == reservedPCs.end());
    assert(bufferedAndReservedLines() < fetchDepth);

    DPRINTF(GPUFetch, "WF[%d][%d]: Id%d reserved fetch buffer entry "
            "for PC = %#x\n", wavefront->simdId, wavefront->wfSlotId,
            wavefront->wfDynId, vaddr);

    /**
     * we reserve buffer space, by moving it out of the
     * free list, however we do not mark the buffered
     * line as valid until the fetch unit for this buffer
     * has receieved the response from the memory system.
     */
    uint8_t *inst_buf = freeList.front();
    reservedPCs.emplace(vaddr, inst_buf);
    freeList.pop_front();
}

void
FetchUnit::FetchBufDesc::fetchDone(Addr vaddr)
{
    if (vaddr == 0) {
        // S_ICACHE_INV fetch done
        wavefront->decLGKMInstsIssued();
        invBuf();
        return;
    }

    assert(bufferedPCs.find(vaddr) == bufferedPCs.end());
    DPRINTF(GPUFetch, "WF[%d][%d]: Id%d done fetching for addr %#x\n",
            wavefront->simdId, wavefront->wfSlotId,
            wavefront->wfDynId, vaddr);

    /**
     * this address should have an entry reserved in the
     * fetch buffer already, however it should be invalid
     * until the fetch completes.
     */
    auto reserved_pc = reservedPCs.find(vaddr);
    assert(reserved_pc != reservedPCs.end());
    bufferedPCs.emplace(vaddr, reserved_pc->second);

    if (readPtr == bufEnd) {
        readPtr = bufStart;
    }

    reserved_pc->second = nullptr;
    reservedPCs.erase(reserved_pc);
}

bool
FetchUnit::FetchBufDesc::hasFetchDataToProcess() const
{
    return fetchBytesRemaining() >= sizeof(TheGpuISA::RawMachInst);
}

void
FetchUnit::FetchBufDesc::checkWaveReleaseBuf()
{
    Addr cur_wave_pc = roundDown(wavefront->pc(),
                                 wavefront->computeUnit->cacheLineSize());
    if (reservedPCs.find(cur_wave_pc) != reservedPCs.end()) {
        DPRINTF(GPUFetch, "WF[%d][%d]: Id%d current wave PC(%#x) still "
                "being fetched.\n", wavefront->simdId, wavefront->wfSlotId,
                wavefront->wfDynId, cur_wave_pc);

        // should be reserved, but not buffered yet
        assert(bufferedPCs.find(cur_wave_pc) == bufferedPCs.end());

        return;
    }

    auto current_buffered_pc = bufferedPCs.find(cur_wave_pc);
    auto oldest_buffered_pc = bufferedPCs.begin();

    DPRINTF(GPUFetch, "WF[%d][%d]: Id%d checking if PC block addr = %#x"
            "(PC = %#x) can be released.\n", wavefront->simdId,
            wavefront->wfSlotId, wavefront->wfDynId, cur_wave_pc,
            wavefront->pc());

#ifdef GEM5_DEBUG
    int idx = 0;
    for (const auto &buf_pc : bufferedPCs) {
        DPRINTF(GPUFetch, "PC[%d] = %#x\n", idx, buf_pc.first);
        ++idx;
    }
#endif

    // if we haven't buffered data for this PC, we shouldn't
    // be fetching from it.
    assert(current_buffered_pc != bufferedPCs.end());

    /**
     * we're using a std::map so the addresses are sorted. if this
     * PC is not the oldest one in the map, we must be fetching from
     * a newer block, and we can release the oldest PC's fetch buffer
     * entry back to the free list.
     */
    if (current_buffered_pc != oldest_buffered_pc) {
        DPRINTF(GPUFetch, "WF[%d][%d]: Id%d done fetching for PC = %#x, "
                "removing it from the fetch buffer.\n", wavefront->simdId,
                wavefront->wfSlotId, wavefront->wfDynId,
                oldest_buffered_pc->first);

        freeList.emplace_back(oldest_buffered_pc->second);
        oldest_buffered_pc->second = nullptr;
        bufferedPCs.erase(oldest_buffered_pc);
        DPRINTF(GPUFetch, "WF[%d][%d]: Id%d has %d lines buffered.\n",
                wavefront->simdId, wavefront->wfSlotId, wavefront->wfDynId,
                bufferedLines());
    }
}

void
FetchUnit::FetchBufDesc::decodeInsts()
{
    assert(readPtr);

    if (splitDecode()) {
        decodeSplitInst();
    }

    while (wavefront->instructionBuffer.size() < maxIbSize
           && hasFetchDataToProcess()) {
        if (splitDecode()) {
            decodeSplitInst();
        } else {
            TheGpuISA::MachInst mach_inst
                = reinterpret_cast<TheGpuISA::MachInst>(readPtr);
            GPUStaticInst *gpu_static_inst = _decoder->decode(mach_inst);
            readPtr += gpu_static_inst->instSize();

            assert(readPtr <= bufEnd);

            GPUDynInstPtr gpu_dyn_inst
                = std::make_shared<GPUDynInst>(wavefront->computeUnit,
                                               wavefront, gpu_static_inst,
                                               wavefront->computeUnit->
                                                getAndIncSeqNum());
            wavefront->instructionBuffer.push_back(gpu_dyn_inst);

            DPRINTF(GPUFetch, "WF[%d][%d]: Id%ld decoded %s (%d bytes). "
                    "%d bytes remain.\n", wavefront->simdId,
                    wavefront->wfSlotId, wavefront->wfDynId,
                    gpu_static_inst->disassemble(),
                    gpu_static_inst->instSize(),
                    fetchBytesRemaining());
        }
    }
}

void
FetchUnit::FetchBufDesc::decodeSplitInst()
{
    TheGpuISA::RawMachInst split_inst = 0;
    int dword_size = sizeof(uint32_t);
    int num_dwords = sizeof(TheGpuISA::RawMachInst) / dword_size;

    for (int i = 0; i < num_dwords; ++i) {
        replaceBits(split_inst, 32*(i+1)-1, 32*i,
            *reinterpret_cast<uint32_t*>(readPtr));
        if (readPtr + dword_size >= bufEnd) {
            readPtr = bufStart;
        }
    }

    assert(readPtr == bufStart);

    TheGpuISA::MachInst mach_inst
        = reinterpret_cast<TheGpuISA::MachInst>(&split_inst);
    GPUStaticInst *gpu_static_inst = _decoder->decode(mach_inst);
    readPtr += (gpu_static_inst->instSize() - dword_size);
    assert(readPtr < bufEnd);

    GPUDynInstPtr gpu_dyn_inst
        = std::make_shared<GPUDynInst>(wavefront->computeUnit,
                                       wavefront, gpu_static_inst,
                                       wavefront->computeUnit->
                                           getAndIncSeqNum());
    wavefront->instructionBuffer.push_back(gpu_dyn_inst);

    DPRINTF(GPUFetch, "WF[%d][%d]: Id%d decoded split inst %s (%#x) "
            "(%d bytes). %d bytes remain in %d buffered lines.\n",
            wavefront->simdId, wavefront->wfSlotId, wavefront->wfDynId,
            gpu_static_inst->disassemble(), split_inst,
            gpu_static_inst->instSize(), fetchBytesRemaining(),
            bufferedLines());
}

bool
FetchUnit::FetchBufDesc::splitDecode() const
{
    /**
     * if a read of a raw instruction would go beyond the end
     * of the fetch buffer, then we must perform a split decode.
     */
    bool is_split = (readPtr + sizeof(TheGpuISA::RawMachInst)) > bufEnd;

    return is_split;
}

int
FetchUnit::FetchBufDesc::fetchBytesRemaining() const
{
    int bytes_remaining = 0;

    if (bufferedLines() && readPtr != bufEnd) {
        auto last_buf_pc = bufferedPCs.rbegin();
        uint8_t *end_ptr = last_buf_pc->second + cacheLineSize;
        int byte_diff = end_ptr - readPtr;

        if (end_ptr > readPtr) {
            bytes_remaining = byte_diff;
        } else if (end_ptr < readPtr) {
            bytes_remaining = bufferedBytes() + byte_diff;
        }
    }

    assert(bytes_remaining <= bufferedBytes());
    return bytes_remaining;
}

void
FetchUnit::SystemHubEvent::process()
{
    reqPkt->makeResponse();
    fetchUnit->computeUnit.handleSQCReturn(reqPkt);
}

} // namespace gem5
