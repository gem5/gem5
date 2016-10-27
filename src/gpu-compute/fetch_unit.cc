/*
 * Copyright (c) 2014-2015 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
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
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
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
 * Author: Brad Beckmann, Sooraj Puthoor
 */

#include "gpu-compute/fetch_unit.hh"

#include "debug/GPUFetch.hh"
#include "debug/GPUPort.hh"
#include "debug/GPUTLB.hh"
#include "gpu-compute/compute_unit.hh"
#include "gpu-compute/gpu_dyn_inst.hh"
#include "gpu-compute/gpu_static_inst.hh"
#include "gpu-compute/shader.hh"
#include "gpu-compute/wavefront.hh"
#include "mem/ruby/system/RubySystem.hh"

uint32_t FetchUnit::globalFetchUnitID;

FetchUnit::FetchUnit(const ComputeUnitParams* params) :
    timingSim(true),
    computeUnit(nullptr),
    fetchScheduler(params),
    waveList(nullptr)
{
}

FetchUnit::~FetchUnit()
{
    fetchQueue.clear();
    fetchStatusQueue.clear();
}

void
FetchUnit::init(ComputeUnit *cu)
{
    computeUnit = cu;
    timingSim = computeUnit->shader->timingSim;
    fetchQueue.clear();
    fetchStatusQueue.resize(computeUnit->shader->n_wf);

    for (int j = 0; j < computeUnit->shader->n_wf; ++j) {
        fetchStatusQueue[j] = std::make_pair(waveList->at(j), false);
    }

    fetchScheduler.bindList(&fetchQueue);
}

void
FetchUnit::exec()
{
    // re-evaluate waves which are marked as not ready for fetch
    for (int j = 0; j < computeUnit->shader->n_wf; ++j) {
        // Following code assumes 64-bit opertaion and all insts are
        // represented by 64-bit pointers to inst objects.
        Wavefront *curWave = fetchStatusQueue[j].first;
        assert (curWave);

        // The wavefront has to be active, the IB occupancy has to be
        // 4 or less instructions and it can not have any branches to
        // prevent speculative instruction fetches
        if (!fetchStatusQueue[j].second) {
            if (curWave->status == Wavefront::S_RUNNING &&
                curWave->instructionBuffer.size() <= 4 &&
                !curWave->instructionBufferHasBranch() &&
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
    // calculate the virtual address to fetch from the SQC
    Addr vaddr = wavefront->pc();

    /**
     * the instruction buffer holds one instruction per entry, regardless
     * of the underlying instruction's size. the PC, however, addresses
     * instrutions on a 32b granularity so we must account for that here.
    */
    for (int i = 0; i < wavefront->instructionBuffer.size(); ++i) {
        vaddr +=
            wavefront->instructionBuffer.at(i)->staticInstruction()->instSize();
    }
    vaddr = wavefront->basePtr +  vaddr;

    DPRINTF(GPUTLB, "CU%d: WF[%d][%d]: Initiating fetch translation: %#x\n",
            computeUnit->cu_id, wavefront->simdId, wavefront->wfSlotId, vaddr);

    // Since this is an instruction prefetch, if you're split then just finish
    // out the current line.
    int block_size = computeUnit->cacheLineSize();
    // check for split accesses
    Addr split_addr = roundDown(vaddr + block_size - 1, block_size);
    int size = block_size;

    if (split_addr > vaddr) {
        // misaligned access, just grab the rest of the line
        size = split_addr - vaddr;
    }

    // set up virtual request
    Request *req = new Request(0, vaddr, size, Request::INST_FETCH,
                               computeUnit->masterId(), 0, 0, 0);

    PacketPtr pkt = new Packet(req, MemCmd::ReadReq);
    // This fetchBlock is kind of faux right now - because the translations so
    // far don't actually return Data
    uint64_t fetchBlock;
    pkt->dataStatic(&fetchBlock);

    if (timingSim) {
        // SenderState needed on Return
        pkt->senderState = new ComputeUnit::ITLBPort::SenderState(wavefront);

        // Sender State needed by TLB hierarchy
        pkt->senderState =
            new TheISA::GpuTLB::TranslationState(BaseTLB::Execute,
                                                 computeUnit->shader->gpuTc,
                                                 false, pkt->senderState);

        if (computeUnit->sqcTLBPort->isStalled()) {
            assert(computeUnit->sqcTLBPort->retries.size() > 0);

            DPRINTF(GPUTLB, "Failed to send TLB req for FETCH addr %#x\n",
                    vaddr);

            computeUnit->sqcTLBPort->retries.push_back(pkt);
        } else if (!computeUnit->sqcTLBPort->sendTimingReq(pkt)) {
            // Stall the data port;
            // No more packet is issued till
            // ruby indicates resources are freed by
            // a recvReqRetry() call back on this port.
            computeUnit->sqcTLBPort->stallPort();

            DPRINTF(GPUTLB, "Failed to send TLB req for FETCH addr %#x\n",
                    vaddr);

            computeUnit->sqcTLBPort->retries.push_back(pkt);
        } else {
            DPRINTF(GPUTLB, "sent FETCH translation request for %#x\n", vaddr);
        }
    } else {
        pkt->senderState =
            new TheISA::GpuTLB::TranslationState(BaseTLB::Execute,
                                                 computeUnit->shader->gpuTc);

        computeUnit->sqcTLBPort->sendFunctional(pkt);

        TheISA::GpuTLB::TranslationState *sender_state =
             safe_cast<TheISA::GpuTLB::TranslationState*>(pkt->senderState);

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
            computeUnit->cu_id, wavefront->simdId, wavefront->wfSlotId,
            pkt->req->getPaddr());

    // this is necessary because the GPU TLB receives packets instead of
    // requests. when the translation is complete, all relevent fields in the
    // request will be populated, but not in the packet. here we create the
    // new packet so we can set the size, addr, and proper flags.
    PacketPtr oldPkt = pkt;
    pkt = new Packet(oldPkt->req, oldPkt->cmd);
    delete oldPkt;

    TheGpuISA::RawMachInst *data =
        new TheGpuISA::RawMachInst[pkt->req->getSize() /
        sizeof(TheGpuISA::RawMachInst)];

    pkt->dataDynamic<TheGpuISA::RawMachInst>(data);

    // New SenderState for the memory access
    pkt->senderState = new ComputeUnit::SQCPort::SenderState(wavefront);

    if (timingSim) {
        // translation is done. Send the appropriate timing memory request.

        if (!computeUnit->sqcPort->sendTimingReq(pkt)) {
            computeUnit->sqcPort->retries.push_back(std::make_pair(pkt,
                                                                   wavefront));

            DPRINTF(GPUPort, "CU%d: WF[%d][%d]: Fetch addr %#x failed!\n",
                    computeUnit->cu_id, wavefront->simdId, wavefront->wfSlotId,
                    pkt->req->getPaddr());
        } else {
            DPRINTF(GPUPort, "CU%d: WF[%d][%d]: Fetch addr %#x sent!\n",
                    computeUnit->cu_id, wavefront->simdId, wavefront->wfSlotId,
                    pkt->req->getPaddr());
        }
    } else {
        computeUnit->sqcPort->sendFunctional(pkt);
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
            "%d bytes, %d instructions!\n", computeUnit->cu_id,
            wavefront->simdId, wavefront->wfSlotId, pkt->req->getPaddr(),
            pkt->req->getSize(), pkt->req->getSize() /
            sizeof(TheGpuISA::RawMachInst));

    if (wavefront->dropFetch) {
        assert(wavefront->instructionBuffer.empty());
        wavefront->dropFetch = false;
    } else {
        TheGpuISA::RawMachInst *inst_index_ptr =
            (TheGpuISA::RawMachInst*)pkt->getPtr<uint8_t>();

        assert(wavefront->instructionBuffer.size() <= 4);

        for (int i = 0; i < pkt->req->getSize() /
             sizeof(TheGpuISA::RawMachInst); ++i) {
            GPUStaticInst *inst_ptr = decoder.decode(inst_index_ptr[i]);

            assert(inst_ptr);

            if (inst_ptr->instSize() == 8) {
                /**
                 * this instruction occupies 2 consecutive
                 * entries in the instruction array, the
                 * second of which contains a nullptr. so if
                 * this inst is 8 bytes we advance two entries
                 * instead of 1
                 */
                ++i;
            }

            DPRINTF(GPUFetch, "CU%d: WF[%d][%d]: added %s\n",
                    computeUnit->cu_id, wavefront->simdId,
                    wavefront->wfSlotId, inst_ptr->disassemble());

            GPUDynInstPtr gpuDynInst =
                std::make_shared<GPUDynInst>(computeUnit, wavefront, inst_ptr,
                                             computeUnit->getAndIncSeqNum());

            wavefront->instructionBuffer.push_back(gpuDynInst);
        }
    }

    wavefront->pendingFetch = false;

    delete pkt->senderState;
    delete pkt->req;
    delete pkt;
}

void
FetchUnit::bindWaveList(std::vector<Wavefront*> *wave_list)
{
    waveList = wave_list;
}
