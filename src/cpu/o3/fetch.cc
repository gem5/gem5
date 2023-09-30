/*
 * Copyright (c) 2010-2014 ARM Limited
 * Copyright (c) 2012-2013 AMD
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "cpu/o3/fetch.hh"

#include <algorithm>
#include <cstring>
#include <list>
#include <map>
#include <queue>

#include "arch/generic/tlb.hh"
#include "base/random.hh"
#include "base/types.hh"
#include "cpu/base.hh"
#include "cpu/exetrace.hh"
#include "cpu/nop_static_inst.hh"
#include "cpu/o3/cpu.hh"
#include "cpu/o3/dyn_inst.hh"
#include "cpu/o3/limits.hh"
#include "debug/Activity.hh"
#include "debug/Drain.hh"
#include "debug/FDIP.hh"
#include "debug/Fetch.hh"
#include "debug/O3CPU.hh"
#include "debug/O3PipeView.hh"
#include "mem/packet.hh"
#include "params/BaseO3CPU.hh"
#include "sim/byteswap.hh"
#include "sim/core.hh"
#include "sim/eventq.hh"
#include "sim/full_system.hh"
#include "sim/system.hh"

namespace gem5
{

namespace o3
{

#define CACHE_LINE_SIZE 64 // line size in bytes
#define CACHE_LINE_SIZE_WIDTH 6 // number of bits
#define ICACHE_ACCESS_LATENCY 2 // in cycles
#define INST_SIZE 4 // in bytes
Fetch::IcachePort::IcachePort(Fetch *_fetch, CPU *_cpu) :
        RequestPort(_cpu->name() + ".icache_port"), fetch(_fetch)
{}


Fetch::Fetch(CPU *_cpu, const BaseO3CPUParams &params)
    : fetchPolicy(params.smtFetchPolicy),
      cpu(_cpu),
      branchPred(nullptr),
      decodeToFetchDelay(params.decodeToFetchDelay),
      renameToFetchDelay(params.renameToFetchDelay),
      iewToFetchDelay(params.iewToFetchDelay),
      commitToFetchDelay(params.commitToFetchDelay),
      fetchWidth(params.fetchWidth),
      decodeWidth(params.decodeWidth),
      retryPkt(NULL),
      retryTid(InvalidThreadID),
      cacheBlkSize(cpu->cacheLineSize()),
      fetchBufferSize(params.fetchBufferSize),
      fetchBufferMask(fetchBufferSize - 1),
      fetchQueueSize(params.fetchQueueSize),
      numThreads(params.numThreads),
      numFetchingThreads(params.smtNumFetchingThreads),
      enableFDIP(params.enableFDIP),
      icachePort(this, _cpu),
      finishTranslationEvent(this),
      ftqSize(params.ftqSize),
      ftqInst(params.ftqInst),
      fetchStats(_cpu, this)
{
    if (numThreads > MaxThreads)
        fatal("numThreads (%d) is larger than compiled limit (%d),\n"
              "\tincrease MaxThreads in src/cpu/o3/limits.hh\n",
              numThreads, static_cast<int>(MaxThreads));
    if (fetchWidth > MaxWidth)
        fatal("fetchWidth (%d) is larger than compiled limit (%d),\n"
             "\tincrease MaxWidth in src/cpu/o3/limits.hh\n",
             fetchWidth, static_cast<int>(MaxWidth));
    if (fetchBufferSize > cacheBlkSize)
        fatal("fetch buffer size (%u bytes) is greater than the cache "
              "block size (%u bytes)\n", fetchBufferSize, cacheBlkSize);
    if (cacheBlkSize % fetchBufferSize)
        fatal("cache block (%u bytes) is not a multiple of the "
              "fetch buffer (%u bytes)\n", cacheBlkSize, fetchBufferSize);

    for (int i = 0; i < MaxThreads; i++) {
        fetchStatus[i] = Idle;
        decoder[i] = nullptr;
        pc[i].reset(params.isa[0]->newPCState());
        fetchOffset[i] = 0;
        macroop[i] = nullptr;
        delayedCommit[i] = false;
        //memReq[i] = nullptr;
        stalls[i] = {false, false};
        fetchBuffer[i].clear();
        prefetchBufferPC[i].clear();
        ftq[i].clear();
        lastIcacheStall[i] = 0;
        issuePipelinedIfetch[i] = false;
        seq[i] = 1;
        brseq[i] = 0;
    }

    branchPred = params.branchPred;

    for (ThreadID tid = 0; tid < numThreads; tid++) {
        decoder[tid] = params.decoder[tid];
        // Create space to buffer the cache line data,
        // which may not hold the entire cache line.
        //fetchBuffer[tid] = new uint8_t[fetchBufferSize];
    }

    // Get the size of an instruction.
    instSize = decoder[0]->moreBytesSize();
}

std::string Fetch::name() const { return cpu->name() + ".fetch"; }

void
Fetch::regProbePoints()
{
    ppFetch = new ProbePointArg<DynInstPtr>(cpu->getProbeManager(), "Fetch");
    ppFetchRequestSent = new ProbePointArg<RequestPtr>(cpu->getProbeManager(),
                                                       "FetchRequest");

}

Fetch::FetchStatGroup::FetchStatGroup(CPU *cpu, Fetch *fetch)
    : statistics::Group(cpu, "fetch"),
    ADD_STAT(predictedBranches, statistics::units::Count::get(),
             "Number of branches that fetch has predicted taken"),
    ADD_STAT(cycles, statistics::units::Cycle::get(),
             "Number of cycles fetch has run and was not squashing or "
             "blocked"),
    ADD_STAT(squashCycles, statistics::units::Cycle::get(),
             "Number of cycles fetch has spent squashing"),
    ADD_STAT(tlbCycles, statistics::units::Cycle::get(),
             "Number of cycles fetch has spent waiting for tlb"),
    ADD_STAT(idleCycles, statistics::units::Cycle::get(),
             "Number of cycles fetch was idle"),
    ADD_STAT(blockedCycles, statistics::units::Cycle::get(),
             "Number of cycles fetch has spent blocked"),
    ADD_STAT(miscStallCycles, statistics::units::Cycle::get(),
             "Number of cycles fetch has spent waiting on interrupts, or bad "
             "addresses, or out of MSHRs"),
    ADD_STAT(pendingDrainCycles, statistics::units::Cycle::get(),
             "Number of cycles fetch has spent waiting on pipes to drain"),
    ADD_STAT(noActiveThreadStallCycles, statistics::units::Cycle::get(),
             "Number of stall cycles due to no active thread to fetch from"),
    ADD_STAT(pendingTrapStallCycles, statistics::units::Cycle::get(),
             "Number of stall cycles due to pending traps"),
    ADD_STAT(pendingQuiesceStallCycles, statistics::units::Cycle::get(),
             "Number of stall cycles due to pending quiesce instructions"),
    ADD_STAT(icacheWaitRetryStallCycles, statistics::units::Cycle::get(),
             "Number of stall cycles due to full MSHR"),
    ADD_STAT(cacheLines, statistics::units::Count::get(),
             "Number of cache lines fetched"),
    ADD_STAT(icacheSquashes, statistics::units::Count::get(),
             "Number of outstanding Icache misses that were squashed"),
    ADD_STAT(tlbSquashes, statistics::units::Count::get(),
             "Number of outstanding ITLB misses that were squashed"),
    ADD_STAT(nisnDist, statistics::units::Count::get(),
             "Number of instructions fetched each cycle (Total)"),
    ADD_STAT(idleRate, statistics::units::Ratio::get(),
             "Ratio of cycles fetch was idle",
             idleCycles / cpu->baseStats.numCycles)
{
        predictedBranches
            .prereq(predictedBranches);
        cycles
            .prereq(cycles);
        squashCycles
            .prereq(squashCycles);
        tlbCycles
            .prereq(tlbCycles);
        idleCycles
            .prereq(idleCycles);
        blockedCycles
            .prereq(blockedCycles);
        cacheLines
            .prereq(cacheLines);
        miscStallCycles
            .prereq(miscStallCycles);
        pendingDrainCycles
            .prereq(pendingDrainCycles);
        noActiveThreadStallCycles
            .prereq(noActiveThreadStallCycles);
        pendingTrapStallCycles
            .prereq(pendingTrapStallCycles);
        pendingQuiesceStallCycles
            .prereq(pendingQuiesceStallCycles);
        icacheWaitRetryStallCycles
            .prereq(icacheWaitRetryStallCycles);
        icacheSquashes
            .prereq(icacheSquashes);
        tlbSquashes
            .prereq(tlbSquashes);
        nisnDist
            .init(/* base value */ 0,
              /* last value */ fetch->fetchWidth,
              /* bucket size */ 1)
            .flags(statistics::pdf);
        idleRate
            .prereq(idleRate);
}
void
Fetch::setTimeBuffer(TimeBuffer<TimeStruct> *time_buffer)
{
    timeBuffer = time_buffer;

    // Create wires to get information from proper places in time buffer.
    fromDecode = timeBuffer->getWire(-decodeToFetchDelay);
    fromRename = timeBuffer->getWire(-renameToFetchDelay);
    fromIEW = timeBuffer->getWire(-iewToFetchDelay);
    fromCommit = timeBuffer->getWire(-commitToFetchDelay);
}

void
Fetch::setActiveThreads(std::list<ThreadID> *at_ptr)
{
    activeThreads = at_ptr;
}

void
Fetch::setFetchQueue(TimeBuffer<FetchStruct> *ftb_ptr)
{
    // Create wire to write information to proper place in fetch time buf.
    toDecode = ftb_ptr->getWire(0);
}

void
Fetch::startupStage()
{
    assert(priorityList.empty());
    resetStage();

    // Fetch needs to start fetching instructions at the very beginning,
    // so it must start up in active state.
    switchToActive();
}

void
Fetch::clearStates(ThreadID tid)
{
    fetchStatus[tid] = Running;
    set(pc[tid], cpu->pcState(tid));
    seq[tid] = 1;
    brseq[tid] = 0;
    fetchOffset[tid] = 0;
    macroop[tid] = NULL;
    delayedCommit[tid] = false;
    fetchBuffer[tid].clear();
    prefetchBufferPC[tid].clear();
    prefetchFTQIndex=0;
    ftq[tid].clear();
    //memReq[tid] = NULL;
    stalls[tid].decode = false;
    stalls[tid].drain = false;
    fetchQueue[tid].clear();

    // TODO not sure what to do with priorityList for now
    // priorityList.push_back(tid);
}

void
Fetch::resetStage()
{
    numInst = 0;
    interruptPending = false;
    cacheBlocked = false;

    priorityList.clear();

    // Setup PC and nextPC with initial state.
    for (ThreadID tid = 0; tid < numThreads; ++tid) {
        fetchStatus[tid] = Running;
        set(pc[tid], cpu->pcState(tid));
        seq[tid] = 1;
        brseq[tid] = 0;
        fetchOffset[tid] = 0;
        macroop[tid] = NULL;

        delayedCommit[tid] = false;
        fetchBuffer[tid].clear();
        prefetchBufferPC[tid].clear();
        prefetchFTQIndex=0;
        ftq[tid].clear();
        //memReq[tid] = NULL;

        stalls[tid].decode = false;
        stalls[tid].drain = false;


        fetchQueue[tid].clear();

        priorityList.push_back(tid);
    }

    wroteToTimeBuffer = false;
    _status = Inactive;
}

void
Fetch::processCacheCompletion(PacketPtr pkt)
{
    ThreadID tid = cpu->contextToThread(pkt->req->contextId());

    DPRINTF(Fetch, "[tid:%i] Waking up from cache miss.\n", tid);
    assert(!cpu->switchedOut());


    //Iterate through fetchBuffer to figure out which entry the packet received
    //is serving
    fetchBufIt fb_it = fetchBuffer[tid].begin();

    while (fb_it != fetchBuffer[tid].end()){
        if ( fb_it->memReq == pkt->req) {
            break;
        }

        fb_it++;
    }

    if (fb_it == fetchBuffer[tid].end()){
        DPRINTF(FDIP, "Returning because reached "
                "end of fetchBuffer fot tid:%d\n", tid);
        ++fetchStats.icacheSquashes;
        delete pkt;
        return;
    }

    // if fetch is waiting for ICache response and recevied packet
    // is satisfying head
    // of fetchBuffer then wake up the CPU
    if ((fetchStatus[tid] == IcacheWaitResponse) &&
            fb_it == fetchBuffer[tid].begin()){
        DPRINTF(FDIP, "Wakingup CPU\n");
        cpu->wakeCPU();
        switchToActive();
        // Only switch to IcacheAccessComplete if we're not stalled as well.
        if (checkStall(tid)) {
            fetchStatus[tid] = Blocked;
        } else {
            fetchStatus[tid] = IcacheAccessComplete;
        }
    }

    //// Only change the status if it's still waiting on the icache access
    //// to return.
    //if (fetchStatus[tid] != IcacheWaitResponse ||
    //    pkt->req != memReq[tid]) {
    //    ++fetchStats.icacheSquashes;
    //    delete pkt;
    //    return;
    //}

    memcpy(fb_it->fetchBuffer.get(), pkt->getConstPtr<uint8_t>(),
            fetchBufferSize);
    fb_it->fetchBufferValid = true;

    // Wake up the CPU (if it went to sleep and was waiting on
    // this completion event).
    cpu->wakeCPU();

    DPRINTF(Activity, "[tid:%i] Activating fetch due to cache completion\n",
            tid);

    switchToActive();

    // Only switch to IcacheAccessComplete if we're not stalled as well.
    if (checkStall(tid)) {
        fetchStatus[tid] = Blocked;
    } else {
        fetchStatus[tid] = IcacheAccessComplete;
    }

    pkt->req->setAccessLatency();
    cpu->ppInstAccessComplete->notify(pkt);
    // Reset the mem req to NULL.
    delete pkt;
    fb_it->memReq = NULL;
}

void
Fetch::drainResume()
{
    for (ThreadID i = 0; i < numThreads; ++i) {
        stalls[i].decode = false;
        stalls[i].drain = false;
    }
}

void
Fetch::drainSanityCheck() const
{
    assert(isDrained());
    assert(retryPkt == NULL);
    assert(retryTid == InvalidThreadID);
    assert(!cacheBlocked);
    assert(!interruptPending);

    for (ThreadID i = 0; i < numThreads; ++i) {
        //assert(!memReq[i]);
        assert(fetchStatus[i] == Idle || stalls[i].drain);
    }

    branchPred->drainSanityCheck();
}

bool
Fetch::isDrained() const
{
    /* Make sure that threads are either idle of that the commit stage
     * has signaled that draining has completed by setting the drain
     * stall flag. This effectively forces the pipeline to be disabled
     * until the whole system is drained (simulation may continue to
     * drain other components).
     */
    for (ThreadID i = 0; i < numThreads; ++i) {
        // Verify fetch queues are drained
        if (!fetchQueue[i].empty())
            return false;

        // Return false if not idle or drain stalled
        if (fetchStatus[i] != Idle) {
            if (fetchStatus[i] == Blocked && stalls[i].drain)
                continue;
            else
                return false;
        }
    }

    /* The pipeline might start up again in the middle of the drain
     * cycle if the finish translation event is scheduled, so make
     * sure that's not the case.
     */
    return !finishTranslationEvent.scheduled();
}

void
Fetch::takeOverFrom()
{
    assert(cpu->getInstPort().isConnected());
    resetStage();

}

void
Fetch::drainStall(ThreadID tid)
{
    assert(cpu->isDraining());
    assert(!stalls[tid].drain);
    DPRINTF(Drain, "%i: Thread drained.\n", tid);
    stalls[tid].drain = true;
}

void
Fetch::wakeFromQuiesce()
{
    DPRINTF(Fetch, "Waking up from quiesce\n");
    // Hopefully this is safe
    // @todo: Allow other threads to wake from quiesce.
    fetchStatus[0] = Running;
}

void
Fetch::switchToActive()
{
    if (_status == Inactive) {
        DPRINTF(Activity, "Activating stage.\n");

        cpu->activateStage(CPU::FetchIdx);

        _status = Active;
    }
}

void
Fetch::switchToInactive()
{
    if (_status == Active) {
        DPRINTF(Activity, "Deactivating stage.\n");

        cpu->deactivateStage(CPU::FetchIdx);

        _status = Inactive;
    }
}

void
Fetch::deactivateThread(ThreadID tid)
{
    // Update priority list
    auto thread_it = std::find(priorityList.begin(), priorityList.end(), tid);
    if (thread_it != priorityList.end()) {
        priorityList.erase(thread_it);
    }
}

bool
Fetch::lookupAndUpdateNextPC(const DynInstPtr &inst, PCStateBase &next_pc)
{
    // Do branch prediction check here.
    // A bit of a misnomer...next_PC is actually the current PC until
    // this function updates it.
    ThreadID tid = inst->threadNumber;
    bool predict_taken;
    std::unique_ptr<PCStateBase> branchPC, tempPC;
    bool predictorInvoked = false;
    std::unique_ptr<PCStateBase> ftPC(next_pc.clone());
    inst->staticInst->advancePC(*ftPC);

    set(tempPC, next_pc);

    if (!inst->isControl()) {
        //inst->staticInst->advancePC(next_pc);
        inst->setBblSize(bblSize[tid]);
        inst->setBblAddr(bblAddr[tid]);
        if (!inst->isMicroop() || inst->isLastMicroop())
            bblSize[tid] += next_pc.size();
        inst->staticInst->advancePC(next_pc);
        inst->setPredTarg(next_pc);
        inst->setPredTaken(false);
        DPRINTF(Fetch, "[tid:%i] [sn:%llu, %llu] isControl %d, PFQ Size: %d, "
                "Uncond: %d, Cond: %d, Direct: %d, Addr %#x, Target %#x\n",
                tid, inst->seqNum, brseq[tid], inst->isControl(),
                ftq[tid].size(), inst->isUncondCtrl(),
                inst->isCondCtrl(), inst->isDirectCtrl(),
                inst->pcState().instAddr(), //branchPC.instAddr(),
                inst->isDirectCtrl() ? inst->branchTarget()->instAddr() : 0);
        return false;
    }

    DPRINTF(Fetch, "IsControl lookup: %d %d, %#x, %llu %llu\n",
            inst->isControl(), ftq[tid].empty(),
            inst->pcState().instAddr(), inst->seqNum, brseq[tid]);


    //Pre-decode branch instruction and update BTB
    if (enableFDIP){
        auto bblLen = inst->pcState().instAddr() - bblAddr[tid];
        if (inst->isDirectCtrl() && bblAddr[tid] != 0) {
            DPRINTF(FDIP, "BBLInsert Inserting bblAddr[tid]: %#x "
                    "instAddr: %#x branchTarget: %#x "
                    "bblSize: %d diff: %d\n",
                    bblAddr[tid], inst->pcState().instAddr(),
                    *inst->branchTarget(), bblSize[tid],
                    inst->pcState().instAddr() - bblAddr[tid]);
            std::unique_ptr<PCStateBase> brTarg = inst->branchTarget();
            branchPred->BTBUpdate(bblAddr[tid],
                                  inst->staticInst,
                                  inst->pcState(),
                                  bblLen,
                                  *brTarg,
                                  inst->isUncondCtrl(),
                                  tid);
        }
        else if (inst->isControl() && bblAddr[tid] != 0) {
            //dummyBranchTarget.pc(-1);
            //dummyBranchTarget.npc(-1);

            DPRINTF(FDIP, "BBLInsert Inserting Indirect ctrl "
                    "bblAddr[tid]: %#x instAddr: %#x "
                    "branchTarget: %#x bblSize: %d diff: %d\n",
                    bblAddr[tid], inst->pcState().instAddr(),
                    *ftPC, bblSize[tid],
                    inst->pcState().instAddr() - bblAddr[tid]);
            branchPred->BTBUpdate(bblAddr[tid],
                                  inst->staticInst,
                                  inst->pcState(),
                                  bblLen,
                                  *ftPC,
                                  inst->isUncondCtrl(),
                                  tid);
        }
    }

    if (enableFDIP){
        if (!ftq[tid].empty()) {

            // Dump FTQ here for better debuggin
            for (auto it = ftq[tid].cbegin(); it != ftq[tid].cend(); ++it){
                DPRINTF(Fetch, "beginPC: %s branchPC: %s targetPC: %s "
                        "brSeq: %llu taken: %d\n",
                        *(it->beginPC), *(it->branchPC),
                        *(it->targetPC), it->brSeq, it->isTaken);
            }

            auto &ftq_entry = ftq[tid].front();

            set(tempPC, ftq_entry.targetPC);
            set(branchPC, ftq_entry.branchPC);

            Addr beginAddr = ftq_entry.beginPC->instAddr();

            predict_taken = ftq_entry.isTaken;


            if (inst->pcState().instAddr() < branchPC->instAddr()){
                brseq[tid] = ftq_entry.brSeq;
                // squash branch predictor state to remove stale entries
                branchPred->squash(brseq[tid]-1, tid);
                //prefetchBufferPC[tid].clear();
                set(tempPC,  next_pc);


                //Remove this call to predictor after adding feature to
                //let missing branches in BPU
                predict_taken = branchPred->predict(inst->staticInst,
                                                    seq[tid],
                                                    bblAddr[tid],
                                                    *tempPC,
                                                    tid);


                brseq[tid] = seq[tid];
                seq[tid]++;
                set(prefPC[tid],tempPC);
                //predict_taken = false;
                inst->staticInst->advancePC(*tempPC);
                DPRINTF(Fetch, "New branch found before the terimination "
                        "branch of FTQ %#x, %#x\n",
                        branchPC->instAddr(), inst->pcState().instAddr());

                //assert(false && "branchpred called from fetch\n");
                // Reset prefetch Queue
                prefetchFTQIndex=0;
                ftq[tid].clear();
                stopPrefetching = true;
            }

            if (branchPC->instAddr()==inst->pcState().instAddr()){
                brseq[tid] = ftq_entry.brSeq;
                DPRINTF(Fetch, "FTQ: popping Br seq %llu\n", brseq[tid]);

                set(tempPC, ftq_entry.targetPC);

                ftq[tid].erase(ftq[tid].begin());
                prefetchFTQIndex--;
            }


            if (inst->pcState().instAddr() > branchPC->instAddr() ||
               (inst->pcState().instAddr() < (beginAddr))){
                brseq[tid] = ftq_entry.brSeq;

                DPRINTF(FDIP, "Outside the range of FTQ entry. "
                              "thisPC: %s begin: 0x%llx end: 0x%llx\n",
                              next_pc, beginAddr, branchPC->instAddr());

                // delete branchPC since it could be a bogus branch
                if (inst->pcState().instAddr() > branchPC->instAddr()){
                    DPRINTFN("Deleting potential bogus branch 0x%llx\n",
                             branchPC->instAddr());

                    //TODO: Add this function later
                    //Remove potentially bogus branches from BTB
                    //branchPred->BTBRemove(branchPC->instAddr(), tid);
                }
                DPRINTF(FDIP, "Squash and reset the FTQ state\n");
                // squash branch predictor state to remove stale entries
                branchPred->squash(brseq[tid]-1, tid);

                set(tempPC,  next_pc);
                //Remove this call to predictor after adding feature to
                //let missing branches in BPU
                predict_taken = branchPred->predict(inst->staticInst,
                                                    seq[tid],
                                                    bblAddr[tid],
                                                    *tempPC, tid);

                // Reset prefetch Queue
                prefetchFTQIndex=0;
                ftq[tid].clear();
                brseq[tid] = seq[tid];
                seq[tid]++;

                // Stop prefetching
                //*prefPC[tid] = 0;
                stopPrefetching = true;
            }


            DPRINTF(Fetch, "[tid:%i] [sn:%llu, %llu] Branch at PC %#x "
                    "predicted to go to %s %d, %#x. Queue %d\n",
                    tid, inst->seqNum, brseq[tid],
                    inst->pcState().instAddr(),
                    *tempPC, predict_taken,
                    branchPC->instAddr(), ftq[tid].size());
        } else {
            set(tempPC, next_pc);
            //TODO: reset
            DPRINTF(FDIP, "[tid:%i] FTQ is empty."
                    "Reset FTQ and go down fall through if "
                    "no branch is found\n", tid);

            DPRINTF(FDIP, "[tid:%i] FTQ is empty. "
                    "Calling predictor for brSeq: %llu\n", seq[tid]);
            //Remove this call to predictor after adding feature to
            //let missing branches in BPU
            predict_taken = branchPred->predict(inst->staticInst, seq[tid],
                                                bblAddr[tid], *tempPC, tid);


            brseq[tid] = seq[tid];
            seq[tid]++;
            stopPrefetching = true;
        }
    } else {
        //FDIP is not enabled so fallback to calling branch predictor
        DPRINTF(Fetch, "lookupAndupdate\n");
        set(tempPC, next_pc);
            DPRINTF(Fetch, "tempPC is %#x\n", tempPC->instAddr());
        predict_taken = branchPred->predict(inst->staticInst, seq[tid],
                                      bblAddr[tid], *tempPC, tid);
        brseq[tid] = seq[tid];
        seq[tid]++;
        //assert(false && "branchpred called from fetch\n");

    }

    if (!enableFDIP){
        auto bblLen = inst->pcState().instAddr() - bblAddr[tid];
        if (inst->isDirectCtrl() && bblAddr[tid] != 0) {
            DPRINTF(FDIP, "BBLInsert Inserting bblAddr[tid]: %#x "
                    "instAddr: %#x branchTarget: %#x bblSize: %d diff: %d\n",
                    bblAddr[tid], inst->pcState().instAddr(),
                    *inst->branchTarget(), bblSize[tid],
                    inst->pcState().instAddr() - bblAddr[tid]);
            std::unique_ptr<PCStateBase> brTarg = inst->branchTarget();

            branchPred->BTBUpdate(bblAddr[tid],
                                  inst->staticInst,
                                  inst->pcState(),
                                  bblLen,
                                  *brTarg,
                                  inst->isUncondCtrl(),
                                  tid);
        }
        else if (inst->isControl() && bblAddr[tid] != 0) {
            //dummyBranchTarget.pc(-1);
            //dummyBranchTarget.npc(-1);

            DPRINTF(FDIP, "BBLInsert Inserting Indirect ctrl "
                    "bblAddr[tid]: %#x instAddr: %#x branchTarget: %#x "
                    "bblSize: %d diff: %d\n",
                    bblAddr[tid], inst->pcState().instAddr(),
                    *ftPC, bblSize[tid],
                    inst->pcState().instAddr() - bblAddr[tid]);
            branchPred->BTBUpdate(bblAddr[tid],
                                  inst->staticInst,
                                  inst->pcState(),
                                  bblLen,
                                  *ftPC,
                                  inst->isUncondCtrl(),
                                  tid);
        }
    }

    if (tempPC->instAddr()<0x10) {
        inst->staticInst->advancePC(next_pc);
        predict_taken = false;
    } else {
            DPRINTF(Fetch, "setting nextPC is %#x\n", tempPC->instAddr());
        set( next_pc, *tempPC);
    }


    if (predict_taken) {
        DPRINTF(Fetch, "[tid:%i] [sn:%llu] Branch at PC %#x "
                "predicted to be taken to %s\n",
                tid, inst->seqNum, inst->pcState().instAddr(), next_pc);
    } else {
        DPRINTF(Fetch, "[tid:%i] [sn:%llu] Branch at PC %#x "
                "predicted to be not taken\n",
                tid, inst->seqNum, inst->pcState().instAddr());
    }

    DPRINTF(Fetch, "[tid:%i] [sn:%llu] Branch at PC %#x "
            "predicted to go to %s\n",
            tid, inst->seqNum, inst->pcState().instAddr(), next_pc);
    inst->setPredTarg(next_pc);
    inst->setPredTaken(predict_taken);

    cpu->fetchStats[tid]->numBranches++;

    if (predict_taken) {
        ++fetchStats.predictedBranches;
    }

    inst->setBblSize(bblSize[tid]);
    inst->setBblAddr(bblAddr[tid]);

    bblAddr[tid] = next_pc.instAddr();
    bblSize[tid] = 0;

    //TODO: If predictor is invoked when FTQ is empty
    //reset any FTQ state so that it starts prefetching at
    //correct PC
    if (enableFDIP && predictorInvoked){
        //TODO: Add this optimization later
    }

    return predict_taken;
}

bool
Fetch::fetchCacheLine(Addr vaddr, ThreadID tid, Addr pc)
{
    Fault fault = NoFault;

    assert(!cpu->switchedOut());

    // @todo: not sure if these should block translation.
    //AlphaDep
    if (cacheBlocked) {
        DPRINTF(Fetch, "[tid:%i] Can't fetch cache line, cache blocked\n",
                tid);
        return false;
    } else if (checkInterrupt(pc) && !delayedCommit[tid]) {
        // Hold off fetch from getting new instructions when:
        // Cache is blocked, or
        // while an interrupt is pending and we're not in PAL mode, or
        // fetch is switched out.
        DPRINTF(Fetch, "[tid:%i] Can't fetch cache line, interrupt pending\n",
                tid);
        return false;
    }

    // Align the fetch address to the start of a fetch buffer segment.
    Addr fetchBufferBlockPC = fetchBufferAlignPC(vaddr);

    DPRINTF(Fetch, "[tid:%i] Fetching cache line %#x for addr %#x\n",
            tid, fetchBufferBlockPC, vaddr);

    // Setup the memReq to do a read of the first instruction's address.
    // Set the appropriate read size and flags as well.
    // Build request here.
    RequestPtr mem_req = std::make_shared<Request>(
        fetchBufferBlockPC, fetchBufferSize,
        Request::INST_FETCH, cpu->instRequestorId(), pc,
        cpu->thread[tid]->contextId());

    mem_req->taskId(cpu->taskId());

    //memReq[tid] = mem_req;

    uint8_t *buf = new uint8_t[fetchBufferSize];

    std::shared_ptr<uint8_t> bufPtr(buf);


    // Initiate translation of the icache block
    if (!enableFDIP){
        fetchBuffer[tid].clear();
        prefetchBufferPC[tid].clear();
        prefetchBufferPC[tid].push_back(fetchBufferBlockPC);
        fetchBuffer[tid].emplace_back(curTick(),
                fetchBufferBlockPC, mem_req, bufPtr);
    } else {
        fetchBuffer[tid].emplace_back(curTick(),
                fetchBufferBlockPC, mem_req, bufPtr);
    }

    if (fetchBuffer[tid].size() == 1){
        fetchStatus[tid] = ItlbWait;
    }
    FetchTranslation *trans = new FetchTranslation(this);
    cpu->mmu->translateTiming(mem_req, cpu->thread[tid]->getTC(),
                              trans, BaseMMU::Execute);
    return true;
}

void
Fetch::finishTranslation(const Fault &fault, const RequestPtr &mem_req)
{
    ThreadID tid = cpu->contextToThread(mem_req->contextId());
    Addr fetchBufferBlockPC = mem_req->getVaddr();

    assert(!cpu->switchedOut());

    // Wake up CPU if it was idle
    cpu->wakeCPU();
    fetchBufIt fb_it = fetchBuffer[tid].begin();

    while (fb_it != fetchBuffer[tid].end()){
        if ( fb_it->memReq == mem_req ) {
            break;
        }

        fb_it++;
    }

    auto &thisPC = *pc[tid];
    Addr pcOffset = fetchOffset[tid];
    Addr fetchAddr = (thisPC.instAddr() + pcOffset) & decoder[tid]->pcMask();
    Addr fetchBufferExpectedPC = fetchBufferAlignPC(fetchAddr);
    bool foundPC = false;
    DPRINTF(Fetch, "fetchBuffer size is %d\n",fetchBuffer[tid].size());
    //
    //reqIt mreq_it = fetchBufferReqPtr[tid].begin();
    //validIt tr_val_it =fetchBufferTranslationValid[tid].begin();

    if (fb_it != fetchBuffer[tid].end() &&
            fb_it->fetchBufferPC == mem_req->getVaddr()){
        foundPC = true;
        if (fault == NoFault){
            fb_it->translationValid = true;
        }
    }

    if (fetchBuffer[tid].empty() || !foundPC){
        DPRINTF(Fetch, "[tid:%i] Ignoring itlb completed after squash vaddr "
                "%#x pc[tid:%i] %#x memReq[tid].empty(): %d foundPC: %d\n",
                tid, mem_req->getVaddr(), tid, thisPC.instAddr(),
                fetchBuffer[tid].empty(), foundPC);
        ++fetchStats.tlbSquashes;
        return;
    }


    //if (fetchStatus[tid] != ItlbWait || mem_req != fb_it->memReq ||
    //    mem_req->getVaddr() != fb_it->memReq->getVaddr()) {
    //    DPRINTF(Fetch, "[tid:%i] Ignoring itlb completed after squash\n",
    //            tid);
    //    ++fetchStats.tlbSquashes;
    //    return;
    //}


    // If translation was successful, attempt to read the icache block.
    if (fault == NoFault) {
        // Check that we're not going off into random memory
        // If we have, just wait around for commit to squash something and put
        // us on the right track
        if (!cpu->system->isMemAddr(mem_req->getPaddr())) {
            warn("Address %#x is outside of physical memory, stopping fetch\n",
                    mem_req->getPaddr());
            fetchStatus[tid] = NoGoodAddr;
            fb_it->memReq = NULL;
            return;
        }

        // Build packet here.
        PacketPtr data_pkt = new Packet(mem_req, MemCmd::ReadReq);
        data_pkt->dataDynamic(new uint8_t[fetchBufferSize]);

        //fetchBufferPC[tid] = fetchBufferBlockPC;
        //fetchBufferValid[tid] = false;
        DPRINTF(Fetch, "Fetch: Doing instruction read.\n");

        fetchStats.cacheLines++;

        // Access the cache.
        if (!icachePort.sendTimingReq(data_pkt)) {
                DPRINTF(Fetch, "SendTimingReq failed\n");
                if (fetchBufferBlockPC==fetchBufferExpectedPC &&
                        fetchBuffer[tid].begin() == fb_it) {

                    if (retryPkt == NULL){
                        assert(retryPkt == NULL);
                        assert(retryTid == InvalidThreadID);
                        DPRINTF(Fetch, "[tid:%i] Out of MSHRs!\n", tid);

                        fetchStatus[tid] = IcacheWaitRetry;
                        retryPkt = data_pkt;
                        retryTid = tid;
                    }
                } else {
                    //FIXME: pop only the request that failed
                    auto pos = std::distance(fetchBuffer[tid].begin(), fb_it);

                    fetchBuffer[tid].erase(fb_it, fetchBuffer[tid].end());
                    prefetchBufferPC[tid].erase(
                            prefetchBufferPC[tid].begin() + pos,
                            prefetchBufferPC[tid].end());
                    //Stop prefetching and also predecoding
                    //*prefPC[tid]=0;
                    stopPrefetching = true;
                }
                cacheBlocked = true;
        } else {
            DPRINTF(Fetch, "[tid:%i] Doing Icache access.\n", tid);
            DPRINTF(Activity, "[tid:%i] Activity: Waiting on I-cache "
                    "response.\n", tid);
            lastIcacheStall[tid] = curTick();
            fetchStatus[tid] = IcacheWaitResponse;

            // Notify Fetch Request probe when a packet containing a fetch
            // request is successfully sent
            if (fetchBufferBlockPC==fetchBufferExpectedPC &&
                    fetchBuffer[tid].begin() == fb_it)
                fetchStatus[tid] = IcacheWaitResponse;

            ppFetchRequestSent->notify(mem_req);
        }
    } else {
        // Translation Failed
        DPRINTF(Fetch, "Translation fault\n");
        if (fetchBuffer[tid].begin() != fb_it) {
            auto pos = std::distance(fetchBuffer[tid].begin(), fb_it);

            assert( fb_it->fetchBufferPC == prefetchBufferPC[tid][pos]
                    && "fetchBuffer and prefetchBuffer mismatch");
            fetchBuffer[tid].erase(fb_it, fetchBuffer[tid].end());
            prefetchBufferPC[tid].erase(prefetchBufferPC[tid].begin() + pos,
                    prefetchBufferPC[tid].end());

            //Stop prefetching
            stopPrefetching = true;
            return;
        }
        // Don't send an instruction to decode if we can't handle it.
        if (!(numInst < fetchWidth) ||
                !(fetchQueue[tid].size() < fetchQueueSize)) {
            //assert(!finishTranslationEvent.scheduled());
            if (finishTranslationEvent.scheduled()){
                return;
            }
            finishTranslationEvent.setFault(fault);
            finishTranslationEvent.setReq(mem_req);
            cpu->schedule(finishTranslationEvent,
                          cpu->clockEdge(Cycles(1)));
            return;
        }
        DPRINTF(Fetch, "TRAP: fetchBufferBlockPC: %#x and "
                "fetchBufferExpectedPC: %#x\n",
                fetchBufferBlockPC,
                fetchBufferExpectedPC);
        DPRINTF(Fetch,
                "[tid:%i] Got back req with addr %#x but expected %#x\n",
                tid, mem_req->getVaddr(), fb_it->memReq->getVaddr());
        // Translation faulted, icache request won't be sent.
        //memReq[tid] = NULL;
        fetchBuffer[tid].erase(fb_it, fetchBuffer[tid].end());
        //*prefPC[tid]=0;
        stopPrefetching = true;

        // Send the fault to commit.  This thread will not do anything
        // until commit handles the fault.  The only other way it can
        // wake up is if a squash comes along and changes the PC.
        const PCStateBase &fetch_pc = *pc[tid];

        DPRINTF(Fetch, "[tid:%i] Translation faulted, building noop.\n", tid);
        // We will use a nop in ordier to carry the fault.
        DynInstPtr instruction = buildInst(tid, nopStaticInstPtr, nullptr,
                fetch_pc, fetch_pc, false);
        instruction->setNotAnInst();
        instruction->setBrSeq(seq[tid]);
        seq[tid]++;
        instruction->setBblSize(bblSize[tid]);
        instruction->setBblAddr(bblAddr[tid]);

        instruction->setPredTarg(fetch_pc);
        instruction->fault = fault;
        wroteToTimeBuffer = true;

        DPRINTF(Activity, "Activity this cycle.\n");
        cpu->activityThisCycle();

        fetchStatus[tid] = TrapPending;

        DPRINTF(Fetch, "[tid:%i] Blocked, need to handle the trap.\n", tid);
        DPRINTF(Fetch, "[tid:%i] fault (%s) detected @ PC %s.\n",
                tid, fault->name(), *pc[tid]);
    }
    _status = updateFetchStatus();
}

void
Fetch::doSquash(const PCStateBase &new_pc, const DynInstPtr squashInst,
        ThreadID tid)
{
    DPRINTF(Fetch, "[tid:%i] Squashing, setting PC to: %s.\n",
            tid, new_pc);
    prefetchFTQIndex=0;
    ftq[tid].clear();
    fetchBuffer[tid].clear();
    prefetchBufferPC[tid].clear();

    //resume prefetching here
    stopPrefetching = false;

    set(pc[tid], new_pc);
    set(prefPC[tid], new_pc);
    fetchOffset[tid] = 0;
    if (squashInst && squashInst->pcState().instAddr() == new_pc.instAddr())
        macroop[tid] = squashInst->macroop;
    else
        macroop[tid] = NULL;
    decoder[tid]->reset();
    bblAddr[tid] = 0;

    // Clear the icache miss if it's outstanding.
    //if (fetchStatus[tid] == IcacheWaitResponse) {
    //    DPRINTF(Fetch, "[tid:%i] Squashing outstanding Icache miss.\n",
    //            tid);
    //    memReq[tid] = NULL;
    //} else if (fetchStatus[tid] == ItlbWait) {
    //    DPRINTF(Fetch, "[tid:%i] Squashing outstanding ITLB miss.\n",
    //            tid);
    //    memReq[tid] = NULL;
    //}

    // Get rid of the retrying packet if it was from this thread.
    if (retryTid == tid) {
        assert(cacheBlocked);
        if (retryPkt) {
            delete retryPkt;
        }
        retryPkt = NULL;
        retryTid = InvalidThreadID;
    }

    fetchStatus[tid] = Squashing;

    // Empty fetch queue
    fetchQueue[tid].clear();

    // microops are being squashed, it is not known wheather the
    // youngest non-squashed microop was  marked delayed commit
    // or not. Setting the flag to true ensures that the
    // interrupts are not handled when they cannot be, though
    // some opportunities to handle interrupts may be missed.
    delayedCommit[tid] = true;

    ++fetchStats.squashCycles;
}

void
Fetch::squashFromDecode(const PCStateBase &new_pc, const DynInstPtr squashInst,
        const InstSeqNum seq_num, ThreadID tid)
{
    DPRINTF(Fetch, "[tid:%i] Squashing from decode.\n", tid);

    doSquash(new_pc, squashInst, tid);

    // Tell the CPU to remove any instructions that are in flight between
    // fetch and decode.
    cpu->removeInstsUntil(seq_num, tid);
}

bool
Fetch::checkStall(ThreadID tid) const
{
    bool ret_val = false;

    if (stalls[tid].drain) {
        assert(cpu->isDraining());
        DPRINTF(Fetch,"[tid:%i] Drain stall detected.\n",tid);
        ret_val = true;
    }

    return ret_val;
}

Fetch::FetchStatus
Fetch::updateFetchStatus()
{
    //Check Running
    std::list<ThreadID>::iterator threads = activeThreads->begin();
    std::list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (fetchStatus[tid] == Running ||
            fetchStatus[tid] == Squashing ||
            fetchStatus[tid] == IcacheAccessComplete) {

            if (_status == Inactive) {
                DPRINTF(Activity, "[tid:%i] Activating stage.\n",tid);

                if (fetchStatus[tid] == IcacheAccessComplete) {
                    DPRINTF(Activity, "[tid:%i] Activating fetch due to cache"
                            "completion\n",tid);
                }

                cpu->activateStage(CPU::FetchIdx);
            }

            return Active;
        }
    }

    // Stage is switching from active to inactive, notify CPU of it.
    if (_status == Active) {
        DPRINTF(Activity, "Deactivating stage.\n");

        cpu->deactivateStage(CPU::FetchIdx);
    }

    return Inactive;
}

void
Fetch::squash(const PCStateBase &new_pc, const InstSeqNum seq_num,
        DynInstPtr squashInst, ThreadID tid)
{
    DPRINTF(Fetch, "[tid:%i] Squash from commit.\n", tid);

    doSquash(new_pc, squashInst, tid);

    // Tell the CPU to remove any instructions that are not in the ROB.
    cpu->removeInstsNotInROB(tid);
}

void
Fetch::tick()
{
    std::list<ThreadID>::iterator threads = activeThreads->begin();
    std::list<ThreadID>::iterator end = activeThreads->end();
    bool status_change = false;

    wroteToTimeBuffer = false;

    for (ThreadID i = 0; i < numThreads; ++i) {
        issuePipelinedIfetch[i] = false;
    }

    while (threads != end) {
        ThreadID tid = *threads++;

        // Check the signals for each thread to determine the proper status
        // for each thread.
        bool updated_status = checkSignalsAndUpdate(tid);
        status_change =  status_change || updated_status;
    }

    DPRINTF(Fetch, "Running stage.\n");

    if (FullSystem) {
        if (fromCommit->commitInfo[0].interruptPending) {
            interruptPending = true;
        }

        if (fromCommit->commitInfo[0].clearInterrupt) {
            interruptPending = false;
        }
    }

    for (threadFetched = 0; threadFetched < numFetchingThreads;
         threadFetched++) {
        // Fetch each of the actively fetching threads.
        if (enableFDIP){
            addToFTQ();
        }
        fetch(status_change);
    }

    // Record number of instructions fetched this cycle for distribution.
    fetchStats.nisnDist.sample(numInst);

    if (status_change) {
        // Change the fetch stage status if there was a status change.
        _status = updateFetchStatus();
    }

    // Issue the next I-cache request if possible.
    for (ThreadID i = 0; i < numThreads; ++i) {
        //if (issuePipelinedIfetch[i]) {
        if (fetchStatus[i] != Squashing) {
            pipelineIcacheAccesses(i);
        }
    }

    // Send instructions enqueued into the fetch queue to decode.
    // Limit rate by fetchWidth.  Stall if decode is stalled.
    unsigned insts_to_decode = 0;
    unsigned available_insts = 0;

    for (auto tid : *activeThreads) {
        if (!stalls[tid].decode) {
            available_insts += fetchQueue[tid].size();
        }
    }

    // Pick a random thread to start trying to grab instructions from
    auto tid_itr = activeThreads->begin();
    std::advance(tid_itr,
            random_mt.random<uint8_t>(0, activeThreads->size() - 1));

    while (available_insts != 0 && insts_to_decode < decodeWidth) {
        ThreadID tid = *tid_itr;
        if (!stalls[tid].decode && !fetchQueue[tid].empty()) {
            const auto& inst = fetchQueue[tid].front();
            toDecode->insts[toDecode->size++] = inst;
            DPRINTF(Fetch, "[tid:%i] [sn:%llu] Sending instruction to decode "
                    "from fetch queue. Fetch queue size: %i.\n",
                    tid, inst->seqNum, fetchQueue[tid].size());

            wroteToTimeBuffer = true;
            fetchQueue[tid].pop_front();
            insts_to_decode++;
            available_insts--;
        }

        tid_itr++;
        // Wrap around if at end of active threads list
        if (tid_itr == activeThreads->end())
            tid_itr = activeThreads->begin();
    }

    // If there was activity this cycle, inform the CPU of it.
    if (wroteToTimeBuffer) {
        DPRINTF(Activity, "Activity this cycle.\n");
        cpu->activityThisCycle();
    }

    // Reset the number of the instruction we've fetched.
    numInst = 0;
}

bool
Fetch::checkSignalsAndUpdate(ThreadID tid)
{
    if ((fetchStatus[tid] == IcacheWaitResponse ||
        fetchStatus[tid] == IcacheAccessComplete) &&
       fetchBuffer[tid].size()>0 &&
       fetchBuffer[tid].front().fetchBufferValid) {
        fetchStatus[tid] = Running;
    }

    // Update the per thread stall statuses.
    if (fromDecode->decodeBlock[tid]) {
        stalls[tid].decode = true;
    }

    if (fromDecode->decodeUnblock[tid]) {
        assert(stalls[tid].decode);
        assert(!fromDecode->decodeBlock[tid]);
        stalls[tid].decode = false;
    }

    // Check squash signals from commit.
    if (fromCommit->commitInfo[tid].squash) {

        DPRINTF(Fetch, "[tid:%i] Squashing instructions due to squash "
                "from commit.\n",tid);
        // In any case, squash.
        squash(*fromCommit->commitInfo[tid].pc,
               fromCommit->commitInfo[tid].doneSeqNum,
               fromCommit->commitInfo[tid].squashInst, tid);

        brseq[tid] = seq[tid];
        seq[tid]++;
        // If it was a branch mispredict on a control instruction, update the
        // branch predictor with that instruction, otherwise just kill the
        // invalid state we generated in after sequence number
        if (fromCommit->commitInfo[tid].mispredictInst &&
            fromCommit->commitInfo[tid].mispredictInst->isControl()) {

            auto mispredInst = fromCommit->commitInfo[tid].mispredictInst;
            bblAddr[tid] = fromCommit->commitInfo[tid].pc->instAddr();
            bblSize[tid] = 0;
            resteer = true;

            //branchPred->squash(fromCommit->commitInfo[tid].doneSeqNum,
            //        *fromCommit->commitInfo[tid].pc,
            //        fromCommit->commitInfo[tid].branchTaken, tid);

            auto &instPC = mispredInst->pcState();
            std::unique_ptr<PCStateBase> ftPC(instPC.clone());

            mispredInst->staticInst->advancePC(*ftPC);

            uint64_t bblSz = (instPC.instAddr() -
                    fromCommit->commitInfo[tid].bblAddr) / 4;

            DPRINTF(Fetch, "%s, %#x, %d\n",
                    mispredInst->staticInst,
                    ftPC->instAddr(),
                    bblSz);

            branchPred->squash(//fromCommit->commitInfo[tid].doneSeqNum,
                              fromCommit->commitInfo[tid].doneBrSeqNum,
                              *fromCommit->commitInfo[tid].pc,
                              fromCommit->commitInfo[tid].branchTaken,
                              tid,
                              false);
        } else {
            if (fromCommit->commitInfo[tid].doneBrSeqNum){
                branchPred->squash(fromCommit->commitInfo[tid].doneBrSeqNum,
                                  tid);
            }

        }

        return true;
    } else if (fromCommit->commitInfo[tid].doneSeqNum) {
        // Update the branch predictor if it wasn't a squashed instruction
        // that was broadcasted.
        branchPred->update(fromCommit->commitInfo[tid].doneBrSeqNum, tid);
    }

    // Check squash signals from decode.
    if (fromDecode->decodeInfo[tid].squash) {
        DPRINTF(Fetch, "[tid:%i] Squashing instructions due to squash "
                "from decode.\n",tid);

        brseq[tid]= seq[tid];
        seq[tid]++;
        // Update the branch predictor.
        if (fromDecode->decodeInfo[tid].branchMispredict) {
            resteer = true;
            bblSize[tid] = 0;

            auto mispredInst = fromDecode->decodeInfo[tid].mispredictInst;

            bblAddr[tid]=fromDecode->decodeInfo[tid].nextPC->instAddr();

            auto &instPC = mispredInst->pcState();
            std::unique_ptr<PCStateBase> ftPC(instPC.clone());
            mispredInst->staticInst->advancePC(*ftPC);

            branchPred->squash(//fromDecode->decodeInfo[tid].doneSeqNum,
                              fromDecode->decodeInfo[tid].doneBrSeqNum,
                              *fromDecode->decodeInfo[tid].nextPC,
                              fromDecode->decodeInfo[tid].branchTaken,
                              tid,
                              false);
        } else {
            bblAddr[tid] = 0;
            assert(false && "unknown case\n");
            DPRINTF(Fetch, "No idea when this will happen\n");
            branchPred->squash(fromDecode->decodeInfo[tid].doneBrSeqNum,
                              tid);
        }

        if (fetchStatus[tid] != Squashing) {

            DPRINTF(Fetch, "Squashing from decode with PC = %s\n",
                *fromDecode->decodeInfo[tid].nextPC);
            // Squash unless we're already squashing
            squashFromDecode(*fromDecode->decodeInfo[tid].nextPC,
                             fromDecode->decodeInfo[tid].squashInst,
                             fromDecode->decodeInfo[tid].doneSeqNum,
                             tid);

            bblAddr[tid]=fromDecode->decodeInfo[tid].nextPC->instAddr();
            bblSize[tid]=0;
            return true;
        }
    }

    if (checkStall(tid) &&
        fetchStatus[tid] != IcacheWaitResponse &&
        fetchBuffer[tid].size()==0 &&
        fetchStatus[tid] != IcacheWaitRetry &&
        fetchStatus[tid] != ItlbWait &&
        fetchStatus[tid] != QuiescePending) {
        DPRINTF(Fetch, "[tid:%i] Setting to blocked\n",tid);

        fetchStatus[tid] = Blocked;

        return true;
    }

    if (fetchStatus[tid] == Blocked ||
        fetchStatus[tid] == Squashing) {
        // Switch status to running if fetch isn't being told to block or
        // squash this cycle.
        DPRINTF(Fetch, "[tid:%i] Done squashing, switching to running.\n",
                tid);

        fetchStatus[tid] = Running;

        return true;
    }

    // If we've reached this point, we have not gotten any signals that
    // cause fetch to change its status.  Fetch remains the same as before.
    return false;
}

DynInstPtr
Fetch::buildInst(ThreadID tid, StaticInstPtr staticInst,
        StaticInstPtr curMacroop, const PCStateBase &this_pc,
        const PCStateBase &next_pc, bool trace)
{
    // Get a sequence number.
    InstSeqNum seq = cpu->getAndIncrementInstSeq();

    DynInst::Arrays arrays;
    arrays.numSrcs = staticInst->numSrcRegs();
    arrays.numDests = staticInst->numDestRegs();

    // Create a new DynInst from the instruction fetched.
    DynInstPtr instruction = new (arrays) DynInst(
            arrays, staticInst, curMacroop, this_pc, next_pc, seq, cpu);
    instruction->setTid(tid);

    instruction->setThreadState(cpu->thread[tid]);

    DPRINTF(Fetch, "[tid:%i] Instruction PC %s created [sn:%lli].\n",
            tid, this_pc, seq);

    DPRINTF(Fetch, "[tid:%i] Instruction is: %s\n", tid,
            instruction->staticInst->disassemble(this_pc.instAddr()));

#if TRACING_ON
    if (trace) {
        instruction->traceData =
            cpu->getTracer()->getInstRecord(curTick(), cpu->tcBase(tid),
                    instruction->staticInst, this_pc, curMacroop);
    }
#else
    instruction->traceData = NULL;
#endif

    // Add instruction to the CPU's list of instructions.
    instruction->setInstListIt(cpu->addInst(instruction));

    // Write the instruction to the first slot in the queue
    // that heads to decode.
    assert(numInst < fetchWidth);
    fetchQueue[tid].push_back(instruction);
    assert(fetchQueue[tid].size() <= fetchQueueSize);
    DPRINTF(Fetch, "[tid:%i] Fetch queue entry created (%i/%i).\n",
            tid, fetchQueue[tid].size(), fetchQueueSize);
    //toDecode->insts[toDecode->size++] = instruction;

    // Keep track of if we can take an interrupt at this boundary
    delayedCommit[tid] = instruction->isDelayedCommit();

    return instruction;
}

void
Fetch::predictNextBasicBlock(PCStateBase *prefetchPc,
        std::unique_ptr<PCStateBase> &branchPC,
        std::unique_ptr<PCStateBase> &nextPC,
        ThreadID tid, bool &stopPrefetch,
        bool &instLimitReached, bool &isTaken)
{
    std::unique_ptr<PCStateBase> predictPC;
    set(predictPC, prefetchPc);

    // Confidence based filtering code
    //auto& btbConf = cpu->btbConfMap[prefetchPc.instAddr()];
    //uint64_t &btbTotal = std::get<0>(btbConf);
    //uint64_t &btbMisPred = std::get<1>(btbConf);
    //btbTotal++;

    if (!branchPred->getBblValid(prefetchPc->instAddr(), tid)){
        //btbMisPred++;
       nextPC->reset();
       return;
    }

    DPRINTF(FDIP, "predictNextBasicBlock prefetchPC 0x%lx\n",
            prefetchPc->instAddr());

    // TODO: Multiple BBLs
    StaticInstPtr staticBranchInst =
        branchPred->getBranch(prefetchPc->instAddr(), tid);

    set(branchPC,branchPred->getBranchPC(prefetchPc->instAddr(), tid));

    //if (!staticBranchInst->isDirectCtrl()){
    //    return 0;
    //}

    if (branchPC->instAddr() < prefetchPc->instAddr()){
        DPRINTF(FDIP, "Fix this case later\n");
        nextPC->reset();
        return ;
    }

    // Limit number of instructions prefetched
    //uint64_t num_insts = (branchPC.instAddr() - prefetchPc.instAddr());

    //num_insts = (num_insts/INST_SIZE) + 1;

    //std::deque<int>::iterator it_sz = prefetchQueueBblSize[tid].begin();

    //while (it_sz!= prefetchQueueBblSize[tid].end()) {
    //    num_insts += ((*it_sz/INST_SIZE) + 1);
    //    it_sz++;
    //}

    //DPRINTF(FDIP, "insts pre-fetched %llu\n",num_insts);
    //if (num_insts > ftqInst){
    //    DPRINTF(FDIP, "ftqInst limit reached\n");
    //    instLimitReached = true;
    //    return 0;
    //}

    //TODO: Add FTQ inst limit here

    set(nextPC, branchPC);

    DPRINTF(FDIP,"nextPC instAddr is %#x\n",nextPC->instAddr());

    DPRINTF(FDIP,"TESTING branchPC: %s nextPC: %s\n", *branchPC, *nextPC);
    DPRINTF(FDIP, "Staticinst 0x%lx and BranchPC %s\n",
            staticBranchInst, *branchPC);

    bool predict_taken = branchPred->predict(staticBranchInst, seq[tid],
                                  prefetchPc->instAddr(), *nextPC, tid);


    isTaken = predict_taken;


    if (nextPC->instAddr() < 0x1000){
        set(nextPC, branchPC);
        staticBranchInst->advancePC(*nextPC);

        DPRINTF(FDIP, "Stop prefetching on unconditional not taken branch\n");
        stopPrefetch = true;
        //assert(false && "next pc is < 0x1000\n");
    }

    DPRINTF(FDIP, "prefetchPc: 0x%lx branchPC: 0x%lx nextPC: 0x%lx\n",
            prefetchPc->instAddr(), branchPC->instAddr(), nextPC->instAddr());

    if (predict_taken) {
        DPRINTF(FDIP, "[tid:%i] [sn:%llu] Branch at PC %#x "
                "predicted to be taken to %s\n",
                tid, seq[tid], branchPC->instAddr(), *nextPC);
    } else {
        DPRINTF(FDIP, "[tid:%i] [sn:%llu] Branch at PC %#x "
                "predicted to be not taken\n",
                tid, seq[tid], branchPC->instAddr());
    }

    Addr branch = branchPC->instAddr();

    set(predictPC, nextPC);

    DPRINTF(FDIP, "Prefetch predict: %#x, %s, %#x, %#x\n",
            prefetchPc->instAddr(), staticBranchInst,
            branch, predictPC->instAddr());

    if ((staticBranchInst->isUncondCtrl() ||
                staticBranchInst->isIndirectCtrl()) && !predict_taken){
        DPRINTF(FDIP, "Stop prefetching on unconditional not taken branch\n");
        stopPrefetch = true;
    }

    // Filter using branch confidence
    //auto& brConf = cpu->brConfMap[branch];
    //uint64_t &total = std::get<0>(brConf);
    //uint64_t &misPred = std::get<1>(brConf);

    //if (total > 100 && (float)misPred/total > 0.1){
    //    stopPrefetch = true;
    //}

    //return predictPC;
}

void
Fetch::addToFTQ()
{
    /////////////////////////////////////////////////
    // Add Prefetch entries to Fetch Target Queue
    /////////////////////////////////////////////////
    ThreadID tid = getFetchingThread();

    assert(!cpu->switchedOut());
    // Do not prefetch when status is TrapPending
    if ( fetchStatus[tid] == Squashing ||
         fetchStatus[tid] == TrapPending ||
         fetchStatus[tid] == QuiescePending) {
        issuePipelinedIfetch[tid] = false;
        return;
    }


    if (stopPrefetching)
        return;

    if (prefPC[tid] == 0){
        return;
    }
    if (prefPC[tid]->instAddr() < 0x10){
        return;
    }
    //assert(prefPC[tid].instAddr() != 0 && "prefPC cannot be 0\n");

    // The current Prefetch PC.
    std::unique_ptr<PCStateBase> thisPC;
    std::unique_ptr<PCStateBase> nextPC;
    std::unique_ptr<PCStateBase> branchPC;

    set(thisPC, *prefPC[tid]);
    set(nextPC, *thisPC);

    DPRINTF(Fetch, "Attempting to prefetch from [tid:%i] %i %#x\n",
            tid, ftq[tid].size(), *thisPC);

    // Keep issuing while FTQ is available
    while ( ftq[tid].size() < ftqSize ) {
        bool stopPrefetch = false;
        bool limitReached = false;
        bool taken = false;
        predictNextBasicBlock(thisPC.get(), branchPC, nextPC,
                              tid, stopPrefetch, limitReached, taken);

            // Stop prefetching if the instruction count limit is reached
        if (limitReached){
            return;
        }

        if (nextPC->instAddr()>0x10) {

            set(prefPC[tid], nextPC);

            // branchPC can be either same as thisPC or greater.
                // It cannot be lower than thisPC
            if (branchPC->instAddr() < thisPC->instAddr()){
                DPRINTF(FDIP, "should not happen branchPC: %s thisPC: %s\n",
                        *branchPC, *thisPC);
                assert(false && "Should not happen\n");
            }

            int tempBblSize = branchPred->getBblSize(thisPC->instAddr(), tid);

            if (stopPrefetch){
                //*prefPC[tid] = 0;
                stopPrefetching = true;
                DPRINTF(FDIP, "Stopping prefetching due to "
                        "stopPrefetch is set\n");

                branchPred->squash(seq[tid]-1,tid);
                seq[tid]++;
                return;
            }

            // Stop prefetching if the bbl size in the BTB is not same
            // as the computed difference
            if (tempBblSize != (branchPC->instAddr() - thisPC->instAddr())){
                    DPRINTF(FDIP, "bblSize Mismatch\n");
                    DPRINTF(FDIP, "bblSize:%d diff is %d\n",tempBblSize,
                            (branchPC->instAddr() - thisPC->instAddr()));
                    DPRINTF(FDIP, "thisPC: %s and branchPC: %s",
                            *thisPC, *branchPC);

                // stop prefetching
                //*prefPC[tid] = 0;
                stopPrefetching = true;
                //assert(false && "Check BTB parameters\n");
                branchPred->squash(seq[tid]-1,tid);
                seq[tid]++;
                return;
            }

            auto bblSize = branchPC->instAddr() - thisPC->instAddr();

            ftq[tid].emplace_back(*thisPC, *branchPC, *nextPC, seq[tid],
                                  taken, bblSize);

            seq[tid]++;

            // stop adding into FTQ when a taken branch is found
            if (taken){
                DPRINTF(FDIP, "Taken branch found in addToFTQ\n");
                break;
            }

            set(thisPC, nextPC);
            set(branchPC, nextPC);
        } else{

            //TODO: Fix this later
            break;


            ////When BBL is not found pre-fetch target and following two lines

            ////FIXME: prefPC line here and set lastAddrFetched
            //auto lastAddrFetched = 0;

            //if (!prefetchBuffer[tid].empty()){
            //    lastAddrFetched = prefetchBuffer[tid].back();
            //}
            //DPRINTF(FDIP, "lastProcessedLine %#x and lastAddrFetched %#x\n",
            //lastProcessedLine, lastAddrFetched);
            //
            //Addr fetchAddr = prefPC[tid].instAddr() & decoder[tid]->pcMask();
            //Addr fetchBufferBlockPC = fetchBufferAlignPC(fetchAddr);

            //Addr prefetchPCLine = 0;
            //if (lastAddrFetched == fetchBufferBlockPC){
            //    prefetchPCLine = fetchBufferBlockPC + CACHE_LINE_SIZE;
            //}else{
            //    prefetchPCLine = fetchBufferBlockPC;
            //}

            //prefetchBuffer[tid].push_back(prefetchPCLine);
            //prefetchBuffer[tid].push_back(prefetchPCLine + CACHE_LINE_SIZE);

            ////Also update actual PC to avoid any size mismatch issues
            //prefetchBufferActualPC[tid].push_back(prefetchPCLine);
            //prefetchBufferActualPC[tid].push_back(
            //    prefetchPCLine + CACHE_LINE_SIZE);

            //lastAddrFetched = prefetchPCLine + CACHE_LINE_SIZE;

            ////Stop prefetching till new branch is found
            //prefPC[tid]=0;

            //break;
        }
    }
 }

void
Fetch::resetFTQ(ThreadID tid)
{
    // Clear FTQ and prefetchBuffer
    // Reset all pointers

}

Addr
Fetch::alignToCacheBlock(Addr pc)
{
    Addr blockMask = ~((1ULL << CACHE_LINE_SIZE_WIDTH ) - 1);

    return pc & blockMask;
}

//NOTE: Perform this operation before FTQ entry is evicted
void
Fetch::updatePrefetchBuffer(ThreadID tid)
{
    // Do nothing if ftq is empty
    if (ftq[tid].empty()){
        DPRINTF(FDIP, "[tid:%d] Returning because ftq is empty\n", tid);
        return;
    }

    if (prefetchFTQIndex < 0 || prefetchFTQIndex >= ftq[tid].size()){
        DPRINTF(FDIP, "[tid:%d] Returning because prefetchFTQ is out of range "
                "prefetchFTQIndex %d ftq size %d \n",
                tid,
                prefetchFTQIndex,
                ftq[tid].size());
        return;
    }

    Addr lastFetchedBlock = 0;
    // if index to prefetch is 0 then insert cachelines without any checks
    // else check if the last line is same as the line being prefetched
    // if so then skip that line
    if (prefetchFTQIndex != 0){
        lastFetchedBlock = ftq[tid][prefetchFTQIndex - 1].branchPC->instAddr();

        lastFetchedBlock = alignToCacheBlock(lastFetchedBlock);
    }

    //TODO: check if prefetchFTQIndex reached end of ftq

    Addr curPCLine = ftq[tid][prefetchFTQIndex].beginPC->instAddr();
    curPCLine = alignToCacheBlock(curPCLine);

    Addr branchPCLine = ftq[tid][prefetchFTQIndex].branchPC->instAddr();
    branchPCLine += ftq[tid][prefetchFTQIndex].branchPC->size();
    branchPCLine = alignToCacheBlock(branchPCLine);

    //TODO: In X86 branchPC could go over onto the next cache line

    if (lastFetchedBlock && curPCLine == lastFetchedBlock){
        DPRINTF(FDIP, "skipping curPCLine %#x\n",curPCLine);
        curPCLine += CACHE_LINE_SIZE;
    }

    while (curPCLine <= branchPCLine){
        DPRINTF(FDIP, "curPCLine %#x and branchPCLine %#x\n",
                curPCLine, branchPCLine);
        curPCLine += CACHE_LINE_SIZE;
        //actualPC = curPCLine;
    }

    // Update FTQ Index
    prefetchFTQIndex++;
}

void
Fetch::fetch(bool &status_change)
{
    //////////////////////////////////////////
    // Start actual fetch
    //////////////////////////////////////////
    ThreadID tid = getFetchingThread();

    assert(!cpu->switchedOut());

    if (tid == InvalidThreadID) {
        // Breaks looping condition in tick()
        threadFetched = numFetchingThreads;

        if (numThreads == 1) {  // @todo Per-thread stats
            profileStall(0);
        }

        return;
    }

    DPRINTF(Fetch, "Attempting to fetch from [tid:%i]\n", tid);
    DPRINTF(Fetch, "fetchStatus is %d\n", fetchStatus[tid]);
    DPRINTF(Fetch, "FTQ size %d fetchBuffer size %d prefetchBuffer size %d\n",
            ftq[tid].size(), fetchBuffer[tid].size(),
            prefetchBufferPC[tid].size());

    // The current PC.
    PCStateBase &this_pc = *pc[tid];

    Addr pcOffset = fetchOffset[tid];
    Addr fetchAddr = (this_pc.instAddr() + pcOffset) & decoder[tid]->pcMask();

    bool inRom = isRomMicroPC(this_pc.microPC());
    Addr fetchBufferBlockPC = fetchBufferAlignPC(fetchAddr);

    // If returning from the delay of a cache miss, then update the status
    // to running, otherwise do the cache access.  Possibly move this up
    // to tick() function.
    if (fetchStatus[tid] == IcacheAccessComplete) {
        DPRINTF(Fetch, "[tid:%i] Icache miss is complete.\n", tid);

        fetchStatus[tid] = Running;
        status_change = true;
        if (fetchBuffer[tid].size() > 0 &&
                !fetchBuffer[tid].front().fetchBufferValid){
            return;
        }
    } else if (fetchStatus[tid] == Running) {
        // Align the fetch PC so its at the start of a fetch buffer segment.
        Addr fetchBufferBlockPC = fetchBufferAlignPC(fetchAddr);
        auto fb_head = fetchBuffer[tid].begin();

        // If buffer is no longer valid or fetchAddr has moved to point
        // to the next cache block, AND we have no remaining ucode
        // from a macro-op, then start fetch from icache.
        if (!macroop[tid] && !inRom){

            if (!(fetchBuffer[tid].size()>0 &&
                        fb_head->fetchBufferValid &&
                        fb_head->fetchBufferPC == fetchBufferBlockPC)
                ) {

                DPRINTF(Fetch, "[tid:%i] Attempting to translate and read "
                        "instruction, starting at PC %s.\n", tid, this_pc);

                if (fetchBuffer[tid].empty() ||
                        fb_head->fetchBufferPC != fetchBufferBlockPC) {
                    if (!enableFDIP){
                        fetchCacheLine(fetchAddr, tid, this_pc.instAddr());
                    }
                }

                if (fetchStatus[tid] == IcacheWaitResponse) {
                    cpu->fetchStats[tid]->icacheStallCycles++;
                }
                else if (fetchStatus[tid] == ItlbWait)
                    ++fetchStats.tlbCycles;
                else
                    ++fetchStats.miscStallCycles;

                return;

            } else if (checkInterrupt(this_pc.instAddr()) &&
                    !delayedCommit[tid]) {
                // Stall CPU if an interrupt is posted and we're not issuing
                // an delayed commit micro-op currently (delayed commit
                // instructions are not interruptable by interrupts, only
                // faults)
                ++fetchStats.miscStallCycles;
                DPRINTF(Fetch, "[tid:%i] Fetch is stalled!\n", tid);
                return;
            }
        }
    } else if (fetchStatus[tid] == TrapPending ||
            fetchStatus[tid] == QuiescePending){
        return;
    } else if (fetchBuffer[tid].size()>0 &&
            fetchBuffer[tid].front().fetchBufferValid &&
            fetchBufferBlockPC == fetchBuffer[tid].front().fetchBufferPC) {
        DPRINTF(Fetch, "[tid:%i] Nayana added.\n", tid);

        fetchStatus[tid] = Running;
        status_change = true;
    } else {
        if (fetchStatus[tid] == Idle) {
            ++fetchStats.idleCycles;
            DPRINTF(Fetch, "[tid:%i] Fetch is idle!\n", tid);
        }

        // Status is Idle, so fetch should do nothing.
        return;
    }

    ++fetchStats.cycles;

    std::unique_ptr<PCStateBase> next_pc(this_pc.clone());

    StaticInstPtr staticInst = NULL;
    StaticInstPtr curMacroop = macroop[tid];

    // If the read of the first instruction was successful, then grab the
    // instructions from the rest of the cache line and put them into the
    // queue heading to decode.

    DPRINTF(Fetch, "[tid:%i] Adding instructions to queue to "
            "decode.\n", tid);

    // Need to keep track of whether or not a predicted branch
    // ended this fetch block.
    bool predictedBranch = false;

    // Need to halt fetch if quiesce instruction detected
    bool quiesce = false;

    uint8_t* fetchBufferHead = NULL;
    if (fetchBuffer[tid].empty()){
        DPRINTF(FDIP, "fetchBuffer is empty so returning from fetch\n");
    }else{
        fetchBufferHead = fetchBuffer[tid].front().fetchBuffer.get();
    }

    unsigned blkOffset =
        (fetchAddr - fetchBufferBlockPC) / instSize;

    const unsigned numInsts = fetchBufferSize / instSize;

    auto *dec_ptr = decoder[tid];
    const Addr pc_mask = dec_ptr->pcMask();

    // Loop through instruction memory from the cache.
    // Keep issuing while fetchWidth is available and branch is not
    // predicted taken
    while (numInst < fetchWidth && fetchQueue[tid].size() < fetchQueueSize
           && !predictedBranch && !quiesce) {
        // We need to process more memory if we aren't going to get a
        // StaticInst from the rom, the current macroop, or what's already
        // in the decoder.
        bool needMem = !inRom && !curMacroop && !dec_ptr->instReady();
        fetchAddr = (this_pc.instAddr() + pcOffset) & pc_mask;
        Addr fetchBufferBlockPC = fetchBufferAlignPC(fetchAddr);
        auto &fBuf = fetchBuffer[tid].front();

        if (needMem) {

            if (fetchBuffer[tid].size() == 0){
                warn("Fix this case!");
                break;
            }
            // If buffer is no longer valid or fetchAddr has moved to point
            // to the next cache block then start fetch from icache.
            if (!fBuf.fetchBufferValid ||
                fetchBufferBlockPC != fBuf.fetchBufferPC)
                break;

            if (blkOffset >= numInsts) {
                // We need to process more memory, but we've run out of the
                // current block.
                //pcOffset = 0;
                break;
            }

            memcpy(dec_ptr->moreBytesPtr(),
                    fetchBufferHead + blkOffset * instSize, instSize);
            decoder[tid]->moreBytes(this_pc, fetchAddr);

            if (dec_ptr->needMoreBytes()) {
                blkOffset++;
                fetchAddr += instSize;
                pcOffset += instSize;
            }
        }

        // Extract as many instructions and/or microops as we can from
        // the memory we've processed so far.
        do {
            if (!(curMacroop || inRom)) {
                if (dec_ptr->instReady()) {
                    staticInst = dec_ptr->decode(this_pc);

                    // Increment stat of fetched instructions.
                    cpu->fetchStats[tid]->numInsts++;

                    if (staticInst->isMacroop()) {
                        curMacroop = staticInst;
                    } else {
                        pcOffset = 0;
                    }
                } else {
                    // We need more bytes for this instruction so blkOffset and
                    // pcOffset will be updated
                    break;
                }
            }
            // Whether we're moving to a new macroop because we're at the
            // end of the current one, or the branch predictor incorrectly
            // thinks we are...
            bool newMacro = false;
            if (curMacroop || inRom) {
                if (inRom) {
                    staticInst = dec_ptr->fetchRomMicroop(
                            this_pc.microPC(), curMacroop);
                } else {
                    staticInst = curMacroop->fetchMicroop(this_pc.microPC());
                }
                newMacro |= staticInst->isLastMicroop();
            }

            DynInstPtr instruction = buildInst(
                    tid, staticInst, curMacroop, this_pc, *next_pc, true);

            ppFetch->notify(instruction);
            numInst++;

#if TRACING_ON
            if (debug::O3PipeView) {
                instruction->fetchTick = curTick();
            }
#endif

            set(next_pc, this_pc);

            // If we're branching after this instruction, quit fetching
            // from the same block.
            predictedBranch |= this_pc.branching();
            predictedBranch |= lookupAndUpdateNextPC(instruction, *next_pc);
            instruction->setBrSeq(brseq[tid]);
            if (predictedBranch) {
                DPRINTF(Fetch, "Branch detected with PC = %s\n", this_pc);
            }

            newMacro |= this_pc.instAddr() != next_pc->instAddr();

            // Move to the next instruction, unless we have a branch.
            set(this_pc, *next_pc);
            inRom = isRomMicroPC(this_pc.microPC());

            if (newMacro) {
                fetchAddr = this_pc.instAddr() & pc_mask;
                fetchBufferBlockPC = fetchBufferAlignPC(fetchAddr);
                blkOffset = (fetchAddr - fetchBufferBlockPC) / instSize;
                pcOffset = 0;
                curMacroop = NULL;
            }

            if (instruction->isQuiesce()) {
                DPRINTF(Fetch,
                        "Quiesce instruction encountered, halting fetch!\n");
                fetchStatus[tid] = QuiescePending;
                status_change = true;
                quiesce = true;
                break;
            }
        } while ((curMacroop || dec_ptr->instReady()) &&
                 numInst < fetchWidth &&
                 fetchQueue[tid].size() < fetchQueueSize);

        // Re-evaluate whether the next instruction to fetch is in micro-op ROM
        // or not.
        inRom = isRomMicroPC(this_pc.microPC());
    }

    if (predictedBranch) {
        DPRINTF(Fetch, "[tid:%i] Done fetching, predicted branch "
                "instruction encountered.\n", tid);
    } else if (numInst >= fetchWidth) {
        DPRINTF(Fetch, "[tid:%i] Done fetching, reached fetch bandwidth "
                "for this cycle.\n", tid);
    } else if (blkOffset >= fetchBufferSize) {
        DPRINTF(Fetch, "[tid:%i] Done fetching, reached the end of the"
                "fetch buffer.\n", tid);
    }

    macroop[tid] = curMacroop;
    fetchOffset[tid] = pcOffset;

    if (numInst > 0) {
        wroteToTimeBuffer = true;
    }

    // pipeline a fetch if we're crossing a fetch buffer boundary and not in
    // a state that would preclude fetching
    fetchAddr = (this_pc.instAddr() + pcOffset) & pc_mask;
    fetchBufferBlockPC = fetchBufferAlignPC(fetchAddr);
    //issuePipelinedIfetch[tid] = fetchBufferBlockPC != fetchBufferPC[tid] &&
    //    fetchStatus[tid] != IcacheWaitResponse &&
    //    fetchStatus[tid] != ItlbWait &&
    //    fetchStatus[tid] != IcacheWaitRetry &&
    //    fetchStatus[tid] != QuiescePending &&
    //    !curMacroop;

    auto &fBuf = fetchBuffer[tid].front();

    if (!fetchBuffer[tid].empty() &&
            fetchBufferBlockPC != fBuf.fetchBufferPC && !curMacroop) {

        assert(!curMacroop && "Curmacroop should not be not null!");
        fetchBuffer[tid].erase(fetchBuffer[tid].begin());

        if (prefetchBufferPC[tid].size()>0){
          prefetchBufferPC[tid].erase(prefetchBufferPC[tid].begin());
        }

        DPRINTF(Fetch, "[tid:%i] Popping queue %d\n",
                tid, fetchBuffer[tid].size());

        if (prefetchBufferPC[tid].size()>0 &&
                fetchBufferBlockPC != prefetchBufferPC[tid].front() &&
                !curMacroop){

            DPRINTF(Fetch, "Front is still not same. fetchBufferBlockPC: %#x "
                    "fetchBufferPC: %#x\n",
                    fetchBufferBlockPC,
                    fetchBuffer[tid].front().fetchBufferPC);

            // Do not clear these queues here or else
            //stale branches will not be squashed


            prefetchBufferPC[tid].clear();
            fetchBuffer[tid].clear();
        }
    }
}

void
Fetch::recvReqRetry()
{
    if (retryPkt != NULL) {
        assert(cacheBlocked);
        assert(retryTid != InvalidThreadID);
        assert(fetchStatus[retryTid] == IcacheWaitRetry);

        if (icachePort.sendTimingReq(retryPkt)) {
            fetchStatus[retryTid] = IcacheWaitResponse;
            // Notify Fetch Request probe when a retryPkt is successfully sent.
            // Note that notify must be called before retryPkt is set to NULL.
            ppFetchRequestSent->notify(retryPkt->req);
            retryPkt = NULL;
            retryTid = InvalidThreadID;
            cacheBlocked = false;
        }
    } else {
        assert(retryTid == InvalidThreadID);
        // Access has been squashed since it was sent out.  Just clear
        // the cache being blocked.
        cacheBlocked = false;
    }
}

///////////////////////////////////////
//                                   //
//  SMT FETCH POLICY MAINTAINED HERE //
//                                   //
///////////////////////////////////////
ThreadID
Fetch::getFetchingThread()
{
    if (numThreads > 1) {
        switch (fetchPolicy) {
          case SMTFetchPolicy::RoundRobin:
            return roundRobin();
          case SMTFetchPolicy::IQCount:
            return iqCount();
          case SMTFetchPolicy::LSQCount:
            return lsqCount();
          case SMTFetchPolicy::Branch:
            return branchCount();
          default:
            return InvalidThreadID;
        }
    } else {
        std::list<ThreadID>::iterator thread = activeThreads->begin();
        if (thread == activeThreads->end()) {
            return InvalidThreadID;
        }

        ThreadID tid = *thread;

        if (fetchStatus[tid] == Running ||
            fetchStatus[tid] == IcacheAccessComplete ||
            fetchStatus[tid] == Idle) {
            return tid;
        } else {
            return InvalidThreadID;
        }
    }
}


ThreadID
Fetch::roundRobin()
{
    std::list<ThreadID>::iterator pri_iter = priorityList.begin();
    std::list<ThreadID>::iterator end      = priorityList.end();

    ThreadID high_pri;

    while (pri_iter != end) {
        high_pri = *pri_iter;

        assert(high_pri <= numThreads);

        if (fetchStatus[high_pri] == Running ||
            fetchStatus[high_pri] == IcacheAccessComplete ||
            fetchStatus[high_pri] == Idle) {

            priorityList.erase(pri_iter);
            priorityList.push_back(high_pri);

            return high_pri;
        }

        pri_iter++;
    }

    return InvalidThreadID;
}

ThreadID
Fetch::iqCount()
{
    //sorted from lowest->highest
    std::priority_queue<unsigned, std::vector<unsigned>,
                        std::greater<unsigned> > PQ;
    std::map<unsigned, ThreadID> threadMap;

    std::list<ThreadID>::iterator threads = activeThreads->begin();
    std::list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;
        unsigned iqCount = fromIEW->iewInfo[tid].iqCount;

        //we can potentially get tid collisions if two threads
        //have the same iqCount, but this should be rare.
        PQ.push(iqCount);
        threadMap[iqCount] = tid;
    }

    while (!PQ.empty()) {
        ThreadID high_pri = threadMap[PQ.top()];

        if (fetchStatus[high_pri] == Running ||
            fetchStatus[high_pri] == IcacheAccessComplete ||
            fetchStatus[high_pri] == Idle)
            return high_pri;
        else
            PQ.pop();

    }

    return InvalidThreadID;
}

ThreadID
Fetch::lsqCount()
{
    //sorted from lowest->highest
    std::priority_queue<unsigned, std::vector<unsigned>,
                        std::greater<unsigned> > PQ;
    std::map<unsigned, ThreadID> threadMap;

    std::list<ThreadID>::iterator threads = activeThreads->begin();
    std::list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;
        unsigned ldstqCount = fromIEW->iewInfo[tid].ldstqCount;

        //we can potentially get tid collisions if two threads
        //have the same iqCount, but this should be rare.
        PQ.push(ldstqCount);
        threadMap[ldstqCount] = tid;
    }

    while (!PQ.empty()) {
        ThreadID high_pri = threadMap[PQ.top()];

        if (fetchStatus[high_pri] == Running ||
            fetchStatus[high_pri] == IcacheAccessComplete ||
            fetchStatus[high_pri] == Idle)
            return high_pri;
        else
            PQ.pop();
    }

    return InvalidThreadID;
}

ThreadID
Fetch::branchCount()
{
    panic("Branch Count Fetch policy unimplemented\n");
    return InvalidThreadID;
}

void
Fetch::pipelineIcacheAccesses(ThreadID tid)
{
    //if (!issuePipelinedIfetch[tid]) {
    //    return;
    //}
    //if (!issuePipelinedIfetch[tid]) {
    //if (fetchStatus[tid] == ItlbWait
    //    || fetchStatus[tid] == IcacheWaitResponse
    //    || fetchStatus[tid] == IcacheWaitRetry){
    //    return;
    //}


    if (!enableFDIP)
        return;

    if (fetchStatus[tid] == IcacheWaitRetry
        || fetchStatus[tid] == TrapPending
        || fetchStatus[tid] == QuiescePending){
        DPRINTF(FDIP, "returning due to fetchStatus[tid] %d\n",
                fetchStatus[tid]);
        return;
    }

    updatePrefetchBuffer(tid);

    //if (prefetchQueue[tid].empty()) {
    //    return;
    //}

    const PCStateBase &thisPC = *pc[tid];
    bool inRom = isRomMicroPC(thisPC.microPC());

    if (!prefetchBufferPC[tid].empty()){
        Addr curPCLine = (thisPC.instAddr() + fetchOffset[tid]);
        curPCLine = alignToCacheBlock(curPCLine);
        curPCLine = fetchBufferAlignPC(curPCLine);

        //Sanity check: fetch head must be same as prefetchBufferPC
        if (curPCLine != prefetchBufferPC[tid].front()){
            DPRINTF(FDIP, "BUG! fetch at %#x and prefetchBuffer at %#x "
                    "tick: %llu\n",thisPC.instAddr(),
                    prefetchBufferPC[tid].front(), curTick());
        }
    }
    if (prefetchBufferPC[tid].empty() && !inRom && !macroop[tid]){
        Addr curPCLine = (thisPC.instAddr() + fetchOffset[tid]);
        curPCLine = alignToCacheBlock(curPCLine);
        DPRINTF(Fetch,"pipelineIcache addToFTQ pc[tid] is %#x\n",
                thisPC.instAddr());
        prefetchBufferPC[tid].push_back(curPCLine);
        if (!ftq[tid].empty()){
            DPRINTF(FDIP, "Figure out why this is happening\n");
            for (auto &it : ftq[tid]){
                DPRINTF(FDIP, "beginPC: %s branchPC: %s targetPC: %s "
                        "brSeq: %llu taken: %d\n",
                        *(it.beginPC), *(it.branchPC),
                        *(it.targetPC), it.brSeq, it.isTaken);
            }
        }
    }
    //If prefetch buffer is empty then fetch head
    //of the PC and memReq queue is empty
    if (prefetchBufferPC[tid].empty()){
        DPRINTF(FDIP, "prefetchBufferPC[tid:%d] is empty\n", tid);
        return;
    }

    fetchBufIt pc_it = fetchBuffer[tid].begin();
    pcIt pref_pc_it = prefetchBufferPC[tid].begin();

    DPRINTF(Fetch, "Iterating through prefetchBufferPC\n");
    while (pc_it != fetchBuffer[tid].end() &&
            pref_pc_it != prefetchBufferPC[tid].end() &&
            (*pref_pc_it) == pc_it->fetchBufferPC &&
            pc_it->translationValid){
        DPRINTF(Fetch, "%#x\n", pc_it->fetchBufferPC);
        pref_pc_it++;
        pc_it++;
    }

    if (pc_it != fetchBuffer[tid].end()){
        DPRINTF(Fetch, "Something is wrong\n");
        //assert(false && "pc_it is not the end of the fetchBufferPC\n");
        return;
    }

    if (pref_pc_it != prefetchBufferPC[tid].end()){
        assert(pc_it == fetchBuffer[tid].end() &&
                " pc_it is not the end of the fetchBufferPC\n");
        DPRINTF(Fetch, "Issuing a pipelined access %#x\n", *pref_pc_it);
        fetchCacheLine(*pref_pc_it, tid, *pref_pc_it);
    }
    // Original Code

    // // The next PC to access.
    // const PCStateBase &this_pc = *pc[tid];

    // if (isRomMicroPC(this_pc.microPC())) {
    //     return;
    // }

    // Addr pcOffset = fetchOffset[tid];
    // Addr fetchAddr = (this_pc.instAddr() + pcOffset) &
    //     decoder[tid]->pcMask();

    // // Align the fetch PC so its at the start of a fetch buffer segment.
    // Addr fetchBufferBlockPC = fetchBufferAlignPC(fetchAddr);

    // // Unless buffer already got the block, fetch it from icache.
    // if (!(fetchBufferValid[tid] &&
    //         fetchBufferBlockPC == fetchBufferPC[tid])
    //         ) {
    //
    //     DPRINTF(Fetch, "[tid:%i] Issuing a pipelined I-cache access, "
    //             "starting at PC %s.\n", tid, this_pc);

    //     fetchCacheLine(fetchAddr, tid, this_pc.instAddr());
    // }
}

void
Fetch::profileStall(ThreadID tid)
{
    DPRINTF(Fetch,"There are no more threads available to fetch from.\n");

    // @todo Per-thread stats

    if (stalls[tid].drain) {
        ++fetchStats.pendingDrainCycles;
        DPRINTF(Fetch, "Fetch is waiting for a drain!\n");
    } else if (activeThreads->empty()) {
        ++fetchStats.noActiveThreadStallCycles;
        DPRINTF(Fetch, "Fetch has no active thread!\n");
    } else if (fetchStatus[tid] == Blocked) {
        ++fetchStats.blockedCycles;
        DPRINTF(Fetch, "[tid:%i] Fetch is blocked!\n", tid);
    } else if (fetchStatus[tid] == Squashing) {
        ++fetchStats.squashCycles;
        DPRINTF(Fetch, "[tid:%i] Fetch is squashing!\n", tid);
    } else if (fetchStatus[tid] == IcacheWaitResponse) {
        cpu->fetchStats[tid]->icacheStallCycles++;
        DPRINTF(Fetch, "[tid:%i] Fetch is waiting cache response!\n",
                tid);
    } else if (fetchStatus[tid] == ItlbWait) {
        ++fetchStats.tlbCycles;
        DPRINTF(Fetch, "[tid:%i] Fetch is waiting ITLB walk to "
                "finish!\n", tid);
    } else if (fetchStatus[tid] == TrapPending) {
        ++fetchStats.pendingTrapStallCycles;
        DPRINTF(Fetch, "[tid:%i] Fetch is waiting for a pending trap!\n",
                tid);
    } else if (fetchStatus[tid] == QuiescePending) {
        ++fetchStats.pendingQuiesceStallCycles;
        DPRINTF(Fetch, "[tid:%i] Fetch is waiting for a pending quiesce "
                "instruction!\n", tid);
    } else if (fetchStatus[tid] == IcacheWaitRetry) {
        ++fetchStats.icacheWaitRetryStallCycles;
        DPRINTF(Fetch, "[tid:%i] Fetch is waiting for an I-cache retry!\n",
                tid);
    } else if (fetchStatus[tid] == NoGoodAddr) {
            DPRINTF(Fetch, "[tid:%i] Fetch predicted non-executable address\n",
                    tid);
    } else {
        DPRINTF(Fetch, "[tid:%i] Unexpected fetch stall reason "
            "(Status: %i)\n",
            tid, fetchStatus[tid]);
    }
}

bool
Fetch::IcachePort::recvTimingResp(PacketPtr pkt)
{
    DPRINTF(O3CPU, "Fetch unit received timing\n");
    // We shouldn't ever get a cacheable block in Modified state
    assert(pkt->req->isUncacheable() ||
           !(pkt->cacheResponding() && !pkt->hasSharers()));
    fetch->processCacheCompletion(pkt);

    return true;
}

void
Fetch::IcachePort::recvReqRetry()
{
    fetch->recvReqRetry();
}

} // namespace o3
} // namespace gem5
