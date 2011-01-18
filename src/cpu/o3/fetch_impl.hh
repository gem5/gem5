/*
 * Copyright (c) 2010 ARM Limited
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
 *
 * Authors: Kevin Lim
 *          Korey Sewell
 */

#include <algorithm>
#include <cstring>

#include "arch/isa_traits.hh"
#include "arch/utility.hh"
#include "base/types.hh"
#include "config/the_isa.hh"
#include "config/use_checker.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/exetrace.hh"
#include "cpu/o3/fetch.hh"
#include "mem/packet.hh"
#include "mem/request.hh"
#include "params/DerivO3CPU.hh"
#include "sim/byteswap.hh"
#include "sim/core.hh"

#if FULL_SYSTEM
#include "arch/tlb.hh"
#include "arch/vtophys.hh"
#include "sim/system.hh"
#endif // FULL_SYSTEM

using namespace std;

template<class Impl>
void
DefaultFetch<Impl>::IcachePort::setPeer(Port *port)
{
    Port::setPeer(port);

    fetch->setIcache();
}

template<class Impl>
Tick
DefaultFetch<Impl>::IcachePort::recvAtomic(PacketPtr pkt)
{
    panic("DefaultFetch doesn't expect recvAtomic callback!");
    return curTick();
}

template<class Impl>
void
DefaultFetch<Impl>::IcachePort::recvFunctional(PacketPtr pkt)
{
    DPRINTF(Fetch, "DefaultFetch doesn't update its state from a "
            "functional call.");
}

template<class Impl>
void
DefaultFetch<Impl>::IcachePort::recvStatusChange(Status status)
{
    if (status == RangeChange) {
        if (!snoopRangeSent) {
            snoopRangeSent = true;
            sendStatusChange(Port::RangeChange);
        }
        return;
    }

    panic("DefaultFetch doesn't expect recvStatusChange callback!");
}

template<class Impl>
bool
DefaultFetch<Impl>::IcachePort::recvTiming(PacketPtr pkt)
{
    DPRINTF(Fetch, "Received timing\n");
    if (pkt->isResponse()) {
        fetch->processCacheCompletion(pkt);
    }
    //else Snooped a coherence request, just return
    return true;
}

template<class Impl>
void
DefaultFetch<Impl>::IcachePort::recvRetry()
{
    fetch->recvRetry();
}

template<class Impl>
DefaultFetch<Impl>::DefaultFetch(O3CPU *_cpu, DerivO3CPUParams *params)
    : cpu(_cpu),
      branchPred(params),
      predecoder(NULL),
      decodeToFetchDelay(params->decodeToFetchDelay),
      renameToFetchDelay(params->renameToFetchDelay),
      iewToFetchDelay(params->iewToFetchDelay),
      commitToFetchDelay(params->commitToFetchDelay),
      fetchWidth(params->fetchWidth),
      cacheBlocked(false),
      retryPkt(NULL),
      retryTid(InvalidThreadID),
      numThreads(params->numThreads),
      numFetchingThreads(params->smtNumFetchingThreads),
      interruptPending(false),
      drainPending(false),
      switchedOut(false)
{
    if (numThreads > Impl::MaxThreads)
        fatal("numThreads (%d) is larger than compiled limit (%d),\n"
              "\tincrease MaxThreads in src/cpu/o3/impl.hh\n",
              numThreads, static_cast<int>(Impl::MaxThreads));

    // Set fetch stage's status to inactive.
    _status = Inactive;

    std::string policy = params->smtFetchPolicy;

    // Convert string to lowercase
    std::transform(policy.begin(), policy.end(), policy.begin(),
                   (int(*)(int)) tolower);

    // Figure out fetch policy
    if (policy == "singlethread") {
        fetchPolicy = SingleThread;
        if (numThreads > 1)
            panic("Invalid Fetch Policy for a SMT workload.");
    } else if (policy == "roundrobin") {
        fetchPolicy = RoundRobin;
        DPRINTF(Fetch, "Fetch policy set to Round Robin\n");
    } else if (policy == "branch") {
        fetchPolicy = Branch;
        DPRINTF(Fetch, "Fetch policy set to Branch Count\n");
    } else if (policy == "iqcount") {
        fetchPolicy = IQ;
        DPRINTF(Fetch, "Fetch policy set to IQ count\n");
    } else if (policy == "lsqcount") {
        fetchPolicy = LSQ;
        DPRINTF(Fetch, "Fetch policy set to LSQ count\n");
    } else {
        fatal("Invalid Fetch Policy. Options Are: {SingleThread,"
              " RoundRobin,LSQcount,IQcount}\n");
    }

    // Get the size of an instruction.
    instSize = sizeof(TheISA::MachInst);

    // Name is finally available, so create the port.
    icachePort = new IcachePort(this);

    icachePort->snoopRangeSent = false;

#if USE_CHECKER
    if (cpu->checker) {
        cpu->checker->setIcachePort(icachePort);
    }
#endif
}

template <class Impl>
std::string
DefaultFetch<Impl>::name() const
{
    return cpu->name() + ".fetch";
}

template <class Impl>
void
DefaultFetch<Impl>::regStats()
{
    icacheStallCycles
        .name(name() + ".icacheStallCycles")
        .desc("Number of cycles fetch is stalled on an Icache miss")
        .prereq(icacheStallCycles);

    fetchedInsts
        .name(name() + ".Insts")
        .desc("Number of instructions fetch has processed")
        .prereq(fetchedInsts);

    fetchedBranches
        .name(name() + ".Branches")
        .desc("Number of branches that fetch encountered")
        .prereq(fetchedBranches);

    predictedBranches
        .name(name() + ".predictedBranches")
        .desc("Number of branches that fetch has predicted taken")
        .prereq(predictedBranches);

    fetchCycles
        .name(name() + ".Cycles")
        .desc("Number of cycles fetch has run and was not squashing or"
              " blocked")
        .prereq(fetchCycles);

    fetchSquashCycles
        .name(name() + ".SquashCycles")
        .desc("Number of cycles fetch has spent squashing")
        .prereq(fetchSquashCycles);

    fetchIdleCycles
        .name(name() + ".IdleCycles")
        .desc("Number of cycles fetch was idle")
        .prereq(fetchIdleCycles);

    fetchBlockedCycles
        .name(name() + ".BlockedCycles")
        .desc("Number of cycles fetch has spent blocked")
        .prereq(fetchBlockedCycles);

    fetchedCacheLines
        .name(name() + ".CacheLines")
        .desc("Number of cache lines fetched")
        .prereq(fetchedCacheLines);

    fetchMiscStallCycles
        .name(name() + ".MiscStallCycles")
        .desc("Number of cycles fetch has spent waiting on interrupts, or "
              "bad addresses, or out of MSHRs")
        .prereq(fetchMiscStallCycles);

    fetchIcacheSquashes
        .name(name() + ".IcacheSquashes")
        .desc("Number of outstanding Icache misses that were squashed")
        .prereq(fetchIcacheSquashes);

    fetchNisnDist
        .init(/* base value */ 0,
              /* last value */ fetchWidth,
              /* bucket size */ 1)
        .name(name() + ".rateDist")
        .desc("Number of instructions fetched each cycle (Total)")
        .flags(Stats::pdf);

    idleRate
        .name(name() + ".idleRate")
        .desc("Percent of cycles fetch was idle")
        .prereq(idleRate);
    idleRate = fetchIdleCycles * 100 / cpu->numCycles;

    branchRate
        .name(name() + ".branchRate")
        .desc("Number of branch fetches per cycle")
        .flags(Stats::total);
    branchRate = fetchedBranches / cpu->numCycles;

    fetchRate
        .name(name() + ".rate")
        .desc("Number of inst fetches per cycle")
        .flags(Stats::total);
    fetchRate = fetchedInsts / cpu->numCycles;

    branchPred.regStats();
}

template<class Impl>
void
DefaultFetch<Impl>::setTimeBuffer(TimeBuffer<TimeStruct> *time_buffer)
{
    timeBuffer = time_buffer;

    // Create wires to get information from proper places in time buffer.
    fromDecode = timeBuffer->getWire(-decodeToFetchDelay);
    fromRename = timeBuffer->getWire(-renameToFetchDelay);
    fromIEW = timeBuffer->getWire(-iewToFetchDelay);
    fromCommit = timeBuffer->getWire(-commitToFetchDelay);
}

template<class Impl>
void
DefaultFetch<Impl>::setActiveThreads(std::list<ThreadID> *at_ptr)
{
    activeThreads = at_ptr;
}

template<class Impl>
void
DefaultFetch<Impl>::setFetchQueue(TimeBuffer<FetchStruct> *fq_ptr)
{
    fetchQueue = fq_ptr;

    // Create wire to write information to proper place in fetch queue.
    toDecode = fetchQueue->getWire(0);
}

template<class Impl>
void
DefaultFetch<Impl>::initStage()
{
    // Setup PC and nextPC with initial state.
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        pc[tid] = cpu->pcState(tid);
        fetchOffset[tid] = 0;
        macroop[tid] = NULL;
    }

    for (ThreadID tid = 0; tid < numThreads; tid++) {

        fetchStatus[tid] = Running;

        priorityList.push_back(tid);

        memReq[tid] = NULL;

        stalls[tid].decode = false;
        stalls[tid].rename = false;
        stalls[tid].iew = false;
        stalls[tid].commit = false;
    }

    // Schedule fetch to get the correct PC from the CPU
    // scheduleFetchStartupEvent(1);

    // Fetch needs to start fetching instructions at the very beginning,
    // so it must start up in active state.
    switchToActive();
}

template<class Impl>
void
DefaultFetch<Impl>::setIcache()
{
    // Size of cache block.
    cacheBlkSize = icachePort->peerBlockSize();

    // Create mask to get rid of offset bits.
    cacheBlkMask = (cacheBlkSize - 1);

    for (ThreadID tid = 0; tid < numThreads; tid++) {
        // Create space to store a cache line.
        cacheData[tid] = new uint8_t[cacheBlkSize];
        cacheDataPC[tid] = 0;
        cacheDataValid[tid] = false;
    }
}

template<class Impl>
void
DefaultFetch<Impl>::processCacheCompletion(PacketPtr pkt)
{
    ThreadID tid = pkt->req->threadId();

    DPRINTF(Fetch, "[tid:%u] Waking up from cache miss.\n",tid);

    assert(!pkt->wasNacked());

    // Only change the status if it's still waiting on the icache access
    // to return.
    if (fetchStatus[tid] != IcacheWaitResponse ||
        pkt->req != memReq[tid] ||
        isSwitchedOut()) {
        ++fetchIcacheSquashes;
        delete pkt->req;
        delete pkt;
        return;
    }

    memcpy(cacheData[tid], pkt->getPtr<uint8_t>(), cacheBlkSize);
    cacheDataValid[tid] = true;

    if (!drainPending) {
        // Wake up the CPU (if it went to sleep and was waiting on
        // this completion event).
        cpu->wakeCPU();

        DPRINTF(Activity, "[tid:%u] Activating fetch due to cache completion\n",
                tid);

        switchToActive();
    }

    // Only switch to IcacheAccessComplete if we're not stalled as well.
    if (checkStall(tid)) {
        fetchStatus[tid] = Blocked;
    } else {
        fetchStatus[tid] = IcacheAccessComplete;
    }

    // Reset the mem req to NULL.
    delete pkt->req;
    delete pkt;
    memReq[tid] = NULL;
}

template <class Impl>
bool
DefaultFetch<Impl>::drain()
{
    // Fetch is ready to drain at any time.
    cpu->signalDrained();
    drainPending = true;
    return true;
}

template <class Impl>
void
DefaultFetch<Impl>::resume()
{
    drainPending = false;
}

template <class Impl>
void
DefaultFetch<Impl>::switchOut()
{
    switchedOut = true;
    // Branch predictor needs to have its state cleared.
    branchPred.switchOut();
}

template <class Impl>
void
DefaultFetch<Impl>::takeOverFrom()
{
    // Reset all state
    for (ThreadID i = 0; i < Impl::MaxThreads; ++i) {
        stalls[i].decode = 0;
        stalls[i].rename = 0;
        stalls[i].iew = 0;
        stalls[i].commit = 0;
        pc[i] = cpu->pcState(i);
        fetchStatus[i] = Running;
    }
    numInst = 0;
    wroteToTimeBuffer = false;
    _status = Inactive;
    switchedOut = false;
    interruptPending = false;
    branchPred.takeOverFrom();
}

template <class Impl>
void
DefaultFetch<Impl>::wakeFromQuiesce()
{
    DPRINTF(Fetch, "Waking up from quiesce\n");
    // Hopefully this is safe
    // @todo: Allow other threads to wake from quiesce.
    fetchStatus[0] = Running;
}

template <class Impl>
inline void
DefaultFetch<Impl>::switchToActive()
{
    if (_status == Inactive) {
        DPRINTF(Activity, "Activating stage.\n");

        cpu->activateStage(O3CPU::FetchIdx);

        _status = Active;
    }
}

template <class Impl>
inline void
DefaultFetch<Impl>::switchToInactive()
{
    if (_status == Active) {
        DPRINTF(Activity, "Deactivating stage.\n");

        cpu->deactivateStage(O3CPU::FetchIdx);

        _status = Inactive;
    }
}

template <class Impl>
bool
DefaultFetch<Impl>::lookupAndUpdateNextPC(
        DynInstPtr &inst, TheISA::PCState &nextPC)
{
    // Do branch prediction check here.
    // A bit of a misnomer...next_PC is actually the current PC until
    // this function updates it.
    bool predict_taken;

    if (!inst->isControl()) {
        TheISA::advancePC(nextPC, inst->staticInst);
        inst->setPredTarg(nextPC);
        inst->setPredTaken(false);
        return false;
    }

    ThreadID tid = inst->threadNumber;
    predict_taken = branchPred.predict(inst, nextPC, tid);

    if (predict_taken) {
        DPRINTF(Fetch, "[tid:%i]: [sn:%i]:  Branch predicted to be taken to %s.\n",
                tid, inst->seqNum, nextPC);
    } else {
        DPRINTF(Fetch, "[tid:%i]: [sn:%i]:Branch predicted to be not taken.\n",
                tid, inst->seqNum);
    }

    DPRINTF(Fetch, "[tid:%i]: [sn:%i] Branch predicted to go to %s.\n",
            tid, inst->seqNum, nextPC);
    inst->setPredTarg(nextPC);
    inst->setPredTaken(predict_taken);

    ++fetchedBranches;

    if (predict_taken) {
        ++predictedBranches;
    }

    return predict_taken;
}

template <class Impl>
bool
DefaultFetch<Impl>::fetchCacheLine(Addr vaddr, Fault &ret_fault, ThreadID tid,
                                   Addr pc)
{
    Fault fault = NoFault;

    //AlphaDep
    if (cacheBlocked) {
        DPRINTF(Fetch, "[tid:%i] Can't fetch cache line, cache blocked\n",
                tid);
        return false;
    } else if (isSwitchedOut()) {
        DPRINTF(Fetch, "[tid:%i] Can't fetch cache line, switched out\n",
                tid);
        return false;
    } else if (checkInterrupt(pc)) {
        // Hold off fetch from getting new instructions when:
        // Cache is blocked, or
        // while an interrupt is pending and we're not in PAL mode, or
        // fetch is switched out.
        DPRINTF(Fetch, "[tid:%i] Can't fetch cache line, interrupt pending\n",
                tid);
        return false;
    }

    // Align the fetch address so it's at the start of a cache block.
    Addr block_PC = icacheBlockAlignPC(vaddr);

    // If we've already got the block, no need to try to fetch it again.
    if (cacheDataValid[tid] && block_PC == cacheDataPC[tid]) {
        return true;
    }

    // Setup the memReq to do a read of the first instruction's address.
    // Set the appropriate read size and flags as well.
    // Build request here.
    RequestPtr mem_req =
        new Request(tid, block_PC, cacheBlkSize, Request::INST_FETCH,
                    pc, cpu->thread[tid]->contextId(), tid);

    memReq[tid] = mem_req;

    // Translate the instruction request.
    fault = cpu->itb->translateAtomic(mem_req, cpu->thread[tid]->getTC(),
                                      BaseTLB::Execute);

    // In the case of faults, the fetch stage may need to stall and wait
    // for the ITB miss to be handled.

    // If translation was successful, attempt to read the first
    // instruction.
    if (fault == NoFault) {
#if 0
        if (cpu->system->memctrl->badaddr(memReq[tid]->paddr) ||
            memReq[tid]->isUncacheable()) {
            DPRINTF(Fetch, "Fetch: Bad address %#x (hopefully on a "
                    "misspeculating path)!",
                    memReq[tid]->paddr);
            ret_fault = TheISA::genMachineCheckFault();
            return false;
        }
#endif

        // Build packet here.
        PacketPtr data_pkt = new Packet(mem_req,
                                        MemCmd::ReadReq, Packet::Broadcast);
        data_pkt->dataDynamicArray(new uint8_t[cacheBlkSize]);

        cacheDataPC[tid] = block_PC;
        cacheDataValid[tid] = false;

        DPRINTF(Fetch, "Fetch: Doing instruction read.\n");

        fetchedCacheLines++;

        // Now do the timing access to see whether or not the instruction
        // exists within the cache.
        if (!icachePort->sendTiming(data_pkt)) {
            assert(retryPkt == NULL);
            assert(retryTid == InvalidThreadID);
            DPRINTF(Fetch, "[tid:%i] Out of MSHRs!\n", tid);
            fetchStatus[tid] = IcacheWaitRetry;
            retryPkt = data_pkt;
            retryTid = tid;
            cacheBlocked = true;
            return false;
        }

        DPRINTF(Fetch, "[tid:%i]: Doing cache access.\n", tid);

        lastIcacheStall[tid] = curTick();

        DPRINTF(Activity, "[tid:%i]: Activity: Waiting on I-cache "
                "response.\n", tid);

        fetchStatus[tid] = IcacheWaitResponse;
    } else {
        delete mem_req;
        memReq[tid] = NULL;
    }

    ret_fault = fault;
    return true;
}

template <class Impl>
inline void
DefaultFetch<Impl>::doSquash(const TheISA::PCState &newPC, ThreadID tid)
{
    DPRINTF(Fetch, "[tid:%i]: Squashing, setting PC to: %s.\n",
            tid, newPC);

    pc[tid] = newPC;
    fetchOffset[tid] = 0;
    macroop[tid] = NULL;
    predecoder.reset();

    // Clear the icache miss if it's outstanding.
    if (fetchStatus[tid] == IcacheWaitResponse) {
        DPRINTF(Fetch, "[tid:%i]: Squashing outstanding Icache miss.\n",
                tid);
        memReq[tid] = NULL;
    }

    // Get rid of the retrying packet if it was from this thread.
    if (retryTid == tid) {
        assert(cacheBlocked);
        if (retryPkt) {
            delete retryPkt->req;
            delete retryPkt;
        }
        retryPkt = NULL;
        retryTid = InvalidThreadID;
    }

    fetchStatus[tid] = Squashing;

    ++fetchSquashCycles;
}

template<class Impl>
void
DefaultFetch<Impl>::squashFromDecode(const TheISA::PCState &newPC,
                                     const InstSeqNum &seq_num, ThreadID tid)
{
    DPRINTF(Fetch, "[tid:%i]: Squashing from decode.\n", tid);

    doSquash(newPC, tid);

    // Tell the CPU to remove any instructions that are in flight between
    // fetch and decode.
    cpu->removeInstsUntil(seq_num, tid);
}

template<class Impl>
bool
DefaultFetch<Impl>::checkStall(ThreadID tid) const
{
    bool ret_val = false;

    if (cpu->contextSwitch) {
        DPRINTF(Fetch,"[tid:%i]: Stalling for a context switch.\n",tid);
        ret_val = true;
    } else if (stalls[tid].decode) {
        DPRINTF(Fetch,"[tid:%i]: Stall from Decode stage detected.\n",tid);
        ret_val = true;
    } else if (stalls[tid].rename) {
        DPRINTF(Fetch,"[tid:%i]: Stall from Rename stage detected.\n",tid);
        ret_val = true;
    } else if (stalls[tid].iew) {
        DPRINTF(Fetch,"[tid:%i]: Stall from IEW stage detected.\n",tid);
        ret_val = true;
    } else if (stalls[tid].commit) {
        DPRINTF(Fetch,"[tid:%i]: Stall from Commit stage detected.\n",tid);
        ret_val = true;
    }

    return ret_val;
}

template<class Impl>
typename DefaultFetch<Impl>::FetchStatus
DefaultFetch<Impl>::updateFetchStatus()
{
    //Check Running
    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (fetchStatus[tid] == Running ||
            fetchStatus[tid] == Squashing ||
            fetchStatus[tid] == IcacheAccessComplete) {

            if (_status == Inactive) {
                DPRINTF(Activity, "[tid:%i]: Activating stage.\n",tid);

                if (fetchStatus[tid] == IcacheAccessComplete) {
                    DPRINTF(Activity, "[tid:%i]: Activating fetch due to cache"
                            "completion\n",tid);
                }

                cpu->activateStage(O3CPU::FetchIdx);
            }

            return Active;
        }
    }

    // Stage is switching from active to inactive, notify CPU of it.
    if (_status == Active) {
        DPRINTF(Activity, "Deactivating stage.\n");

        cpu->deactivateStage(O3CPU::FetchIdx);
    }

    return Inactive;
}

template <class Impl>
void
DefaultFetch<Impl>::squash(const TheISA::PCState &newPC,
                           const InstSeqNum &seq_num, ThreadID tid)
{
    DPRINTF(Fetch, "[tid:%u]: Squash from commit.\n", tid);

    doSquash(newPC, tid);

    // Tell the CPU to remove any instructions that are not in the ROB.
    cpu->removeInstsNotInROB(tid);
}

template <class Impl>
void
DefaultFetch<Impl>::tick()
{
    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();
    bool status_change = false;

    wroteToTimeBuffer = false;

    while (threads != end) {
        ThreadID tid = *threads++;

        // Check the signals for each thread to determine the proper status
        // for each thread.
        bool updated_status = checkSignalsAndUpdate(tid);
        status_change =  status_change || updated_status;
    }

    DPRINTF(Fetch, "Running stage.\n");

    // Reset the number of the instruction we're fetching.
    numInst = 0;

#if FULL_SYSTEM
    if (fromCommit->commitInfo[0].interruptPending) {
        interruptPending = true;
    }

    if (fromCommit->commitInfo[0].clearInterrupt) {
        interruptPending = false;
    }
#endif

    for (threadFetched = 0; threadFetched < numFetchingThreads;
         threadFetched++) {
        // Fetch each of the actively fetching threads.
        fetch(status_change);
    }

    // Record number of instructions fetched this cycle for distribution.
    fetchNisnDist.sample(numInst);

    if (status_change) {
        // Change the fetch stage status if there was a status change.
        _status = updateFetchStatus();
    }

    // If there was activity this cycle, inform the CPU of it.
    if (wroteToTimeBuffer || cpu->contextSwitch) {
        DPRINTF(Activity, "Activity this cycle.\n");

        cpu->activityThisCycle();
    }
}

template <class Impl>
bool
DefaultFetch<Impl>::checkSignalsAndUpdate(ThreadID tid)
{
    // Update the per thread stall statuses.
    if (fromDecode->decodeBlock[tid]) {
        stalls[tid].decode = true;
    }

    if (fromDecode->decodeUnblock[tid]) {
        assert(stalls[tid].decode);
        assert(!fromDecode->decodeBlock[tid]);
        stalls[tid].decode = false;
    }

    if (fromRename->renameBlock[tid]) {
        stalls[tid].rename = true;
    }

    if (fromRename->renameUnblock[tid]) {
        assert(stalls[tid].rename);
        assert(!fromRename->renameBlock[tid]);
        stalls[tid].rename = false;
    }

    if (fromIEW->iewBlock[tid]) {
        stalls[tid].iew = true;
    }

    if (fromIEW->iewUnblock[tid]) {
        assert(stalls[tid].iew);
        assert(!fromIEW->iewBlock[tid]);
        stalls[tid].iew = false;
    }

    if (fromCommit->commitBlock[tid]) {
        stalls[tid].commit = true;
    }

    if (fromCommit->commitUnblock[tid]) {
        assert(stalls[tid].commit);
        assert(!fromCommit->commitBlock[tid]);
        stalls[tid].commit = false;
    }

    // Check squash signals from commit.
    if (fromCommit->commitInfo[tid].squash) {

        DPRINTF(Fetch, "[tid:%u]: Squashing instructions due to squash "
                "from commit.\n",tid);
        // In any case, squash.
        squash(fromCommit->commitInfo[tid].pc,
               fromCommit->commitInfo[tid].doneSeqNum,
               tid);

        // Also check if there's a mispredict that happened.
        if (fromCommit->commitInfo[tid].branchMispredict) {
            branchPred.squash(fromCommit->commitInfo[tid].doneSeqNum,
                              fromCommit->commitInfo[tid].pc,
                              fromCommit->commitInfo[tid].branchTaken,
                              tid);
        } else {
            branchPred.squash(fromCommit->commitInfo[tid].doneSeqNum,
                              tid);
        }

        return true;
    } else if (fromCommit->commitInfo[tid].doneSeqNum) {
        // Update the branch predictor if it wasn't a squashed instruction
        // that was broadcasted.
        branchPred.update(fromCommit->commitInfo[tid].doneSeqNum, tid);
    }

    // Check ROB squash signals from commit.
    if (fromCommit->commitInfo[tid].robSquashing) {
        DPRINTF(Fetch, "[tid:%u]: ROB is still squashing.\n", tid);

        // Continue to squash.
        fetchStatus[tid] = Squashing;

        return true;
    }

    // Check squash signals from decode.
    if (fromDecode->decodeInfo[tid].squash) {
        DPRINTF(Fetch, "[tid:%u]: Squashing instructions due to squash "
                "from decode.\n",tid);

        // Update the branch predictor.
        if (fromDecode->decodeInfo[tid].branchMispredict) {
            branchPred.squash(fromDecode->decodeInfo[tid].doneSeqNum,
                              fromDecode->decodeInfo[tid].nextPC,
                              fromDecode->decodeInfo[tid].branchTaken,
                              tid);
        } else {
            branchPred.squash(fromDecode->decodeInfo[tid].doneSeqNum,
                              tid);
        }

        if (fetchStatus[tid] != Squashing) {

            TheISA::PCState nextPC = fromDecode->decodeInfo[tid].nextPC;
            DPRINTF(Fetch, "Squashing from decode with PC = %s\n", nextPC);
            // Squash unless we're already squashing
            squashFromDecode(fromDecode->decodeInfo[tid].nextPC,
                             fromDecode->decodeInfo[tid].doneSeqNum,
                             tid);

            return true;
        }
    }

    if (checkStall(tid) &&
        fetchStatus[tid] != IcacheWaitResponse &&
        fetchStatus[tid] != IcacheWaitRetry) {
        DPRINTF(Fetch, "[tid:%i]: Setting to blocked\n",tid);

        fetchStatus[tid] = Blocked;

        return true;
    }

    if (fetchStatus[tid] == Blocked ||
        fetchStatus[tid] == Squashing) {
        // Switch status to running if fetch isn't being told to block or
        // squash this cycle.
        DPRINTF(Fetch, "[tid:%i]: Done squashing, switching to running.\n",
                tid);

        fetchStatus[tid] = Running;

        return true;
    }

    // If we've reached this point, we have not gotten any signals that
    // cause fetch to change its status.  Fetch remains the same as before.
    return false;
}

template<class Impl>
typename Impl::DynInstPtr
DefaultFetch<Impl>::buildInst(ThreadID tid, StaticInstPtr staticInst,
                              StaticInstPtr curMacroop, TheISA::PCState thisPC,
                              TheISA::PCState nextPC, bool trace)
{
    // Get a sequence number.
    InstSeqNum seq = cpu->getAndIncrementInstSeq();

    // Create a new DynInst from the instruction fetched.
    DynInstPtr instruction =
        new DynInst(staticInst, thisPC, nextPC, seq, cpu);
    instruction->setTid(tid);

    instruction->setASID(tid);

    instruction->setThreadState(cpu->thread[tid]);

    DPRINTF(Fetch, "[tid:%i]: Instruction PC %#x (%d) created "
            "[sn:%lli]\n", tid, thisPC.instAddr(),
            thisPC.microPC(), seq);

    DPRINTF(Fetch, "[tid:%i]: Instruction is: %s\n", tid,
            instruction->staticInst->
            disassemble(thisPC.instAddr()));

#if TRACING_ON
    if (trace) {
        instruction->traceData =
            cpu->getTracer()->getInstRecord(curTick(), cpu->tcBase(tid),
                    instruction->staticInst, thisPC, curMacroop);
    }
#else
    instruction->traceData = NULL;
#endif

    // Add instruction to the CPU's list of instructions.
    instruction->setInstListIt(cpu->addInst(instruction));

    // Write the instruction to the first slot in the queue
    // that heads to decode.
    assert(numInst < fetchWidth);
    toDecode->insts[toDecode->size++] = instruction;

    return instruction;
}

template<class Impl>
void
DefaultFetch<Impl>::fetch(bool &status_change)
{
    //////////////////////////////////////////
    // Start actual fetch
    //////////////////////////////////////////
    ThreadID tid = getFetchingThread(fetchPolicy);

    if (tid == InvalidThreadID || drainPending) {
        DPRINTF(Fetch,"There are no more threads available to fetch from.\n");

        // Breaks looping condition in tick()
        threadFetched = numFetchingThreads;
        return;
    }

    DPRINTF(Fetch, "Attempting to fetch from [tid:%i]\n", tid);

    // The current PC.
    TheISA::PCState thisPC = pc[tid];

    // Fault code for memory access.
    Fault fault = NoFault;

    Addr pcOffset = fetchOffset[tid];
    Addr fetchAddr = (thisPC.instAddr() + pcOffset) & BaseCPU::PCMask;

    // If returning from the delay of a cache miss, then update the status
    // to running, otherwise do the cache access.  Possibly move this up
    // to tick() function.
    if (fetchStatus[tid] == IcacheAccessComplete) {
        DPRINTF(Fetch, "[tid:%i]: Icache miss is complete.\n",tid);

        fetchStatus[tid] = Running;
        status_change = true;
    } else if (fetchStatus[tid] == Running) {
        DPRINTF(Fetch, "[tid:%i]: Attempting to translate and read "
                "instruction, starting at PC %#x.\n", tid, fetchAddr);

        bool fetch_success = fetchCacheLine(fetchAddr, fault, tid,
                                            thisPC.instAddr());
        if (!fetch_success) {
            if (cacheBlocked) {
                ++icacheStallCycles;
            } else {
                ++fetchMiscStallCycles;
            }
            return;
        }
    } else {
        if (fetchStatus[tid] == Idle) {
            ++fetchIdleCycles;
            DPRINTF(Fetch, "[tid:%i]: Fetch is idle!\n", tid);
        } else if (fetchStatus[tid] == Blocked) {
            ++fetchBlockedCycles;
            DPRINTF(Fetch, "[tid:%i]: Fetch is blocked!\n", tid);
        } else if (fetchStatus[tid] == Squashing) {
            ++fetchSquashCycles;
            DPRINTF(Fetch, "[tid:%i]: Fetch is squashing!\n", tid);
        } else if (fetchStatus[tid] == IcacheWaitResponse) {
            ++icacheStallCycles;
            DPRINTF(Fetch, "[tid:%i]: Fetch is waiting cache response!\n", tid);
        }

        // Status is Idle, Squashing, Blocked, or IcacheWaitResponse, so
        // fetch should do nothing.
        return;
    }

    ++fetchCycles;

    // If we had a stall due to an icache miss, then return.
    if (fetchStatus[tid] == IcacheWaitResponse) {
        ++icacheStallCycles;
        status_change = true;
        return;
    }

    TheISA::PCState nextPC = thisPC;

    StaticInstPtr staticInst = NULL;
    StaticInstPtr curMacroop = macroop[tid];

    if (fault == NoFault) {

        // If the read of the first instruction was successful, then grab the
        // instructions from the rest of the cache line and put them into the
        // queue heading to decode.

        DPRINTF(Fetch,
                "[tid:%i]: Adding instructions to queue to decode.\n", tid);

        // Need to keep track of whether or not a predicted branch
        // ended this fetch block.
        bool predictedBranch = false;

        TheISA::MachInst *cacheInsts =
            reinterpret_cast<TheISA::MachInst *>(cacheData[tid]);

        const unsigned numInsts = cacheBlkSize / instSize;
        unsigned blkOffset = (fetchAddr - cacheDataPC[tid]) / instSize;

        // Loop through instruction memory from the cache.
        while (blkOffset < numInsts &&
               numInst < fetchWidth &&
               !predictedBranch) {

            // If we need to process more memory, do it now.
            if (!curMacroop && !predecoder.extMachInstReady()) {
                if (ISA_HAS_DELAY_SLOT && pcOffset == 0) {
                    // Walk past any annulled delay slot instructions.
                    Addr pcAddr = thisPC.instAddr() & BaseCPU::PCMask;
                    while (fetchAddr != pcAddr && blkOffset < numInsts) {
                        blkOffset++;
                        fetchAddr += instSize;
                    }
                    if (blkOffset >= numInsts)
                        break;
                }
                MachInst inst = TheISA::gtoh(cacheInsts[blkOffset]);

                predecoder.setTC(cpu->thread[tid]->getTC());
                predecoder.moreBytes(thisPC, fetchAddr, inst);

                if (predecoder.needMoreBytes()) {
                    blkOffset++;
                    fetchAddr += instSize;
                    pcOffset += instSize;
                }
            }

            // Extract as many instructions and/or microops as we can from
            // the memory we've processed so far.
            do {
                if (!curMacroop) {
                    if (predecoder.extMachInstReady()) {
                        ExtMachInst extMachInst;

                        extMachInst = predecoder.getExtMachInst(thisPC);
                        pcOffset = 0;
                        staticInst = StaticInstPtr(extMachInst,
                                                   thisPC.instAddr());

                        // Increment stat of fetched instructions.
                        ++fetchedInsts;

                        if (staticInst->isMacroop())
                            curMacroop = staticInst;
                    } else {
                        // We need more bytes for this instruction.
                        break;
                    }
                }
                if (curMacroop) {
                    staticInst = curMacroop->fetchMicroop(thisPC.microPC());
                    if (staticInst->isLastMicroop())
                        curMacroop = NULL;
                }

                DynInstPtr instruction =
                    buildInst(tid, staticInst, curMacroop,
                              thisPC, nextPC, true);

                numInst++;

                nextPC = thisPC;

                // If we're branching after this instruction, quite fetching
                // from the same block then.
                predictedBranch |= thisPC.branching();
                predictedBranch |=
                    lookupAndUpdateNextPC(instruction, nextPC);
                if (predictedBranch) {
                    DPRINTF(Fetch, "Branch detected with PC = %s\n", thisPC);
                }

                // Move to the next instruction, unless we have a branch.
                thisPC = nextPC;

                if (instruction->isQuiesce()) {
                    DPRINTF(Fetch,
                            "Quiesce instruction encountered, halting fetch!");
                    fetchStatus[tid] = QuiescePending;
                    status_change = true;
                    break;
                }
            } while ((curMacroop || predecoder.extMachInstReady()) &&
                     numInst < fetchWidth);
        }

        if (predictedBranch) {
            DPRINTF(Fetch, "[tid:%i]: Done fetching, predicted branch "
                    "instruction encountered.\n", tid);
        } else if (numInst >= fetchWidth) {
            DPRINTF(Fetch, "[tid:%i]: Done fetching, reached fetch bandwidth "
                    "for this cycle.\n", tid);
        } else if (blkOffset >= cacheBlkSize) {
            DPRINTF(Fetch, "[tid:%i]: Done fetching, reached the end of cache "
                    "block.\n", tid);
        }
    }

    macroop[tid] = curMacroop;
    fetchOffset[tid] = pcOffset;

    if (numInst > 0) {
        wroteToTimeBuffer = true;
    }

    // Now that fetching is completed, update the PC to signify what the next
    // cycle will be.
    if (fault == NoFault) {
        pc[tid] = nextPC;
        DPRINTF(Fetch, "[tid:%i]: Setting PC to %s.\n", tid, nextPC);
    } else {
        // We shouldn't be in an icache miss and also have a fault (an ITB
        // miss)
        if (fetchStatus[tid] == IcacheWaitResponse) {
            panic("Fetch should have exited prior to this!");
        }

        // Send the fault to commit.  This thread will not do anything
        // until commit handles the fault.  The only other way it can
        // wake up is if a squash comes along and changes the PC.  Send the
        // fault on a dummy nop.
        staticInst = StaticInstPtr(TheISA::NoopMachInst, thisPC.instAddr());

        DynInstPtr instruction =
            buildInst(tid, staticInst, NULL, thisPC, nextPC, false);

        TheISA::advancePC(nextPC, staticInst);
        instruction->setPredTarg(nextPC);
        instruction->fault = fault;

        DPRINTF(Fetch, "[tid:%i]: Blocked, need to handle the trap.\n",tid);

        fetchStatus[tid] = TrapPending;
        status_change = true;

        DPRINTF(Fetch, "[tid:%i]: fault (%s) detected @ PC %s, sending nop "
                       "[sn:%lli]\n", tid, fault->name(), thisPC, inst_seq);
    }
}

template<class Impl>
void
DefaultFetch<Impl>::recvRetry()
{
    if (retryPkt != NULL) {
        assert(cacheBlocked);
        assert(retryTid != InvalidThreadID);
        assert(fetchStatus[retryTid] == IcacheWaitRetry);

        if (icachePort->sendTiming(retryPkt)) {
            fetchStatus[retryTid] = IcacheWaitResponse;
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
template<class Impl>
ThreadID
DefaultFetch<Impl>::getFetchingThread(FetchPriority &fetch_priority)
{
    if (numThreads > 1) {
        switch (fetch_priority) {

          case SingleThread:
            return 0;

          case RoundRobin:
            return roundRobin();

          case IQ:
            return iqCount();

          case LSQ:
            return lsqCount();

          case Branch:
            return branchCount();

          default:
            return InvalidThreadID;
        }
    } else {
        list<ThreadID>::iterator thread = activeThreads->begin();
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


template<class Impl>
ThreadID
DefaultFetch<Impl>::roundRobin()
{
    list<ThreadID>::iterator pri_iter = priorityList.begin();
    list<ThreadID>::iterator end      = priorityList.end();

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

template<class Impl>
ThreadID
DefaultFetch<Impl>::iqCount()
{
    std::priority_queue<ThreadID> PQ;

    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        PQ.push(fromIEW->iewInfo[tid].iqCount);
    }

    while (!PQ.empty()) {
        ThreadID high_pri = PQ.top();

        if (fetchStatus[high_pri] == Running ||
            fetchStatus[high_pri] == IcacheAccessComplete ||
            fetchStatus[high_pri] == Idle)
            return high_pri;
        else
            PQ.pop();

    }

    return InvalidThreadID;
}

template<class Impl>
ThreadID
DefaultFetch<Impl>::lsqCount()
{
    std::priority_queue<ThreadID> PQ;

    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        PQ.push(fromIEW->iewInfo[tid].ldstqCount);
    }

    while (!PQ.empty()) {
        ThreadID high_pri = PQ.top();

        if (fetchStatus[high_pri] == Running ||
            fetchStatus[high_pri] == IcacheAccessComplete ||
            fetchStatus[high_pri] == Idle)
            return high_pri;
        else
            PQ.pop();
    }

    return InvalidThreadID;
}

template<class Impl>
ThreadID
DefaultFetch<Impl>::branchCount()
{
#if 0
    list<ThreadID>::iterator thread = activeThreads->begin();
    assert(thread != activeThreads->end());
    ThreadID tid = *thread;
#endif

    panic("Branch Count Fetch policy unimplemented\n");
    return InvalidThreadID;
}
