/*
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

#include "config/use_checker.hh"

#include "arch/isa_traits.hh"
#include "arch/utility.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/exetrace.hh"
#include "cpu/o3/fetch.hh"
#include "mem/packet.hh"
#include "mem/request.hh"
#include "sim/byteswap.hh"
#include "sim/host.hh"
#include "sim/root.hh"

#if FULL_SYSTEM
#include "arch/tlb.hh"
#include "arch/vtophys.hh"
#include "base/remote_gdb.hh"
#include "sim/system.hh"
#endif // FULL_SYSTEM

#include <algorithm>

using namespace std;
using namespace TheISA;

template<class Impl>
Tick
DefaultFetch<Impl>::IcachePort::recvAtomic(PacketPtr pkt)
{
    panic("DefaultFetch doesn't expect recvAtomic callback!");
    return curTick;
}

template<class Impl>
void
DefaultFetch<Impl>::IcachePort::recvFunctional(PacketPtr pkt)
{
    panic("DefaultFetch doesn't expect recvFunctional callback!");
}

template<class Impl>
void
DefaultFetch<Impl>::IcachePort::recvStatusChange(Status status)
{
    if (status == RangeChange)
        return;

    panic("DefaultFetch doesn't expect recvStatusChange callback!");
}

template<class Impl>
bool
DefaultFetch<Impl>::IcachePort::recvTiming(Packet *pkt)
{
    fetch->processCacheCompletion(pkt);
    return true;
}

template<class Impl>
void
DefaultFetch<Impl>::IcachePort::recvRetry()
{
    fetch->recvRetry();
}

template<class Impl>
DefaultFetch<Impl>::DefaultFetch(Params *params)
    : mem(params->mem),
      branchPred(params),
      decodeToFetchDelay(params->decodeToFetchDelay),
      renameToFetchDelay(params->renameToFetchDelay),
      iewToFetchDelay(params->iewToFetchDelay),
      commitToFetchDelay(params->commitToFetchDelay),
      fetchWidth(params->fetchWidth),
      cacheBlocked(false),
      retryPkt(NULL),
      retryTid(-1),
      numThreads(params->numberOfThreads),
      numFetchingThreads(params->smtNumFetchingThreads),
      interruptPending(false),
      switchedOut(false)
{
    if (numThreads > Impl::MaxThreads)
        fatal("numThreads is not a valid value\n");

    DPRINTF(Fetch, "Fetch constructor called\n");

    // Set fetch stage's status to inactive.
    _status = Inactive;

    string policy = params->smtFetchPolicy;

    // Convert string to lowercase
    std::transform(policy.begin(), policy.end(), policy.begin(),
                   (int(*)(int)) tolower);

    // Figure out fetch policy
    if (policy == "singlethread") {
        fetchPolicy = SingleThread;
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

    // Size of cache block.
    cacheBlkSize = 64;

    // Create mask to get rid of offset bits.
    cacheBlkMask = (cacheBlkSize - 1);

    for (int tid=0; tid < numThreads; tid++) {

        fetchStatus[tid] = Running;

        priorityList.push_back(tid);

        memReq[tid] = NULL;

        // Create space to store a cache line.
        cacheData[tid] = new uint8_t[cacheBlkSize];

        stalls[tid].decode = 0;
        stalls[tid].rename = 0;
        stalls[tid].iew = 0;
        stalls[tid].commit = 0;
    }

    // Get the size of an instruction.
    instSize = sizeof(MachInst);
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
DefaultFetch<Impl>::setCPU(O3CPU *cpu_ptr)
{
    DPRINTF(Fetch, "Setting the CPU pointer.\n");
    cpu = cpu_ptr;

    // Name is finally available, so create the port.
    icachePort = new IcachePort(this);

    Port *mem_dport = mem->getPort("");
    icachePort->setPeer(mem_dport);
    mem_dport->setPeer(icachePort);

#if USE_CHECKER
    if (cpu->checker) {
        cpu->checker->setIcachePort(icachePort);
    }
#endif

    // Fetch needs to start fetching instructions at the very beginning,
    // so it must start up in active state.
    switchToActive();
}

template<class Impl>
void
DefaultFetch<Impl>::setTimeBuffer(TimeBuffer<TimeStruct> *time_buffer)
{
    DPRINTF(Fetch, "Setting the time buffer pointer.\n");
    timeBuffer = time_buffer;

    // Create wires to get information from proper places in time buffer.
    fromDecode = timeBuffer->getWire(-decodeToFetchDelay);
    fromRename = timeBuffer->getWire(-renameToFetchDelay);
    fromIEW = timeBuffer->getWire(-iewToFetchDelay);
    fromCommit = timeBuffer->getWire(-commitToFetchDelay);
}

template<class Impl>
void
DefaultFetch<Impl>::setActiveThreads(list<unsigned> *at_ptr)
{
    DPRINTF(Fetch, "Setting active threads list pointer.\n");
    activeThreads = at_ptr;
}

template<class Impl>
void
DefaultFetch<Impl>::setFetchQueue(TimeBuffer<FetchStruct> *fq_ptr)
{
    DPRINTF(Fetch, "Setting the fetch queue pointer.\n");
    fetchQueue = fq_ptr;

    // Create wire to write information to proper place in fetch queue.
    toDecode = fetchQueue->getWire(0);
}

template<class Impl>
void
DefaultFetch<Impl>::initStage()
{
    // Setup PC and nextPC with initial state.
    for (int tid = 0; tid < numThreads; tid++) {
        PC[tid] = cpu->readPC(tid);
        nextPC[tid] = cpu->readNextPC(tid);
#if THE_ISA != ALPHA_ISA
        nextNPC[tid] = cpu->readNextNPC(tid);
#endif
    }
}

template<class Impl>
void
DefaultFetch<Impl>::processCacheCompletion(PacketPtr pkt)
{
    unsigned tid = pkt->req->getThreadNum();

    DPRINTF(Fetch, "[tid:%u] Waking up from cache miss.\n",tid);

    // Only change the status if it's still waiting on the icache access
    // to return.
    if (fetchStatus[tid] != IcacheWaitResponse ||
        pkt->req != memReq[tid] ||
        isSwitchedOut()) {
        ++fetchIcacheSquashes;
        delete pkt->req;
        delete pkt;
        memReq[tid] = NULL;
        return;
    }

    // Wake up the CPU (if it went to sleep and was waiting on this completion
    // event).
    cpu->wakeCPU();

    DPRINTF(Activity, "[tid:%u] Activating fetch due to cache completion\n",
            tid);

    switchToActive();

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
void
DefaultFetch<Impl>::switchOut()
{
    // Fetch is ready to switch out at any time.
    switchedOut = true;
    cpu->signalSwitched();
}

template <class Impl>
void
DefaultFetch<Impl>::doSwitchOut()
{
    // Branch predictor needs to have its state cleared.
    branchPred.switchOut();
}

template <class Impl>
void
DefaultFetch<Impl>::takeOverFrom()
{
    // Reset all state
    for (int i = 0; i < Impl::MaxThreads; ++i) {
        stalls[i].decode = 0;
        stalls[i].rename = 0;
        stalls[i].iew = 0;
        stalls[i].commit = 0;
        PC[i] = cpu->readPC(i);
        nextPC[i] = cpu->readNextPC(i);
#if THE_ISA != ALPHA_ISA
        nextNPC[i] = cpu->readNextNPC(i);
#endif
        fetchStatus[i] = Running;
    }
    numInst = 0;
    wroteToTimeBuffer = false;
    _status = Inactive;
    switchedOut = false;
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
DefaultFetch<Impl>::lookupAndUpdateNextPC(DynInstPtr &inst, Addr &next_PC)
{
    // Do branch prediction check here.
    // A bit of a misnomer...next_PC is actually the current PC until
    // this function updates it.
    bool predict_taken;

    if (!inst->isControl()) {
        next_PC = next_PC + instSize;
        inst->setPredTarg(next_PC);
        return false;
    }

    predict_taken = branchPred.predict(inst, next_PC, inst->threadNumber);

    ++fetchedBranches;

    if (predict_taken) {
        ++predictedBranches;
    }

    return predict_taken;
}

template <class Impl>
bool
DefaultFetch<Impl>::fetchCacheLine(Addr fetch_PC, Fault &ret_fault, unsigned tid)
{
    Fault fault = NoFault;

#if FULL_SYSTEM
    // Flag to say whether or not address is physical addr.
    unsigned flags = cpu->inPalMode(fetch_PC) ? PHYSICAL : 0;
#else
    unsigned flags = 0;
#endif // FULL_SYSTEM

    if (cacheBlocked || (interruptPending && flags == 0) || switchedOut) {
        // Hold off fetch from getting new instructions when:
        // Cache is blocked, or
        // while an interrupt is pending and we're not in PAL mode, or
        // fetch is switched out.
        return false;
    }

    // Align the fetch PC so it's at the start of a cache block.
    fetch_PC = icacheBlockAlignPC(fetch_PC);

    // Setup the memReq to do a read of the first instruction's address.
    // Set the appropriate read size and flags as well.
    // Build request here.
    RequestPtr mem_req = new Request(tid, fetch_PC, cacheBlkSize, flags,
                                     fetch_PC, cpu->readCpuId(), tid);

    memReq[tid] = mem_req;

    // Translate the instruction request.
    fault = cpu->translateInstReq(mem_req, cpu->thread[tid]);

    // In the case of faults, the fetch stage may need to stall and wait
    // for the ITB miss to be handled.

    // If translation was successful, attempt to read the first
    // instruction.
    if (fault == NoFault) {
#if 0
        if (cpu->system->memctrl->badaddr(memReq[tid]->paddr) ||
            memReq[tid]->flags & UNCACHEABLE) {
            DPRINTF(Fetch, "Fetch: Bad address %#x (hopefully on a "
                    "misspeculating path)!",
                    memReq[tid]->paddr);
            ret_fault = TheISA::genMachineCheckFault();
            return false;
        }
#endif

        // Build packet here.
        PacketPtr data_pkt = new Packet(mem_req,
                                        Packet::ReadReq, Packet::Broadcast);
        data_pkt->dataStatic(cacheData[tid]);

        DPRINTF(Fetch, "Fetch: Doing instruction read.\n");

        fetchedCacheLines++;

        // Now do the timing access to see whether or not the instruction
        // exists within the cache.
        if (!icachePort->sendTiming(data_pkt)) {
            assert(retryPkt == NULL);
            assert(retryTid == -1);
            DPRINTF(Fetch, "[tid:%i] Out of MSHRs!\n", tid);
            fetchStatus[tid] = IcacheWaitRetry;
            retryPkt = data_pkt;
            retryTid = tid;
            cacheBlocked = true;
            return false;
        }

        DPRINTF(Fetch, "Doing cache access.\n");

        lastIcacheStall[tid] = curTick;

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
DefaultFetch<Impl>::doSquash(const Addr &new_PC, unsigned tid)
{
    DPRINTF(Fetch, "[tid:%i]: Squashing, setting PC to: %#x.\n",
            tid, new_PC);

    PC[tid] = new_PC;
    nextPC[tid] = new_PC + instSize;

    // Clear the icache miss if it's outstanding.
    if (fetchStatus[tid] == IcacheWaitResponse) {
        DPRINTF(Fetch, "[tid:%i]: Squashing outstanding Icache miss.\n",
                tid);
        memReq[tid] = NULL;
    }

    // Get rid of the retrying packet if it was from this thread.
    if (retryTid == tid) {
        assert(cacheBlocked);
        cacheBlocked = false;
        retryTid = -1;
        retryPkt = NULL;
        delete retryPkt->req;
        delete retryPkt;
    }

    fetchStatus[tid] = Squashing;

    ++fetchSquashCycles;
}

template<class Impl>
void
DefaultFetch<Impl>::squashFromDecode(const Addr &new_PC,
                                     const InstSeqNum &seq_num,
                                     unsigned tid)
{
    DPRINTF(Fetch, "[tid:%i]: Squashing from decode.\n",tid);

    doSquash(new_PC, tid);

    // Tell the CPU to remove any instructions that are in flight between
    // fetch and decode.
    cpu->removeInstsUntil(seq_num, tid);
}

template<class Impl>
bool
DefaultFetch<Impl>::checkStall(unsigned tid) const
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
    list<unsigned>::iterator threads = (*activeThreads).begin();

    while (threads != (*activeThreads).end()) {

        unsigned tid = *threads++;

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
DefaultFetch<Impl>::squash(const Addr &new_PC, unsigned tid)
{
    DPRINTF(Fetch, "[tid:%u]: Squash from commit.\n",tid);

    doSquash(new_PC, tid);

    // Tell the CPU to remove any instructions that are not in the ROB.
    cpu->removeInstsNotInROB(tid);
}

template <class Impl>
void
DefaultFetch<Impl>::tick()
{
    list<unsigned>::iterator threads = (*activeThreads).begin();
    bool status_change = false;

    wroteToTimeBuffer = false;

    while (threads != (*activeThreads).end()) {
        unsigned tid = *threads++;

        // Check the signals for each thread to determine the proper status
        // for each thread.
        bool updated_status = checkSignalsAndUpdate(tid);
        status_change =  status_change || updated_status;
    }

    DPRINTF(Fetch, "Running stage.\n");

    // Reset the number of the instruction we're fetching.
    numInst = 0;

    if (fromCommit->commitInfo[0].interruptPending) {
        interruptPending = true;
    }
    if (fromCommit->commitInfo[0].clearInterrupt) {
        interruptPending = false;
    }

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
DefaultFetch<Impl>::checkSignalsAndUpdate(unsigned tid)
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
        squash(fromCommit->commitInfo[tid].nextPC,tid);

        // Also check if there's a mispredict that happened.
        if (fromCommit->commitInfo[tid].branchMispredict) {
            branchPred.squash(fromCommit->commitInfo[tid].doneSeqNum,
                              fromCommit->commitInfo[tid].nextPC,
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
            // Squash unless we're already squashing
            squashFromDecode(fromDecode->decodeInfo[tid].nextPC,
                             fromDecode->decodeInfo[tid].doneSeqNum,
                             tid);

            return true;
        }
    }

    if (checkStall(tid) && fetchStatus[tid] != IcacheWaitResponse) {
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
void
DefaultFetch<Impl>::fetch(bool &status_change)
{
    //////////////////////////////////////////
    // Start actual fetch
    //////////////////////////////////////////
    int tid = getFetchingThread(fetchPolicy);

    if (tid == -1) {
        DPRINTF(Fetch,"There are no more threads available to fetch from.\n");

        // Breaks looping condition in tick()
        threadFetched = numFetchingThreads;
        return;
    }

    // The current PC.
    Addr &fetch_PC = PC[tid];

    // Fault code for memory access.
    Fault fault = NoFault;

    // If returning from the delay of a cache miss, then update the status
    // to running, otherwise do the cache access.  Possibly move this up
    // to tick() function.
    if (fetchStatus[tid] == IcacheAccessComplete) {
        DPRINTF(Fetch, "[tid:%i]: Icache miss is complete.\n",
                tid);

        fetchStatus[tid] = Running;
        status_change = true;
    } else if (fetchStatus[tid] == Running) {
        DPRINTF(Fetch, "[tid:%i]: Attempting to translate and read "
                "instruction, starting at PC %08p.\n",
                tid, fetch_PC);

        bool fetch_success = fetchCacheLine(fetch_PC, fault, tid);
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
        } else if (fetchStatus[tid] == Blocked) {
            ++fetchBlockedCycles;
        } else if (fetchStatus[tid] == Squashing) {
            ++fetchSquashCycles;
        } else if (fetchStatus[tid] == IcacheWaitResponse) {
            ++icacheStallCycles;
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

    Addr next_PC = fetch_PC;
    InstSeqNum inst_seq;
    MachInst inst;
    ExtMachInst ext_inst;
    // @todo: Fix this hack.
    unsigned offset = (fetch_PC & cacheBlkMask) & ~3;

    if (fault == NoFault) {
        // If the read of the first instruction was successful, then grab the
        // instructions from the rest of the cache line and put them into the
        // queue heading to decode.

        DPRINTF(Fetch, "[tid:%i]: Adding instructions to queue to "
                "decode.\n",tid);

        // Need to keep track of whether or not a predicted branch
        // ended this fetch block.
        bool predicted_branch = false;

        for (;
             offset < cacheBlkSize &&
                 numInst < fetchWidth &&
                 !predicted_branch;
             ++numInst) {

            // Get a sequence number.
            inst_seq = cpu->getAndIncrementInstSeq();

            // Make sure this is a valid index.
            assert(offset <= cacheBlkSize - instSize);

            // Get the instruction from the array of the cache line.
            inst = gtoh(*reinterpret_cast<MachInst *>
                        (&cacheData[tid][offset]));

            ext_inst = TheISA::makeExtMI(inst, fetch_PC);

            // Create a new DynInst from the instruction fetched.
            DynInstPtr instruction = new DynInst(ext_inst, fetch_PC,
                                                 next_PC,
                                                 inst_seq, cpu);
            instruction->setTid(tid);

            instruction->setASID(tid);

            instruction->setThreadState(cpu->thread[tid]);

            DPRINTF(Fetch, "[tid:%i]: Instruction PC %#x created "
                    "[sn:%lli]\n",
                    tid, instruction->readPC(), inst_seq);

            DPRINTF(Fetch, "[tid:%i]: Instruction is: %s\n",
                    tid, instruction->staticInst->disassemble(fetch_PC));

            instruction->traceData =
                Trace::getInstRecord(curTick, cpu->tcBase(tid), cpu,
                                     instruction->staticInst,
                                     instruction->readPC(),tid);

            predicted_branch = lookupAndUpdateNextPC(instruction, next_PC);

            // Add instruction to the CPU's list of instructions.
            instruction->setInstListIt(cpu->addInst(instruction));

            // Write the instruction to the first slot in the queue
            // that heads to decode.
            toDecode->insts[numInst] = instruction;

            toDecode->size++;

            // Increment stat of fetched instructions.
            ++fetchedInsts;

            // Move to the next instruction, unless we have a branch.
            fetch_PC = next_PC;

            if (instruction->isQuiesce()) {
                warn("cycle %lli: Quiesce instruction encountered, halting fetch!",
                     curTick);
                fetchStatus[tid] = QuiescePending;
                ++numInst;
                status_change = true;
                break;
            }

            offset+= instSize;
        }
    }

    if (numInst > 0) {
        wroteToTimeBuffer = true;
    }

    // Now that fetching is completed, update the PC to signify what the next
    // cycle will be.
    if (fault == NoFault) {
        DPRINTF(Fetch, "[tid:%i]: Setting PC to %08p.\n",tid, next_PC);

#if THE_ISA == ALPHA_ISA
        PC[tid] = next_PC;
        nextPC[tid] = next_PC + instSize;
#else
        PC[tid] = next_PC;
        nextPC[tid] = next_PC + instSize;
        nextPC[tid] = next_PC + instSize;

        thread->setNextPC(thread->readNextNPC());
        thread->setNextNPC(thread->readNextNPC() + sizeof(MachInst));
#endif
    } else {
        // We shouldn't be in an icache miss and also have a fault (an ITB
        // miss)
        if (fetchStatus[tid] == IcacheWaitResponse) {
            panic("Fetch should have exited prior to this!");
        }

        // Send the fault to commit.  This thread will not do anything
        // until commit handles the fault.  The only other way it can
        // wake up is if a squash comes along and changes the PC.
#if FULL_SYSTEM
        assert(numInst != fetchWidth);
        // Get a sequence number.
        inst_seq = cpu->getAndIncrementInstSeq();
        // We will use a nop in order to carry the fault.
        ext_inst = TheISA::NoopMachInst;

        // Create a new DynInst from the dummy nop.
        DynInstPtr instruction = new DynInst(ext_inst, fetch_PC,
                                             next_PC,
                                             inst_seq, cpu);
        instruction->setPredTarg(next_PC + instSize);
        instruction->setTid(tid);

        instruction->setASID(tid);

        instruction->setThreadState(cpu->thread[tid]);

        instruction->traceData = NULL;

        instruction->setInstListIt(cpu->addInst(instruction));

        instruction->fault = fault;

        toDecode->insts[numInst] = instruction;
        toDecode->size++;

        DPRINTF(Fetch, "[tid:%i]: Blocked, need to handle the trap.\n",tid);

        fetchStatus[tid] = TrapPending;
        status_change = true;

        warn("cycle %lli: fault (%d) detected @ PC %08p", curTick, fault, PC[tid]);
#else // !FULL_SYSTEM
        warn("cycle %lli: fault (%d) detected @ PC %08p", curTick, fault, PC[tid]);
#endif // FULL_SYSTEM
    }
}

template<class Impl>
void
DefaultFetch<Impl>::recvRetry()
{
    assert(cacheBlocked);
    if (retryPkt != NULL) {
        assert(retryTid != -1);
        assert(fetchStatus[retryTid] == IcacheWaitRetry);

        if (icachePort->sendTiming(retryPkt)) {
            fetchStatus[retryTid] = IcacheWaitResponse;
            retryPkt = NULL;
            retryTid = -1;
            cacheBlocked = false;
        }
    } else {
        assert(retryTid == -1);
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
int
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
            return -1;
        }
    } else {
        int tid = *((*activeThreads).begin());

        if (fetchStatus[tid] == Running ||
            fetchStatus[tid] == IcacheAccessComplete ||
            fetchStatus[tid] == Idle) {
            return tid;
        } else {
            return -1;
        }
    }

}


template<class Impl>
int
DefaultFetch<Impl>::roundRobin()
{
    list<unsigned>::iterator pri_iter = priorityList.begin();
    list<unsigned>::iterator end      = priorityList.end();

    int high_pri;

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

    return -1;
}

template<class Impl>
int
DefaultFetch<Impl>::iqCount()
{
    priority_queue<unsigned> PQ;

    list<unsigned>::iterator threads = (*activeThreads).begin();

    while (threads != (*activeThreads).end()) {
        unsigned tid = *threads++;

        PQ.push(fromIEW->iewInfo[tid].iqCount);
    }

    while (!PQ.empty()) {

        unsigned high_pri = PQ.top();

        if (fetchStatus[high_pri] == Running ||
            fetchStatus[high_pri] == IcacheAccessComplete ||
            fetchStatus[high_pri] == Idle)
            return high_pri;
        else
            PQ.pop();

    }

    return -1;
}

template<class Impl>
int
DefaultFetch<Impl>::lsqCount()
{
    priority_queue<unsigned> PQ;


    list<unsigned>::iterator threads = (*activeThreads).begin();

    while (threads != (*activeThreads).end()) {
        unsigned tid = *threads++;

        PQ.push(fromIEW->iewInfo[tid].ldstqCount);
    }

    while (!PQ.empty()) {

        unsigned high_pri = PQ.top();

        if (fetchStatus[high_pri] == Running ||
            fetchStatus[high_pri] == IcacheAccessComplete ||
            fetchStatus[high_pri] == Idle)
            return high_pri;
        else
            PQ.pop();

    }

    return -1;
}

template<class Impl>
int
DefaultFetch<Impl>::branchCount()
{
    list<unsigned>::iterator threads = (*activeThreads).begin();
    warn("Branch Count Fetch policy unimplemented\n");
    return *threads;
}
