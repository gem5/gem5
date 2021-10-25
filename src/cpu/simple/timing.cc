/*
 * Copyright 2014 Google, Inc.
 * Copyright (c) 2010-2013,2015,2017-2018, 2020-2021 ARM Limited
 * All rights reserved
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
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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

#include "cpu/simple/timing.hh"

#include "arch/generic/decoder.hh"
#include "base/compiler.hh"
#include "cpu/exetrace.hh"
#include "debug/Config.hh"
#include "debug/Drain.hh"
#include "debug/ExecFaulting.hh"
#include "debug/HtmCpu.hh"
#include "debug/Mwait.hh"
#include "debug/SimpleCPU.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/BaseTimingSimpleCPU.hh"
#include "sim/faults.hh"
#include "sim/full_system.hh"
#include "sim/system.hh"

namespace gem5
{

void
TimingSimpleCPU::init()
{
    BaseSimpleCPU::init();
}

void
TimingSimpleCPU::TimingCPUPort::TickEvent::schedule(PacketPtr _pkt, Tick t)
{
    pkt = _pkt;
    cpu->schedule(this, t);
}

TimingSimpleCPU::TimingSimpleCPU(const BaseTimingSimpleCPUParams &p)
    : BaseSimpleCPU(p), fetchTranslation(this), icachePort(this),
      dcachePort(this), ifetch_pkt(NULL), dcache_pkt(NULL), previousCycle(0),
      fetchEvent([this]{ fetch(); }, name())
{
    _status = Idle;
}



TimingSimpleCPU::~TimingSimpleCPU()
{
}

DrainState
TimingSimpleCPU::drain()
{
    // Deschedule any power gating event (if any)
    deschedulePowerGatingEvent();

    if (switchedOut())
        return DrainState::Drained;

    if (_status == Idle ||
        (_status == BaseSimpleCPU::Running && isCpuDrained())) {
        DPRINTF(Drain, "No need to drain.\n");
        activeThreads.clear();
        return DrainState::Drained;
    } else {
        DPRINTF(Drain, "Requesting drain.\n");

        // The fetch event can become descheduled if a drain didn't
        // succeed on the first attempt. We need to reschedule it if
        // the CPU is waiting for a microcode routine to complete.
        if (_status == BaseSimpleCPU::Running && !fetchEvent.scheduled())
            schedule(fetchEvent, clockEdge());

        return DrainState::Draining;
    }
}

void
TimingSimpleCPU::drainResume()
{
    assert(!fetchEvent.scheduled());
    if (switchedOut())
        return;

    DPRINTF(SimpleCPU, "Resume\n");
    verifyMemoryMode();

    assert(!threadContexts.empty());

    _status = BaseSimpleCPU::Idle;

    for (ThreadID tid = 0; tid < numThreads; tid++) {
        if (threadInfo[tid]->thread->status() == ThreadContext::Active) {
            threadInfo[tid]->execContextStats.notIdleFraction = 1;

            activeThreads.push_back(tid);

            _status = BaseSimpleCPU::Running;

            // Fetch if any threads active
            if (!fetchEvent.scheduled()) {
                schedule(fetchEvent, nextCycle());
            }
        } else {
            threadInfo[tid]->execContextStats.notIdleFraction = 0;
        }
    }

    // Reschedule any power gating event (if any)
    schedulePowerGatingEvent();
}

bool
TimingSimpleCPU::tryCompleteDrain()
{
    if (drainState() != DrainState::Draining)
        return false;

    DPRINTF(Drain, "tryCompleteDrain.\n");
    if (!isCpuDrained())
        return false;

    DPRINTF(Drain, "CPU done draining, processing drain event\n");
    signalDrainDone();

    return true;
}

void
TimingSimpleCPU::switchOut()
{
    SimpleExecContext& t_info = *threadInfo[curThread];
    [[maybe_unused]] SimpleThread* thread = t_info.thread;

    // hardware transactional memory
    // Cannot switch out the CPU in the middle of a transaction
    assert(!t_info.inHtmTransactionalState());

    BaseSimpleCPU::switchOut();

    assert(!fetchEvent.scheduled());
    assert(_status == BaseSimpleCPU::Running || _status == Idle);
    assert(!t_info.stayAtPC);
    assert(thread->pcState().microPC() == 0);

    updateCycleCounts();
    updateCycleCounters(BaseCPU::CPU_STATE_ON);
}


void
TimingSimpleCPU::takeOverFrom(BaseCPU *oldCPU)
{
    BaseSimpleCPU::takeOverFrom(oldCPU);

    previousCycle = curCycle();
}

void
TimingSimpleCPU::verifyMemoryMode() const
{
    if (!system->isTimingMode()) {
        fatal("The timing CPU requires the memory system to be in "
              "'timing' mode.\n");
    }
}

void
TimingSimpleCPU::activateContext(ThreadID thread_num)
{
    DPRINTF(SimpleCPU, "ActivateContext %d\n", thread_num);

    assert(thread_num < numThreads);

    threadInfo[thread_num]->execContextStats.notIdleFraction = 1;
    if (_status == BaseSimpleCPU::Idle)
        _status = BaseSimpleCPU::Running;

    // kick things off by initiating the fetch of the next instruction
    if (!fetchEvent.scheduled())
        schedule(fetchEvent, clockEdge(Cycles(0)));

    if (std::find(activeThreads.begin(), activeThreads.end(), thread_num)
         == activeThreads.end()) {
        activeThreads.push_back(thread_num);
    }

    BaseCPU::activateContext(thread_num);
}


void
TimingSimpleCPU::suspendContext(ThreadID thread_num)
{
    DPRINTF(SimpleCPU, "SuspendContext %d\n", thread_num);

    assert(thread_num < numThreads);
    activeThreads.remove(thread_num);

    // hardware transactional memory
    // Cannot suspend context in the middle of a transaction.
    assert(!threadInfo[curThread]->inHtmTransactionalState());

    if (_status == Idle)
        return;

    assert(_status == BaseSimpleCPU::Running);

    threadInfo[thread_num]->execContextStats.notIdleFraction = 0;

    if (activeThreads.empty()) {
        _status = Idle;

        if (fetchEvent.scheduled()) {
            deschedule(fetchEvent);
        }
    }

    BaseCPU::suspendContext(thread_num);
}

bool
TimingSimpleCPU::handleReadPacket(PacketPtr pkt)
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    const RequestPtr &req = pkt->req;

    // hardware transactional memory
    // sanity check
    if (req->isHTMCmd()) {
        assert(!req->isLocalAccess());
    }

    // We're about the issues a locked load, so tell the monitor
    // to start caring about this address
    if (pkt->isRead() && pkt->req->isLLSC()) {
        thread->getIsaPtr()->handleLockedRead(pkt->req);
    }
    if (req->isLocalAccess()) {
        Cycles delay = req->localAccessor(thread->getTC(), pkt);
        new IprEvent(pkt, this, clockEdge(delay));
        _status = DcacheWaitResponse;
        dcache_pkt = NULL;
    } else if (!dcachePort.sendTimingReq(pkt)) {
        _status = DcacheRetry;
        dcache_pkt = pkt;
    } else {
        _status = DcacheWaitResponse;
        // memory system takes ownership of packet
        dcache_pkt = NULL;
    }
    return dcache_pkt == NULL;
}

void
TimingSimpleCPU::sendData(const RequestPtr &req, uint8_t *data, uint64_t *res,
                          bool read)
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    PacketPtr pkt = buildPacket(req, read);
    pkt->dataDynamic<uint8_t>(data);

    // hardware transactional memory
    // If the core is in transactional mode or if the request is HtmCMD
    // to abort a transaction, the packet should reflect that it is
    // transactional and also contain a HtmUid for debugging.
    const bool is_htm_speculative = t_info.inHtmTransactionalState();
    if (is_htm_speculative || req->isHTMAbort()) {
        pkt->setHtmTransactional(t_info.getHtmTransactionUid());
    }
    if (req->isHTMAbort())
        DPRINTF(HtmCpu, "htmabort htmUid=%u\n", t_info.getHtmTransactionUid());

    if (req->getFlags().isSet(Request::NO_ACCESS)) {
        assert(!dcache_pkt);
        pkt->makeResponse();
        completeDataAccess(pkt);
    } else if (read) {
        handleReadPacket(pkt);
    } else {
        bool do_access = true;  // flag to suppress cache access

        if (req->isLLSC()) {
            do_access = thread->getIsaPtr()->handleLockedWrite(
                    req, dcachePort.cacheBlockMask);
        } else if (req->isCondSwap()) {
            assert(res);
            req->setExtraData(*res);
        }

        if (do_access) {
            dcache_pkt = pkt;
            handleWritePacket();
            threadSnoop(pkt, curThread);
        } else {
            _status = DcacheWaitResponse;
            completeDataAccess(pkt);
        }
    }
}

void
TimingSimpleCPU::sendSplitData(const RequestPtr &req1, const RequestPtr &req2,
                               const RequestPtr &req, uint8_t *data, bool read)
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    PacketPtr pkt1, pkt2;
    buildSplitPacket(pkt1, pkt2, req1, req2, req, data, read);

    // hardware transactional memory
    // HTM commands should never use SplitData
    assert(!req1->isHTMCmd() && !req2->isHTMCmd());

    // If the thread is executing transactionally,
    // reflect this in the packets.
    if (t_info.inHtmTransactionalState()) {
        pkt1->setHtmTransactional(t_info.getHtmTransactionUid());
        pkt2->setHtmTransactional(t_info.getHtmTransactionUid());
    }

    if (req->getFlags().isSet(Request::NO_ACCESS)) {
        assert(!dcache_pkt);
        pkt1->makeResponse();
        completeDataAccess(pkt1);
    } else if (read) {
        SplitFragmentSenderState * send_state =
            dynamic_cast<SplitFragmentSenderState *>(pkt1->senderState);
        if (handleReadPacket(pkt1)) {
            send_state->clearFromParent();
            send_state = dynamic_cast<SplitFragmentSenderState *>(
                    pkt2->senderState);
            if (handleReadPacket(pkt2)) {
                send_state->clearFromParent();
            }
        }
    } else {
        dcache_pkt = pkt1;
        SplitFragmentSenderState * send_state =
            dynamic_cast<SplitFragmentSenderState *>(pkt1->senderState);
        if (handleWritePacket()) {
            send_state->clearFromParent();
            dcache_pkt = pkt2;
            send_state = dynamic_cast<SplitFragmentSenderState *>(
                    pkt2->senderState);
            if (handleWritePacket()) {
                send_state->clearFromParent();
            }
        }
    }
}

void
TimingSimpleCPU::translationFault(const Fault &fault)
{
    // fault may be NoFault in cases where a fault is suppressed,
    // for instance prefetches.
    updateCycleCounts();
    updateCycleCounters(BaseCPU::CPU_STATE_ON);

    if ((fault != NoFault) && traceData) {
        traceFault();
    }

    postExecute();

    advanceInst(fault);
}

PacketPtr
TimingSimpleCPU::buildPacket(const RequestPtr &req, bool read)
{
    return read ? Packet::createRead(req) : Packet::createWrite(req);
}

void
TimingSimpleCPU::buildSplitPacket(PacketPtr &pkt1, PacketPtr &pkt2,
        const RequestPtr &req1, const RequestPtr &req2, const RequestPtr &req,
        uint8_t *data, bool read)
{
    pkt1 = pkt2 = NULL;

    assert(!req1->isLocalAccess() && !req2->isLocalAccess());

    if (req->getFlags().isSet(Request::NO_ACCESS)) {
        pkt1 = buildPacket(req, read);
        return;
    }

    pkt1 = buildPacket(req1, read);
    pkt2 = buildPacket(req2, read);

    PacketPtr pkt = new Packet(req, pkt1->cmd.responseCommand());

    pkt->dataDynamic<uint8_t>(data);
    pkt1->dataStatic<uint8_t>(data);
    pkt2->dataStatic<uint8_t>(data + req1->getSize());

    SplitMainSenderState * main_send_state = new SplitMainSenderState;
    pkt->senderState = main_send_state;
    main_send_state->fragments[0] = pkt1;
    main_send_state->fragments[1] = pkt2;
    main_send_state->outstanding = 2;
    pkt1->senderState = new SplitFragmentSenderState(pkt, 0);
    pkt2->senderState = new SplitFragmentSenderState(pkt, 1);
}

Fault
TimingSimpleCPU::initiateMemRead(Addr addr, unsigned size,
                                 Request::Flags flags,
                                 const std::vector<bool>& byte_enable)
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    Fault fault;
    const Addr pc = thread->pcState().instAddr();
    unsigned block_size = cacheLineSize();
    BaseMMU::Mode mode = BaseMMU::Read;

    if (traceData)
        traceData->setMem(addr, size, flags);

    RequestPtr req = std::make_shared<Request>(
        addr, size, flags, dataRequestorId(), pc, thread->contextId());
    req->setByteEnable(byte_enable);

    req->taskId(taskId());

    Addr split_addr = roundDown(addr + size - 1, block_size);
    assert(split_addr <= addr || split_addr - addr < block_size);

    _status = DTBWaitResponse;
    if (split_addr > addr) {
        RequestPtr req1, req2;
        assert(!req->isLLSC() && !req->isSwap());
        req->splitOnVaddr(split_addr, req1, req2);

        WholeTranslationState *state =
            new WholeTranslationState(req, req1, req2, new uint8_t[size],
                                      NULL, mode);
        DataTranslation<TimingSimpleCPU *> *trans1 =
            new DataTranslation<TimingSimpleCPU *>(this, state, 0);
        DataTranslation<TimingSimpleCPU *> *trans2 =
            new DataTranslation<TimingSimpleCPU *>(this, state, 1);

        thread->mmu->translateTiming(req1, thread->getTC(), trans1, mode);
        thread->mmu->translateTiming(req2, thread->getTC(), trans2, mode);
    } else {
        WholeTranslationState *state =
            new WholeTranslationState(req, new uint8_t[size], NULL, mode);
        DataTranslation<TimingSimpleCPU *> *translation
            = new DataTranslation<TimingSimpleCPU *>(this, state);
        thread->mmu->translateTiming(req, thread->getTC(), translation, mode);
    }

    return NoFault;
}

bool
TimingSimpleCPU::handleWritePacket()
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    const RequestPtr &req = dcache_pkt->req;
    if (req->isLocalAccess()) {
        Cycles delay = req->localAccessor(thread->getTC(), dcache_pkt);
        new IprEvent(dcache_pkt, this, clockEdge(delay));
        _status = DcacheWaitResponse;
        dcache_pkt = NULL;
    } else if (!dcachePort.sendTimingReq(dcache_pkt)) {
        _status = DcacheRetry;
    } else {
        _status = DcacheWaitResponse;
        // memory system takes ownership of packet
        dcache_pkt = NULL;
    }
    return dcache_pkt == NULL;
}

Fault
TimingSimpleCPU::writeMem(uint8_t *data, unsigned size,
                          Addr addr, Request::Flags flags, uint64_t *res,
                          const std::vector<bool>& byte_enable)
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    uint8_t *newData = new uint8_t[size];
    const Addr pc = thread->pcState().instAddr();
    unsigned block_size = cacheLineSize();
    BaseMMU::Mode mode = BaseMMU::Write;

    if (data == NULL) {
        assert(flags & Request::STORE_NO_DATA);
        // This must be a cache block cleaning request
        memset(newData, 0, size);
    } else {
        memcpy(newData, data, size);
    }

    if (traceData)
        traceData->setMem(addr, size, flags);

    RequestPtr req = std::make_shared<Request>(
        addr, size, flags, dataRequestorId(), pc, thread->contextId());
    req->setByteEnable(byte_enable);

    req->taskId(taskId());

    Addr split_addr = roundDown(addr + size - 1, block_size);
    assert(split_addr <= addr || split_addr - addr < block_size);

    _status = DTBWaitResponse;

    // TODO: TimingSimpleCPU doesn't support arbitrarily long multi-line mem.
    // accesses yet

    if (split_addr > addr) {
        RequestPtr req1, req2;
        assert(!req->isLLSC() && !req->isSwap());
        req->splitOnVaddr(split_addr, req1, req2);

        WholeTranslationState *state =
            new WholeTranslationState(req, req1, req2, newData, res, mode);
        DataTranslation<TimingSimpleCPU *> *trans1 =
            new DataTranslation<TimingSimpleCPU *>(this, state, 0);
        DataTranslation<TimingSimpleCPU *> *trans2 =
            new DataTranslation<TimingSimpleCPU *>(this, state, 1);

        thread->mmu->translateTiming(req1, thread->getTC(), trans1, mode);
        thread->mmu->translateTiming(req2, thread->getTC(), trans2, mode);
    } else {
        WholeTranslationState *state =
            new WholeTranslationState(req, newData, res, mode);
        DataTranslation<TimingSimpleCPU *> *translation =
            new DataTranslation<TimingSimpleCPU *>(this, state);
        thread->mmu->translateTiming(req, thread->getTC(), translation, mode);
    }

    // Translation faults will be returned via finishTranslation()
    return NoFault;
}

Fault
TimingSimpleCPU::initiateMemAMO(Addr addr, unsigned size,
                                Request::Flags flags,
                                AtomicOpFunctorPtr amo_op)
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    Fault fault;
    const Addr pc = thread->pcState().instAddr();
    unsigned block_size = cacheLineSize();
    BaseMMU::Mode mode = BaseMMU::Write;

    if (traceData)
        traceData->setMem(addr, size, flags);

    RequestPtr req = std::make_shared<Request>(addr, size, flags,
                            dataRequestorId(), pc, thread->contextId(),
                            std::move(amo_op));

    assert(req->hasAtomicOpFunctor());

    req->taskId(taskId());

    Addr split_addr = roundDown(addr + size - 1, block_size);

    // AMO requests that access across a cache line boundary are not
    // allowed since the cache does not guarantee AMO ops to be executed
    // atomically in two cache lines
    // For ISAs such as x86 that requires AMO operations to work on
    // accesses that cross cache-line boundaries, the cache needs to be
    // modified to support locking both cache lines to guarantee the
    // atomicity.
    if (split_addr > addr) {
        panic("AMO requests should not access across a cache line boundary\n");
    }

    _status = DTBWaitResponse;

    WholeTranslationState *state =
        new WholeTranslationState(req, new uint8_t[size], NULL, mode);
    DataTranslation<TimingSimpleCPU *> *translation
        = new DataTranslation<TimingSimpleCPU *>(this, state);
    thread->mmu->translateTiming(req, thread->getTC(), translation, mode);

    return NoFault;
}

void
TimingSimpleCPU::threadSnoop(PacketPtr pkt, ThreadID sender)
{
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        if (tid != sender) {
            if (getCpuAddrMonitor(tid)->doMonitor(pkt)) {
                wakeup(tid);
            }
            threadInfo[tid]->thread->getIsaPtr()->handleLockedSnoop(pkt,
                    dcachePort.cacheBlockMask);
        }
    }
}

void
TimingSimpleCPU::finishTranslation(WholeTranslationState *state)
{
    _status = BaseSimpleCPU::Running;

    if (state->getFault() != NoFault) {
        if (state->isPrefetch()) {
            state->setNoFault();
        }
        delete [] state->data;
        state->deleteReqs();
        translationFault(state->getFault());
    } else {
        if (!state->isSplit) {
            sendData(state->mainReq, state->data, state->res,
                     state->mode == BaseMMU::Read);
        } else {
            sendSplitData(state->sreqLow, state->sreqHigh, state->mainReq,
                          state->data, state->mode == BaseMMU::Read);
        }
    }

    delete state;
}


void
TimingSimpleCPU::fetch()
{
    // Change thread if multi-threaded
    swapActiveThread();

    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    DPRINTF(SimpleCPU, "Fetch\n");

    if (!curStaticInst || !curStaticInst->isDelayedCommit()) {
        checkForInterrupts();
        checkPcEventQueue();
    }

    // We must have just got suspended by a PC event
    if (_status == Idle)
        return;

    MicroPC upc = thread->pcState().microPC();
    bool needToFetch = !isRomMicroPC(upc) && !curMacroStaticInst;

    if (needToFetch) {
        _status = BaseSimpleCPU::Running;
        RequestPtr ifetch_req = std::make_shared<Request>();
        ifetch_req->taskId(taskId());
        ifetch_req->setContext(thread->contextId());
        setupFetchRequest(ifetch_req);
        DPRINTF(SimpleCPU, "Translating address %#x\n", ifetch_req->getVaddr());
        thread->mmu->translateTiming(ifetch_req, thread->getTC(),
                &fetchTranslation, BaseMMU::Execute);
    } else {
        _status = IcacheWaitResponse;
        completeIfetch(NULL);

        updateCycleCounts();
        updateCycleCounters(BaseCPU::CPU_STATE_ON);
    }
}


void
TimingSimpleCPU::sendFetch(const Fault &fault, const RequestPtr &req,
                           ThreadContext *tc)
{
    auto &decoder = threadInfo[curThread]->thread->decoder;

    if (fault == NoFault) {
        DPRINTF(SimpleCPU, "Sending fetch for addr %#x(pa: %#x)\n",
                req->getVaddr(), req->getPaddr());
        ifetch_pkt = new Packet(req, MemCmd::ReadReq);
        ifetch_pkt->dataStatic(decoder->moreBytesPtr());
        DPRINTF(SimpleCPU, " -- pkt addr: %#x\n", ifetch_pkt->getAddr());

        if (!icachePort.sendTimingReq(ifetch_pkt)) {
            // Need to wait for retry
            _status = IcacheRetry;
        } else {
            // Need to wait for cache to respond
            _status = IcacheWaitResponse;
            // ownership of packet transferred to memory system
            ifetch_pkt = NULL;
        }
    } else {
        DPRINTF(SimpleCPU, "Translation of addr %#x faulted\n", req->getVaddr());
        // fetch fault: advance directly to next instruction (fault handler)
        _status = BaseSimpleCPU::Running;
        advanceInst(fault);
    }

    updateCycleCounts();
    updateCycleCounters(BaseCPU::CPU_STATE_ON);
}


void
TimingSimpleCPU::advanceInst(const Fault &fault)
{
    SimpleExecContext &t_info = *threadInfo[curThread];

    if (_status == Faulting)
        return;

    if (fault != NoFault) {
        // hardware transactional memory
        // If a fault occurred within a transaction
        // ensure that the transaction aborts
        if (t_info.inHtmTransactionalState() &&
            !std::dynamic_pointer_cast<GenericHtmFailureFault>(fault)) {
            DPRINTF(HtmCpu, "fault (%s) occurred - "
                "replacing with HTM abort fault htmUid=%u\n",
                fault->name(), t_info.getHtmTransactionUid());

            Fault tmfault = std::make_shared<GenericHtmFailureFault>(
                t_info.getHtmTransactionUid(),
                HtmFailureFaultCause::EXCEPTION);

            advancePC(tmfault);
            reschedule(fetchEvent, clockEdge(), true);
            _status = Faulting;
            return;
        }

        DPRINTF(SimpleCPU, "Fault occured. Handling the fault\n");

        advancePC(fault);

        // A syscall fault could suspend this CPU (e.g., futex_wait)
        // If the _status is not Idle, schedule an event to fetch the next
        // instruction after 'stall' ticks.
        // If the cpu has been suspended (i.e., _status == Idle), another
        // cpu will wake this cpu up later.
        if (_status != Idle) {
            DPRINTF(SimpleCPU, "Scheduling fetch event after the Fault\n");

            Tick stall = std::dynamic_pointer_cast<SyscallRetryFault>(fault) ?
                         clockEdge(syscallRetryLatency) : clockEdge();
            reschedule(fetchEvent, stall, true);
            _status = Faulting;
        }

        return;
    }

    if (!t_info.stayAtPC)
        advancePC(fault);

    if (tryCompleteDrain())
        return;

    serviceInstCountEvents();

    if (_status == BaseSimpleCPU::Running) {
        // kick off fetch of next instruction... callback from icache
        // response will cause that instruction to be executed,
        // keeping the CPU running.
        fetch();
    }
}


void
TimingSimpleCPU::completeIfetch(PacketPtr pkt)
{
    SimpleExecContext& t_info = *threadInfo[curThread];

    DPRINTF(SimpleCPU, "Complete ICache Fetch for addr %#x\n", pkt ?
            pkt->getAddr() : 0);

    // received a response from the icache: execute the received
    // instruction
    panic_if(pkt && pkt->isError(), "Instruction fetch (%s) failed: %s",
            pkt->getAddrRange().to_string(), pkt->print());
    assert(_status == IcacheWaitResponse);

    _status = BaseSimpleCPU::Running;

    updateCycleCounts();
    updateCycleCounters(BaseCPU::CPU_STATE_ON);

    if (pkt)
        pkt->req->setAccessLatency();


    preExecute();

    // hardware transactional memory
    if (curStaticInst && curStaticInst->isHtmStart()) {
        // if this HtmStart is not within a transaction,
        // then assign it a new htmTransactionUid
        if (!t_info.inHtmTransactionalState())
            t_info.newHtmTransactionUid();
        SimpleThread* thread = t_info.thread;
        thread->htmTransactionStarts++;
        DPRINTF(HtmCpu, "htmTransactionStarts++=%u\n",
            thread->htmTransactionStarts);
    }

    if (curStaticInst && curStaticInst->isMemRef()) {
        // load or store: just send to dcache
        Fault fault = curStaticInst->initiateAcc(&t_info, traceData);

        // If we're not running now the instruction will complete in a dcache
        // response callback or the instruction faulted and has started an
        // ifetch
        if (_status == BaseSimpleCPU::Running) {
            if (fault != NoFault && traceData) {
                traceFault();
            }

            postExecute();
            // @todo remove me after debugging with legion done
            if (curStaticInst && (!curStaticInst->isMicroop() ||
                        curStaticInst->isFirstMicroop()))
                instCnt++;
            advanceInst(fault);
        }
    } else if (curStaticInst) {
        // non-memory instruction: execute completely now
        Fault fault = curStaticInst->execute(&t_info, traceData);

        // keep an instruction count
        if (fault == NoFault)
            countInst();
        else if (traceData) {
            traceFault();
        }

        postExecute();
        // @todo remove me after debugging with legion done
        if (curStaticInst && (!curStaticInst->isMicroop() ||
                curStaticInst->isFirstMicroop()))
            instCnt++;
        advanceInst(fault);
    } else {
        advanceInst(NoFault);
    }

    if (pkt) {
        delete pkt;
    }
}

void
TimingSimpleCPU::IcachePort::ITickEvent::process()
{
    cpu->completeIfetch(pkt);
}

bool
TimingSimpleCPU::IcachePort::recvTimingResp(PacketPtr pkt)
{
    DPRINTF(SimpleCPU, "Received fetch response %#x\n", pkt->getAddr());

    // hardware transactional memory
    // Currently, there is no support for tracking instruction fetches
    // in an transaction's read set.
    if (pkt->htmTransactionFailedInCache()) {
        panic("HTM transactional support for"
              " instruction stream not yet supported\n");
    }

    // we should only ever see one response per cycle since we only
    // issue a new request once this response is sunk
    assert(!tickEvent.scheduled());
    // delay processing of returned data until next CPU clock edge
    tickEvent.schedule(pkt, cpu->clockEdge());

    return true;
}

void
TimingSimpleCPU::IcachePort::recvReqRetry()
{
    // we shouldn't get a retry unless we have a packet that we're
    // waiting to transmit
    assert(cpu->ifetch_pkt != NULL);
    assert(cpu->_status == IcacheRetry);
    PacketPtr tmp = cpu->ifetch_pkt;
    if (sendTimingReq(tmp)) {
        cpu->_status = IcacheWaitResponse;
        cpu->ifetch_pkt = NULL;
    }
}

void
TimingSimpleCPU::completeDataAccess(PacketPtr pkt)
{
    // hardware transactional memory

    SimpleExecContext *t_info = threadInfo[curThread];
    [[maybe_unused]] const bool is_htm_speculative =
        t_info->inHtmTransactionalState();

    // received a response from the dcache: complete the load or store
    // instruction
    panic_if(pkt->isError(), "Data access (%s) failed: %s",
            pkt->getAddrRange().to_string(), pkt->print());
    assert(_status == DcacheWaitResponse || _status == DTBWaitResponse ||
           pkt->req->getFlags().isSet(Request::NO_ACCESS));

    pkt->req->setAccessLatency();

    updateCycleCounts();
    updateCycleCounters(BaseCPU::CPU_STATE_ON);

    if (pkt->senderState) {
        // hardware transactional memory
        // There shouldn't be HtmCmds occurring in multipacket requests
        if (pkt->req->isHTMCmd()) {
            panic("unexpected HTM case");
        }

        SplitFragmentSenderState * send_state =
            dynamic_cast<SplitFragmentSenderState *>(pkt->senderState);
        assert(send_state);
        PacketPtr big_pkt = send_state->bigPkt;
        delete send_state;

        if (pkt->isHtmTransactional()) {
            assert(is_htm_speculative);

            big_pkt->setHtmTransactional(
                pkt->getHtmTransactionUid()
            );
        }

        if (pkt->htmTransactionFailedInCache()) {
            assert(is_htm_speculative);
            big_pkt->setHtmTransactionFailedInCache(
                pkt->getHtmTransactionFailedInCacheRC()
            );
        }

        delete pkt;

        SplitMainSenderState * main_send_state =
            dynamic_cast<SplitMainSenderState *>(big_pkt->senderState);
        assert(main_send_state);
        // Record the fact that this packet is no longer outstanding.
        assert(main_send_state->outstanding != 0);
        main_send_state->outstanding--;

        if (main_send_state->outstanding) {
            return;
        } else {
            delete main_send_state;
            big_pkt->senderState = NULL;
            pkt = big_pkt;
        }
    }

    _status = BaseSimpleCPU::Running;

    Fault fault;

    // hardware transactional memory
    // sanity checks
    // ensure htmTransactionUids are equivalent
    if (pkt->isHtmTransactional())
        assert (pkt->getHtmTransactionUid() ==
                t_info->getHtmTransactionUid());

    // can't have a packet that fails a transaction while not in a transaction
    if (pkt->htmTransactionFailedInCache())
        assert(is_htm_speculative);

    // shouldn't fail through stores because this would be inconsistent w/ O3
    // which cannot fault after the store has been sent to memory
    if (pkt->htmTransactionFailedInCache() && !pkt->isWrite()) {
        const HtmCacheFailure htm_rc =
            pkt->getHtmTransactionFailedInCacheRC();
        DPRINTF(HtmCpu, "HTM abortion in cache (rc=%s) detected htmUid=%u\n",
            htmFailureToStr(htm_rc), pkt->getHtmTransactionUid());

        // Currently there are only two reasons why a transaction would
        // fail in the memory subsystem--
        // (1) A transactional line was evicted from the cache for
        //     space (or replacement policy) reasons.
        // (2) Another core/device requested a cache line that is in this
        //     transaction's read/write set that is incompatible with the
        //     HTM's semantics, e.g. another core requesting exclusive access
        //     of a line in this core's read set.
        if (htm_rc == HtmCacheFailure::FAIL_SELF) {
            fault = std::make_shared<GenericHtmFailureFault>(
                t_info->getHtmTransactionUid(),
                HtmFailureFaultCause::SIZE);
        } else if (htm_rc == HtmCacheFailure::FAIL_REMOTE) {
            fault = std::make_shared<GenericHtmFailureFault>(
                t_info->getHtmTransactionUid(),
                HtmFailureFaultCause::MEMORY);
        } else {
            panic("HTM - unhandled rc %s", htmFailureToStr(htm_rc));
        }
    } else {
        fault = curStaticInst->completeAcc(pkt, t_info,
                                     traceData);
    }

    // hardware transactional memory
    // Track HtmStop instructions,
    // e.g. instructions which commit a transaction.
    if (curStaticInst && curStaticInst->isHtmStop()) {
        t_info->thread->htmTransactionStops++;
        DPRINTF(HtmCpu, "htmTransactionStops++=%u\n",
            t_info->thread->htmTransactionStops);
    }

    // keep an instruction count
    if (fault == NoFault)
        countInst();
    else if (traceData) {
        traceFault();
    }

    delete pkt;

    postExecute();

    advanceInst(fault);
}

void
TimingSimpleCPU::updateCycleCounts()
{
    const Cycles delta(curCycle() - previousCycle);

    baseStats.numCycles += delta;

    previousCycle = curCycle();
}

void
TimingSimpleCPU::DcachePort::recvTimingSnoopReq(PacketPtr pkt)
{
    for (ThreadID tid = 0; tid < cpu->numThreads; tid++) {
        if (cpu->getCpuAddrMonitor(tid)->doMonitor(pkt)) {
            cpu->wakeup(tid);
        }
    }

    // Making it uniform across all CPUs:
    // The CPUs need to be woken up only on an invalidation packet
    // (when using caches) or on an incoming write packet (when not
    // using caches) It is not necessary to wake up the processor on
    // all incoming packets
    if (pkt->isInvalidate() || pkt->isWrite()) {
        for (auto &t_info : cpu->threadInfo) {
            t_info->thread->getIsaPtr()->handleLockedSnoop(pkt,
                    cacheBlockMask);
        }
    } else if (pkt->req && pkt->req->isTlbiExtSync()) {
        // We received a TLBI_EXT_SYNC request.
        // In a detailed sim we would wait for memory ops to complete,
        // but in our simple case we just respond immediately
        auto reply_req = Request::createMemManagement(
            Request::TLBI_EXT_SYNC_COMP,
            cpu->dataRequestorId());

        // Extra Data = the transaction ID of the Sync we're completing
        reply_req->setExtraData(pkt->req->getExtraData());
        PacketPtr reply_pkt = Packet::createRead(reply_req);

        // TODO - reserve some credit for these responses?
        if (!sendTimingReq(reply_pkt)) {
            panic("Couldn't send TLBI_EXT_SYNC_COMP message");
        }
    }
}

void
TimingSimpleCPU::DcachePort::recvFunctionalSnoop(PacketPtr pkt)
{
    for (ThreadID tid = 0; tid < cpu->numThreads; tid++) {
        if (cpu->getCpuAddrMonitor(tid)->doMonitor(pkt)) {
            cpu->wakeup(tid);
        }
    }
}

bool
TimingSimpleCPU::DcachePort::recvTimingResp(PacketPtr pkt)
{
    DPRINTF(SimpleCPU, "Received load/store response %#x\n", pkt->getAddr());

    // The timing CPU is not really ticked, instead it relies on the
    // memory system (fetch and load/store) to set the pace.
    if (!tickEvent.scheduled()) {
        // Delay processing of returned data until next CPU clock edge
        tickEvent.schedule(pkt, cpu->clockEdge());
        return true;
    } else {
        // In the case of a split transaction and a cache that is
        // faster than a CPU we could get two responses in the
        // same tick, delay the second one
        if (!retryRespEvent.scheduled())
            cpu->schedule(retryRespEvent, cpu->clockEdge(Cycles(1)));
        return false;
    }
}

void
TimingSimpleCPU::DcachePort::DTickEvent::process()
{
    cpu->completeDataAccess(pkt);
}

void
TimingSimpleCPU::DcachePort::recvReqRetry()
{
    // we shouldn't get a retry unless we have a packet that we're
    // waiting to transmit
    assert(cpu->dcache_pkt != NULL);
    assert(cpu->_status == DcacheRetry);
    PacketPtr tmp = cpu->dcache_pkt;
    if (tmp->senderState) {
        // This is a packet from a split access.
        SplitFragmentSenderState * send_state =
            dynamic_cast<SplitFragmentSenderState *>(tmp->senderState);
        assert(send_state);
        PacketPtr big_pkt = send_state->bigPkt;

        SplitMainSenderState * main_send_state =
            dynamic_cast<SplitMainSenderState *>(big_pkt->senderState);
        assert(main_send_state);

        if (sendTimingReq(tmp)) {
            // If we were able to send without retrying, record that fact
            // and try sending the other fragment.
            send_state->clearFromParent();
            int other_index = main_send_state->getPendingFragment();
            if (other_index > 0) {
                tmp = main_send_state->fragments[other_index];
                cpu->dcache_pkt = tmp;
                if ((big_pkt->isRead() && cpu->handleReadPacket(tmp)) ||
                        (big_pkt->isWrite() && cpu->handleWritePacket())) {
                    main_send_state->fragments[other_index] = NULL;
                }
            } else {
                cpu->_status = DcacheWaitResponse;
                // memory system takes ownership of packet
                cpu->dcache_pkt = NULL;
            }
        }
    } else if (sendTimingReq(tmp)) {
        cpu->_status = DcacheWaitResponse;
        // memory system takes ownership of packet
        cpu->dcache_pkt = NULL;
    }
}

TimingSimpleCPU::IprEvent::IprEvent(Packet *_pkt, TimingSimpleCPU *_cpu,
    Tick t)
    : pkt(_pkt), cpu(_cpu)
{
    cpu->schedule(this, t);
}

void
TimingSimpleCPU::IprEvent::process()
{
    cpu->completeDataAccess(pkt);
}

const char *
TimingSimpleCPU::IprEvent::description() const
{
    return "Timing Simple CPU Delay IPR event";
}


void
TimingSimpleCPU::printAddr(Addr a)
{
    dcachePort.printAddr(a);
}

Fault
TimingSimpleCPU::initiateMemMgmtCmd(Request::Flags flags)
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    const Addr addr = 0x0ul;
    const Addr pc = thread->pcState().instAddr();
    const int size = 8;

    if (traceData)
        traceData->setMem(addr, size, flags);

    RequestPtr req = std::make_shared<Request>(
        addr, size, flags, dataRequestorId());

    req->setPC(pc);
    req->setContext(thread->contextId());
    req->taskId(taskId());
    req->setInstCount(t_info.numInst);

    assert(req->isHTMCmd() || req->isTlbiCmd());

    // Use the payload as a sanity check,
    // the memory subsystem will clear allocated data
    uint8_t *data = new uint8_t[size];
    assert(data);
    uint64_t rc = 0xdeadbeeflu;
    memcpy (data, &rc, size);

    // debugging output
    if (req->isHTMCmd()) {
        if (req->isHTMStart())
            DPRINTF(HtmCpu, "HTMstart htmUid=%u\n",
                t_info.getHtmTransactionUid());
        else if (req->isHTMCommit())
            DPRINTF(HtmCpu, "HTMcommit htmUid=%u\n",
                t_info.getHtmTransactionUid());
        else if (req->isHTMCancel())
            DPRINTF(HtmCpu, "HTMcancel htmUid=%u\n",
                t_info.getHtmTransactionUid());
        else
            panic("initiateMemMgmtCmd: unknown HTM CMD");
    }

    sendData(req, data, nullptr, true);

    return NoFault;
}

void
TimingSimpleCPU::htmSendAbortSignal(ThreadID tid, uint64_t htm_uid,
                                    HtmFailureFaultCause cause)
{
    SimpleExecContext& t_info = *threadInfo[tid];
    SimpleThread* thread = t_info.thread;

    const Addr addr = 0x0ul;
    const Addr pc = thread->pcState().instAddr();
    const int size = 8;
    const Request::Flags flags =
        Request::PHYSICAL|Request::STRICT_ORDER|Request::HTM_ABORT;

    if (traceData)
        traceData->setMem(addr, size, flags);

    // notify l1 d-cache (ruby) that core has aborted transaction

    RequestPtr req = std::make_shared<Request>(
        addr, size, flags, dataRequestorId());

    req->setPC(pc);
    req->setContext(thread->contextId());
    req->taskId(taskId());
    req->setInstCount(t_info.numInst);
    req->setHtmAbortCause(cause);

    assert(req->isHTMAbort());

    uint8_t *data = new uint8_t[size];
    assert(data);
    uint64_t rc = 0lu;
    memcpy (data, &rc, size);

    sendData(req, data, nullptr, true);
}

} // namespace gem5
