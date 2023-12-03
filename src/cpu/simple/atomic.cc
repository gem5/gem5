/*
 * Copyright 2014 Google, Inc.
 * Copyright (c) 2012-2013,2015,2017-2020 ARM Limited
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

#include "cpu/simple/atomic.hh"

#include "arch/generic/decoder.hh"
#include "base/output.hh"
#include "cpu/exetrace.hh"
#include "cpu/utils.hh"
#include "debug/Drain.hh"
#include "debug/ExecFaulting.hh"
#include "debug/SimpleCPU.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "mem/physical.hh"
#include "params/BaseAtomicSimpleCPU.hh"
#include "sim/faults.hh"
#include "sim/full_system.hh"
#include "sim/system.hh"

namespace gem5
{

void
AtomicSimpleCPU::init()
{
    BaseSimpleCPU::init();

    int cid = threadContexts[0]->contextId();
    ifetch_req->setContext(cid);
    data_read_req->setContext(cid);
    data_write_req->setContext(cid);
    data_amo_req->setContext(cid);
}

AtomicSimpleCPU::AtomicSimpleCPU(const BaseAtomicSimpleCPUParams &p)
    : BaseSimpleCPU(p),
      tickEvent([this] { tick(); }, "AtomicSimpleCPU tick", false,
                Event::CPU_Tick_Pri),
      width(p.width),
      locked(false),
      simulate_data_stalls(p.simulate_data_stalls),
      simulate_inst_stalls(p.simulate_inst_stalls),
      icachePort(name() + ".icache_port"),
      dcachePort(name() + ".dcache_port", this),
      dcache_access(false),
      dcache_latency(0),
      ppCommit(nullptr)
{
    _status = Idle;
    ifetch_req = std::make_shared<Request>();
    data_read_req = std::make_shared<Request>();
    data_write_req = std::make_shared<Request>();
    data_amo_req = std::make_shared<Request>();
}

AtomicSimpleCPU::~AtomicSimpleCPU()
{
    if (tickEvent.scheduled()) {
        deschedule(tickEvent);
    }
}

DrainState
AtomicSimpleCPU::drain()
{
    // Deschedule any power gating event (if any)
    deschedulePowerGatingEvent();

    if (switchedOut())
        return DrainState::Drained;

    if (!isCpuDrained()) {
        DPRINTF(Drain, "Requesting drain.\n");
        return DrainState::Draining;
    } else {
        if (tickEvent.scheduled())
            deschedule(tickEvent);

        activeThreads.clear();
        DPRINTF(Drain, "Not executing microcode, no need to drain.\n");
        return DrainState::Drained;
    }
}

void
AtomicSimpleCPU::threadSnoop(PacketPtr pkt, ThreadID sender)
{
    DPRINTF(SimpleCPU, "%s received snoop pkt for addr:%#x %s\n", __func__,
            pkt->getAddr(), pkt->cmdString());

    for (ThreadID tid = 0; tid < numThreads; tid++) {
        if (tid != sender) {
            if (getCpuAddrMonitor(tid)->doMonitor(pkt)) {
                wakeup(tid);
            }

            threadInfo[tid]->thread->getIsaPtr()->handleLockedSnoop(
                pkt, dcachePort.cacheBlockMask);
        }
    }
}

void
AtomicSimpleCPU::drainResume()
{
    assert(!tickEvent.scheduled());
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

            // Tick if any threads active
            if (!tickEvent.scheduled()) {
                schedule(tickEvent, nextCycle());
            }
        } else {
            threadInfo[tid]->execContextStats.notIdleFraction = 0;
        }
    }

    // Reschedule any power gating event (if any)
    schedulePowerGatingEvent();
}

bool
AtomicSimpleCPU::tryCompleteDrain()
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
AtomicSimpleCPU::switchOut()
{
    BaseSimpleCPU::switchOut();

    assert(!tickEvent.scheduled());
    assert(_status == BaseSimpleCPU::Running || _status == Idle);
    assert(isCpuDrained());
}

void
AtomicSimpleCPU::takeOverFrom(BaseCPU *old_cpu)
{
    BaseSimpleCPU::takeOverFrom(old_cpu);

    // The tick event should have been descheduled by drain()
    assert(!tickEvent.scheduled());
}

void
AtomicSimpleCPU::verifyMemoryMode() const
{
    fatal_if(!system->isAtomicMode(),
             "The atomic CPU requires the memory system to be in "
             "'atomic' mode.");
}

void
AtomicSimpleCPU::activateContext(ThreadID thread_num)
{
    DPRINTF(SimpleCPU, "ActivateContext %d\n", thread_num);

    assert(thread_num < numThreads);

    threadInfo[thread_num]->execContextStats.notIdleFraction = 1;
    Cycles delta = ticksToCycles(threadInfo[thread_num]->thread->lastActivate -
                                 threadInfo[thread_num]->thread->lastSuspend);
    baseStats.numCycles += delta;

    if (!tickEvent.scheduled()) {
        // Make sure ticks are still on multiples of cycles
        schedule(tickEvent, clockEdge(Cycles(0)));
    }
    _status = BaseSimpleCPU::Running;
    if (std::find(activeThreads.begin(), activeThreads.end(), thread_num) ==
        activeThreads.end()) {
        activeThreads.push_back(thread_num);
    }

    BaseCPU::activateContext(thread_num);
}

void
AtomicSimpleCPU::suspendContext(ThreadID thread_num)
{
    DPRINTF(SimpleCPU, "SuspendContext %d\n", thread_num);

    assert(thread_num < numThreads);
    activeThreads.remove(thread_num);

    if (_status == Idle)
        return;

    assert(_status == BaseSimpleCPU::Running);

    threadInfo[thread_num]->execContextStats.notIdleFraction = 0;

    if (activeThreads.empty()) {
        _status = Idle;

        if (tickEvent.scheduled()) {
            deschedule(tickEvent);
        }
    }

    BaseCPU::suspendContext(thread_num);
}

Tick
AtomicSimpleCPU::sendPacket(RequestPort &port, const PacketPtr &pkt)
{
    return port.sendAtomic(pkt);
}

Tick
AtomicSimpleCPU::AtomicCPUDPort::recvAtomicSnoop(PacketPtr pkt)
{
    DPRINTF(SimpleCPU, "%s received atomic snoop pkt for addr:%#x %s\n",
            __func__, pkt->getAddr(), pkt->cmdString());

    // X86 ISA: Snooping an invalidation for monitor/mwait
    for (ThreadID tid = 0; tid < cpu->numThreads; tid++) {
        if (cpu->getCpuAddrMonitor(tid)->doMonitor(pkt)) {
            cpu->wakeup(tid);
        }
    }

    // if snoop invalidates, release any associated locks
    // When run without caches, Invalidation packets will not be received
    // hence we must check if the incoming packets are writes and wakeup
    // the processor accordingly
    if (pkt->isInvalidate() || pkt->isWrite()) {
        DPRINTF(SimpleCPU, "received invalidation for addr:%#x\n",
                pkt->getAddr());
        for (auto &t_info : cpu->threadInfo) {
            t_info->thread->getIsaPtr()->handleLockedSnoop(pkt,
                                                           cacheBlockMask);
        }
    }

    return 0;
}

void
AtomicSimpleCPU::AtomicCPUDPort::recvFunctionalSnoop(PacketPtr pkt)
{
    DPRINTF(SimpleCPU, "%s received functional snoop pkt for addr:%#x %s\n",
            __func__, pkt->getAddr(), pkt->cmdString());

    // X86 ISA: Snooping an invalidation for monitor/mwait
    for (ThreadID tid = 0; tid < cpu->numThreads; tid++) {
        if (cpu->getCpuAddrMonitor(tid)->doMonitor(pkt)) {
            cpu->wakeup(tid);
        }
    }

    // if snoop invalidates, release any associated locks
    if (pkt->isInvalidate()) {
        DPRINTF(SimpleCPU, "received invalidation for addr:%#x\n",
                pkt->getAddr());
        for (auto &t_info : cpu->threadInfo) {
            t_info->thread->getIsaPtr()->handleLockedSnoop(pkt,
                                                           cacheBlockMask);
        }
    }
}

bool
AtomicSimpleCPU::genMemFragmentRequest(const RequestPtr &req, Addr frag_addr,
                                       int size, Request::Flags flags,
                                       const std::vector<bool> &byte_enable,
                                       int &frag_size, int &size_left) const
{
    bool predicate = true;
    Addr inst_addr = threadInfo[curThread]->thread->pcState().instAddr();

    frag_size =
        std::min(cacheLineSize() - addrBlockOffset(frag_addr, cacheLineSize()),
                 (Addr)size_left);
    size_left -= frag_size;

    // Set up byte-enable mask for the current fragment
    auto it_start = byte_enable.begin() + (size - (frag_size + size_left));
    auto it_end = byte_enable.begin() + (size - size_left);
    if (isAnyActiveElement(it_start, it_end)) {
        req->setVirt(frag_addr, frag_size, flags, dataRequestorId(),
                     inst_addr);
        req->setByteEnable(std::vector<bool>(it_start, it_end));
    } else {
        predicate = false;
    }

    return predicate;
}

Fault
AtomicSimpleCPU::readMem(Addr addr, uint8_t *data, unsigned size,
                         Request::Flags flags,
                         const std::vector<bool> &byte_enable)
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread *thread = t_info.thread;

    // use the CPU's statically allocated read request and packet objects
    const RequestPtr &req = data_read_req;

    if (traceData)
        traceData->setMem(addr, size, flags);

    dcache_latency = 0;

    req->taskId(taskId());

    Addr frag_addr = addr;
    int frag_size = 0;
    int size_left = size;
    bool predicate;
    Fault fault = NoFault;

    while (1) {
        predicate = genMemFragmentRequest(req, frag_addr, size, flags,
                                          byte_enable, frag_size, size_left);

        // translate to physical address
        if (predicate) {
            fault = thread->mmu->translateAtomic(req, thread->getTC(),
                                                 BaseMMU::Read);
        }

        // Now do the access.
        if (predicate && fault == NoFault &&
            !req->getFlags().isSet(Request::NO_ACCESS)) {
            Packet pkt(req, Packet::makeReadCmd(req));
            pkt.dataStatic(data);

            if (req->isLocalAccess()) {
                dcache_latency += req->localAccessor(thread->getTC(), &pkt);
            } else {
                dcache_latency += sendPacket(dcachePort, &pkt);
            }
            dcache_access = true;

            panic_if(pkt.isError(), "Data fetch (%s) failed: %s",
                     pkt.getAddrRange().to_string(), pkt.print());

            if (req->isLLSC()) {
                thread->getIsaPtr()->handleLockedRead(req);
            }
        }

        // If there's a fault, return it
        if (fault != NoFault)
            return req->isPrefetch() ? NoFault : fault;

        // If we don't need to access further cache lines, stop now.
        if (size_left == 0) {
            if (req->isLockedRMW() && fault == NoFault) {
                assert(!locked);
                locked = true;
            }
            return fault;
        }

        /*
         * Set up for accessing the next cache line.
         */
        frag_addr += frag_size;

        // Move the pointer we're reading into to the correct location.
        data += frag_size;
    }
}

Fault
AtomicSimpleCPU::writeMem(uint8_t *data, unsigned size, Addr addr,
                          Request::Flags flags, uint64_t *res,
                          const std::vector<bool> &byte_enable)
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread *thread = t_info.thread;
    static uint8_t zero_array[64] = {};

    if (data == NULL) {
        assert(size <= 64);
        assert(flags & Request::STORE_NO_DATA);
        // This must be a cache block cleaning request
        data = zero_array;
    }

    // use the CPU's statically allocated write request and packet objects
    const RequestPtr &req = data_write_req;

    if (traceData)
        traceData->setMem(addr, size, flags);

    dcache_latency = 0;

    req->taskId(taskId());

    Addr frag_addr = addr;
    int frag_size = 0;
    int size_left = size;
    [[maybe_unused]] int curr_frag_id = 0;
    bool predicate;
    Fault fault = NoFault;

    while (1) {
        predicate = genMemFragmentRequest(req, frag_addr, size, flags,
                                          byte_enable, frag_size, size_left);

        // translate to physical address
        if (predicate)
            fault = thread->mmu->translateAtomic(req, thread->getTC(),
                                                 BaseMMU::Write);

        // Now do the access.
        if (predicate && fault == NoFault) {
            bool do_access = true; // flag to suppress cache access

            if (req->isLLSC()) {
                assert(curr_frag_id == 0);
                do_access = thread->getIsaPtr()->handleLockedWrite(
                    req, dcachePort.cacheBlockMask);
            } else if (req->isSwap()) {
                assert(curr_frag_id == 0);
                if (req->isCondSwap()) {
                    assert(res);
                    req->setExtraData(*res);
                }
            }

            if (do_access && !req->getFlags().isSet(Request::NO_ACCESS)) {
                Packet pkt(req, Packet::makeWriteCmd(req));
                pkt.dataStatic(data);

                if (req->isLocalAccess()) {
                    dcache_latency +=
                        req->localAccessor(thread->getTC(), &pkt);
                } else {
                    dcache_latency += sendPacket(dcachePort, &pkt);

                    // Notify other threads on this CPU of write
                    threadSnoop(&pkt, curThread);
                }
                dcache_access = true;
                panic_if(pkt.isError(), "Data write (%s) failed: %s",
                         pkt.getAddrRange().to_string(), pkt.print());
                if (req->isSwap()) {
                    assert(res && curr_frag_id == 0);
                    memcpy(res, pkt.getConstPtr<uint8_t>(), size);
                }
            }

            if (res && !req->isSwap()) {
                *res = req->getExtraData();
            }
        }

        // If there's a fault or we don't need to access a second cache line,
        // stop now.
        if (fault != NoFault || size_left == 0) {
            if (req->isLockedRMW() && fault == NoFault) {
                assert(!req->isMasked());
                locked = false;
            }

            // Supress faults from prefetches.
            return req->isPrefetch() ? NoFault : fault;
        }

        /*
         * Set up for accessing the next cache line.
         */
        frag_addr += frag_size;

        // Move the pointer we're reading into to the correct location.
        data += frag_size;

        curr_frag_id++;
    }
}

Fault
AtomicSimpleCPU::amoMem(Addr addr, uint8_t *data, unsigned size,
                        Request::Flags flags, AtomicOpFunctorPtr amo_op)
{
    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread *thread = t_info.thread;

    // use the CPU's statically allocated amo request and packet objects
    const RequestPtr &req = data_amo_req;

    if (traceData)
        traceData->setMem(addr, size, flags);

    // The address of the second part of this access if it needs to be split
    // across a cache line boundary.
    Addr secondAddr = roundDown(addr + size - 1, cacheLineSize());

    // AMO requests that access across a cache line boundary are not
    // allowed since the cache does not guarantee AMO ops to be executed
    // atomically in two cache lines
    // For ISAs such as x86 that requires AMO operations to work on
    // accesses that cross cache-line boundaries, the cache needs to be
    // modified to support locking both cache lines to guarantee the
    // atomicity.
    panic_if(secondAddr > addr,
             "AMO request should not access across a cache line boundary.");

    dcache_latency = 0;

    req->taskId(taskId());
    req->setVirt(addr, size, flags, dataRequestorId(),
                 thread->pcState().instAddr(), std::move(amo_op));

    // translate to physical address
    Fault fault =
        thread->mmu->translateAtomic(req, thread->getTC(), BaseMMU::Write);

    // Now do the access.
    if (fault == NoFault && !req->getFlags().isSet(Request::NO_ACCESS)) {
        // We treat AMO accesses as Write accesses with SwapReq command
        // data will hold the return data of the AMO access
        Packet pkt(req, Packet::makeWriteCmd(req));
        pkt.dataStatic(data);

        if (req->isLocalAccess()) {
            dcache_latency += req->localAccessor(thread->getTC(), &pkt);
        } else {
            dcache_latency += sendPacket(dcachePort, &pkt);
        }

        dcache_access = true;

        panic_if(pkt.isError(), "Atomic access (%s) failed: %s",
                 pkt.getAddrRange().to_string(), pkt.print());
        assert(!req->isLLSC());
    }

    if (fault != NoFault && req->isPrefetch()) {
        return NoFault;
    }

    // If there's a fault and we're not doing prefetch, return it
    return fault;
}

void
AtomicSimpleCPU::tick()
{
    DPRINTF(SimpleCPU, "Tick\n");

    // Change thread if multi-threaded
    swapActiveThread();

    // Set memory request ids to current thread
    if (numThreads > 1) {
        ContextID cid = threadContexts[curThread]->contextId();

        ifetch_req->setContext(cid);
        data_read_req->setContext(cid);
        data_write_req->setContext(cid);
        data_amo_req->setContext(cid);
    }

    SimpleExecContext &t_info = *threadInfo[curThread];
    SimpleThread *thread = t_info.thread;

    Tick latency = 0;

    for (int i = 0; i < width || locked; ++i) {
        baseStats.numCycles++;
        updateCycleCounters(BaseCPU::CPU_STATE_ON);

        if (!curStaticInst || !curStaticInst->isDelayedCommit()) {
            checkForInterrupts();
            checkPcEventQueue();
        }

        // We must have just got suspended by a PC event
        if (_status == Idle) {
            tryCompleteDrain();
            return;
        }

        serviceInstCountEvents();

        Fault fault = NoFault;

        const PCStateBase &pc = thread->pcState();

        bool needToFetch = !isRomMicroPC(pc.microPC()) && !curMacroStaticInst;
        if (needToFetch) {
            ifetch_req->taskId(taskId());
            setupFetchRequest(ifetch_req);
            fault = thread->mmu->translateAtomic(ifetch_req, thread->getTC(),
                                                 BaseMMU::Execute);
        }

        if (fault == NoFault) {
            Tick icache_latency = 0;
            bool icache_access = false;
            dcache_access = false; // assume no dcache access

            if (needToFetch) {
                // This is commented out because the decoder would act like
                // a tiny cache otherwise. It wouldn't be flushed when needed
                // like the I cache. It should be flushed, and when that works
                // this code should be uncommented.
                // Fetch more instruction memory if necessary
                // if (decoder.needMoreBytes())
                //{
                icache_access = true;
                icache_latency = fetchInstMem();
                //}
            }

            preExecute();

            Tick stall_ticks = 0;
            if (curStaticInst) {
                fault = curStaticInst->execute(&t_info, traceData);

                // keep an instruction count
                if (fault == NoFault) {
                    countInst();
                    ppCommit->notify(std::make_pair(thread, curStaticInst));
                } else if (traceData) {
                    traceFault();
                }

                if (fault != NoFault &&
                    std::dynamic_pointer_cast<SyscallRetryFault>(fault)) {
                    // Retry execution of system calls after a delay.
                    // Prevents immediate re-execution since conditions which
                    // caused the retry are unlikely to change every tick.
                    stall_ticks += clockEdge(syscallRetryLatency) - curTick();
                }

                postExecute();
            }

            // @todo remove me after debugging with legion done
            if (curStaticInst && (!curStaticInst->isMicroop() ||
                                  curStaticInst->isFirstMicroop())) {
                instCnt++;
            }

            if (simulate_inst_stalls && icache_access)
                stall_ticks += icache_latency;

            if (simulate_data_stalls && dcache_access)
                stall_ticks += dcache_latency;

            if (stall_ticks) {
                // the atomic cpu does its accounting in ticks, so
                // keep counting in ticks but round to the clock
                // period
                latency += divCeil(stall_ticks, clockPeriod()) * clockPeriod();
            }
        }
        if (fault != NoFault || !t_info.stayAtPC)
            advancePC(fault);
    }

    if (tryCompleteDrain())
        return;

    // instruction takes at least one cycle
    if (latency < clockPeriod())
        latency = clockPeriod();

    if (_status != Idle)
        reschedule(tickEvent, curTick() + latency, true);
}

Tick
AtomicSimpleCPU::fetchInstMem()
{
    auto &decoder = threadInfo[curThread]->thread->decoder;

    Packet pkt = Packet(ifetch_req, MemCmd::ReadReq);

    // ifetch_req is initialized to read the instruction
    // directly into the CPU object's inst field.
    pkt.dataStatic(decoder->moreBytesPtr());

    Tick latency = sendPacket(icachePort, &pkt);
    panic_if(pkt.isError(), "Instruction fetch (%s) failed: %s",
             pkt.getAddrRange().to_string(), pkt.print());

    return latency;
}

void
AtomicSimpleCPU::regProbePoints()
{
    BaseCPU::regProbePoints();

    ppCommit =
        new ProbePointArg<std::pair<SimpleThread *, const StaticInstPtr>>(
            getProbeManager(), "Commit");
}

void
AtomicSimpleCPU::printAddr(Addr a)
{
    dcachePort.printAddr(a);
}

} // namespace gem5
