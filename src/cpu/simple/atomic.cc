/*
 * Copyright 2014 Google, Inc.
 * Copyright (c) 2012-2013,2015,2017 ARM Limited
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
 *
 * Authors: Steve Reinhardt
 */

#include "cpu/simple/atomic.hh"

#include "arch/locked_mem.hh"
#include "arch/mmapped_ipr.hh"
#include "arch/utility.hh"
#include "base/output.hh"
#include "config/the_isa.hh"
#include "cpu/exetrace.hh"
#include "debug/Drain.hh"
#include "debug/ExecFaulting.hh"
#include "debug/SimpleCPU.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "mem/physical.hh"
#include "params/AtomicSimpleCPU.hh"
#include "sim/faults.hh"
#include "sim/full_system.hh"
#include "sim/system.hh"

using namespace std;
using namespace TheISA;

void
AtomicSimpleCPU::init()
{
    BaseSimpleCPU::init();

    int cid = threadContexts[0]->contextId();
    ifetch_req.setContext(cid);
    data_read_req.setContext(cid);
    data_write_req.setContext(cid);
}

AtomicSimpleCPU::AtomicSimpleCPU(AtomicSimpleCPUParams *p)
    : BaseSimpleCPU(p),
      tickEvent([this]{ tick(); }, "AtomicSimpleCPU tick",
                false, Event::CPU_Tick_Pri),
      width(p->width), locked(false),
      simulate_data_stalls(p->simulate_data_stalls),
      simulate_inst_stalls(p->simulate_inst_stalls),
      icachePort(name() + ".icache_port", this),
      dcachePort(name() + ".dcache_port", this),
      fastmem(p->fastmem), dcache_access(false), dcache_latency(0),
      ppCommit(nullptr)
{
    _status = Idle;
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

    if (!isDrained()) {
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
    DPRINTF(SimpleCPU, "received snoop pkt for addr:%#x %s\n", pkt->getAddr(),
            pkt->cmdString());

    for (ThreadID tid = 0; tid < numThreads; tid++) {
        if (tid != sender) {
            if (getCpuAddrMonitor(tid)->doMonitor(pkt)) {
                wakeup(tid);
            }

            TheISA::handleLockedSnoop(threadInfo[tid]->thread,
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
            threadInfo[tid]->notIdleFraction = 1;
            activeThreads.push_back(tid);
            _status = BaseSimpleCPU::Running;

            // Tick if any threads active
            if (!tickEvent.scheduled()) {
                schedule(tickEvent, nextCycle());
            }
        } else {
            threadInfo[tid]->notIdleFraction = 0;
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
    if (!isDrained())
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
    assert(isDrained());
}


void
AtomicSimpleCPU::takeOverFrom(BaseCPU *oldCPU)
{
    BaseSimpleCPU::takeOverFrom(oldCPU);

    // The tick event should have been descheduled by drain()
    assert(!tickEvent.scheduled());
}

void
AtomicSimpleCPU::verifyMemoryMode() const
{
    if (!system->isAtomicMode()) {
        fatal("The atomic CPU requires the memory system to be in "
              "'atomic' mode.\n");
    }
}

void
AtomicSimpleCPU::activateContext(ThreadID thread_num)
{
    DPRINTF(SimpleCPU, "ActivateContext %d\n", thread_num);

    assert(thread_num < numThreads);

    threadInfo[thread_num]->notIdleFraction = 1;
    Cycles delta = ticksToCycles(threadInfo[thread_num]->thread->lastActivate -
                                 threadInfo[thread_num]->thread->lastSuspend);
    numCycles += delta;

    if (!tickEvent.scheduled()) {
        //Make sure ticks are still on multiples of cycles
        schedule(tickEvent, clockEdge(Cycles(0)));
    }
    _status = BaseSimpleCPU::Running;
    if (std::find(activeThreads.begin(), activeThreads.end(), thread_num)
        == activeThreads.end()) {
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

    threadInfo[thread_num]->notIdleFraction = 0;

    if (activeThreads.empty()) {
        _status = Idle;

        if (tickEvent.scheduled()) {
            deschedule(tickEvent);
        }
    }

    BaseCPU::suspendContext(thread_num);
}


Tick
AtomicSimpleCPU::AtomicCPUDPort::recvAtomicSnoop(PacketPtr pkt)
{
    DPRINTF(SimpleCPU, "received snoop pkt for addr:%#x %s\n", pkt->getAddr(),
            pkt->cmdString());

    // X86 ISA: Snooping an invalidation for monitor/mwait
    AtomicSimpleCPU *cpu = (AtomicSimpleCPU *)(&owner);

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
            TheISA::handleLockedSnoop(t_info->thread, pkt, cacheBlockMask);
        }
    }

    return 0;
}

void
AtomicSimpleCPU::AtomicCPUDPort::recvFunctionalSnoop(PacketPtr pkt)
{
    DPRINTF(SimpleCPU, "received snoop pkt for addr:%#x %s\n", pkt->getAddr(),
            pkt->cmdString());

    // X86 ISA: Snooping an invalidation for monitor/mwait
    AtomicSimpleCPU *cpu = (AtomicSimpleCPU *)(&owner);
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
            TheISA::handleLockedSnoop(t_info->thread, pkt, cacheBlockMask);
        }
    }
}

Fault
AtomicSimpleCPU::readMem(Addr addr, uint8_t * data, unsigned size,
                         Request::Flags flags)
{
    SimpleExecContext& t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    // use the CPU's statically allocated read request and packet objects
    Request *req = &data_read_req;

    if (traceData)
        traceData->setMem(addr, size, flags);

    //The size of the data we're trying to read.
    int fullSize = size;

    //The address of the second part of this access if it needs to be split
    //across a cache line boundary.
    Addr secondAddr = roundDown(addr + size - 1, cacheLineSize());

    if (secondAddr > addr)
        size = secondAddr - addr;

    dcache_latency = 0;

    req->taskId(taskId());
    while (1) {
        req->setVirt(0, addr, size, flags, dataMasterId(), thread->pcState().instAddr());

        // translate to physical address
        Fault fault = thread->dtb->translateAtomic(req, thread->getTC(),
                                                          BaseTLB::Read);

        // Now do the access.
        if (fault == NoFault && !req->getFlags().isSet(Request::NO_ACCESS)) {
            Packet pkt(req, Packet::makeReadCmd(req));
            pkt.dataStatic(data);

            if (req->isMmappedIpr())
                dcache_latency += TheISA::handleIprRead(thread->getTC(), &pkt);
            else {
                if (fastmem && system->isMemAddr(pkt.getAddr()))
                    system->getPhysMem().access(&pkt);
                else
                    dcache_latency += dcachePort.sendAtomic(&pkt);
            }
            dcache_access = true;

            assert(!pkt.isError());

            if (req->isLLSC()) {
                TheISA::handleLockedRead(thread, req);
            }
        }

        //If there's a fault, return it
        if (fault != NoFault) {
            if (req->isPrefetch()) {
                return NoFault;
            } else {
                return fault;
            }
        }

        //If we don't need to access a second cache line, stop now.
        if (secondAddr <= addr)
        {
            if (req->isLockedRMW() && fault == NoFault) {
                assert(!locked);
                locked = true;
            }

            return fault;
        }

        /*
         * Set up for accessing the second cache line.
         */

        //Move the pointer we're reading into to the correct location.
        data += size;
        //Adjust the size to get the remaining bytes.
        size = addr + fullSize - secondAddr;
        //And access the right address.
        addr = secondAddr;
    }
}

Fault
AtomicSimpleCPU::initiateMemRead(Addr addr, unsigned size,
                                 Request::Flags flags)
{
    panic("initiateMemRead() is for timing accesses, and should "
          "never be called on AtomicSimpleCPU.\n");
}

Fault
AtomicSimpleCPU::writeMem(uint8_t *data, unsigned size, Addr addr,
                          Request::Flags flags, uint64_t *res)
{
    SimpleExecContext& t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;
    static uint8_t zero_array[64] = {};

    if (data == NULL) {
        assert(size <= 64);
        assert(flags & Request::STORE_NO_DATA);
        // This must be a cache block cleaning request
        data = zero_array;
    }

    // use the CPU's statically allocated write request and packet objects
    Request *req = &data_write_req;

    if (traceData)
        traceData->setMem(addr, size, flags);

    //The size of the data we're trying to read.
    int fullSize = size;

    //The address of the second part of this access if it needs to be split
    //across a cache line boundary.
    Addr secondAddr = roundDown(addr + size - 1, cacheLineSize());

    if (secondAddr > addr)
        size = secondAddr - addr;

    dcache_latency = 0;

    req->taskId(taskId());
    while (1) {
        req->setVirt(0, addr, size, flags, dataMasterId(), thread->pcState().instAddr());

        // translate to physical address
        Fault fault = thread->dtb->translateAtomic(req, thread->getTC(), BaseTLB::Write);

        // Now do the access.
        if (fault == NoFault) {
            bool do_access = true;  // flag to suppress cache access

            if (req->isLLSC()) {
                do_access = TheISA::handleLockedWrite(thread, req, dcachePort.cacheBlockMask);
            } else if (req->isSwap()) {
                if (req->isCondSwap()) {
                    assert(res);
                    req->setExtraData(*res);
                }
            }

            if (do_access && !req->getFlags().isSet(Request::NO_ACCESS)) {
                Packet pkt(req, Packet::makeWriteCmd(req));
                pkt.dataStatic(data);

                if (req->isMmappedIpr()) {
                    dcache_latency +=
                        TheISA::handleIprWrite(thread->getTC(), &pkt);
                } else {
                    if (fastmem && system->isMemAddr(pkt.getAddr()))
                        system->getPhysMem().access(&pkt);
                    else
                        dcache_latency += dcachePort.sendAtomic(&pkt);

                    // Notify other threads on this CPU of write
                    threadSnoop(&pkt, curThread);
                }
                dcache_access = true;
                assert(!pkt.isError());

                if (req->isSwap()) {
                    assert(res);
                    memcpy(res, pkt.getConstPtr<uint8_t>(), fullSize);
                }
            }

            if (res && !req->isSwap()) {
                *res = req->getExtraData();
            }
        }

        //If there's a fault or we don't need to access a second cache line,
        //stop now.
        if (fault != NoFault || secondAddr <= addr)
        {
            if (req->isLockedRMW() && fault == NoFault) {
                assert(locked);
                locked = false;
            }


            if (fault != NoFault && req->isPrefetch()) {
                return NoFault;
            } else {
                return fault;
            }
        }

        /*
         * Set up for accessing the second cache line.
         */

        //Move the pointer we're reading into to the correct location.
        data += size;
        //Adjust the size to get the remaining bytes.
        size = addr + fullSize - secondAddr;
        //And access the right address.
        addr = secondAddr;
    }
}


void
AtomicSimpleCPU::tick()
{
    DPRINTF(SimpleCPU, "Tick\n");

    // Change thread if multi-threaded
    swapActiveThread();

    // Set memroy request ids to current thread
    if (numThreads > 1) {
        ContextID cid = threadContexts[curThread]->contextId();

        ifetch_req.setContext(cid);
        data_read_req.setContext(cid);
        data_write_req.setContext(cid);
    }

    SimpleExecContext& t_info = *threadInfo[curThread];
    SimpleThread* thread = t_info.thread;

    Tick latency = 0;

    for (int i = 0; i < width || locked; ++i) {
        numCycles++;
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

        Fault fault = NoFault;

        TheISA::PCState pcState = thread->pcState();

        bool needToFetch = !isRomMicroPC(pcState.microPC()) &&
                           !curMacroStaticInst;
        if (needToFetch) {
            ifetch_req.taskId(taskId());
            setupFetchRequest(&ifetch_req);
            fault = thread->itb->translateAtomic(&ifetch_req, thread->getTC(),
                                                 BaseTLB::Execute);
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
                //Fetch more instruction memory if necessary
                //if (decoder.needMoreBytes())
                //{
                    icache_access = true;
                    Packet ifetch_pkt = Packet(&ifetch_req, MemCmd::ReadReq);
                    ifetch_pkt.dataStatic(&inst);

                    if (fastmem && system->isMemAddr(ifetch_pkt.getAddr()))
                        system->getPhysMem().access(&ifetch_pkt);
                    else
                        icache_latency = icachePort.sendAtomic(&ifetch_pkt);

                    assert(!ifetch_pkt.isError());

                    // ifetch_req is initialized to read the instruction directly
                    // into the CPU object's inst field.
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
                }
                else if (traceData && !DTRACE(ExecFaulting)) {
                    delete traceData;
                    traceData = NULL;
                }

                if (dynamic_pointer_cast<SyscallRetryFault>(fault)) {
                    // Retry execution of system calls after a delay.
                    // Prevents immediate re-execution since conditions which
                    // caused the retry are unlikely to change every tick.
                    stall_ticks += clockEdge(syscallRetryLatency) - curTick();
                }

                postExecute();
            }

            // @todo remove me after debugging with legion done
            if (curStaticInst && (!curStaticInst->isMicroop() ||
                        curStaticInst->isFirstMicroop()))
                instCnt++;

            if (simulate_inst_stalls && icache_access)
                stall_ticks += icache_latency;

            if (simulate_data_stalls && dcache_access)
                stall_ticks += dcache_latency;

            if (stall_ticks) {
                // the atomic cpu does its accounting in ticks, so
                // keep counting in ticks but round to the clock
                // period
                latency += divCeil(stall_ticks, clockPeriod()) *
                    clockPeriod();
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

void
AtomicSimpleCPU::regProbePoints()
{
    BaseCPU::regProbePoints();

    ppCommit = new ProbePointArg<pair<SimpleThread*, const StaticInstPtr>>
                                (getProbeManager(), "Commit");
}

void
AtomicSimpleCPU::printAddr(Addr a)
{
    dcachePort.printAddr(a);
}

////////////////////////////////////////////////////////////////////////
//
//  AtomicSimpleCPU Simulation Object
//
AtomicSimpleCPU *
AtomicSimpleCPUParams::create()
{
    return new AtomicSimpleCPU(this);
}
