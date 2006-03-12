/*
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

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <list>
#include <sstream>
#include <string>

#include "arch/utility.hh"
#include "base/cprintf.hh"
#include "base/inifile.hh"
#include "base/loader/symtab.hh"
#include "base/misc.hh"
#include "base/pollevent.hh"
#include "base/range.hh"
#include "base/stats/events.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/cpu_exec_context.hh"
#include "cpu/exec_context.hh"
#include "cpu/exetrace.hh"
#include "cpu/profile.hh"
#include "cpu/sampler/sampler.hh"
#include "cpu/simple/cpu.hh"
#include "cpu/smt.hh"
#include "cpu/static_inst.hh"
#include "kern/kernel_stats.hh"
#include "sim/byteswap.hh"
#include "sim/builder.hh"
#include "sim/debug.hh"
#include "sim/host.hh"
#include "sim/sim_events.hh"
#include "sim/sim_object.hh"
#include "sim/stats.hh"

#if FULL_SYSTEM
#include "base/remote_gdb.hh"
#include "mem/functional/memory_control.hh"
#include "mem/functional/physical.hh"
#include "sim/system.hh"
#include "arch/tlb.hh"
#include "arch/stacktrace.hh"
#include "arch/vtophys.hh"
#else // !FULL_SYSTEM
#include "mem/memory.hh"
#endif // FULL_SYSTEM

using namespace std;
using namespace TheISA;

SimpleCPU::TickEvent::TickEvent(SimpleCPU *c, int w)
    : Event(&mainEventQueue, CPU_Tick_Pri), cpu(c), width(w)
{
}


void
SimpleCPU::init()
{
    BaseCPU::init();
#if FULL_SYSTEM
    for (int i = 0; i < execContexts.size(); ++i) {
        ExecContext *xc = execContexts[i];

        // initialize CPU, including PC
        TheISA::initCPU(xc, xc->readCpuId());
    }
#endif
}

void
SimpleCPU::TickEvent::process()
{
    int count = width;
    do {
        cpu->tick();
    } while (--count > 0 && cpu->status() == Running);
}

const char *
SimpleCPU::TickEvent::description()
{
    return "SimpleCPU tick event";
}


bool
SimpleCPU::CpuPort::recvTiming(Packet &pkt)
{
    cpu->processResponse(pkt);
    return true;
}

Tick
SimpleCPU::CpuPort::recvAtomic(Packet &pkt)
{
    panic("CPU doesn't expect callback!");
    return curTick;
}

void
SimpleCPU::CpuPort::recvFunctional(Packet &pkt)
{
    panic("CPU doesn't expect callback!");
}

void
SimpleCPU::CpuPort::recvStatusChange(Status status)
{
    cpu->recvStatusChange(status);
}

Packet *
SimpleCPU::CpuPort::recvRetry()
{
    return cpu->processRetry();
}

SimpleCPU::SimpleCPU(Params *p)
    : BaseCPU(p), icachePort(this),
      dcachePort(this), tickEvent(this, p->width), cpuXC(NULL)
{
    _status = Idle;

    //Create Memory Ports (conect them up)
    p->mem->addPort("DCACHE");
    dcachePort.setPeer(p->mem->getPort("DCACHE"));
    (p->mem->getPort("DCACHE"))->setPeer(&dcachePort);

    p->mem->addPort("ICACHE");
    icachePort.setPeer(p->mem->getPort("ICACHE"));
    (p->mem->getPort("ICACHE"))->setPeer(&icachePort);

#if FULL_SYSTEM
    cpuXC = new CPUExecContext(this, 0, p->system, p->itb, p->dtb, p->mem);
#else
    cpuXC = new CPUExecContext(this, /* thread_num */ 0, p->process, /* asid */ 0,
                         &dcachePort);
#endif // !FULL_SYSTEM

    xcProxy = cpuXC->getProxy();

#if SIMPLE_CPU_MEM_ATOMIC || SIMPLE_CPU_MEM_IMMEDIATE
    ifetch_req = new CpuRequest;
    ifetch_req->asid = 0;
    ifetch_req->size = sizeof(MachInst);
    ifetch_pkt = new Packet;
    ifetch_pkt->cmd = Read;
    ifetch_pkt->data = (uint8_t *)&inst;
    ifetch_pkt->req = ifetch_req;
    ifetch_pkt->size = sizeof(MachInst);

    data_read_req = new CpuRequest;
    data_read_req->asid = 0;
    data_read_pkt = new Packet;
    data_read_pkt->cmd = Read;
    data_read_pkt->data = new uint8_t[8];
    data_read_pkt->req = data_read_req;

    data_write_req = new CpuRequest;
    data_write_req->asid = 0;
    data_write_pkt = new Packet;
    data_write_pkt->cmd = Write;
    data_write_pkt->req = data_write_req;
#endif

    numInst = 0;
    startNumInst = 0;
    numLoad = 0;
    startNumLoad = 0;
    lastIcacheStall = 0;
    lastDcacheStall = 0;

    execContexts.push_back(xcProxy);
}

SimpleCPU::~SimpleCPU()
{
}

void
SimpleCPU::switchOut(Sampler *s)
{
    sampler = s;
    if (status() == DcacheWaitResponse) {
        DPRINTF(Sampler,"Outstanding dcache access, waiting for completion\n");
        _status = DcacheWaitSwitch;
    }
    else {
        _status = SwitchedOut;

        if (tickEvent.scheduled())
            tickEvent.squash();

        sampler->signalSwitched();
    }
}


void
SimpleCPU::takeOverFrom(BaseCPU *oldCPU)
{
    BaseCPU::takeOverFrom(oldCPU);

    assert(!tickEvent.scheduled());

    // if any of this CPU's ExecContexts are active, mark the CPU as
    // running and schedule its tick event.
    for (int i = 0; i < execContexts.size(); ++i) {
        ExecContext *xc = execContexts[i];
        if (xc->status() == ExecContext::Active && _status != Running) {
            _status = Running;
            tickEvent.schedule(curTick);
        }
    }
}


void
SimpleCPU::activateContext(int thread_num, int delay)
{
    assert(thread_num == 0);
    assert(cpuXC);

    assert(_status == Idle);
    notIdleFraction++;
    scheduleTickEvent(delay);
    _status = Running;
}


void
SimpleCPU::suspendContext(int thread_num)
{
    assert(thread_num == 0);
    assert(cpuXC);

    assert(_status == Running);
    notIdleFraction--;
    unscheduleTickEvent();
    _status = Idle;
}


void
SimpleCPU::deallocateContext(int thread_num)
{
    // for now, these are equivalent
    suspendContext(thread_num);
}


void
SimpleCPU::haltContext(int thread_num)
{
    // for now, these are equivalent
    suspendContext(thread_num);
}


void
SimpleCPU::regStats()
{
    using namespace Stats;

    BaseCPU::regStats();

    numInsts
        .name(name() + ".num_insts")
        .desc("Number of instructions executed")
        ;

    numMemRefs
        .name(name() + ".num_refs")
        .desc("Number of memory references")
        ;

    notIdleFraction
        .name(name() + ".not_idle_fraction")
        .desc("Percentage of non-idle cycles")
        ;

    idleFraction
        .name(name() + ".idle_fraction")
        .desc("Percentage of idle cycles")
        ;

    icacheStallCycles
        .name(name() + ".icache_stall_cycles")
        .desc("ICache total stall cycles")
        .prereq(icacheStallCycles)
        ;

    dcacheStallCycles
        .name(name() + ".dcache_stall_cycles")
        .desc("DCache total stall cycles")
        .prereq(dcacheStallCycles)
        ;

    icacheRetryCycles
        .name(name() + ".icache_retry_cycles")
        .desc("ICache total retry cycles")
        .prereq(icacheRetryCycles)
        ;

    dcacheRetryCycles
        .name(name() + ".dcache_retry_cycles")
        .desc("DCache total retry cycles")
        .prereq(dcacheRetryCycles)
        ;

    idleFraction = constant(1.0) - notIdleFraction;
}

void
SimpleCPU::resetStats()
{
    startNumInst = numInst;
    notIdleFraction = (_status != Idle);
}

void
SimpleCPU::serialize(ostream &os)
{
    BaseCPU::serialize(os);
    SERIALIZE_ENUM(_status);
    SERIALIZE_SCALAR(inst);
    nameOut(os, csprintf("%s.xc", name()));
    cpuXC->serialize(os);
    nameOut(os, csprintf("%s.tickEvent", name()));
    tickEvent.serialize(os);
    nameOut(os, csprintf("%s.cacheCompletionEvent", name()));
}

void
SimpleCPU::unserialize(Checkpoint *cp, const string &section)
{
    BaseCPU::unserialize(cp, section);
    UNSERIALIZE_ENUM(_status);
    UNSERIALIZE_SCALAR(inst);
    cpuXC->unserialize(cp, csprintf("%s.xc", section));
    tickEvent.unserialize(cp, csprintf("%s.tickEvent", section));
}

void
change_thread_state(int thread_number, int activate, int priority)
{
}

Fault
SimpleCPU::copySrcTranslate(Addr src)
{
#if 0
    static bool no_warn = true;
    int blk_size = (dcacheInterface) ? dcacheInterface->getBlockSize() : 64;
    // Only support block sizes of 64 atm.
    assert(blk_size == 64);
    int offset = src & (blk_size - 1);

    // Make sure block doesn't span page
    if (no_warn &&
        (src & PageMask) != ((src + blk_size) & PageMask) &&
        (src >> 40) != 0xfffffc) {
        warn("Copied block source spans pages %x.", src);
        no_warn = false;
    }

    memReq->reset(src & ~(blk_size - 1), blk_size);

    // translate to physical address    Fault fault = cpuXC->translateDataReadReq(req);

    if (fault == NoFault) {
        cpuXC->copySrcAddr = src;
        cpuXC->copySrcPhysAddr = memReq->paddr + offset;
    } else {
        assert(!fault->isAlignmentFault());

        cpuXC->copySrcAddr = 0;
        cpuXC->copySrcPhysAddr = 0;
    }
    return fault;
#else
    return NoFault;
#endif
}

Fault
SimpleCPU::copy(Addr dest)
{
#if 0
    static bool no_warn = true;
    int blk_size = (dcacheInterface) ? dcacheInterface->getBlockSize() : 64;
    // Only support block sizes of 64 atm.
    assert(blk_size == 64);
    uint8_t data[blk_size];
    //assert(cpuXC->copySrcAddr);
    int offset = dest & (blk_size - 1);

    // Make sure block doesn't span page
    if (no_warn &&
        (dest & PageMask) != ((dest + blk_size) & PageMask) &&
        (dest >> 40) != 0xfffffc) {
        no_warn = false;
        warn("Copied block destination spans pages %x. ", dest);
    }

    memReq->reset(dest & ~(blk_size -1), blk_size);
    // translate to physical address
    Fault fault = cpuXC->translateDataWriteReq(req);

    if (fault == NoFault) {
        Addr dest_addr = memReq->paddr + offset;
        // Need to read straight from memory since we have more than 8 bytes.
        memReq->paddr = cpuXC->copySrcPhysAddr;
        cpuXC->mem->read(memReq, data);
        memReq->paddr = dest_addr;
        cpuXC->mem->write(memReq, data);
        if (dcacheInterface) {
            memReq->cmd = Copy;
            memReq->completionEvent = NULL;
            memReq->paddr = cpuXC->copySrcPhysAddr;
            memReq->dest = dest_addr;
            memReq->size = 64;
            memReq->time = curTick;
            memReq->flags &= ~INST_READ;
            dcacheInterface->access(memReq);
        }
    }
    else
        assert(!fault->isAlignmentFault());

    return fault;
#else
    panic("copy not implemented");
    return NoFault;
#endif
}

// precise architected memory state accessor macros
template <class T>
Fault
SimpleCPU::read(Addr addr, T &data, unsigned flags)
{
    if (status() == DcacheWaitResponse || status() == DcacheWaitSwitch) {
//	Fault fault = xc->read(memReq,data);
        // Not sure what to check for no fault...
        if (data_read_pkt->result == Success) {
            memcpy(&data, data_read_pkt->data, sizeof(T));
        }

        if (traceData) {
            traceData->setAddr(addr);
        }

        // @todo: Figure out a way to create a Fault from the packet result.
        return NoFault;
    }

//    memReq->reset(addr, sizeof(T), flags);

#if SIMPLE_CPU_MEM_TIMING
    CpuRequest *data_read_req = new CpuRequest;
#endif

    data_read_req->vaddr = addr;
    data_read_req->size = sizeof(T);
    data_read_req->flags = flags;
    data_read_req->time = curTick;

    // translate to physical address
    Fault fault = cpuXC->translateDataReadReq(data_read_req);

    // Now do the access.
    if (fault == NoFault) {
#if SIMPLE_CPU_MEM_TIMING
        data_read_pkt = new Packet;
        data_read_pkt->cmd = Read;
        data_read_pkt->req = data_read_req;
        data_read_pkt->data = new uint8_t[8];
#endif
        data_read_pkt->addr = data_read_req->paddr;
        data_read_pkt->size = sizeof(T);

        sendDcacheRequest(data_read_pkt);

#if SIMPLE_CPU_MEM_IMMEDIATE
        // Need to find a way to not duplicate code above.

        if (data_read_pkt->result == Success) {
            memcpy(&data, data_read_pkt->data, sizeof(T));
        }

        if (traceData) {
            traceData->setAddr(addr);
        }

        // @todo: Figure out a way to create a Fault from the packet result.
        return NoFault;
#endif
    }
/*
        memReq->cmd = Read;
        memReq->completionEvent = NULL;
        memReq->time = curTick;
        memReq->flags &= ~INST_READ;
        MemAccessResult result = dcacheInterface->access(memReq);

        // Ugly hack to get an event scheduled *only* if the access is
        // a miss.  We really should add first-class support for this
        // at some point.
        if (result != MA_HIT && dcacheInterface->doEvents()) {
            memReq->completionEvent = &cacheCompletionEvent;
            lastDcacheStall = curTick;
            unscheduleTickEvent();
            _status = DcacheMissStall;
        } else {
            // do functional access
            fault = cpuXC->read(memReq, data);

        }
    } else if(fault == NoFault) {
        // do functional access
        fault = cpuXC->read(memReq, data);

    }
*/
    // This will need a new way to tell if it has a dcache attached.
    if (data_read_req->flags & UNCACHEABLE)
        recordEvent("Uncached Read");

    return fault;
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS

template
Fault
SimpleCPU::read(Addr addr, uint64_t &data, unsigned flags);

template
Fault
SimpleCPU::read(Addr addr, uint32_t &data, unsigned flags);

template
Fault
SimpleCPU::read(Addr addr, uint16_t &data, unsigned flags);

template
Fault
SimpleCPU::read(Addr addr, uint8_t &data, unsigned flags);

#endif //DOXYGEN_SHOULD_SKIP_THIS

template<>
Fault
SimpleCPU::read(Addr addr, double &data, unsigned flags)
{
    return read(addr, *(uint64_t*)&data, flags);
}

template<>
Fault
SimpleCPU::read(Addr addr, float &data, unsigned flags)
{
    return read(addr, *(uint32_t*)&data, flags);
}


template<>
Fault
SimpleCPU::read(Addr addr, int32_t &data, unsigned flags)
{
    return read(addr, (uint32_t&)data, flags);
}


template <class T>
Fault
SimpleCPU::write(T data, Addr addr, unsigned flags, uint64_t *res)
{
    data_write_req->vaddr = addr;
    data_write_req->time = curTick;
    data_write_req->size = sizeof(T);
    data_write_req->flags = flags;

    // translate to physical address
    Fault fault = cpuXC->translateDataWriteReq(data_write_req);
    // Now do the access.
    if (fault == NoFault) {
#if SIMPLE_CPU_MEM_TIMING
        data_write_pkt = new Packet;
        data_write_pkt->cmd = Write;
        data_write_pkt->req = data_write_req;
        data_write_pkt->data = new uint8_t[64];
        memcpy(data_write_pkt->data, &data, sizeof(T));
#else
        data_write_pkt->data = (uint8_t *)&data;
#endif
        data_write_pkt->addr = data_write_req->paddr;
        data_write_pkt->size = sizeof(T);

        sendDcacheRequest(data_write_pkt);
    }

/*
    // do functional access
    if (fault == NoFault)
        fault = cpuXC->write(memReq, data);

    if (fault == NoFault && dcacheInterface) {
        memReq->cmd = Write;
        memcpy(memReq->data,(uint8_t *)&data,memReq->size);
        memReq->completionEvent = NULL;
        memReq->time = curTick;
        memReq->flags &= ~INST_READ;
        MemAccessResult result = dcacheInterface->access(memReq);

        // Ugly hack to get an event scheduled *only* if the access is
        // a miss.  We really should add first-class support for this
        // at some point.
        if (result != MA_HIT && dcacheInterface->doEvents()) {
            memReq->completionEvent = &cacheCompletionEvent;
            lastDcacheStall = curTick;
            unscheduleTickEvent();
            _status = DcacheMissStall;
        }
    }
*/
    if (res && (fault == NoFault))
        *res = data_write_pkt->result;

    // This will need a new way to tell if it's hooked up to a cache or not.
    if (data_write_req->flags & UNCACHEABLE)
        recordEvent("Uncached Write");

    // If the write needs to have a fault on the access, consider calling
    // changeStatus() and changing it to "bad addr write" or something.
    return fault;
}


#ifndef DOXYGEN_SHOULD_SKIP_THIS
template
Fault
SimpleCPU::write(uint64_t data, Addr addr, unsigned flags, uint64_t *res);

template
Fault
SimpleCPU::write(uint32_t data, Addr addr, unsigned flags, uint64_t *res);

template
Fault
SimpleCPU::write(uint16_t data, Addr addr, unsigned flags, uint64_t *res);

template
Fault
SimpleCPU::write(uint8_t data, Addr addr, unsigned flags, uint64_t *res);

#endif //DOXYGEN_SHOULD_SKIP_THIS

template<>
Fault
SimpleCPU::write(double data, Addr addr, unsigned flags, uint64_t *res)
{
    return write(*(uint64_t*)&data, addr, flags, res);
}

template<>
Fault
SimpleCPU::write(float data, Addr addr, unsigned flags, uint64_t *res)
{
    return write(*(uint32_t*)&data, addr, flags, res);
}


template<>
Fault
SimpleCPU::write(int32_t data, Addr addr, unsigned flags, uint64_t *res)
{
    return write((uint32_t)data, addr, flags, res);
}


#if FULL_SYSTEM
Addr
SimpleCPU::dbg_vtophys(Addr addr)
{
    return vtophys(xcProxy, addr);
}
#endif // FULL_SYSTEM

void
SimpleCPU::sendIcacheRequest(Packet *pkt)
{
    assert(!tickEvent.scheduled());
#if SIMPLE_CPU_MEM_TIMING
    retry_pkt = pkt;
    bool success = icachePort.sendTiming(*pkt);

    unscheduleTickEvent();

    lastIcacheStall = curTick;

    if (!success) {
        // Need to wait for retry
        _status = IcacheRetry;
    } else {
        // Need to wait for cache to respond
        _status = IcacheWaitResponse;
    }
#elif SIMPLE_CPU_MEM_ATOMIC
    Tick latency = icachePort.sendAtomic(*pkt);

    unscheduleTickEvent();
    scheduleTickEvent(latency);

    // Note that Icache miss cycles will be incorrect.  Unless
    // we check the status of the packet sent (is this valid?),
    // we won't know if the latency is a hit or a miss.
    icacheStallCycles += latency;

    _status = IcacheAccessComplete;
#elif SIMPLE_CPU_MEM_IMMEDIATE
    icachePort.sendAtomic(*pkt);
#else
#error "SimpleCPU has no mem model set"
#endif
}

void
SimpleCPU::sendDcacheRequest(Packet *pkt)
{
    assert(!tickEvent.scheduled());
#if SIMPLE_CPU_MEM_TIMING
    unscheduleTickEvent();

    retry_pkt = pkt;
    bool success = dcachePort.sendTiming(*pkt);

    lastDcacheStall = curTick;

    if (!success) {
        _status = DcacheRetry;
    } else {
        _status = DcacheWaitResponse;
    }
#elif SIMPLE_CPU_MEM_ATOMIC
    unscheduleTickEvent();

    Tick latency = dcachePort.sendAtomic(*pkt);

    scheduleTickEvent(latency);

    // Note that Dcache miss cycles will be incorrect.  Unless
    // we check the status of the packet sent (is this valid?),
    // we won't know if the latency is a hit or a miss.
    dcacheStallCycles += latency;
#elif SIMPLE_CPU_MEM_IMMEDIATE
    dcachePort.sendAtomic(*pkt);
#else
#error "SimpleCPU has no mem model set"
#endif
}

void
SimpleCPU::processResponse(Packet &response)
{
    assert(SIMPLE_CPU_MEM_TIMING);

    // For what things is the CPU the consumer of the packet it sent
    // out?  This may create a memory leak if that's the case and it's
    // expected of the SimpleCPU to delete its own packet.
    Packet *pkt = &response;

    switch (status()) {
      case IcacheWaitResponse:
        icacheStallCycles += curTick - lastIcacheStall;

        _status = IcacheAccessComplete;
        scheduleTickEvent(1);

        // Copy the icache data into the instruction itself.
        memcpy(&inst, pkt->data, sizeof(inst));

        delete pkt;
        break;
      case DcacheWaitResponse:
        if (pkt->cmd == Read) {
            curStaticInst->execute(this,traceData);
            if (traceData)
                traceData->finalize();
        }

        delete pkt;

        dcacheStallCycles += curTick - lastDcacheStall;
        _status = Running;
        scheduleTickEvent(1);
        break;
      case DcacheWaitSwitch:
        if (pkt->cmd == Read) {
            curStaticInst->execute(this,traceData);
            if (traceData)
                traceData->finalize();
        }

        delete pkt;

        _status = SwitchedOut;
        sampler->signalSwitched();
      case SwitchedOut:
        // If this CPU has been switched out due to sampling/warm-up,
        // ignore any further status changes (e.g., due to cache
        // misses outstanding at the time of the switch).
        delete pkt;

        return;
      default:
        panic("SimpleCPU::processCacheCompletion: bad state");
        break;
    }
}

Packet *
SimpleCPU::processRetry()
{
#if SIMPLE_CPU_MEM_TIMING
    switch(status()) {
      case IcacheRetry:
        icacheRetryCycles += curTick - lastIcacheStall;
        return retry_pkt;
        break;
      case DcacheRetry:
        dcacheRetryCycles += curTick - lastDcacheStall;
        return retry_pkt;
        break;
      default:
        panic("SimpleCPU::processRetry: bad state");
        break;
    }
#else
    panic("shouldn't be here");
#endif
}

#if FULL_SYSTEM
void
SimpleCPU::post_interrupt(int int_num, int index)
{
    BaseCPU::post_interrupt(int_num, index);

    if (cpuXC->status() == ExecContext::Suspended) {
                DPRINTF(IPI,"Suspended Processor awoke\n");
        cpuXC->activate();
    }
}
#endif // FULL_SYSTEM

/* start simulation, program loaded, processor precise state initialized */
void
SimpleCPU::tick()
{
    numCycles++;

    traceData = NULL;

    Fault fault = NoFault;

#if FULL_SYSTEM
    if (checkInterrupts && check_interrupts() && !cpuXC->inPalMode() &&
        status() != IcacheMissComplete) {
        int ipl = 0;
        int summary = 0;
        checkInterrupts = false;

        if (cpuXC->readMiscReg(IPR_SIRR)) {
            for (int i = INTLEVEL_SOFTWARE_MIN;
                 i < INTLEVEL_SOFTWARE_MAX; i++) {
                if (cpuXC->readMiscReg(IPR_SIRR) & (ULL(1) << i)) {
                    // See table 4-19 of 21164 hardware reference
                    ipl = (i - INTLEVEL_SOFTWARE_MIN) + 1;
                    summary |= (ULL(1) << i);
                }
            }
        }

        uint64_t interrupts = cpuXC->cpu->intr_status();
        for (int i = INTLEVEL_EXTERNAL_MIN;
            i < INTLEVEL_EXTERNAL_MAX; i++) {
            if (interrupts & (ULL(1) << i)) {
                // See table 4-19 of 21164 hardware reference
                ipl = i;
                summary |= (ULL(1) << i);
            }
        }

        if (cpuXC->readMiscReg(IPR_ASTRR))
            panic("asynchronous traps not implemented\n");

        if (ipl && ipl > cpuXC->readMiscReg(IPR_IPLR)) {
            cpuXC->setMiscReg(IPR_ISR, summary);
            cpuXC->setMiscReg(IPR_INTID, ipl);

            Fault(new InterruptFault)->invoke(xcProxy);

            DPRINTF(Flow, "Interrupt! IPLR=%d ipl=%d summary=%x\n",
                    cpuXC->readMiscReg(IPR_IPLR), ipl, summary);
        }
    }
#endif

    // maintain $r0 semantics
    cpuXC->setIntReg(ZeroReg, 0);
#if THE_ISA == ALPHA_ISA
    cpuXC->setFloatRegDouble(ZeroReg, 0.0);
#endif // ALPHA_ISA

    if (status() == IcacheAccessComplete) {
        // We've already fetched an instruction and were stalled on an
        // I-cache miss.  No need to fetch it again.

        // Set status to running; tick event will get rescheduled if
        // necessary at end of tick() function.
        _status = Running;
    } else {
        // Try to fetch an instruction

        // set up memory request for instruction fetch
#if FULL_SYSTEM
#define IFETCH_FLAGS(pc)	((pc) & 1) ? PHYSICAL : 0
#else
#define IFETCH_FLAGS(pc)	0
#endif

#if SIMPLE_CPU_MEM_TIMING
        CpuRequest *ifetch_req = new CpuRequest();
        ifetch_req->size = sizeof(MachInst);
#endif

        ifetch_req->vaddr = cpuXC->readPC() & ~3;
        ifetch_req->time = curTick;

/*	memReq->reset(xc->regs.pc & ~3, sizeof(uint32_t),
                     IFETCH_FLAGS(xc->regs.pc));
*/

        fault = cpuXC->translateInstReq(ifetch_req);

        if (fault == NoFault) {
#if SIMPLE_CPU_MEM_TIMING
            Packet *ifetch_pkt = new Packet;
            ifetch_pkt->cmd = Read;
            ifetch_pkt->data = (uint8_t *)&inst;
            ifetch_pkt->req = ifetch_req;
            ifetch_pkt->size = sizeof(MachInst);
#endif
            ifetch_pkt->addr = ifetch_req->paddr;

            sendIcacheRequest(ifetch_pkt);
#if SIMPLE_CPU_MEM_TIMING || SIMPLE_CPU_MEM_ATOMIC
            return;
#endif
/*
        if (icacheInterface && fault == NoFault) {
            memReq->completionEvent = NULL;

            memReq->time = curTick;
            memReq->flags |= INST_READ;
            MemAccessResult result = icacheInterface->access(memReq);

            // Ugly hack to get an event scheduled *only* if the access is
            // a miss.  We really should add first-class support for this
            // at some point.
                if (result != MA_HIT && icacheInterface->doEvents()) {
                memReq->completionEvent = &cacheCompletionEvent;
                lastIcacheStall = curTick;
                unscheduleTickEvent();
                _status = IcacheMissStall;
                return;
            }
        }
*/
        }
    }

    // If we've got a valid instruction (i.e., no fault on instruction
    // fetch), then execute it.
    if (fault == NoFault) {

        // keep an instruction count
        numInst++;
        numInsts++;

        // check for instruction-count-based events
        comInstEventQueue[0]->serviceEvents(numInst);

        // decode the instruction
        inst = gtoh(inst);
        curStaticInst = StaticInst::decode(makeExtMI(inst, cpuXC->readPC()));

        traceData = Trace::getInstRecord(curTick, xcProxy, this, curStaticInst,
                                         cpuXC->readPC());

#if FULL_SYSTEM
        cpuXC->setInst(inst);
#endif // FULL_SYSTEM

        cpuXC->func_exe_inst++;

        fault = curStaticInst->execute(this, traceData);

#if FULL_SYSTEM
        if (system->kernelBinning->fnbin) {
            assert(kernelStats);
            system->kernelBinning->execute(xcProxy, inst);
        }

        if (cpuXC->profile) {
            bool usermode =
                (cpuXC->readMiscReg(AlphaISA::IPR_DTB_CM) & 0x18) != 0;
            cpuXC->profilePC = usermode ? 1 : cpuXC->readPC();
            ProfileNode *node = cpuXC->profile->consume(xcProxy, inst);
            if (node)
                cpuXC->profileNode = node;
        }
#endif

        if (curStaticInst->isMemRef()) {
            numMemRefs++;
        }

        if (curStaticInst->isLoad()) {
            ++numLoad;
            comLoadEventQueue[0]->serviceEvents(numLoad);
        }

        // If we have a dcache miss, then we can't finialize the instruction
        // trace yet because we want to populate it with the data later
        if (traceData && (status() != DcacheWaitResponse)) {
            traceData->finalize();
        }

        traceFunctions(cpuXC->readPC());

    }	// if (fault == NoFault)

    if (fault != NoFault) {
#if FULL_SYSTEM
        fault->invoke(xcProxy);
#else // !FULL_SYSTEM
        fatal("fault (%d) detected @ PC %08p", fault, cpuXC->readPC());
#endif // FULL_SYSTEM
    }
    else {
#if THE_ISA != MIPS_ISA
        // go to the next instruction
        cpuXC->setPC(cpuXC->readNextPC());
        cpuXC->setNextPC(cpuXC->readNextPC() + sizeof(MachInst));
#else
        // go to the next instruction
        cpuXC->setPC(cpuXC->readNextPC());
        cpuXC->setNextPC(cpuXC->readNextNPC());
        cpuXC->setNextNPC(cpuXC->readNextNPC() + sizeof(MachInst));
#endif

    }

#if FULL_SYSTEM
    Addr oldpc;
    do {
        oldpc = cpuXC->readPC();
        system->pcEventQueue.service(xcProxy);
    } while (oldpc != cpuXC->readPC());
#endif

    assert(status() == Running ||
           status() == Idle ||
           status() == DcacheWaitResponse);

    if (status() == Running && !tickEvent.scheduled())
        tickEvent.schedule(curTick + cycles(1));
}

////////////////////////////////////////////////////////////////////////
//
//  SimpleCPU Simulation Object
//
BEGIN_DECLARE_SIM_OBJECT_PARAMS(SimpleCPU)

    Param<Counter> max_insts_any_thread;
    Param<Counter> max_insts_all_threads;
    Param<Counter> max_loads_any_thread;
    Param<Counter> max_loads_all_threads;

#if FULL_SYSTEM
    SimObjectParam<AlphaITB *> itb;
    SimObjectParam<AlphaDTB *> dtb;
    SimObjectParam<System *> system;
    Param<int> cpu_id;
    Param<Tick> profile;
#else
    SimObjectParam<Memory *> mem;
    SimObjectParam<Process *> workload;
#endif // FULL_SYSTEM

    Param<int> clock;

    Param<bool> defer_registration;
    Param<int> width;
    Param<bool> function_trace;
    Param<Tick> function_trace_start;

END_DECLARE_SIM_OBJECT_PARAMS(SimpleCPU)

BEGIN_INIT_SIM_OBJECT_PARAMS(SimpleCPU)

    INIT_PARAM(max_insts_any_thread,
               "terminate when any thread reaches this inst count"),
    INIT_PARAM(max_insts_all_threads,
               "terminate when all threads have reached this inst count"),
    INIT_PARAM(max_loads_any_thread,
               "terminate when any thread reaches this load count"),
    INIT_PARAM(max_loads_all_threads,
               "terminate when all threads have reached this load count"),

#if FULL_SYSTEM
    INIT_PARAM(itb, "Instruction TLB"),
    INIT_PARAM(dtb, "Data TLB"),
    INIT_PARAM(system, "system object"),
    INIT_PARAM(cpu_id, "processor ID"),
    INIT_PARAM(profile, ""),
#else
    INIT_PARAM(mem, "memory"),
    INIT_PARAM(workload, "processes to run"),
#endif // FULL_SYSTEM

    INIT_PARAM(clock, "clock speed"),
    INIT_PARAM(defer_registration, "defer system registration (for sampling)"),
    INIT_PARAM(width, "cpu width"),
    INIT_PARAM(function_trace, "Enable function trace"),
    INIT_PARAM(function_trace_start, "Cycle to start function trace")

END_INIT_SIM_OBJECT_PARAMS(SimpleCPU)


CREATE_SIM_OBJECT(SimpleCPU)
{
    SimpleCPU::Params *params = new SimpleCPU::Params();
    params->name = getInstanceName();
    params->numberOfThreads = 1;
    params->max_insts_any_thread = max_insts_any_thread;
    params->max_insts_all_threads = max_insts_all_threads;
    params->max_loads_any_thread = max_loads_any_thread;
    params->max_loads_all_threads = max_loads_all_threads;
    params->deferRegistration = defer_registration;
    params->clock = clock;
    params->functionTrace = function_trace;
    params->functionTraceStart = function_trace_start;
    params->width = width;

#if FULL_SYSTEM
    params->itb = itb;
    params->dtb = dtb;
    params->system = system;
    params->cpu_id = cpu_id;
    params->profile = profile;
#else
    params->mem = mem;
    params->process = workload;
#endif

    SimpleCPU *cpu = new SimpleCPU(params);
    return cpu;
}

REGISTER_SIM_OBJECT("SimpleCPU", SimpleCPU)

