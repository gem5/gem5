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
 *
 * Authors: Steve Reinhardt
 */

#include "arch/locked_mem.hh"
#include "arch/utility.hh"
#include "cpu/exetrace.hh"
#include "cpu/simple/atomic.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/builder.hh"
#include "sim/system.hh"

using namespace std;
using namespace TheISA;

AtomicSimpleCPU::TickEvent::TickEvent(AtomicSimpleCPU *c)
    : Event(&mainEventQueue, CPU_Tick_Pri), cpu(c)
{
}


void
AtomicSimpleCPU::TickEvent::process()
{
    cpu->tick();
}

const char *
AtomicSimpleCPU::TickEvent::description()
{
    return "AtomicSimpleCPU tick event";
}

Port *
AtomicSimpleCPU::getPort(const std::string &if_name, int idx)
{
    if (if_name == "dcache_port")
        return &dcachePort;
    else if (if_name == "icache_port")
        return &icachePort;
    else
        panic("No Such Port\n");
}

void
AtomicSimpleCPU::init()
{
    BaseCPU::init();
#if FULL_SYSTEM
    for (int i = 0; i < threadContexts.size(); ++i) {
        ThreadContext *tc = threadContexts[i];

        // initialize CPU, including PC
        TheISA::initCPU(tc, tc->readCpuId());
    }
#endif
}

bool
AtomicSimpleCPU::CpuPort::recvTiming(PacketPtr pkt)
{
    panic("AtomicSimpleCPU doesn't expect recvTiming callback!");
    return true;
}

Tick
AtomicSimpleCPU::CpuPort::recvAtomic(PacketPtr pkt)
{
    //Snooping a coherence request, just return
    return curTick;
}

void
AtomicSimpleCPU::CpuPort::recvFunctional(PacketPtr pkt)
{
    //No internal storage to update, just return
    return;
}

void
AtomicSimpleCPU::CpuPort::recvStatusChange(Status status)
{
    if (status == RangeChange)
        return;

    panic("AtomicSimpleCPU doesn't expect recvStatusChange callback!");
}

void
AtomicSimpleCPU::CpuPort::recvRetry()
{
    panic("AtomicSimpleCPU doesn't expect recvRetry callback!");
}


AtomicSimpleCPU::AtomicSimpleCPU(Params *p)
    : BaseSimpleCPU(p), tickEvent(this),
      width(p->width), simulate_stalls(p->simulate_stalls),
      icachePort(name() + "-iport", this), dcachePort(name() + "-iport", this)
{
    _status = Idle;

    ifetch_req = new Request();
    ifetch_req->setThreadContext(p->cpu_id, 0); // Add thread ID if we add MT
    ifetch_pkt = new Packet(ifetch_req, Packet::ReadReq, Packet::Broadcast);
    ifetch_pkt->dataStatic(&inst);

    data_read_req = new Request();
    data_read_req->setThreadContext(p->cpu_id, 0); // Add thread ID here too
    data_read_pkt = new Packet(data_read_req, Packet::ReadReq,
                               Packet::Broadcast);
    data_read_pkt->dataStatic(&dataReg);

    data_write_req = new Request();
    data_write_req->setThreadContext(p->cpu_id, 0); // Add thread ID here too
    data_write_pkt = new Packet(data_write_req, Packet::WriteReq,
                                Packet::Broadcast);
}


AtomicSimpleCPU::~AtomicSimpleCPU()
{
}

void
AtomicSimpleCPU::serialize(ostream &os)
{
    SimObject::State so_state = SimObject::getState();
    SERIALIZE_ENUM(so_state);
    Status _status = status();
    SERIALIZE_ENUM(_status);
    BaseSimpleCPU::serialize(os);
    nameOut(os, csprintf("%s.tickEvent", name()));
    tickEvent.serialize(os);
}

void
AtomicSimpleCPU::unserialize(Checkpoint *cp, const string &section)
{
    SimObject::State so_state;
    UNSERIALIZE_ENUM(so_state);
    UNSERIALIZE_ENUM(_status);
    BaseSimpleCPU::unserialize(cp, section);
    tickEvent.unserialize(cp, csprintf("%s.tickEvent", section));
}

void
AtomicSimpleCPU::resume()
{
    if (_status != SwitchedOut && _status != Idle) {
        assert(system->getMemoryMode() == System::Atomic);

        changeState(SimObject::Running);
        if (thread->status() == ThreadContext::Active) {
            if (!tickEvent.scheduled()) {
                tickEvent.schedule(nextCycle());
            }
        }
    }
}

void
AtomicSimpleCPU::switchOut()
{
    assert(status() == Running || status() == Idle);
    _status = SwitchedOut;

    tickEvent.squash();
}


void
AtomicSimpleCPU::takeOverFrom(BaseCPU *oldCPU)
{
    BaseCPU::takeOverFrom(oldCPU);

    assert(!tickEvent.scheduled());

    // if any of this CPU's ThreadContexts are active, mark the CPU as
    // running and schedule its tick event.
    for (int i = 0; i < threadContexts.size(); ++i) {
        ThreadContext *tc = threadContexts[i];
        if (tc->status() == ThreadContext::Active && _status != Running) {
            _status = Running;
            tickEvent.schedule(nextCycle());
            break;
        }
    }
}


void
AtomicSimpleCPU::activateContext(int thread_num, int delay)
{
    assert(thread_num == 0);
    assert(thread);

    assert(_status == Idle);
    assert(!tickEvent.scheduled());

    notIdleFraction++;
    //Make sure ticks are still on multiples of cycles
    tickEvent.schedule(nextCycle(curTick + cycles(delay)));
    _status = Running;
}


void
AtomicSimpleCPU::suspendContext(int thread_num)
{
    assert(thread_num == 0);
    assert(thread);

    assert(_status == Running);

    // tick event may not be scheduled if this gets called from inside
    // an instruction's execution, e.g. "quiesce"
    if (tickEvent.scheduled())
        tickEvent.deschedule();

    notIdleFraction--;
    _status = Idle;
}


template <class T>
Fault
AtomicSimpleCPU::read(Addr addr, T &data, unsigned flags)
{
    // use the CPU's statically allocated read request and packet objects
    Request *req = data_read_req;
    PacketPtr pkt = data_read_pkt;

    req->setVirt(0, addr, sizeof(T), flags, thread->readPC());

    if (traceData) {
        traceData->setAddr(addr);
    }

    // translate to physical address
    Fault fault = thread->translateDataReadReq(req);

    // Now do the access.
    if (fault == NoFault) {
        pkt->reinitFromRequest();

        dcache_latency = dcachePort.sendAtomic(pkt);
        dcache_access = true;

        assert(pkt->result == Packet::Success);
        data = pkt->get<T>();

        if (req->isLocked()) {
            TheISA::handleLockedRead(thread, req);
        }
    }

    // This will need a new way to tell if it has a dcache attached.
    if (req->isUncacheable())
        recordEvent("Uncached Read");

    return fault;
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS

template
Fault
AtomicSimpleCPU::read(Addr addr, uint64_t &data, unsigned flags);

template
Fault
AtomicSimpleCPU::read(Addr addr, uint32_t &data, unsigned flags);

template
Fault
AtomicSimpleCPU::read(Addr addr, uint16_t &data, unsigned flags);

template
Fault
AtomicSimpleCPU::read(Addr addr, uint8_t &data, unsigned flags);

#endif //DOXYGEN_SHOULD_SKIP_THIS

template<>
Fault
AtomicSimpleCPU::read(Addr addr, double &data, unsigned flags)
{
    return read(addr, *(uint64_t*)&data, flags);
}

template<>
Fault
AtomicSimpleCPU::read(Addr addr, float &data, unsigned flags)
{
    return read(addr, *(uint32_t*)&data, flags);
}


template<>
Fault
AtomicSimpleCPU::read(Addr addr, int32_t &data, unsigned flags)
{
    return read(addr, (uint32_t&)data, flags);
}


template <class T>
Fault
AtomicSimpleCPU::write(T data, Addr addr, unsigned flags, uint64_t *res)
{
    // use the CPU's statically allocated write request and packet objects
    Request *req = data_write_req;
    PacketPtr pkt = data_write_pkt;

    req->setVirt(0, addr, sizeof(T), flags, thread->readPC());

    if (traceData) {
        traceData->setAddr(addr);
    }

    // translate to physical address
    Fault fault = thread->translateDataWriteReq(req);

    // Now do the access.
    if (fault == NoFault) {
        bool do_access = true;  // flag to suppress cache access

        if (req->isLocked()) {
            do_access = TheISA::handleLockedWrite(thread, req);
        }

        if (do_access) {
            data = htog(data);
            pkt->reinitFromRequest();
            pkt->dataStatic(&data);

            dcache_latency = dcachePort.sendAtomic(pkt);
            dcache_access = true;

            assert(pkt->result == Packet::Success);
        }

        if (req->isLocked()) {
            uint64_t scResult = req->getScResult();
            if (scResult != 0) {
                // clear failure counter
                thread->setStCondFailures(0);
            }
            if (res) {
                *res = req->getScResult();
            }
        }
    }

    // This will need a new way to tell if it's hooked up to a cache or not.
    if (req->isUncacheable())
        recordEvent("Uncached Write");

    // If the write needs to have a fault on the access, consider calling
    // changeStatus() and changing it to "bad addr write" or something.
    return fault;
}


#ifndef DOXYGEN_SHOULD_SKIP_THIS
template
Fault
AtomicSimpleCPU::write(uint64_t data, Addr addr,
                       unsigned flags, uint64_t *res);

template
Fault
AtomicSimpleCPU::write(uint32_t data, Addr addr,
                       unsigned flags, uint64_t *res);

template
Fault
AtomicSimpleCPU::write(uint16_t data, Addr addr,
                       unsigned flags, uint64_t *res);

template
Fault
AtomicSimpleCPU::write(uint8_t data, Addr addr,
                       unsigned flags, uint64_t *res);

#endif //DOXYGEN_SHOULD_SKIP_THIS

template<>
Fault
AtomicSimpleCPU::write(double data, Addr addr, unsigned flags, uint64_t *res)
{
    return write(*(uint64_t*)&data, addr, flags, res);
}

template<>
Fault
AtomicSimpleCPU::write(float data, Addr addr, unsigned flags, uint64_t *res)
{
    return write(*(uint32_t*)&data, addr, flags, res);
}


template<>
Fault
AtomicSimpleCPU::write(int32_t data, Addr addr, unsigned flags, uint64_t *res)
{
    return write((uint32_t)data, addr, flags, res);
}


void
AtomicSimpleCPU::tick()
{
    Tick latency = cycles(1); // instruction takes one cycle by default

    for (int i = 0; i < width; ++i) {
        numCycles++;

        if (!curStaticInst || !curStaticInst->isDelayedCommit())
            checkForInterrupts();

        Fault fault = setupFetchRequest(ifetch_req);

        if (fault == NoFault) {
            ifetch_pkt->reinitFromRequest();

            Tick icache_latency = icachePort.sendAtomic(ifetch_pkt);
            // ifetch_req is initialized to read the instruction directly
            // into the CPU object's inst field.

            dcache_access = false; // assume no dcache access
            preExecute();
            fault = curStaticInst->execute(this, traceData);
            postExecute();

            if (simulate_stalls) {
                Tick icache_stall = icache_latency - cycles(1);
                Tick dcache_stall =
                    dcache_access ? dcache_latency - cycles(1) : 0;
                Tick stall_cycles = (icache_stall + dcache_stall) / cycles(1);
                if (cycles(stall_cycles) < (icache_stall + dcache_stall))
                    latency += cycles(stall_cycles+1);
                else
                    latency += cycles(stall_cycles);
            }

        }

        advancePC(fault);
    }

    if (_status != Idle)
        tickEvent.schedule(curTick + latency);
}


////////////////////////////////////////////////////////////////////////
//
//  AtomicSimpleCPU Simulation Object
//
BEGIN_DECLARE_SIM_OBJECT_PARAMS(AtomicSimpleCPU)

    Param<Counter> max_insts_any_thread;
    Param<Counter> max_insts_all_threads;
    Param<Counter> max_loads_any_thread;
    Param<Counter> max_loads_all_threads;
    Param<Tick> progress_interval;
    SimObjectParam<System *> system;
    Param<int> cpu_id;

#if FULL_SYSTEM
    SimObjectParam<TheISA::ITB *> itb;
    SimObjectParam<TheISA::DTB *> dtb;
    Param<Tick> profile;
#else
    SimObjectParam<Process *> workload;
#endif // FULL_SYSTEM

    Param<int> clock;

    Param<bool> defer_registration;
    Param<int> width;
    Param<bool> function_trace;
    Param<Tick> function_trace_start;
    Param<bool> simulate_stalls;

END_DECLARE_SIM_OBJECT_PARAMS(AtomicSimpleCPU)

BEGIN_INIT_SIM_OBJECT_PARAMS(AtomicSimpleCPU)

    INIT_PARAM(max_insts_any_thread,
               "terminate when any thread reaches this inst count"),
    INIT_PARAM(max_insts_all_threads,
               "terminate when all threads have reached this inst count"),
    INIT_PARAM(max_loads_any_thread,
               "terminate when any thread reaches this load count"),
    INIT_PARAM(max_loads_all_threads,
               "terminate when all threads have reached this load count"),
    INIT_PARAM(progress_interval, "Progress interval"),
    INIT_PARAM(system, "system object"),
    INIT_PARAM(cpu_id, "processor ID"),

#if FULL_SYSTEM
    INIT_PARAM(itb, "Instruction TLB"),
    INIT_PARAM(dtb, "Data TLB"),
    INIT_PARAM(profile, ""),
#else
    INIT_PARAM(workload, "processes to run"),
#endif // FULL_SYSTEM

    INIT_PARAM(clock, "clock speed"),
    INIT_PARAM(defer_registration, "defer system registration (for sampling)"),
    INIT_PARAM(width, "cpu width"),
    INIT_PARAM(function_trace, "Enable function trace"),
    INIT_PARAM(function_trace_start, "Cycle to start function trace"),
    INIT_PARAM(simulate_stalls, "Simulate cache stall cycles")

END_INIT_SIM_OBJECT_PARAMS(AtomicSimpleCPU)


CREATE_SIM_OBJECT(AtomicSimpleCPU)
{
    AtomicSimpleCPU::Params *params = new AtomicSimpleCPU::Params();
    params->name = getInstanceName();
    params->numberOfThreads = 1;
    params->max_insts_any_thread = max_insts_any_thread;
    params->max_insts_all_threads = max_insts_all_threads;
    params->max_loads_any_thread = max_loads_any_thread;
    params->max_loads_all_threads = max_loads_all_threads;
    params->progress_interval = progress_interval;
    params->deferRegistration = defer_registration;
    params->clock = clock;
    params->functionTrace = function_trace;
    params->functionTraceStart = function_trace_start;
    params->width = width;
    params->simulate_stalls = simulate_stalls;
    params->system = system;
    params->cpu_id = cpu_id;

#if FULL_SYSTEM
    params->itb = itb;
    params->dtb = dtb;
    params->profile = profile;
#else
    params->process = workload;
#endif

    AtomicSimpleCPU *cpu = new AtomicSimpleCPU(params);
    return cpu;
}

REGISTER_SIM_OBJECT("AtomicSimpleCPU", AtomicSimpleCPU)

