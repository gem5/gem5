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

#include "arch/utility.hh"
#include "cpu/exetrace.hh"
#include "cpu/simple/timing.hh"
#include "mem/packet_impl.hh"
#include "sim/builder.hh"

using namespace std;
using namespace TheISA;


void
TimingSimpleCPU::init()
{
    //Create Memory Ports (conect them up)
    Port *mem_dport = mem->getPort("");
    dcachePort.setPeer(mem_dport);
    mem_dport->setPeer(&dcachePort);

    Port *mem_iport = mem->getPort("");
    icachePort.setPeer(mem_iport);
    mem_iport->setPeer(&icachePort);

    BaseCPU::init();
#if FULL_SYSTEM
    for (int i = 0; i < execContexts.size(); ++i) {
        ExecContext *xc = execContexts[i];

        // initialize CPU, including PC
        TheISA::initCPU(xc, xc->readCpuId());
    }
#endif
}

Tick
TimingSimpleCPU::CpuPort::recvAtomic(Packet *pkt)
{
    panic("TimingSimpleCPU doesn't expect recvAtomic callback!");
    return curTick;
}

void
TimingSimpleCPU::CpuPort::recvFunctional(Packet *pkt)
{
    panic("TimingSimpleCPU doesn't expect recvFunctional callback!");
}

void
TimingSimpleCPU::CpuPort::recvStatusChange(Status status)
{
    if (status == RangeChange)
        return;

    panic("TimingSimpleCPU doesn't expect recvStatusChange callback!");
}

TimingSimpleCPU::TimingSimpleCPU(Params *p)
    : BaseSimpleCPU(p), icachePort(this), dcachePort(this)
{
    _status = Idle;
    ifetch_pkt = dcache_pkt = NULL;
}


TimingSimpleCPU::~TimingSimpleCPU()
{
}

void
TimingSimpleCPU::serialize(ostream &os)
{
    BaseSimpleCPU::serialize(os);
    SERIALIZE_ENUM(_status);
}

void
TimingSimpleCPU::unserialize(Checkpoint *cp, const string &section)
{
    BaseSimpleCPU::unserialize(cp, section);
    UNSERIALIZE_ENUM(_status);
}

void
TimingSimpleCPU::switchOut(Sampler *s)
{
    sampler = s;
    if (status() == Running) {
        _status = SwitchedOut;
    }
    sampler->signalSwitched();
}


void
TimingSimpleCPU::takeOverFrom(BaseCPU *oldCPU)
{
    BaseCPU::takeOverFrom(oldCPU);

    // if any of this CPU's ExecContexts are active, mark the CPU as
    // running and schedule its tick event.
    for (int i = 0; i < execContexts.size(); ++i) {
        ExecContext *xc = execContexts[i];
        if (xc->status() == ExecContext::Active && _status != Running) {
            _status = Running;
            break;
        }
    }
}


void
TimingSimpleCPU::activateContext(int thread_num, int delay)
{
    assert(thread_num == 0);
    assert(cpuXC);

    assert(_status == Idle);

    notIdleFraction++;
    _status = Running;
    // kick things off by initiating the fetch of the next instruction
    Event *e =
        new EventWrapper<TimingSimpleCPU, &TimingSimpleCPU::fetch>(this, true);
    e->schedule(curTick + cycles(delay));
}


void
TimingSimpleCPU::suspendContext(int thread_num)
{
    assert(thread_num == 0);
    assert(cpuXC);

    panic("TimingSimpleCPU::suspendContext not implemented");

    assert(_status == Running);

    notIdleFraction--;
    _status = Idle;
}


template <class T>
Fault
TimingSimpleCPU::read(Addr addr, T &data, unsigned flags)
{
    Request *data_read_req = new Request(true);

    data_read_req->setVaddr(addr);
    data_read_req->setSize(sizeof(T));
    data_read_req->setFlags(flags);
    data_read_req->setTime(curTick);

    if (traceData) {
        traceData->setAddr(data_read_req->getVaddr());
    }

   // translate to physical address
    Fault fault = cpuXC->translateDataReadReq(data_read_req);

    // Now do the access.
    if (fault == NoFault) {
        Packet *data_read_pkt = new Packet;
        data_read_pkt->cmd = Read;
        data_read_pkt->req = data_read_req;
        data_read_pkt->dataDynamic<T>(new T);
        data_read_pkt->addr = data_read_req->getPaddr();
        data_read_pkt->size = sizeof(T);
        data_read_pkt->dest = Packet::Broadcast;

        if (!dcachePort.sendTiming(data_read_pkt)) {
            _status = DcacheRetry;
            dcache_pkt = data_read_pkt;
        } else {
            _status = DcacheWaitResponse;
            dcache_pkt = NULL;
        }
    }

    // This will need a new way to tell if it has a dcache attached.
    if (data_read_req->getFlags() & UNCACHEABLE)
        recordEvent("Uncached Read");

    return fault;
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS

template
Fault
TimingSimpleCPU::read(Addr addr, uint64_t &data, unsigned flags);

template
Fault
TimingSimpleCPU::read(Addr addr, uint32_t &data, unsigned flags);

template
Fault
TimingSimpleCPU::read(Addr addr, uint16_t &data, unsigned flags);

template
Fault
TimingSimpleCPU::read(Addr addr, uint8_t &data, unsigned flags);

#endif //DOXYGEN_SHOULD_SKIP_THIS

template<>
Fault
TimingSimpleCPU::read(Addr addr, double &data, unsigned flags)
{
    return read(addr, *(uint64_t*)&data, flags);
}

template<>
Fault
TimingSimpleCPU::read(Addr addr, float &data, unsigned flags)
{
    return read(addr, *(uint32_t*)&data, flags);
}


template<>
Fault
TimingSimpleCPU::read(Addr addr, int32_t &data, unsigned flags)
{
    return read(addr, (uint32_t&)data, flags);
}


template <class T>
Fault
TimingSimpleCPU::write(T data, Addr addr, unsigned flags, uint64_t *res)
{
    Request *data_write_req = new Request(true);
    data_write_req->setVaddr(addr);
    data_write_req->setTime(curTick);
    data_write_req->setSize(sizeof(T));
    data_write_req->setFlags(flags);

    // translate to physical address
    Fault fault = cpuXC->translateDataWriteReq(data_write_req);
    // Now do the access.
    if (fault == NoFault) {
        Packet *data_write_pkt = new Packet;
        data_write_pkt->cmd = Write;
        data_write_pkt->req = data_write_req;
        data_write_pkt->allocate();
        data_write_pkt->size = sizeof(T);
        data_write_pkt->set(data);
        data_write_pkt->addr = data_write_req->getPaddr();
        data_write_pkt->dest = Packet::Broadcast;

        if (!dcachePort.sendTiming(data_write_pkt)) {
            _status = DcacheRetry;
            dcache_pkt = data_write_pkt;
        } else {
            _status = DcacheWaitResponse;
            dcache_pkt = NULL;
        }
    }

    // This will need a new way to tell if it's hooked up to a cache or not.
    if (data_write_req->getFlags() & UNCACHEABLE)
        recordEvent("Uncached Write");

    // If the write needs to have a fault on the access, consider calling
    // changeStatus() and changing it to "bad addr write" or something.
    return fault;
}


#ifndef DOXYGEN_SHOULD_SKIP_THIS
template
Fault
TimingSimpleCPU::write(uint64_t data, Addr addr,
                       unsigned flags, uint64_t *res);

template
Fault
TimingSimpleCPU::write(uint32_t data, Addr addr,
                       unsigned flags, uint64_t *res);

template
Fault
TimingSimpleCPU::write(uint16_t data, Addr addr,
                       unsigned flags, uint64_t *res);

template
Fault
TimingSimpleCPU::write(uint8_t data, Addr addr,
                       unsigned flags, uint64_t *res);

#endif //DOXYGEN_SHOULD_SKIP_THIS

template<>
Fault
TimingSimpleCPU::write(double data, Addr addr, unsigned flags, uint64_t *res)
{
    return write(*(uint64_t*)&data, addr, flags, res);
}

template<>
Fault
TimingSimpleCPU::write(float data, Addr addr, unsigned flags, uint64_t *res)
{
    return write(*(uint32_t*)&data, addr, flags, res);
}


template<>
Fault
TimingSimpleCPU::write(int32_t data, Addr addr, unsigned flags, uint64_t *res)
{
    return write((uint32_t)data, addr, flags, res);
}


void
TimingSimpleCPU::fetch()
{
    checkForInterrupts();

    Request *ifetch_req = new Request(true);
    ifetch_req->setSize(sizeof(MachInst));

    ifetch_pkt = new Packet;
    ifetch_pkt->cmd = Read;
    ifetch_pkt->dataStatic(&inst);
    ifetch_pkt->req = ifetch_req;
    ifetch_pkt->size = sizeof(MachInst);
    ifetch_pkt->dest = Packet::Broadcast;

    Fault fault = setupFetchPacket(ifetch_pkt);
    if (fault == NoFault) {
        if (!icachePort.sendTiming(ifetch_pkt)) {
            // Need to wait for retry
            _status = IcacheRetry;
        } else {
            // Need to wait for cache to respond
            _status = IcacheWaitResponse;
            // ownership of packet transferred to memory system
            ifetch_pkt = NULL;
        }
    } else {
        panic("TimingSimpleCPU fetch fault handling not implemented");
    }
}


void
TimingSimpleCPU::completeInst(Fault fault)
{
    postExecute();

    if (traceData) {
        traceData->finalize();
    }

    advancePC(fault);

    if (_status == Running) {
        // kick off fetch of next instruction... callback from icache
        // response will cause that instruction to be executed,
        // keeping the CPU running.
        fetch();
    }
}


void
TimingSimpleCPU::completeIfetch()
{
    // received a response from the icache: execute the received
    // instruction
    assert(_status == IcacheWaitResponse);
    _status = Running;
    preExecute();
    if (curStaticInst->isMemRef()) {
        // load or store: just send to dcache
        Fault fault = curStaticInst->initiateAcc(this, traceData);
        assert(fault == NoFault);
        assert(_status == DcacheWaitResponse);
        // instruction will complete in dcache response callback
    } else {
        // non-memory instruction: execute completely now
        Fault fault = curStaticInst->execute(this, traceData);
        completeInst(fault);
    }
}


bool
TimingSimpleCPU::IcachePort::recvTiming(Packet *pkt)
{
    cpu->completeIfetch();
    return true;
}

Packet *
TimingSimpleCPU::IcachePort::recvRetry()
{
    // we shouldn't get a retry unless we have a packet that we're
    // waiting to transmit
    assert(cpu->ifetch_pkt != NULL);
    assert(cpu->_status == IcacheRetry);
    cpu->_status = IcacheWaitResponse;
    Packet *tmp = cpu->ifetch_pkt;
    cpu->ifetch_pkt = NULL;
    return tmp;
}

void
TimingSimpleCPU::completeDataAccess(Packet *pkt)
{
    // received a response from the dcache: complete the load or store
    // instruction
    assert(pkt->result == Success);
    assert(_status == DcacheWaitResponse);
    _status = Running;

    Fault fault = curStaticInst->completeAcc(pkt, this, traceData);

    completeInst(fault);
}



bool
TimingSimpleCPU::DcachePort::recvTiming(Packet *pkt)
{
    cpu->completeDataAccess(pkt);
    return true;
}

Packet *
TimingSimpleCPU::DcachePort::recvRetry()
{
    // we shouldn't get a retry unless we have a packet that we're
    // waiting to transmit
    assert(cpu->dcache_pkt != NULL);
    assert(cpu->_status == DcacheRetry);
    cpu->_status = DcacheWaitResponse;
    Packet *tmp = cpu->dcache_pkt;
    cpu->dcache_pkt = NULL;
    return tmp;
}


////////////////////////////////////////////////////////////////////////
//
//  TimingSimpleCPU Simulation Object
//
BEGIN_DECLARE_SIM_OBJECT_PARAMS(TimingSimpleCPU)

    Param<Counter> max_insts_any_thread;
    Param<Counter> max_insts_all_threads;
    Param<Counter> max_loads_any_thread;
    Param<Counter> max_loads_all_threads;
    SimObjectParam<MemObject *> mem;

#if FULL_SYSTEM
    SimObjectParam<AlphaITB *> itb;
    SimObjectParam<AlphaDTB *> dtb;
    SimObjectParam<System *> system;
    Param<int> cpu_id;
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

END_DECLARE_SIM_OBJECT_PARAMS(TimingSimpleCPU)

BEGIN_INIT_SIM_OBJECT_PARAMS(TimingSimpleCPU)

    INIT_PARAM(max_insts_any_thread,
               "terminate when any thread reaches this inst count"),
    INIT_PARAM(max_insts_all_threads,
               "terminate when all threads have reached this inst count"),
    INIT_PARAM(max_loads_any_thread,
               "terminate when any thread reaches this load count"),
    INIT_PARAM(max_loads_all_threads,
               "terminate when all threads have reached this load count"),
    INIT_PARAM(mem, "memory"),

#if FULL_SYSTEM
    INIT_PARAM(itb, "Instruction TLB"),
    INIT_PARAM(dtb, "Data TLB"),
    INIT_PARAM(system, "system object"),
    INIT_PARAM(cpu_id, "processor ID"),
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

END_INIT_SIM_OBJECT_PARAMS(TimingSimpleCPU)


CREATE_SIM_OBJECT(TimingSimpleCPU)
{
    TimingSimpleCPU::Params *params = new TimingSimpleCPU::Params();
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
    params->mem = mem;

#if FULL_SYSTEM
    params->itb = itb;
    params->dtb = dtb;
    params->system = system;
    params->cpu_id = cpu_id;
    params->profile = profile;
#else
    params->process = workload;
#endif

    TimingSimpleCPU *cpu = new TimingSimpleCPU(params);
    return cpu;
}

REGISTER_SIM_OBJECT("TimingSimpleCPU", TimingSimpleCPU)

