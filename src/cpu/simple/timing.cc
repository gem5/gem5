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

#include "arch/utility.hh"
#include "cpu/exetrace.hh"
#include "cpu/simple/timing.hh"
#include "mem/packet_impl.hh"
#include "sim/builder.hh"

using namespace std;
using namespace TheISA;

Port *
TimingSimpleCPU::getPort(const std::string &if_name, int idx)
{
    if (if_name == "dcache_port")
        return &dcachePort;
    else if (if_name == "icache_port")
        return &icachePort;
    else
        panic("No Such Port\n");
}

void
TimingSimpleCPU::init()
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
    drainEvent = NULL;
    state = SimObject::Timing;
}


TimingSimpleCPU::~TimingSimpleCPU()
{
}

void
TimingSimpleCPU::serialize(ostream &os)
{
    SERIALIZE_ENUM(_status);
    BaseSimpleCPU::serialize(os);
}

void
TimingSimpleCPU::unserialize(Checkpoint *cp, const string &section)
{
    UNSERIALIZE_ENUM(_status);
    BaseSimpleCPU::unserialize(cp, section);
}

bool
TimingSimpleCPU::drain(Event *drain_event)
{
    // TimingSimpleCPU is ready to drain if it's not waiting for
    // an access to complete.
    if (status() == Idle || status() == Running || status() == SwitchedOut) {
        changeState(SimObject::DrainedTiming);
        return false;
    } else {
        changeState(SimObject::Draining);
        drainEvent = drain_event;
        return true;
    }
}

void
TimingSimpleCPU::resume()
{
    if (_status != SwitchedOut && _status != Idle) {
        Event *e =
            new EventWrapper<TimingSimpleCPU, &TimingSimpleCPU::fetch>(this, true);
        e->schedule(curTick);
    }
}

void
TimingSimpleCPU::setMemoryMode(State new_mode)
{
    assert(new_mode == SimObject::Timing);
}

void
TimingSimpleCPU::switchOut()
{
    assert(status() == Running || status() == Idle);
    _status = SwitchedOut;
}


void
TimingSimpleCPU::takeOverFrom(BaseCPU *oldCPU)
{
    BaseCPU::takeOverFrom(oldCPU);

    // if any of this CPU's ThreadContexts are active, mark the CPU as
    // running and schedule its tick event.
    for (int i = 0; i < threadContexts.size(); ++i) {
        ThreadContext *tc = threadContexts[i];
        if (tc->status() == ThreadContext::Active && _status != Running) {
            _status = Running;
            break;
        }
    }
}


void
TimingSimpleCPU::activateContext(int thread_num, int delay)
{
    assert(thread_num == 0);
    assert(thread);

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
    assert(thread);

    assert(_status == Running);

    // just change status to Idle... if status != Running,
    // completeInst() will not initiate fetch of next instruction.

    notIdleFraction--;
    _status = Idle;
}


template <class T>
Fault
TimingSimpleCPU::read(Addr addr, T &data, unsigned flags)
{
    // need to fill in CPU & thread IDs here
    Request *data_read_req = new Request();
    data_read_req->setThreadContext(0,0); //Need CPU/Thread IDS HERE
    data_read_req->setVirt(0, addr, sizeof(T), flags, thread->readPC());

    if (traceData) {
        traceData->setAddr(data_read_req->getVaddr());
    }

   // translate to physical address
    Fault fault = thread->translateDataReadReq(data_read_req);

    // Now do the access.
    if (fault == NoFault) {
        Packet *data_read_pkt =
            new Packet(data_read_req, Packet::ReadReq, Packet::Broadcast);
        data_read_pkt->dataDynamic<T>(new T);

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
    // need to fill in CPU & thread IDs here
    Request *data_write_req = new Request();
    data_write_req->setThreadContext(0,0); //Need CPU/Thread IDS HERE
    data_write_req->setVirt(0, addr, sizeof(T), flags, thread->readPC());

    // translate to physical address
    Fault fault = thread->translateDataWriteReq(data_write_req);
    // Now do the access.
    if (fault == NoFault) {
        Packet *data_write_pkt =
            new Packet(data_write_req, Packet::WriteReq, Packet::Broadcast);
        data_write_pkt->allocate();
        data_write_pkt->set(data);

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

    // need to fill in CPU & thread IDs here
    Request *ifetch_req = new Request();
    ifetch_req->setThreadContext(0,0); //Need CPU/Thread IDS HERE
    Fault fault = setupFetchRequest(ifetch_req);

    ifetch_pkt = new Packet(ifetch_req, Packet::ReadReq, Packet::Broadcast);
    ifetch_pkt->dataStatic(&inst);

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
        // fetch fault: advance directly to next instruction (fault handler)
        advanceInst(fault);
    }
}


void
TimingSimpleCPU::advanceInst(Fault fault)
{
    advancePC(fault);

    if (_status == Running) {
        // kick off fetch of next instruction... callback from icache
        // response will cause that instruction to be executed,
        // keeping the CPU running.
        fetch();
    }
}


void
TimingSimpleCPU::completeIfetch(Packet *pkt)
{
    // received a response from the icache: execute the received
    // instruction
    assert(pkt->result == Packet::Success);
    assert(_status == IcacheWaitResponse);

    _status = Running;

    delete pkt->req;
    delete pkt;

    if (getState() == SimObject::Draining) {
        completeDrain();
        return;
    }

    preExecute();
    if (curStaticInst->isMemRef() && !curStaticInst->isDataPrefetch()) {
        // load or store: just send to dcache
        Fault fault = curStaticInst->initiateAcc(this, traceData);
        if (fault == NoFault) {
            // successfully initiated access: instruction will
            // complete in dcache response callback
            assert(_status == DcacheWaitResponse);
        } else {
            // fault: complete now to invoke fault handler
            postExecute();
            advanceInst(fault);
        }
    } else {
        // non-memory instruction: execute completely now
        Fault fault = curStaticInst->execute(this, traceData);
        postExecute();
        advanceInst(fault);
    }
}


bool
TimingSimpleCPU::IcachePort::recvTiming(Packet *pkt)
{
    if (cpu->_status == DcacheWaitResponse)
        cpu->completeDataAccess(pkt);
    else if (cpu->_status == IcacheWaitResponse)
        cpu->completeIfetch(pkt);
    else
        assert("OOPS" && 0);
    return true;
}

void
TimingSimpleCPU::IcachePort::recvRetry()
{
    // we shouldn't get a retry unless we have a packet that we're
    // waiting to transmit
    assert(cpu->ifetch_pkt != NULL);
    assert(cpu->_status == IcacheRetry);
    Packet *tmp = cpu->ifetch_pkt;
    if (sendTiming(tmp)) {
        cpu->_status = IcacheWaitResponse;
        cpu->ifetch_pkt = NULL;
    }
}

void
TimingSimpleCPU::completeDataAccess(Packet *pkt)
{
    // received a response from the dcache: complete the load or store
    // instruction
    assert(pkt->result == Packet::Success);
    assert(_status == DcacheWaitResponse);
    _status = Running;

    if (getState() == SimObject::Draining) {
        completeDrain();

        delete pkt->req;
        delete pkt;

        return;
    }

    Fault fault = curStaticInst->completeAcc(pkt, this, traceData);

    delete pkt->req;
    delete pkt;

    postExecute();
    advanceInst(fault);
}


void
TimingSimpleCPU::completeDrain()
{
    DPRINTF(Config, "Done draining\n");
    changeState(SimObject::DrainedTiming);
    drainEvent->process();
}

bool
TimingSimpleCPU::DcachePort::recvTiming(Packet *pkt)
{
    cpu->completeDataAccess(pkt);
    return true;
}

void
TimingSimpleCPU::DcachePort::recvRetry()
{
    // we shouldn't get a retry unless we have a packet that we're
    // waiting to transmit
    assert(cpu->dcache_pkt != NULL);
    assert(cpu->_status == DcacheRetry);
    Packet *tmp = cpu->dcache_pkt;
    if (sendTiming(tmp)) {
        cpu->_status = DcacheWaitResponse;
        cpu->dcache_pkt = NULL;
    }
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

