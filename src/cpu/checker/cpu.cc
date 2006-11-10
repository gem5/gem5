/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 */

#include <list>
#include <string>

#include "cpu/base.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/simple_thread.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"

#if FULL_SYSTEM
#include "arch/kernel_stats.hh"
#include "arch/vtophys.hh"
#endif // FULL_SYSTEM

using namespace std;
//The CheckerCPU does alpha only
using namespace AlphaISA;

void
CheckerCPU::init()
{
}

CheckerCPU::CheckerCPU(Params *p)
    : BaseCPU(p), thread(NULL), tc(NULL)
{
    memReq = NULL;

    numInst = 0;
    startNumInst = 0;
    numLoad = 0;
    startNumLoad = 0;
    youngestSN = 0;

    changedPC = willChangePC = changedNextPC = false;

    exitOnError = p->exitOnError;
    warnOnlyOnLoadError = p->warnOnlyOnLoadError;
#if FULL_SYSTEM
    itb = p->itb;
    dtb = p->dtb;
    systemPtr = NULL;
#else
    process = p->process;
    thread = new SimpleThread(this, /* thread_num */ 0, process,
                              /* asid */ 0);

    thread->setStatus(ThreadContext::Suspended);
    tc = thread->getTC();
    threadContexts.push_back(tc);
#endif

    result.integer = 0;
}

CheckerCPU::~CheckerCPU()
{
}

void
CheckerCPU::setSystem(System *system)
{
#if FULL_SYSTEM
    systemPtr = system;

    thread = new SimpleThread(this, 0, systemPtr, itb, dtb, false);

    thread->setStatus(ThreadContext::Suspended);
    tc = thread->getTC();
    threadContexts.push_back(tc);
    delete thread->kernelStats;
    thread->kernelStats = NULL;
#endif
}

void
CheckerCPU::setIcachePort(Port *icache_port)
{
    icachePort = icache_port;
}

void
CheckerCPU::setDcachePort(Port *dcache_port)
{
    dcachePort = dcache_port;
}

void
CheckerCPU::serialize(ostream &os)
{
/*
    BaseCPU::serialize(os);
    SERIALIZE_SCALAR(inst);
    nameOut(os, csprintf("%s.xc", name()));
    thread->serialize(os);
    cacheCompletionEvent.serialize(os);
*/
}

void
CheckerCPU::unserialize(Checkpoint *cp, const string &section)
{
/*
    BaseCPU::unserialize(cp, section);
    UNSERIALIZE_SCALAR(inst);
    thread->unserialize(cp, csprintf("%s.xc", section));
*/
}

Fault
CheckerCPU::copySrcTranslate(Addr src)
{
    panic("Unimplemented!");
}

Fault
CheckerCPU::copy(Addr dest)
{
    panic("Unimplemented!");
}

template <class T>
Fault
CheckerCPU::read(Addr addr, T &data, unsigned flags)
{
    // need to fill in CPU & thread IDs here
    memReq = new Request();

    memReq->setVirt(0, addr, sizeof(T), flags, thread->readPC());

    // translate to physical address
    translateDataReadReq(memReq);

    PacketPtr pkt = new Packet(memReq, Packet::ReadReq, Packet::Broadcast);

    pkt->dataStatic(&data);

    if (!(memReq->isUncacheable())) {
        // Access memory to see if we have the same data
        dcachePort->sendFunctional(pkt);
    } else {
        // Assume the data is correct if it's an uncached access
        memcpy(&data, &unverifiedResult.integer, sizeof(T));
    }

    delete pkt;

    return NoFault;
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS

template
Fault
CheckerCPU::read(Addr addr, uint64_t &data, unsigned flags);

template
Fault
CheckerCPU::read(Addr addr, uint32_t &data, unsigned flags);

template
Fault
CheckerCPU::read(Addr addr, uint16_t &data, unsigned flags);

template
Fault
CheckerCPU::read(Addr addr, uint8_t &data, unsigned flags);

#endif //DOXYGEN_SHOULD_SKIP_THIS

template<>
Fault
CheckerCPU::read(Addr addr, double &data, unsigned flags)
{
    return read(addr, *(uint64_t*)&data, flags);
}

template<>
Fault
CheckerCPU::read(Addr addr, float &data, unsigned flags)
{
    return read(addr, *(uint32_t*)&data, flags);
}

template<>
Fault
CheckerCPU::read(Addr addr, int32_t &data, unsigned flags)
{
    return read(addr, (uint32_t&)data, flags);
}

template <class T>
Fault
CheckerCPU::write(T data, Addr addr, unsigned flags, uint64_t *res)
{
    // need to fill in CPU & thread IDs here
    memReq = new Request();

    memReq->setVirt(0, addr, sizeof(T), flags, thread->readPC());

    // translate to physical address
    thread->translateDataWriteReq(memReq);

    // Can compare the write data and result only if it's cacheable,
    // not a store conditional, or is a store conditional that
    // succeeded.
    // @todo: Verify that actual memory matches up with these values.
    // Right now it only verifies that the instruction data is the
    // same as what was in the request that got sent to memory; there
    // is no verification that it is the same as what is in memory.
    // This is because the LSQ would have to be snooped in the CPU to
    // verify this data.
    if (unverifiedReq &&
        !(unverifiedReq->isUncacheable()) &&
        (!(unverifiedReq->isLocked()) ||
         ((unverifiedReq->isLocked()) &&
          unverifiedReq->getScResult() == 1))) {
        T inst_data;
/*
        // This code would work if the LSQ allowed for snooping.
        PacketPtr pkt = new Packet(memReq, Packet::ReadReq, Packet::Broadcast);
        pkt.dataStatic(&inst_data);

        dcachePort->sendFunctional(pkt);

        delete pkt;
*/
        memcpy(&inst_data, unverifiedMemData, sizeof(T));

        if (data != inst_data) {
            warn("%lli: Store value does not match value in memory! "
                 "Instruction: %#x, memory: %#x",
                 curTick, inst_data, data);
            handleError();
        }
    }

    // Assume the result was the same as the one passed in.  This checker
    // doesn't check if the SC should succeed or fail, it just checks the
    // value.
    if (res && unverifiedReq->scResultValid())
        *res = unverifiedReq->getScResult();

    return NoFault;
}


#ifndef DOXYGEN_SHOULD_SKIP_THIS
template
Fault
CheckerCPU::write(uint64_t data, Addr addr, unsigned flags, uint64_t *res);

template
Fault
CheckerCPU::write(uint32_t data, Addr addr, unsigned flags, uint64_t *res);

template
Fault
CheckerCPU::write(uint16_t data, Addr addr, unsigned flags, uint64_t *res);

template
Fault
CheckerCPU::write(uint8_t data, Addr addr, unsigned flags, uint64_t *res);

#endif //DOXYGEN_SHOULD_SKIP_THIS

template<>
Fault
CheckerCPU::write(double data, Addr addr, unsigned flags, uint64_t *res)
{
    return write(*(uint64_t*)&data, addr, flags, res);
}

template<>
Fault
CheckerCPU::write(float data, Addr addr, unsigned flags, uint64_t *res)
{
    return write(*(uint32_t*)&data, addr, flags, res);
}

template<>
Fault
CheckerCPU::write(int32_t data, Addr addr, unsigned flags, uint64_t *res)
{
    return write((uint32_t)data, addr, flags, res);
}


#if FULL_SYSTEM
Addr
CheckerCPU::dbg_vtophys(Addr addr)
{
    return vtophys(tc, addr);
}
#endif // FULL_SYSTEM

bool
CheckerCPU::translateInstReq(Request *req)
{
#if FULL_SYSTEM
    return (thread->translateInstReq(req) == NoFault);
#else
    thread->translateInstReq(req);
    return true;
#endif
}

void
CheckerCPU::translateDataReadReq(Request *req)
{
    thread->translateDataReadReq(req);

    if (req->getVaddr() != unverifiedReq->getVaddr()) {
        warn("%lli: Request virtual addresses do not match! Inst: %#x, "
             "checker: %#x",
             curTick, unverifiedReq->getVaddr(), req->getVaddr());
        handleError();
    }
    req->setPaddr(unverifiedReq->getPaddr());

    if (checkFlags(req)) {
        warn("%lli: Request flags do not match! Inst: %#x, checker: %#x",
             curTick, unverifiedReq->getFlags(), req->getFlags());
        handleError();
    }
}

void
CheckerCPU::translateDataWriteReq(Request *req)
{
    thread->translateDataWriteReq(req);

    if (req->getVaddr() != unverifiedReq->getVaddr()) {
        warn("%lli: Request virtual addresses do not match! Inst: %#x, "
             "checker: %#x",
             curTick, unverifiedReq->getVaddr(), req->getVaddr());
        handleError();
    }
    req->setPaddr(unverifiedReq->getPaddr());

    if (checkFlags(req)) {
        warn("%lli: Request flags do not match! Inst: %#x, checker: %#x",
             curTick, unverifiedReq->getFlags(), req->getFlags());
        handleError();
    }
}

bool
CheckerCPU::checkFlags(Request *req)
{
    // Remove any dynamic flags that don't have to do with the request itself.
    unsigned flags = unverifiedReq->getFlags();
    unsigned mask = LOCKED | PHYSICAL | VPTE | ALTMODE | UNCACHEABLE | NO_FAULT;
    flags = flags & (mask);
    if (flags == req->getFlags()) {
        return false;
    } else {
        return true;
    }
}

void
CheckerCPU::dumpAndExit()
{
    warn("%lli: Checker PC:%#x, next PC:%#x",
         curTick, thread->readPC(), thread->readNextPC());
    panic("Checker found an error!");
}
