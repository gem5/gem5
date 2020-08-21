/*
 * Copyright (c) 2013 ARM Limited
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
 * Copyright (c) 2011 Google
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

#ifndef __ARCH_GENERIC_MEMHELPERS_HH__
#define __ARCH_GENERIC_MEMHELPERS_HH__

#include "base/types.hh"
#include "mem/packet.hh"
#include "mem/request.hh"
#include "sim/byteswap.hh"
#include "sim/insttracer.hh"

/// Initiate a read from memory in timing mode.  Note that the 'mem'
/// parameter is unused; only the type of that parameter is used
/// to determine the size of the access.
template <class XC, class MemT>
Fault
initiateMemRead(XC *xc, Trace::InstRecord *traceData, Addr addr,
                MemT &mem, Request::Flags flags)
{
    return xc->initiateMemRead(addr, sizeof(MemT), flags);
}

/// Extract the data returned from a timing mode read.
template <ByteOrder Order, class MemT>
void
getMem(PacketPtr pkt, MemT &mem, Trace::InstRecord *traceData)
{
    mem = pkt->get<MemT>(Order);
    if (traceData)
        traceData->setData(mem);
}

template <class MemT>
void
getMemLE(PacketPtr pkt, MemT &mem, Trace::InstRecord *traceData)
{
    getMem<ByteOrder::little>(pkt, mem, traceData);
}

template <class MemT>
void
getMemBE(PacketPtr pkt, MemT &mem, Trace::InstRecord *traceData)
{
    getMem<ByteOrder::big>(pkt, mem, traceData);
}

/// Read from memory in atomic mode.
template <ByteOrder Order, class XC, class MemT>
Fault
readMemAtomic(XC *xc, Trace::InstRecord *traceData, Addr addr, MemT &mem,
              Request::Flags flags)
{
    memset(&mem, 0, sizeof(mem));
    Fault fault = xc->readMem(addr, (uint8_t *)&mem, sizeof(MemT), flags);
    if (fault == NoFault) {
        mem = gtoh(mem, Order);
        if (traceData)
            traceData->setData(mem);
    }
    return fault;
}

template <class XC, class MemT>
Fault
readMemAtomicLE(XC *xc, Trace::InstRecord *traceData, Addr addr, MemT &mem,
                Request::Flags flags)
{
    return readMemAtomic<ByteOrder::little>(
            xc, traceData, addr, mem, flags);
}

template <class XC, class MemT>
Fault
readMemAtomicBE(XC *xc, Trace::InstRecord *traceData, Addr addr, MemT &mem,
                Request::Flags flags)
{
    return readMemAtomic<ByteOrder::big>(xc, traceData, addr, mem, flags);
}

/// Write to memory in timing mode.
template <ByteOrder Order, class XC, class MemT>
Fault
writeMemTiming(XC *xc, Trace::InstRecord *traceData, MemT mem, Addr addr,
               Request::Flags flags, uint64_t *res)
{
    if (traceData) {
        traceData->setData(mem);
    }
    mem = htog(mem, Order);
    return xc->writeMem((uint8_t *)&mem, sizeof(MemT), addr, flags, res);
}

template <class XC, class MemT>
Fault
writeMemTimingLE(XC *xc, Trace::InstRecord *traceData, MemT mem, Addr addr,
               Request::Flags flags, uint64_t *res)
{
    return writeMemTiming<ByteOrder::little>(
            xc, traceData, mem, addr, flags, res);
}

template <class XC, class MemT>
Fault
writeMemTimingBE(XC *xc, Trace::InstRecord *traceData, MemT mem, Addr addr,
               Request::Flags flags, uint64_t *res)
{
    return writeMemTiming<ByteOrder::big>(
            xc, traceData, mem, addr, flags, res);
}

/// Write to memory in atomic mode.
template <ByteOrder Order, class XC, class MemT>
Fault
writeMemAtomic(XC *xc, Trace::InstRecord *traceData, const MemT &mem,
               Addr addr, Request::Flags flags, uint64_t *res)
{
    if (traceData) {
        traceData->setData(mem);
    }
    MemT host_mem = htog(mem, Order);
    Fault fault =
          xc->writeMem((uint8_t *)&host_mem, sizeof(MemT), addr, flags, res);
    if (fault == NoFault && res != NULL) {
        if (flags & Request::MEM_SWAP || flags & Request::MEM_SWAP_COND)
            *(MemT *)res = gtoh(*(MemT *)res, Order);
        else
            *res = gtoh(*res, Order);
    }
    return fault;
}

template <class XC, class MemT>
Fault
writeMemAtomicLE(XC *xc, Trace::InstRecord *traceData, const MemT &mem,
                 Addr addr, Request::Flags flags, uint64_t *res)
{
    return writeMemAtomic<ByteOrder::little>(
            xc, traceData, mem, addr, flags, res);
}

template <class XC, class MemT>
Fault
writeMemAtomicBE(XC *xc, Trace::InstRecord *traceData, const MemT &mem,
                 Addr addr, Request::Flags flags, uint64_t *res)
{
    return writeMemAtomic<ByteOrder::big>(
            xc, traceData, mem, addr, flags, res);
}

/// Do atomic read-modify-write (AMO) in atomic mode
template <ByteOrder Order, class XC, class MemT>
Fault
amoMemAtomic(XC *xc, Trace::InstRecord *traceData, MemT &mem, Addr addr,
             Request::Flags flags, AtomicOpFunctor *_amo_op)
{
    assert(_amo_op);

    // mem will hold the previous value at addr after the AMO completes
    memset(&mem, 0, sizeof(mem));

    AtomicOpFunctorPtr amo_op = AtomicOpFunctorPtr(_amo_op);
    Fault fault = xc->amoMem(addr, (uint8_t *)&mem, sizeof(MemT), flags,
                             std::move(amo_op));

    if (fault == NoFault) {
        mem = gtoh(mem, Order);
        if (traceData)
            traceData->setData(mem);
    }
    return fault;
}

template <class XC, class MemT>
Fault
amoMemAtomicLE(XC *xc, Trace::InstRecord *traceData, MemT &mem, Addr addr,
               Request::Flags flags, AtomicOpFunctor *_amo_op)
{
    return amoMemAtomic<ByteOrder::little>(
            xc, traceData, mem, addr, flags, _amo_op);
}

template <class XC, class MemT>
Fault
amoMemAtomicBE(XC *xc, Trace::InstRecord *traceData, MemT &mem, Addr addr,
               Request::Flags flags, AtomicOpFunctor *_amo_op)
{
    return amoMemAtomic<ByteOrder::big>(
            xc, traceData, mem, addr, flags, _amo_op);
}

/// Do atomic read-modify-wrote (AMO) in timing mode
template <class XC, class MemT>
Fault
initiateMemAMO(XC *xc, Trace::InstRecord *traceData, Addr addr, MemT& mem,
               Request::Flags flags, AtomicOpFunctor *_amo_op)
{
    assert(_amo_op);
    AtomicOpFunctorPtr amo_op = AtomicOpFunctorPtr(_amo_op);
    return xc->initiateMemAMO(addr, sizeof(MemT), flags, std::move(amo_op));
}

#endif
