/*
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
 *
 * Authors: Gabe Black
 */

#ifndef __ARCH_X86_MEMHELPERS_HH__
#define __ARCH_X86_MEMHELPERS_HH__

#include "base/types.hh"
#include "sim/byteswap.hh"
#include "sim/insttracer.hh"

namespace X86ISA
{

template <class XC>
Fault
readMemTiming(XC *xc, Trace::InstRecord *traceData, Addr addr,
        uint64_t &mem, unsigned dataSize, unsigned flags)
{
    return xc->readMem(addr, (uint8_t *)&mem, dataSize, flags);
}

static inline uint64_t
getMem(PacketPtr pkt, unsigned dataSize, Trace::InstRecord *traceData)
{
    uint64_t mem;
    switch (dataSize) {
      case 1:
        mem = pkt->get<uint8_t>();
        break;
      case 2:
        mem = pkt->get<uint16_t>();
        break;
      case 4:
        mem = pkt->get<uint32_t>();
        break;
      case 8:
        mem = pkt->get<uint64_t>();
        break;
      default:
        panic("Unhandled size in getMem.\n");
    }
    if (traceData)
        traceData->setData(mem);
    return mem;
}

template <class XC>
Fault
readMemAtomic(XC *xc, Trace::InstRecord *traceData, Addr addr, uint64_t &mem,
        unsigned dataSize, unsigned flags)
{
    memset(&mem, 0, sizeof(mem));
    Fault fault = readMemTiming(xc, traceData, addr, mem, dataSize, flags);
    if (fault == NoFault) {
        // If LE to LE, this is a nop, if LE to BE, the actual data ends up
        // in the right place because the LSBs where at the low addresses on
        // access. This doesn't work for BE guests.
        mem = gtoh(mem);
        if (traceData)
            traceData->setData(mem);
    }
    return fault;
}

template <class XC>
Fault
writeMemTiming(XC *xc, Trace::InstRecord *traceData, uint64_t mem,
        unsigned dataSize, Addr addr, unsigned flags, uint64_t *res)
{
    if (traceData) {
        traceData->setData(mem);
    }
    mem = TheISA::htog(mem);
    return xc->writeMem((uint8_t *)&mem, dataSize, addr, flags, res);
}

template <class XC>
Fault
writeMemAtomic(XC *xc, Trace::InstRecord *traceData, uint64_t mem,
        unsigned dataSize, Addr addr, unsigned flags, uint64_t *res)
{
    Fault fault = writeMemTiming(xc, traceData, mem, dataSize, addr, flags,
            res);
    if (fault == NoFault && res != NULL) {
        *res = gtoh(*res);
    }
    return fault;
}

}

#endif
