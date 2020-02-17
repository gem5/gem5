/*
 * Copyright (c) 2011 Google
 * Copyright (c) 2015 Advanced Micro Devices, Inc.
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

#ifndef __ARCH_X86_MEMHELPERS_HH__
#define __ARCH_X86_MEMHELPERS_HH__

#include <array>

#include "base/types.hh"
#include "cpu/exec_context.hh"
#include "sim/byteswap.hh"
#include "sim/insttracer.hh"

namespace X86ISA
{

/// Initiate a read from memory in timing mode.
static Fault
initiateMemRead(ExecContext *xc, Trace::InstRecord *traceData, Addr addr,
                unsigned dataSize, Request::Flags flags)
{
    return xc->initiateMemRead(addr, dataSize, flags);
}

static void
getMem(PacketPtr pkt, uint64_t &mem, unsigned dataSize,
       Trace::InstRecord *traceData)
{
    switch (dataSize) {
      case 1:
        mem = pkt->getLE<uint8_t>();
        break;
      case 2:
        mem = pkt->getLE<uint16_t>();
        break;
      case 4:
        mem = pkt->getLE<uint32_t>();
        break;
      case 8:
        mem = pkt->getLE<uint64_t>();
        break;
      default:
        panic("Unhandled size in getMem.\n");
    }
    if (traceData)
        traceData->setData(mem);
}

template <typename T, size_t N>
static void
getPackedMem(PacketPtr pkt, std::array<uint64_t, N> &mem, unsigned dataSize)
{
    std::array<T, N> real_mem = pkt->getLE<std::array<T, N> >();
    for (int i = 0; i < N; i++)
        mem[i] = real_mem[i];
}

template <size_t N>
static void
getMem(PacketPtr pkt, std::array<uint64_t, N> &mem, unsigned dataSize,
       Trace::InstRecord *traceData)
{
    switch (dataSize) {
      case 4:
        getPackedMem<uint32_t, N>(pkt, mem, dataSize);
        break;
      case 8:
        getPackedMem<uint64_t, N>(pkt, mem, dataSize);
        break;
      default:
        panic("Unhandled element size in getMem.\n");
    }
    if (traceData)
        traceData->setData(mem[0]);
}


static Fault
readMemAtomic(ExecContext *xc, Trace::InstRecord *traceData, Addr addr,
              uint64_t &mem, unsigned dataSize, Request::Flags flags)
{
    memset(&mem, 0, sizeof(mem));
    Fault fault = xc->readMem(addr, (uint8_t *)&mem, dataSize, flags);
    if (fault == NoFault) {
        // If LE to LE, this is a nop, if LE to BE, the actual data ends up
        // in the right place because the LSBs where at the low addresses on
        // access. This doesn't work for BE guests.
        mem = letoh(mem);
        if (traceData)
            traceData->setData(mem);
    }
    return fault;
}

template <typename T, size_t N>
static Fault
readPackedMemAtomic(ExecContext *xc, Addr addr, std::array<uint64_t, N> &mem,
                    unsigned flags)
{
    std::array<T, N> real_mem;
    Fault fault = xc->readMem(addr, (uint8_t *)&real_mem,
                              sizeof(T) * N, flags);
    if (fault == NoFault) {
        real_mem = letoh(real_mem);
        for (int i = 0; i < N; i++)
            mem[i] = real_mem[i];
    }
    return fault;
}

template <size_t N>
static Fault
readMemAtomic(ExecContext *xc, Trace::InstRecord *traceData, Addr addr,
              std::array<uint64_t, N> &mem, unsigned dataSize,
              unsigned flags)
{
    Fault fault = NoFault;

    switch (dataSize) {
      case 4:
        fault = readPackedMemAtomic<uint32_t, N>(xc, addr, mem, flags);
        break;
      case 8:
        fault = readPackedMemAtomic<uint64_t, N>(xc, addr, mem, flags);
        break;
      default:
        panic("Unhandled element size in readMemAtomic\n");
    }
    if (fault == NoFault && traceData)
        traceData->setData(mem[0]);
    return fault;
}

template <typename T, size_t N>
static Fault
writePackedMem(ExecContext *xc, std::array<uint64_t, N> &mem, Addr addr,
               unsigned flags, uint64_t *res)
{
    std::array<T, N> real_mem;
    for (int i = 0; i < N; i++)
        real_mem[i] = mem[i];
    real_mem = htole(real_mem);
    return xc->writeMem((uint8_t *)&real_mem, sizeof(T) * N,
                        addr, flags, res);
}

static Fault
writeMemTiming(ExecContext *xc, Trace::InstRecord *traceData, uint64_t mem,
               unsigned dataSize, Addr addr, Request::Flags flags,
               uint64_t *res)
{
    if (traceData)
        traceData->setData(mem);
    mem = htole(mem);
    return xc->writeMem((uint8_t *)&mem, dataSize, addr, flags, res);
}

template <size_t N>
static Fault
writeMemTiming(ExecContext *xc, Trace::InstRecord *traceData,
               std::array<uint64_t, N> &mem, unsigned dataSize,
               Addr addr, unsigned flags, uint64_t *res)
{
    if (traceData)
        traceData->setData(mem[0]);

    switch (dataSize) {
      case 4:
        return writePackedMem<uint32_t, N>(xc, mem, addr, flags, res);
      case 8:
        return writePackedMem<uint64_t, N>(xc, mem, addr, flags, res);
      default:
        panic("Unhandled element size in writeMemTiming.\n");
    }
}

static Fault
writeMemAtomic(ExecContext *xc, Trace::InstRecord *traceData, uint64_t mem,
               unsigned dataSize, Addr addr, Request::Flags flags,
               uint64_t *res)
{
    if (traceData)
        traceData->setData(mem);
    uint64_t host_mem = htole(mem);
    Fault fault =
          xc->writeMem((uint8_t *)&host_mem, dataSize, addr, flags, res);
    if (fault == NoFault && res)
        *res = letoh(*res);
    return fault;
}

template <size_t N>
static Fault
writeMemAtomic(ExecContext *xc, Trace::InstRecord *traceData,
               std::array<uint64_t, N> &mem, unsigned dataSize,
               Addr addr, unsigned flags, uint64_t *res)
{
    if (traceData)
        traceData->setData(mem[0]);

    Fault fault;
    switch (dataSize) {
      case 4:
        fault = writePackedMem<uint32_t, N>(xc, mem, addr, flags, res);
        break;
      case 8:
        fault = writePackedMem<uint64_t, N>(xc, mem, addr, flags, res);
        break;
      default:
        panic("Unhandled element size in writeMemAtomic.\n");
    }

    if (fault == NoFault && res)
        *res = letoh(*res);

    return fault;
}

}

#endif
