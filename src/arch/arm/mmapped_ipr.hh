/*
 * Copyright (c) 2006 The Regents of The University of Michigan
 * Copyright (c) 2007-2008 The Florida State University
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

#ifndef __ARCH_ARM_MMAPPED_IPR_HH__
#define __ARCH_ARM_MMAPPED_IPR_HH__

/**
 * @file
 *
 * ISA-specific helper functions for memory mapped IPR accesses.
 */

#include "base/types.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/pseudo_inst.hh"
#include "sim/system.hh"

class ThreadContext;

namespace ArmISA
{

inline Cycles
handleIprRead(ThreadContext *tc, Packet *pkt)
{
    Addr addr = pkt->getAddr();
    auto m5opRange = tc->getSystemPtr()->m5opRange();
    if (m5opRange.contains(addr)) {
        uint8_t func;
        PseudoInst::decodeAddrOffset(addr - m5opRange.start(), func);
        uint64_t ret = PseudoInst::pseudoInst<PseudoInstABI>(tc, func);
        pkt->setLE(ret);
    }
    return Cycles(1);
}

inline Cycles
handleIprWrite(ThreadContext *tc, Packet *pkt)
{
    Addr addr = pkt->getAddr();
    auto m5opRange = tc->getSystemPtr()->m5opRange();
    if (m5opRange.contains(addr)) {
        uint8_t func;
        PseudoInst::decodeAddrOffset(addr - m5opRange.start(), func);
        PseudoInst::pseudoInst<PseudoInstABI>(tc, func);
    }
    return Cycles(1);
}

} // namespace ArmISA

#endif
