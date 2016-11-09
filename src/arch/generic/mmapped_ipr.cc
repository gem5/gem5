/*
 * Copyright (c) 2013 Andreas Sandberg
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
 * Authors: Andreas Sandberg
 */

#include "arch/generic/mmapped_ipr.hh"

#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/pseudo_inst.hh"

using namespace GenericISA;

static void
handlePseudoInst(ThreadContext *xc, Packet *pkt)
{
    const Addr offset(pkt->getAddr() & IPR_IN_CLASS_MASK);
    const uint8_t func((offset >> 8) & 0xFF);
    const uint8_t subfunc(offset & 0xFF);
    uint64_t ret;

    assert((offset >> 16) == 0);
    ret = PseudoInst::pseudoInst(xc, func, subfunc);
    if (pkt->isRead())
        pkt->set(ret);
}

Cycles
GenericISA::handleGenericIprRead(ThreadContext *xc, Packet *pkt)
{
    Addr va(pkt->getAddr());
    Addr cls(va >> IPR_CLASS_SHIFT);

    switch (cls) {
    case IPR_CLASS_PSEUDO_INST:
        handlePseudoInst(xc, pkt);
        break;
    default:
        panic("Unhandled generic IPR read: 0x%x\n", va);
    }

    return Cycles(1);
}

Cycles
GenericISA::handleGenericIprWrite(ThreadContext *xc, Packet *pkt)
{
    Addr va(pkt->getAddr());
    Addr cls(va >> IPR_CLASS_SHIFT);

    switch (cls) {
    case IPR_CLASS_PSEUDO_INST:
        handlePseudoInst(xc, pkt);
        break;
    default:
        panic("Unhandled generic IPR write: 0x%x\n", va);
    }

    return Cycles(1);
}
