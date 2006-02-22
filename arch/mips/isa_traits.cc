/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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

#include "arch/mips/isa_traits.hh"
#include "config/full_system.hh"
#include "cpu/static_inst.hh"
#include "sim/serialize.hh"

using namespace MipsISA;

const Addr MipsISA::PageShift = 13;
const Addr MipsISA::PageBytes = ULL(1) << PageShift;
const Addr MipsISA::PageMask = ~(PageBytes - 1);
const Addr MipsISA::PageOffset = PageBytes - 1;

#if FULL_SYSTEM

////////////////////////////////////////////////////////////////////////
//
//  Translation stuff
//

const Addr MipsISA::PteShift = 3;
const Addr MipsISA::NPtePageShift = PageShift - PteShift;
const Addr MipsISA::NPtePage = ULL(1) << NPtePageShift;
const Addr MipsISA::PteMask = NPtePage - 1;

// User Virtual
const Addr MipsISA::USegBase = ULL(0x0);
const Addr MipsISA::USegEnd = ULL(0x000003ffffffffff);

// Kernel Direct Mapped
const Addr MipsISA::K0SegBase = ULL(0xfffffc0000000000);
const Addr MipsISA::K0SegEnd = ULL(0xfffffdffffffffff);

// Kernel Virtual
const Addr MipsISA::K1SegBase = ULL(0xfffffe0000000000);
const Addr MipsISA::K1SegEnd = ULL(0xffffffffffffffff);

#endif

// Mips UNOP (ldq_u r31,0(r0))
const MachInst MipsISA::NoopMachInst = 0x2ffe0000;

static inline Addr
TruncPage(Addr addr)
{ return addr & ~(MipsISA::PageBytes - 1); }

static inline Addr
RoundPage(Addr addr)
{ return (addr + MipsISA::PageBytes - 1) & ~(MipsISA::PageBytes - 1); }
void
RegFile::serialize(std::ostream &os)
{
    SERIALIZE_ARRAY(intRegFile, NumIntRegs);
    SERIALIZE_ARRAY(floatRegFile.q, NumFloatRegs);
    SERIALIZE_SCALAR(miscRegs.fpcr);
    SERIALIZE_SCALAR(miscRegs.uniq);
    SERIALIZE_SCALAR(miscRegs.lock_flag);
    SERIALIZE_SCALAR(miscRegs.lock_addr);
    SERIALIZE_SCALAR(pc);
    SERIALIZE_SCALAR(npc);
#if FULL_SYSTEM
    SERIALIZE_ARRAY(palregs, NumIntRegs);
    SERIALIZE_ARRAY(ipr, NumInternalProcRegs);
    SERIALIZE_SCALAR(intrflag);
    SERIALIZE_SCALAR(pal_shadow);
#endif
}


void
RegFile::unserialize(Checkpoint *cp, const std::string &section)
{
    UNSERIALIZE_ARRAY(intRegFile, NumIntRegs);
    UNSERIALIZE_ARRAY(floatRegFile.q, NumFloatRegs);
    UNSERIALIZE_SCALAR(miscRegs.fpcr);
    UNSERIALIZE_SCALAR(miscRegs.uniq);
    UNSERIALIZE_SCALAR(miscRegs.lock_flag);
    UNSERIALIZE_SCALAR(miscRegs.lock_addr);
    UNSERIALIZE_SCALAR(pc);
    UNSERIALIZE_SCALAR(npc);
#if FULL_SYSTEM
    UNSERIALIZE_ARRAY(palregs, NumIntRegs);
    UNSERIALIZE_ARRAY(ipr, NumInternalProcRegs);
    UNSERIALIZE_SCALAR(intrflag);
    UNSERIALIZE_SCALAR(pal_shadow);
#endif
}


#if FULL_SYSTEM
void
PTE::serialize(std::ostream &os)
{
    SERIALIZE_SCALAR(tag);
    SERIALIZE_SCALAR(ppn);
    SERIALIZE_SCALAR(xre);
    SERIALIZE_SCALAR(xwe);
    SERIALIZE_SCALAR(asn);
    SERIALIZE_SCALAR(asma);
    SERIALIZE_SCALAR(fonr);
    SERIALIZE_SCALAR(fonw);
    SERIALIZE_SCALAR(valid);
}


void
PTE::unserialize(Checkpoint *cp, const std::string &section)
{
    UNSERIALIZE_SCALAR(tag);
    UNSERIALIZE_SCALAR(ppn);
    UNSERIALIZE_SCALAR(xre);
    UNSERIALIZE_SCALAR(xwe);
    UNSERIALIZE_SCALAR(asn);
    UNSERIALIZE_SCALAR(asma);
    UNSERIALIZE_SCALAR(fonr);
    UNSERIALIZE_SCALAR(fonw);
    UNSERIALIZE_SCALAR(valid);
}

#endif //FULL_SYSTEM
