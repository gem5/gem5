/*
 * Copyright (c) 2017-2018 ARM Limited
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
 * Authors: Giacomo Travaglini
 */

#include "arch/arm/tracers/tarmac_base.hh"

#include <algorithm>
#include <string>

#include "config/the_isa.hh"
#include "cpu/reg_class.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"

using namespace TheISA;

namespace Trace {

TarmacBaseRecord::TarmacBaseRecord(Tick _when, ThreadContext *_thread,
                                   const StaticInstPtr _staticInst,
                                   PCState _pc,
                                   const StaticInstPtr _macroStaticInst)
    : InstRecord(_when, _thread, _staticInst, _pc, _macroStaticInst)
{
}

TarmacBaseRecord::InstEntry::InstEntry(
    ThreadContext* thread,
    PCState pc,
    const StaticInstPtr staticInst,
    bool predicate)
        : taken(predicate) ,
          addr(pc.instAddr()) ,
          opcode(staticInst->machInst & 0xffffffff),
          disassemble(staticInst->disassemble(addr)),
          isetstate(pcToISetState(pc)),
          mode(MODE_USER)
{

    // Operating mode gained by reading the architectural register (CPSR)
    const CPSR cpsr = thread->readMiscRegNoEffect(MISCREG_CPSR);
    mode = (OperatingMode) (uint8_t)cpsr.mode;

    // In Tarmac, instruction names are printed in capital
    // letters.
    std::for_each(disassemble.begin(), disassemble.end(),
                  [](char& c) { c = toupper(c); });
}

TarmacBaseRecord::RegEntry::RegEntry(PCState pc)
  : isetstate(pcToISetState(pc))
{
}

TarmacBaseRecord::MemEntry::MemEntry (
    uint8_t _size,
    Addr _addr,
    uint64_t _data)
      : size(_size), addr(_addr), data(_data)
{
}

TarmacBaseRecord::ISetState
TarmacBaseRecord::pcToISetState(PCState pc)
{
    TarmacBaseRecord::ISetState isetstate;

    if (pc.aarch64())
        isetstate = TarmacBaseRecord::ISET_A64;
    else if (!pc.thumb() && !pc.jazelle())
        isetstate = TarmacBaseRecord::ISET_ARM;
    else if (pc.thumb() && !pc.jazelle())
        isetstate = TarmacBaseRecord::ISET_THUMB;
    else
        // No Jazelle state in TARMAC
        isetstate = TarmacBaseRecord::ISET_UNSUPPORTED;

    return isetstate;
}

} // namespace Trace
