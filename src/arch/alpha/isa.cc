/*
 * Copyright (c) 2009 The Regents of The University of Michigan
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

#include "arch/alpha/isa.hh"

#include <cassert>

#include "base/logging.hh"
#include "cpu/thread_context.hh"
#include "params/AlphaISA.hh"
#include "sim/serialize.hh"

namespace AlphaISA
{

ISA::ISA(Params *p)
    : SimObject(p), system(p->system)
{
    clear();
    initializeIprTable();
}

const AlphaISAParams *
ISA::params() const
{
    return dynamic_cast<const Params *>(_params);
}

void
ISA::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(fpcr);
    SERIALIZE_SCALAR(uniq);
    SERIALIZE_SCALAR(lock_flag);
    SERIALIZE_SCALAR(lock_addr);
    SERIALIZE_ARRAY(ipr, NumInternalProcRegs);
}

void
ISA::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(fpcr);
    UNSERIALIZE_SCALAR(uniq);
    UNSERIALIZE_SCALAR(lock_flag);
    UNSERIALIZE_SCALAR(lock_addr);
    UNSERIALIZE_ARRAY(ipr, NumInternalProcRegs);
}


RegVal
ISA::readMiscRegNoEffect(int misc_reg, ThreadID tid) const
{
    switch (misc_reg) {
      case MISCREG_FPCR:
        return fpcr;
      case MISCREG_UNIQ:
        return uniq;
      case MISCREG_LOCKFLAG:
        return lock_flag;
      case MISCREG_LOCKADDR:
        return lock_addr;
      case MISCREG_INTR:
        return intr_flag;
      default:
        assert(misc_reg < NumInternalProcRegs);
        return ipr[misc_reg];
    }
}

RegVal
ISA::readMiscReg(int misc_reg, ThreadContext *tc, ThreadID tid)
{
    switch (misc_reg) {
      case MISCREG_FPCR:
        return fpcr;
      case MISCREG_UNIQ:
        return uniq;
      case MISCREG_LOCKFLAG:
        return lock_flag;
      case MISCREG_LOCKADDR:
        return lock_addr;
      case MISCREG_INTR:
        return intr_flag;
      default:
        return readIpr(misc_reg, tc);
    }
}

void
ISA::setMiscRegNoEffect(int misc_reg, RegVal val, ThreadID tid)
{
    switch (misc_reg) {
      case MISCREG_FPCR:
        fpcr = val;
        return;
      case MISCREG_UNIQ:
        uniq = val;
        return;
      case MISCREG_LOCKFLAG:
        lock_flag = val;
        return;
      case MISCREG_LOCKADDR:
        lock_addr = val;
        return;
      case MISCREG_INTR:
        intr_flag = val;
        return;
      default:
        assert(misc_reg < NumInternalProcRegs);
        ipr[misc_reg] = val;
        return;
    }
}

void
ISA::setMiscReg(int misc_reg, RegVal val, ThreadContext *tc, ThreadID tid)
{
    switch (misc_reg) {
      case MISCREG_FPCR:
        fpcr = val;
        return;
      case MISCREG_UNIQ:
        uniq = val;
        return;
      case MISCREG_LOCKFLAG:
        lock_flag = val;
        return;
      case MISCREG_LOCKADDR:
        lock_addr = val;
        return;
      case MISCREG_INTR:
        intr_flag = val;
        return;
      default:
        setIpr(misc_reg, val, tc);
        return;
    }
}

}

AlphaISA::ISA *
AlphaISAParams::create()
{
    return new AlphaISA::ISA(this);
}
