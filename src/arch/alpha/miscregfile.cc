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
 *
 * Authors: Steve Reinhardt
 *          Gabe Black
 *          Kevin Lim
 */

#include "arch/alpha/miscregfile.hh"
#include "base/misc.hh"

namespace AlphaISA
{

    void
    MiscRegFile::serialize(std::ostream &os)
    {
        SERIALIZE_SCALAR(fpcr);
        SERIALIZE_SCALAR(uniq);
        SERIALIZE_SCALAR(lock_flag);
        SERIALIZE_SCALAR(lock_addr);
#if FULL_SYSTEM
        SERIALIZE_ARRAY(ipr, NumInternalProcRegs);
#endif
    }

    void
    MiscRegFile::unserialize(Checkpoint *cp, const std::string &section)
    {
        UNSERIALIZE_SCALAR(fpcr);
        UNSERIALIZE_SCALAR(uniq);
        UNSERIALIZE_SCALAR(lock_flag);
        UNSERIALIZE_SCALAR(lock_addr);
#if FULL_SYSTEM
        UNSERIALIZE_ARRAY(ipr, NumInternalProcRegs);
#endif
    }

    MiscReg
    MiscRegFile::readReg(int misc_reg)
    {
        switch(misc_reg) {
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
#if FULL_SYSTEM
          default:
            assert(misc_reg < NumInternalProcRegs);
            return ipr[misc_reg];
#else
          default:
            panic("Attempt to read an invalid misc register!");
            return 0;
#endif
        }
    }

    MiscReg
    MiscRegFile::readRegWithEffect(int misc_reg, ThreadContext *tc)
    {
#if FULL_SYSTEM
        return readIpr(misc_reg, tc);
#else
        panic("No faulting misc regs in SE mode!");
        return 0;
#endif
    }

    void
    MiscRegFile::setReg(int misc_reg, const MiscReg &val)
    {
        switch(misc_reg) {
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
#if FULL_SYSTEM
          default:
            assert(misc_reg < NumInternalProcRegs);
            ipr[misc_reg] = val;
            return;
#else
          default:
            panic("Attempt to write to an invalid misc register!");
#endif
        }
    }

    void
    MiscRegFile::setRegWithEffect(int misc_reg, const MiscReg &val,
            ThreadContext *tc)
    {
#if FULL_SYSTEM
        switch(misc_reg) {
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
            return setIpr(misc_reg, val, tc);
        }
#else
        //panic("No registers with side effects in SE mode!");
        return;
#endif
    }

}
