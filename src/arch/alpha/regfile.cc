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

#include "arch/alpha/regfile.hh"
#include "cpu/thread_context.hh"

namespace AlphaISA
{
    void
    RegFile::serialize(std::ostream &os)
    {
        intRegFile.serialize(os);
        floatRegFile.serialize(os);
        miscRegFile.serialize(os);
        SERIALIZE_SCALAR(pc);
        SERIALIZE_SCALAR(npc);
#if FULL_SYSTEM
        SERIALIZE_SCALAR(intrflag);
#endif
    }

    void
    RegFile::unserialize(Checkpoint *cp, const std::string &section)
    {
        intRegFile.unserialize(cp, section);
        floatRegFile.unserialize(cp, section);
        miscRegFile.unserialize(cp, section);
        UNSERIALIZE_SCALAR(pc);
        UNSERIALIZE_SCALAR(npc);
#if FULL_SYSTEM
        UNSERIALIZE_SCALAR(intrflag);
#endif
    }

    void
    copyRegs(ThreadContext *src, ThreadContext *dest)
    {
        // First loop through the integer registers.
        for (int i = 0; i < NumIntRegs; ++i) {
            dest->setIntReg(i, src->readIntReg(i));
        }

        // Then loop through the floating point registers.
        for (int i = 0; i < TheISA::NumFloatRegs; ++i) {
            dest->setFloatRegBits(i, src->readFloatRegBits(i));
        }

        // Copy misc. registers
        copyMiscRegs(src, dest);

        // Lastly copy PC/NPC
        dest->setPC(src->readPC());
        dest->setNextPC(src->readNextPC());
    }

    void
    copyMiscRegs(ThreadContext *src, ThreadContext *dest)
    {
        dest->setMiscRegNoEffect(AlphaISA::MISCREG_FPCR,
                src->readMiscRegNoEffect(AlphaISA::MISCREG_FPCR));
        dest->setMiscRegNoEffect(AlphaISA::MISCREG_UNIQ,
                src->readMiscRegNoEffect(AlphaISA::MISCREG_UNIQ));
        dest->setMiscRegNoEffect(AlphaISA::MISCREG_LOCKFLAG,
                src->readMiscRegNoEffect(AlphaISA::MISCREG_LOCKFLAG));
        dest->setMiscRegNoEffect(AlphaISA::MISCREG_LOCKADDR,
                src->readMiscRegNoEffect(AlphaISA::MISCREG_LOCKADDR));

#if FULL_SYSTEM
        copyIprs(src, dest);
#endif
    }
}
