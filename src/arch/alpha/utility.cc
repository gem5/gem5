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
 * Authors: Nathan Binkert
 *          Ali Saidi
 */

#include "arch/alpha/ev5.hh"
#include "arch/alpha/utility.hh"

#if FULL_SYSTEM
#include "arch/alpha/vtophys.hh"
#include "mem/vport.hh"
#endif

namespace AlphaISA {

uint64_t
getArgument(ThreadContext *tc, int &number, uint8_t size, bool fp)
{
#if FULL_SYSTEM
    const int NumArgumentRegs = 6;
    if (number < NumArgumentRegs) {
        if (fp)
            return tc->readFloatRegBits(16 + number);
        else
            return tc->readIntReg(16 + number);
    } else {
        Addr sp = tc->readIntReg(StackPointerReg);
        VirtualPort *vp = tc->getVirtPort();
        uint64_t arg = vp->read<uint64_t>(sp +
                           (number-NumArgumentRegs) * sizeof(uint64_t));
        return arg;
    }
#else
    panic("getArgument() is Full system only\n");
    M5_DUMMY_RETURN;
#endif
}

void
copyRegs(ThreadContext *src, ThreadContext *dest)
{
    // First loop through the integer registers.
    for (int i = 0; i < NumIntRegs; ++i)
        dest->setIntReg(i, src->readIntReg(i));

    // Then loop through the floating point registers.
    for (int i = 0; i < NumFloatRegs; ++i)
        dest->setFloatRegBits(i, src->readFloatRegBits(i));

    // Copy misc. registers
    copyMiscRegs(src, dest);

    // Lastly copy PC/NPC
    dest->setPC(src->readPC());
    dest->setNextPC(src->readNextPC());
}

void
copyMiscRegs(ThreadContext *src, ThreadContext *dest)
{
    dest->setMiscRegNoEffect(MISCREG_FPCR,
        src->readMiscRegNoEffect(MISCREG_FPCR));
    dest->setMiscRegNoEffect(MISCREG_UNIQ,
        src->readMiscRegNoEffect(MISCREG_UNIQ));
    dest->setMiscRegNoEffect(MISCREG_LOCKFLAG,
        src->readMiscRegNoEffect(MISCREG_LOCKFLAG));
    dest->setMiscRegNoEffect(MISCREG_LOCKADDR,
        src->readMiscRegNoEffect(MISCREG_LOCKADDR));

    copyIprs(src, dest);
}

void
skipFunction(ThreadContext *tc)
{
    Addr newpc = tc->readIntReg(ReturnAddressReg);
    tc->setPC(newpc);
    tc->setNextPC(tc->readPC() + sizeof(TheISA::MachInst));
}


} // namespace AlphaISA

