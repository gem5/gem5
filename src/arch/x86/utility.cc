/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * Copyright (c) 2011 Advanced Micro Devices, Inc.
 * All rights reserved.
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
 */

#include "arch/x86/utility.hh"

#include "arch/x86/interrupts.hh"
#include "arch/x86/registers.hh"
#include "arch/x86/x86_traits.hh"
#include "cpu/base.hh"
#include "fputils/fp80.h"
#include "sim/full_system.hh"

namespace X86ISA {

uint64_t
getArgument(ThreadContext *tc, int &number, uint16_t size, bool fp)
{
    if (fp) {
        panic("getArgument(): Floating point arguments not implemented\n");
    } else if (size != 8) {
        panic("getArgument(): Can only handle 64-bit arguments.\n");
    }

    // The first 6 integer arguments are passed in registers, the rest
    // are passed on the stack.
    const int int_reg_map[] = {
        INTREG_RDI, INTREG_RSI, INTREG_RDX,
        INTREG_RCX, INTREG_R8, INTREG_R9
    };
    if (number < sizeof(int_reg_map) / sizeof(*int_reg_map)) {
        return tc->readIntReg(int_reg_map[number]);
    } else {
        panic("getArgument(): Don't know how to handle stack arguments.\n");
    }
}

void
copyMiscRegs(ThreadContext *src, ThreadContext *dest)
{
    // This function assumes no side effects other than TLB invalidation
    // need to be considered while copying state. That will likely not be
    // true in the future.
    for (int i = 0; i < NUM_MISCREGS; ++i) {
        if (!isValidMiscReg(i))
             continue;

        dest->setMiscRegNoEffect(i, src->readMiscRegNoEffect(i));
    }

    // The TSC has to be updated with side-effects if the CPUs in a
    // CPU switch have different frequencies.
    dest->setMiscReg(MISCREG_TSC, src->readMiscReg(MISCREG_TSC));

    dest->getITBPtr()->flushAll();
    dest->getDTBPtr()->flushAll();
}

void
copyRegs(ThreadContext *src, ThreadContext *dest)
{
    //copy int regs
    for (int i = 0; i < NumIntRegs; ++i)
         dest->setIntRegFlat(i, src->readIntRegFlat(i));
    //copy float regs
    for (int i = 0; i < NumFloatRegs; ++i)
         dest->setFloatRegFlat(i, src->readFloatRegFlat(i));
    //copy condition-code regs
    for (int i = 0; i < NumCCRegs; ++i)
         dest->setCCRegFlat(i, src->readCCRegFlat(i));
    copyMiscRegs(src, dest);
    dest->pcState(src->pcState());
}

uint64_t
getRFlags(ThreadContext *tc)
{
    const uint64_t ncc_flags(tc->readMiscRegNoEffect(MISCREG_RFLAGS));
    const uint64_t cc_flags(tc->readCCReg(X86ISA::CCREG_ZAPS));
    const uint64_t cfof_bits(tc->readCCReg(X86ISA::CCREG_CFOF));
    const uint64_t df_bit(tc->readCCReg(X86ISA::CCREG_DF));
    // ecf (PSEUDO(3)) & ezf (PSEUDO(4)) are only visible to
    // microcode, so we can safely ignore them.

    // Reconstruct the real rflags state, mask out internal flags, and
    // make sure reserved bits have the expected values.
    return ((ncc_flags | cc_flags | cfof_bits | df_bit) & 0x3F7FD5)
        | 0x2;
}

void
setRFlags(ThreadContext *tc, uint64_t val)
{
    tc->setCCReg(X86ISA::CCREG_ZAPS, val & ccFlagMask);
    tc->setCCReg(X86ISA::CCREG_CFOF, val & cfofMask);
    tc->setCCReg(X86ISA::CCREG_DF, val & DFBit);

    // Internal microcode registers (ECF & EZF)
    tc->setCCReg(X86ISA::CCREG_ECF, 0);
    tc->setCCReg(X86ISA::CCREG_EZF, 0);

    // Update the RFLAGS misc reg with whatever didn't go into the
    // magic registers.
    tc->setMiscReg(MISCREG_RFLAGS, val & ~(ccFlagMask | cfofMask | DFBit));
}

uint8_t
convX87TagsToXTags(uint16_t ftw)
{
    uint8_t ftwx(0);
    for (int i = 0; i < 8; ++i) {
        // Extract the tag for the current element on the FP stack
        const unsigned tag((ftw >> (2 * i)) & 0x3);

        /*
         * Check the type of the current FP element. Valid values are:
         * 0 == Valid
         * 1 == Zero
         * 2 == Special (Nan, unsupported, infinity, denormal)
         * 3 == Empty
         */
        // The xsave version of the tag word only keeps track of
        // whether the element is empty or not. Set the corresponding
        // bit in the ftwx if it's not empty,
        if (tag != 0x3)
            ftwx |= 1 << i;
    }

    return ftwx;
}

uint16_t
convX87XTagsToTags(uint8_t ftwx)
{
    uint16_t ftw(0);
    for (int i = 0; i < 8; ++i) {
        const unsigned xtag(((ftwx >> i) & 0x1));

        // The xtag for an x87 stack position is 0 for empty stack positions.
        if (!xtag) {
            // Set the tag word to 3 (empty) for the current element.
            ftw |= 0x3 << (2 * i);
        } else {
            // TODO: We currently assume that non-empty elements are
            // valid (0x0), but we should ideally reconstruct the full
            // state (valid/zero/special).
        }
    }

    return ftw;
}

uint16_t
genX87Tags(uint16_t ftw, uint8_t top, int8_t spm)
{
    const uint8_t new_top((top + spm + 8) % 8);

    if (spm > 0) {
        // Removing elements from the stack. Flag the elements as empty.
        for (int i = top; i != new_top; i = (i + 1 + 8) % 8)
            ftw |= 0x3 << (2 * i);
    } else if (spm < 0) {
        // Adding elements to the stack. Flag the new elements as
        // valid. We should ideally decode them and "do the right
        // thing".
        for (int i = new_top; i != top; i = (i + 1 + 8) % 8)
            ftw &= ~(0x3 << (2 * i));
    }

    return ftw;
}

double
loadFloat80(const void *_mem)
{
    fp80_t fp80;
    memcpy(fp80.bits, _mem, 10);

    return fp80_cvtd(fp80);
}

void
storeFloat80(void *_mem, double value)
{
    fp80_t fp80 = fp80_cvfd(value);
    memcpy(_mem, fp80.bits, 10);
}

} // namespace X86_ISA
