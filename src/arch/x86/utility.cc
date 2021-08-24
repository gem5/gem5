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
#include "arch/x86/mmu.hh"
#include "arch/x86/regs/ccr.hh"
#include "arch/x86/regs/float.hh"
#include "arch/x86/regs/int.hh"
#include "arch/x86/regs/misc.hh"
#include "arch/x86/x86_traits.hh"
#include "cpu/base.hh"
#include "fputils/fp80.h"

namespace gem5
{

namespace X86ISA
{

uint64_t
getRFlags(ThreadContext *tc)
{
    const uint64_t ncc_flags(tc->readMiscRegNoEffect(misc_reg::Rflags));
    const uint64_t cc_flags(tc->getReg(X86ISA::cc_reg::Zaps));
    const uint64_t cfof_bits(tc->getReg(X86ISA::cc_reg::Cfof));
    const uint64_t df_bit(tc->getReg(X86ISA::cc_reg::Df));
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
    tc->setReg(X86ISA::cc_reg::Zaps, val & CcFlagMask);
    tc->setReg(X86ISA::cc_reg::Cfof, val & CfofMask);
    tc->setReg(X86ISA::cc_reg::Df, val & DFBit);

    // Internal microcode registers (ECF & EZF)
    tc->setReg(X86ISA::cc_reg::Ecf, (RegVal)0);
    tc->setReg(X86ISA::cc_reg::Ezf, (RegVal)0);

    // Update the RFLAGS misc reg with whatever didn't go into the
    // magic registers.
    tc->setMiscReg(misc_reg::Rflags, val & ~(CcFlagMask | CfofMask | DFBit));
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
} // namespace gem5
