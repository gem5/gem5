/*
 * Copyright (c) 2003-2007 The Regents of The University of Michigan
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

/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
 *
 * Redistribution and use of this software in source and binary forms,
 * with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * The software must be used only for Non-Commercial Use which means any
 * use which is NOT directed to receiving any direct monetary
 * compensation for, or commercial advantage from such use.  Illustrative
 * examples of non-commercial use are academic research, personal study,
 * teaching, education and corporate research & development.
 * Illustrative examples of commercial use are distributing products for
 * commercial advantage and providing services using the software for
 * commercial advantage.
 *
 * If you wish to use this software or functionality therein that may be
 * covered by patents for commercial use, please contact:
 *     Director of Intellectual Property Licensing
 *     Office of Strategy and Technology
 *     Hewlett-Packard Company
 *     1501 Page Mill Road
 *     Palo Alto, California  94304
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.  Redistributions
 * in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.  Neither the name of
 * the COPYRIGHT HOLDER(s), HEWLETT-PACKARD COMPANY, nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.  No right of
 * sublicense is granted herewith.  Derivatives of the software and
 * output created using the software may be prepared, but only for
 * Non-Commercial Uses.  Derivatives of the software may be shared with
 * others provided: (i) the others agree to abide by the list of
 * conditions herein which includes the Non-Commercial Use restrictions;
 * and (ii) such Derivatives of the software include the above copyright
 * notice to acknowledge the contribution from this software where
 * applicable, this list of conditions and the disclaimer below.
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

#include "arch/x86/decoder.hh"
#include "arch/x86/faults.hh"
#include "base/trace.hh"
#include "config/full_system.hh"
#include "cpu/thread_context.hh"
#if !FULL_SYSTEM
#include "arch/x86/isa_traits.hh"
#include "mem/page_table.hh"
#include "sim/process.hh"
#else
#include "arch/x86/tlb.hh"
#endif

namespace X86ISA
{
#if FULL_SYSTEM
    void X86FaultBase::invoke(ThreadContext * tc)
    {
        using namespace X86ISAInst::RomLabels;
        HandyM5Reg m5reg = tc->readMiscRegNoEffect(MISCREG_M5_REG);
        MicroPC entry;
        if (m5reg.mode == LongMode) {
            if (isSoft()) {
                entry = extern_label_longModeSoftInterrupt;
            } else {
                entry = extern_label_longModeInterrupt;
            }
        } else {
            entry = extern_label_legacyModeInterrupt;
        }
        tc->setIntReg(INTREG_MICRO(1), vector);
        tc->setIntReg(INTREG_MICRO(7), tc->readPC());
        if (errorCode != (uint64_t)(-1)) {
            if (m5reg.mode == LongMode) {
                entry = extern_label_longModeInterruptWithError;
            } else {
                panic("Legacy mode interrupts with error codes "
                        "aren't implementde.\n");
            }
            // Software interrupts shouldn't have error codes. If one does,
            // there would need to be microcode to set it up.
            assert(!isSoft());
            tc->setIntReg(INTREG_MICRO(15), errorCode);
        }
        tc->setMicroPC(romMicroPC(entry));
        tc->setNextMicroPC(romMicroPC(entry) + 1);
    }
    
    void X86Trap::invoke(ThreadContext * tc)
    {
        X86FaultBase::invoke(tc);
        // This is the same as a fault, but it happens -after- the instruction.
        tc->setPC(tc->readNextPC());
        tc->setNextPC(tc->readNextNPC());
        tc->setNextNPC(tc->readNextNPC() + sizeof(MachInst));
    }

    void X86Abort::invoke(ThreadContext * tc)
    {
        panic("Abort exception!");
    }

    void PageFault::invoke(ThreadContext * tc)
    {
        HandyM5Reg m5reg = tc->readMiscRegNoEffect(MISCREG_M5_REG);
        X86FaultBase::invoke(tc);
        /*
         * If something bad happens while trying to enter the page fault
         * handler, I'm pretty sure that's a double fault and then all bets are
         * off. That means it should be safe to update this state now.
         */
        if (m5reg.mode == LongMode) {
            tc->setMiscReg(MISCREG_CR2, addr);
        } else {
            tc->setMiscReg(MISCREG_CR2, (uint32_t)addr);
        }
    }

#endif
} // namespace X86ISA

