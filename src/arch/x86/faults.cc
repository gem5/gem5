/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
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

#include "arch/x86/generated/decoder.hh"
#include "arch/x86/faults.hh"
#include "arch/x86/isa_traits.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "debug/Faults.hh"
#include "sim/full_system.hh"

namespace X86ISA
{
    void X86FaultBase::invoke(ThreadContext * tc, StaticInstPtr inst)
    {
        if (!FullSystem) {
            FaultBase::invoke(tc, inst);
            return;
        }

        PCState pcState = tc->pcState();
        Addr pc = pcState.pc();
        DPRINTF(Faults, "RIP %#x: vector %d: %s\n",
                pc, vector, describe());
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
        tc->setIntReg(INTREG_MICRO(7), pc);
        if (errorCode != (uint64_t)(-1)) {
            if (m5reg.mode == LongMode) {
                entry = extern_label_longModeInterruptWithError;
            } else {
                panic("Legacy mode interrupts with error codes "
                        "aren't implementde.\n");
            }
            // Software interrupts shouldn't have error codes. If one
            // does, there would need to be microcode to set it up.
            assert(!isSoft());
            tc->setIntReg(INTREG_MICRO(15), errorCode);
        }
        pcState.upc(romMicroPC(entry));
        pcState.nupc(romMicroPC(entry) + 1);
        tc->pcState(pcState);
    }

    std::string
    X86FaultBase::describe() const
    {
        std::stringstream ss;
        ccprintf(ss, "%s", mnemonic());
        if (errorCode != (uint64_t)(-1)) {
            ccprintf(ss, "(%#x)", errorCode);
        }

        return ss.str();
    }
    
    void X86Trap::invoke(ThreadContext * tc, StaticInstPtr inst)
    {
        X86FaultBase::invoke(tc);
        if (!FullSystem)
            return;

        // This is the same as a fault, but it happens -after- the
        // instruction.
        PCState pc = tc->pcState();
        pc.uEnd();
    }

    void X86Abort::invoke(ThreadContext * tc, StaticInstPtr inst)
    {
        panic("Abort exception!");
    }

    void
    InvalidOpcode::invoke(ThreadContext * tc, StaticInstPtr inst)
    {
        if (FullSystem) {
            X86Fault::invoke(tc, inst);
        } else {
            panic("Unrecognized/invalid instruction executed:\n %s",
                    inst->machInst);
        }
    }

    void PageFault::invoke(ThreadContext * tc, StaticInstPtr inst)
    {
        if (FullSystem) {
            HandyM5Reg m5reg = tc->readMiscRegNoEffect(MISCREG_M5_REG);
            X86FaultBase::invoke(tc);
            /*
             * If something bad happens while trying to enter the page fault
             * handler, I'm pretty sure that's a double fault and then all
             * bets are off. That means it should be safe to update this
             * state now.
             */
            if (m5reg.mode == LongMode) {
                tc->setMiscReg(MISCREG_CR2, addr);
            } else {
                tc->setMiscReg(MISCREG_CR2, (uint32_t)addr);
            }
        } else {
            PageFaultErrorCode code = errorCode;
            const char *modeStr = "";
            if (code.fetch)
                modeStr = "execute";
            else if (code.write)
                modeStr = "write";
            else
                modeStr = "read";
            panic("Tried to %s unmapped address %#x.\n", modeStr, addr);
        }
    }

    std::string
    PageFault::describe() const
    {
        std::stringstream ss;
        ccprintf(ss, "%s at %#x", X86FaultBase::describe(), addr);
        return ss.str();
    }

    void
    InitInterrupt::invoke(ThreadContext *tc, StaticInstPtr inst)
    {
        DPRINTF(Faults, "Init interrupt.\n");
        // The otherwise unmodified integer registers should be set to 0.
        for (int index = 0; index < NUM_INTREGS; index++) {
            tc->setIntReg(index, 0);
        }

        CR0 cr0 = tc->readMiscReg(MISCREG_CR0);
        CR0 newCR0 = 1 << 4;
        newCR0.cd = cr0.cd;
        newCR0.nw = cr0.nw;
        tc->setMiscReg(MISCREG_CR0, newCR0);
        tc->setMiscReg(MISCREG_CR2, 0);
        tc->setMiscReg(MISCREG_CR3, 0);
        tc->setMiscReg(MISCREG_CR4, 0);

        tc->setMiscReg(MISCREG_RFLAGS, 0x0000000000000002ULL);

        tc->setMiscReg(MISCREG_EFER, 0);

        SegAttr dataAttr = 0;
        dataAttr.dpl = 0;
        dataAttr.unusable = 0;
        dataAttr.defaultSize = 0;
        dataAttr.longMode = 0;
        dataAttr.avl = 0;
        dataAttr.granularity = 0;
        dataAttr.present = 1;
        dataAttr.type = 3;
        dataAttr.writable = 1;
        dataAttr.readable = 1;
        dataAttr.expandDown = 0;
        dataAttr.system = 1;

        for (int seg = 0; seg != NUM_SEGMENTREGS; seg++) {
            tc->setMiscReg(MISCREG_SEG_SEL(seg), 0);
            tc->setMiscReg(MISCREG_SEG_BASE(seg), 0);
            tc->setMiscReg(MISCREG_SEG_EFF_BASE(seg), 0);
            tc->setMiscReg(MISCREG_SEG_LIMIT(seg), 0xffff);
            tc->setMiscReg(MISCREG_SEG_ATTR(seg), dataAttr);
        }

        SegAttr codeAttr = 0;
        codeAttr.dpl = 0;
        codeAttr.unusable = 0;
        codeAttr.defaultSize = 0;
        codeAttr.longMode = 0;
        codeAttr.avl = 0;
        codeAttr.granularity = 0;
        codeAttr.present = 1;
        codeAttr.type = 10;
        codeAttr.writable = 0;
        codeAttr.readable = 1;
        codeAttr.expandDown = 0;
        codeAttr.system = 1;

        tc->setMiscReg(MISCREG_CS, 0xf000);
        tc->setMiscReg(MISCREG_CS_BASE,
                0x00000000ffff0000ULL);
        tc->setMiscReg(MISCREG_CS_EFF_BASE,
                0x00000000ffff0000ULL);
        // This has the base value pre-added.
        tc->setMiscReg(MISCREG_CS_LIMIT, 0xffffffff);
        tc->setMiscReg(MISCREG_CS_ATTR, codeAttr);

        PCState pc(0x000000000000fff0ULL + tc->readMiscReg(MISCREG_CS_BASE));
        tc->pcState(pc);

        tc->setMiscReg(MISCREG_TSG_BASE, 0);
        tc->setMiscReg(MISCREG_TSG_LIMIT, 0xffff);

        tc->setMiscReg(MISCREG_IDTR_BASE, 0);
        tc->setMiscReg(MISCREG_IDTR_LIMIT, 0xffff);

        SegAttr tslAttr = 0;
        tslAttr.present = 1;
        tslAttr.type = 2; // LDT
        tc->setMiscReg(MISCREG_TSL, 0);
        tc->setMiscReg(MISCREG_TSL_BASE, 0);
        tc->setMiscReg(MISCREG_TSL_LIMIT, 0xffff);
        tc->setMiscReg(MISCREG_TSL_ATTR, tslAttr);

        SegAttr trAttr = 0;
        trAttr.present = 1;
        trAttr.type = 3; // Busy 16-bit TSS
        tc->setMiscReg(MISCREG_TR, 0);
        tc->setMiscReg(MISCREG_TR_BASE, 0);
        tc->setMiscReg(MISCREG_TR_LIMIT, 0xffff);
        tc->setMiscReg(MISCREG_TR_ATTR, trAttr);

        // This value should be the family/model/stepping of the processor.
        // (page 418). It should be consistent with the value from CPUID, but
        // the actual value probably doesn't matter much.
        tc->setIntReg(INTREG_RDX, 0);

        tc->setMiscReg(MISCREG_DR0, 0);
        tc->setMiscReg(MISCREG_DR1, 0);
        tc->setMiscReg(MISCREG_DR2, 0);
        tc->setMiscReg(MISCREG_DR3, 0);

        tc->setMiscReg(MISCREG_DR6, 0x00000000ffff0ff0ULL);
        tc->setMiscReg(MISCREG_DR7, 0x0000000000000400ULL);

        tc->setMiscReg(MISCREG_MXCSR, 0x1f80);

        // Flag all elements on the x87 stack as empty.
        tc->setMiscReg(MISCREG_FTW, 0xFFFF);

        // Update the handy M5 Reg.
        tc->setMiscReg(MISCREG_M5_REG, 0);
        MicroPC entry = X86ISAInst::RomLabels::extern_label_initIntHalt;
        pc.upc(romMicroPC(entry));
        pc.nupc(romMicroPC(entry) + 1);
        tc->pcState(pc);
    }

    void
    StartupInterrupt::invoke(ThreadContext *tc, StaticInstPtr inst)
    {
        DPRINTF(Faults, "Startup interrupt with vector %#x.\n", vector);
        HandyM5Reg m5Reg = tc->readMiscReg(MISCREG_M5_REG);
        if (m5Reg.mode != LegacyMode || m5Reg.submode != RealMode) {
            panic("Startup IPI recived outside of real mode. "
                    "Don't know what to do. %d, %d", m5Reg.mode, m5Reg.submode);
        }

        tc->setMiscReg(MISCREG_CS, vector << 8);
        tc->setMiscReg(MISCREG_CS_BASE, vector << 12);
        tc->setMiscReg(MISCREG_CS_EFF_BASE, vector << 12);
        // This has the base value pre-added.
        tc->setMiscReg(MISCREG_CS_LIMIT, 0xffff);

        tc->pcState(tc->readMiscReg(MISCREG_CS_BASE));
    }
} // namespace X86ISA

