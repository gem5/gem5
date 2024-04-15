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
 */

#include "arch/x86/faults.hh"

#include "arch/x86/generated/decoder.hh"
#include "arch/x86/insts/static_inst.hh"
#include "arch/x86/mmu.hh"
#include "arch/x86/regs/int.hh"
#include "arch/x86/regs/misc.hh"
#include "base/loader/symtab.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "debug/Faults.hh"
#include "sim/full_system.hh"
#include "sim/process.hh"

namespace gem5
{

namespace X86ISA
{

void
X86FaultBase::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    if (!FullSystem) {
        FaultBase::invoke(tc, inst);
        return;
    }

    PCState pc = tc->pcState().as<PCState>();
    DPRINTF(Faults, "RIP %#x: vector %d: %s\n", pc.pc(), vector, describe());
    using namespace X86ISAInst::rom_labels;
    HandyM5Reg m5reg = tc->readMiscRegNoEffect(misc_reg::M5Reg);
    MicroPC entry;
    if (m5reg.mode == LongMode) {
        entry = extern_label_longModeInterrupt;
    } else {
        if (m5reg.submode == RealMode)
            entry = extern_label_realModeInterrupt;
        else
            entry = extern_label_legacyModeInterrupt;
    }
    tc->setReg(intRegMicro(1), vector);
    Addr cs_base = tc->readMiscRegNoEffect(misc_reg::CsEffBase);
    tc->setReg(intRegMicro(7), pc.pc() - cs_base);
    if (errorCode != (uint64_t)(-1)) {
        if (m5reg.mode == LongMode) {
            entry = extern_label_longModeInterruptWithError;
        } else {
            panic("Legacy mode interrupts with error codes "
                  "aren't implemented.");
        }
        tc->setReg(intRegMicro(15), errorCode);
    }
    pc.upc(romMicroPC(entry));
    pc.nupc(romMicroPC(entry) + 1);
    tc->pcState(pc);
}

std::string
X86FaultBase::describe() const
{
    std::stringstream ss;
    ccprintf(ss, "%s", mnemonic());
    if (errorCode != (uint64_t)(-1))
        ccprintf(ss, "(%#x)", errorCode);

    return ss.str();
}

void
X86Trap::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    // This is the same as a fault, but it happens -after- the
    // instruction.
    X86FaultBase::invoke(tc);
}

void
X86Abort::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    panic("Abort exception!");
}

void
InvalidOpcode::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    if (FullSystem) {
        X86Fault::invoke(tc, inst);
    } else {
        auto *xsi = static_cast<X86StaticInst *>(inst.get());
        panic("Unrecognized/invalid instruction executed:\n %s",
              xsi->machInst);
    }
}

void
PageFault::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    if (FullSystem) {
        // Invalidate any matching TLB entries before handling the page fault.
        tc->getMMUPtr()->demapPage(addr, 0);
        HandyM5Reg m5reg = tc->readMiscRegNoEffect(misc_reg::M5Reg);
        X86FaultBase::invoke(tc);
        // If something bad happens while trying to enter the page fault
        // handler, I'm pretty sure that's a double fault and then all
        // bets are off. That means it should be safe to update this
        // state now.
        if (m5reg.mode == LongMode)
            tc->setMiscReg(misc_reg::Cr2, addr);
        else
            tc->setMiscReg(misc_reg::Cr2, (uint32_t)addr);
    } else if (!tc->getProcessPtr()->fixupFault(addr)) {
        PageFaultErrorCode code = errorCode;
        const char *modeStr = "";
        if (code.fetch)
            modeStr = "execute";
        else if (code.write)
            modeStr = "write";
        else
            modeStr = "read";

        // print information about what we are panic'ing on
        if (!inst) {
            panic("Tried to %s unmapped address %#x.", modeStr, addr);
        } else {
            panic("Tried to %s unmapped address %#x.\nPC: %#x, Instr: %s",
                  modeStr, addr, tc->pcState(),
                  inst->disassemble(tc->pcState().instAddr(),
                                    &loader::debugSymbolTable));
        }
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
InitInterrupt::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    DPRINTF(Faults, "Init interrupt.\n");
    // The otherwise unmodified integer registers should be set to 0.
    for (int index = 0; index < int_reg::NumArchRegs; index++) {
        tc->setReg(intRegClass[index], (RegVal)0);
    }

    CR0 cr0 = tc->readMiscReg(misc_reg::Cr0);
    CR0 newCR0 = 1 << 4;
    newCR0.cd = cr0.cd;
    newCR0.nw = cr0.nw;
    tc->setMiscReg(misc_reg::Cr0, newCR0);
    tc->setMiscReg(misc_reg::Cr2, 0);
    tc->setMiscReg(misc_reg::Cr3, 0);
    tc->setMiscReg(misc_reg::Cr4, 0);

    tc->setMiscReg(misc_reg::Rflags, 0x0000000000000002ULL);

    tc->setMiscReg(misc_reg::Efer, 0);

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

    for (int seg = 0; seg != segment_idx::NumIdxs; seg++) {
        tc->setMiscReg(misc_reg::segSel(seg), 0);
        tc->setMiscReg(misc_reg::segBase(seg), 0);
        tc->setMiscReg(misc_reg::segEffBase(seg), 0);
        tc->setMiscReg(misc_reg::segLimit(seg), 0xffff);
        tc->setMiscReg(misc_reg::segAttr(seg), dataAttr);
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

    tc->setMiscReg(misc_reg::Cs, 0xf000);
    tc->setMiscReg(misc_reg::CsBase, 0x00000000ffff0000ULL);
    tc->setMiscReg(misc_reg::CsEffBase, 0x00000000ffff0000ULL);
    // This has the base value pre-added.
    tc->setMiscReg(misc_reg::CsLimit, 0xffffffff);
    tc->setMiscReg(misc_reg::CsAttr, codeAttr);

    PCState pc(0x000000000000fff0ULL + tc->readMiscReg(misc_reg::CsBase));
    tc->pcState(pc);

    tc->setMiscReg(misc_reg::TsgBase, 0);
    tc->setMiscReg(misc_reg::TsgLimit, 0xffff);

    tc->setMiscReg(misc_reg::IdtrBase, 0);
    tc->setMiscReg(misc_reg::IdtrLimit, 0xffff);

    SegAttr tslAttr = 0;
    tslAttr.unusable = 1;
    tslAttr.present = 1;
    tslAttr.type = 2; // LDT
    tc->setMiscReg(misc_reg::Tsl, 0);
    tc->setMiscReg(misc_reg::TslBase, 0);
    tc->setMiscReg(misc_reg::TslLimit, 0xffff);
    tc->setMiscReg(misc_reg::TslAttr, tslAttr);

    SegAttr trAttr = 0;
    trAttr.unusable = 0;
    trAttr.present = 1;
    trAttr.type = 3; // Busy 16-bit TSS
    tc->setMiscReg(misc_reg::Tr, 0);
    tc->setMiscReg(misc_reg::TrBase, 0);
    tc->setMiscReg(misc_reg::TrLimit, 0xffff);
    tc->setMiscReg(misc_reg::TrAttr, trAttr);

    // This value should be the family/model/stepping of the processor.
    // (page 418). It should be consistent with the value from CPUID, but
    // the actual value probably doesn't matter much.
    tc->setReg(int_reg::Rdx, (RegVal)0);

    tc->setMiscReg(misc_reg::Dr0, 0);
    tc->setMiscReg(misc_reg::Dr1, 0);
    tc->setMiscReg(misc_reg::Dr2, 0);
    tc->setMiscReg(misc_reg::Dr3, 0);

    tc->setMiscReg(misc_reg::Dr6, 0x00000000ffff0ff0ULL);
    tc->setMiscReg(misc_reg::Dr7, 0x0000000000000400ULL);

    tc->setMiscReg(misc_reg::Mxcsr, 0x1f80);

    // Flag all elements on the x87 stack as empty.
    tc->setMiscReg(misc_reg::Ftw, 0xFFFF);

    // Update the handy M5 Reg.
    tc->setMiscReg(misc_reg::M5Reg, 0);
    MicroPC entry = X86ISAInst::rom_labels::extern_label_initIntHalt;
    pc.upc(romMicroPC(entry));
    pc.nupc(romMicroPC(entry) + 1);
    tc->pcState(pc);
}

void
StartupInterrupt::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    DPRINTF(Faults, "Startup interrupt with vector %#x.\n", vector);
    HandyM5Reg m5Reg = tc->readMiscReg(misc_reg::M5Reg);
    if (m5Reg.mode != LegacyMode || m5Reg.submode != RealMode) {
        panic("Startup IPI recived outside of real mode. "
              "Don't know what to do. %d, %d",
              m5Reg.mode, m5Reg.submode);
    }

    tc->setMiscReg(misc_reg::Cs, vector << 8);
    tc->setMiscReg(misc_reg::CsBase, vector << 12);
    tc->setMiscReg(misc_reg::CsEffBase, vector << 12);
    // This has the base value pre-added.
    tc->setMiscReg(misc_reg::CsLimit, 0xffff);

    tc->pcState(tc->readMiscReg(misc_reg::CsBase));
}

} // namespace X86ISA
} // namespace gem5
