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
 */

#include "arch/x86/isa.hh"

#include "arch/x86/decoder.hh"
#include "arch/x86/mmu.hh"
#include "arch/x86/regs/ccr.hh"
#include "arch/x86/regs/float.hh"
#include "arch/x86/regs/int.hh"
#include "arch/x86/regs/misc.hh"
#include "base/compiler.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/MatRegs.hh"
#include "params/X86ISA.hh"
#include "sim/serialize.hh"

namespace gem5
{

namespace X86ISA
{

void
ISA::updateHandyM5Reg(Efer efer, CR0 cr0,
                      SegAttr csAttr, SegAttr ssAttr, RFLAGS rflags)
{
    HandyM5Reg m5reg = 0;
    if (efer.lma) {
        m5reg.mode = LongMode;
        if (csAttr.longMode)
            m5reg.submode = SixtyFourBitMode;
        else
            m5reg.submode = CompatabilityMode;
    } else {
        m5reg.mode = LegacyMode;
        if (cr0.pe) {
            if (rflags.vm)
                m5reg.submode = Virtual8086Mode;
            else
                m5reg.submode = ProtectedMode;
        } else {
            m5reg.submode = RealMode;
        }
    }
    m5reg.cpl = csAttr.dpl;
    m5reg.paging = cr0.pg;
    m5reg.prot = cr0.pe;

    // Compute the default and alternate operand size.
    if (m5reg.submode == SixtyFourBitMode || csAttr.defaultSize) {
        m5reg.defOp = 2;
        m5reg.altOp = 1;
    } else {
        m5reg.defOp = 1;
        m5reg.altOp = 2;
    }

    // Compute the default and alternate address size.
    if (m5reg.submode == SixtyFourBitMode) {
        m5reg.defAddr = 3;
        m5reg.altAddr = 2;
    } else if (csAttr.defaultSize) {
        m5reg.defAddr = 2;
        m5reg.altAddr = 1;
    } else {
        m5reg.defAddr = 1;
        m5reg.altAddr = 2;
    }

    // Compute the stack size
    if (m5reg.submode == SixtyFourBitMode) {
        m5reg.stack = 3;
    } else if (ssAttr.defaultSize) {
        m5reg.stack = 2;
    } else {
        m5reg.stack = 1;
    }

    regVal[misc_reg::M5Reg] = m5reg;
    if (tc)
        tc->getDecoderPtr()->as<Decoder>().setM5Reg(m5reg);
}

void
ISA::clear()
{
    // Blank everything. 0 might not be an appropriate value for some things,
    // but it is for most.
    memset(regVal, 0, misc_reg::NumRegs * sizeof(RegVal));

    // If some state should be non-zero after a reset, set those values here.
    regVal[misc_reg::Cr0] = 0x0000000060000010ULL;

    regVal[misc_reg::Mtrrcap] = 0x0508;

    regVal[misc_reg::McgCap] = 0x104;

    regVal[misc_reg::Pat] = 0x0007040600070406ULL;

    regVal[misc_reg::Syscfg] = 0x20601;

    regVal[misc_reg::TopMem] = 0x4000000;

    regVal[misc_reg::Dr6] = (mask(8) << 4) | (mask(16) << 16);
    regVal[misc_reg::Dr7] = 1 << 10;

    LocalApicBase lApicBase = 0;
    lApicBase.base = 0xFEE00000 >> 12;
    lApicBase.enable = 1;
    // The "bsp" bit will be set when this register is read, since then we'll
    // have a ThreadContext to check the contextId from.
    regVal[misc_reg::ApicBase] = lApicBase;
}

namespace
{

/* Not applicable to X86 */
RegClass vecRegClass(VecRegClass, VecRegClassName, 1, debug::IntRegs);
RegClass vecElemClass(VecElemClass, VecElemClassName, 2, debug::IntRegs);
RegClass vecPredRegClass(VecPredRegClass, VecPredRegClassName, 1,
        debug::IntRegs);
RegClass matRegClass(MatRegClass, MatRegClassName, 0, debug::MatRegs);

} // anonymous namespace

ISA::ISA(const X86ISAParams &p)
    : BaseISA(p, "x86"), cpuid(new X86CPUID(p.vendor_string, p.name_string))
{
    cpuid->addStandardFunc(FamilyModelStepping, p.FamilyModelStepping);
    cpuid->addStandardFunc(CacheParams, p.CacheParams);
    cpuid->addStandardFunc(ExtendedFeatures, p.ExtendedFeatures);
    cpuid->addStandardFunc(ExtendedState, p.ExtendedState);

    cpuid->addExtendedFunc(FamilyModelSteppingBrandFeatures,
                          p.FamilyModelSteppingBrandFeatures);
    cpuid->addExtendedFunc(L1CacheAndTLB, p.L1CacheAndTLB);
    cpuid->addExtendedFunc(L2L3CacheAndL2TLB, p.L2L3CacheAndL2TLB);
    cpuid->addExtendedFunc(APMInfo, p.APMInfo);
    cpuid->addExtendedFunc(LongModeAddressSize, p.LongModeAddressSize);

    _regClasses.push_back(&flatIntRegClass);
    _regClasses.push_back(&flatFloatRegClass);
    _regClasses.push_back(&vecRegClass);
    _regClasses.push_back(&vecElemClass);
    _regClasses.push_back(&vecPredRegClass);
    _regClasses.push_back(&matRegClass);
    _regClasses.push_back(&ccRegClass);
    _regClasses.push_back(&miscRegClass);

    clear();
}

static void
copyMiscRegs(ThreadContext *src, ThreadContext *dest)
{
    // This function assumes no side effects other than TLB invalidation
    // need to be considered while copying state. That will likely not be
    // true in the future.
    for (int i = 0; i < misc_reg::NumRegs; ++i) {
        if (!misc_reg::isValid(i))
             continue;

        dest->setMiscRegNoEffect(i, src->readMiscRegNoEffect(i));
    }

    // The TSC has to be updated with side-effects if the CPUs in a
    // CPU switch have different frequencies.
    dest->setMiscReg(misc_reg::Tsc, src->readMiscReg(misc_reg::Tsc));

	// MCG_CAP register count field holds how many MC MSRs are defined,
	// to function correctly it must be updated with the number of simulated MC MSRs.
	X86ISA::McgCap mcgCap = src->readMiscReg(misc_reg::McgCap);
	mcgCap.count = 8;
	dest->setMiscReg(misc_reg::McgCap, mcgCap);


    dest->getMMUPtr()->flushAll();
}

void
ISA::copyRegsFrom(ThreadContext *src)
{
    //copy int regs
    for (auto &id: flatIntRegClass)
        tc->setReg(id, src->getReg(id));
    //copy float regs
    for (auto &id: flatFloatRegClass)
        tc->setReg(id, src->getReg(id));
    //copy condition-code regs
    for (auto &id: ccRegClass)
        tc->setReg(id, src->getReg(id));
    copyMiscRegs(src, tc);
    tc->pcState(src->pcState());
}

RegVal
ISA::readMiscRegNoEffect(RegIndex idx) const
{
    // Make sure we're not dealing with an illegal control register.
    // Instructions should filter out these indexes, and nothing else should
    // attempt to read them directly.
    assert(misc_reg::isValid(idx));

    return regVal[idx];
}

RegVal
ISA::readMiscReg(RegIndex idx)
{
    if (idx == misc_reg::Tsc) {
        return regVal[misc_reg::Tsc] + tc->getCpuPtr()->curCycle();
    }

    if (idx == misc_reg::Fsw) {
        RegVal fsw = regVal[misc_reg::Fsw];
        RegVal top = regVal[misc_reg::X87Top];
        return insertBits(fsw, 13, 11, top);
    }

    if (idx == misc_reg::ApicBase) {
        LocalApicBase base = regVal[misc_reg::ApicBase];
        base.bsp = (tc->contextId() == 0);
        return base;
    }

    if (idx == misc_reg::Xcr0) {
        return regVal[idx] | 1;
    }

    return readMiscRegNoEffect(idx);
}

void
ISA::setMiscRegNoEffect(RegIndex idx, RegVal val)
{
    // Make sure we're not dealing with an illegal control register.
    // Instructions should filter out these indexes, and nothing else should
    // attempt to write to them directly.
    assert(misc_reg::isValid(idx));

    HandyM5Reg m5Reg = regVal[misc_reg::M5Reg];
    int reg_width = 64;
    switch (idx) {
      case misc_reg::X87Top:
        reg_width = 3;
        break;
      case misc_reg::Ftw:
        reg_width = 16;
        break;
      case misc_reg::Fsw:
      case misc_reg::Fcw:
      case misc_reg::Fop:
        reg_width = 16;
        break;
      case misc_reg::Mxcsr:
        reg_width = 32;
        break;
      case misc_reg::Fiseg:
      case misc_reg::Foseg:
        if (m5Reg.submode != SixtyFourBitMode)
            reg_width = 16;
        break;
      case misc_reg::Fioff:
      case misc_reg::Fooff:
        if (m5Reg.submode != SixtyFourBitMode)
            reg_width = 32;
        break;
      default:
        break;
    }

    regVal[idx] = val & mask(reg_width);
}

void
ISA::setMiscReg(RegIndex idx, RegVal val)
{
    RegVal newVal = val;
    switch (idx) {
      case misc_reg::Cr0:
        {
            CR0 toggled = regVal[idx] ^ val;
            CR0 newCR0 = val;
            Efer efer = regVal[misc_reg::Efer];
            if (toggled.pg && efer.lme) {
                if (newCR0.pg) {
                    //Turning on long mode
                    efer.lma = 1;
                    regVal[misc_reg::Efer] = efer;
                } else {
                    //Turning off long mode
                    efer.lma = 0;
                    regVal[misc_reg::Efer] = efer;
                }
            }
            if (toggled.pg) {
                tc->getMMUPtr()->flushAll();
            }
            //This must always be 1.
            newCR0.et = 1;
            newVal = newCR0;
            updateHandyM5Reg(regVal[misc_reg::Efer],
                             newCR0,
                             regVal[misc_reg::CsAttr],
                             regVal[misc_reg::SsAttr],
                             regVal[misc_reg::Rflags]);
        }
        break;
      case misc_reg::Cr2:
        break;
      case misc_reg::Cr3:
        static_cast<MMU *>(tc->getMMUPtr())->flushNonGlobal();
        break;
      case misc_reg::Cr4:
        {
            CR4 toggled = regVal[idx] ^ val;
            if (toggled.pae || toggled.pse || toggled.pge) {
                tc->getMMUPtr()->flushAll();
            }
        }
        break;
      case misc_reg::Cr8:
        break;
      case misc_reg::Xcr0:
        break;
      case misc_reg::Rflags:
        {
            RFLAGS rflags = val;
            panic_if(rflags.vm, "Virtual 8086 mode is not supported.");
            break;
        }
      case misc_reg::CsAttr:
        {
            SegAttr toggled = regVal[idx] ^ val;
            SegAttr newCSAttr = val;
            if (toggled.longMode) {
                if (newCSAttr.longMode) {
                    regVal[misc_reg::EsEffBase] = 0;
                    regVal[misc_reg::CsEffBase] = 0;
                    regVal[misc_reg::SsEffBase] = 0;
                    regVal[misc_reg::DsEffBase] = 0;
                } else {
                    regVal[misc_reg::EsEffBase] = regVal[misc_reg::EsBase];
                    regVal[misc_reg::CsEffBase] = regVal[misc_reg::CsBase];
                    regVal[misc_reg::SsEffBase] = regVal[misc_reg::SsBase];
                    regVal[misc_reg::DsEffBase] = regVal[misc_reg::DsBase];
                }
            }
            updateHandyM5Reg(regVal[misc_reg::Efer],
                             regVal[misc_reg::Cr0],
                             newCSAttr,
                             regVal[misc_reg::SsAttr],
                             regVal[misc_reg::Rflags]);
        }
        break;
      case misc_reg::SsAttr:
        updateHandyM5Reg(regVal[misc_reg::Efer],
                         regVal[misc_reg::Cr0],
                         regVal[misc_reg::CsAttr],
                         val,
                         regVal[misc_reg::Rflags]);
        break;
      // These segments always actually use their bases, or in other words
      // their effective bases must stay equal to their actual bases.
      case misc_reg::FsBase:
      case misc_reg::GsBase:
      case misc_reg::HsBase:
      case misc_reg::TslBase:
      case misc_reg::TsgBase:
      case misc_reg::TrBase:
      case misc_reg::IdtrBase:
        regVal[misc_reg::segEffBase(idx - misc_reg::SegBaseBase)] = val;
        break;
      // These segments ignore their bases in 64 bit mode.
      // their effective bases must stay equal to their actual bases.
      case misc_reg::EsBase:
      case misc_reg::CsBase:
      case misc_reg::SsBase:
      case misc_reg::DsBase:
        {
            Efer efer = regVal[misc_reg::Efer];
            SegAttr csAttr = regVal[misc_reg::CsAttr];
            if (!efer.lma || !csAttr.longMode) // Check for non 64 bit mode.
                regVal[misc_reg::segEffBase(idx -
                        misc_reg::SegBaseBase)] = val;
        }
        break;
      case misc_reg::Tsc:
        regVal[misc_reg::Tsc] = val - tc->getCpuPtr()->curCycle();
        return;
      case misc_reg::Dr0:
      case misc_reg::Dr1:
      case misc_reg::Dr2:
      case misc_reg::Dr3:
        /* These should eventually set up breakpoints. */
        break;
      case misc_reg::Dr4:
        idx = misc_reg::Dr6;
        [[fallthrough]];
      case misc_reg::Dr6:
        {
            DR6 dr6 = regVal[misc_reg::Dr6];
            DR6 newDR6 = val;
            dr6.b0 = newDR6.b0;
            dr6.b1 = newDR6.b1;
            dr6.b2 = newDR6.b2;
            dr6.b3 = newDR6.b3;
            dr6.bd = newDR6.bd;
            dr6.bs = newDR6.bs;
            dr6.bt = newDR6.bt;
            newVal = dr6;
        }
        break;
      case misc_reg::Dr5:
        idx = misc_reg::Dr7;
        [[fallthrough]];
      case misc_reg::Dr7:
        {
            DR7 dr7 = regVal[misc_reg::Dr7];
            DR7 newDR7 = val;
            dr7.l0 = newDR7.l0;
            dr7.g0 = newDR7.g0;
            if (dr7.l0 || dr7.g0) {
                panic("Debug register breakpoints not implemented.\n");
            } else {
                /* Disable breakpoint 0. */
            }
            dr7.l1 = newDR7.l1;
            dr7.g1 = newDR7.g1;
            if (dr7.l1 || dr7.g1) {
                panic("Debug register breakpoints not implemented.\n");
            } else {
                /* Disable breakpoint 1. */
            }
            dr7.l2 = newDR7.l2;
            dr7.g2 = newDR7.g2;
            if (dr7.l2 || dr7.g2) {
                panic("Debug register breakpoints not implemented.\n");
            } else {
                /* Disable breakpoint 2. */
            }
            dr7.l3 = newDR7.l3;
            dr7.g3 = newDR7.g3;
            if (dr7.l3 || dr7.g3) {
                panic("Debug register breakpoints not implemented.\n");
            } else {
                /* Disable breakpoint 3. */
            }
            dr7.gd = newDR7.gd;
            dr7.rw0 = newDR7.rw0;
            dr7.len0 = newDR7.len0;
            dr7.rw1 = newDR7.rw1;
            dr7.len1 = newDR7.len1;
            dr7.rw2 = newDR7.rw2;
            dr7.len2 = newDR7.len2;
            dr7.rw3 = newDR7.rw3;
            dr7.len3 = newDR7.len3;
        }
        break;
      case misc_reg::M5Reg:
        // Writing anything to the m5reg with side effects makes it update
        // based on the current values of the relevant registers. The actual
        // value written is discarded.
        updateHandyM5Reg(regVal[misc_reg::Efer],
                         regVal[misc_reg::Cr0],
                         regVal[misc_reg::CsAttr],
                         regVal[misc_reg::SsAttr],
                         regVal[misc_reg::Rflags]);
        return;
      default:
        break;
    }
    setMiscRegNoEffect(idx, newVal);
}

void
ISA::serialize(CheckpointOut &cp) const
{
    BaseISA::serialize(cp);

    SERIALIZE_ARRAY(regVal, misc_reg::NumRegs);
}

void
ISA::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_ARRAY(regVal, misc_reg::NumRegs);
    updateHandyM5Reg(regVal[misc_reg::Efer],
                     regVal[misc_reg::Cr0],
                     regVal[misc_reg::CsAttr],
                     regVal[misc_reg::SsAttr],
                     regVal[misc_reg::Rflags]);
}

void
ISA::setThreadContext(ThreadContext *_tc)
{
    BaseISA::setThreadContext(_tc);
    tc->getDecoderPtr()->as<Decoder>().setM5Reg(regVal[misc_reg::M5Reg]);
}

std::string
ISA::getVendorString() const
{
    return vendorString;
}

} // namespace X86ISA
} // namespace gem5
