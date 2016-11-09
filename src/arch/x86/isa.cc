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

#include "arch/x86/isa.hh"

#include "arch/x86/decoder.hh"
#include "arch/x86/tlb.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "params/X86ISA.hh"
#include "sim/serialize.hh"

namespace X86ISA
{

void
ISA::updateHandyM5Reg(Efer efer, CR0 cr0,
                      SegAttr csAttr, SegAttr ssAttr, RFLAGS rflags,
                      ThreadContext *tc)
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

    regVal[MISCREG_M5_REG] = m5reg;
    if (tc)
        tc->getDecoderPtr()->setM5Reg(m5reg);
}

void
ISA::clear()
{
    // Blank everything. 0 might not be an appropriate value for some things,
    // but it is for most.
    memset(regVal, 0, NumMiscRegs * sizeof(MiscReg));
    regVal[MISCREG_DR6] = (mask(8) << 4) | (mask(16) << 16);
    regVal[MISCREG_DR7] = 1 << 10;
}

ISA::ISA(Params *p)
    : SimObject(p)
{
    clear();
}

const X86ISAParams *
ISA::params() const
{
    return dynamic_cast<const Params *>(_params);
}

MiscReg
ISA::readMiscRegNoEffect(int miscReg) const
{
    // Make sure we're not dealing with an illegal control register.
    // Instructions should filter out these indexes, and nothing else should
    // attempt to read them directly.
    assert(isValidMiscReg(miscReg));

    return regVal[miscReg];
}

MiscReg
ISA::readMiscReg(int miscReg, ThreadContext * tc)
{
    if (miscReg == MISCREG_TSC) {
        return regVal[MISCREG_TSC] + tc->getCpuPtr()->curCycle();
    }

    if (miscReg == MISCREG_FSW) {
        MiscReg fsw = regVal[MISCREG_FSW];
        MiscReg top = regVal[MISCREG_X87_TOP];
        return (fsw & (~(7ULL << 11))) + (top << 11);
    }

    return readMiscRegNoEffect(miscReg);
}

void
ISA::setMiscRegNoEffect(int miscReg, MiscReg val)
{
    // Make sure we're not dealing with an illegal control register.
    // Instructions should filter out these indexes, and nothing else should
    // attempt to write to them directly.
    assert(isValidMiscReg(miscReg));

    HandyM5Reg m5Reg = readMiscRegNoEffect(MISCREG_M5_REG);
    switch (miscReg) {
      case MISCREG_FSW:
        val &= (1ULL << 16) - 1;
        regVal[miscReg] = val;
        miscReg = MISCREG_X87_TOP;
        val <<= 11;
      case MISCREG_X87_TOP:
        val &= (1ULL << 3) - 1;
        break;
      case MISCREG_FTW:
        val &= (1ULL << 8) - 1;
        break;
      case MISCREG_FCW:
      case MISCREG_FOP:
        val &= (1ULL << 16) - 1;
        break;
      case MISCREG_MXCSR:
        val &= (1ULL << 32) - 1;
        break;
      case MISCREG_FISEG:
      case MISCREG_FOSEG:
        if (m5Reg.submode != SixtyFourBitMode)
            val &= (1ULL << 16) - 1;
        break;
      case MISCREG_FIOFF:
      case MISCREG_FOOFF:
        if (m5Reg.submode != SixtyFourBitMode)
            val &= (1ULL << 32) - 1;
        break;
      default:
        break;
    }

    regVal[miscReg] = val;
}

void
ISA::setMiscReg(int miscReg, MiscReg val, ThreadContext * tc)
{
    MiscReg newVal = val;
    switch(miscReg)
    {
      case MISCREG_CR0:
        {
            CR0 toggled = regVal[miscReg] ^ val;
            CR0 newCR0 = val;
            Efer efer = regVal[MISCREG_EFER];
            if (toggled.pg && efer.lme) {
                if (newCR0.pg) {
                    //Turning on long mode
                    efer.lma = 1;
                    regVal[MISCREG_EFER] = efer;
                } else {
                    //Turning off long mode
                    efer.lma = 0;
                    regVal[MISCREG_EFER] = efer;
                }
            }
            if (toggled.pg) {
                tc->getITBPtr()->flushAll();
                tc->getDTBPtr()->flushAll();
            }
            //This must always be 1.
            newCR0.et = 1;
            newVal = newCR0;
            updateHandyM5Reg(regVal[MISCREG_EFER],
                             newCR0,
                             regVal[MISCREG_CS_ATTR],
                             regVal[MISCREG_SS_ATTR],
                             regVal[MISCREG_RFLAGS],
                             tc);
        }
        break;
      case MISCREG_CR2:
        break;
      case MISCREG_CR3:
        tc->getITBPtr()->flushNonGlobal();
        tc->getDTBPtr()->flushNonGlobal();
        break;
      case MISCREG_CR4:
        {
            CR4 toggled = regVal[miscReg] ^ val;
            if (toggled.pae || toggled.pse || toggled.pge) {
                tc->getITBPtr()->flushAll();
                tc->getDTBPtr()->flushAll();
            }
        }
        break;
      case MISCREG_CR8:
        break;
      case MISCREG_CS_ATTR:
        {
            SegAttr toggled = regVal[miscReg] ^ val;
            SegAttr newCSAttr = val;
            if (toggled.longMode) {
                if (newCSAttr.longMode) {
                    regVal[MISCREG_ES_EFF_BASE] = 0;
                    regVal[MISCREG_CS_EFF_BASE] = 0;
                    regVal[MISCREG_SS_EFF_BASE] = 0;
                    regVal[MISCREG_DS_EFF_BASE] = 0;
                } else {
                    regVal[MISCREG_ES_EFF_BASE] = regVal[MISCREG_ES_BASE];
                    regVal[MISCREG_CS_EFF_BASE] = regVal[MISCREG_CS_BASE];
                    regVal[MISCREG_SS_EFF_BASE] = regVal[MISCREG_SS_BASE];
                    regVal[MISCREG_DS_EFF_BASE] = regVal[MISCREG_DS_BASE];
                }
            }
            updateHandyM5Reg(regVal[MISCREG_EFER],
                             regVal[MISCREG_CR0],
                             newCSAttr,
                             regVal[MISCREG_SS_ATTR],
                             regVal[MISCREG_RFLAGS],
                             tc);
        }
        break;
      case MISCREG_SS_ATTR:
        updateHandyM5Reg(regVal[MISCREG_EFER],
                         regVal[MISCREG_CR0],
                         regVal[MISCREG_CS_ATTR],
                         val,
                         regVal[MISCREG_RFLAGS],
                         tc);
        break;
      // These segments always actually use their bases, or in other words
      // their effective bases must stay equal to their actual bases.
      case MISCREG_FS_BASE:
      case MISCREG_GS_BASE:
      case MISCREG_HS_BASE:
      case MISCREG_TSL_BASE:
      case MISCREG_TSG_BASE:
      case MISCREG_TR_BASE:
      case MISCREG_IDTR_BASE:
        regVal[MISCREG_SEG_EFF_BASE(miscReg - MISCREG_SEG_BASE_BASE)] = val;
        break;
      // These segments ignore their bases in 64 bit mode.
      // their effective bases must stay equal to their actual bases.
      case MISCREG_ES_BASE:
      case MISCREG_CS_BASE:
      case MISCREG_SS_BASE:
      case MISCREG_DS_BASE:
        {
            Efer efer = regVal[MISCREG_EFER];
            SegAttr csAttr = regVal[MISCREG_CS_ATTR];
            if (!efer.lma || !csAttr.longMode) // Check for non 64 bit mode.
                regVal[MISCREG_SEG_EFF_BASE(miscReg -
                        MISCREG_SEG_BASE_BASE)] = val;
        }
        break;
      case MISCREG_TSC:
        regVal[MISCREG_TSC] = val - tc->getCpuPtr()->curCycle();
        return;
      case MISCREG_DR0:
      case MISCREG_DR1:
      case MISCREG_DR2:
      case MISCREG_DR3:
        /* These should eventually set up breakpoints. */
        break;
      case MISCREG_DR4:
        miscReg = MISCREG_DR6;
        /* Fall through to have the same effects as DR6. */
      case MISCREG_DR6:
        {
            DR6 dr6 = regVal[MISCREG_DR6];
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
      case MISCREG_DR5:
        miscReg = MISCREG_DR7;
        /* Fall through to have the same effects as DR7. */
      case MISCREG_DR7:
        {
            DR7 dr7 = regVal[MISCREG_DR7];
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
      case MISCREG_M5_REG:
        // Writing anything to the m5reg with side effects makes it update
        // based on the current values of the relevant registers. The actual
        // value written is discarded.
        updateHandyM5Reg(regVal[MISCREG_EFER],
                         regVal[MISCREG_CR0],
                         regVal[MISCREG_CS_ATTR],
                         regVal[MISCREG_SS_ATTR],
                         regVal[MISCREG_RFLAGS],
                         tc);
        return;
      default:
        break;
    }
    setMiscRegNoEffect(miscReg, newVal);
}

void
ISA::serialize(CheckpointOut &cp) const
{
    SERIALIZE_ARRAY(regVal, NumMiscRegs);
}

void
ISA::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_ARRAY(regVal, NumMiscRegs);
    updateHandyM5Reg(regVal[MISCREG_EFER],
                     regVal[MISCREG_CR0],
                     regVal[MISCREG_CS_ATTR],
                     regVal[MISCREG_SS_ATTR],
                     regVal[MISCREG_RFLAGS],
                     NULL);
}

void
ISA::startup(ThreadContext *tc)
{
    tc->getDecoderPtr()->setM5Reg(regVal[MISCREG_M5_REG]);
}

}

X86ISA::ISA *
X86ISAParams::create()
{
    return new X86ISA::ISA(this);
}
