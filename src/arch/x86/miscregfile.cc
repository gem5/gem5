/*
 * Copyright (c) 2003-2006, 2008 The Regents of The University of Michigan
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
 * Copyright (c) 2007-2008 The Hewlett-Packard Development Company
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

#include "arch/x86/miscregfile.hh"
#include "arch/x86/tlb.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "sim/serialize.hh"

using namespace X86ISA;
using namespace std;

class Checkpoint;

//These functions map register indices to names
string X86ISA::getMiscRegName(RegIndex index)
{
    panic("No misc registers in x86 yet!\n");
}

void MiscRegFile::clear()
{
    // Blank everything. 0 might not be an appropriate value for some things.
    memset(regVal, 0, NumMiscRegs * sizeof(MiscReg));
}

MiscReg MiscRegFile::readRegNoEffect(MiscRegIndex miscReg)
{
    // Make sure we're not dealing with an illegal control register.
    // Instructions should filter out these indexes, and nothing else should
    // attempt to read them directly.
    assert( miscReg != MISCREG_CR1 &&
            !(miscReg > MISCREG_CR4 &&
              miscReg < MISCREG_CR8) &&
            !(miscReg > MISCREG_CR8 &&
              miscReg <= MISCREG_CR15));

    if (isApicReg(miscReg)) {
        panic("Can't readRegNoEffect from the local APIC.\n");
    }
    return regVal[miscReg];
}

MiscReg MiscRegFile::readReg(MiscRegIndex miscReg, ThreadContext * tc)
{
#if FULL_SYSTEM
    if (isApicReg(miscReg)) {
        Interrupts * interrupts = dynamic_cast<Interrupts *>(
                tc->getCpuPtr()->getInterruptController());
        assert(interrupts);
        return interrupts->readReg(
                (ApicRegIndex)(miscReg - MISCREG_APIC_START), tc);
    }
#endif
    if (miscReg == MISCREG_TSC) {
        return regVal[MISCREG_TSC] + tc->getCpuPtr()->curCycle();
    }
    return readRegNoEffect(miscReg);
}

void MiscRegFile::setRegNoEffect(MiscRegIndex miscReg, const MiscReg &val)
{
    // Make sure we're not dealing with an illegal control register.
    // Instructions should filter out these indexes, and nothing else should
    // attempt to write to them directly.
    assert( miscReg != MISCREG_CR1 &&
            !(miscReg > MISCREG_CR4 &&
              miscReg < MISCREG_CR8) &&
            !(miscReg > MISCREG_CR8 &&
              miscReg <= MISCREG_CR15));
    if (isApicReg(miscReg)) {
        panic("Can't setRegNoEffect from the local APIC.\n");
    }
    regVal[miscReg] = val;
}

void MiscRegFile::setReg(MiscRegIndex miscReg,
        const MiscReg &val, ThreadContext * tc)
{
    MiscReg newVal = val;
#if FULL_SYSTEM
    if (isApicReg(miscReg)) {
        Interrupts * interrupts = dynamic_cast<Interrupts *>(
                tc->getCpuPtr()->getInterruptController());
        assert(interrupts);
        interrupts->setReg(
                ApicRegIndex(miscReg - MISCREG_APIC_START), val, tc);
        return;
    }
#endif
    switch(miscReg)
    {
      case MISCREG_CR0:
        {
            CR0 toggled = regVal[miscReg] ^ val;
            CR0 newCR0 = val;
            Efer efer = regVal[MISCREG_EFER];
            HandyM5Reg m5reg = regVal[MISCREG_M5_REG];
            if (toggled.pg && efer.lme) {
                if (newCR0.pg) {
                    //Turning on long mode
                    efer.lma = 1;
                    m5reg.mode = LongMode;
                    regVal[MISCREG_EFER] = efer;
                } else {
                    //Turning off long mode
                    efer.lma = 0;
                    m5reg.mode = LegacyMode;
                    regVal[MISCREG_EFER] = efer;
                }
            }
            // Figure out what submode we're in.
            if (m5reg.mode == LongMode) {
                SegAttr csAttr = regVal[MISCREG_CS_ATTR];
                if (csAttr.longMode)
                    m5reg.submode = SixtyFourBitMode;
                else
                    m5reg.submode = CompatabilityMode;
            } else {
                if (newCR0.pe) {
                    RFLAGS rflags = regVal[MISCREG_RFLAGS];
                    if (rflags.vm)
                        m5reg.submode = Virtual8086Mode;
                    else
                        m5reg.submode = ProtectedMode;
                } else {
                    m5reg.submode = RealMode;
                }
            }
            regVal[MISCREG_M5_REG] = m5reg;
            if (toggled.pg) {
                tc->getITBPtr()->invalidateAll();
                tc->getDTBPtr()->invalidateAll();
            }
            //This must always be 1.
            newCR0.et = 1;
            newVal = newCR0;
        }
        break;
      case MISCREG_CR2:
        break;
      case MISCREG_CR3:
        tc->getITBPtr()->invalidateNonGlobal();
        tc->getDTBPtr()->invalidateNonGlobal();
        break;
      case MISCREG_CR4:
        {
            CR4 toggled = regVal[miscReg] ^ val;
            if (toggled.pae || toggled.pse || toggled.pge) {
                tc->getITBPtr()->invalidateAll();
                tc->getDTBPtr()->invalidateAll();
            }
        }
        break;
      case MISCREG_CR8:
        break;
      case MISCREG_CS_ATTR:
        {
            SegAttr toggled = regVal[miscReg] ^ val;
            SegAttr newCSAttr = val;
            HandyM5Reg m5reg = regVal[MISCREG_M5_REG];
            if (toggled.longMode) {
                if (newCSAttr.longMode) {
                    if (m5reg.mode == LongMode)
                        m5reg.submode = SixtyFourBitMode;
                    regVal[MISCREG_ES_EFF_BASE] = 0;
                    regVal[MISCREG_CS_EFF_BASE] = 0;
                    regVal[MISCREG_SS_EFF_BASE] = 0;
                    regVal[MISCREG_DS_EFF_BASE] = 0;
                } else {
                    if (m5reg.mode == LongMode)
                        m5reg.submode = CompatabilityMode;
                    regVal[MISCREG_ES_EFF_BASE] = regVal[MISCREG_ES_BASE];
                    regVal[MISCREG_CS_EFF_BASE] = regVal[MISCREG_CS_BASE];
                    regVal[MISCREG_SS_EFF_BASE] = regVal[MISCREG_SS_BASE];
                    regVal[MISCREG_DS_EFF_BASE] = regVal[MISCREG_DS_BASE];
                }
            }
            m5reg.cpl = newCSAttr.dpl;
            regVal[MISCREG_M5_REG] = m5reg;
        }
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
      default:
        break;
    }
    setRegNoEffect(miscReg, newVal);
}

void MiscRegFile::serialize(std::ostream & os)
{
    SERIALIZE_ARRAY(regVal, NumMiscRegs);
}

void MiscRegFile::unserialize(Checkpoint * cp, const std::string & section)
{
    UNSERIALIZE_ARRAY(regVal, NumMiscRegs);
}
