/*
 * Copyright (c) 2010, 2012-2013, 2016 ARM Limited
 * All rights reserved
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
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
 * Copyright (c) 2007-2008 The Florida State University
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
 * Authors: Korey Sewell
 *          Stephen Hines
 */

#ifndef __ARCH_ARM_UTILITY_HH__
#define __ARCH_ARM_UTILITY_HH__

#include "arch/arm/isa_traits.hh"
#include "arch/arm/miscregs.hh"
#include "arch/arm/types.hh"
#include "base/misc.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"

class ArmSystem;

namespace ArmISA {

inline PCState
buildRetPC(const PCState &curPC, const PCState &callPC)
{
    PCState retPC = callPC;
    retPC.uEnd();
    return retPC;
}

inline bool
testPredicate(uint32_t nz, uint32_t c, uint32_t v, ConditionCode code)
{
    bool n = (nz & 0x2);
    bool z = (nz & 0x1);

    switch (code)
    {
        case COND_EQ: return  z;
        case COND_NE: return !z;
        case COND_CS: return  c;
        case COND_CC: return !c;
        case COND_MI: return  n;
        case COND_PL: return !n;
        case COND_VS: return  v;
        case COND_VC: return !v;
        case COND_HI: return  (c && !z);
        case COND_LS: return !(c && !z);
        case COND_GE: return !(n ^ v);
        case COND_LT: return  (n ^ v);
        case COND_GT: return !(n ^ v || z);
        case COND_LE: return  (n ^ v || z);
        case COND_AL: return true;
        case COND_UC: return true;
        default:
            panic("Unhandled predicate condition: %d\n", code);
    }
}

/**
 * Function to insure ISA semantics about 0 registers.
 * @param tc The thread context.
 */
template <class TC>
void zeroRegisters(TC *tc);

inline void startupCPU(ThreadContext *tc, int cpuId)
{
    tc->activate();
}

void copyRegs(ThreadContext *src, ThreadContext *dest);

static inline void
copyMiscRegs(ThreadContext *src, ThreadContext *dest)
{
    panic("Copy Misc. Regs Not Implemented Yet\n");
}

void initCPU(ThreadContext *tc, int cpuId);

static inline bool
inUserMode(CPSR cpsr)
{
    return cpsr.mode == MODE_USER || cpsr.mode == MODE_EL0T;
}

static inline bool
inUserMode(ThreadContext *tc)
{
    return inUserMode(tc->readMiscRegNoEffect(MISCREG_CPSR));
}

static inline bool
inPrivilegedMode(CPSR cpsr)
{
    return !inUserMode(cpsr);
}

static inline bool
inPrivilegedMode(ThreadContext *tc)
{
    return !inUserMode(tc);
}

bool inAArch64(ThreadContext *tc);

static inline OperatingMode
currOpMode(ThreadContext *tc)
{
    CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);
    return (OperatingMode) (uint8_t) cpsr.mode;
}

static inline ExceptionLevel
currEL(ThreadContext *tc)
{
    CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);
    return (ExceptionLevel) (uint8_t) cpsr.el;
}

bool ELIs64(ThreadContext *tc, ExceptionLevel el);

bool isBigEndian64(ThreadContext *tc);

static inline uint8_t
itState(CPSR psr)
{
    ITSTATE it = 0;
    it.top6 = psr.it2;
    it.bottom2 = psr.it1;

    return (uint8_t)it;
}

/**
 * Removes the tag from tagged addresses if that mode is enabled.
 * @param addr The address to be purified.
 * @param tc The thread context.
 * @param el The controlled exception level.
 * @return The purified address.
 */
Addr purifyTaggedAddr(Addr addr, ThreadContext *tc, ExceptionLevel el,
                      TTBCR tcr);
Addr purifyTaggedAddr(Addr addr, ThreadContext *tc, ExceptionLevel el);

static inline bool
inSecureState(SCR scr, CPSR cpsr)
{
    switch ((OperatingMode) (uint8_t) cpsr.mode) {
      case MODE_MON:
      case MODE_EL3T:
      case MODE_EL3H:
        return true;
      case MODE_HYP:
      case MODE_EL2T:
      case MODE_EL2H:
        return false;
      default:
        return !scr.ns;
    }
}

bool longDescFormatInUse(ThreadContext *tc);

bool inSecureState(ThreadContext *tc);

uint32_t getMPIDR(ArmSystem *arm_sys, ThreadContext *tc);

static inline uint32_t
mcrMrcIssBuild(bool isRead, uint32_t crm, IntRegIndex rt, uint32_t crn,
               uint32_t opc1, uint32_t opc2)
{
    return (isRead <<  0) |
           (crm    <<  1) |
           (rt     <<  5) |
           (crn    << 10) |
           (opc1   << 14) |
           (opc2   << 17);
}

static inline void
mcrMrcIssExtract(uint32_t iss, bool &isRead, uint32_t &crm, IntRegIndex &rt,
                 uint32_t &crn, uint32_t &opc1, uint32_t &opc2)
{
    isRead = (iss >>  0) & 0x1;
    crm    = (iss >>  1) & 0xF;
    rt     = (IntRegIndex) ((iss >>  5) & 0xF);
    crn    = (iss >> 10) & 0xF;
    opc1   = (iss >> 14) & 0x7;
    opc2   = (iss >> 17) & 0x7;
}

static inline uint32_t
mcrrMrrcIssBuild(bool isRead, uint32_t crm, IntRegIndex rt, IntRegIndex rt2,
                 uint32_t opc1)
{
    return (isRead <<  0) |
           (crm    <<  1) |
           (rt     <<  5) |
           (rt2    << 10) |
           (opc1   << 16);
}

static inline uint32_t
msrMrs64IssBuild(bool isRead, uint32_t op0, uint32_t op1, uint32_t crn,
                 uint32_t crm, uint32_t op2, IntRegIndex rt)
{
    return isRead |
        (crm << 1) |
        (rt << 5) |
        (crn << 10) |
        (op1 << 14) |
        (op2 << 17) |
        (op0 << 20);
}

bool
mcrMrc15TrapToHyp(const MiscRegIndex miscReg, HCR hcr, CPSR cpsr, SCR scr,
                  HDCR hdcr, HSTR hstr, HCPTR hcptr, uint32_t iss);
bool
mcrMrc14TrapToHyp(const MiscRegIndex miscReg, HCR hcr, CPSR cpsr, SCR scr,
                  HDCR hdcr, HSTR hstr, HCPTR hcptr, uint32_t iss);
bool
mcrrMrrc15TrapToHyp(const MiscRegIndex miscReg, CPSR cpsr, SCR scr, HSTR hstr,
                    HCR hcr, uint32_t iss);

bool msrMrs64TrapToSup(const MiscRegIndex miscReg, ExceptionLevel el,
                       CPACR cpacr);
bool msrMrs64TrapToHyp(const MiscRegIndex miscReg, bool isRead, CPTR cptr,
                       HCR hcr, bool * isVfpNeon);
bool msrMrs64TrapToMon(const MiscRegIndex miscReg, CPTR cptr,
                       ExceptionLevel el, bool * isVfpNeon);

bool SPAlignmentCheckEnabled(ThreadContext* tc);

uint64_t getArgument(ThreadContext *tc, int &number, uint16_t size, bool fp);

void skipFunction(ThreadContext *tc);

inline void
advancePC(PCState &pc, const StaticInstPtr &inst)
{
    inst->advancePC(pc);
}

Addr truncPage(Addr addr);
Addr roundPage(Addr addr);

inline uint64_t
getExecutingAsid(ThreadContext *tc)
{
    return tc->readMiscReg(MISCREG_CONTEXTIDR);
}

// Decodes the register index to access based on the fields used in a MSR
// or MRS instruction
bool
decodeMrsMsrBankedReg(uint8_t sysM, bool r, bool &isIntReg, int &regIdx,
                      CPSR cpsr, SCR scr, NSACR nsacr,
                      bool checkSecurity = true);

// This wrapper function is used to turn the register index into a source
// parameter for the instruction. See Operands.isa
static inline int
decodeMrsMsrBankedIntRegIndex(uint8_t sysM, bool r)
{
    int  regIdx;
    bool isIntReg;
    bool validReg;

    validReg = decodeMrsMsrBankedReg(sysM, r, isIntReg, regIdx, 0, 0, 0, false);
    return (validReg && isIntReg) ? regIdx : INTREG_DUMMY;
}

/**
 * Returns the n. of PA bits corresponding to the specified encoding.
 */
int decodePhysAddrRange64(uint8_t pa_enc);

/**
 * Returns the encoding corresponding to the specified n. of PA bits.
 */
uint8_t encodePhysAddrRange64(int pa_size);

}

#endif
