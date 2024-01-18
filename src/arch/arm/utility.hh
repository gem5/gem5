/*
 * Copyright (c) 2010, 2012-2013, 2016-2020, 2022-2024 Arm Limited
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
 */

#ifndef __ARCH_ARM_UTILITY_HH__
#define __ARCH_ARM_UTILITY_HH__

#include "arch/arm/regs/cc.hh"
#include "arch/arm/regs/int.hh"
#include "arch/arm/regs/misc.hh"
#include "arch/arm/types.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "enums/ArmExtension.hh"

namespace gem5
{

class ArmSystem;

namespace ArmISA
{

inline bool
testPredicate(uint32_t nz, uint32_t c, uint32_t v, ConditionCode code)
{
    bool n = (nz & 0x2);
    bool z = (nz & 0x1);

    switch (code) {
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

/** Send an event (SEV) to a specific PE if there isn't
 * already a pending event */
void sendEvent(ThreadContext *tc);

static inline bool
inUserMode(CPSR cpsr)
{
    return cpsr.mode == MODE_USER || cpsr.mode == MODE_EL0T;
}

static inline bool
inPrivilegedMode(CPSR cpsr)
{
    return !inUserMode(cpsr);
}

bool isSecure(ThreadContext *tc);

bool inAArch64(ThreadContext *tc);

/**
 * Returns the current Exception Level (EL) of the
 * provided ThreadContext
 */
ExceptionLevel currEL(const ThreadContext *tc);

inline ExceptionLevel
currEL(CPSR cpsr)
{
    return opModeToEL((OperatingMode) (uint8_t)cpsr.mode);
}

/**
 * Returns true if the provided ThreadContext supports the ArmExtension
 * passed as a second argument.
 */
bool HaveExt(ThreadContext *tc, ArmExtension ext);

bool IsSecureEL2Enabled(ThreadContext *tc);
bool EL2Enabled(ThreadContext *tc);

/**
 * This function checks whether selected EL provided as an argument
 * is using the AArch32 ISA. This information might be unavailable
 * at the current EL status: it hence returns a pair of boolean values:
 * a first boolean, true if information is available (known),
 * and a second one, true if EL is using AArch32, false for AArch64.
 *
 * @param tc The thread context.
 * @param el The target exception level.
 * @retval known is FALSE for EL0 if the current Exception level
 *               is not EL0 and EL1 is using AArch64, since it cannot
 *               determine the state of EL0; TRUE otherwise.
 * @retval aarch32 is TRUE if the specified Exception level is using AArch32;
 *                 FALSE otherwise.
 */
std::pair<bool, bool> ELUsingAArch32K(ThreadContext *tc, ExceptionLevel el);

std::pair<bool, bool> ELStateUsingAArch32K(ThreadContext *tc,
        ExceptionLevel el, bool secure);

bool ELStateUsingAArch32(ThreadContext *tc, ExceptionLevel el, bool secure);

bool ELIs32(ThreadContext *tc, ExceptionLevel el);

bool ELIs64(ThreadContext *tc, ExceptionLevel el);

/**
 * Returns true if the current exception level `el` is executing a Host OS or
 * an application of a Host OS (Armv8.1 Virtualization Host Extensions).
 */
bool ELIsInHost(ThreadContext *tc, ExceptionLevel el);

ExceptionLevel debugTargetFrom(ThreadContext *tc, bool secure);

bool isBigEndian64(const ThreadContext *tc);


/**
 * badMode is checking if the execution mode provided as an argument is
 * valid and implemented for AArch32
 *
 * @param tc ThreadContext
 * @param mode OperatingMode to check
 * @return false if mode is valid and implemented, true otherwise
 */
bool badMode32(ThreadContext *tc, OperatingMode mode);

/**
 * badMode is checking if the execution mode provided as an argument is
 * valid and implemented.
 *
 * @param tc ThreadContext
 * @param mode OperatingMode to check
 * @return false if mode is valid and implemented, true otherwise
 */
bool badMode(ThreadContext *tc, OperatingMode mode);

static inline uint8_t
itState(CPSR psr)
{
    ITSTATE it = 0;
    it.top6 = psr.it2;
    it.bottom2 = psr.it1;

    return (uint8_t)it;
}

ExceptionLevel s1TranslationRegime(ThreadContext *tc, ExceptionLevel el);

/**
 * Removes the tag from tagged addresses if that mode is enabled.
 * @param addr The address to be purified.
 * @param tc The thread context.
 * @param el The controlled exception level.
 * @return The purified address.
 */
Addr purifyTaggedAddr(Addr addr, ThreadContext *tc, ExceptionLevel el,
                      TCR tcr, bool isInstr);
Addr purifyTaggedAddr(Addr addr, ThreadContext *tc, ExceptionLevel el,
                      bool isInstr);
Addr maskTaggedAddr(Addr addr, ThreadContext *tc, ExceptionLevel el,
                    int topbit);
int computeAddrTop(ThreadContext *tc, bool selbit, bool isInstr,
                   TCR tcr, ExceptionLevel el);

bool isSecureBelowEL3(ThreadContext *tc);

SecurityState securityStateAtEL(ThreadContext *tc, ExceptionLevel el);

bool longDescFormatInUse(ThreadContext *tc);

/** This helper function is either returing the value of
 * MPIDR_EL1 (by calling getMPIDR), or it is issuing a read
 * to VMPIDR_EL2 (as it happens in virtualized systems) */
RegVal readMPIDR(ArmSystem *arm_sys, ThreadContext *tc);

/** This helper function is returning the value of MPIDR_EL1 */
RegVal getMPIDR(ArmSystem *arm_sys, ThreadContext *tc);

/** Retrieves MPIDR_EL1.{Aff2,Aff1,Aff0} affinity numbers */
Affinity getAffinity(ArmSystem *arm_sys, ThreadContext *tc);

static inline uint32_t
mcrMrcIssBuild(bool isRead, uint32_t crm, RegIndex rt, uint32_t crn,
               uint32_t opc1, uint32_t opc2)
{
    return (isRead << 0) |
           (crm << 1) |
           (rt << 5) |
           (crn << 10) |
           (opc1 << 14) |
           (opc2 << 17);
}

static inline void
mcrMrcIssExtract(uint32_t iss, bool &isRead, uint32_t &crm, RegIndex &rt,
                 uint32_t &crn, uint32_t &opc1, uint32_t &opc2)
{
    isRead = (iss >> 0) & 0x1;
    crm = (iss >> 1) & 0xF;
    rt = (RegIndex)((iss >> 5) & 0xF);
    crn = (iss >> 10) & 0xF;
    opc1 = (iss >> 14) & 0x7;
    opc2 = (iss >> 17) & 0x7;
}

static inline uint32_t
mcrrMrrcIssBuild(bool isRead, uint32_t crm, RegIndex rt, RegIndex rt2,
                 uint32_t opc1)
{
    return (isRead << 0) |
           (crm << 1) |
           (rt << 5) |
           (rt2 << 10) |
           (opc1 << 16);
}

Fault mcrMrc15Trap(const MiscRegIndex miscReg, ExtMachInst machInst,
                   ThreadContext *tc, uint32_t imm);
bool mcrMrc15TrapToHyp(const MiscRegIndex miscReg, ThreadContext *tc,
                       uint32_t iss, ExceptionClass *ec=nullptr);

bool mcrMrc14TrapToHyp(const MiscRegIndex miscReg, ThreadContext *tc,
                       uint32_t iss);

Fault mcrrMrrc15Trap(const MiscRegIndex miscReg, ExtMachInst machInst,
                     ThreadContext *tc, uint32_t imm);
bool mcrrMrrc15TrapToHyp(const MiscRegIndex miscReg, ThreadContext *tc,
                         uint32_t iss, ExceptionClass *ec=nullptr);

Fault AArch64AArch32SystemAccessTrap(const MiscRegIndex miscReg,
                                     ExtMachInst machInst, ThreadContext *tc,
                                     uint32_t imm, ExceptionClass ec);
bool isAArch64AArch32SystemAccessTrapEL1(const MiscRegIndex miscReg,
                                         ThreadContext *tc);
bool isAArch64AArch32SystemAccessTrapEL2(const MiscRegIndex miscReg,
                                         ThreadContext *tc);
bool isGenericTimerHypTrap(const MiscRegIndex miscReg, ThreadContext *tc,
                           ExceptionClass *ec);
bool condGenericTimerPhysHypTrap(const MiscRegIndex miscReg,
                                 ThreadContext *tc);
bool isGenericTimerCommonEL0HypTrap(const MiscRegIndex miscReg,
                                    ThreadContext *tc, ExceptionClass *ec);
bool isGenericTimerPhysHypTrap(const MiscRegIndex miscReg, ThreadContext *tc,
                               ExceptionClass *ec);
bool condGenericTimerPhysHypTrap(const MiscRegIndex miscReg,
                                 ThreadContext *tc);
bool isGenericTimerSystemAccessTrapEL1(const MiscRegIndex miscReg,
                                       ThreadContext *tc);
bool condGenericTimerSystemAccessTrapEL1(const MiscRegIndex miscReg,
                                         ThreadContext *tc);
bool isGenericTimerSystemAccessTrapEL2(const MiscRegIndex miscReg,
                                       ThreadContext *tc);
bool isGenericTimerCommonEL0SystemAccessTrapEL2(const MiscRegIndex miscReg,
                                                ThreadContext *tc);
bool isGenericTimerPhysEL0SystemAccessTrapEL2(const MiscRegIndex miscReg,
                                              ThreadContext *tc);
bool isGenericTimerPhysEL1SystemAccessTrapEL2(const MiscRegIndex miscReg,
                                              ThreadContext *tc);
bool isGenericTimerVirtSystemAccessTrapEL2(const MiscRegIndex miscReg,
                                           ThreadContext *tc);
bool condGenericTimerCommonEL0SystemAccessTrapEL2(const MiscRegIndex miscReg,
                                                  ThreadContext *tc);
bool condGenericTimerCommonEL1SystemAccessTrapEL2(const MiscRegIndex miscReg,
                                                  ThreadContext *tc);
bool condGenericTimerPhysEL1SystemAccessTrapEL2(const MiscRegIndex miscReg,
                                                ThreadContext *tc);
bool isGenericTimerSystemAccessTrapEL3(const MiscRegIndex miscReg,
                                       ThreadContext *tc);

bool SPAlignmentCheckEnabled(ThreadContext *tc);

Addr truncPage(Addr addr);
Addr roundPage(Addr addr);

// Decodes the register index to access based on the fields used in a MSR
// or MRS instruction
bool decodeMrsMsrBankedReg(uint8_t sysM, bool r, bool &isIntReg, int &regIdx,
                           CPSR cpsr, SCR scr, NSACR nsacr,
                           bool checkSecurity=true);

// This wrapper function is used to turn the register index into a source
// parameter for the instruction. See Operands.isa
static inline int
decodeMrsMsrBankedIntRegIndex(uint8_t sysM, bool r)
{
    int  regIdx;
    bool isIntReg;
    bool validReg;

    validReg = decodeMrsMsrBankedReg(
            sysM, r, isIntReg, regIdx, 0, 0, 0, false);
    return (validReg && isIntReg) ? regIdx : int_reg::Zero;
}

/**
 * Returns the n. of PA bits corresponding to the specified encoding.
 */
int decodePhysAddrRange64(uint8_t pa_enc);

/**
 * Returns the encoding corresponding to the specified n. of PA bits.
 */
uint8_t encodePhysAddrRange64(int pa_size);

inline ByteOrder
byteOrder(const ThreadContext *tc)
{
    return isBigEndian64(tc) ? ByteOrder::big : ByteOrder::little;
};

bool isUnpriviledgeAccess(ThreadContext *tc);

void syncVecRegsToElems(ThreadContext *tc);
void syncVecElemsToRegs(ThreadContext *tc);

bool fgtEnabled(ThreadContext *tc);
bool isHcrxEL2Enabled(ThreadContext *tc);

TranslationRegime translationRegime(ThreadContext *tc, ExceptionLevel el);
ExceptionLevel translationEl(TranslationRegime regime);

static inline bool
useVMID(TranslationRegime regime)
{
    return regime == TranslationRegime::EL10;
}

} // namespace ArmISA
} // namespace gem5

#endif
