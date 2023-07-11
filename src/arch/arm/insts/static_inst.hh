/*
 * Copyright (c) 2010-2013,2016-2018, 2022 Arm Limited
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

#ifndef __ARCH_ARM_INSTS_STATICINST_HH__
#define __ARCH_ARM_INSTS_STATICINST_HH__

#include <memory>

#include "arch/arm/faults.hh"
#include "arch/arm/utility.hh"
#include "arch/arm/isa.hh"
#include "arch/arm/pcstate.hh"
#include "arch/arm/self_debug.hh"
#include "arch/arm/system.hh"
#include "base/trace.hh"
#include "cpu/exec_context.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "sim/byteswap.hh"
#include "sim/full_system.hh"

namespace gem5
{

namespace ArmISA
{

class ArmStaticInst : public StaticInst
{
  protected:
    bool aarch64;
    uint8_t intWidth;

    int32_t shift_rm_imm(uint32_t base, uint32_t shamt,
                         uint32_t type, uint32_t cfval) const;
    int32_t shift_rm_rs(uint32_t base, uint32_t shamt,
                        uint32_t type, uint32_t cfval) const;

    bool shift_carry_imm(uint32_t base, uint32_t shamt,
                         uint32_t type, uint32_t cfval) const;
    bool shift_carry_rs(uint32_t base, uint32_t shamt,
                        uint32_t type, uint32_t cfval) const;

    int64_t shiftReg64(uint64_t base, uint64_t shiftAmt,
                       ArmShiftType type, uint8_t width) const;
    int64_t extendReg64(uint64_t base, ArmExtendType type,
                        uint64_t shiftAmt, uint8_t width) const;

    template<int width>
    static inline bool
    saturateOp(int32_t &res, int64_t op1, int64_t op2, bool sub=false)
    {
        int64_t midRes = sub ? (op1 - op2) : (op1 + op2);
        if (bits(midRes, width) != bits(midRes, width - 1)) {
            if (midRes > 0)
                res = (1LL << (width - 1)) - 1;
            else
                res = -(1LL << (width - 1));
            return true;
        } else {
            res = midRes;
            return false;
        }
    }

    static inline bool
    satInt(int32_t &res, int64_t op, int width)
    {
        width--;
        if (op >= (1LL << width)) {
            res = (1LL << width) - 1;
            return true;
        } else if (op < -(1LL << width)) {
            res = -(1LL << width);
            return true;
        } else {
            res = op;
            return false;
        }
    }

    template<int width>
    static inline bool
    uSaturateOp(uint32_t &res, int64_t op1, int64_t op2, bool sub=false)
    {
        int64_t midRes = sub ? (op1 - op2) : (op1 + op2);
        if (midRes >= (1LL << width)) {
            res = (1LL << width) - 1;
            return true;
        } else if (midRes < 0) {
            res = 0;
            return true;
        } else {
            res = midRes;
            return false;
        }
    }

    static inline bool
    uSatInt(int32_t &res, int64_t op, int width)
    {
        if (op >= (1LL << width)) {
            res = (1LL << width) - 1;
            return true;
        } else if (op < 0) {
            res = 0;
            return true;
        } else {
            res = op;
            return false;
        }
    }

    ExtMachInst machInst;

    // Constructor
    ArmStaticInst(const char *mnem, ExtMachInst _machInst,
                  OpClass __opClass)
        : StaticInst(mnem, __opClass), machInst(_machInst)
    {
        aarch64 = machInst.aarch64;
        if (bits(machInst, 28, 24) == 0x10)
            intWidth = 64;  // Force 64-bit width for ADR/ADRP
        else
            intWidth = (aarch64 && bits(machInst, 31)) ? 64 : 32;
    }

    /// Print a register name for disassembly given the unique
    /// dependence tag number (FP or int).
    void printIntReg(std::ostream &os, RegIndex reg_idx,
                     uint8_t opWidth = 0) const;
    void printFloatReg(std::ostream &os, RegIndex reg_idx) const;
    void printVecReg(std::ostream &os, RegIndex reg_idx,
                     bool isSveVecReg = false) const;
    void printVecPredReg(std::ostream &os, RegIndex reg_idx) const;
    void printCCReg(std::ostream &os, RegIndex reg_idx) const;
    void printMiscReg(std::ostream &os, RegIndex reg_idx) const;
    void printMnemonic(std::ostream &os,
                       const std::string &suffix = "",
                       bool withPred = true,
                       bool withCond64 = false,
                       ConditionCode cond64 = COND_UC) const;
    void printTarget(std::ostream &os, Addr target,
                     const loader::SymbolTable *symtab) const;
    void printCondition(std::ostream &os, unsigned code,
                        bool noImplicit=false) const;
    void printMemSymbol(std::ostream &os, const loader::SymbolTable *symtab,
                        const std::string &prefix, const Addr addr,
                        const std::string &suffix) const;
    void printShiftOperand(std::ostream &os, RegIndex rm,
                           bool immShift, uint32_t shiftAmt,
                           RegIndex rs, ArmShiftType type) const;
    void printExtendOperand(bool firstOperand, std::ostream &os,
                            RegIndex rm, ArmExtendType type,
                            int64_t shiftAmt) const;
    void printPFflags(std::ostream &os, int flag) const;

    void printDataInst(std::ostream &os, bool withImm) const;
    void printDataInst(std::ostream &os, bool withImm, bool immShift, bool s,
                       RegIndex rd, RegIndex rn, RegIndex rm,
                       RegIndex rs, uint32_t shiftAmt, ArmShiftType type,
                       uint64_t imm) const;

    void
    advancePC(PCStateBase &pcState) const override
    {
        pcState.as<PCState>().advance();
    }

    void
    advancePC(ThreadContext *tc) const override
    {
        PCState pc = tc->pcState().as<PCState>();
        pc.advance();
        tc->pcState(pc);
    }

    uint64_t getEMI() const override { return machInst; }

    std::unique_ptr<PCStateBase>
    buildRetPC(const PCStateBase &cur_pc,
            const PCStateBase &call_pc) const override
    {
        PCStateBase *ret_pc = call_pc.clone();
        ret_pc->as<PCState>().uEnd();
        return std::unique_ptr<PCStateBase>{ret_pc};
    }

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;

    static void
    activateBreakpoint(ThreadContext *tc)
    {
        SelfDebug *sd = ArmISA::ISA::getSelfDebug(tc);
        sd->activateDebug();
    }

    static inline uint32_t
    cpsrWriteByInstr(CPSR cpsr, uint32_t val, SCR scr, NSACR nsacr,
            uint8_t byteMask, bool affectState, bool nmfi, ThreadContext *tc)
    {
        bool privileged   = (cpsr.mode != MODE_USER);
        bool haveVirt     = ArmSystem::haveEL(tc, EL2);
        bool isSecure     = ArmISA::isSecure(tc);

        uint32_t bitMask = 0;

        if (affectState && byteMask==0xF){
            activateBreakpoint(tc);
        }
        if (bits(byteMask, 3)) {
            unsigned lowIdx = affectState ? 24 : 27;
            bitMask = bitMask | mask(31, lowIdx);
        }
        if (bits(byteMask, 2)) {
            bitMask = bitMask | mask(19, 16);
        }
        if (bits(byteMask, 1)) {
            unsigned highIdx = affectState ? 15 : 9;
            unsigned lowIdx = (privileged && (isSecure || scr.aw || haveVirt))
                            ? 8 : 9;
            bitMask = bitMask | mask(highIdx, lowIdx);
        }
        if (bits(byteMask, 0)) {
            if (privileged) {
                bitMask |= 1 << 7;
                if ( (!nmfi || !((val >> 6) & 0x1)) &&
                     (isSecure || scr.fw || haveVirt) ) {
                    bitMask |= 1 << 6;
                }
                // Now check the new mode is allowed
                OperatingMode newMode = (OperatingMode) (val & mask(5));
                OperatingMode oldMode = (OperatingMode)(uint32_t)cpsr.mode;
                if (!badMode(tc, newMode)) {
                    bool validModeChange = true;
                    // Check for attempts to enter modes only permitted in
                    // Secure state from Non-secure state. These are Monitor
                    // mode ('10110'), and FIQ mode ('10001') if the Security
                    // Extensions have reserved it.
                    if (!isSecure && newMode == MODE_MON)
                        validModeChange = false;
                    if (!isSecure && newMode == MODE_FIQ && nsacr.rfr == '1')
                        validModeChange = false;
                    // There is no Hyp mode ('11010') in Secure state, so that
                    // is UNPREDICTABLE
                    if (scr.ns == 0 && newMode == MODE_HYP)
                        validModeChange = false;
                    // Cannot move into Hyp mode directly from a Non-secure
                    // PL1 mode
                    if (!isSecure && oldMode != MODE_HYP && newMode == MODE_HYP)
                        validModeChange = false;
                    // Cannot move out of Hyp mode with this function except
                    // on an exception return
                    if (oldMode == MODE_HYP && newMode != MODE_HYP && !affectState)
                        validModeChange = false;
                    // Must not change to 64 bit when running in 32 bit mode
                    if (!opModeIs64(oldMode) && opModeIs64(newMode))
                        validModeChange = false;

                    // If we passed all of the above then set the bit mask to
                    // copy the mode accross
                    if (validModeChange) {
                        bitMask = bitMask | mask(5);
                    } else {
                        warn_once("Illegal change to CPSR mode attempted\n");
                    }
                } else {
                    warn_once("Ignoring write of bad mode to CPSR.\n");
                }
            }
            if (affectState)
                bitMask = bitMask | (1 << 5);
        }

        return ((uint32_t)cpsr & ~bitMask) | (val & bitMask);
    }

    static inline uint32_t
    spsrWriteByInstr(uint32_t spsr, uint32_t val,
            uint8_t byteMask, bool affectState)
    {
        uint32_t bitMask = 0;

        if (bits(byteMask, 3))
            bitMask = bitMask | mask(31, 24);
        if (bits(byteMask, 2))
            bitMask = bitMask | mask(19, 16);
        if (bits(byteMask, 1))
            bitMask = bitMask | mask(15, 8);
        if (bits(byteMask, 0))
            bitMask = bitMask | mask(7, 0);

        return ((spsr & ~bitMask) | (val & bitMask));
    }

    static inline Addr
    readPC(ExecContext *xc)
    {
        return xc->pcState().as<PCState>().instPC();
    }

    static inline void
    setNextPC(ExecContext *xc, Addr val)
    {
        PCState pc = xc->pcState().as<PCState>();
        pc.instNPC(val);
        xc->pcState(pc);
    }

    template<class T>
    static inline T
    cSwap(T val, bool big)
    {
        if (big) {
            return letobe(val);
        } else {
            return val;
        }
    }

    template<class T, class E>
    static inline T
    cSwap(T val, bool big)
    {
        const unsigned count = sizeof(T) / sizeof(E);
        union
        {
            T tVal;
            E eVals[count];
        } conv;
        conv.tVal = htole(val);
        if (big) {
            for (unsigned i = 0; i < count; i++) {
                conv.eVals[i] = letobe(conv.eVals[i]);
            }
        } else {
            for (unsigned i = 0; i < count; i++) {
                conv.eVals[i] = conv.eVals[i];
            }
        }
        return letoh(conv.tVal);
    }

    // Perform an interworking branch.
    static inline void
    setIWNextPC(ExecContext *xc, Addr val)
    {
        PCState pc = xc->pcState().as<PCState>();
        pc.instIWNPC(val);
        xc->pcState(pc);
    }

    // Perform an interworking branch in ARM mode, a regular branch
    // otherwise.
    static inline void
    setAIWNextPC(ExecContext *xc, Addr val)
    {
        PCState pc = xc->pcState().as<PCState>();
        pc.instAIWNPC(val);
        xc->pcState(pc);
    }

    inline Fault disabledFault() const { return undefined(true); }

    // Utility function used by checkForWFxTrap32 and checkForWFxTrap64
    // Returns true if processor has to trap a WFI/WFE instruction.
    bool isWFxTrapping(ThreadContext *tc,
                       ExceptionLevel targetEL, bool isWfe) const;

    /**
     * Trigger a Software Breakpoint.
     *
     * See aarch32/exceptions/debug/AArch32.SoftwareBreakpoint in the
     * ARM ARM psueodcode library.
     */
    Fault softwareBreakpoint32(ExecContext *xc, uint16_t imm) const;

    /**
     * Trap an access to Advanced SIMD or FP registers due to access
     * control bits.
     *
     * See aarch64/exceptions/traps/AArch64.AdvSIMDFPAccessTrap in the
     * ARM ARM psueodcode library.
     *
     * @param el Target EL for the trap
     */
    Fault advSIMDFPAccessTrap64(ExceptionLevel el) const;


    /**
     * Check an Advaned SIMD access against CPTR_EL2 and CPTR_EL3.
     *
     * See aarch64/exceptions/traps/AArch64.CheckFPAdvSIMDTrap in the
     * ARM ARM psueodcode library.
     */
    Fault checkFPAdvSIMDTrap64(ThreadContext *tc, CPSR cpsr) const;

    /**
     * Check an Advaned SIMD access against CPACR_EL1, CPTR_EL2, and
     * CPTR_EL3.
     *
     * See aarch64/exceptions/traps/AArch64.CheckFPAdvSIMDEnabled in the
     * ARM ARM psueodcode library.
     */
    Fault checkFPAdvSIMDEnabled64(ThreadContext *tc,
                                  CPSR cpsr, CPACR cpacr) const;

    /**
     * Check if a VFP/SIMD access from aarch32 should be allowed.
     *
     * See aarch32/exceptions/traps/AArch32.CheckAdvSIMDOrFPEnabled in the
     * ARM ARM psueodcode library.
     */
    Fault checkAdvSIMDOrFPEnabled32(ThreadContext *tc,
                                    CPSR cpsr, CPACR cpacr,
                                    NSACR nsacr, FPEXC fpexc,
                                    bool fpexc_check, bool advsimd) const;

    /**
     * Check if WFE/WFI instruction execution in aarch32 should be trapped.
     *
     * See aarch32/exceptions/traps/AArch32.checkForWFxTrap in the
     * ARM ARM psueodcode library.
     */
    Fault checkForWFxTrap32(ThreadContext *tc,
                            ExceptionLevel tgtEl, bool isWfe) const;

    /**
     * Check if WFE/WFI instruction execution in aarch64 should be trapped.
     *
     * See aarch64/exceptions/traps/AArch64.checkForWFxTrap in the
     * ARM ARM psueodcode library.
     */
    Fault checkForWFxTrap64(ThreadContext *tc,
                            ExceptionLevel tgtEl, bool isWfe) const;

    /**
     * WFE/WFI trapping helper function.
     */
    Fault trapWFx(ThreadContext *tc, CPSR cpsr, SCR scr, bool isWfe) const;

    /**
     * Check if SETEND instruction execution in aarch32 should be trapped.
     *
     * See aarch32/exceptions/traps/AArch32.CheckSETENDEnabled in the
     * ARM ARM pseudocode library.
     */
    Fault checkSETENDEnabled(ThreadContext *tc, CPSR cpsr) const;

    /**
     * UNDEFINED behaviour in AArch32
     *
     * See aarch32/exceptions/traps/AArch32.UndefinedFault in the
     * ARM ARM pseudocode library.
     */
    Fault undefinedFault32(ThreadContext *tc, ExceptionLevel el) const;

    /**
     * UNDEFINED behaviour in AArch64
     *
     * See aarch64/exceptions/traps/AArch64.UndefinedFault in the
     * ARM ARM pseudocode library.
     */
    Fault undefinedFault64(ThreadContext *tc, ExceptionLevel el) const;

    /**
     * Trap an access to SVE registers due to access control bits.
     *
     * @param el Target EL for the trap.
     */
    Fault sveAccessTrap(ExceptionLevel el) const;

    /**
     * Check an SVE access against CPACR_EL1, CPTR_EL2, and CPTR_EL3.
     */
    Fault checkSveEnabled(ThreadContext *tc, CPSR cpsr, CPACR cpacr) const;


    /**
     * Trap an access to SME registers due to access control bits.
     *
     * @param el Target EL for the trap.
     * @param iss ISS to be used for the trap.
     */
    Fault smeAccessTrap(ExceptionLevel el, uint32_t iss = 0) const;

    /**
     * Check if SME is enabled by checking the SME and FP bits of
     * CPACR_EL1, CPTR_EL2, and CPTR_EL3
     */
    Fault checkSmeEnabled(ThreadContext *tc, CPSR cpsr, CPACR cpacr) const;

    /**
     * Check an SME access against CPACR_EL1, CPTR_EL2, and CPTR_EL3.
     * This is purely used from the management instructions as it should
     * be possible to call SMSTART/SMSTOP without having the floating
     * point flags correctly set up.
     */
    Fault checkSmeAccess(ThreadContext *tc, CPSR cpsr, CPACR cpacr) const;

    /**
     * Check an SVE access against CPACR_EL1, CPTR_EL2, and CPTR_EL3, but
     * choosing the correct set of traps to check based on Streaming Mode
     */
    Fault checkSveSmeEnabled(ThreadContext *tc, CPSR cpsr, CPACR cpacr) const;

    /**
     * Get the new PSTATE from a SPSR register in preparation for an
     * exception return.
     *
     * See shared/functions/system/SetPSTATEFromPSR in the ARM ARM
     * pseudocode library.
     */
    CPSR getPSTATEFromPSR(ThreadContext *tc, CPSR cpsr, CPSR spsr) const;

    /**
     * Return true if exceptions normally routed to EL1 are being handled
     * at an Exception level using AArch64, because either EL1 is using
     * AArch64 or TGE is in force and EL2 is using AArch64.
     *
     * See aarch32/exceptions/exceptions/AArch32.GeneralExceptionsToAArch64
     * in the ARM ARM pseudocode library.
     */
    bool generalExceptionsToAArch64(ThreadContext *tc,
                                    ExceptionLevel pstateEL) const;

  public:
    virtual void
    annotateFault(ArmFault *fault) {}

    uint8_t
    getIntWidth() const
    {
        return intWidth;
    }

    /** Returns the byte size of current instruction */
    ssize_t
    instSize() const
    {
        return (!machInst.thumb || machInst.bigThumb) ? 4 : 2;
    }

    /**
     * Returns the real encoding of the instruction:
     * the machInst field is in fact always 64 bit wide and
     * contains some instruction metadata, which means it differs
     * from the real opcode.
     */
    MachInst
    encoding() const
    {
        return static_cast<MachInst>(machInst & (mask(instSize() * 8)));
    }

    size_t
    asBytes(void *buf, size_t max_size) override
    {
        return simpleAsBytes(buf, max_size, machInst);
    }

    static unsigned getCurSveVecLenInBits(ThreadContext *tc);

    static unsigned
    getCurSveVecLenInQWords(ThreadContext *tc)
    {
        return getCurSveVecLenInBits(tc) >> 6;
    }

    template<typename T>
    static unsigned
    getCurSveVecLen(ThreadContext *tc)
    {
        return getCurSveVecLenInBits(tc) / (8 * sizeof(T));
    }

    static unsigned getCurSmeVecLenInBits(ThreadContext *tc);

    static unsigned
    getCurSmeVecLenInQWords(ThreadContext *tc)
    {
        return getCurSmeVecLenInBits(tc) >> 6;
    }

    template<typename T>
    static unsigned
    getCurSmeVecLen(ThreadContext *tc)
    {
        return getCurSmeVecLenInBits(tc) / (8 * sizeof(T));
    }

    inline Fault
    undefined(bool disabled=false) const
    {
        return std::make_shared<UndefinedInstruction>(
            machInst, false, mnemonic, disabled);
    }
};

} // namespace ArmISA
} // namespace gem5

#endif //__ARCH_ARM_INSTS_STATICINST_HH__
