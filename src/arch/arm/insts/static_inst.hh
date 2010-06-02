/*
 * Copyright (c) 2010 ARM Limited
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
 *
 * Authors: Stephen Hines
 */
#ifndef __ARCH_ARM_INSTS_STATICINST_HH__
#define __ARCH_ARM_INSTS_STATICINST_HH__

#include "base/trace.hh"
#include "cpu/static_inst.hh"

namespace ArmISA
{
class ArmStaticInst : public StaticInst
{
  protected:
    int32_t shift_rm_imm(uint32_t base, uint32_t shamt,
                         uint32_t type, uint32_t cfval) const;
    int32_t shift_rm_rs(uint32_t base, uint32_t shamt,
                        uint32_t type, uint32_t cfval) const;

    bool shift_carry_imm(uint32_t base, uint32_t shamt,
                         uint32_t type, uint32_t cfval) const;
    bool shift_carry_rs(uint32_t base, uint32_t shamt,
                        uint32_t type, uint32_t cfval) const;

    // Constructor
    ArmStaticInst(const char *mnem, ExtMachInst _machInst,
                  OpClass __opClass)
        : StaticInst(mnem, _machInst, __opClass)
    {
    }

    inline static std::string
    inst2string(MachInst machInst)
    {
        std::string str = "";
        uint32_t mask = (1 << 31);

        while (mask) {
            str += ((machInst & mask) ? "1" : "0");
            mask = mask >> 1;
        }

        return str;
    }

    /// Print a register name for disassembly given the unique
    /// dependence tag number (FP or int).
    void printReg(std::ostream &os, int reg) const;
    void printMnemonic(std::ostream &os,
                       const std::string &suffix = "",
                       bool withPred = true) const;
    void printMemSymbol(std::ostream &os, const SymbolTable *symtab,
                        const std::string &prefix, const Addr addr,
                        const std::string &suffix) const;
    void printShiftOperand(std::ostream &os, IntRegIndex rm,
                           bool immShift, uint32_t shiftAmt,
                           IntRegIndex rs, ArmShiftType type) const;


    void printDataInst(std::ostream &os, bool withImm) const;
    void printDataInst(std::ostream &os, bool withImm, bool immShift, bool s,
                       IntRegIndex rd, IntRegIndex rn, IntRegIndex rm,
                       IntRegIndex rs, uint32_t shiftAmt, ArmShiftType type,
                       uint32_t imm) const;

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;

    static uint32_t
    cpsrWriteByInstr(CPSR cpsr, uint32_t val,
            uint8_t byteMask, bool affectState)
    {
        bool privileged = (cpsr.mode != MODE_USER);

        uint32_t bitMask = 0;

        if (bits(byteMask, 3)) {
            unsigned lowIdx = affectState ? 24 : 27;
            bitMask = bitMask | mask(31, lowIdx);
        }
        if (bits(byteMask, 2)) {
            bitMask = bitMask | mask(19, 16);
        }
        if (bits(byteMask, 1)) {
            unsigned highIdx = affectState ? 15 : 9;
            unsigned lowIdx = privileged ? 8 : 9;
            bitMask = bitMask | mask(highIdx, lowIdx);
        }
        if (bits(byteMask, 0)) {
            if (privileged) {
                bitMask = bitMask | mask(7, 6);
                bitMask = bitMask | mask(5);
            }
            if (affectState)
                bitMask = bitMask | (1 << 5);
        }

        return ((uint32_t)cpsr & ~bitMask) | (val & bitMask);
    }

    static uint32_t
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

    template<class XC>
    static Addr
    readPC(XC *xc)
    {
        Addr pc = xc->readPC();
        Addr tBit = pc & (ULL(1) << PcTBitShift);
        if (tBit)
            return pc + 4;
        else
            return pc + 8;
    }

    // Perform an regular branch.
    template<class XC>
    static void
    setNextPC(XC *xc, Addr val)
    {
        xc->setNextPC((xc->readNextPC() & PcModeMask) |
                      (val & ~PcModeMask));
    }

    // Perform an interworking branch.
    template<class XC>
    static void
    setIWNextPC(XC *xc, Addr val)
    {
        Addr stateBits = xc->readPC() & PcModeMask;
        Addr jBit = (ULL(1) << PcJBitShift);
        Addr tBit = (ULL(1) << PcTBitShift);
        bool thumbEE = (stateBits == (tBit | jBit));

        Addr newPc = (val & ~PcModeMask);
        if (thumbEE) {
            if (bits(newPc, 0)) {
                warn("Bad thumbEE interworking branch address %#x.\n", newPc);
            } else {
                newPc = newPc & ~mask(1);
            }
        } else {
            if (bits(newPc, 0)) {
                stateBits = tBit;
                newPc = newPc & ~mask(1);
            } else if (!bits(newPc, 1)) {
                stateBits = 0;
            } else {
                warn("Bad interworking branch address %#x.\n", newPc);
            }
        }
        newPc = newPc | stateBits;
        xc->setNextPC(newPc);
    }

    // Perform an interworking branch in ARM mode, a regular branch
    // otherwise.
    template<class XC>
    static void
    setAIWNextPC(XC *xc, Addr val)
    {
        Addr stateBits = xc->readPC() & PcModeMask;
        Addr jBit = (ULL(1) << PcJBitShift);
        Addr tBit = (ULL(1) << PcTBitShift);
        if (!jBit && !tBit) {
            setIWNextPC(xc, val);
        } else {
            setNextPC(xc, val);
        }
    }
};
}

#endif //__ARCH_ARM_INSTS_STATICINST_HH__
