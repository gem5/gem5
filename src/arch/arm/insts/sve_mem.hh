/*
 * Copyright (c) 2017 ARM Limited
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

#ifndef __ARCH_ARM_SVE_MEM_HH__
#define __ARCH_ARM_SVE_MEM_HH__

#include "arch/arm/insts/static_inst.hh"
#include "arch/arm/tlb.hh"

namespace gem5
{

namespace ArmISA
{

class SveMemVecFillSpill : public ArmStaticInst
{
  protected:
    RegIndex dest;
    RegIndex base;
    uint64_t imm;

    /// True if the base register is SP (used for SP alignment checking).
    bool baseIsSP;

    unsigned memAccessFlags;

    SveMemVecFillSpill(const char *mnem, ExtMachInst _machInst,
                       OpClass __opClass, RegIndex _dest,
                       RegIndex _base, uint64_t _imm)
        : ArmStaticInst(mnem, _machInst, __opClass),
          dest(_dest), base(_base), imm(_imm),
          memAccessFlags(ArmISA::MMU::AllowUnaligned)
    {
        baseIsSP = isSP(_base);
    }

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

class SveMemPredFillSpill : public ArmStaticInst
{
  protected:
    RegIndex dest;
    RegIndex base;
    uint64_t imm;

    /// True if the base register is SP (used for SP alignment checking).
    bool baseIsSP;

    unsigned memAccessFlags;

    SveMemPredFillSpill(const char *mnem, ExtMachInst _machInst,
                        OpClass __opClass, RegIndex _dest,
                        RegIndex _base, uint64_t _imm)
        : ArmStaticInst(mnem, _machInst, __opClass),
          dest(_dest), base(_base), imm(_imm),
          memAccessFlags(ArmISA::MMU::AllowUnaligned)
    {
        baseIsSP = isSP(_base);
    }

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

class SveContigMemSS : public ArmStaticInst
{
  protected:
    RegIndex dest;
    RegIndex gp;
    RegIndex base;
    RegIndex offset;

    /// True if the base register is SP (used for SP alignment checking).
    bool baseIsSP;

    unsigned memAccessFlags;

    SveContigMemSS(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                   RegIndex _dest, RegIndex _gp, RegIndex _base,
                   RegIndex _offset)
        : ArmStaticInst(mnem, _machInst, __opClass),
          dest(_dest), gp(_gp), base(_base), offset(_offset),
          memAccessFlags(ArmISA::MMU::AllowUnaligned)
    {
        baseIsSP = isSP(_base);
    }

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

class SveContigMemSI : public ArmStaticInst
{
  protected:
    RegIndex dest;
    RegIndex gp;
    RegIndex base;
    uint64_t imm;

    /// True if the base register is SP (used for SP alignment checking).
    bool baseIsSP;

    unsigned memAccessFlags;

    SveContigMemSI(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                   RegIndex _dest, RegIndex _gp, RegIndex _base,
                   uint64_t _imm)
        : ArmStaticInst(mnem, _machInst, __opClass),
          dest(_dest), gp(_gp), base(_base), imm(_imm),
          memAccessFlags(ArmISA::MMU::AllowUnaligned)
    {
        baseIsSP = isSP(_base);
    }

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

} // namespace ArmISA
} // namespace gem5

#endif  // __ARCH_ARM_SVE_MEM_HH__
