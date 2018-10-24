/*
 * Copyright (c) 2011-2013,2017 ARM Limited
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
 *
 * Authors: Gabe Black
 */
#ifndef __ARCH_ARM_MEM64_HH__
#define __ARCH_ARM_MEM64_HH__

#include "arch/arm/insts/misc64.hh"
#include "arch/arm/insts/static_inst.hh"

namespace ArmISA
{

class SysDC64 : public MiscRegOp64
{
  protected:
    IntRegIndex base;
    IntRegIndex dest;
    uint64_t imm;

    SysDC64(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
            IntRegIndex _base, MiscRegIndex miscReg, uint64_t _imm)
        : MiscRegOp64(mnem, _machInst, __opClass, false),
          base(_base), dest((IntRegIndex)miscReg), imm(_imm)
    {}

    std::string generateDisassembly(
            Addr pc, const SymbolTable *symtab) const override;
};

class MightBeMicro64 : public ArmStaticInst
{
  protected:
    MightBeMicro64(const char *mnem, ExtMachInst _machInst, OpClass __opClass)
        : ArmStaticInst(mnem, _machInst, __opClass)
    {}

    void
    advancePC(PCState &pcState) const
    {
        if (flags[IsLastMicroop]) {
            pcState.uEnd();
        } else if (flags[IsMicroop]) {
            pcState.uAdvance();
        } else {
            pcState.advance();
        }
    }
};

class Memory64 : public MightBeMicro64
{
  public:
    enum AddrMode {
        AddrMd_Offset,
        AddrMd_PreIndex,
        AddrMd_PostIndex
    };

  protected:

    IntRegIndex dest;
    IntRegIndex base;
    /// True if the base register is SP (used for SP alignment checking).
    bool baseIsSP;
    static const unsigned numMicroops = 3;

    StaticInstPtr *uops;

    Memory64(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
             IntRegIndex _dest, IntRegIndex _base)
        : MightBeMicro64(mnem, _machInst, __opClass),
          dest(_dest), base(_base), uops(NULL), memAccessFlags(0)
    {
        baseIsSP = isSP(_base);
    }

    virtual
    ~Memory64()
    {
        delete [] uops;
    }

    StaticInstPtr
    fetchMicroop(MicroPC microPC) const override
    {
        assert(uops != NULL && microPC < numMicroops);
        return uops[microPC];
    }

    void startDisassembly(std::ostream &os) const;

    unsigned memAccessFlags;

    void setExcAcRel(bool exclusive, bool acrel);
};

class MemoryImm64 : public Memory64
{
  protected:
    int64_t imm;

    MemoryImm64(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                IntRegIndex _dest, IntRegIndex _base, int64_t _imm)
        : Memory64(mnem, _machInst, __opClass, _dest, _base), imm(_imm)
    {}

    std::string generateDisassembly(
            Addr pc, const SymbolTable *symtab) const override;
};

class MemoryDImm64 : public MemoryImm64
{
  protected:
    IntRegIndex dest2;

    MemoryDImm64(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                IntRegIndex _dest, IntRegIndex _dest2, IntRegIndex _base,
                int64_t _imm)
        : MemoryImm64(mnem, _machInst, __opClass, _dest, _base, _imm),
          dest2(_dest2)
    {}

    std::string generateDisassembly(
            Addr pc, const SymbolTable *symtab) const override;
};

class MemoryDImmEx64 : public MemoryDImm64
{
  protected:
    IntRegIndex result;

    MemoryDImmEx64(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                 IntRegIndex _result, IntRegIndex _dest, IntRegIndex _dest2,
                 IntRegIndex _base, int32_t _imm)
        : MemoryDImm64(mnem, _machInst, __opClass, _dest, _dest2,
                     _base, _imm), result(_result)
    {}

    std::string generateDisassembly(
            Addr pc, const SymbolTable *symtab) const override;
};

class MemoryPreIndex64 : public MemoryImm64
{
  protected:
    MemoryPreIndex64(const char *mnem, ExtMachInst _machInst,
                     OpClass __opClass, IntRegIndex _dest, IntRegIndex _base,
                     int64_t _imm)
        : MemoryImm64(mnem, _machInst, __opClass, _dest, _base, _imm)
    {}

    std::string generateDisassembly(
            Addr pc, const SymbolTable *symtab) const override;
};

class MemoryPostIndex64 : public MemoryImm64
{
  protected:
    MemoryPostIndex64(const char *mnem, ExtMachInst _machInst,
                      OpClass __opClass, IntRegIndex _dest, IntRegIndex _base,
                      int64_t _imm)
        : MemoryImm64(mnem, _machInst, __opClass, _dest, _base, _imm)
    {}

    std::string generateDisassembly(
            Addr pc, const SymbolTable *symtab) const override;
};

class MemoryReg64 : public Memory64
{
  protected:
    IntRegIndex offset;
    ArmExtendType type;
    uint64_t shiftAmt;

    MemoryReg64(const char *mnem, ExtMachInst _machInst,
                OpClass __opClass, IntRegIndex _dest, IntRegIndex _base,
                IntRegIndex _offset, ArmExtendType _type,
                uint64_t _shiftAmt)
        : Memory64(mnem, _machInst, __opClass, _dest, _base),
          offset(_offset), type(_type), shiftAmt(_shiftAmt)
    {}

    std::string generateDisassembly(
            Addr pc, const SymbolTable *symtab) const override;
};

class MemoryRaw64 : public Memory64
{
  protected:
    MemoryRaw64(const char *mnem, ExtMachInst _machInst,
                OpClass __opClass, IntRegIndex _dest, IntRegIndex _base)
        : Memory64(mnem, _machInst, __opClass, _dest, _base)
    {}

    std::string generateDisassembly(
            Addr pc, const SymbolTable *symtab) const override;
};

class MemoryEx64 : public Memory64
{
  protected:
    IntRegIndex result;

    MemoryEx64(const char *mnem, ExtMachInst _machInst,
               OpClass __opClass, IntRegIndex _dest, IntRegIndex _base,
               IntRegIndex _result)
        : Memory64(mnem, _machInst, __opClass, _dest, _base), result(_result)
    {}

    std::string generateDisassembly(
            Addr pc, const SymbolTable *symtab) const override;
};

class MemoryLiteral64 : public Memory64
{
  protected:
    int64_t imm;

    MemoryLiteral64(const char *mnem, ExtMachInst _machInst,
                    OpClass __opClass, IntRegIndex _dest, int64_t _imm)
        : Memory64(mnem, _machInst, __opClass, _dest, INTREG_ZERO), imm(_imm)
    {}

    std::string generateDisassembly(
            Addr pc, const SymbolTable *symtab) const override;
};
}

#endif //__ARCH_ARM_INSTS_MEM_HH__
