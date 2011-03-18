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
#ifndef __ARCH_ARM_MEM_HH__
#define __ARCH_ARM_MEM_HH__

#include "arch/arm/insts/pred_inst.hh"

namespace ArmISA
{

class Swap : public PredOp
{
  protected:
    IntRegIndex dest;
    IntRegIndex op1;
    IntRegIndex base;

    Swap(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
         IntRegIndex _dest, IntRegIndex _op1, IntRegIndex _base)
        : PredOp(mnem, _machInst, __opClass),
          dest(_dest), op1(_op1), base(_base)
    {}

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

class MightBeMicro : public PredOp
{
  protected:
    MightBeMicro(const char *mnem, ExtMachInst _machInst, OpClass __opClass)
        : PredOp(mnem, _machInst, __opClass)
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

// The address is a base register plus an immediate.
class RfeOp : public MightBeMicro
{
  public:
    enum AddrMode {
        DecrementAfter,
        DecrementBefore,
        IncrementAfter,
        IncrementBefore
    };
  protected:
    IntRegIndex base;
    AddrMode mode;
    bool wb;
    IntRegIndex ura, urb, urc;
    static const unsigned numMicroops = 3;

    StaticInstPtr *uops;

    RfeOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
          IntRegIndex _base, AddrMode _mode, bool _wb)
        : MightBeMicro(mnem, _machInst, __opClass),
          base(_base), mode(_mode), wb(_wb),
          ura(INTREG_UREG0), urb(INTREG_UREG1),
          urc(INTREG_UREG2),
          uops(NULL)
    {}

    virtual
    ~RfeOp()
    {
        delete [] uops;
    }

    StaticInstPtr
    fetchMicroop(MicroPC microPC) const
    {
        assert(uops != NULL && microPC < numMicroops);
        return uops[microPC];
    }

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

// The address is a base register plus an immediate.
class SrsOp : public MightBeMicro
{
  public:
    enum AddrMode {
        DecrementAfter,
        DecrementBefore,
        IncrementAfter,
        IncrementBefore
    };
  protected:
    uint32_t regMode;
    AddrMode mode;
    bool wb;
    static const unsigned numMicroops = 2;

    StaticInstPtr *uops;

    SrsOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
          uint32_t _regMode, AddrMode _mode, bool _wb)
        : MightBeMicro(mnem, _machInst, __opClass),
          regMode(_regMode), mode(_mode), wb(_wb), uops(NULL)
    {}

    virtual
    ~SrsOp()
    {
        delete [] uops;
    }

    StaticInstPtr
    fetchMicroop(MicroPC microPC) const
    {
        assert(uops != NULL && microPC < numMicroops);
        return uops[microPC];
    }

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

class Memory : public MightBeMicro
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
    bool add;
    static const unsigned numMicroops = 3;

    StaticInstPtr *uops;

    Memory(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
           IntRegIndex _dest, IntRegIndex _base, bool _add)
        : MightBeMicro(mnem, _machInst, __opClass),
          dest(_dest), base(_base), add(_add), uops(NULL)
    {}

    virtual
    ~Memory()
    {
        delete [] uops;
    }

    StaticInstPtr
    fetchMicroop(MicroPC microPC) const
    {
        assert(uops != NULL && microPC < numMicroops);
        return uops[microPC];
    }

    virtual void
    printOffset(std::ostream &os) const
    {}

    virtual void
    printDest(std::ostream &os) const
    {
        printReg(os, dest);
    }

    void printInst(std::ostream &os, AddrMode addrMode) const;
};

// The address is a base register plus an immediate.
class MemoryImm : public Memory
{
  protected:
    int32_t imm;

    MemoryImm(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
              IntRegIndex _dest, IntRegIndex _base, bool _add, int32_t _imm)
        : Memory(mnem, _machInst, __opClass, _dest, _base, _add), imm(_imm)
    {}

    void
    printOffset(std::ostream &os) const
    {
        int32_t pImm = imm;
        if (!add)
            pImm = -pImm;
        ccprintf(os, "#%d", pImm);
    }
};

class MemoryExImm : public MemoryImm
{
  protected:
    IntRegIndex result;

    MemoryExImm(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                IntRegIndex _result, IntRegIndex _dest, IntRegIndex _base,
                bool _add, int32_t _imm)
        : MemoryImm(mnem, _machInst, __opClass, _dest, _base, _add, _imm),
                    result(_result)
    {}

    void
    printDest(std::ostream &os) const
    {
        printReg(os, result);
        os << ", ";
        MemoryImm::printDest(os);
    }
};

// The address is a base register plus an immediate.
class MemoryDImm : public MemoryImm
{
  protected:
    IntRegIndex dest2;

    MemoryDImm(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
              IntRegIndex _dest, IntRegIndex _dest2,
              IntRegIndex _base, bool _add, int32_t _imm)
        : MemoryImm(mnem, _machInst, __opClass, _dest, _base, _add, _imm),
          dest2(_dest2)
    {}

    void
    printDest(std::ostream &os) const
    {
        MemoryImm::printDest(os);
        os << ", ";
        printReg(os, dest2);
    }
};

class MemoryExDImm : public MemoryDImm
{
  protected:
    IntRegIndex result;

    MemoryExDImm(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                 IntRegIndex _result, IntRegIndex _dest, IntRegIndex _dest2,
                 IntRegIndex _base, bool _add, int32_t _imm)
        : MemoryDImm(mnem, _machInst, __opClass, _dest, _dest2,
                     _base, _add, _imm), result(_result)
    {}

    void
    printDest(std::ostream &os) const
    {
        printReg(os, result);
        os << ", ";
        MemoryDImm::printDest(os);
    }
};

// The address is a shifted register plus an immediate
class MemoryReg : public Memory
{
  protected:
    int32_t shiftAmt;
    ArmShiftType shiftType;
    IntRegIndex index;

    MemoryReg(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
              IntRegIndex _dest, IntRegIndex _base, bool _add,
              int32_t _shiftAmt, ArmShiftType _shiftType,
              IntRegIndex _index)
        : Memory(mnem, _machInst, __opClass, _dest, _base, _add),
          shiftAmt(_shiftAmt), shiftType(_shiftType), index(_index)
    {}

    void printOffset(std::ostream &os) const;
};

class MemoryDReg : public MemoryReg
{
  protected:
    IntRegIndex dest2;

    MemoryDReg(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
               IntRegIndex _dest, IntRegIndex _dest2,
               IntRegIndex _base, bool _add,
               int32_t _shiftAmt, ArmShiftType _shiftType,
               IntRegIndex _index)
        : MemoryReg(mnem, _machInst, __opClass, _dest, _base, _add,
                    _shiftAmt, _shiftType, _index),
          dest2(_dest2)
    {}

    void
    printDest(std::ostream &os) const
    {
        MemoryReg::printDest(os);
        os << ", ";
        printReg(os, dest2);
    }
};

template<class Base>
class MemoryOffset : public Base
{
  protected:
    MemoryOffset(const char *mnem, ExtMachInst _machInst,
                 OpClass __opClass, IntRegIndex _dest, IntRegIndex _base,
                 bool _add, int32_t _imm)
        : Base(mnem, _machInst, __opClass, _dest, _base, _add, _imm)
    {}

    MemoryOffset(const char *mnem, ExtMachInst _machInst,
                 OpClass __opClass, IntRegIndex _dest, IntRegIndex _base,
                 bool _add, int32_t _shiftAmt, ArmShiftType _shiftType,
                 IntRegIndex _index)
        : Base(mnem, _machInst, __opClass, _dest, _base, _add,
                _shiftAmt, _shiftType, _index)
    {}

    MemoryOffset(const char *mnem, ExtMachInst _machInst,
                 OpClass __opClass, IntRegIndex _dest, IntRegIndex _dest2,
                 IntRegIndex _base, bool _add, int32_t _imm)
        : Base(mnem, _machInst, __opClass, _dest, _dest2, _base, _add, _imm)
    {}

    MemoryOffset(const char *mnem, ExtMachInst _machInst,
                 OpClass __opClass, IntRegIndex _result,
                 IntRegIndex _dest, IntRegIndex _dest2,
                 IntRegIndex _base, bool _add, int32_t _imm)
        : Base(mnem, _machInst, __opClass, _result,
                _dest, _dest2, _base, _add, _imm)
    {}

    MemoryOffset(const char *mnem, ExtMachInst _machInst,
                 OpClass __opClass, IntRegIndex _dest, IntRegIndex _dest2,
                 IntRegIndex _base, bool _add,
                 int32_t _shiftAmt, ArmShiftType _shiftType,
                 IntRegIndex _index)
        : Base(mnem, _machInst, __opClass, _dest, _dest2, _base, _add,
                _shiftAmt, _shiftType, _index)
    {}

    std::string
    generateDisassembly(Addr pc, const SymbolTable *symtab) const
    {
        std::stringstream ss;
        this->printInst(ss, Memory::AddrMd_Offset);
        return ss.str();
    }
};

template<class Base>
class MemoryPreIndex : public Base
{
  protected:
    MemoryPreIndex(const char *mnem, ExtMachInst _machInst,
                   OpClass __opClass, IntRegIndex _dest, IntRegIndex _base,
                   bool _add, int32_t _imm)
        : Base(mnem, _machInst, __opClass, _dest, _base, _add, _imm)
    {}

    MemoryPreIndex(const char *mnem, ExtMachInst _machInst,
                   OpClass __opClass, IntRegIndex _dest, IntRegIndex _base,
                   bool _add, int32_t _shiftAmt, ArmShiftType _shiftType,
                   IntRegIndex _index)
        : Base(mnem, _machInst, __opClass, _dest, _base, _add,
                _shiftAmt, _shiftType, _index)
    {}

    MemoryPreIndex(const char *mnem, ExtMachInst _machInst,
                   OpClass __opClass, IntRegIndex _dest, IntRegIndex _dest2,
                   IntRegIndex _base, bool _add, int32_t _imm)
        : Base(mnem, _machInst, __opClass, _dest, _dest2, _base, _add, _imm)
    {}

    MemoryPreIndex(const char *mnem, ExtMachInst _machInst,
                   OpClass __opClass, IntRegIndex _result,
                   IntRegIndex _dest, IntRegIndex _dest2,
                   IntRegIndex _base, bool _add, int32_t _imm)
        : Base(mnem, _machInst, __opClass, _result,
                _dest, _dest2, _base, _add, _imm)
    {}

    MemoryPreIndex(const char *mnem, ExtMachInst _machInst,
                   OpClass __opClass, IntRegIndex _dest, IntRegIndex _dest2,
                   IntRegIndex _base, bool _add,
                   int32_t _shiftAmt, ArmShiftType _shiftType,
                   IntRegIndex _index)
        : Base(mnem, _machInst, __opClass, _dest, _dest2, _base, _add,
                _shiftAmt, _shiftType, _index)
    {}

    std::string
    generateDisassembly(Addr pc, const SymbolTable *symtab) const
    {
        std::stringstream ss;
        this->printInst(ss, Memory::AddrMd_PreIndex);
        return ss.str();
    }
};

template<class Base>
class MemoryPostIndex : public Base
{
  protected:
    MemoryPostIndex(const char *mnem, ExtMachInst _machInst,
                    OpClass __opClass, IntRegIndex _dest, IntRegIndex _base,
                    bool _add, int32_t _imm)
        : Base(mnem, _machInst, __opClass, _dest, _base, _add, _imm)
    {}

    MemoryPostIndex(const char *mnem, ExtMachInst _machInst,
                    OpClass __opClass, IntRegIndex _dest, IntRegIndex _base,
                    bool _add, int32_t _shiftAmt, ArmShiftType _shiftType,
                    IntRegIndex _index)
        : Base(mnem, _machInst, __opClass, _dest, _base, _add,
                _shiftAmt, _shiftType, _index)
    {}

    MemoryPostIndex(const char *mnem, ExtMachInst _machInst,
                    OpClass __opClass, IntRegIndex _dest, IntRegIndex _dest2,
                    IntRegIndex _base, bool _add, int32_t _imm)
        : Base(mnem, _machInst, __opClass, _dest, _dest2, _base, _add, _imm)
    {}

    MemoryPostIndex(const char *mnem, ExtMachInst _machInst,
                    OpClass __opClass, IntRegIndex _result,
                    IntRegIndex _dest, IntRegIndex _dest2,
                    IntRegIndex _base, bool _add, int32_t _imm)
        : Base(mnem, _machInst, __opClass, _result,
                _dest, _dest2, _base, _add, _imm)
    {}

    MemoryPostIndex(const char *mnem, ExtMachInst _machInst,
                    OpClass __opClass, IntRegIndex _dest, IntRegIndex _dest2,
                    IntRegIndex _base, bool _add,
                    int32_t _shiftAmt, ArmShiftType _shiftType,
                    IntRegIndex _index)
        : Base(mnem, _machInst, __opClass, _dest, _dest2, _base, _add,
                _shiftAmt, _shiftType, _index)
    {}

    std::string
    generateDisassembly(Addr pc, const SymbolTable *symtab) const
    {
        std::stringstream ss;
        this->printInst(ss, Memory::AddrMd_PostIndex);
        return ss.str();
    }
};
}

#endif //__ARCH_ARM_INSTS_MEM_HH__
