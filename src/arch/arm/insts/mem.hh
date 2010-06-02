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

class MemoryNew : public PredOp
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

    MemoryNew(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
              IntRegIndex _dest, IntRegIndex _base, bool _add)
        : PredOp(mnem, _machInst, __opClass),
          dest(_dest), base(_base), add(_add)
    {}

    virtual void
    printOffset(std::ostream &os) const
    {}

    void printInst(std::ostream &os, AddrMode addrMode) const;
};

// The address is a base register plus an immediate.
class MemoryNewImm : public MemoryNew
{
  protected:
    int32_t imm;

    MemoryNewImm(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                 IntRegIndex _dest, IntRegIndex _base, bool _add, int32_t _imm)
        : MemoryNew(mnem, _machInst, __opClass, _dest, _base, _add), imm(_imm)
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

// The address is a shifted register plus an immediate
class MemoryNewReg : public MemoryNew
{
  protected:
    int32_t shiftAmt;
    ArmShiftType shiftType;
    IntRegIndex index;

    MemoryNewReg(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                 IntRegIndex _dest, IntRegIndex _base, bool _add,
                 int32_t _shiftAmt, ArmShiftType _shiftType,
                 IntRegIndex _index)
        : MemoryNew(mnem, _machInst, __opClass, _dest, _base, _add),
          shiftAmt(_shiftAmt), shiftType(_shiftType), index(_index)
    {}

    void
    printOffset(std::ostream &os) const
    {
        if (!add)
            os << "-";
        printReg(os, index);
        if (shiftType != LSL || shiftAmt != 0) {
            switch (shiftType) {
              case LSL:
                ccprintf(os, " LSL #%d", shiftAmt);
                break;
              case LSR:
                if (shiftAmt == 0) {
                    ccprintf(os, " LSR #%d", 32);
                } else {
                    ccprintf(os, " LSR #%d", shiftAmt);
                }
                break;
              case ASR:
                if (shiftAmt == 0) {
                    ccprintf(os, " ASR #%d", 32);
                } else {
                    ccprintf(os, " ASR #%d", shiftAmt);
                }
                break;
              case ROR:
                if (shiftAmt == 0) {
                    ccprintf(os, " RRX");
                } else {
                    ccprintf(os, " ROR #%d", shiftAmt);
                }
                break;
            }
        }
    }
};

template<class Base>
class MemoryNewOffset : public Base
{
  protected:
    MemoryNewOffset(const char *mnem, ExtMachInst _machInst,
                    OpClass __opClass, IntRegIndex _dest, IntRegIndex _base,
                    bool _add, int32_t _imm)
        : Base(mnem, _machInst, __opClass, _dest, _base, _add, _imm)
    {}

    MemoryNewOffset(const char *mnem, ExtMachInst _machInst,
                    OpClass __opClass, IntRegIndex _dest, IntRegIndex _base,
                    bool _add, int32_t _shiftAmt, ArmShiftType _shiftType,
                    IntRegIndex _index)
        : Base(mnem, _machInst, __opClass, _dest, _base, _add,
                _shiftAmt, _shiftType, _index)
    {}

    std::string
    generateDisassembly(Addr pc, const SymbolTable *symtab) const
    {
        std::stringstream ss;
        this->printInst(ss, MemoryNew::AddrMd_Offset);
        return ss.str();
    }
};

template<class Base>
class MemoryNewPreIndex : public Base
{
  protected:
    MemoryNewPreIndex(const char *mnem, ExtMachInst _machInst,
                      OpClass __opClass, IntRegIndex _dest, IntRegIndex _base,
                      bool _add, int32_t _imm)
        : Base(mnem, _machInst, __opClass, _dest, _base, _add, _imm)
    {}

    MemoryNewPreIndex(const char *mnem, ExtMachInst _machInst,
                      OpClass __opClass, IntRegIndex _dest, IntRegIndex _base,
                      bool _add, int32_t _shiftAmt, ArmShiftType _shiftType,
                      IntRegIndex _index)
        : Base(mnem, _machInst, __opClass, _dest, _base, _add,
                _shiftAmt, _shiftType, _index)
    {}

    std::string
    generateDisassembly(Addr pc, const SymbolTable *symtab) const
    {
        std::stringstream ss;
        this->printInst(ss, MemoryNew::AddrMd_PreIndex);
        return ss.str();
    }
};

template<class Base>
class MemoryNewPostIndex : public Base
{
  protected:
    MemoryNewPostIndex(const char *mnem, ExtMachInst _machInst,
                       OpClass __opClass, IntRegIndex _dest, IntRegIndex _base,
                       bool _add, int32_t _imm)
        : Base(mnem, _machInst, __opClass, _dest, _base, _add, _imm)
    {}

    MemoryNewPostIndex(const char *mnem, ExtMachInst _machInst,
                       OpClass __opClass, IntRegIndex _dest, IntRegIndex _base,
                       bool _add, int32_t _shiftAmt, ArmShiftType _shiftType,
                       IntRegIndex _index)
        : Base(mnem, _machInst, __opClass, _dest, _base, _add,
                _shiftAmt, _shiftType, _index)
    {}

    std::string
    generateDisassembly(Addr pc, const SymbolTable *symtab) const
    {
        std::stringstream ss;
        this->printInst(ss, MemoryNew::AddrMd_PostIndex);
        return ss.str();
    }
};
}

#endif //__ARCH_ARM_INSTS_MEM_HH__
