/* Copyright (c) 2007-2008 The Florida State University
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
/**
 * Base class for general Arm memory-format instructions.
 */
class Memory : public PredOp
{
  protected:

    /// Memory request flags.  See mem_req_base.hh.
    unsigned memAccessFlags;

    /// Displacement for EA calculation (signed).
    int32_t disp;
    int32_t disp8;
    int32_t up;
    int32_t hilo,
            shift_size,
            shift;

    /// Constructor
    Memory(const char *mnem, ExtMachInst _machInst, OpClass __opClass)
        : PredOp(mnem, _machInst, __opClass),
                 memAccessFlags(0),
                 disp(machInst.immed11_0),
                 disp8(machInst.immed7_0 << 2),
                 up(machInst.puswl.up),
                 hilo((machInst.immedHi11_8 << 4) | machInst.immedLo3_0),
                 shift_size(machInst.shiftSize), shift(machInst.shift)
    {
    }

    std::string
    generateDisassembly(Addr pc, const SymbolTable *symtab) const;

    virtual void
    printOffset(std::ostream &os) const
    {}
};

class MemoryDisp : public Memory
{
  protected:
    /// Constructor
    MemoryDisp(const char *mnem, ExtMachInst _machInst, OpClass __opClass)
        : Memory(mnem, _machInst, __opClass)
    {
    }

    void
    printOffset(std::ostream &os) const
    {
        ccprintf(os, "#%#x", (machInst.puswl.up ? disp : -disp));
    }
};

class MemoryHilo : public Memory
{
  protected:
    /// Constructor
    MemoryHilo(const char *mnem, ExtMachInst _machInst, OpClass __opClass)
        : Memory(mnem, _machInst, __opClass)
    {
    }

    void
    printOffset(std::ostream &os) const
    {
        ccprintf(os, "#%#x", (machInst.puswl.up ? hilo : -hilo));
    }
};

class MemoryShift : public Memory
{
  protected:
    /// Constructor
    MemoryShift(const char *mnem, ExtMachInst _machInst, OpClass __opClass)
        : Memory(mnem, _machInst, __opClass)
    {
    }

    void
    printOffset(std::ostream &os) const
    {
        printShiftOperand(os);
    }
};

class MemoryReg : public Memory
{
  protected:
    /// Constructor
    MemoryReg(const char *mnem, ExtMachInst _machInst, OpClass __opClass)
        : Memory(mnem, _machInst, __opClass)
    {
    }

    void
    printOffset(std::ostream &os) const
    {
        os << (machInst.puswl.up ? "+ " : "- ");
        printReg(os, machInst.rm);
    }
};
}

#endif //__ARCH_ARM_INSTS_MEM_HH__
