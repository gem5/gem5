/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
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

#ifndef __ARCH_X86_INSTS_MICROREGOP_HH__
#define __ARCH_X86_INSTS_MICROREGOP_HH__

#include "arch/x86/insts/microop.hh"

namespace X86ISA
{

struct RegOpDest
{
    using ArgType = InstRegIndex;

    RegIndex dest;
    size_t size;

    template <class InstType>
    RegOpDest(InstType *inst, ArgType idx) :
        dest(INTREG_FOLDED(idx.index(), inst->foldOBit)),
        size(inst->dataSize)
    {}

    void
    print(std::ostream &os) const
    {
        X86StaticInst::printReg(os, RegId(IntRegClass, dest), size);
    }
};

struct RegOpDbgDest
{
    using ArgType = InstRegIndex;

    RegIndex dest;
    size_t size;

    template <class InstType>
    RegOpDbgDest(InstType *inst, ArgType idx) : dest(idx.index()),
        size(inst->dataSize)
    {}

    void
    print(std::ostream &os) const
    {
        ccprintf(os, "dr%d", dest);
    }
};

struct RegOpCrDest
{
    using ArgType = InstRegIndex;

    RegIndex dest;
    size_t size;

    template <class InstType>
    RegOpCrDest(InstType *inst, ArgType idx) : dest(idx.index()),
        size(inst->dataSize)
    {}

    void
    print(std::ostream &os) const
    {
        ccprintf(os, "cr%d", dest);
    }
};

struct RegOpSegDest
{
    using ArgType = InstRegIndex;

    RegIndex dest;
    size_t size;

    template <class InstType>
    RegOpSegDest(InstType *inst, ArgType idx) : dest(idx.index()),
        size(inst->dataSize)
    {}

    void
    print(std::ostream &os) const
    {
        X86StaticInst::printSegment(os, dest);
    }
};

struct RegOpMiscDest
{
    using ArgType = InstRegIndex;

    RegIndex dest;
    size_t size;

    template <class InstType>
    RegOpMiscDest(InstType *inst, ArgType idx) : dest(idx.index()),
        size(inst->dataSize)
    {}

    void
    print(std::ostream &os) const
    {
        X86StaticInst::printReg(os, RegId(MiscRegClass, dest), size);
    }
};

struct RegOpSrc1
{
    using ArgType = InstRegIndex;

    RegIndex src1;
    size_t size;

    template <class InstType>
    RegOpSrc1(InstType *inst, ArgType idx) :
        src1(INTREG_FOLDED(idx.index(), inst->foldOBit)),
        size(inst->dataSize)
    {}

    void
    print(std::ostream &os) const
    {
        X86StaticInst::printReg(os, RegId(IntRegClass, src1), size);
    }
};

struct RegOpDbgSrc1
{
    using ArgType = InstRegIndex;

    RegIndex src1;
    size_t size;

    template <class InstType>
    RegOpDbgSrc1(InstType *inst, ArgType idx) : src1(idx.index()),
        size(inst->dataSize)
    {}

    void
    print(std::ostream &os) const
    {
        ccprintf(os, "dr%d", src1);
    }
};

struct RegOpCrSrc1
{
    using ArgType = InstRegIndex;

    RegIndex src1;
    size_t size;

    template <class InstType>
    RegOpCrSrc1(InstType *inst, ArgType idx) : src1(idx.index()),
        size(inst->dataSize)
    {}

    void
    print(std::ostream &os) const
    {
        ccprintf(os, "cr%d", src1);
    }
};

struct RegOpSegSrc1
{
    using ArgType = InstRegIndex;

    RegIndex src1;
    size_t size;

    template <class InstType>
    RegOpSegSrc1(InstType *inst, ArgType idx) : src1(idx.index()),
        size(inst->dataSize)
    {}

    void
    print(std::ostream &os) const
    {
        X86StaticInst::printSegment(os, src1);
    }
};

struct RegOpMiscSrc1
{
    using ArgType = InstRegIndex;

    RegIndex src1;
    size_t size;

    template <class InstType>
    RegOpMiscSrc1(InstType *inst, ArgType idx) : src1(idx.index()),
        size(inst->dataSize)
    {}

    void
    print(std::ostream &os) const
    {
        X86StaticInst::printReg(os, RegId(MiscRegClass, src1), size);
    }
};

struct RegOpSrc2
{
    using ArgType = InstRegIndex;

    RegIndex src2;
    size_t size;

    template <class InstType>
    RegOpSrc2(InstType *inst, ArgType idx) :
        src2(INTREG_FOLDED(idx.index(), inst->foldOBit)),
        size(inst->dataSize)
    {}

    void
    print(std::ostream &os) const
    {
        X86StaticInst::printReg(os, RegId(IntRegClass, src2), size);
    }
};

struct RegOpImm8
{
    using ArgType = uint8_t;

    uint8_t imm8;

    template <class InstType>
    RegOpImm8(InstType *inst, ArgType _imm8) : imm8(_imm8) {}

    void
    print(std::ostream &os) const
    {
        ccprintf(os, "%#x", imm8);
    }
};

class RegOpBase : public X86MicroopBase
{
  protected:
    const uint16_t ext;

    RegOpBase(ExtMachInst mach_inst, const char *mnem, const char *inst_mnem,
            uint64_t set_flags, uint8_t data_size, uint16_t _ext,
            OpClass op_class) :
        X86MicroopBase(mach_inst, mnem, inst_mnem, set_flags, op_class),
        ext(_ext), dataSize(data_size),
        foldOBit((data_size == 1 && !mach_inst.rex.present) ? 1 << 6 : 0)
    {}

    //Figure out what the condition code flags should be.
    uint64_t genFlags(uint64_t old_flags, uint64_t flag_mask,
            uint64_t _dest, uint64_t _src1, uint64_t _src2,
            bool subtract=false)const ;

  public:
    const uint8_t dataSize;
    const RegIndex foldOBit;
};

template <typename ...Operands>
class RegOpT : public RegOpBase, public Operands...
{
  protected:
    RegOpT(ExtMachInst mach_inst, const char *mnem, const char *inst_mnem,
            uint64_t set_flags, uint8_t data_size, uint16_t _ext,
            OpClass op_class, typename Operands::ArgType... args) :
        RegOpBase(mach_inst, mnem, inst_mnem, set_flags, data_size, _ext,
                op_class), Operands(this, args)...
    {}

    std::string
    generateDisassembly(Addr pc,
            const Loader::SymbolTable *symtab) const override
    {
        std::stringstream response;
        printMnemonic(response, instMnem, mnemonic);
        int count = 0;
        M5_FOR_EACH_IN_PACK(ccprintf(response, count++ ? ", " : ""),
                            Operands::print(response));
        return response.str();
    }
};

}

#endif //__ARCH_X86_INSTS_MICROREGOP_HH__
