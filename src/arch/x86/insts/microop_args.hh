/*
 * Copyright 2021 Google Inc.
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

#ifndef __ARCH_X86_INSTS_MICROOP_ARGS_HH__
#define __ARCH_X86_INSTS_MICROOP_ARGS_HH__

#include <cstdint>
#include <sstream>
#include <string>

#include "arch/x86/insts/static_inst.hh"
#include "arch/x86/regs/int.hh"
#include "arch/x86/types.hh"
#include "base/cprintf.hh"
#include "cpu/reg_class.hh"

namespace X86ISA
{

struct DestOp
{
    using ArgType = InstRegIndex;
    const RegIndex dest;
    const size_t size;
    RegIndex index() const { return dest; }

    DestOp(RegIndex _dest, size_t _size) : dest(_dest), size(_size) {}
};

struct Src1Op
{
    using ArgType = InstRegIndex;
    const RegIndex src1;
    const size_t size;
    RegIndex index() const { return src1; }

    Src1Op(RegIndex _src1, size_t _size) : src1(_src1), size(_size) {}
};

struct Src2Op
{
    using ArgType = InstRegIndex;
    const RegIndex src2;
    const size_t size;
    RegIndex index() const { return src2; }

    Src2Op(RegIndex _src2, size_t _size) : src2(_src2), size(_size) {}
};

template <class Base>
struct FoldedOp : public Base
{
    template <class InstType>
    FoldedOp(InstType *inst, typename Base::ArgType idx) :
        Base(INTREG_FOLDED(idx.index(), inst->foldOBit), inst->dataSize)
    {}

    void
    print(std::ostream &os) const
    {
        X86StaticInst::printReg(os, RegId(IntRegClass, this->index()),
                this->size);
    }
};

template <class Base>
struct CrOp : public Base
{
    template <class InstType>
    CrOp(InstType *inst, typename Base::ArgType idx) : Base(idx.index(), 0) {}

    void
    print(std::ostream &os) const
    {
        ccprintf(os, "cr%d", this->index());
    }
};

template <class Base>
struct DbgOp : public Base
{
    template <class InstType>
    DbgOp(InstType *inst, typename Base::ArgType idx) : Base(idx.index(), 0) {}

    void
    print(std::ostream &os) const
    {
        ccprintf(os, "dr%d", this->index());
    }

};

template <class Base>
struct SegOp : public Base
{
    template <class InstType>
    SegOp(InstType *inst, typename Base::ArgType idx) : Base(idx.index(), 0) {}

    void
    print(std::ostream &os) const
    {
        X86StaticInst::printSegment(os, this->index());
    }
};

template <class Base>
struct MiscOp : public Base
{
    template <class InstType>
    MiscOp(InstType *inst, typename Base::ArgType idx) :
        Base(idx.index(), inst->dataSize)
    {}

    void
    print(std::ostream &os) const
    {
        X86StaticInst::printReg(os, RegId(MiscRegClass, this->index()),
                this->size);
    }
};

using FoldedDestOp = FoldedOp<DestOp>;
using DbgDestOp = DbgOp<DestOp>;
using CrDestOp = CrOp<DestOp>;
using SegDestOp = SegOp<DestOp>;
using MiscDestOp = MiscOp<DestOp>;

using FoldedSrc1Op = FoldedOp<Src1Op>;
using DbgSrc1Op = DbgOp<Src1Op>;
using CrSrc1Op = CrOp<Src1Op>;
using SegSrc1Op = SegOp<Src1Op>;
using MiscSrc1Op = MiscOp<Src1Op>;

using FoldedSrc2Op = FoldedOp<Src2Op>;

struct Imm8Op
{
    using ArgType = uint8_t;

    uint8_t imm8;

    template <class InstType>
    Imm8Op(InstType *inst, ArgType _imm8) : imm8(_imm8) {}

    void
    print(std::ostream &os) const
    {
        ccprintf(os, "%#x", imm8);
    }
};

template <typename Base, typename ...Operands>
class InstOperands : public Base, public Operands...
{
  protected:
    template <typename ...CTorArgs>
    InstOperands(ExtMachInst mach_inst, const char *mnem,
            const char *inst_mnem, uint64_t set_flags, OpClass op_class,
            typename Operands::ArgType... args, CTorArgs... ctor_args) :
        Base(mach_inst, mnem, inst_mnem, set_flags, op_class, ctor_args...),
        Operands(this, args)...
    {}

    std::string
    generateDisassembly(Addr pc,
            const Loader::SymbolTable *symtab) const override
    {
        std::stringstream response;
        Base::printMnemonic(response, this->instMnem, this->mnemonic);
        int count = 0;
        M5_FOR_EACH_IN_PACK(ccprintf(response, count++ ? ", " : ""),
                            Operands::print(response));
        return response.str();
    }
};

}

#endif //__ARCH_X86_INSTS_MICROOP_ARGS_HH__
