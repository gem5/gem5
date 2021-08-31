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
#include <tuple>
#include <type_traits>
#include <utility>

#include "arch/x86/insts/static_inst.hh"
#include "arch/x86/regs/float.hh"
#include "arch/x86/regs/int.hh"
#include "arch/x86/regs/misc.hh"
#include "arch/x86/regs/segment.hh"
#include "arch/x86/types.hh"
#include "base/compiler.hh"
#include "base/cprintf.hh"
#include "cpu/reg_class.hh"
#include "sim/faults.hh"

namespace gem5
{

namespace X86ISA
{

struct DestOp
{
    const RegIndex dest;
    const size_t size;
    RegIndex opIndex() const { return dest; }

    DestOp(RegIndex _dest, size_t _size) : dest(_dest), size(_size) {}
    template <class InstType>
    DestOp(RegIndex _dest, InstType *inst) : dest(_dest),
        size(inst->getDestSize())
    {}
};

struct Src1Op
{
    const RegIndex src1;
    const size_t size;
    RegIndex opIndex() const { return src1; }

    Src1Op(RegIndex _src1, size_t _size) : src1(_src1), size(_size) {}
    template <class InstType>
    Src1Op(RegIndex _src1, InstType *inst) : src1(_src1),
        size(inst->getSrcSize())
    {}
};

struct Src2Op
{
    const RegIndex src2;
    const size_t size;
    RegIndex opIndex() const { return src2; }

    Src2Op(RegIndex _src2, size_t _size) : src2(_src2), size(_size) {}
    template <class InstType>
    Src2Op(RegIndex _src2, InstType *inst) : src2(_src2),
        size(inst->getSrcSize())
    {}
};

struct DataOp
{
    const RegIndex data;
    const size_t size;
    RegIndex opIndex() const { return data; }

    DataOp(RegIndex _data, size_t _size) : data(_data), size(_size) {}
};

struct DataHiOp
{
    const RegIndex dataHi;
    const size_t size;
    RegIndex opIndex() const { return dataHi; }

    DataHiOp(RegIndex data_hi, size_t _size) : dataHi(data_hi), size(_size) {}
};

struct DataLowOp
{
    const RegIndex dataLow;
    const size_t size;
    RegIndex opIndex() const { return dataLow; }

    DataLowOp(RegIndex data_low, size_t _size) : dataLow(data_low), size(_size)
    {}
};

template <class T, class Enabled=void>
struct HasDataSize : public std::false_type {};

template <class T>
struct HasDataSize<T, decltype((void)&T::dataSize)> : public std::true_type {};

template <class T>
constexpr bool HasDataSizeV = HasDataSize<T>::value;

template <class Base>
struct IntOp : public Base
{
    using ArgType = GpRegIndex;

    template <class Inst>
    IntOp(Inst *inst, std::enable_if_t<HasDataSizeV<Inst>, ArgType> idx) :
        Base(idx.index, inst->dataSize)
    {}

    template <class Inst>
    IntOp(Inst *inst, std::enable_if_t<!HasDataSizeV<Inst>, ArgType> idx) :
        Base(idx.index, inst)
    {}

    void
    print(std::ostream &os) const
    {
        X86StaticInst::printReg(os, intRegClass[this->opIndex()], this->size);
    }
};

template <class Base>
struct FoldedOp : public Base
{
    using ArgType = GpRegIndex;

    template <class InstType>
    FoldedOp(InstType *inst, ArgType idx) :
        Base(intRegFolded(idx.index, inst->foldOBit), inst->dataSize)
    {}

    void
    print(std::ostream &os) const
    {
        X86StaticInst::printReg(os, intRegClass[this->opIndex()], this->size);
    }
};

template <class Base>
struct CrOp : public Base
{
    using ArgType = CrRegIndex;

    template <class InstType>
    CrOp(InstType *inst, ArgType idx) : Base(idx.index, 0) {}

    void
    print(std::ostream &os) const
    {
        ccprintf(os, "cr%d", this->opIndex());
    }
};

template <class Base>
struct DbgOp : public Base
{
    using ArgType = DbgRegIndex;

    template <class InstType>
    DbgOp(InstType *inst, ArgType idx) : Base(idx.index, 0) {}

    void
    print(std::ostream &os) const
    {
        ccprintf(os, "dr%d", this->opIndex());
    }

};

template <class Base>
struct SegOp : public Base
{
    using ArgType = SegRegIndex;

    template <class InstType>
    SegOp(InstType *inst, ArgType idx) : Base(idx.index, 0) {}

    void
    print(std::ostream &os) const
    {
        X86StaticInst::printSegment(os, this->opIndex());
    }
};

template <class Base>
struct MiscOp : public Base
{
    using ArgType = CtrlRegIndex;

    template <class InstType>
    MiscOp(InstType *inst, ArgType idx) : Base(idx.index, inst->dataSize) {}

    void
    print(std::ostream &os) const
    {
        X86StaticInst::printReg(os, miscRegClass[this->opIndex()], this->size);
    }
};

template <class Base>
struct FloatOp : public Base
{
    using ArgType = FpRegIndex;

    template <class Inst>
    FloatOp(Inst *inst, std::enable_if_t<HasDataSizeV<Inst>, ArgType> idx) :
        Base(idx.index, inst->dataSize)
    {}

    template <class Inst>
    FloatOp(Inst *inst, std::enable_if_t<!HasDataSizeV<Inst>, ArgType> idx) :
        Base(idx.index, inst)
    {}

    void
    print(std::ostream &os) const
    {
        X86StaticInst::printReg(os, floatRegClass[this->opIndex()],
                this->size);
    }
};

using FoldedDestOp = FoldedOp<DestOp>;
using DbgDestOp = DbgOp<DestOp>;
using CrDestOp = CrOp<DestOp>;
using SegDestOp = SegOp<DestOp>;
using MiscDestOp = MiscOp<DestOp>;
using FloatDestOp = FloatOp<DestOp>;
using IntDestOp = IntOp<DestOp>;

using FoldedSrc1Op = FoldedOp<Src1Op>;
using DbgSrc1Op = DbgOp<Src1Op>;
using CrSrc1Op = CrOp<Src1Op>;
using SegSrc1Op = SegOp<Src1Op>;
using MiscSrc1Op = MiscOp<Src1Op>;
using FloatSrc1Op = FloatOp<Src1Op>;
using IntSrc1Op = IntOp<Src1Op>;

using FoldedSrc2Op = FoldedOp<Src2Op>;
using FloatSrc2Op = FloatOp<Src2Op>;
using IntSrc2Op = IntOp<Src2Op>;

using FoldedDataOp = FoldedOp<DataOp>;
using FloatDataOp = FloatOp<DataOp>;
using FoldedDataHiOp = FoldedOp<DataHiOp>;
using FoldedDataLowOp = FoldedOp<DataLowOp>;

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

struct Imm64Op
{
    using ArgType = uint64_t;

    uint64_t imm64;

    template <class InstType>
    Imm64Op(InstType *inst, ArgType _imm64) : imm64(_imm64) {}

    void
    print(std::ostream &os) const
    {
        ccprintf(os, "%#x", imm64);
    }
};

struct UpcOp
{
    using ArgType = MicroPC;

    MicroPC target;

    template <class InstType>
    UpcOp(InstType *inst, ArgType _target) : target(_target) {}

    void
    print(std::ostream &os) const
    {
        ccprintf(os, "%#x", target);
    }
};

struct FaultOp
{
    using ArgType = Fault;

    Fault fault;

    template <class InstType>
    FaultOp(InstType *inst, ArgType _fault) : fault(_fault) {}

    void
    print(std::ostream &os) const
    {
        ccprintf(os, fault ? fault->name() : "NoFault");
    }
};

struct AddrOp
{
    struct ArgType
    {
        uint8_t scale;
        GpRegIndex index;
        GpRegIndex base;
        uint64_t disp;
        SegRegIndex segment;
    };

    const uint8_t scale;
    const RegIndex index;
    const RegIndex base;
    const uint64_t disp;
    const uint8_t segment;
    const size_t size;

    template <class InstType>
    AddrOp(InstType *inst, const ArgType &args) : scale(args.scale),
        index(intRegFolded(args.index.index, inst->foldABit)),
        base(intRegFolded(args.base.index, inst->foldABit)),
        disp(args.disp), segment(args.segment.index),
        size(inst->addressSize)
    {
        assert(segment < segment_idx::NumIdxs);
    }

    void
    print(std::ostream &os) const
    {
        X86StaticInst::printMem(
                os, segment, scale, index, base, disp, size, false);
    }
};

template <typename Base, typename ...Operands>
class InstOperands : public Base, public Operands...
{
  private:
    using ArgTuple = std::tuple<typename Operands::ArgType...>;

    template <std::size_t ...I, typename ...CTorArgs>
    InstOperands(std::index_sequence<I...>, ExtMachInst mach_inst,
            const char *mnem, const char *inst_mnem, uint64_t set_flags,
            OpClass op_class, [[maybe_unused]] ArgTuple args,
            CTorArgs... ctor_args) :
        Base(mach_inst, mnem, inst_mnem, set_flags, op_class, ctor_args...),
        Operands(this, std::get<I>(args))...
    {}

  protected:
    template <typename ...CTorArgs>
    InstOperands(ExtMachInst mach_inst, const char *mnem,
            const char *inst_mnem, uint64_t set_flags, OpClass op_class,
            ArgTuple args, CTorArgs... ctor_args) :
        InstOperands(std::make_index_sequence<sizeof...(Operands)>{},
                mach_inst, mnem, inst_mnem, set_flags, op_class,
                std::move(args), ctor_args...)
    {}

    std::string
    generateDisassembly(Addr pc,
            const loader::SymbolTable *symtab) const override
    {
        std::stringstream response;
        Base::printMnemonic(response, this->instMnem, this->mnemonic);
        int count = 0;
        GEM5_FOR_EACH_IN_PACK(ccprintf(response, count++ ? ", " : ""),
                              Operands::print(response));
        return response.str();
    }
};

} // namespace X86ISA
} // namespace gem5

#endif //__ARCH_X86_INSTS_MICROOP_ARGS_HH__
