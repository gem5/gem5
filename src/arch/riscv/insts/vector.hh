/*
 * Copyright (c) 2022 PLCT Lab
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

#ifndef __ARCH_RISCV_INSTS_VECTOR_HH__
#define __ARCH_RISCV_INSTS_VECTOR_HH__

#include <string>

#include "arch/riscv/faults.hh"
#include "arch/riscv/insts/static_inst.hh"
#include "arch/riscv/isa.hh"
#include "arch/riscv/regs/misc.hh"
#include "arch/riscv/utility.hh"
#include "cpu/exec_context.hh"
#include "cpu/static_inst.hh"

namespace gem5
{

namespace RiscvISA
{

float
getVflmul(uint32_t vlmul_encoding);

inline uint32_t
getSew(uint32_t vsew)
{
    assert(vsew <= 3);
    return (8 << vsew);
}

uint32_t
getVlmax(VTYPE vtype, uint32_t vlen);

/**
 * Base class for Vector Config operations
 */
class VConfOp : public RiscvStaticInst
{
  protected:
    uint64_t bit30;
    uint64_t bit31;
    uint64_t zimm10;
    uint64_t zimm11;
    uint64_t uimm;
    uint32_t elen;
    VConfOp(const char *mnem, ExtMachInst _extMachInst,
            uint32_t _elen, OpClass __opClass)
        : RiscvStaticInst(mnem, _extMachInst, __opClass),
          bit30(_extMachInst.bit30), bit31(_extMachInst.bit31),
          zimm10(_extMachInst.zimm_vsetivli),
          zimm11(_extMachInst.zimm_vsetvli),
          uimm(_extMachInst.uimm_vsetivli),
          elen(_elen)
    {
        this->flags[IsVector] = true;
    }

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;

    std::string generateZimmDisassembly() const;
};

inline uint8_t checked_vtype(bool vill, uint8_t vtype) {
    panic_if(vill, "vill has been set");
    const uint8_t vsew = bits(vtype, 5, 3);
    panic_if(vsew >= 0b100, "vsew: %#x not supported", vsew);
    const uint8_t vlmul = bits(vtype, 2, 0);
    panic_if(vlmul == 0b100, "vlmul: %#x not supported", vlmul);
    return vtype;
}

class VectorNonSplitInst : public RiscvStaticInst
{
  protected:
    uint32_t vl;
    uint8_t vtype;
    VectorNonSplitInst(const char* mnem, ExtMachInst _machInst,
                   OpClass __opClass)
        : RiscvStaticInst(mnem, _machInst, __opClass),
        vl(_machInst.vl),
        vtype(_machInst.vtype8)
    {
        this->flags[IsVector] = true;
    }

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class VectorMacroInst : public RiscvMacroInst
{
  protected:
    uint32_t vl;
    uint8_t vtype;
    uint32_t vlen;

    VectorMacroInst(const char* mnem, ExtMachInst _machInst,
                   OpClass __opClass, uint32_t _vlen = 256)
        : RiscvMacroInst(mnem, _machInst, __opClass),
        vl(_machInst.vl),
        vtype(_machInst.vtype8),
        vlen(_vlen)
    {
        this->flags[IsVector] = true;
    }
};

class VectorMicroInst : public RiscvMicroInst
{
protected:
    uint32_t vlen;
    uint32_t microVl;
    uint32_t microIdx;
    uint8_t vtype;
    VectorMicroInst(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
      uint32_t _microVl, uint32_t _microIdx, uint32_t _vlen = 256)
        : RiscvMicroInst(mnem, _machInst, __opClass),
        vlen(_vlen),
        microVl(_microVl),
        microIdx(_microIdx),
        vtype(_machInst.vtype8)
    {
        this->flags[IsVector] = true;
    }
};

class VectorNopMicroInst : public RiscvMicroInst
{
public:
    VectorNopMicroInst(ExtMachInst _machInst)
        : RiscvMicroInst("vnop", _machInst, No_OpClass)
    {}

    Fault execute(ExecContext* xc, trace::InstRecord* traceData)
        const override
    {
        return NoFault;
    }

    std::string generateDisassembly(Addr pc, const loader::SymbolTable *symtab)
      const override
    {
        std::stringstream ss;
        ss << mnemonic;
        return ss.str();
    }
};

class VectorArithMicroInst : public VectorMicroInst
{
protected:
    VectorArithMicroInst(const char *mnem, ExtMachInst _machInst,
                         OpClass __opClass, uint32_t _microVl,
                         uint32_t _microIdx)
        : VectorMicroInst(mnem, _machInst, __opClass, _microVl, _microIdx)
    {}

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

class VectorArithMacroInst : public VectorMacroInst
{
  protected:
    VectorArithMacroInst(const char* mnem, ExtMachInst _machInst,
                         OpClass __opClass, uint32_t _vlen = 256)
        : VectorMacroInst(mnem, _machInst, __opClass, _vlen)
    {
        this->flags[IsVector] = true;
    }
    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

class VectorVMUNARY0MicroInst : public VectorMicroInst
{
protected:
    VectorVMUNARY0MicroInst(const char *mnem, ExtMachInst _machInst,
                         OpClass __opClass, uint32_t _microVl,
                         uint32_t _microIdx)
        : VectorMicroInst(mnem, _machInst, __opClass, _microVl, _microIdx)
    {}

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

class VectorVMUNARY0MacroInst : public VectorMacroInst
{
  protected:
    VectorVMUNARY0MacroInst(const char* mnem, ExtMachInst _machInst,
                         OpClass __opClass, uint32_t _vlen)
        : VectorMacroInst(mnem, _machInst, __opClass, _vlen)
    {
        this->flags[IsVector] = true;
    }

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

class VectorSlideMacroInst : public VectorMacroInst
{
  protected:
    VectorSlideMacroInst(const char* mnem, ExtMachInst _machInst,
                         OpClass __opClass, uint32_t _vlen = 256)
        : VectorMacroInst(mnem, _machInst, __opClass, _vlen)
    {
        this->flags[IsVector] = true;
    }

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

class VectorSlideMicroInst : public VectorMicroInst
{
  protected:
    uint32_t vdIdx;
    uint32_t vs2Idx;
    VectorSlideMicroInst(const char *mnem, ExtMachInst _machInst,
                         OpClass __opClass, uint32_t _microVl,
                         uint32_t _microIdx, uint32_t _vdIdx, uint32_t _vs2Idx)
        : VectorMicroInst(mnem, _machInst, __opClass, _microVl, _microIdx)
        , vdIdx(_vdIdx), vs2Idx(_vs2Idx)
    {}

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

class VectorMemMicroInst : public VectorMicroInst
{
  protected:
    uint32_t offset; // Used to calculate EA.
    Request::Flags memAccessFlags;

    VectorMemMicroInst(const char* mnem, ExtMachInst _machInst,
                       OpClass __opClass, uint32_t _microVl,
                       uint32_t _microIdx, uint32_t _offset)
        : VectorMicroInst(mnem, _machInst, __opClass, _microVl, _microIdx)
        , offset(_offset)
        , memAccessFlags(0)
    {}
};

class VectorMemMacroInst : public VectorMacroInst
{
  protected:
    VectorMemMacroInst(const char* mnem, ExtMachInst _machInst,
                        OpClass __opClass, uint32_t _vlen = 256)
        : VectorMacroInst(mnem, _machInst, __opClass, _vlen)
    {}
};

class VleMacroInst : public VectorMemMacroInst
{
  protected:
    VleMacroInst(const char* mnem, ExtMachInst _machInst,
                   OpClass __opClass, uint32_t _vlen)
        : VectorMemMacroInst(mnem, _machInst, __opClass, _vlen)
    {}

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

class VseMacroInst : public VectorMemMacroInst
{
  protected:
    VseMacroInst(const char* mnem, ExtMachInst _machInst,
                   OpClass __opClass, uint32_t _vlen)
        : VectorMemMacroInst(mnem, _machInst, __opClass, _vlen)
    {}

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

class VleMicroInst : public VectorMicroInst
{
  public:
    mutable bool trimVl;
    mutable uint32_t faultIdx;

  protected:
    Request::Flags memAccessFlags;

    VleMicroInst(const char *mnem, ExtMachInst _machInst,OpClass __opClass,
                  uint32_t _microVl, uint32_t _microIdx, uint32_t _vlen)
        : VectorMicroInst(mnem, _machInst, __opClass, _microVl,
                            _microIdx, _vlen)
        , trimVl(false), faultIdx(_microVl)
    {
        this->flags[IsLoad] = true;
    }

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class VseMicroInst : public VectorMicroInst
{
  protected:
    Request::Flags memAccessFlags;

    VseMicroInst(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                  uint32_t _microVl, uint32_t _microIdx, uint32_t _vlen)
        : VectorMicroInst(mnem, _machInst, __opClass, _microVl,
                            _microIdx, _vlen)
    {
        this->flags[IsStore] = true;
    }

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class VlWholeMacroInst : public VectorMemMacroInst
{
  protected:
    VlWholeMacroInst(const char *mnem, ExtMachInst _machInst,
                     OpClass __opClass, uint32_t _vlen)
      : VectorMemMacroInst(mnem, _machInst, __opClass, _vlen)
    {}

    std::string generateDisassembly(
      Addr pc, const loader::SymbolTable *symtab) const override;
};

class VlWholeMicroInst : public VectorMicroInst
{
  protected:
    Request::Flags memAccessFlags;

    VlWholeMicroInst(const char *mnem, ExtMachInst _machInst,
          OpClass __opClass, uint32_t _microVl, uint32_t _microIdx,
          uint32_t _vlen)
        : VectorMicroInst(mnem, _machInst, __opClass, _microVl,
                            _microIdx, _vlen)
    {}

    std::string generateDisassembly(
      Addr pc, const loader::SymbolTable *symtab) const override;
};

class VsWholeMacroInst : public VectorMemMacroInst
{
  protected:
    VsWholeMacroInst(const char *mnem, ExtMachInst _machInst,
                     OpClass __opClass, uint32_t _vlen)
        : VectorMemMacroInst(mnem, _machInst, __opClass, _vlen)
    {}

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

class VsWholeMicroInst : public VectorMicroInst
{
  protected:
    Request::Flags memAccessFlags;

    VsWholeMicroInst(const char *mnem, ExtMachInst _machInst,
                      OpClass __opClass, uint32_t _microVl,
                      uint32_t _microIdx, uint32_t _vlen)
        : VectorMicroInst(mnem, _machInst, __opClass , _microVl,
                          _microIdx, _vlen)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class VlStrideMacroInst : public VectorMemMacroInst
{
  protected:
    VlStrideMacroInst(const char* mnem, ExtMachInst _machInst,
                   OpClass __opClass, uint32_t _vlen)
        : VectorMemMacroInst(mnem, _machInst, __opClass, _vlen)
    {}

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

class VlStrideMicroInst : public VectorMemMicroInst
{
  protected:
  uint32_t regIdx;
    VlStrideMicroInst(const char *mnem, ExtMachInst _machInst,
                      OpClass __opClass, uint32_t _regIdx,
                      uint32_t _microIdx, uint32_t _microVl)
        : VectorMemMicroInst(mnem, _machInst, __opClass, _microVl,
                             _microIdx, 0)
        , regIdx(_regIdx)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class VsStrideMacroInst : public VectorMemMacroInst
{
  protected:
    VsStrideMacroInst(const char* mnem, ExtMachInst _machInst,
                   OpClass __opClass, uint32_t _vlen)
        : VectorMemMacroInst(mnem, _machInst, __opClass, _vlen)
    {}

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

class VsStrideMicroInst : public VectorMemMicroInst
{
  protected:
    uint32_t regIdx;
    VsStrideMicroInst(const char *mnem, ExtMachInst _machInst,
                      OpClass __opClass, uint32_t _regIdx,
                      uint32_t _microIdx, uint32_t _microVl)
        : VectorMemMicroInst(mnem, _machInst, __opClass, _microVl,
                             _microIdx, 0)
        , regIdx(_regIdx)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class VlIndexMacroInst : public VectorMemMacroInst
{
  protected:
    VlIndexMacroInst(const char* mnem, ExtMachInst _machInst,
                   OpClass __opClass, uint32_t _vlen)
        : VectorMemMacroInst(mnem, _machInst, __opClass, _vlen)
    {}

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

class VlIndexMicroInst : public VectorMemMicroInst
{
  protected:
    uint32_t vdRegIdx;
    uint32_t vdElemIdx;
    uint32_t vs2RegIdx;
    uint32_t vs2ElemIdx;
    VlIndexMicroInst(const char *mnem, ExtMachInst _machInst,
                    OpClass __opClass, uint32_t _vdRegIdx, uint32_t _vdElemIdx,
                    uint32_t _vs2RegIdx, uint32_t _vs2ElemIdx)
        : VectorMemMicroInst(mnem, _machInst, __opClass, 1,
                             0, 0)
        , vdRegIdx(_vdRegIdx), vdElemIdx(_vdElemIdx)
        , vs2RegIdx(_vs2RegIdx), vs2ElemIdx(_vs2ElemIdx)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class VsIndexMacroInst : public VectorMemMacroInst
{
  protected:
    VsIndexMacroInst(const char* mnem, ExtMachInst _machInst,
                   OpClass __opClass, uint32_t _vlen)
        : VectorMemMacroInst(mnem, _machInst, __opClass, _vlen)
    {}

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

class VsIndexMicroInst : public VectorMemMicroInst
{
  protected:
    uint32_t vs3RegIdx;
    uint32_t vs3ElemIdx;
    uint32_t vs2RegIdx;
    uint32_t vs2ElemIdx;
    VsIndexMicroInst(const char *mnem, ExtMachInst _machInst,
                    OpClass __opClass, uint32_t _vs3RegIdx,
                    uint32_t _vs3ElemIdx, uint32_t _vs2RegIdx,
                    uint32_t _vs2ElemIdx)
        : VectorMemMicroInst(mnem, _machInst, __opClass, 1, 0, 0),
          vs3RegIdx(_vs3RegIdx), vs3ElemIdx(_vs3ElemIdx),
          vs2RegIdx(_vs2RegIdx), vs2ElemIdx(_vs2ElemIdx)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class VMvWholeMacroInst : public VectorArithMacroInst
{
  protected:
    VMvWholeMacroInst(const char* mnem, ExtMachInst _machInst,
                         OpClass __opClass)
        : VectorArithMacroInst(mnem, _machInst, __opClass)
    {}

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

class VMvWholeMicroInst : public VectorArithMicroInst
{
  protected:
    VMvWholeMicroInst(const char *mnem, ExtMachInst _machInst,
                         OpClass __opClass, uint32_t _microVl,
                         uint32_t _microIdx)
        : VectorArithMicroInst(mnem, _machInst, __opClass, _microVl, _microIdx)
    {}

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};


class VMaskMergeMicroInst : public VectorArithMicroInst
{
  private:
    RegId srcRegIdxArr[NumVecInternalRegs];
    RegId destRegIdxArr[1];

  public:
    uint32_t vlen;
    size_t elemSize;
    VMaskMergeMicroInst(ExtMachInst extMachInst,
        uint8_t _dstReg, uint8_t _numSrcs, uint32_t _vlen, size_t _elemSize);
    Fault execute(ExecContext *, trace::InstRecord *) const override;
    std::string generateDisassembly(Addr,
        const loader::SymbolTable *) const override;
};

class VxsatMicroInst : public VectorArithMicroInst
{
  private:
    bool* vxsat;
  public:
    VxsatMicroInst(bool* Vxsat, ExtMachInst extMachInst)
        : VectorArithMicroInst("vxsat_micro", extMachInst,
          VectorIntegerArithOp, 0, 0)
    {
        vxsat = Vxsat;
    }
    Fault execute(ExecContext *, trace::InstRecord *) const override;
    std::string generateDisassembly(Addr, const loader::SymbolTable *)
        const override;
};

class VlFFTrimVlMicroOp : public VectorMicroInst
{
  private:
    RegId srcRegIdxArr[8];
    RegId destRegIdxArr[0];
    std::vector<StaticInstPtr>& microops;

  public:
    VlFFTrimVlMicroOp(ExtMachInst _machInst, uint32_t _microVl,
        uint32_t _microIdx, uint32_t _vlen,
        std::vector<StaticInstPtr>& _microops);
    uint32_t calcVl() const;
    Fault execute(ExecContext *, trace::InstRecord *) const override;
    std::unique_ptr<PCStateBase> branchTarget(ThreadContext *) const override;
    std::string generateDisassembly(Addr, const loader::SymbolTable *)
        const override;
};

class VlSegMacroInst : public VectorMemMacroInst
{
  protected:
    VlSegMacroInst(const char* mnem, ExtMachInst _machInst,
                   OpClass __opClass, uint32_t _vlen)
        : VectorMemMacroInst(mnem, _machInst, __opClass, _vlen)
    {}

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

class VlSegMicroInst : public VectorMicroInst
{
  protected:
    Request::Flags memAccessFlags;
    uint8_t regIdx;

    VlSegMicroInst(const char *mnem, ExtMachInst _machInst,
                   OpClass __opClass, uint32_t _microVl,
                   uint32_t _microIdx, uint32_t _numMicroops,
                   uint32_t _field, uint32_t _numFields,
                   uint32_t _vlen)
        : VectorMicroInst(mnem, _machInst, __opClass, _microVl,
                          _microIdx, _vlen)
    {
      this->flags[IsLoad] = true;
    }

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class VlSegDeIntrlvMicroInst : public VectorArithMicroInst
{
  private:
    RegId srcRegIdxArr[NumVecInternalRegs];
    RegId destRegIdxArr[1];
    uint32_t numSrcs;
    uint32_t numMicroops;
    uint32_t field;
    uint32_t sizeOfElement;
    uint32_t micro_vl;

  public:
    uint32_t vlen;

    VlSegDeIntrlvMicroInst(ExtMachInst extMachInst, uint32_t _micro_vl,
                            uint32_t _dstReg, uint32_t _numSrcs,
                            uint32_t _microIdx, uint32_t _numMicroops,
                            uint32_t _field, uint32_t _vlen,
                            uint32_t _sizeOfElement);

    Fault execute(ExecContext *, trace::InstRecord *) const override;

    std::string generateDisassembly(Addr,
        const loader::SymbolTable *)  const override;
};

} // namespace RiscvISA
} // namespace gem5


#endif // __ARCH_RISCV_INSTS_VECTOR_HH__
