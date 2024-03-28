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

#include "arch/riscv/insts/vector.hh"

#include <sstream>
#include <string>

#include "arch/riscv/insts/static_inst.hh"
#include "arch/riscv/isa.hh"
#include "arch/riscv/regs/misc.hh"
#include "arch/riscv/regs/vector.hh"
#include "arch/riscv/utility.hh"
#include "cpu/static_inst.hh"

namespace gem5
{

namespace RiscvISA
{

/**
 * This function translates the 3-bit value of vlmul bits to the corresponding
 * lmul value as specified in RVV 1.0 spec p11-12 chapter 3.4.2.
 *
 * I.e.,
 * vlmul = -3 -> LMUL = 1/8
 * vlmul = -2 -> LMUL = 1/4
 * vlmul = -1 -> LMUL = 1/2
 * vlmul = 0 -> LMUL = 1
 * vlmul = 1 -> LMUL = 2
 * vlmul = 2 -> LMUL = 4
 * vlmul = 3 -> LMUL = 8
 *
**/
float
getVflmul(uint32_t vlmul_encoding)
{
    int vlmul = sext<3>(vlmul_encoding & 7);
    float vflmul = vlmul >= 0 ? 1 << vlmul : 1.0 / (1 << -vlmul);
    return vflmul;
}

uint32_t
getVlmax(VTYPE vtype, uint32_t vlen)
{
    uint32_t sew = getSew(vtype.vsew);
    // vlmax is defined in RVV 1.0 spec p12 chapter 3.4.2.
    uint32_t vlmax = (vlen/sew) * getVflmul(vtype.vlmul);
    return vlmax;
}

std::string
VConfOp::generateDisassembly(Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", ";
    if (bit31 && bit30 == 0) {
        ss << registerName(srcRegIdx(0)) << ", " << registerName(srcRegIdx(1));
    } else if (bit31 && bit30) {
        ss << uimm << ", " << generateZimmDisassembly();
    } else {
        ss << registerName(srcRegIdx(0)) << ", " << generateZimmDisassembly();
    }
    return ss.str();
}

std::string
VConfOp::generateZimmDisassembly() const
{
    std::stringstream s;

    // VSETIVLI uses ZIMM10 and VSETVLI uses ZIMM11
    uint64_t zimm = (bit31 && bit30) ? zimm10 : zimm11;

    bool frac_lmul = bits(zimm, 2);
    int sew = 1 << (bits(zimm, 5, 3) + 3);
    int lmul = bits(zimm, 1, 0);
    auto vta = bits(zimm, 6) == 1 ? "ta" : "tu";
    auto vma = bits(zimm, 7) == 1 ? "ma" : "mu";
    s << "e" << sew;
    if (frac_lmul) {
        std::string lmul_str = "";
        switch(lmul){
        case 3:
            lmul_str = "f2";
            break;
        case 2:
            lmul_str = "f4";
            break;
        case 1:
            lmul_str = "f8";
            break;
        default:
            panic("Unsupport fractional LMUL");
        }
        s << ", m" << lmul_str;
    } else {
        s << ", m" << (1 << lmul);
    }
    s << ", " << vta << ", " << vma;
    return s.str();
}

std::string
VectorNonSplitInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", "
        << registerName(srcRegIdx(0));
    if (machInst.vm == 0) ss << ", v0.t";
    return ss.str();
}

std::string VectorArithMicroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", ";
    if (machInst.funct3 == 0x3) {
        // OPIVI
      ss  << registerName(srcRegIdx(0)) << ", " << machInst.vecimm;
    } else {
      ss  << registerName(srcRegIdx(1)) << ", " << registerName(srcRegIdx(0));
    }
    if (machInst.vm == 0) ss << ", v0.t";
    return ss.str();
}

std::string VectorArithMacroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", ";
    if (machInst.funct3 == 0x3) {
        // OPIVI
      ss  << registerName(srcRegIdx(0)) << ", " << machInst.vecimm;
    } else {
      ss  << registerName(srcRegIdx(1)) << ", " << registerName(srcRegIdx(0));
    }
    if (machInst.vm == 0) ss << ", v0.t";
    return ss.str();
}

std::string VectorVMUNARY0MicroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0));
    if (machInst.vm == 0) ss << ", v0.t";
    return ss.str();
}

std::string VectorVMUNARY0MacroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0));
    if (machInst.vm == 0) ss << ", v0.t";
    return ss.str();
}

std::string VectorSlideMicroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) <<  ", ";
    if (machInst.funct3 == 0x3) {
      ss  << registerName(srcRegIdx(0)) << ", " << machInst.vecimm;
    } else {
      ss  << registerName(srcRegIdx(1)) << ", " << registerName(srcRegIdx(0));
    }
    if (machInst.vm == 0) ss << ", v0.t";
    return ss.str();
}

std::string VectorSlideMacroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", ";
    if (machInst.funct3 == 0x3) {
      ss  << registerName(srcRegIdx(0)) << ", " << machInst.vecimm;
    } else {
      ss  << registerName(srcRegIdx(1)) << ", " << registerName(srcRegIdx(0));
    }
    if (machInst.vm == 0) ss << ", v0.t";
    return ss.str();
}

std::string VleMicroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    unsigned vlenb = vlen >> 3;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", "
       << vlenb * microIdx << '(' << registerName(srcRegIdx(0)) << ')' << ", "
       << registerName(srcRegIdx(1));
    if (!machInst.vm) ss << ", v0.t";
    return ss.str();
}

std::string VlWholeMicroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    unsigned vlenb = vlen >> 3;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", "
       << vlenb * microIdx << '(' << registerName(srcRegIdx(0)) << ')';
    return ss.str();
}

std::string VseMicroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    unsigned vlenb = vlen >> 3;
    ss << mnemonic << ' ' << registerName(srcRegIdx(1)) << ", "
       << vlenb * microIdx  << '(' << registerName(srcRegIdx(0)) << ')';
    if (!machInst.vm) ss << ", v0.t";
    return ss.str();
}

std::string VsWholeMicroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    unsigned vlenb = vlen >> 3;
    ss << mnemonic << ' ' << registerName(srcRegIdx(1)) << ", "
       << vlenb * microIdx << '(' << registerName(srcRegIdx(0)) << ')';
    return ss.str();
}

std::string VleMacroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", " <<
        '(' << registerName(srcRegIdx(0)) << ')';
    if (!machInst.vm) ss << ", v0.t";
    return ss.str();
}

std::string VlWholeMacroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", " <<
        '(' << registerName(srcRegIdx(0)) << ')';
    return ss.str();
}

std::string VseMacroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(srcRegIdx(1)) << ", " <<
        '(' << registerName(srcRegIdx(0)) << ')';
    if (!machInst.vm) ss << ", v0.t";
    return ss.str();
}

std::string VsWholeMacroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(srcRegIdx(1)) << ", " <<
        '(' << registerName(srcRegIdx(0)) << ')';
    return ss.str();
}

std::string VlStrideMacroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", " <<
        '(' << registerName(srcRegIdx(0)) << ')' <<
        ", " << registerName(srcRegIdx(1));
    if (!machInst.vm) ss << ", v0.t";
    return ss.str();
}

std::string VlStrideMicroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", " <<
        '(' << registerName(srcRegIdx(0)) << ')' <<
        ", "<< registerName(srcRegIdx(1));
    if (microIdx != 0 || machInst.vtype8.vma == 0 || machInst.vtype8.vta == 0)
        ss << ", " << registerName(srcRegIdx(2));
    if (!machInst.vm) ss << ", v0.t";
    return ss.str();
}

std::string VsStrideMacroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(srcRegIdx(2)) << ", " <<
        '(' << registerName(srcRegIdx(0)) << ')' <<
        ", " << registerName(srcRegIdx(1));
    if (!machInst.vm) ss << ", v0.t";
    return ss.str();
}

std::string VsStrideMicroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(srcRegIdx(2)) << ", " <<
        '(' << registerName(srcRegIdx(0)) << ')' <<
        ", "<< registerName(srcRegIdx(1));
    if (microIdx != 0 || machInst.vtype8.vma == 0 || machInst.vtype8.vta == 0)
        ss << ", " << registerName(srcRegIdx(2));
    if (!machInst.vm) ss << ", v0.t";
    return ss.str();
}

std::string VlIndexMacroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", "
        << '(' << registerName(srcRegIdx(0)) << "),"
        << registerName(srcRegIdx(1));
    if (!machInst.vm) ss << ", v0.t";
    return ss.str();
}

std::string VlIndexMicroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' '
        << registerName(destRegIdx(0)) << "[" << uint16_t(vdElemIdx) << "], "
        << '(' << registerName(srcRegIdx(0)) << "), "
        << registerName(srcRegIdx(1)) << "[" << uint16_t(vs2ElemIdx) << "]";
    if (microIdx != 0 || machInst.vtype8.vma == 0 || machInst.vtype8.vta == 0)
        ss << ", " << registerName(srcRegIdx(2));
    if (!machInst.vm) ss << ", v0.t";
    return ss.str();
}

std::string VsIndexMacroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(srcRegIdx(2)) << ", "
        << '(' << registerName(srcRegIdx(0)) << "),"
        << registerName(srcRegIdx(1));
    if (!machInst.vm) ss << ", v0.t";
    return ss.str();
}

std::string VsIndexMicroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' '
        << registerName(srcRegIdx(2)) << "[" << uint16_t(vs3ElemIdx) << "], "
        << '(' << registerName(srcRegIdx(0)) << "), "
        << registerName(srcRegIdx(1)) << "[" << uint16_t(vs2ElemIdx) << "]";
    if (!machInst.vm) ss << ", v0.t";
    return ss.str();
}

std::string
VMvWholeMacroInst::generateDisassembly(Addr pc,
    const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", " <<
        registerName(srcRegIdx(1));
    return ss.str();
}

std::string
VMvWholeMicroInst::generateDisassembly(Addr pc,
    const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", " <<
        registerName(srcRegIdx(1));
    return ss.str();
}

VMaskMergeMicroInst::VMaskMergeMicroInst(ExtMachInst extMachInst,
    uint8_t _dstReg, uint8_t _numSrcs, uint32_t _vlen, size_t _elemSize)
    : VectorArithMicroInst("vmask_mv_micro", extMachInst,
                            VectorIntegerArithOp, 0, 0),
      vlen(_vlen),
      elemSize(_elemSize)
{
    setRegIdxArrays(
        reinterpret_cast<RegIdArrayPtr>(
            &std::remove_pointer_t<decltype(this)>::srcRegIdxArr),
        reinterpret_cast<RegIdArrayPtr>(
            &std::remove_pointer_t<decltype(this)>::destRegIdxArr));

    _numSrcRegs = 0;
    _numDestRegs = 0;

    setDestRegIdx(_numDestRegs++, vecRegClass[_dstReg]);
    _numTypedDestRegs[VecRegClass]++;
    for (uint8_t i=0; i<_numSrcs; i++) {
        setSrcRegIdx(_numSrcRegs++, vecRegClass[VecMemInternalReg0 + i]);
    }
}

Fault
VMaskMergeMicroInst::execute(ExecContext* xc,
    trace::InstRecord* traceData) const
{
    vreg_t& tmp_d0 = *(vreg_t *)xc->getWritableRegOperand(this, 0);
    PCStateBase *pc_ptr = xc->tcBase()->pcState().clone();
    auto Vd = tmp_d0.as<uint8_t>();
    uint32_t vlenb = pc_ptr->as<PCState>().vlenb();
    const uint32_t elems_per_vreg = vlenb / elemSize;
    size_t bit_cnt = elems_per_vreg;
    vreg_t tmp_s;
    xc->getRegOperand(this, 0, &tmp_s);
    auto s = tmp_s.as<uint8_t>();
    // cp the first result and tail
    memcpy(Vd, s, vlenb);
    for (uint8_t i = 1; i < this->_numSrcRegs; i++) {
        xc->getRegOperand(this, i, &tmp_s);
        s = tmp_s.as<uint8_t>();
        if (elems_per_vreg < 8) {
            const uint32_t m = (1 << elems_per_vreg) - 1;
            const uint32_t mask = m << (i * elems_per_vreg % 8);
            // clr & ext bits
            Vd[bit_cnt/8] ^= Vd[bit_cnt/8] & mask;
            Vd[bit_cnt/8] |= s[bit_cnt/8] & mask;
            bit_cnt += elems_per_vreg;
        } else {
            const uint32_t byte_offset = elems_per_vreg / 8;
            memcpy(Vd + i * byte_offset, s + i * byte_offset, byte_offset);
        }
    }
    if (traceData)
        traceData->setData(vecRegClass, &tmp_d0);
    return NoFault;
}

std::string
VMaskMergeMicroInst::generateDisassembly(Addr pc,
    const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0));
    for (uint8_t i = 0; i < this->_numSrcRegs; i++) {
        ss << ", " << registerName(srcRegIdx(i));
    }
    unsigned vlenb = vlen >> 3;
    ss << ", offset:" << vlenb / elemSize;
    return ss.str();
}

Fault
VxsatMicroInst::execute(ExecContext* xc, trace::InstRecord* traceData) const
{
    xc->setMiscReg(MISCREG_VXSAT, *vxsat);
    auto vcsr = xc->readMiscReg(MISCREG_VCSR);
    xc->setMiscReg(MISCREG_VCSR, ((vcsr&~1)|*vxsat));
    return NoFault;
}

std::string
VxsatMicroInst::generateDisassembly(Addr pc,
    const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << "VXSAT" << ", " << (*vxsat ? "0x1" : "0x0");
    return ss.str();
}

VlFFTrimVlMicroOp::VlFFTrimVlMicroOp(ExtMachInst _machInst, uint32_t _microVl,
    uint32_t _microIdx, uint32_t _vlen, std::vector<StaticInstPtr>& _microops)
    : VectorMicroInst("vlff_trimvl_v_micro", _machInst, VectorConfigOp,
                      _microVl, _microIdx, _vlen),
      microops(_microops)
{
    setRegIdxArrays(
        reinterpret_cast<RegIdArrayPtr>(
            &std::remove_pointer_t<decltype(this)>::srcRegIdxArr),
        nullptr
    );

    // Create data dependency with load micros
    for (uint8_t i=0; i<microIdx; i++) {
        setSrcRegIdx(_numSrcRegs++, vecRegClass[_machInst.vd + i]);
    }

    this->flags[IsControl] = true;
    this->flags[IsIndirectControl] = true;
    this->flags[IsInteger] = true;
    this->flags[IsUncondControl] = true;
}

uint32_t
VlFFTrimVlMicroOp::calcVl() const
{
    uint32_t vl = 0;
    for (uint8_t i=0; i<microIdx; i++) {
        VleMicroInst& micro = static_cast<VleMicroInst&>(*microops[i]);
        vl += micro.faultIdx;

        if (micro.trimVl)
            break;
    }
    return vl;
}

Fault
VlFFTrimVlMicroOp::execute(ExecContext *xc, trace::InstRecord *traceData) const
{
    auto tc = xc->tcBase();
    MISA misa = xc->readMiscReg(MISCREG_ISA);
    STATUS status = xc->readMiscReg(MISCREG_STATUS);
    if (!misa.rvv || status.vs == VPUStatus::OFF) {
        return std::make_shared<IllegalInstFault>(
                "RVV is disabled or VPU is off", machInst);
    }

    PCState pc;
    set(pc, xc->pcState());

    uint32_t new_vl = calcVl();

    tc->setMiscReg(MISCREG_VSTART, 0);

    RegVal final_val = new_vl;
    if (traceData) {
        traceData->setData(miscRegClass, final_val);
    }

    pc.vl(new_vl);
    xc->pcState(pc);

    return NoFault;
}

std::unique_ptr<PCStateBase>
VlFFTrimVlMicroOp::branchTarget(ThreadContext *tc) const
{
    PCStateBase *pc_ptr = tc->pcState().clone();

    uint32_t new_vl = calcVl();

    pc_ptr->as<PCState>().vl(new_vl);
    return std::unique_ptr<PCStateBase>{pc_ptr};
}

std::string
VlFFTrimVlMicroOp::generateDisassembly(Addr pc,
    const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << " vl";
    return ss.str();
}

std::string VlSegMacroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", " <<
        '(' << registerName(srcRegIdx(0)) << ')' <<
        ", " << registerName(srcRegIdx(1));
    if (!machInst.vm)
        ss << ", v0.t";
    return ss.str();
}

std::string VlSegMicroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", " <<
        '(' << registerName(srcRegIdx(0)) << ')' <<
        ", "<< registerName(srcRegIdx(1));
    if (microIdx != 0 || machInst.vtype8.vma == 0 || machInst.vtype8.vta == 0)
        ss << ", " << registerName(srcRegIdx(2));
    if (!machInst.vm)
        ss << ", v0.t";
    return ss.str();
}

VlSegDeIntrlvMicroInst::VlSegDeIntrlvMicroInst(ExtMachInst extMachInst, uint32_t _micro_vl,
                        uint32_t _dstReg, uint32_t _numSrcs,
                        uint32_t _microIdx, uint32_t _numMicroops,
                        uint32_t _field, uint32_t _vlen, uint32_t _sizeOfElement)
    : VectorArithMicroInst("vlseg_deintrlv_micro", extMachInst,
                            VectorIntegerArithOp, 0, 0),
        vlen(_vlen)
{
    setRegIdxArrays(
        reinterpret_cast<RegIdArrayPtr>(
            &std::remove_pointer_t<decltype(this)>::srcRegIdxArr),
        reinterpret_cast<RegIdArrayPtr>(
            &std::remove_pointer_t<decltype(this)>::destRegIdxArr));

    _numSrcRegs = 0;
    _numDestRegs = 0;
    numSrcs = _numSrcs;
    numMicroops = _numMicroops;
    field =_field;
    sizeOfElement = _sizeOfElement;
    microIdx = _microIdx;
    micro_vl = _micro_vl;

    setDestRegIdx(_numDestRegs++, vecRegClass[_dstReg]);
    _numTypedDestRegs[VecRegClass]++;
    for (uint32_t i=0; i < _numSrcs; i++) {
        uint32_t index = VecMemInternalReg0 + i + (microIdx * _numSrcs);
        setSrcRegIdx(_numSrcRegs++, vecRegClass[index]);
    }
}

Fault
VlSegDeIntrlvMicroInst::execute(ExecContext* xc, trace::InstRecord* traceData) const
{
    vreg_t& tmp_d0 = *(vreg_t *)xc->getWritableRegOperand(this, 0);
    auto Vd = tmp_d0.as<uint8_t>();
    const uint32_t elems_per_vreg =  micro_vl;
    vreg_t tmp_s;
    auto s = tmp_s.as<uint8_t>();
    uint32_t elem = 0;
    uint32_t index = field;
    for (uint32_t i = 0; i < numSrcs; i++) {
        xc->getRegOperand(this, i, &tmp_s);
        s = tmp_s.as<uint8_t>();
        while(index < (i + 1) * elems_per_vreg)
        {
            memcpy(Vd + (elem * sizeOfElement),
                    s + ((index  %  elems_per_vreg) * sizeOfElement),
                    sizeOfElement);
            index += numSrcs;
            elem++;
        }
    }
    if (traceData)
        traceData->setData(vecRegClass, &tmp_d0);
    return NoFault;
}

std::string
VlSegDeIntrlvMicroInst::generateDisassembly(Addr pc, const loader::SymbolTable *symtab)
    const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0));
    for (uint8_t i = 0; i < this->_numSrcRegs; i++) {
        ss << ", " << registerName(srcRegIdx(i));
    }
    ss << ", field: " << field;
    return ss.str();
}

std::string VsSegMacroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", " <<
        '(' << registerName(srcRegIdx(0)) << ')' <<
        ", " << registerName(srcRegIdx(1));
    if (!machInst.vm)
        ss << ", v0.t";
    return ss.str();
}

std::string VsSegMicroInst::generateDisassembly(Addr pc,
        const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0)) << ", " <<
        '(' << registerName(srcRegIdx(0)) << ')' <<
        ", "<< registerName(srcRegIdx(1));
    if (microIdx != 0 || machInst.vtype8.vma == 0 || machInst.vtype8.vta == 0)
        ss << ", " << registerName(srcRegIdx(2));
    if (!machInst.vm)
        ss << ", v0.t";
    return ss.str();
}

VsSegIntrlvMicroInst::VsSegIntrlvMicroInst(ExtMachInst extMachInst, uint32_t _micro_vl,
                        uint32_t _dstReg, uint32_t _numSrcs,
                        uint32_t _microIdx, uint32_t _numMicroops,
                        uint32_t _field, uint32_t _vlen, uint32_t _sizeOfElement)
    : VectorArithMicroInst("vsseg_reintrlv_micro", extMachInst,
                            VectorIntegerArithOp, 0, 0),
        vlen(_vlen)
{
    setRegIdxArrays(
        reinterpret_cast<RegIdArrayPtr>(
            &std::remove_pointer_t<decltype(this)>::srcRegIdxArr),
        reinterpret_cast<RegIdArrayPtr>(
            &std::remove_pointer_t<decltype(this)>::destRegIdxArr));

    _numSrcRegs = 0;
    _numDestRegs = 0;
    numSrcs = _numSrcs;
    numMicroops = _numMicroops;
    field =_field;
    sizeOfElement = _sizeOfElement;
    microIdx = _microIdx;
    micro_vl = _micro_vl;

    setDestRegIdx(_numDestRegs++, vecRegClass[VecMemInternalReg0 + field +
        (_microIdx * numSrcs)]);

    _numTypedDestRegs[VecRegClass]++;
    for (uint8_t i=0; i<_numSrcs; i++) {
        setSrcRegIdx(_numSrcRegs++, vecRegClass[_dstReg + (i * numMicroops) +
            (microIdx)]);
    }
}

Fault
VsSegIntrlvMicroInst::execute(ExecContext* xc,
    trace::InstRecord* traceData) const
{
    const uint32_t elems_per_vreg = micro_vl;
    vreg_t& tmp_d0 = *(vreg_t *)xc->getWritableRegOperand(this, 0);
    auto Vd = tmp_d0.as<uint8_t>();

    vreg_t tmp_s;
    auto s = tmp_s.as<uint8_t>();
    xc->getRegOperand(this, 0, &tmp_s);
    s = tmp_s.as<uint8_t>();

    uint32_t indexVd = 0;
    uint32_t srcReg = (field * elems_per_vreg) % numSrcs;
    uint32_t indexs = (field * elems_per_vreg) / numSrcs;

    while (indexVd < elems_per_vreg) {
        xc->getRegOperand(this, srcReg, &tmp_s);
        s = tmp_s.as<uint8_t>();

        memcpy(Vd + (indexVd * sizeOfElement),
                    s + (indexs * sizeOfElement),
                    sizeOfElement);

        indexVd++;
        srcReg++;
        if (srcReg >= numSrcs) {
            srcReg = 0;
            indexs++;
        }
    }

    if (traceData)
        traceData->setData(vecRegClass, &tmp_d0);
    return NoFault;
}

std::string
VsSegIntrlvMicroInst::generateDisassembly(Addr pc,
    const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << ' ' << registerName(destRegIdx(0));
    for (uint8_t i = 0; i < this->_numSrcRegs; i++) {
        ss << ", " << registerName(srcRegIdx(i));
    }
    ss << ", field: " << field;
    return ss.str();
}

} // namespace RiscvISA
} // namespace gem5
