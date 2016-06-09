/*
 * Copyright (c) 2012-2015 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Steve Reinhardt
 */

#ifndef __ARCH_HSAIL_OPERAND_HH__
#define __ARCH_HSAIL_OPERAND_HH__

/**
 *  @file operand.hh
 *
 *  Defines classes encapsulating HSAIL instruction operands.
 */

#include <limits>
#include <string>

#include "arch/hsail/Brig.h"
#include "base/trace.hh"
#include "base/types.hh"
#include "debug/GPUReg.hh"
#include "enums/RegisterType.hh"
#include "gpu-compute/brig_object.hh"
#include "gpu-compute/compute_unit.hh"
#include "gpu-compute/hsail_code.hh"
#include "gpu-compute/shader.hh"
#include "gpu-compute/vector_register_file.hh"
#include "gpu-compute/wavefront.hh"

class Label;
class StorageElement;

class BaseOperand
{
  public:
    Enums::RegisterType registerType;
    uint32_t regOperandSize;
    BaseOperand() { registerType = Enums::RT_NONE; regOperandSize = 0; }
    bool isVectorRegister() { return registerType == Enums::RT_VECTOR; }
    bool isScalarRegister() { return registerType == Enums::RT_SCALAR; }
    bool isCondRegister() { return registerType == Enums::RT_CONDITION; }
    unsigned int regIndex() { return 0; }
    uint32_t opSize() { return regOperandSize; }
    virtual ~BaseOperand() { }
};

class BrigRegOperandInfo
{
  public:
    Brig::BrigKind16_t kind;
    Brig::BrigType type;
    Brig::BrigRegisterKind regKind;

    BrigRegOperandInfo(Brig::BrigKind16_t _kind,
                       Brig::BrigRegisterKind _regKind)
        : kind(_kind), regKind(_regKind)
    {
    }

    BrigRegOperandInfo(Brig::BrigKind16_t _kind, Brig::BrigType _type)
        : kind(_kind), type(_type)
    {
    }

    BrigRegOperandInfo() : kind(Brig::BRIG_KIND_OPERAND_CONSTANT_BYTES),
                           type(Brig::BRIG_TYPE_NONE)
    {
    }
};

BrigRegOperandInfo findRegDataType(unsigned opOffset, const BrigObject *obj);

class BaseRegOperand : public BaseOperand
{
  public:
    unsigned regIdx;
    char regFileChar;

    bool init(unsigned opOffset, const BrigObject *obj,
              unsigned &maxRegIdx, char _regFileChar);

    bool init_from_vect(unsigned opOffset, const BrigObject *obj, int at,
                        unsigned &maxRegIdx, char _regFileChar);

    void initWithStrOffset(unsigned strOffset, const BrigObject *obj,
                           unsigned &maxRegIdx, char _regFileChar);
    unsigned int regIndex() { return regIdx; }
};

class SRegOperand : public BaseRegOperand
{
  public:
    static unsigned maxRegIdx;

    bool
    init(unsigned opOffset, const BrigObject *obj)
    {
        regOperandSize = sizeof(uint32_t);
        registerType = Enums::RT_VECTOR;

        return BaseRegOperand::init(opOffset, obj, maxRegIdx, 's');
    }

    bool
    init_from_vect(unsigned opOffset, const BrigObject *obj, int at)
    {
        regOperandSize = sizeof(uint32_t);
        registerType = Enums::RT_VECTOR;

        return BaseRegOperand::init_from_vect(opOffset, obj, at, maxRegIdx,
                                              's');
    }

    void
    initWithStrOffset(unsigned strOffset, const BrigObject *obj)
    {
        regOperandSize = sizeof(uint32_t);
        registerType = Enums::RT_VECTOR;

        return BaseRegOperand::initWithStrOffset(strOffset, obj, maxRegIdx,
                                                 's');
    }

    template<typename OperandType>
    OperandType
    get(Wavefront *w, int lane)
    {
        assert(sizeof(OperandType) <= sizeof(uint32_t));
        assert(regIdx < w->maxSpVgprs);
        // if OperandType is smaller than 32-bit, we truncate the value
        OperandType ret;
        uint32_t vgprIdx;

        switch (sizeof(OperandType)) {
          case 1: // 1 byte operand
              vgprIdx = w->remap(regIdx, 1, 1);
              ret = (w->computeUnit->vrf[w->simdId]->
                      read<uint32_t>(vgprIdx, lane)) & 0xff;
            break;
          case 2: // 2 byte operand
              vgprIdx = w->remap(regIdx, 2, 1);
              ret = (w->computeUnit->vrf[w->simdId]->
                      read<uint32_t>(vgprIdx, lane)) & 0xffff;
            break;
          case 4: // 4 byte operand
              vgprIdx = w->remap(regIdx,sizeof(OperandType), 1);
              ret = w->computeUnit->vrf[w->simdId]->
                  read<OperandType>(vgprIdx, lane);
            break;
          default:
            panic("Bad OperandType\n");
            break;
        }

        return (OperandType)ret;
    }

    // special get method for compatibility with LabelOperand
    uint32_t
    getTarget(Wavefront *w, int lane)
    {
        return get<uint32_t>(w, lane);
    }

    template<typename OperandType>
    void set(Wavefront *w, int lane, OperandType &val);
    std::string disassemble();
};

template<typename OperandType>
void
SRegOperand::set(Wavefront *w, int lane, OperandType &val)
{
    DPRINTF(GPUReg, "CU%d, WF[%d][%d], lane %d: $s%d <- %d\n",
            w->computeUnit->cu_id, w->simdId, w->wfSlotId, lane, regIdx, val);

    assert(sizeof(OperandType) == sizeof(uint32_t));
    assert(regIdx < w->maxSpVgprs);
    uint32_t vgprIdx = w->remap(regIdx, sizeof(OperandType), 1);
    w->computeUnit->vrf[w->simdId]->write<OperandType>(vgprIdx,val,lane);
}

template<>
inline void
SRegOperand::set(Wavefront *w, int lane, uint64_t &val)
{
    DPRINTF(GPUReg, "CU%d, WF[%d][%d], lane %d: $s%d <- %d\n",
            w->computeUnit->cu_id, w->simdId, w->wfSlotId, lane, regIdx, val);

    assert(regIdx < w->maxSpVgprs);
    uint32_t vgprIdx = w->remap(regIdx, sizeof(uint32_t), 1);
    w->computeUnit->vrf[w->simdId]->write<uint32_t>(vgprIdx, val, lane);
}

class DRegOperand : public BaseRegOperand
{
  public:
    static unsigned maxRegIdx;

    bool
    init(unsigned opOffset, const BrigObject *obj)
    {
        regOperandSize = sizeof(uint64_t);
        registerType = Enums::RT_VECTOR;

        return BaseRegOperand::init(opOffset, obj, maxRegIdx, 'd');
    }

    bool
    init_from_vect(unsigned opOffset, const BrigObject *obj, int at)
    {
        regOperandSize = sizeof(uint64_t);
        registerType = Enums::RT_VECTOR;

        return BaseRegOperand::init_from_vect(opOffset, obj, at, maxRegIdx,
                                              'd');
    }

    void
    initWithStrOffset(unsigned strOffset, const BrigObject *obj)
    {
        regOperandSize = sizeof(uint64_t);
        registerType = Enums::RT_VECTOR;

        return BaseRegOperand::initWithStrOffset(strOffset, obj, maxRegIdx,
                                                 'd');
    }

    template<typename OperandType>
    OperandType
    get(Wavefront *w, int lane)
    {
        assert(sizeof(OperandType) <= sizeof(uint64_t));
        // TODO: this check is valid only for HSAIL
        assert(regIdx < w->maxDpVgprs);
        uint32_t vgprIdx = w->remap(regIdx, sizeof(OperandType), 1);

        return w->computeUnit->vrf[w->simdId]->read<OperandType>(vgprIdx,lane);
    }

    template<typename OperandType>
    void
    set(Wavefront *w, int lane, OperandType &val)
    {
        DPRINTF(GPUReg, "CU%d, WF[%d][%d], lane %d: $d%d <- %d\n",
                w->computeUnit->cu_id, w->simdId, w->wfSlotId, lane, regIdx,
                val);

        assert(sizeof(OperandType) <= sizeof(uint64_t));
        // TODO: this check is valid only for HSAIL
        assert(regIdx < w->maxDpVgprs);
        uint32_t vgprIdx = w->remap(regIdx, sizeof(OperandType), 1);
        w->computeUnit->vrf[w->simdId]->write<OperandType>(vgprIdx,val,lane);
    }

    std::string disassemble();
};

class CRegOperand : public BaseRegOperand
{
  public:
    static unsigned maxRegIdx;

    bool
    init(unsigned opOffset, const BrigObject *obj)
    {
        regOperandSize = sizeof(uint8_t);
        registerType = Enums::RT_CONDITION;

        return BaseRegOperand::init(opOffset, obj, maxRegIdx, 'c');
    }

    bool
    init_from_vect(unsigned opOffset, const BrigObject *obj, int at)
    {
        regOperandSize = sizeof(uint8_t);
        registerType = Enums::RT_CONDITION;

        return BaseRegOperand::init_from_vect(opOffset, obj, at, maxRegIdx,
                                              'c');
    }

    void
    initWithStrOffset(unsigned strOffset, const BrigObject *obj)
    {
        regOperandSize = sizeof(uint8_t);
        registerType = Enums::RT_CONDITION;

        return BaseRegOperand::initWithStrOffset(strOffset, obj, maxRegIdx,
                                                 'c');
    }

    template<typename OperandType>
    OperandType
    get(Wavefront *w, int lane)
    {
        assert(regIdx < w->condRegState->numRegs());

        return w->condRegState->read<OperandType>((int)regIdx, lane);
    }

    template<typename OperandType>
    void
    set(Wavefront *w, int lane, OperandType &val)
    {
        DPRINTF(GPUReg, "CU%d, WF[%d][%d], lane %d: $c%d <- %d\n",
                w->computeUnit->cu_id, w->simdId, w->wfSlotId, lane, regIdx,
                val);

        assert(regIdx < w->condRegState->numRegs());
        w->condRegState->write<OperandType>(regIdx,lane,val);
    }

    std::string disassemble();
};

template<typename T>
class ImmOperand : public BaseOperand
{
  private:
    uint16_t kind;
  public:
    T bits;

    bool init(unsigned opOffset, const BrigObject *obj);
    bool init_from_vect(unsigned opOffset, const BrigObject *obj, int at);
    std::string disassemble();

    template<typename OperandType>
    OperandType
    get(Wavefront *w)
    {
        assert(sizeof(OperandType) <= sizeof(T));
        panic_if(w == nullptr, "WF pointer needs to be set");

        switch (kind) {
          // immediate operand is WF size
          case Brig::BRIG_KIND_OPERAND_WAVESIZE:
            return (OperandType)w->computeUnit->wfSize();
            break;

          default:
            return *(OperandType*)&bits;
            break;
        }
    }

    // This version of get() takes a WF* and a lane id for
    // compatibility with the register-based get() methods.
    template<typename OperandType>
    OperandType
    get(Wavefront *w, int lane)
    {
        return get<OperandType>(w);
    }
};

template<typename T>
bool
ImmOperand<T>::init(unsigned opOffset, const BrigObject *obj)
{
    const Brig::BrigOperand *brigOp = obj->getOperand(opOffset);

    switch (brigOp->kind) {
      // this is immediate operand
      case Brig::BRIG_KIND_OPERAND_CONSTANT_BYTES:
        {
            DPRINTF(GPUReg, "sizeof(T): %lu, byteCount: %d\n", sizeof(T),
                    brigOp->byteCount);

            auto cbptr = (Brig::BrigOperandConstantBytes*)brigOp;

            bits = *((T*)(obj->getData(cbptr->bytes + 4)));
            kind = brigOp->kind;
            return true;
        }
        break;

      case Brig::BRIG_KIND_OPERAND_WAVESIZE:
        kind = brigOp->kind;
        bits = std::numeric_limits<unsigned long long>::digits;
        return true;

      default:
        kind = Brig::BRIG_KIND_NONE;
        return false;
    }
}

template <typename T>
bool
ImmOperand<T>::init_from_vect(unsigned opOffset, const BrigObject *obj, int at)
{
    const Brig::BrigOperand *brigOp = obj->getOperand(opOffset);

    if (brigOp->kind != Brig::BRIG_KIND_OPERAND_OPERAND_LIST) {
        kind = Brig::BRIG_KIND_NONE;
        return false;
    }


    const Brig::BrigOperandOperandList *brigVecOp =
         (const Brig::BrigOperandOperandList *)brigOp;

    unsigned *data_offset =
        (unsigned *)obj->getData(brigVecOp->elements + 4 * (at + 1));

    const Brig::BrigOperand *p =
        (const Brig::BrigOperand *)obj->getOperand(*data_offset);

    if (p->kind != Brig::BRIG_KIND_OPERAND_CONSTANT_BYTES) {
        kind = Brig::BRIG_KIND_NONE;
        return false;
    }

    return init(*data_offset, obj);
}
template<typename T>
std::string
ImmOperand<T>::disassemble()
{
    return csprintf("0x%08x", bits);
}

template<typename RegOperand, typename T>
class RegOrImmOperand : public BaseOperand
{
  private:
    bool is_imm;

  public:
    void setImm(const bool value) { is_imm = value; }

    ImmOperand<T> imm_op;
    RegOperand reg_op;

    RegOrImmOperand() { is_imm = false; }
    void init(unsigned opOffset, const BrigObject *obj);
    void init_from_vect(unsigned opOffset, const BrigObject *obj, int at);
    std::string disassemble();

    template<typename OperandType>
    OperandType
    get(Wavefront *w, int lane)
    {
        return is_imm ?  imm_op.template get<OperandType>(w) :
                         reg_op.template get<OperandType>(w, lane);
    }

    uint32_t
    opSize()
    {
        if (!is_imm) {
            return reg_op.opSize();
        }

        return 0;
    }

    bool
    isVectorRegister()
    {
        if (!is_imm) {
            return reg_op.registerType == Enums::RT_VECTOR;
        }
        return false;
    }

    bool
    isCondRegister()
    {
        if (!is_imm) {
            return reg_op.registerType == Enums::RT_CONDITION;
        }

        return false;
    }

    bool
    isScalarRegister()
    {
        if (!is_imm) {
            return reg_op.registerType == Enums::RT_SCALAR;
        }

        return false;
    }

    unsigned int
    regIndex()
    {
        if (!is_imm) {
            return reg_op.regIndex();
        }
        return 0;
    }
};

template<typename RegOperand, typename T>
void
RegOrImmOperand<RegOperand, T>::init(unsigned opOffset, const BrigObject *obj)
{
    is_imm = false;

    if (reg_op.init(opOffset, obj)) {
        return;
    }

    if (imm_op.init(opOffset, obj)) {
        is_imm = true;
        return;
    }

    fatal("RegOrImmOperand::init(): bad operand kind %d\n",
          obj->getOperand(opOffset)->kind);
}

template<typename RegOperand, typename T>
void
RegOrImmOperand<RegOperand, T>::init_from_vect(unsigned opOffset,
                                               const BrigObject *obj, int at)
{
    if (reg_op.init_from_vect(opOffset, obj, at)) {
        is_imm = false;

        return;
    }

    if (imm_op.init_from_vect(opOffset, obj, at)) {
        is_imm = true;

        return;
    }

    fatal("RegOrImmOperand::init(): bad operand kind %d\n",
          obj->getOperand(opOffset)->kind);
}

template<typename RegOperand, typename T>
std::string
RegOrImmOperand<RegOperand, T>::disassemble()
{
    return is_imm ? imm_op.disassemble() : reg_op.disassemble();
}

typedef RegOrImmOperand<SRegOperand, uint32_t> SRegOrImmOperand;
typedef RegOrImmOperand<DRegOperand, uint64_t> DRegOrImmOperand;
typedef RegOrImmOperand<CRegOperand, bool> CRegOrImmOperand;

class AddrOperandBase : public BaseOperand
{
  protected:
    // helper function for init()
    void parseAddr(const Brig::BrigOperandAddress *op, const BrigObject *obj);

    // helper function for disassemble()
    std::string disassemble(std::string reg_disassembly);
    uint64_t calcUniformBase();

  public:
    virtual void calcVector(Wavefront *w, std::vector<Addr> &addrVec) = 0;
    virtual uint64_t calcLane(Wavefront *w, int lane=0) = 0;

    uint64_t offset;
    const char *name = nullptr;
    StorageElement *storageElement;
};

template<typename RegOperandType>
class RegAddrOperand : public AddrOperandBase
{
  public:
    RegOperandType reg;
    void init(unsigned opOffset, const BrigObject *obj);
    uint64_t calcUniform();
    void calcVector(Wavefront *w, std::vector<Addr> &addrVec);
    uint64_t calcLane(Wavefront *w, int lane=0);
    uint32_t opSize() { return reg.opSize(); }
    bool isVectorRegister() { return reg.registerType == Enums::RT_VECTOR; }
    bool isCondRegister() { return reg.registerType == Enums::RT_CONDITION; }
    bool isScalarRegister() { return reg.registerType == Enums::RT_SCALAR; }
    unsigned int regIndex() { return reg.regIndex(); }
    std::string disassemble();
};

template<typename RegOperandType>
void
RegAddrOperand<RegOperandType>::init(unsigned opOffset, const BrigObject *obj)
{
    using namespace Brig;

    const BrigOperand *baseOp = obj->getOperand(opOffset);

    switch (baseOp->kind) {
      case BRIG_KIND_OPERAND_ADDRESS:
        {
            const BrigOperandAddress *op = (BrigOperandAddress*)baseOp;
            storageElement = nullptr;

            offset = (uint64_t(op->offset.hi) << 32) | uint64_t(op->offset.lo);
            reg.init(op->reg, obj);

            if (reg.regFileChar == 's') {
                reg.regOperandSize = sizeof(uint32_t);
                registerType = Enums::RT_VECTOR;
            }
            else if (reg.regFileChar == 'd') {
                reg.regOperandSize = sizeof(uint64_t);
                registerType = Enums::RT_VECTOR;
            }
        }
        break;

      default:
        fatal("RegAddrOperand: bad operand kind %d\n", baseOp->kind);
        break;
    }
}

template<typename RegOperandType>
uint64_t
RegAddrOperand<RegOperandType>::calcUniform()
{
    fatal("can't do calcUniform() on register-based address\n");

    return 0;
}

template<typename RegOperandType>
void
RegAddrOperand<RegOperandType>::calcVector(Wavefront *w,
                                           std::vector<Addr> &addrVec)
{
    Addr address = calcUniformBase();

    for (int lane = 0; lane < w->computeUnit->wfSize(); ++lane) {
        if (w->execMask(lane)) {
            if (reg.regFileChar == 's') {
                addrVec[lane] = address + reg.template get<uint32_t>(w, lane);
            } else {
                addrVec[lane] = address + reg.template get<Addr>(w, lane);
            }
        }
    }
}

template<typename RegOperandType>
uint64_t
RegAddrOperand<RegOperandType>::calcLane(Wavefront *w, int lane)
{
    Addr address = calcUniformBase();

    return address + reg.template get<Addr>(w, lane);
}

template<typename RegOperandType>
std::string
RegAddrOperand<RegOperandType>::disassemble()
{
    return AddrOperandBase::disassemble(reg.disassemble());
}

typedef RegAddrOperand<SRegOperand> SRegAddrOperand;
typedef RegAddrOperand<DRegOperand> DRegAddrOperand;

class NoRegAddrOperand : public AddrOperandBase
{
  public:
    void init(unsigned opOffset, const BrigObject *obj);
    uint64_t calcUniform();
    void calcVector(Wavefront *w, std::vector<Addr> &addrVec);
    uint64_t calcLane(Wavefront *w, int lane=0);
    std::string disassemble();
};

inline uint64_t
NoRegAddrOperand::calcUniform()
{
    return AddrOperandBase::calcUniformBase();
}

inline uint64_t
NoRegAddrOperand::calcLane(Wavefront *w, int lane)
{
    return calcUniform();
}

inline void
NoRegAddrOperand::calcVector(Wavefront *w, std::vector<Addr> &addrVec)
{
    uint64_t address = calcUniformBase();

    for (int lane = 0; lane < w->computeUnit->wfSize(); ++lane)
        addrVec[lane] = address;
}

class LabelOperand : public BaseOperand
{
  public:
    Label *label;

    void init(unsigned opOffset, const BrigObject *obj);
    std::string disassemble();

    // special get method for compatibility with SRegOperand
    uint32_t getTarget(Wavefront *w, int lane);

};

class ListOperand : public BaseOperand
{
  public:
    int elementCount;
    std::vector<StorageElement*> callArgs;

    int
    getSrcOperand(int idx)
    {
        DPRINTF(GPUReg, "getSrcOperand, idx: %d, sz_args: %d\n", idx,
                callArgs.size());

        return callArgs.at(idx)->offset;
    }

    void init(unsigned opOffset, const BrigObject *obj);

    std::string disassemble();

    template<typename OperandType>
    OperandType
    get(Wavefront *w, int lane, int arg_idx)
    {
        return w->readCallArgMem<OperandType>(lane, getSrcOperand(arg_idx));
    }

    template<typename OperandType>
    void
    set(Wavefront *w, int lane, OperandType val)
    {
        w->writeCallArgMem<OperandType>(lane, getSrcOperand(0), val);
        DPRINTF(GPUReg, "CU%d, WF[%d][%d], lane %d: arg[%d] <- %d\n",
                w->computeUnit->cu_id, w->simdId, w->wfSlotId, lane,
                getSrcOperand(0), val);
    }
};

class FunctionRefOperand : public BaseOperand
{
  public:
    const char *func_name;

    void init(unsigned opOffset, const BrigObject *obj);
    std::string disassemble();
};

#endif // __ARCH_HSAIL_OPERAND_HH__
