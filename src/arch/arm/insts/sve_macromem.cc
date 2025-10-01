/*
 * Copyright (c) 2020, 2025 ARM Limited
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
 */

#include "arch/arm/insts/sve_macromem.hh"

#include "arch/arm/generated/decoder.hh"

namespace gem5
{

namespace ArmISA
{

template <typename RegElemType, typename MemElemType,
          template <typename, typename> class MicroopType,
          template <typename> class FirstFaultWritebackMicroopType>
SveIndexedMemVI<RegElemType, MemElemType, MicroopType,
                FirstFaultWritebackMicroopType>::
    SveIndexedMemVI(const char *mnem, ExtMachInst machInst, OpClass __opClass,
                    RegIndex _dest, RegIndex _gp, RegIndex _base,
                    uint64_t _imm, bool firstFault)
    : PredMacroOp(mnem, machInst, __opClass),
      dest(_dest),
      gp(_gp),
      base(_base),
      imm(_imm)
{
    bool isLoad = (__opClass == MemReadOp);
    assert(!firstFault || isLoad);

    int num_elems = ((machInst.sveLen + 1) * 16) / sizeof(RegElemType);

    numMicroops = num_elems;
    if (isLoad) {
        if (firstFault) {
            numMicroops += 2;
        } else {
            numMicroops++;
        }
    }

    microOps = new StaticInstPtr[numMicroops];

    StaticInstPtr *uop = microOps;

    for (int i = 0; i < num_elems; i++, uop++) {
        *uop = new MicroopType<RegElemType, MemElemType>(
            mnem, machInst, __opClass, isLoad ? (RegIndex)VECREG_UREG0 : _dest,
            _gp, _base, _imm, i, num_elems, firstFault);
    }

    if (isLoad) {
        // The last microop of a gather load copies the auxiliary register
        // to the destination vector register. Because when any fault
        // occurs, the destination should keept the same.
        *uop = new ArmISAInst::SveGatherLoadCpyDstVecMicroop(mnem, machInst,
                                                             _dest, this);
        uop++;
    }

    if (firstFault) {
        *uop = new FirstFaultWritebackMicroopType<RegElemType>(
            mnem, machInst, __opClass, num_elems, this);
    } else {
        --uop;
    }

    (*uop)->setLastMicroop();
    microOps[0]->setFirstMicroop();

    for (StaticInstPtr *uop = microOps; !(*uop)->isLastMicroop(); uop++) {
        (*uop)->setDelayedCommit();
    }
}

template <typename RegElemType, typename MemElemType,
          template <typename, typename> class MicroopType,
          template <typename> class FirstFaultWritebackMicroopType>
SveIndexedMemSV<RegElemType, MemElemType, MicroopType,
                FirstFaultWritebackMicroopType>::
    SveIndexedMemSV(const char *mnem, ExtMachInst machInst, OpClass __opClass,
                    RegIndex _dest, RegIndex _gp, RegIndex _base,
                    RegIndex _offset, bool _offsetIs32, bool _offsetIsSigned,
                    bool _offsetIsScaled, bool firstFault)
    : PredMacroOp(mnem, machInst, __opClass),
      dest(_dest),
      gp(_gp),
      base(_base),
      offset(_offset),
      offsetIs32(_offsetIs32),
      offsetIsSigned(_offsetIsSigned),
      offsetIsScaled(_offsetIsScaled)
{
    bool isLoad = (__opClass == MemReadOp);
    assert(!firstFault || isLoad);

    int num_elems = ((machInst.sveLen + 1) * 16) / sizeof(RegElemType);

    numMicroops = num_elems;
    if (isLoad) {
        if (firstFault) {
            numMicroops += 2;
        } else {
            numMicroops++;
        }
    }

    microOps = new StaticInstPtr[numMicroops];

    StaticInstPtr *uop = microOps;

    for (int i = 0; i < num_elems; i++, uop++) {
        *uop = new MicroopType<RegElemType, MemElemType>(
            mnem, machInst, __opClass, isLoad ? (RegIndex)VECREG_UREG0 : _dest,
            _gp, _base, _offset, _offsetIs32, _offsetIsSigned, _offsetIsScaled,
            i, num_elems, firstFault);
    }

    if (isLoad) {
        // The last microop of a gather load copies the auxiliary register
        // to the destination vector register. Because when any fault
        // occurs, the destination should keept the same.
        *uop = new ArmISAInst::SveGatherLoadCpyDstVecMicroop(mnem, machInst,
                                                             _dest, this);
        uop++;
    }

    if (firstFault) {
        *uop = new FirstFaultWritebackMicroopType<RegElemType>(
            mnem, machInst, __opClass, num_elems, this);
    } else {
        --uop;
    }

    (*uop)->setLastMicroop();
    microOps[0]->setFirstMicroop();

    for (StaticInstPtr *uop = microOps; !(*uop)->isLastMicroop(); uop++) {
        (*uop)->setDelayedCommit();
    }
}
template <typename RegElemType, typename MemElemType,
          template <typename, typename> class MicroopType,
          template <typename> class FirstFaultWritebackMicroopType>
SveIndexedMemVS<RegElemType, MemElemType, MicroopType,
                FirstFaultWritebackMicroopType>::
    SveIndexedMemVS(const char *mnem, ExtMachInst machInst, OpClass __opClass,
                    RegIndex _dest, RegIndex _gp, RegIndex _base,
                    RegIndex _offset, bool _offsetIs32)
    : PredMacroOp(mnem, machInst, __opClass),
      dest(_dest),
      gp(_gp),
      base(_base),
      offset(_offset),
      offsetIs32(_offsetIs32)
{
    bool isLoad = (__opClass == MemReadOp);

    int num_elems = ((machInst.sveLen + 1) * 16) / sizeof(RegElemType);

    numMicroops = num_elems;
    if (isLoad) {
        numMicroops++;
    }

    microOps = new StaticInstPtr[numMicroops];

    StaticInstPtr *uop = microOps;

    for (int i = 0; i < num_elems; i++, uop++) {
        *uop = new MicroopType<RegElemType, MemElemType>(
            mnem, machInst, __opClass, isLoad ? (RegIndex)VECREG_UREG0 : _dest,
            _gp, _base, _offset, _offsetIs32, i, num_elems);
    }

    if (isLoad) {
        // The last microop of a gather load copies the auxiliary register
        // to the destination vector register. Because when any fault
        // occurs, the destination should keept the same.
        *uop = new ArmISAInst::SveGatherLoadCpyDstVecMicroop(mnem, machInst,
                                                             _dest, this);
        uop++;
    }

    --uop;

    (*uop)->setLastMicroop();
    microOps[0]->setFirstMicroop();

    for (StaticInstPtr *uop = microOps; !(*uop)->isLastMicroop(); uop++) {
        (*uop)->setDelayedCommit();
    }
}

#define SVE_GATHER_LOAD_VI(REGELEM, MEMELEM)                                  \
    template class SveIndexedMemVI<                                           \
        REGELEM, MEMELEM, ArmISAInst::SveGatherLoadVIMicroop,                 \
        ArmISAInst::SveFirstFaultWritebackMicroop>;

#define SVE_GATHER_LOAD_SV(REGELEM, MEMELEM)                                  \
    template class SveIndexedMemSV<                                           \
        REGELEM, MEMELEM, ArmISAInst::SveGatherLoadSVMicroop,                 \
        ArmISAInst::SveFirstFaultWritebackMicroop>;

#define SVE_SCATTER_STORE_VI(REGELEM, MEMELEM)                                \
    template class SveIndexedMemVI<                                           \
        REGELEM, MEMELEM, ArmISAInst::SveScatterStoreVIMicroop,               \
        ArmISAInst::SveFirstFaultWritebackMicroop>;

#define SVE_SCATTER_STORE_SV(REGELEM, MEMELEM)                                \
    template class SveIndexedMemSV<                                           \
        REGELEM, MEMELEM, ArmISAInst::SveScatterStoreSVMicroop,               \
        ArmISAInst::SveFirstFaultWritebackMicroop>;

#define SVE_GATHER_LOAD_VS(REGELEM, MEMELEM)                                  \
    template class SveIndexedMemVS<                                           \
        REGELEM, MEMELEM, ArmISAInst::SveGatherLoadVSMicroop,                 \
        ArmISAInst::SveFirstFaultWritebackMicroop>;

#define SVE_SCATTER_STORE_VS(REGELEM, MEMELEM)                                \
    template class SveIndexedMemVS<                                           \
        REGELEM, MEMELEM, ArmISAInst::SveScatterStoreVSMicroop,               \
        ArmISAInst::SveFirstFaultWritebackMicroop>;

SVE_GATHER_LOAD_VI(uint32_t, uint8_t)
SVE_GATHER_LOAD_VI(uint32_t, uint16_t)
SVE_GATHER_LOAD_VI(uint32_t, uint32_t)
SVE_GATHER_LOAD_VI(int32_t, int8_t)
SVE_GATHER_LOAD_VI(int32_t, int16_t)
SVE_GATHER_LOAD_VI(uint64_t, uint8_t)
SVE_GATHER_LOAD_VI(uint64_t, uint16_t)
SVE_GATHER_LOAD_VI(uint64_t, uint32_t)
SVE_GATHER_LOAD_VI(uint64_t, uint64_t)
SVE_GATHER_LOAD_VI(int64_t, int8_t)
SVE_GATHER_LOAD_VI(int64_t, int16_t)
SVE_GATHER_LOAD_VI(int64_t, int32_t)

SVE_GATHER_LOAD_SV(uint32_t, uint8_t)
SVE_GATHER_LOAD_SV(uint32_t, uint16_t)
SVE_GATHER_LOAD_SV(uint32_t, uint32_t)
SVE_GATHER_LOAD_SV(int32_t, int8_t)
SVE_GATHER_LOAD_SV(int32_t, int16_t)
SVE_GATHER_LOAD_SV(uint64_t, uint8_t)
SVE_GATHER_LOAD_SV(uint64_t, uint16_t)
SVE_GATHER_LOAD_SV(uint64_t, uint32_t)
SVE_GATHER_LOAD_SV(uint64_t, uint64_t)
SVE_GATHER_LOAD_SV(int64_t, int8_t)
SVE_GATHER_LOAD_SV(int64_t, int16_t)
SVE_GATHER_LOAD_SV(int64_t, int32_t)

SVE_SCATTER_STORE_VI(uint32_t, uint8_t)
SVE_SCATTER_STORE_VI(uint32_t, uint16_t)
SVE_SCATTER_STORE_VI(uint32_t, uint32_t)
SVE_SCATTER_STORE_VI(uint64_t, uint8_t)
SVE_SCATTER_STORE_VI(uint64_t, uint16_t)
SVE_SCATTER_STORE_VI(uint64_t, uint32_t)
SVE_SCATTER_STORE_VI(uint64_t, uint64_t)

SVE_SCATTER_STORE_SV(uint32_t, uint8_t)
SVE_SCATTER_STORE_SV(uint32_t, uint16_t)
SVE_SCATTER_STORE_SV(uint32_t, uint32_t)
SVE_SCATTER_STORE_SV(uint64_t, uint8_t)
SVE_SCATTER_STORE_SV(uint64_t, uint16_t)
SVE_SCATTER_STORE_SV(uint64_t, uint32_t)
SVE_SCATTER_STORE_SV(uint64_t, uint64_t)

SVE_GATHER_LOAD_VS(int32_t, int8_t)
SVE_GATHER_LOAD_VS(int32_t, int16_t)
SVE_GATHER_LOAD_VS(uint32_t, uint8_t)
SVE_GATHER_LOAD_VS(uint32_t, uint32_t)
SVE_GATHER_LOAD_VS(uint32_t, uint16_t)
SVE_GATHER_LOAD_VS(int64_t, int8_t)
SVE_GATHER_LOAD_VS(int64_t, int32_t)
SVE_GATHER_LOAD_VS(int64_t, int16_t)
SVE_GATHER_LOAD_VS(uint64_t, uint8_t)
SVE_GATHER_LOAD_VS(uint64_t, uint32_t)
SVE_GATHER_LOAD_VS(uint64_t, uint16_t)
SVE_GATHER_LOAD_VS(uint64_t, uint64_t)

SVE_SCATTER_STORE_VS(uint32_t, uint8_t)
SVE_SCATTER_STORE_VS(uint32_t, uint32_t)
SVE_SCATTER_STORE_VS(uint32_t, uint16_t)
SVE_SCATTER_STORE_VS(uint64_t, uint8_t)
SVE_SCATTER_STORE_VS(uint64_t, uint32_t)
SVE_SCATTER_STORE_VS(uint64_t, uint16_t)
SVE_SCATTER_STORE_VS(uint64_t, uint64_t)

template class SveIndexedMemVS<__uint128_t, __uint128_t,
                               ArmISAInst::SveGatherLoadQuadVSMicroop,
                               ArmISAInst::SveFirstFaultWritebackMicroop>;

template class SveIndexedMemVS<__uint128_t, __uint128_t,
                               ArmISAInst::SveScatterStoreQuadVSMicroop,
                               ArmISAInst::SveFirstFaultWritebackMicroop>;

} // namespace ArmISA
} // namespace gem5
