/*
 * Copyright (c) 2016-2018 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder. You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
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

#include "cpu/o3/regfile.hh"

#include "cpu/o3/free_list.hh"

namespace gem5
{

namespace o3
{

PhysRegFile::PhysRegFile(unsigned _numPhysicalIntRegs,
                         unsigned _numPhysicalFloatRegs,
                         unsigned _numPhysicalVecRegs,
                         unsigned _numPhysicalVecPredRegs,
                         unsigned _numPhysicalCCRegs,
                         const BaseISA::RegClasses &reg_classes)
    : intRegFile(*reg_classes.at(IntRegClass), _numPhysicalIntRegs),
      floatRegFile(*reg_classes.at(FloatRegClass), _numPhysicalFloatRegs),
      vectorRegFile(*reg_classes.at(VecRegClass), _numPhysicalVecRegs),
      vectorElemRegFile(*reg_classes.at(VecElemClass), _numPhysicalVecRegs * (
                  reg_classes.at(VecElemClass)->numRegs() /
                  reg_classes.at(VecRegClass)->numRegs())),
      vecPredRegFile(*reg_classes.at(VecPredRegClass),
              _numPhysicalVecPredRegs),
      ccRegFile(*reg_classes.at(CCRegClass), _numPhysicalCCRegs),
      numPhysicalIntRegs(_numPhysicalIntRegs),
      numPhysicalFloatRegs(_numPhysicalFloatRegs),
      numPhysicalVecRegs(_numPhysicalVecRegs),
      numPhysicalVecElemRegs(_numPhysicalVecRegs * (
                  reg_classes.at(VecElemClass)->numRegs() /
                  reg_classes.at(VecRegClass)->numRegs())),
      numPhysicalVecPredRegs(_numPhysicalVecPredRegs),
      numPhysicalCCRegs(_numPhysicalCCRegs),
      totalNumRegs(_numPhysicalIntRegs
                   + _numPhysicalFloatRegs
                   + _numPhysicalVecRegs
                   + numPhysicalVecElemRegs
                   + _numPhysicalVecPredRegs
                   + _numPhysicalCCRegs)
{
    RegIndex phys_reg;
    RegIndex flat_reg_idx = 0;

    // The initial batch of registers are the integer ones
    for (phys_reg = 0; phys_reg < numPhysicalIntRegs; phys_reg++) {
        intRegIds.emplace_back(*reg_classes.at(IntRegClass),
                phys_reg, flat_reg_idx++);
    }

    // The next batch of the registers are the floating-point physical
    // registers; put them onto the floating-point free list.
    for (phys_reg = 0; phys_reg < numPhysicalFloatRegs; phys_reg++) {
        floatRegIds.emplace_back(*reg_classes.at(FloatRegClass),
                phys_reg, flat_reg_idx++);
    }

    // The next batch of the registers are the vector physical
    // registers; put them onto the vector free list.
    for (phys_reg = 0; phys_reg < numPhysicalVecRegs; phys_reg++) {
        vecRegIds.emplace_back(*reg_classes.at(VecRegClass), phys_reg,
                flat_reg_idx++);
    }
    // The next batch of the registers are the vector element physical
    // registers; put them onto the vector free list.
    for (phys_reg = 0; phys_reg < numPhysicalVecElemRegs; phys_reg++) {
        vecElemIds.emplace_back(*reg_classes.at(VecElemClass), phys_reg,
                flat_reg_idx++);
    }

    // The next batch of the registers are the predicate physical
    // registers; put them onto the predicate free list.
    for (phys_reg = 0; phys_reg < numPhysicalVecPredRegs; phys_reg++) {
        vecPredRegIds.emplace_back(*reg_classes.at(VecPredRegClass), phys_reg,
                flat_reg_idx++);
    }

    // The rest of the registers are the condition-code physical
    // registers; put them onto the condition-code free list.
    for (phys_reg = 0; phys_reg < numPhysicalCCRegs; phys_reg++) {
        ccRegIds.emplace_back(*reg_classes.at(CCRegClass), phys_reg,
                flat_reg_idx++);
    }

    // Misc regs have a fixed mapping but still need PhysRegIds.
    for (phys_reg = 0; phys_reg < reg_classes.at(MiscRegClass)->numRegs();
            phys_reg++) {
        miscRegIds.emplace_back(*reg_classes.at(MiscRegClass), phys_reg, 0);
    }
}


void
PhysRegFile::initFreeList(UnifiedFreeList *freeList)
{
    // Initialize the free lists.
    int reg_idx = 0;

    // The initial batch of registers are the integer ones
    for (reg_idx = 0; reg_idx < numPhysicalIntRegs; reg_idx++) {
        assert(intRegIds[reg_idx].index() == reg_idx);
    }
    freeList->addRegs(intRegIds.begin(), intRegIds.end());

    // The next batch of the registers are the floating-point physical
    // registers; put them onto the floating-point free list.
    for (reg_idx = 0; reg_idx < numPhysicalFloatRegs; reg_idx++) {
        assert(floatRegIds[reg_idx].index() == reg_idx);
    }
    freeList->addRegs(floatRegIds.begin(), floatRegIds.end());

    /* The next batch of the registers are the vector physical
     * registers; put them onto the vector free list. */
    for (reg_idx = 0; reg_idx < numPhysicalVecRegs; reg_idx++) {
        assert(vecRegIds[reg_idx].index() == reg_idx);
    }
    freeList->addRegs(vecRegIds.begin(), vecRegIds.end());
    for (reg_idx = 0; reg_idx < numPhysicalVecElemRegs; reg_idx++) {
        assert(vecElemIds[reg_idx].index() == reg_idx);
    }
    freeList->addRegs(vecElemIds.begin(), vecElemIds.end());

    // The next batch of the registers are the predicate physical
    // registers; put them onto the predicate free list.
    for (reg_idx = 0; reg_idx < numPhysicalVecPredRegs; reg_idx++) {
        assert(vecPredRegIds[reg_idx].index() == reg_idx);
    }
    freeList->addRegs(vecPredRegIds.begin(), vecPredRegIds.end());

    // The rest of the registers are the condition-code physical
    // registers; put them onto the condition-code free list.
    for (reg_idx = 0; reg_idx < numPhysicalCCRegs; reg_idx++) {
        assert(ccRegIds[reg_idx].index() == reg_idx);
    }
    freeList->addRegs(ccRegIds.begin(), ccRegIds.end());
}

PhysRegFile::IdRange
PhysRegFile::getRegIds(RegClassType cls)
{
    switch (cls)
    {
      case IntRegClass:
        return std::make_pair(intRegIds.begin(), intRegIds.end());
      case FloatRegClass:
        return std::make_pair(floatRegIds.begin(), floatRegIds.end());
      case VecRegClass:
        return std::make_pair(vecRegIds.begin(), vecRegIds.end());
      case VecElemClass:
        return std::make_pair(vecElemIds.begin(), vecElemIds.end());
      case VecPredRegClass:
        return std::make_pair(vecPredRegIds.begin(), vecPredRegIds.end());
      case CCRegClass:
        return std::make_pair(ccRegIds.begin(), ccRegIds.end());
      case MiscRegClass:
        return std::make_pair(miscRegIds.begin(), miscRegIds.end());
      case InvalidRegClass:
        panic("Tried to get register IDs for the invalid class.");
    }
    /* There is no way to make an empty iterator */
    return std::make_pair(PhysIds::iterator(),
                          PhysIds::iterator());
}

PhysRegIdPtr
PhysRegFile::getTrueId(PhysRegIdPtr reg)
{
    switch (reg->classValue()) {
    case VecRegClass:
        return &vecRegIds[reg->index()];
    case VecElemClass:
        return &vecElemIds[reg->index()];
    default:
        panic_if(!reg->is(VecElemClass),
            "Trying to get the register of a %s register", reg->className());
    }
    return nullptr;
}

} // namespace o3
} // namespace gem5
