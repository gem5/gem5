/*
 * Copyright (c) 2016-2018,2019 ARM Limited
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

#include "cpu/o3/rename_map.hh"

#include <vector>

#include "cpu/reg_class.hh"
#include "debug/Rename.hh"

using namespace std;

/**** SimpleRenameMap methods ****/

SimpleRenameMap::SimpleRenameMap()
    : freeList(NULL), zeroReg(IntRegClass,0)
{
}


void
SimpleRenameMap::init(unsigned size, SimpleFreeList *_freeList,
                      RegIndex _zeroReg)
{
    assert(freeList == NULL);
    assert(map.empty());

    map.resize(size);
    freeList = _freeList;
    zeroReg = RegId(IntRegClass, _zeroReg);
}

SimpleRenameMap::RenameInfo
SimpleRenameMap::rename(const RegId& arch_reg)
{
    PhysRegIdPtr renamed_reg;
    // Record the current physical register that is renamed to the
    // requested architected register.
    PhysRegIdPtr prev_reg = map[arch_reg.flatIndex()];

    if (arch_reg == zeroReg) {
        assert(prev_reg->isZeroReg());
        renamed_reg = prev_reg;
    } else if (prev_reg->getNumPinnedWrites() > 0) {
        // Do not rename if the register is pinned
        assert(arch_reg.getNumPinnedWrites() == 0);  // Prevent pinning the
                                                     // same register twice
        DPRINTF(Rename, "Renaming pinned reg, numPinnedWrites %d\n",
                prev_reg->getNumPinnedWrites());
        renamed_reg = prev_reg;
        renamed_reg->decrNumPinnedWrites();
    } else {
        renamed_reg = freeList->getReg();
        map[arch_reg.flatIndex()] = renamed_reg;
        renamed_reg->setNumPinnedWrites(arch_reg.getNumPinnedWrites());
        renamed_reg->setNumPinnedWritesToComplete(
            arch_reg.getNumPinnedWrites() + 1);
    }

    DPRINTF(Rename, "Renamed reg %d to physical reg %d (%d) old mapping was"
            " %d (%d)\n",
            arch_reg, renamed_reg->flatIndex(), renamed_reg->flatIndex(),
            prev_reg->flatIndex(), prev_reg->flatIndex());

    return RenameInfo(renamed_reg, prev_reg);
}


/**** UnifiedRenameMap methods ****/

void
UnifiedRenameMap::init(PhysRegFile *_regFile,
                       RegIndex _intZeroReg,
                       RegIndex _floatZeroReg,
                       UnifiedFreeList *freeList,
                       VecMode _mode)
{
    regFile = _regFile;
    vecMode = _mode;

    intMap.init(TheISA::NumIntRegs, &(freeList->intList), _intZeroReg);

    floatMap.init(TheISA::NumFloatRegs, &(freeList->floatList), _floatZeroReg);

    vecMap.init(TheISA::NumVecRegs, &(freeList->vecList), (RegIndex)-1);

    vecElemMap.init(TheISA::NumVecRegs * NVecElems,
            &(freeList->vecElemList), (RegIndex)-1);

    predMap.init(TheISA::NumVecPredRegs, &(freeList->predList), (RegIndex)-1);

    ccMap.init(TheISA::NumCCRegs, &(freeList->ccList), (RegIndex)-1);

}

void
UnifiedRenameMap::switchFreeList(UnifiedFreeList* freeList)
{
    if (vecMode == Enums::Elem) {

        /* The free list should currently be tracking full registers. */
        panic_if(freeList->hasFreeVecElems(),
                "The free list is already tracking Vec elems");
        panic_if(freeList->numFreeVecRegs() !=
                regFile->numVecPhysRegs() - TheISA::NumVecRegs,
                "The free list has lost vector registers");

        /* Split the free regs. */
        while (freeList->hasFreeVecRegs()) {
            auto vr = freeList->getVecReg();
            auto range = this->regFile->getRegElemIds(vr);
            freeList->addRegs(range.first, range.second);
        }

    } else if (vecMode == Enums::Full) {

        /* The free list should currently be tracking register elems. */
        panic_if(freeList->hasFreeVecRegs(),
                "The free list is already tracking full Vec");
        panic_if(freeList->numFreeVecElems() !=
                 regFile->numVecElemPhysRegs() -
                 TheISA::NumVecRegs * TheISA::NumVecElemPerVecReg,
                 "The free list has lost vector register elements");

        auto range = regFile->getRegIds(VecRegClass);
        freeList->addRegs(range.first + TheISA::NumVecRegs, range.second);

        /* We remove the elems from the free list. */
        while (freeList->hasFreeVecElems())
            freeList->getVecElem();
    }
}

void
UnifiedRenameMap::switchMode(VecMode newVecMode)
{
    if (newVecMode == Enums::Elem && vecMode == Enums::Full) {

        /* Switch to vector element rename mode. */
        vecMode = Enums::Elem;

        /* Split the mapping of each arch reg. */
        int vec_idx = 0;
        for (auto &vec: vecMap) {
            PhysRegFile::IdRange range = this->regFile->getRegElemIds(vec);
            auto idx = 0;
            for (auto phys_elem = range.first;
                 phys_elem < range.second; idx++, phys_elem++) {

                setEntry(RegId(VecElemClass, vec_idx, idx), &(*phys_elem));
            }
            vec_idx++;
        }

    } else if (newVecMode == Enums::Full && vecMode == Enums::Elem) {

        /* Switch to full vector register rename mode. */
        vecMode = Enums::Full;

        /* To rebuild the arch regs we take the easy road:
         *  1.- Stitch the elems together into vectors.
         *  2.- Replace the contents of the register file with the vectors
         *  3.- Set the remaining registers as free
         */
        TheISA::VecRegContainer new_RF[TheISA::NumVecRegs];
        for (uint32_t i = 0; i < TheISA::NumVecRegs; i++) {
            VecReg dst = new_RF[i].as<TheISA::VecElem>();
            for (uint32_t l = 0; l < NVecElems; l++) {
                RegId s_rid(VecElemClass, i, l);
                PhysRegIdPtr s_prid = vecElemMap.lookup(s_rid);
                dst[l] = regFile->readVecElem(s_prid);
            }
        }

        for (uint32_t i = 0; i < TheISA::NumVecRegs; i++) {
            PhysRegId pregId(VecRegClass, i, 0);
            regFile->setVecReg(regFile->getTrueId(&pregId), new_RF[i]);
        }

        auto range = regFile->getRegIds(VecRegClass);
        for (uint32_t i = 0; i < TheISA::NumVecRegs; i++) {
            setEntry(RegId(VecRegClass, i), &(*(range.first + i)));
        }

    }
}
