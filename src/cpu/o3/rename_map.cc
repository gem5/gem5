/*
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
 *
 * Authors: Kevin Lim
 */

#include "cpu/o3/rename_map.hh"

#include <vector>

#include "debug/Rename.hh"

using namespace std;

/**** SimpleRenameMap methods ****/

SimpleRenameMap::SimpleRenameMap()
    : freeList(NULL), zeroReg(0)
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
    zeroReg = _zeroReg;
}

SimpleRenameMap::RenameInfo
SimpleRenameMap::rename(RegIndex arch_reg)
{
    PhysRegIdPtr renamed_reg;

    // Record the current physical register that is renamed to the
    // requested architected register.
    PhysRegIdPtr prev_reg = map[arch_reg];

    // If it's not referencing the zero register, then rename the
    // register.
    if (arch_reg != zeroReg) {
        renamed_reg = freeList->getReg();

        map[arch_reg] = renamed_reg;
    } else {
        // Otherwise return the zero register so nothing bad happens.
        assert(prev_reg->isZeroReg());
        renamed_reg = prev_reg;
    }

    DPRINTF(Rename, "Renamed reg %d to physical reg %d (%d) old mapping was"
            " %d (%d)\n",
            arch_reg, renamed_reg->regIdx, renamed_reg->flatIdx,
            prev_reg->regIdx, prev_reg->flatIdx);

    return RenameInfo(renamed_reg, prev_reg);
}


/**** UnifiedRenameMap methods ****/

void
UnifiedRenameMap::init(PhysRegFile *_regFile,
                       RegIndex _intZeroReg,
                       RegIndex _floatZeroReg,
                       UnifiedFreeList *freeList)
{
    regFile = _regFile;

    intMap.init(TheISA::NumIntRegs, &(freeList->intList), _intZeroReg);

    floatMap.init(TheISA::NumFloatRegs, &(freeList->floatList), _floatZeroReg);

    ccMap.init(TheISA::NumCCRegs, &(freeList->ccList), (RegIndex)-1);

}


UnifiedRenameMap::RenameInfo
UnifiedRenameMap::rename(RegId arch_reg)
{
    switch (arch_reg.regClass) {
      case IntRegClass:
        return renameInt(arch_reg.regIdx);

      case FloatRegClass:
        return renameFloat(arch_reg.regIdx);

      case CCRegClass:
        return renameCC(arch_reg.regIdx);

      case MiscRegClass:
        return renameMisc(arch_reg.regIdx);

      default:
        panic("rename rename(): unknown reg class %s\n",
              RegClassStrings[arch_reg.regClass]);
    }
}


PhysRegIdPtr
UnifiedRenameMap::lookup(RegId arch_reg) const
{
    switch (arch_reg.regClass) {
      case IntRegClass:
        return lookupInt(arch_reg.regIdx);

      case FloatRegClass:
        return lookupFloat(arch_reg.regIdx);

      case CCRegClass:
        return lookupCC(arch_reg.regIdx);

      case MiscRegClass:
        return lookupMisc(arch_reg.regIdx);

      default:
        panic("rename lookup(): unknown reg class %s\n",
              RegClassStrings[arch_reg.regClass]);
    }
}

void
UnifiedRenameMap::setEntry(RegId arch_reg, PhysRegIdPtr phys_reg)
{
    switch (arch_reg.regClass) {
      case IntRegClass:
        return setIntEntry(arch_reg.regIdx, phys_reg);

      case FloatRegClass:
        return setFloatEntry(arch_reg.regIdx, phys_reg);

      case CCRegClass:
        return setCCEntry(arch_reg.regIdx, phys_reg);

      case MiscRegClass:
        // Misc registers do not actually rename, so don't change
        // their mappings.  We end up here when a commit or squash
        // tries to update or undo a hardwired misc reg nmapping,
        // which should always be setting it to what it already is.
        assert(phys_reg == lookupMisc(arch_reg.regIdx));
        return;

      default:
        panic("rename setEntry(): unknown reg class %s\n",
              RegClassStrings[arch_reg.regClass]);
    }
}
