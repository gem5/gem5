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

#include <vector>

#include "cpu/o3/rename_map.hh"
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
    PhysRegIndex renamed_reg;

    // Record the current physical register that is renamed to the
    // requested architected register.
    PhysRegIndex prev_reg = map[arch_reg];

    // If it's not referencing the zero register, then rename the
    // register.
    if (arch_reg != zeroReg) {
        renamed_reg = freeList->getReg();

        map[arch_reg] = renamed_reg;
    } else {
        // Otherwise return the zero register so nothing bad happens.
        assert(prev_reg == zeroReg);
        renamed_reg = zeroReg;
    }

    DPRINTF(Rename, "Renamed reg %d to physical reg %d old mapping was %d\n",
            arch_reg, renamed_reg, prev_reg);

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
UnifiedRenameMap::rename(RegIndex arch_reg)
{
    RegIndex rel_arch_reg;

    switch (regIdxToClass(arch_reg, &rel_arch_reg)) {
      case IntRegClass:
        return renameInt(rel_arch_reg);

      case FloatRegClass:
        return renameFloat(rel_arch_reg);

      case CCRegClass:
        return renameCC(rel_arch_reg);

      case MiscRegClass:
        return renameMisc(rel_arch_reg);

      default:
        panic("rename rename(): unknown reg class %s\n",
              RegClassStrings[regIdxToClass(arch_reg)]);
    }
}


PhysRegIndex
UnifiedRenameMap::lookup(RegIndex arch_reg) const
{
    RegIndex rel_arch_reg;

    switch (regIdxToClass(arch_reg, &rel_arch_reg)) {
      case IntRegClass:
        return lookupInt(rel_arch_reg);

      case FloatRegClass:
        return lookupFloat(rel_arch_reg);

      case CCRegClass:
        return lookupCC(rel_arch_reg);

      case MiscRegClass:
        return lookupMisc(rel_arch_reg);

      default:
        panic("rename lookup(): unknown reg class %s\n",
              RegClassStrings[regIdxToClass(arch_reg)]);
    }
}

void
UnifiedRenameMap::setEntry(RegIndex arch_reg, PhysRegIndex phys_reg)
{
    RegIndex rel_arch_reg;

    switch (regIdxToClass(arch_reg, &rel_arch_reg)) {
      case IntRegClass:
        return setIntEntry(rel_arch_reg, phys_reg);

      case FloatRegClass:
        return setFloatEntry(rel_arch_reg, phys_reg);

      case CCRegClass:
        return setCCEntry(rel_arch_reg, phys_reg);

      case MiscRegClass:
        // Misc registers do not actually rename, so don't change
        // their mappings.  We end up here when a commit or squash
        // tries to update or undo a hardwired misc reg nmapping,
        // which should always be setting it to what it already is.
        assert(phys_reg == lookupMisc(rel_arch_reg));
        return;

      default:
        panic("rename setEntry(): unknown reg class %s\n",
              RegClassStrings[regIdxToClass(arch_reg)]);
    }
}
