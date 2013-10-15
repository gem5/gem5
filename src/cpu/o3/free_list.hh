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

#ifndef __CPU_O3_FREE_LIST_HH__
#define __CPU_O3_FREE_LIST_HH__

#include <iostream>
#include <queue>

#include "base/misc.hh"
#include "base/trace.hh"
#include "cpu/o3/comm.hh"
#include "cpu/o3/regfile.hh"
#include "debug/FreeList.hh"

/**
 * Free list for a single class of registers (e.g., integer
 * or floating point).  Because the register class is implicitly
 * determined by the rename map instance being accessed, all
 * architectural register index parameters and values in this class
 * are relative (e.g., %fp2 is just index 2).
 */
class SimpleFreeList
{
  private:

    /** The actual free list */
    std::queue<PhysRegIndex> freeRegs;

  public:

    SimpleFreeList() {};

    /** Add a physical register to the free list */
    void addReg(PhysRegIndex reg) { freeRegs.push(reg); }

    /** Get the next available register from the free list */
    PhysRegIndex getReg()
    {
        assert(!freeRegs.empty());
        PhysRegIndex free_reg = freeRegs.front();
        freeRegs.pop();
        return free_reg;
    }

    /** Return the number of free registers on the list. */
    unsigned numFreeRegs() const { return freeRegs.size(); }

    /** True iff there are free registers on the list. */
    bool hasFreeRegs() const { return !freeRegs.empty(); }
};


/**
 * FreeList class that simply holds the list of free integer and floating
 * point registers.  Can request for a free register of either type, and
 * also send back free registers of either type.  This is a very simple
 * class, but it should be sufficient for most implementations.  Like all
 * other classes, it assumes that the indices for the floating point
 * registers starts after the integer registers end.  Hence the variable
 * numPhysicalIntRegs is logically equivalent to the baseFP dependency.
 * Note that while this most likely should be called FreeList, the name
 * "FreeList" is used in a typedef within the CPU Policy, and therefore no
 * class can be named simply "FreeList".
 * @todo: Give a better name to the base FP dependency.
 */
class UnifiedFreeList
{
  private:

    /** The object name, for DPRINTF.  We have to declare this
     *  explicitly because Scoreboard is not a SimObject. */
    const std::string _name;

    /** The list of free integer registers. */
    SimpleFreeList intList;

    /** The list of free floating point registers. */
    SimpleFreeList floatList;

    /** The list of free condition-code registers. */
    SimpleFreeList ccList;

    /**
     * The register file object is used only to distinguish integer
     * from floating-point physical register indices.
     */
    PhysRegFile *regFile;

    /*
     * We give UnifiedRenameMap internal access so it can get at the
     * internal per-class free lists and associate those with its
     * per-class rename maps. See UnifiedRenameMap::init().
     */
    friend class UnifiedRenameMap;

  public:
    /** Constructs a free list.
     *  @param _numPhysicalIntRegs Number of physical integer registers.
     *  @param reservedIntRegs Number of integer registers already
     *                         used by initial mappings.
     *  @param _numPhysicalFloatRegs Number of physical fp registers.
     *  @param reservedFloatRegs Number of fp registers already
     *                           used by initial mappings.
     */
    UnifiedFreeList(const std::string &_my_name, PhysRegFile *_regFile);

    /** Gives the name of the freelist. */
    std::string name() const { return _name; };

    /** Returns a pointer to the condition-code free list */
    SimpleFreeList *getCCList() { return &ccList; }

    /** Gets a free integer register. */
    PhysRegIndex getIntReg() { return intList.getReg(); }

    /** Gets a free fp register. */
    PhysRegIndex getFloatReg() { return floatList.getReg(); }

    /** Gets a free cc register. */
    PhysRegIndex getCCReg() { return ccList.getReg(); }

    /** Adds a register back to the free list. */
    void addReg(PhysRegIndex freed_reg);

    /** Adds an integer register back to the free list. */
    void addIntReg(PhysRegIndex freed_reg) { intList.addReg(freed_reg); }

    /** Adds a fp register back to the free list. */
    void addFloatReg(PhysRegIndex freed_reg) { floatList.addReg(freed_reg); }

    /** Adds a cc register back to the free list. */
    void addCCReg(PhysRegIndex freed_reg) { ccList.addReg(freed_reg); }

    /** Checks if there are any free integer registers. */
    bool hasFreeIntRegs() const { return intList.hasFreeRegs(); }

    /** Checks if there are any free fp registers. */
    bool hasFreeFloatRegs() const { return floatList.hasFreeRegs(); }

    /** Checks if there are any free cc registers. */
    bool hasFreeCCRegs() const { return ccList.hasFreeRegs(); }

    /** Returns the number of free integer registers. */
    unsigned numFreeIntRegs() const { return intList.numFreeRegs(); }

    /** Returns the number of free fp registers. */
    unsigned numFreeFloatRegs() const { return floatList.numFreeRegs(); }

    /** Returns the number of free cc registers. */
    unsigned numFreeCCRegs() const { return ccList.numFreeRegs(); }
};

inline void
UnifiedFreeList::addReg(PhysRegIndex freed_reg)
{
    DPRINTF(FreeList,"Freeing register %i.\n", freed_reg);
    //Might want to add in a check for whether or not this register is
    //already in there.  A bit vector or something similar would be useful.
    if (regFile->isIntPhysReg(freed_reg)) {
        intList.addReg(freed_reg);
    } else if (regFile->isFloatPhysReg(freed_reg)) {
        floatList.addReg(freed_reg);
    } else {
        assert(regFile->isCCPhysReg(freed_reg));
        ccList.addReg(freed_reg);
    }

    // These assert conditions ensure that the number of free
    // registers are not more than the # of total Physical  Registers.
    // If this were false, it would mean that registers
    // have been freed twice, overflowing the free register
    // pool and potentially crashing SMT workloads.
    // ----
    // Comment out for now so as to not potentially break
    // CMP and single-threaded workloads
    // ----
    // assert(freeIntRegs.size() <= numPhysicalIntRegs);
    // assert(freeFloatRegs.size() <= numPhysicalFloatRegs);
}


#endif // __CPU_O3_FREE_LIST_HH__
