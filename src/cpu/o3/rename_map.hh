/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

// Todo:  Create destructor.
// Have it so that there's a more meaningful name given to the variable
// that marks the beginning of the FP registers.

#ifndef __CPU_O3_RENAME_MAP_HH__
#define __CPU_O3_RENAME_MAP_HH__

#include <iostream>
#include <utility>
#include <vector>

#include "arch/types.hh"
#include "config/the_isa.hh"
#include "cpu/o3/free_list.hh"

class SimpleRenameMap
{
  protected:
    typedef TheISA::RegIndex RegIndex;
  public:
    /**
     * Pair of a logical register and a physical register.  Tells the
     * previous mapping of a logical register to a physical register.
     * Used to roll back the rename map to a previous state.
     */
    typedef std::pair<RegIndex, PhysRegIndex> UnmapInfo;

    /**
     * Pair of a physical register and a physical register.  Used to
     * return the physical register that a logical register has been
     * renamed to, and the previous physical register that the same
     * logical register was previously mapped to.
     */
    typedef std::pair<PhysRegIndex, PhysRegIndex> RenameInfo;

  public:
    /** Default constructor.  init() must be called prior to use. */
    SimpleRenameMap() {};

    /** Destructor. */
    ~SimpleRenameMap();

    /** Initializes rename map with given parameters. */
    void init(unsigned _numLogicalIntRegs,
              unsigned _numPhysicalIntRegs,
              PhysRegIndex &_int_reg_start,

              unsigned _numLogicalFloatRegs,
              unsigned _numPhysicalFloatRegs,
              PhysRegIndex &_float_reg_start,

              unsigned _numMiscRegs,

              RegIndex _intZeroReg,
              RegIndex _floatZeroReg,

              int id,
              bool bindRegs);

    /** Sets the free list used with this rename map. */
    void setFreeList(SimpleFreeList *fl_ptr);

    //Tell rename map to get a free physical register for a given
    //architected register.  Not sure it should have a return value,
    //but perhaps it should have some sort of fault in case there are
    //no free registers.
    RenameInfo rename(RegIndex arch_reg);

    PhysRegIndex lookup(RegIndex phys_reg);

    /**
     * Marks the given register as ready, meaning that its value has been
     * calculated and written to the register file.
     * @param ready_reg The index of the physical register that is now ready.
     */
    void setEntry(RegIndex arch_reg, PhysRegIndex renamed_reg);

    int numFreeEntries();

  private:
    /** Rename Map ID  */
    int id;

    /** Number of logical integer registers. */
    int numLogicalIntRegs;

    /** Number of physical integer registers. */
    int numPhysicalIntRegs;

    /** Number of logical floating point registers. */
    int numLogicalFloatRegs;

    /** Number of physical floating point registers. */
    int numPhysicalFloatRegs;

    /** Number of miscellaneous registers. */
    int numMiscRegs;

    /** Number of logical integer + float registers. */
    int numLogicalRegs;

    /** Number of physical integer + float registers. */
    int numPhysicalRegs;

    /** The integer zero register.  This implementation assumes it is always
     *  zero and never can be anything else.
     */
    RegIndex intZeroReg;

    /** The floating point zero register.  This implementation assumes it is
     *  always zero and never can be anything else.
     */
    RegIndex floatZeroReg;

    class RenameEntry
    {
      public:
        PhysRegIndex physical_reg;
        bool valid;

        RenameEntry()
            : physical_reg(0), valid(false)
        { }
    };

  private:
    /** Integer rename map. */
    std::vector<RenameEntry> intRenameMap;

    /** Floating point rename map. */
    std::vector<RenameEntry> floatRenameMap;

  private:
    /** Free list interface. */
    SimpleFreeList *freeList;
};

#endif //__CPU_O3_RENAME_MAP_HH__
