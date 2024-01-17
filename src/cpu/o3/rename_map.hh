/*
 * Copyright (c) 2015-2017 ARM Limited
 * All rights reserved.
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

#ifndef __CPU_O3_RENAME_MAP_HH__
#define __CPU_O3_RENAME_MAP_HH__

#include <algorithm>
#include <array>
#include <iostream>
#include <limits>
#include <utility>
#include <vector>

#include "arch/generic/isa.hh"
#include "cpu/o3/dyn_inst_ptr.hh"
#include "cpu/o3/free_list.hh"
#include "cpu/o3/regfile.hh"
#include "cpu/reg_class.hh"

namespace gem5
{

namespace o3
{

/**
 * Register rename map for a single class of registers (e.g., integer
 * or floating point).  Because the register class is implicitly
 * determined by the rename map instance being accessed, all
 * architectural register index parameters and values in this class
 * are relative (e.g., %fp2 is just index 2).
 */
class SimpleRenameMap
{
  private:
    using Arch2PhysMap = std::vector<PhysRegIdPtr>;
    /** The acutal arch-to-phys register map */
    Arch2PhysMap map;
  public:
    using iterator = Arch2PhysMap::iterator;
    using const_iterator = Arch2PhysMap::const_iterator;
  private:

    /**
     * Pointer to the free list from which new physical registers
     * should be allocated in rename()
     */
    SimpleFreeList *freeList;

  public:

    SimpleRenameMap();

    /**
     * Because we have an array of rename maps (one per thread) in the CPU,
     * it's awkward to initialize this object via the constructor.
     * Instead, this method is used for initialization.
     */
    void init(const RegClass &reg_class, SimpleFreeList *_freeList);

    /**
     * Pair of a physical register and a physical register.  Used to
     * return the physical register that a logical register has been
     * renamed to, and the previous physical register that the same
     * logical register was previously mapped to.
     */
    typedef std::pair<PhysRegIdPtr, PhysRegIdPtr> RenameInfo;

    /**
     * Tell rename map to get a new free physical register to remap
     * the specified architectural register.
     * @param arch_reg The architectural register to remap.
     * @return A RenameInfo pair indicating both the new and previous
     * physical registers.
     */
    RenameInfo rename(const RegId& arch_reg);

    /**
     * Look up the physical register mapped to an architectural register.
     * @param arch_reg The architectural register to look up.
     * @return The physical register it is currently mapped to.
     */
    PhysRegIdPtr
    lookup(const RegId& arch_reg) const
    {
        assert(arch_reg.index() <= map.size());
        return map[arch_reg.index()];
    }

    /**
     * Update rename map with a specific mapping.  Generally used to
     * roll back to old mappings on a squash.
     * @param arch_reg The architectural register to remap.
     * @param phys_reg The physical register to remap it to.
     */
    void
    setEntry(const RegId& arch_reg, PhysRegIdPtr phys_reg)
    {
        assert(arch_reg.index() <= map.size());
        map[arch_reg.index()] = phys_reg;
    }

    /** Return the number of free entries on the associated free list. */
    unsigned numFreeEntries() const { return freeList->numFreeRegs(); }

    size_t numArchRegs() const { return map.size(); }

    /** Forward begin/cbegin to the map. */
    /** @{ */
    iterator begin() { return map.begin(); }
    const_iterator begin() const { return map.begin(); }
    const_iterator cbegin() const { return map.cbegin(); }
    /** @} */

    /** Forward end/cend to the map. */
    /** @{ */
    iterator end() { return map.end(); }
    const_iterator end() const { return map.end(); }
    const_iterator cend() const { return map.cend(); }
    /** @} */
};

/**
 * Unified register rename map for all classes of registers.  Wraps a
 * set of class-specific rename maps.  Methods that do not specify a
 * register class (e.g., rename()) take register ids,
 * while methods that do specify a register class (e.g., renameInt())
 * take register indices.
 */
class UnifiedRenameMap
{
  private:
    std::array<SimpleRenameMap, CCRegClass + 1> renameMaps;

    static inline PhysRegId invalidPhysRegId{};

    /**
     * The register file object is used only to get PhysRegIdPtr
     * on MiscRegs, as they are stored in it.
     */
    PhysRegFile *regFile;

  public:

    typedef SimpleRenameMap::RenameInfo RenameInfo;

    /** Default constructor.  init() must be called prior to use. */
    UnifiedRenameMap() : regFile(nullptr) {};

    /** Destructor. */
    ~UnifiedRenameMap() {};

    /** Initializes rename map with given parameters. */
    void init(const BaseISA::RegClasses &regClasses,
              PhysRegFile *_regFile, UnifiedFreeList *freeList);

    /**
     * Tell rename map to get a new free physical register to remap
     * the specified architectural register. This version takes a
     * RegId and reads the  appropriate class-specific rename table.
     * @param arch_reg The architectural register id to remap.
     * @return A RenameInfo pair indicating both the new and previous
     * physical registers.
     */
    RenameInfo
    rename(const RegId& arch_reg)
    {
        if (!arch_reg.isRenameable()) {
            // misc regs aren't really renamed, just remapped
            PhysRegIdPtr phys_reg = lookup(arch_reg);
            // Set the new register to the previous one to keep the same
            // mapping throughout the execution.
            return RenameInfo(phys_reg, phys_reg);
        }

        return renameMaps[arch_reg.classValue()].rename(arch_reg);
    }

    /**
     * Look up the physical register mapped to an architectural register.
     * This version takes a flattened architectural register id
     * and calls the appropriate class-specific rename table.
     * @param arch_reg The architectural register to look up.
     * @return The physical register it is currently mapped to.
     */
    PhysRegIdPtr
    lookup(const RegId& arch_reg) const
    {
        auto reg_class = arch_reg.classValue();
        if (reg_class == InvalidRegClass) {
            return &invalidPhysRegId;
        } else if (reg_class == MiscRegClass) {
            // misc regs aren't really renamed, they keep the same
            // mapping throughout the execution.
            return regFile->getMiscRegId(arch_reg.index());
        }
        return renameMaps[reg_class].lookup(arch_reg);
    }

    /**
     * Update rename map with a specific mapping.  Generally used to
     * roll back to old mappings on a squash.  This version takes a
     * flattened architectural register id and calls the
     * appropriate class-specific rename table.
     * @param arch_reg The architectural register to remap.
     * @param phys_reg The physical register to remap it to.
     */
    void
    setEntry(const RegId& arch_reg, PhysRegIdPtr phys_reg)
    {
        assert(phys_reg->is(arch_reg.classValue()));
        if (!arch_reg.isRenameable()) {
            // Misc registers do not actually rename, so don't change
            // their mappings.  We end up here when a commit or squash
            // tries to update or undo a hardwired misc reg nmapping,
            // which should always be setting it to what it already is.
            assert(phys_reg == lookup(arch_reg));
            return;
        }

        return renameMaps[arch_reg.classValue()].setEntry(arch_reg, phys_reg);
    }

    /**
     * Return the minimum number of free entries across all of the
     * register classes.  The minimum is used so we guarantee that
     * this number of entries is available regardless of which class
     * of registers is requested.
     */
    unsigned
    minFreeEntries() const
    {
        auto min_free = std::numeric_limits<unsigned>::max();
        for (auto &map: renameMaps) {
            // If this map isn't empty (not used)...
            if (map.numArchRegs())
                min_free = std::min(min_free, map.numFreeEntries());
        }
        return min_free;
    }

    unsigned
    numFreeEntries(RegClassType type) const
    {
        return renameMaps[type].numFreeEntries();
    }

    /**
     * Return whether there are enough registers to serve the request.
     */
    bool canRename(DynInstPtr inst) const;
};

} // namespace o3
} // namespace gem5

#endif //__CPU_O3_RENAME_MAP_HH__
