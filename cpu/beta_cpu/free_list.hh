#ifndef __CPU_BETA_CPU_FREE_LIST_HH__
#define __CPU_BETA_CPU_FREE_LIST_HH__

#include <iostream>
#include <queue>

#include "arch/alpha/isa_traits.hh"
#include "base/trace.hh"
#include "base/traceflags.hh"
#include "cpu/beta_cpu/comm.hh"

/**
 * FreeList class that simply holds the list of free integer and floating
 * point registers.  Can request for a free register of either type, and
 * also send back free registers of either type.  This is a very simple
 * class, but it should be sufficient for most implementations.  Like all
 * other classes, it assumes that the indices for the floating point
 * registers starts after the integer registers end.  Hence the variable
 * numPhysicalIntRegs is logically equivalent to the baseFP dependency.
 * Note that
 * while this most likely should be called FreeList, the name "FreeList"
 * is used in a typedef within the CPU Policy, and therefore no class
 * can be named simply "FreeList".
 * @todo: Give a better name to the base FP dependency.
 */
class SimpleFreeList
{
  private:
    /** The list of free integer registers. */
    std::queue<PhysRegIndex> freeIntRegs;

    /** The list of free floating point registers. */
    std::queue<PhysRegIndex> freeFloatRegs;

    /** Number of logical integer registers. */
    int numLogicalIntRegs;

    /** Number of physical integer registers. */
    int numPhysicalIntRegs;

    /** Number of logical floating point registers. */
    int numLogicalFloatRegs;

    /** Number of physical floating point registers. */
    int numPhysicalFloatRegs;

    /** Total number of physical registers. */
    int numPhysicalRegs;

    /** DEBUG stuff below. */
    std::vector<int> freeIntRegsScoreboard;

    std::vector<bool> freeFloatRegsScoreboard;

  public:
    SimpleFreeList(unsigned _numLogicalIntRegs,
                   unsigned _numPhysicalIntRegs,
                   unsigned _numLogicalFloatRegs,
                   unsigned _numPhysicalFloatRegs);

    inline PhysRegIndex getIntReg();

    inline PhysRegIndex getFloatReg();

    inline void addReg(PhysRegIndex freed_reg);

    inline void addIntReg(PhysRegIndex freed_reg);

    inline void addFloatReg(PhysRegIndex freed_reg);

    bool hasFreeIntRegs()
    { return !freeIntRegs.empty(); }

    bool hasFreeFloatRegs()
    { return !freeFloatRegs.empty(); }

    int numFreeIntRegs()
    { return freeIntRegs.size(); }

    int numFreeFloatRegs()
    { return freeFloatRegs.size(); }
};

inline PhysRegIndex
SimpleFreeList::getIntReg()
{
    DPRINTF(Rename, "FreeList: Trying to get free integer register.\n");
    if (freeIntRegs.empty()) {
        panic("No free integer registers!");
    }

    PhysRegIndex free_reg = freeIntRegs.front();

    freeIntRegs.pop();

    // DEBUG
    assert(freeIntRegsScoreboard[free_reg]);
    freeIntRegsScoreboard[free_reg] = 0;

    return(free_reg);
}

inline PhysRegIndex
SimpleFreeList::getFloatReg()
{
    DPRINTF(Rename, "FreeList: Trying to get free float register.\n");
    if (freeFloatRegs.empty()) {
        panic("No free integer registers!");
    }

    PhysRegIndex free_reg = freeFloatRegs.front();

    freeFloatRegs.pop();

    // DEBUG
    assert(freeFloatRegsScoreboard[free_reg]);
    freeFloatRegsScoreboard[free_reg] = 0;

    return(free_reg);
}

inline void
SimpleFreeList::addReg(PhysRegIndex freed_reg)
{
    DPRINTF(Rename, "Freelist: Freeing register %i.\n", freed_reg);
    //Might want to add in a check for whether or not this register is
    //already in there.  A bit vector or something similar would be useful.
    if (freed_reg < numPhysicalIntRegs) {
        freeIntRegs.push(freed_reg);

        // DEBUG
        assert(freeIntRegsScoreboard[freed_reg] == false);
        freeIntRegsScoreboard[freed_reg] = 1;
    } else if (freed_reg < numPhysicalRegs) {
        freeFloatRegs.push(freed_reg);

        // DEBUG
        assert(freeFloatRegsScoreboard[freed_reg] == false);
        freeFloatRegsScoreboard[freed_reg] = 1;
    }
}

inline void
SimpleFreeList::addIntReg(PhysRegIndex freed_reg)
{
    DPRINTF(Rename, "Freelist: Freeing int register %i.\n", freed_reg);

    // DEBUG
    assert(!freeIntRegsScoreboard[freed_reg]);
    freeIntRegsScoreboard[freed_reg] = 1;

    freeIntRegs.push(freed_reg);
}

inline void
SimpleFreeList::addFloatReg(PhysRegIndex freed_reg)
{
    DPRINTF(Rename, "Freelist: Freeing float register %i.\n", freed_reg);

    // DEBUG
    assert(!freeFloatRegsScoreboard[freed_reg]);
    freeFloatRegsScoreboard[freed_reg] = 1;

    freeFloatRegs.push(freed_reg);
}

#endif // __CPU_BETA_CPU_FREE_LIST_HH__
