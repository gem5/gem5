// Todo:  Create destructor.
// Have it so that there's a more meaningful name given to the variable
// that marks the beginning of the FP registers.

#ifndef __RENAME_MAP_HH__
#define __RENAME_MAP_HH__

#include <iostream>
#include <vector>
#include <utility>

#include "cpu/beta_cpu/free_list.hh"

using namespace std;

class SimpleRenameMap
{
  public:
    /**
     * Pair of a logical register and a physical register.  Tells the
     * previous mapping of a logical register to a physical register.
     * Used to roll back the rename map to a previous state.
     */
    typedef pair<RegIndex, PhysRegIndex> UnmapInfo;

    /**
     * Pair of a physical register and a physical register.  Used to
     * return the physical register that a logical register has been
     * renamed to, and the previous physical register that the same
     * logical register was previously mapped to.
     */
    typedef pair<PhysRegIndex, PhysRegIndex> RenameInfo;

  public:
    //Constructor
    SimpleRenameMap(unsigned _numLogicalIntRegs,
                    unsigned _numPhysicalIntRegs,
                    unsigned _numLogicalFloatRegs,
                    unsigned _numPhysicalFloatRegs,
                    unsigned _numMiscRegs,
                    RegIndex _intZeroReg,
                    RegIndex _floatZeroReg);

    /** Destructor. */
    ~SimpleRenameMap();

    void setFreeList(SimpleFreeList *fl_ptr);

    //Tell rename map to get a free physical register for a given
    //architected register.  Not sure it should have a return value,
    //but perhaps it should have some sort of fault in case there are
    //no free registers.
    RenameInfo rename(RegIndex arch_reg);

    PhysRegIndex lookup(RegIndex phys_reg);

    bool isReady(PhysRegIndex arch_reg);

    /**
     * Marks the given register as ready, meaning that its value has been
     * calculated and written to the register file.
     * @params ready_reg The index of the physical register that is now
     *                   ready.
     */
    void markAsReady(PhysRegIndex ready_reg);

    void setEntry(RegIndex arch_reg, PhysRegIndex renamed_reg);

    void squash(vector<RegIndex> freed_regs,
                vector<UnmapInfo> unmaps);

    int numFreeEntries();

  private:
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

    /** Integer rename map. */
    RenameEntry *intRenameMap;

    /** Floating point rename map. */
    RenameEntry *floatRenameMap;

    /** Free list interface. */
    SimpleFreeList *freeList;

    // Might want to make all these scoreboards into one large scoreboard.

    /** Scoreboard of physical integer registers, saying whether or not they
     *  are ready.
     */
    vector<bool> intScoreboard;

    /** Scoreboard of physical floating registers, saying whether or not they
     *  are ready.
     */
    vector<bool> floatScoreboard;

    /** Scoreboard of miscellaneous registers, saying whether or not they
     *  are ready.
     */
    vector<bool> miscScoreboard;
};

#endif //__RENAME_MAP_HH__
