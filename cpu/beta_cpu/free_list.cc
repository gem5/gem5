#include "base/trace.hh"

#include "cpu/beta_cpu/free_list.hh"

SimpleFreeList::SimpleFreeList(unsigned _numLogicalIntRegs,
                               unsigned _numPhysicalIntRegs,
                               unsigned _numLogicalFloatRegs,
                               unsigned _numPhysicalFloatRegs)
    : numLogicalIntRegs(_numLogicalIntRegs),
      numPhysicalIntRegs(_numPhysicalIntRegs),
      numLogicalFloatRegs(_numLogicalFloatRegs),
      numPhysicalFloatRegs(_numPhysicalFloatRegs),
      numPhysicalRegs(numPhysicalIntRegs + numPhysicalFloatRegs)
{
    DPRINTF(FreeList, "FreeList: Creating new free list object.\n");

    // DEBUG stuff.
    freeIntRegsScoreboard.resize(numPhysicalIntRegs);

    freeFloatRegsScoreboard.resize(numPhysicalRegs);

    for (PhysRegIndex i = 0; i < numLogicalIntRegs; ++i) {
        freeIntRegsScoreboard[i] = 0;
    }

    // Put all of the extra physical registers onto the free list.  This
    // means excluding all of the base logical registers.
    for (PhysRegIndex i = numLogicalIntRegs;
         i < numPhysicalIntRegs; ++i)
    {
        freeIntRegs.push(i);

        freeIntRegsScoreboard[i] = 1;
    }

    for (PhysRegIndex i = 0; i < numPhysicalIntRegs + numLogicalFloatRegs;
         ++i)
    {
        freeFloatRegsScoreboard[i] = 0;
    }

    // Put all of the extra physical registers onto the free list.  This
    // means excluding all of the base logical registers.  Because the
    // float registers' indices start where the physical registers end,
    // some math must be done to determine where the free registers start.
    for (PhysRegIndex i = numPhysicalIntRegs + numLogicalFloatRegs;
         i < numPhysicalRegs; ++i)
    {
        freeFloatRegs.push(i);

        freeFloatRegsScoreboard[i] = 1;
    }
}

