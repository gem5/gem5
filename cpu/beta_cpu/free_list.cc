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

    // Put all of the extra physical registers onto the free list.  This
    // means excluding all of the base logical registers.
    for (PhysRegIndex i = numLogicalIntRegs;
         i < numPhysicalIntRegs; ++i)
    {
        freeIntRegs.push(i);
    }

    // Put all of the extra physical registers onto the free list.  This
    // means excluding all of the base logical registers.  Because the
    // float registers' indices start where the physical registers end,
    // some math must be done to determine where the free registers start.
    for (PhysRegIndex i = numPhysicalIntRegs + numLogicalFloatRegs;
         i < numPhysicalRegs; ++i)
    {
        cprintf("Free List: Adding register %i to float list.\n", i);
        freeFloatRegs.push(i);
    }
}

