
#include "cpu/beta_cpu/rename_map.hh"

// Todo: Consider making functions inline.  Avoid having things that are
// using the zero register or misc registers from adding on the registers
// to the free list.  Possibly remove the direct communication between
// this and the freelist.  Considering making inline bool functions that
// determine if the register is a logical int, logical fp, physical int,
// physical fp, etc.

SimpleRenameMap::SimpleRenameMap(unsigned _numLogicalIntRegs,
                                 unsigned _numPhysicalIntRegs,
                                 unsigned _numLogicalFloatRegs,
                                 unsigned _numPhysicalFloatRegs,
                                 unsigned _numMiscRegs,
                                 RegIndex _intZeroReg,
                                 RegIndex _floatZeroReg)
    : numLogicalIntRegs(_numLogicalIntRegs),
      numPhysicalIntRegs(_numPhysicalIntRegs),
      numLogicalFloatRegs(_numLogicalFloatRegs),
      numPhysicalFloatRegs(_numPhysicalFloatRegs),
      numMiscRegs(_numMiscRegs),
      intZeroReg(_intZeroReg),
      floatZeroReg(_floatZeroReg)
{
    DPRINTF(Rename, "Rename: Creating rename map.  Phys: %i / %i, Float: "
            "%i / %i.\n", numLogicalIntRegs, numPhysicalIntRegs,
            numLogicalFloatRegs, numPhysicalFloatRegs);

    numLogicalRegs = numLogicalIntRegs + numLogicalFloatRegs;

    numPhysicalRegs = numPhysicalIntRegs + numPhysicalFloatRegs;

    //Create the rename maps, and their scoreboards.
    intRenameMap = new RenameEntry[numLogicalIntRegs];
    floatRenameMap = new RenameEntry[numLogicalRegs];

    // Should combine this into one scoreboard.
    intScoreboard.resize(numPhysicalIntRegs);
    floatScoreboard.resize(numPhysicalRegs);
    miscScoreboard.resize(numPhysicalRegs + numMiscRegs);

    // Initialize the entries in the integer rename map to point to the
    // physical registers of the same index, and consider each register
    // ready until the first rename occurs.
    for (RegIndex index = 0; index < numLogicalIntRegs; ++index)
    {
        intRenameMap[index].physical_reg = index;
        intScoreboard[index] = 1;
    }

    // Initialize the rest of the physical registers (the ones that don't
    // directly map to a logical register) as unready.
    for (PhysRegIndex index = numLogicalIntRegs;
         index < numPhysicalIntRegs;
         ++index)
    {
        intScoreboard[index] = 0;
    }

    int float_reg_idx = numPhysicalIntRegs;

    // Initialize the entries in the floating point rename map to point to
    // the physical registers of the same index, and consider each register
    // ready until the first rename occurs.
    // Although the index refers purely to architected registers, because
    // the floating reg indices come after the integer reg indices, they
    // may exceed the size of a normal RegIndex (short).
    for (PhysRegIndex index = numLogicalIntRegs;
         index < numLogicalRegs; ++index)
    {
        floatRenameMap[index].physical_reg = float_reg_idx++;
    }

    for (RegIndex index = numPhysicalIntRegs;
         index < numPhysicalIntRegs + numLogicalFloatRegs; ++index)
    {
        floatScoreboard[index] = 1;
    }

    // Initialize the rest of the physical registers (the ones that don't
    // directly map to a logical register) as unready.
    for (PhysRegIndex index = numPhysicalIntRegs + numLogicalFloatRegs;
         index < numPhysicalRegs;
         ++index)
    {
        floatScoreboard[index] = 0;
    }

    // Initialize the entries in the misc register scoreboard to be ready.
    for (RegIndex index = numPhysicalRegs;
         index < numPhysicalRegs + numMiscRegs; ++index)
    {
        miscScoreboard[index] = 1;
    }
}

SimpleRenameMap::~SimpleRenameMap()
{
    // Delete the rename maps as they were allocated with new.
    delete [] intRenameMap;
    delete [] floatRenameMap;
}

void
SimpleRenameMap::setFreeList(SimpleFreeList *fl_ptr)
{
    //Setup the interface to the freelist.
    freeList = fl_ptr;
}


// Don't allow this stage to fault; force that check to the rename stage.
// Simply ask to rename a logical register and get back a new physical
// register index.
SimpleRenameMap::RenameInfo
SimpleRenameMap::rename(RegIndex arch_reg)
{
    PhysRegIndex renamed_reg;
    PhysRegIndex prev_reg;

    if (arch_reg < numLogicalIntRegs) {

        // Record the current physical register that is renamed to the
        // requested architected register.
        prev_reg = intRenameMap[arch_reg].physical_reg;

        // If it's not referencing the zero register, then mark the register
        // as not ready.
        if (arch_reg != intZeroReg) {
            // Get a free physical register to rename to.
            renamed_reg = freeList->getIntReg();

            // Update the integer rename map.
            intRenameMap[arch_reg].physical_reg = renamed_reg;

            assert(renamed_reg >= 0 && renamed_reg < numPhysicalIntRegs);

            // Mark register as not ready.
            intScoreboard[renamed_reg] = false;
        } else {
            // Otherwise return the zero register so nothing bad happens.
            renamed_reg = intZeroReg;
        }
    } else if (arch_reg < numLogicalRegs) {
        // Subtract off the base offset for floating point registers.
//        arch_reg = arch_reg - numLogicalIntRegs;

        // Record the current physical register that is renamed to the
        // requested architected register.
        prev_reg = floatRenameMap[arch_reg].physical_reg;

        // If it's not referencing the zero register, then mark the register
        // as not ready.
        if (arch_reg != floatZeroReg) {
            // Get a free floating point register to rename to.
            renamed_reg = freeList->getFloatReg();

            // Update the floating point rename map.
            floatRenameMap[arch_reg].physical_reg = renamed_reg;

            assert(renamed_reg < numPhysicalRegs &&
                   renamed_reg >= numPhysicalIntRegs);

            // Mark register as not ready.
            floatScoreboard[renamed_reg] = false;
        } else {
            // Otherwise return the zero register so nothing bad happens.
            renamed_reg = floatZeroReg;
        }
    } else {
        // Subtract off the base offset for miscellaneous registers.
        arch_reg = arch_reg - numLogicalRegs;

        // No renaming happens to the misc. registers.  They are simply the
        // registers that come after all the  physical registers; thus
        // take the base architected register and add the physical registers
        // to it.
        renamed_reg = arch_reg + numPhysicalRegs;

        // Set the previous register to the same register; mainly it must be
        // known that the prev reg was outside the range of normal registers
        // so the free list can avoid adding it.
        prev_reg = renamed_reg;

        assert(renamed_reg < numPhysicalRegs + numMiscRegs);

        miscScoreboard[renamed_reg] = false;
    }

    return RenameInfo(renamed_reg, prev_reg);
}

//Perhaps give this a pair as a return value, of the physical register
//and whether or not it's ready.
PhysRegIndex
SimpleRenameMap::lookup(RegIndex arch_reg)
{
    if (arch_reg < numLogicalIntRegs) {
        return intRenameMap[arch_reg].physical_reg;
    } else if (arch_reg < numLogicalRegs) {
        // Subtract off the base FP offset.
//        arch_reg = arch_reg - numLogicalIntRegs;

        return floatRenameMap[arch_reg].physical_reg;
    } else {
        // Subtract off the misc registers offset.
        arch_reg = arch_reg - numLogicalRegs;

        // Misc. regs don't rename, so simply add the base arch reg to
        // the number of physical registers.
        return numPhysicalRegs + arch_reg;
    }
}

bool
SimpleRenameMap::isReady(PhysRegIndex phys_reg)
{
    if (phys_reg < numPhysicalIntRegs) {
        return intScoreboard[phys_reg];
    } else if (phys_reg < numPhysicalRegs) {

        // Subtract off the base FP offset.
//        phys_reg = phys_reg - numPhysicalIntRegs;

        return floatScoreboard[phys_reg];
    } else {
        // Subtract off the misc registers offset.
//        phys_reg = phys_reg - numPhysicalRegs;

        return miscScoreboard[phys_reg];
    }
}

// In this implementation the miscellaneous registers do not actually rename,
// so this function does not allow you to try to change their mappings.
void
SimpleRenameMap::setEntry(RegIndex arch_reg, PhysRegIndex renamed_reg)
{
    if (arch_reg < numLogicalIntRegs) {
        DPRINTF(Rename, "Rename Map: Integer register %i being set to %i.\n",
                (int)arch_reg, renamed_reg);

        intRenameMap[arch_reg].physical_reg = renamed_reg;
    } else {
        assert(arch_reg < (numLogicalIntRegs + numLogicalFloatRegs));

        DPRINTF(Rename, "Rename Map: Float register %i being set to %i.\n",
                (int)arch_reg - numLogicalIntRegs, renamed_reg);

        floatRenameMap[arch_reg].physical_reg = renamed_reg;
    }
}

void
SimpleRenameMap::squash(vector<RegIndex> freed_regs,
                        vector<UnmapInfo> unmaps)
{
    panic("Not sure this function should be called.");

    // Not sure the rename map should be able to access the free list
    // like this.
    while (!freed_regs.empty()) {
        RegIndex free_register = freed_regs.back();

        if (free_register < numPhysicalIntRegs) {
            freeList->addIntReg(free_register);
        } else {
            // Subtract off the base FP dependence tag.
            free_register = free_register - numPhysicalIntRegs;
            freeList->addFloatReg(free_register);
        }

        freed_regs.pop_back();
    }

    // Take unmap info and roll back the rename map.
}

void
SimpleRenameMap::markAsReady(PhysRegIndex ready_reg)
{
    DPRINTF(Rename, "Rename map: Marking register %i as ready.\n",
            (int)ready_reg);

    if (ready_reg < numPhysicalIntRegs) {
        assert(ready_reg >= 0);

        intScoreboard[ready_reg] = 1;
    } else if (ready_reg < numPhysicalRegs) {

        // Subtract off the base FP offset.
//        ready_reg = ready_reg - numPhysicalIntRegs;

        floatScoreboard[ready_reg] = 1;
    } else {
        //Subtract off the misc registers offset.
//        ready_reg = ready_reg - numPhysicalRegs;

        miscScoreboard[ready_reg] = 1;
    }
}

int
SimpleRenameMap::numFreeEntries()
{
    int free_int_regs = freeList->numFreeIntRegs();
    int free_float_regs = freeList->numFreeFloatRegs();

    if (free_int_regs < free_float_regs) {
        return free_int_regs;
    } else {
        return free_float_regs;
    }
}
