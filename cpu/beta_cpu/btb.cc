#include <math.h>

#include "cpu/beta_cpu/btb.hh"
#include "base/trace.hh"

DefaultBTB::DefaultBTB(unsigned _numEntries,
                       unsigned _tagBits,
                       unsigned _instShiftAmt)
    : numEntries(_numEntries),
      tagBits(_tagBits),
      instShiftAmt(_instShiftAmt)
{
    // @todo Check to make sure num_entries is valid (a power of 2)

    DPRINTF(Fetch, "BTB: Creating BTB object.\n");

    btb = new BTBEntry[numEntries];

    for (int i = 0; i < numEntries; ++i)
    {
        btb[i].valid = false;
    }

    idxMask = numEntries - 1;

    tagMask = (1 << tagBits) - 1;

    tagShiftAmt = instShiftAmt + (int)log2(numEntries);
}

inline
unsigned
DefaultBTB::getIndex(const Addr &inst_PC)
{
    // Need to shift PC over by the word offset.
    return (inst_PC >> instShiftAmt) & idxMask;
}

inline
Addr
DefaultBTB::getTag(const Addr &inst_PC)
{
    return (inst_PC >> tagShiftAmt) & tagMask;
}

bool
DefaultBTB::valid(const Addr &inst_PC)
{
    unsigned btb_idx = getIndex(inst_PC);

    Addr inst_tag = getTag(inst_PC);

    if (btb[btb_idx].valid && inst_tag == btb[btb_idx].tag) {
        return true;
    } else {
        return false;
    }
}

// @todo Create some sort of return struct that has both whether or not the
// address is valid, and also the address.  For now will just use addr = 0 to
// represent invalid entry.
Addr
DefaultBTB::lookup(const Addr &inst_PC)
{
    unsigned btb_idx = getIndex(inst_PC);

    Addr inst_tag = getTag(inst_PC);

    if (btb[btb_idx].valid && inst_tag == btb[btb_idx].tag) {
        return btb[btb_idx].target;
    } else {
        return 0;
    }
}

void
DefaultBTB::update(const Addr &inst_PC, const Addr &target)
{
    unsigned btb_idx = getIndex(inst_PC);

    btb[btb_idx].valid = true;
    btb[btb_idx].target = target;
    btb[btb_idx].tag = getTag(inst_PC);
}
