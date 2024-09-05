#include "mem/cache/tags/indexing_policies/tlb_set_associative.hh"

#include "mem/cache/replacement_policies/replaceable_entry.hh"

namespace gem5
{

TLBSetAssociative::TLBSetAssociative(const Params &p)
    : BaseIndexingPolicy(p)
{
}
// added function - this gets the vpn from the virtual address
Addr TLBSetAssociative::getVPNfromVA(const Addr addr) const
{
    return (addr >> pageOffset) & vpnMask;
}

// override from BaseIndexingPolicy because tag uses
// two variables and this is the easiest solution
Addr extractTag(const Addr addr) override const
{
    return addr;
}

// function is not used often because we want to have the option for
// TLB to not be fully associative, even though most times it will
// be fully associative
auto
TLBSetAssociative::extractSet(const Addr addr) const
{
    return (addr >> setShift) & setMask;
}

std::vector<ReplaceableEntry*>
TLBSetAssociative::getPossibleEntries(const Addr addr) const
{
    if (num_entries == associativity) {

       std::vector<ReplaceableEntry*> entries;
       for (int i = 0; i < num_entries; i++) {
		entries.append(sets[i][0]);
       }
       return entries;

    } else {

       return sets[extractSet(addr)];

    }
}

} // namespace gem5
~
