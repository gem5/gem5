#include "mem/cache/tags/indexing_policies/tlb_set_associative.hh"

#include "mem/cache/replacement_policies/replaceable_entry.hh"

namespace gem5
{

TLBSetAssociative::TLBSetAssociative(const Params &p)
    : BaseIndexingPolicy(p)
{
}
// added function - this gets the 
Addr TLBSetAssociative::getVPNfromVA(const Addr addr) const 
{
    return (addr >> pageOffset) & vpnMask;
}

Addr extractTag(const Addr addr) override const 
{
    return addr;
}

uint32_t
SetAssociative::extractSet(const Addr addr) const
{
    return (addr >> setShift) & setMask;
}

std::vector<ReplaceableEntry*>
SetAssociative::getPossibleEntries(const Addr addr) const
{   
    return sets[extractSet(addr)];
}

} // namespace gem5
~      
