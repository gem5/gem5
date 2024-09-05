#include "mem/cache/tags/indexing_policies/tlb_set_associative.hh"
#include "mem/cache/replacement_policies/replaceable_entry.hh"

namespace gem5
{

// Constructor
TLBIndexing::TLBIndexing(const Params &p)
    : BaseIndexingPolicy(p)
{
}

std::vector<ReplaceableEntry*>
TLBIndexing::getPossibleEntries(const Addr addr) const
{ 
    return sets[extractSet(addr)];

}

// useless function
Addr
TLBIndexing::regenerateAddr(const Addr tag, const ReplaceableEntry* entry) const override{
    return tag;	 
}

// used everywhere, but just returns what was there before 
Addr
TLBIndexing::extractTag(const Addr addr) override const
{
    return addr;
}

// [virtual page number][id][0*] -> [virtual page number][0's]
auto
TLBIndexing::extractSet(const Addr addr) const
{
    return (addr >> idShift) & setMask;

}

// [virtual page number][page offset] -> [virtual page number][0's]
Addr
TLBIndexing::getVPNfromVA(const Addr addr) const
{
    Addr vpn = addr >> pageShift;
    return vpn & vpnMask;
}

// [virtual page number][0's] -> [virtual page number][id] OR [virtual page number][id][0's 
Addr 
TLBIndexing::buildKey(Addr vpn, Addr id) const
{
    return (vpn << idShift) | id; 
}


} // namespace gem5
