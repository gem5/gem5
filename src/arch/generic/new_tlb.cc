
// come back to this, after .hh is finalized

#include "base/cache/associative_cache.hh"
#include "generic/new_tlb.hh
#include "params/NewTLB.hh"
#include "sim/sim_object.hh"

namespace gem5
{

// this will literally just be the actual functions
// however, its important to take notes 

// questions:
/*
- do i just return the TLB entry or do I have to return a type? not sure there

*/

// look at the Translator class, make sure that I can access the Associative cache
// Associative Cache specific

TLBEntry * lookup(----)
    // AssociativeCache::findEntry(const Addr addr)

TLBEntry * insert(----)
    // AssociativeCache::insertEntry(const Addr addr, Entry *entry)

TLBEntry * remove(----)
    // AssociativeCache::findEntry(const Addr addr)
    // AssociativeCache::invalidate(Entry *entry)
   
void flushAll(---)
    // AssociateCache::clear()
   
void evictLRU(---)
    // AssociativeCachefindVictim(const Addr addr)
    // plus a few more things!

void demapPage(---) // not entirely sure

// all the Fault funcitons might need to be defined as virtual functions

void serialize(---)
    // no idea where the serialize stuff happens
    tlb[x].serializeSection?

void unserialize(---)


// Stats functions:

// AC Functions that I am not sure how they would incorporate, but they would
/*
- accessEntry(Entry *entry) - this updates the replacement information for an entry
- i think this reminds me of the lru sequence?
- findVictime - this 




*/


};

