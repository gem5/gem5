

#include "base/cache/associative_cache.hh"
#include "generic/new_tlb.hh
#include "params/NewTLB.hh"
#include "sim/sim_object.hh"

/*
Questions:
- How do I specify the return type?
*/

namespace gem5
{


// Cache Management
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
    // plus a few more functions

void demapPage(---)

// Translation

// virtual Fault translate()
// check permissions
virtual Fault TLB::translateAtomic(const RequestPtr &req, ThreadContext *tc, BaseMMU::Mode mode)
virtual Fault TLB::translateFunctional(const RequestPtr &req, ThreadContext *tc, BaseMMU::Mode mode)
virtual Fault TLB::translateTiming(const RequestPtr &req, ThreadContext *tc, BaseMMU::Mode mode)


// Serialization
void serialize(---)
void unserialize(---)


// Statistics
TLB::TlbStats::TlbStats(statistics::Group *parent)
  : statistics::Group(parent),
    ADD_STAT(readHits, statistics::units::Count::get(), "read hits"),
    ADD_STAT(readMisses, statistics::units::Count::get(), "read misses"),
    ADD_STAT(readAccesses, statistics::units::Count::get(), "read accesses"),
    ADD_STAT(writeHits, statistics::units::Count::get(), "write hits"),
    ADD_STAT(writeMisses, statistics::units::Count::get(), "write misses"),
    ADD_STAT(writeAccesses, statistics::units::Count::get(), "write accesses"),
    ADD_STAT(hits, statistics::units::Count::get(),
             "Total TLB (read and write) hits", readHits + writeHits),
    ADD_STAT(misses, statistics::units::Count::get(),
             "Total TLB (read and write) misses", readMisses + writeMisses),
    ADD_STAT(accesses, statistics::units::Count::get(),
             "Total TLB (read and write) accesses",
             readAccesses + writeAccesses)


};
