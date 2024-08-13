#include "base/cache/associative_cache.hh"
#include "generic/new_tlb.hh 
// this generic/new_tlb.hh includes both TLB definition and the TLB Entry definition
#include "params/NewTLB.hh"
#include "sim/sim_object.hh"

/*
Questions:
- How do I specify the return type?
*/

namespace gem5
{

// Constructor


// Cache Management

/** Lookup
 * looks up an entry based off of certain params
 * calculates stats
 * returns an entry
 * 
 * @TBD: call the build key function determine how to access BaseMMU, determine how to access status
 * @AC: findEntry(const Addr addr)
 * @params: key | mode (for stats) | hidden (to know whether to update LRU)
 * @result: returns entry that was found in the AC
 * 
 */
virtual TLBEntry * lookup(Addr key, uint16_t asid, BaseMMU::Mode mode, bool hidden) {
    
    TLBEntry *entry = this->_cache.findEntry(key)

    // following code taken from arch/riscv/tlb.cc
    if (!hidden) {
        if (entry) 
            entry->lruSeq = nextSeq(); 
        
        // noting the misses/hits
        if (mode == BaseMMU::Write)
            stats.writeAccesses++;
        else
            stats.readAccesses++;
        
        // noting the misses/hits
        if (!entry) {
            if (mode == BaseMMU::Write)
                stats.writeMisses++;
            else
                stats.readMisses++;
        }
        else {
            if (mode == BaseMMU::Write)
                stats.writeHits++;
            else
                stats.readHits++;
        }

        // return entry
        return entry;
}

/** Insert
 * insert entry, sets the parameters of the accordingly
 * calculates stats
 * returns an entry
 * 
 * @TBD: 
 * @AC: insertEntry(const Addr addr, Entry *entry), 
 * @params: key | mode (for stats) | hidden (to know whether to update LRU)
 * @result: returns the new entry
 * 
 */
TLBEntry * insert(Addr vpn, const TlbEntry &entry, uint64_t pcid) {
/** Differences
 * key for insert
 * asid, vs. psid
 * logBytes
 * full system vs. not full system
 */

// calling buildKey
// this is function for x86
// this is null for riscv



}

TLBEntry * remove(----)
    // AssociativeCache::findEntry(const Addr addr)
    // AssociativeCache::invalidate(Entry *entry)
   
/**
 * clears all entry in associative cache
 * @params: none
 * @result: none
 */
void flushAll() {
    this->_cache.clear()
    // AssociateCache::clear()

}
   
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

// Helper Function

// riscv
static Addr buildKey(Addr vpn, uint16_t asid)

//x86
inline Addr concAddrPcid(Addr vpn, uint64_t pcid)


};


