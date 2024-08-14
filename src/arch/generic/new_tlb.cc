#include "base/cache/associative_cache.hh"
#include "generic/new_tlb.hh 
#include "params/NewTLB.hh"
#include "sim/sim_object.hh"


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
  /** ISA specification
   *  x86 key = concAddrPcid(vpn, pcid)
   *  RISCV key = buildKey(vpn, asid)
   */
   
virtual TLBEntry * lookup(Addr key, BaseMMU::Mode mode, bool hidden) {
    
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
// insert_key for RISCV = build key(vpn, entry.asid)
// insert_key for x86 = vpn

// lookup_key for RISCV = vpn
// lookup_key for x86 = 

TlbEntry * insert(Addr lookup_key, const TlbEntry &entry, uint64_t pcid, Addr insert_key) {
/** Differences
 * key for insert
 * asid, vs. psid
 * logBytes
 * full system vs. not full system
 */

// internal lookup is called, so the key is already set
TlbEntry *newEntry = lookup(lookup_key, entry.asid, BaseMMU::read, true);

// find an entry

// free list handling


// set the new entry and stuff
RISCV:
- newEntry->pte = 
- creates an Addr of buildKey(vpn, entry.asid)
- sets a newEntry to entry // this is explicity 
- update lruEq

A



// figure out where to actually insert




}

/** Remove
 * removes an index
 * @AC: findEntry(const Addr addr), invalidate(Entry *entry)
 * @params: address to delete
 * @result: none
 */
void remove(Addr addr) {
    this->_cache.invalidate(findEntry(addr));
}
   
/** FlushAll
 * clears all entry in associative cache
 * @AC: clear()
 * @params: none
 * @result: none
 */
void flushAll() {
    this->_cache.clear()
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


