#include "base/cache/associative_cache.hh"
#include "generic/new_tlb.hh"
#include "params/NewTLB.hh"
#include "sim/sim_object.hh"

// set_associative implementation
#include "mem/cache/tags/indexing_policies/tlb_set_assocative.hh"

// lru_rp
#include "mem/cache/replacement_policies/lru_rp.hh"


namespace gem5 {

/** Lookup
 * @AC: findEntry(const Addr addr)
 * @params: key | mode (for stats) | updateLRU (to know whether to update LRU)
 * @result: returns entry that was found in the AC
 */

virtual TLBEntry * lookup(Addr vpn, auto id, BaseMMU::Mode mode, bool updateLRU) {

    // concatenate vpn and id
    Addr key = vpn | id;
    
    TLBEntry *entry = this->_cache.findEntry(key);

    if (updateLRU) {
        accessEntry(entry); // this is standard
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

    return entry;
}

/** Remove
 * removes an index
 * @AC: findEntry(const Addr addr), invalidate(Entry *entry)
 * @params: vpn and id -> in order to find Entry
 * @result: none
 */
void remove(Addr vpn, auto id) {

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

-----

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

virtual TlbEntry * insert(Addr lookup_key, const TlbEntry &entry, Addr insert_key) {
    // LOOKUP
    // CHECK IF EXISTS
    // CHECK IF FULL - evict LRU
    // SET NEW ENTRY
    // FULL SYSTEM
    return newEntry;
}


void evictLRU(---)
    // AssociativeCachefindVictim(const Addr addr)
    // plus a few more functions

void demapPage(---)

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

}; // end of gem5 namspace
