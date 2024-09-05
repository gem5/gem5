#include "base/cache/associative_cache.hh"
#include "generic/new_tlb.hh"
#include "params/NewTLB.hh"
#include "sim/sim_object.hh"
#include "base/cache/cache_entry.hh"
#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/named.hh"
#include "base/types.hh"
#include "mem/cache/replacement_policies/base.hh"
#include "mem/cache/replacement_policies/replaceable_entry.hh"
#include "mem/cache/tags/indexing_policies/base.hh"

// indexing policy - in process of development
#include "mem/cache/tags/indexing_policies/tlb_indexing.hh"

// replacement policy - preexisting
#include "mem/cache/replacement_policies/lru_rp.hh"


namespace gem5 {

/** Constructor for Associative Cache **/
TLB(const char* name, size_t num_entries, size_t associativity,
        BaseReplacementPolicy* repl_policy, BaseIndexingPolicy* indexing_policy)
{
    // Initialize the AssociativeCache through the _cache member in Translator
    _cache.init(num_entries, associativity, repl_policy, indexing_policy);
}


/** Lookup
 * @AC: findEntry(const Addr addr)
 * @params: vpn, id, mode, updateLRU
 * @result: returns entry that was found
 */

virtual TLBEntry * lookup(Addr vpn, auto id, BaseMMU::Mode mode, bool updateLRU) {

    Addr key = buildKey(vpn, id);

    // findEntry
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
 * @params: vpn, id
 * @result: none
 */
void remove(Addr vpn, auto id) {

    Addr key = buildKey(vpn, id);
    this->_cache.invalidate(findEntry(key));

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

/** evictLRU
 * clears the least recently used within a set
 * @AC: findVictim(), invalidate()
 * @params: vpn -> in order to find which set we need to remove
 * @result: none
 */
void evictLRU(Addr vpn) {

    TLBEntry * entry = this->_cache.findVictim(addr);
    this->_cache.invalidate(entry);

}


virtual TlbEntry * insert(Addr vpn, auto id, const TlbEntry &entry);
virtual demapPage(Addr addr, uint64_t address_space)
virtual Fault TLB::translate(const RequestPtr &req, ThreadContext *tc, BaseMMU::Mode mode)
virtual Fault TLB::translateFunctional(const RequestPtr &req, ThreadContext *tc, BaseMMU::Mode mode)
virtual Fault TLB::translateAtomic(const RequestPtr &req, ThreadContext *tc, BaseMMU::Mode mode)
virtual Fault TLB::translateTiming(const RequestPtr &req, ThreadContext *tc, BaseMMU::Mode mode)



virtual serialize(CheckpointIn &cp)
virtual unserialize(CheckpointIn &cp)


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
{

}

}; // end of namespace
