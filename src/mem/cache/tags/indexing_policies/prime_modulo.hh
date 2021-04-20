/**
 * @file
 * Declaration of a prime modulo indexing policy.
 */

#ifndef __MEM_CACHE_INDEXING_POLICIES_PRIME_MODULO_HH__
#define __MEM_CACHE_INDEXING_POLICIES_PRIME_MODULO_HH__

#include <bits/stdint-uintn.h>

#include <cmath>
#include <vector>

#include "mem/cache/tags/indexing_policies/base.hh"
#include "params/PrimeModulo.hh"

class ReplaceableEntry;

class PrimeModulo : public BaseIndexingPolicy
{
  private:

    uint32_t extractSet(const Addr addr) const;

    bool isPrime(const uint64_t modulo) const;

    const uint64_t modulo;

  public:
    /** Convenience typedef. */
     typedef PrimeModuloParams Params;

    /**
     * Construct and initialize this policy.
     */
    PrimeModulo(const Params *p);

    /**
     * Destructor.
     */
    ~PrimeModulo() {};

    /**
     * Generate the tag from the given address.
     *
     * @param addr The address to get the tag from.
     * @return The tag of the address.
     */
    Addr extractTag(const Addr addr) const;

    /**
     * Find all possible entries for insertion and replacement of an address.
     * Should be called immediately before ReplacementPolicy's findVictim()
     * not to break cache resizing.
     *
     * @param addr The addr to a find possible entries for.
     * @return The possible entries.
     */
    std::vector<ReplaceableEntry*> getPossibleEntries(const Addr addr) const
                                                                   override;

    /**
     * Regenerate an entry's address from its tag and assigned set and way.
     * Uses the inverse of the skewing function.
     *
     * @param tag The tag bits.
     * @param entry The entry.
     * @return the entry's address.
     */
    Addr regenerateAddr(const Addr tag, const ReplaceableEntry* entry) const
                                                                   override;
};

#endif //__MEM_CACHE_INDEXING_POLICIES_PRIME_MODULO_HH__
