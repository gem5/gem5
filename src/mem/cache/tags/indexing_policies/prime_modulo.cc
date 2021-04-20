/**
 * @file
 * Definitions of a prime modulo indexing policy.
 */

#include "mem/cache/tags/indexing_policies/prime_modulo.hh"

#include "base/bitfield.hh"
#include "base/intmath.hh"
#include "base/logging.hh"
#include "mem/cache/replacement_policies/replaceable_entry.hh"

PrimeModulo::PrimeModulo(const Params *p)
    : BaseIndexingPolicy(p),
      modulo(p->modulo)
{
    fatal_if(!isPrime(modulo), "modulo should be a prime");
    fatal_if(!(modulo < numSets), "modulo should be a prime less than the \
        number of sets");
}

bool PrimeModulo::isPrime(const uint64_t modulo) const {
    if (modulo <= 1)
        return false;
    if (modulo == 2)
        return true;
    for (unsigned int i = 2; i <= sqrt(modulo); ++i)
        if (modulo % i == 0)
            return false;
    return true;
}

uint32_t
PrimeModulo::extractSet(const Addr addr) const
{
    return addr % modulo;
}

Addr PrimeModulo::extractTag(const Addr addr) const
{
  return addr / modulo;
}

Addr
PrimeModulo::regenerateAddr(const Addr tag, const ReplaceableEntry* entry)
                                                                        const
{
    return tag * modulo + entry->getSet();
}

std::vector<ReplaceableEntry*>
PrimeModulo::getPossibleEntries(const Addr addr) const
{
    return sets[extractSet(addr)];
}

PrimeModulo*
PrimeModuloParams::create()
{
    return new PrimeModulo(this);
}
