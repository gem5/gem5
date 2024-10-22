#ifndef __ARCH_GENERIC_TRANSLATOR_HH__
#define __ARCH_GENERIC_TRANSLATOR_HH__

#include "base/cache/associative_cache.hh"
#include "params/Translator.hh"
#include "sim/clocked_object.hh"

namespace gem5
{
  /**
   * This is the base object from which the TLB and the Page Table Walkers will
   * inherit from. This object focuses on having a small interface which will
   * be used by the MMU to access the TLB and the PTWs.
   */
  template <typename Entry>
  class Translator : public ClockedObject
  {
  private:                          // Change private to protected
    AssociativeCache<Entry> _cache; // This could be TLB or Page Walk Cache
    // The associativeCache will be changed to a TLBCache
    Translator *_nextLevel; // Access to multi-level TLB and PT Walkers

  public:
    PARAMS(Translator);
    Translator(const Params &params)
        : SimObject(p), _nextLevel(p.next_level),
          _cache((name() + ".TranslatorCache").c_str(), p.num_entries,
                 p.associativity, p.repl_policy, p.indexing_policy);

    /**
     * This function allows for easy access to the TLB and PTW hierarchy
     */
    virtual Translator *nextLevel() = 0;
  };

} // namespace gem5

#endif
