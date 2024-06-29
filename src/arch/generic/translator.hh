#ifndef __ARCH_GENERIC_TRANSLATOR_HH__
#define __ARCH_GENERIC_TRANSLATOR_HH__

#include "base/cache/associative_cache.hh"
#include "params/Translator.hh"
#include "sim/clocked_object.hh"

namespace gem5
{
template <typename Entry>
class Translator : public ClockedObject
{
  private:
    AssociativeCache<Entry> _cache;
    Translator* _nextLevel;

  public:
    PARAMS(Translator);
    Translator(const Params &params)
      : SimObject(p), _nextLevel(p.next_level),
        _cache((name() + ".Translator").c_str(), p.num_entries,
            p.associativity, p.repl_policy, p.indexing_policy);

    Translator* nextLevel() const { return _nextLevel; }

};

} // namespace gem5

#endif
