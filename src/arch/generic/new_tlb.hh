#ifndef __ARCH_GENERIC_NEW_TLB_HH__
#define __ARCH_GENERIC_NEW_TLB_HH__

#include "base/cache/associative_cache.hh"
#include "params/NewTLB.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class NewTLB : public AssociativeCache, public SimObject
{
  private:
  public:
    typedef NewTLBParams Params;
    NewTLB(const Params &params)
      : AssociativeCache(name(), p.size, p.assoc,
        p.replacement_policy, p.indexing_policy),
      SimObject(p)
    {

    }
};

} // namespace gem5

#endif
