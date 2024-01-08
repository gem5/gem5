#ifndef __MEM_CACHE_CACHESELECTOR_HH__
#define __MEM_CACHE_CACHESELECTOR_HH__

// #include "base/compiler.hh"
// #include "base/logging.hh"
// #include "base/types.hh"
// #include "mem/cache/base.hh"
// #include "mem/packet.hh"


#include "params/CacheSelector.hh"
#include "sim/sim_object.hh"

namespace gem5
{

    class CacheSelector : public SimObject 
    {
        public:
            CacheSelector(const CacheSelectorParams &p);

    };


}




#endif