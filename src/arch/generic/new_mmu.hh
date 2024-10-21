#ifndef __ARCH_GENERIC_NEW_MMU_HH__
#define __ARCH_GENERIC_NEW_MMU_HH__

#include <set>

#include "base/cache/associative_cache.hh"
#include "mem/request.hh"
#include "mem/translation_gen.hh"
#include "params/BaseMMU.hh"
#include "sim/sim_object.hh"

namespace gem5{

class Translator : public SimObject
{
    protected:
        typedef TranslatorParams Params;
        Translator(const Params &p)
          : SimObject(p), dtb(p.dtb), itb(p.itb)
        {}


};

} // namespace gem5

#endif
