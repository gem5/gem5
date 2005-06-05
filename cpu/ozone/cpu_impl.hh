
#ifndef __CPU_OOO_CPU_OOO_IMPL_HH__
#define __CPU_OOO_CPU_OOO_IMPL_HH__

#include "arch/alpha/isa_traits.hh"

template <class Impl>
class OoOCPU;

template <class Impl>
class OoODynInst;

struct OoOImpl {
    typedef AlphaISA ISA;
    typedef OoOCPU<OoOImpl> OoOCPU;
    typedef OoOCPU FullCPU;
    typedef OoODynInst<OoOImpl> DynInst;
    typedef RefCountingPtr<DynInst> DynInstPtr;
};

#endif // __CPU_OOO_CPU_OOO_IMPL_HH__
