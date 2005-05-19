#ifndef __CPU_BETA_CPU_ALPHA_IMPL_HH__
#define __CPU_BETA_CPU_ALPHA_IMPL_HH__

#include "arch/alpha/isa_traits.hh"

#include "cpu/beta_cpu/alpha_params.hh"
#include "cpu/beta_cpu/cpu_policy.hh"

// Forward declarations.
template <class Impl>
class AlphaDynInst;

template <class Impl>
class AlphaFullCPU;

/** Implementation specific struct that defines several key things to the
 *  CPU, the stages within the CPU, the time buffers, and the DynInst.
 *  The struct defines the ISA, the CPU policy, the specific DynInst, the
 *  specific FullCPU, and all of the structs from the time buffers to do
 *  communication.
 *  This is one of the key things that must be defined for each hardware
 *  specific CPU implementation.
 */
struct AlphaSimpleImpl
{
    /** The ISA to be used. */
    typedef AlphaISA ISA;

    /** The type of MachInst. */
    typedef ISA::MachInst MachInst;

    /** The CPU policy to be used (ie fetch, decode, etc.). */
    typedef SimpleCPUPolicy<AlphaSimpleImpl> CPUPol;

    /** The DynInst to be used. */
    typedef AlphaDynInst<AlphaSimpleImpl> DynInst;

    /** The refcounted DynInst pointer to be used.  In most cases this is
     *  what should be used, and not DynInst *.
     */
    typedef RefCountingPtr<DynInst> DynInstPtr;

    /** The FullCPU to be used. */
    typedef AlphaFullCPU<AlphaSimpleImpl> FullCPU;

    /** The Params to be passed to each stage. */
    typedef AlphaSimpleParams Params;

    enum {
        MaxWidth = 8
    };
};

#endif // __CPU_BETA_CPU_ALPHA_IMPL_HH__
