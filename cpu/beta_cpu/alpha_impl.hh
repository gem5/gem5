#ifndef __ALPHA_IMPL_HH__
#define __ALPHA_IMPL_HH__

#include "arch/alpha/isa_traits.hh"

#include "cpu/beta_cpu/comm.hh"
#include "cpu/beta_cpu/cpu_policy.hh"
#include "cpu/beta_cpu/alpha_params.hh"

#include "cpu/beta_cpu/commit.hh"
#include "cpu/beta_cpu/decode.hh"
#include "cpu/beta_cpu/fetch.hh"
#include "cpu/beta_cpu/free_list.hh"
#include "cpu/beta_cpu/iew.hh"

#include "cpu/beta_cpu/inst_queue.hh"
#include "cpu/beta_cpu/regfile.hh"
#include "cpu/beta_cpu/rename.hh"
#include "cpu/beta_cpu/rename_map.hh"
#include "cpu/beta_cpu/rob.hh"

class AlphaDynInst;
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
    typedef AlphaDynInst DynInst;

    /** The FullCPU to be used. */
    typedef AlphaFullCPU FullCPU;

    /** The Params to be passed to each stage. */
    typedef AlphaSimpleParams Params;

    /** The struct for communication between fetch and decode. */
    typedef SimpleFetchSimpleDecode<AlphaSimpleImpl> FetchStruct;

    /** The struct for communication between decode and rename. */
    typedef SimpleDecodeSimpleRename<AlphaSimpleImpl> DecodeStruct;

    /** The struct for communication between rename and IEW. */
    typedef SimpleRenameSimpleIEW<AlphaSimpleImpl> RenameStruct;

    /** The struct for communication between IEW and commit. */
    typedef SimpleIEWSimpleCommit<AlphaSimpleImpl> IEWStruct;

    /** The struct for communication within the IEW stage. */
    typedef IssueStruct<AlphaSimpleImpl> IssueStruct;

    /** The struct for all backwards communication. */
    typedef TimeBufStruct TimeStruct;
};



#endif // __ALPHA_IMPL_HH__
