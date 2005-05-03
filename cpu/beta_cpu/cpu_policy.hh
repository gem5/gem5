#ifndef __CPU_BETA_CPU_CPU_POLICY_HH__
#define __CPU_BETA_CPU_CPU_POLICY_HH__

#include "cpu/beta_cpu/bpred_unit.hh"
#include "cpu/beta_cpu/inst_queue.hh"
#include "cpu/beta_cpu/regfile.hh"
#include "cpu/beta_cpu/free_list.hh"
#include "cpu/beta_cpu/rename_map.hh"
#include "cpu/beta_cpu/rob.hh"
#include "cpu/beta_cpu/store_set.hh"
#include "cpu/beta_cpu/mem_dep_unit.hh"
#include "cpu/beta_cpu/ldstq.hh"

#include "cpu/beta_cpu/fetch.hh"
#include "cpu/beta_cpu/decode.hh"
#include "cpu/beta_cpu/rename.hh"
#include "cpu/beta_cpu/iew.hh"
#include "cpu/beta_cpu/commit.hh"

#include "cpu/beta_cpu/comm.hh"

template<class Impl>
struct SimpleCPUPolicy
{
    typedef TwobitBPredUnit<Impl> BPredUnit;
    typedef PhysRegFile<Impl> RegFile;
    typedef SimpleFreeList FreeList;
    typedef SimpleRenameMap RenameMap;
    typedef ROB<Impl> ROB;
    typedef InstructionQueue<Impl> IQ;
    typedef MemDepUnit<StoreSet, Impl> MemDepUnit;
    typedef LDSTQ<Impl> LDSTQ;

    typedef SimpleFetch<Impl> Fetch;
    typedef SimpleDecode<Impl> Decode;
    typedef SimpleRename<Impl> Rename;
    typedef SimpleIEW<Impl> IEW;
    typedef SimpleCommit<Impl> Commit;

    /** The struct for communication between fetch and decode. */
    typedef SimpleFetchSimpleDecode<Impl> FetchStruct;

    /** The struct for communication between decode and rename. */
    typedef SimpleDecodeSimpleRename<Impl> DecodeStruct;

    /** The struct for communication between rename and IEW. */
    typedef SimpleRenameSimpleIEW<Impl> RenameStruct;

    /** The struct for communication between IEW and commit. */
    typedef SimpleIEWSimpleCommit<Impl> IEWStruct;

    /** The struct for communication within the IEW stage. */
    typedef IssueStruct<Impl> IssueStruct;

    /** The struct for all backwards communication. */
    typedef TimeBufStruct TimeStruct;

};

#endif //__CPU_BETA_CPU_CPU_POLICY_HH__
