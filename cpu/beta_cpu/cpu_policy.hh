#ifndef __CPU_POLICY_HH__
#define __CPU_POLICY_HH__

#include "cpu/beta_cpu/fetch.hh"
#include "cpu/beta_cpu/decode.hh"
#include "cpu/beta_cpu/rename.hh"
#include "cpu/beta_cpu/iew.hh"
#include "cpu/beta_cpu/commit.hh"

#include "cpu/beta_cpu/inst_queue.hh"
#include "cpu/beta_cpu/regfile.hh"
#include "cpu/beta_cpu/free_list.hh"
#include "cpu/beta_cpu/rename_map.hh"
#include "cpu/beta_cpu/rob.hh"

template<class Impl>
struct SimpleCPUPolicy
{
    typedef PhysRegFile<Impl> RegFile;
    typedef SimpleFreeList FreeList;
    typedef SimpleRenameMap RenameMap;
    typedef ROB<Impl> ROB;
    typedef InstructionQueue<Impl> IQ;

    typedef SimpleFetch<Impl> Fetch;
    typedef SimpleDecode<Impl> Decode;
    typedef SimpleRename<Impl> Rename;
    typedef SimpleIEW<Impl, IQ> IEW;
    typedef SimpleCommit<Impl> Commit;
};

#endif //__CPU_POLICY_HH__
