#ifndef __COMM_HH__
#define __COMM_HH__

#include <stdint.h>
#include "arch/alpha/isa_traits.hh"
#include "cpu/inst_seq.hh"

using namespace std;

// Find better place to put this typedef.
typedef short int PhysRegIndex;

// Might want to put constructors/destructors here.
template<class Impl>
struct SimpleFetchSimpleDecode {
    // Consider having a field of how many ready instructions.
    typename Impl::DynInst *insts[1];
};

template<class Impl>
struct SimpleDecodeSimpleRename {
    // Consider having a field of how many ready instructions.
    typename Impl::DynInst *insts[1];
};

template<class Impl>
struct SimpleRenameSimpleIEW {
    // Consider having a field of how many ready instructions.
    typename Impl::DynInst *insts[1];
};

template<class Impl>
struct SimpleIEWSimpleCommit {
    // Consider having a field of how many ready instructions.
    typename Impl::DynInst *insts[1];
};

template<class Impl>
struct IssueStruct {
    typename Impl::DynInst *insts[1];
};

struct TimeBufStruct {
    struct decodeComm {
        bool squash;
        bool stall;
        bool predIncorrect;
        uint64_t branchAddr;

        //Question, is it worthwhile to have this Addr passed along
        //by each stage, or just have Fetch look it up in the proper
        //amount of cycles in the time buffer?
        //Both might actually be needed because decode can send a different
        //nextPC if the bpred was wrong.
        uint64_t nextPC;
    };

    decodeComm decodeInfo;

    // Rename can't actually tell anything to squash or send a new PC back
    // because it doesn't do anything along those lines.  But maybe leave
    // these fields in here to keep the stages mostly orthagonal.
    struct renameComm {
        bool squash;
        bool stall;

        uint64_t nextPC;
    };

    renameComm renameInfo;

    struct iewComm {
        bool squash;
        bool stall;
        bool predIncorrect;

        // Also eventually include skid buffer space.
        unsigned freeIQEntries;

        uint64_t nextPC;
        // For now hardcode the type.
        // Change this to sequence number eventually.
        InstSeqNum squashedSeqNum;
    };

    iewComm iewInfo;

    struct commitComm {
        bool squash;
        bool stall;
        unsigned freeROBEntries;

        uint64_t nextPC;

        // Think of better names here.
        // Will need to be a variety of sizes...
        // Maybe make it a vector, that way only need one object.
        vector<PhysRegIndex> freeRegs;

        bool robSquashing;
        // Represents the instruction that has either been retired or
        // squashed.  Similar to having a single bus that broadcasts the
        // retired or squashed sequence number.
        InstSeqNum doneSeqNum;
    };

    commitComm commitInfo;
};

#endif //__COMM_HH__
