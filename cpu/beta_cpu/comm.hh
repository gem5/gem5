#ifndef __COMM_HH__
#define __COMM_HH__

#include <stdint.h>
#include <vector>
#include "arch/alpha/isa_traits.hh"
#include "cpu/inst_seq.hh"

using namespace std;

// Find better place to put this typedef.
typedef short int PhysRegIndex;

template<class Impl>
struct SimpleFetchSimpleDecode {
    typedef typename Impl::DynInstPtr DynInstPtr;

    int size;

    DynInstPtr insts[Impl::MaxWidth + 1];
};

template<class Impl>
struct SimpleDecodeSimpleRename {
    typedef typename Impl::DynInstPtr DynInstPtr;

    int size;

    DynInstPtr insts[Impl::MaxWidth + 1];
};

template<class Impl>
struct SimpleRenameSimpleIEW {
    typedef typename Impl::DynInstPtr DynInstPtr;

    int size;

    DynInstPtr insts[Impl::MaxWidth + 1];
};

template<class Impl>
struct SimpleIEWSimpleCommit {
    typedef typename Impl::DynInstPtr DynInstPtr;

    int size;

    DynInstPtr insts[Impl::MaxWidth + 1];
};

template<class Impl>
struct IssueStruct {
    typedef typename Impl::DynInstPtr DynInstPtr;

    int size;

    DynInstPtr insts[Impl::MaxWidth + 1];
};

struct TimeBufStruct {
    struct decodeComm {
        bool squash;
        bool stall;
        bool predIncorrect;
        uint64_t branchAddr;

        bool branchMispredict;
        bool branchTaken;
        uint64_t mispredPC;
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

        // Also eventually include skid buffer space.
        unsigned freeIQEntries;

        bool branchMispredict;
        bool branchTaken;
        uint64_t mispredPC;
        uint64_t nextPC;
        InstSeqNum squashedSeqNum;
    };

    iewComm iewInfo;

    struct commitComm {
        bool squash;
        bool stall;
        unsigned freeROBEntries;

        bool branchMispredict;
        bool branchTaken;
        uint64_t mispredPC;
        uint64_t nextPC;

        // Think of better names here.
        // Will need to be a variety of sizes...
        // Maybe make it a vector, that way only need one object.
        std::vector<PhysRegIndex> freeRegs;

        bool robSquashing;

        // Represents the instruction that has either been retired or
        // squashed.  Similar to having a single bus that broadcasts the
        // retired or squashed sequence number.
        InstSeqNum doneSeqNum;

        // Extra bits of information so that the LDSTQ only updates when it
        // needs to.
        bool commitIsStore;
        bool commitIsLoad;

        // Communication specifically to the IQ to tell the IQ that it can
        // schedule a non-speculative instruction.
        InstSeqNum nonSpecSeqNum;
    };

    commitComm commitInfo;
};

#endif //__COMM_HH__
