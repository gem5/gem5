/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __CPU_O3_CPU_COMM_HH__
#define __CPU_O3_CPU_COMM_HH__

#include <vector>

#include "targetarch/isa_traits.hh"
#include "cpu/inst_seq.hh"
#include "sim/host.hh"

// Find better place to put this typedef.
// The impl might be the best place for this.
typedef short int PhysRegIndex;

template<class Impl>
struct SimpleFetchSimpleDecode {
    typedef typename Impl::DynInstPtr DynInstPtr;

    int size;

    DynInstPtr insts[Impl::MaxWidth];
};

template<class Impl>
struct SimpleDecodeSimpleRename {
    typedef typename Impl::DynInstPtr DynInstPtr;

    int size;

    DynInstPtr insts[Impl::MaxWidth];
};

template<class Impl>
struct SimpleRenameSimpleIEW {
    typedef typename Impl::DynInstPtr DynInstPtr;

    int size;

    DynInstPtr insts[Impl::MaxWidth];
};

template<class Impl>
struct SimpleIEWSimpleCommit {
    typedef typename Impl::DynInstPtr DynInstPtr;

    int size;

    DynInstPtr insts[Impl::MaxWidth];

    bool squash;
    bool branchMispredict;
    bool branchTaken;
    uint64_t mispredPC;
    uint64_t nextPC;
    InstSeqNum squashedSeqNum;
};

template<class Impl>
struct IssueStruct {
    typedef typename Impl::DynInstPtr DynInstPtr;

    int size;

    DynInstPtr insts[Impl::MaxWidth];
};

struct TimeBufStruct {
    struct decodeComm {
        bool squash;
        bool stall;
        bool predIncorrect;
        uint64_t branchAddr;

        InstSeqNum doneSeqNum;

        // Might want to package this kind of branch stuff into a single
        // struct as it is used pretty frequently.
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
        bool stall;

        // Also eventually include skid buffer space.
        unsigned freeIQEntries;
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

        bool robSquashing;

        // Represents the instruction that has either been retired or
        // squashed.  Similar to having a single bus that broadcasts the
        // retired or squashed sequence number.
        InstSeqNum doneSeqNum;

        // Extra bit of information so that the LDSTQ only updates when it
        // needs to.
        bool commitIsLoad;

        // Communication specifically to the IQ to tell the IQ that it can
        // schedule a non-speculative instruction.
        InstSeqNum nonSpecSeqNum;
    };

    commitComm commitInfo;
};

#endif //__CPU_O3_CPU_COMM_HH__
