/*
 * Copyright (c) 2011, 2016 ARM Limited
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
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
 *
 * Authors: Kevin Lim
 *          Nathanael Premillieu
 */

#ifndef __CPU_O3_COMM_HH__
#define __CPU_O3_COMM_HH__

#include <vector>

#include "arch/types.hh"
#include "base/types.hh"
#include "cpu/inst_seq.hh"
#include "sim/faults.hh"

/** Physical register index type.
 * Although the Impl might be a better for this, but there are a few classes
 * that need this typedef yet are not templated on the Impl.
 */
using PhysRegIndex = short int;

/** Physical register ID.
 * Like a register ID but physical. The inheritance is private because the
 * only relationship between this types is functional, and it is done to
 * prevent code replication. */
class PhysRegId : private RegId {
  private:
    PhysRegIndex flatIdx;

  public:
    explicit PhysRegId() : RegId(IntRegClass, -1), flatIdx(-1) {}

    /** Scalar PhysRegId constructor. */
    explicit PhysRegId(RegClass _regClass, PhysRegIndex _regIdx,
              PhysRegIndex _flatIdx)
        : RegId(_regClass, _regIdx), flatIdx(_flatIdx)
    {}

    /** Vector PhysRegId constructor (w/ elemIndex). */
    explicit PhysRegId(RegClass _regClass, PhysRegIndex _regIdx,
              ElemIndex elem_idx, PhysRegIndex flat_idx)
        : RegId(_regClass, _regIdx, elem_idx), flatIdx(flat_idx) { }

    /** Visible RegId methods */
    /** @{ */
    using RegId::index;
    using RegId::classValue;
    using RegId::isZeroReg;
    using RegId::className;
    using RegId::elemIndex;
     /** @} */
    /**
     * Explicit forward methods, to prevent comparisons of PhysRegId with
     * RegIds.
     */
    /** @{ */
    bool operator<(const PhysRegId& that) const {
        return RegId::operator<(that);
    }

    bool operator==(const PhysRegId& that) const {
        return RegId::operator==(that);
    }

    bool operator!=(const PhysRegId& that) const {
        return RegId::operator!=(that);
    }
    /** @} */

    /** @return true if it is an integer physical register. */
    bool isIntPhysReg() const { return isIntReg(); }

    /** @return true if it is a floating-point physical register. */
    bool isFloatPhysReg() const { return isFloatReg(); }

    /** @Return true if it is a  condition-code physical register. */
    bool isCCPhysReg() const { return isCCReg(); }

    /** @Return true if it is a vector physical register. */
    bool isVectorPhysReg() const { return isVecReg(); }

    /** @Return true if it is a vector element physical register. */
    bool isVectorPhysElem() const { return isVecElem(); }

    /** @Return true if it is a  condition-code physical register. */
    bool isMiscPhysReg() const { return isMiscReg(); }

    /**
     * Returns true if this register is always associated to the same
     * architectural register.
     */
    bool isFixedMapping() const
    {
        return !isRenameable();
    }

    /** Flat index accessor */
    const PhysRegIndex& flatIndex() const { return flatIdx; }

    static PhysRegId elemId(const PhysRegId* vid, ElemIndex elem)
    {
        assert(vid->isVectorPhysReg());
        return PhysRegId(VecElemClass, vid->index(), elem);
    }
};

/** Constant pointer definition.
 * PhysRegIds only need to be created once and then we can just share
 * pointers */
using PhysRegIdPtr = const PhysRegId*;

/** Struct that defines the information passed from fetch to decode. */
template<class Impl>
struct DefaultFetchDefaultDecode {
    typedef typename Impl::DynInstPtr DynInstPtr;

    int size;

    DynInstPtr insts[Impl::MaxWidth];
    Fault fetchFault;
    InstSeqNum fetchFaultSN;
    bool clearFetchFault;
};

/** Struct that defines the information passed from decode to rename. */
template<class Impl>
struct DefaultDecodeDefaultRename {
    typedef typename Impl::DynInstPtr DynInstPtr;

    int size;

    DynInstPtr insts[Impl::MaxWidth];
};

/** Struct that defines the information passed from rename to IEW. */
template<class Impl>
struct DefaultRenameDefaultIEW {
    typedef typename Impl::DynInstPtr DynInstPtr;

    int size;

    DynInstPtr insts[Impl::MaxWidth];
};

/** Struct that defines the information passed from IEW to commit. */
template<class Impl>
struct DefaultIEWDefaultCommit {
    typedef typename Impl::DynInstPtr DynInstPtr;

    int size;

    DynInstPtr insts[Impl::MaxWidth];
    DynInstPtr mispredictInst[Impl::MaxThreads];
    Addr mispredPC[Impl::MaxThreads];
    InstSeqNum squashedSeqNum[Impl::MaxThreads];
    TheISA::PCState pc[Impl::MaxThreads];

    bool squash[Impl::MaxThreads];
    bool branchMispredict[Impl::MaxThreads];
    bool branchTaken[Impl::MaxThreads];
    bool includeSquashInst[Impl::MaxThreads];
};

template<class Impl>
struct IssueStruct {
    typedef typename Impl::DynInstPtr DynInstPtr;

    int size;

    DynInstPtr insts[Impl::MaxWidth];
};

/** Struct that defines all backwards communication. */
template<class Impl>
struct TimeBufStruct {
    typedef typename Impl::DynInstPtr DynInstPtr;
    struct decodeComm {
        TheISA::PCState nextPC;
        DynInstPtr mispredictInst;
        DynInstPtr squashInst;
        InstSeqNum doneSeqNum;
        Addr mispredPC;
        uint64_t branchAddr;
        unsigned branchCount;
        bool squash;
        bool predIncorrect;
        bool branchMispredict;
        bool branchTaken;
    };

    decodeComm decodeInfo[Impl::MaxThreads];

    struct renameComm {
    };

    renameComm renameInfo[Impl::MaxThreads];

    struct iewComm {
        // Also eventually include skid buffer space.
        unsigned freeIQEntries;
        unsigned freeLQEntries;
        unsigned freeSQEntries;
        unsigned dispatchedToLQ;
        unsigned dispatchedToSQ;

        unsigned iqCount;
        unsigned ldstqCount;

        unsigned dispatched;
        bool usedIQ;
        bool usedLSQ;
    };

    iewComm iewInfo[Impl::MaxThreads];

    struct commitComm {
        /////////////////////////////////////////////////////////////////////
        // This code has been re-structured for better packing of variables
        // instead of by stage which is the more logical way to arrange the
        // data.
        // F = Fetch
        // D = Decode
        // I = IEW
        // R = Rename
        // As such each member is annotated with who consumes it
        // e.g. bool variable name // *F,R for Fetch and Rename
        /////////////////////////////////////////////////////////////////////

        /// The pc of the next instruction to execute. This is the next
        /// instruction for a branch mispredict, but the same instruction for
        /// order violation and the like
        TheISA::PCState pc; // *F

        /// Provide fetch the instruction that mispredicted, if this
        /// pointer is not-null a misprediction occured
        DynInstPtr mispredictInst;  // *F

        /// Instruction that caused the a non-mispredict squash
        DynInstPtr squashInst; // *F

        /// Hack for now to send back a strictly ordered access to the
        /// IEW stage.
        DynInstPtr strictlyOrderedLoad; // *I

        /// Communication specifically to the IQ to tell the IQ that it can
        /// schedule a non-speculative instruction.
        InstSeqNum nonSpecSeqNum; // *I

        /// Represents the instruction that has either been retired or
        /// squashed.  Similar to having a single bus that broadcasts the
        /// retired or squashed sequence number.
        InstSeqNum doneSeqNum; // *F, I

        /// Tell Rename how many free entries it has in the ROB
        unsigned freeROBEntries; // *R

        bool squash; // *F, D, R, I
        bool robSquashing; // *F, D, R, I

        /// Rename should re-read number of free rob entries
        bool usedROB; // *R

        /// Notify Rename that the ROB is empty
        bool emptyROB; // *R

        /// Was the branch taken or not
        bool branchTaken; // *F
        /// If an interrupt is pending and fetch should stall
        bool interruptPending; // *F
        /// If the interrupt ended up being cleared before being handled
        bool clearInterrupt; // *F

        /// Hack for now to send back an strictly ordered access to
        /// the IEW stage.
        bool strictlyOrdered; // *I

    };

    commitComm commitInfo[Impl::MaxThreads];

    bool decodeBlock[Impl::MaxThreads];
    bool decodeUnblock[Impl::MaxThreads];
    bool renameBlock[Impl::MaxThreads];
    bool renameUnblock[Impl::MaxThreads];
    bool iewBlock[Impl::MaxThreads];
    bool iewUnblock[Impl::MaxThreads];
};

#endif //__CPU_O3_COMM_HH__
