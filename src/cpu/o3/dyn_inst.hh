/*
 * Copyright (c) 2010, 2016 ARM Limited
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
 */

#ifndef __CPU_O3_DYN_INST_HH__
#define __CPU_O3_DYN_INST_HH__

#include <algorithm>
#include <array>
#include <deque>
#include <list>
#include <string>

#include "base/refcnt.hh"
#include "base/trace.hh"
#include "config/the_isa.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/exec_context.hh"
#include "cpu/exetrace.hh"
#include "cpu/inst_res.hh"
#include "cpu/inst_seq.hh"
#include "cpu/o3/cpu.hh"
#include "cpu/o3/dyn_inst_ptr.hh"
#include "cpu/o3/lsq_unit.hh"
#include "cpu/op_class.hh"
#include "cpu/reg_class.hh"
#include "cpu/static_inst.hh"
#include "cpu/translation.hh"
#include "debug/HtmCpu.hh"

namespace gem5
{

class Packet;

namespace o3
{

class DynInst : public ExecContext, public RefCounted
{
  public:
    // The list of instructions iterator type.
    typedef typename std::list<DynInstPtr>::iterator ListIt;

    /** BaseDynInst constructor given a binary instruction. */
    DynInst(const StaticInstPtr &staticInst, const StaticInstPtr
            &macroop, TheISA::PCState pc, TheISA::PCState predPC,
            InstSeqNum seq_num, CPU *cpu);

    /** BaseDynInst constructor given a static inst pointer. */
    DynInst(const StaticInstPtr &_staticInst, const StaticInstPtr &_macroop);

    ~DynInst();

    /** Executes the instruction.*/
    Fault execute();

    /** Initiates the access.  Only valid for memory operations. */
    Fault initiateAcc();

    /** Completes the access.  Only valid for memory operations. */
    Fault completeAcc(PacketPtr pkt);

    /** The sequence number of the instruction. */
    InstSeqNum seqNum = 0;

    /** The StaticInst used by this BaseDynInst. */
    const StaticInstPtr staticInst;

    /** Pointer to the Impl's CPU object. */
    CPU *cpu = nullptr;

    BaseCPU *getCpuPtr() { return cpu; }

    /** Pointer to the thread state. */
    ThreadState *thread = nullptr;

    /** The kind of fault this instruction has generated. */
    Fault fault = NoFault;

    /** InstRecord that tracks this instructions. */
    Trace::InstRecord *traceData = nullptr;

  protected:
    enum Status
    {
        IqEntry,                 /// Instruction is in the IQ
        RobEntry,                /// Instruction is in the ROB
        LsqEntry,                /// Instruction is in the LSQ
        Completed,               /// Instruction has completed
        ResultReady,             /// Instruction has its result
        CanIssue,                /// Instruction can issue and execute
        Issued,                  /// Instruction has issued
        Executed,                /// Instruction has executed
        CanCommit,               /// Instruction can commit
        AtCommit,                /// Instruction has reached commit
        Committed,               /// Instruction has committed
        Squashed,                /// Instruction is squashed
        SquashedInIQ,            /// Instruction is squashed in the IQ
        SquashedInLSQ,           /// Instruction is squashed in the LSQ
        SquashedInROB,           /// Instruction is squashed in the ROB
        PinnedRegsRenamed,       /// Pinned registers are renamed
        PinnedRegsWritten,       /// Pinned registers are written back
        PinnedRegsSquashDone,    /// Regs pinning status updated after squash
        RecoverInst,             /// Is a recover instruction
        BlockingInst,            /// Is a blocking instruction
        ThreadsyncWait,          /// Is a thread synchronization instruction
        SerializeBefore,         /// Needs to serialize on
                                 /// instructions ahead of it
        SerializeAfter,          /// Needs to serialize instructions behind it
        SerializeHandled,        /// Serialization has been handled
        NumStatus
    };

    enum Flags
    {
        NotAnInst,
        TranslationStarted,
        TranslationCompleted,
        PossibleLoadViolation,
        HitExternalSnoop,
        EffAddrValid,
        RecordResult,
        Predicate,
        MemAccPredicate,
        PredTaken,
        IsStrictlyOrdered,
        ReqMade,
        MemOpDone,
        HtmFromTransaction,
        MaxFlags
    };

  private:
    /* An amalgamation of a lot of boolean values into one */
    std::bitset<MaxFlags> instFlags;

    /** The status of this BaseDynInst.  Several bits can be set. */
    std::bitset<NumStatus> status;

  protected:
    /** The result of the instruction; assumes an instruction can have many
     *  destination registers.
     */
    std::queue<InstResult> instResult;

    /** PC state for this instruction. */
    TheISA::PCState pc;

    /** Values to be written to the destination misc. registers. */
    std::vector<RegVal> _destMiscRegVal;

    /** Indexes of the destination misc. registers. They are needed to defer
     * the write accesses to the misc. registers until the commit stage, when
     * the instruction is out of its speculative state.
     */
    std::vector<short> _destMiscRegIdx;

    /**
     * Collect register related information into a single struct. The number of
     * source and destination registers can vary, and storage for information
     * about them needs to be allocated dynamically. This class figures out
     * how much space is needed and allocates it all at once, and then
     * trivially divies it up for each type of per-register array.
     */
    struct Regs
    {
      private:
        size_t _numSrcs;
        size_t _numDests;

        using BackingStorePtr = std::unique_ptr<uint8_t[]>;
        using BufCursor = BackingStorePtr::pointer;

        BackingStorePtr buf;

        // Members should be ordered based on required alignment so that they
        // can be allocated contiguously.

        // Flattened register index of the destination registers of this
        // instruction.
        RegId *_flatDestIdx;

        // Physical register index of the destination registers of this
        // instruction.
        PhysRegIdPtr *_destIdx;

        // Physical register index of the previous producers of the
        // architected destinations.
        PhysRegIdPtr *_prevDestIdx;

        static inline size_t
        bytesForDests(size_t num)
        {
            return (sizeof(RegId) + 2 * sizeof(PhysRegIdPtr)) * num;
        }

        // Physical register index of the source registers of this instruction.
        PhysRegIdPtr *_srcIdx;

        // Whether or not the source register is ready, one bit per register.
        uint8_t *_readySrcIdx;

        static inline size_t
        bytesForSources(size_t num)
        {
            return sizeof(PhysRegIdPtr) * num +
                sizeof(uint8_t) * ((num + 7) / 8);
        }

        template <class T>
        static inline void
        allocate(T *&ptr, BufCursor &cur, size_t count)
        {
            ptr = new (cur) T[count];
            cur += sizeof(T) * count;
        }

      public:
        size_t numSrcs() const { return _numSrcs; }
        size_t numDests() const { return _numDests; }

        void
        init()
        {
            std::fill(_readySrcIdx, _readySrcIdx + (numSrcs() + 7) / 8, 0);
        }

        Regs(size_t srcs, size_t dests) : _numSrcs(srcs), _numDests(dests),
            buf(new uint8_t[bytesForSources(srcs) + bytesForDests(dests)])
        {
            BufCursor cur = buf.get();
            allocate(_flatDestIdx, cur, dests);
            allocate(_destIdx, cur, dests);
            allocate(_prevDestIdx, cur, dests);
            allocate(_srcIdx, cur, srcs);
            allocate(_readySrcIdx, cur, (srcs + 7) / 8);

            init();
        }

        // Returns the flattened register index of the idx'th destination
        // register.
        const RegId &
        flattenedDestIdx(int idx) const
        {
            return _flatDestIdx[idx];
        }

        // Flattens a destination architectural register index into a logical
        // index.
        void
        flattenedDestIdx(int idx, const RegId &reg_id)
        {
            _flatDestIdx[idx] = reg_id;
        }

        // Returns the physical register index of the idx'th destination
        // register.
        PhysRegIdPtr
        renamedDestIdx(int idx) const
        {
            return _destIdx[idx];
        }

        // Set the renamed dest register id.
        void
        renamedDestIdx(int idx, PhysRegIdPtr phys_reg_id)
        {
            _destIdx[idx] = phys_reg_id;
        }

        // Returns the physical register index of the previous physical
        // register that remapped to the same logical register index.
        PhysRegIdPtr
        prevDestIdx(int idx) const
        {
            return _prevDestIdx[idx];
        }

        // Set the previous renamed dest register id.
        void
        prevDestIdx(int idx, PhysRegIdPtr phys_reg_id)
        {
            _prevDestIdx[idx] = phys_reg_id;
        }

        // Returns the physical register index of the i'th source register.
        PhysRegIdPtr
        renamedSrcIdx(int idx) const
        {
            return _srcIdx[idx];
        }

        void
        renamedSrcIdx(int idx, PhysRegIdPtr phys_reg_id)
        {
            _srcIdx[idx] = phys_reg_id;
        }

        bool
        readySrcIdx(int idx) const
        {
            uint8_t &byte = _readySrcIdx[idx / 8];
            return bits(byte, idx % 8);
        }

        void
        readySrcIdx(int idx, bool ready)
        {
            uint8_t &byte = _readySrcIdx[idx / 8];
            replaceBits(byte, idx % 8, ready ? 1 : 0);
        }
    };

  public:
    Regs regs;

    /** The thread this instruction is from. */
    ThreadID threadNumber = 0;

    /** Iterator pointing to this BaseDynInst in the list of all insts. */
    ListIt instListIt;

    ////////////////////// Branch Data ///////////////
    /** Predicted PC state after this instruction. */
    TheISA::PCState predPC;

    /** The Macroop if one exists */
    const StaticInstPtr macroop;

    /** How many source registers are ready. */
    uint8_t readyRegs = 0;

  public:
    /////////////////////// Load Store Data //////////////////////
    /** The effective virtual address (lds & stores only). */
    Addr effAddr = 0;

    /** The effective physical address. */
    Addr physEffAddr = 0;

    /** The memory request flags (from translation). */
    unsigned memReqFlags = 0;

    /** The size of the request */
    unsigned effSize;

    /** Pointer to the data for the memory access. */
    uint8_t *memData = nullptr;

    /** Load queue index. */
    ssize_t lqIdx = -1;
    typename LSQUnit::LQIterator lqIt;

    /** Store queue index. */
    ssize_t sqIdx = -1;
    typename LSQUnit::SQIterator sqIt;


    /////////////////////// TLB Miss //////////////////////
    /**
     * Saved memory request (needed when the DTB address translation is
     * delayed due to a hw page table walk).
     */
    LSQ::LSQRequest *savedReq;

    /////////////////////// Checker //////////////////////
    // Need a copy of main request pointer to verify on writes.
    RequestPtr reqToVerify;

  public:
    /** Records changes to result? */
    void recordResult(bool f) { instFlags[RecordResult] = f; }

    /** Is the effective virtual address valid. */
    bool effAddrValid() const { return instFlags[EffAddrValid]; }
    void effAddrValid(bool b) { instFlags[EffAddrValid] = b; }

    /** Whether or not the memory operation is done. */
    bool memOpDone() const { return instFlags[MemOpDone]; }
    void memOpDone(bool f) { instFlags[MemOpDone] = f; }

    bool notAnInst() const { return instFlags[NotAnInst]; }
    void setNotAnInst() { instFlags[NotAnInst] = true; }


    ////////////////////////////////////////////
    //
    // INSTRUCTION EXECUTION
    //
    ////////////////////////////////////////////

    void
    demapPage(Addr vaddr, uint64_t asn) override
    {
        cpu->demapPage(vaddr, asn);
    }

    Fault initiateMemRead(Addr addr, unsigned size, Request::Flags flags,
            const std::vector<bool> &byte_enable) override;

    Fault initiateHtmCmd(Request::Flags flags) override;

    Fault writeMem(uint8_t *data, unsigned size, Addr addr,
                   Request::Flags flags, uint64_t *res,
                   const std::vector<bool> &byte_enable) override;

    Fault initiateMemAMO(Addr addr, unsigned size, Request::Flags flags,
                         AtomicOpFunctorPtr amo_op) override;

    /** True if the DTB address translation has started. */
    bool translationStarted() const { return instFlags[TranslationStarted]; }
    void translationStarted(bool f) { instFlags[TranslationStarted] = f; }

    /** True if the DTB address translation has completed. */
    bool
    translationCompleted() const
    {
        return instFlags[TranslationCompleted];
    }
    void translationCompleted(bool f) { instFlags[TranslationCompleted] = f; }

    /** True if this address was found to match a previous load and they issued
     * out of order. If that happend, then it's only a problem if an incoming
     * snoop invalidate modifies the line, in which case we need to squash.
     * If nothing modified the line the order doesn't matter.
     */
    bool
    possibleLoadViolation() const
    {
        return instFlags[PossibleLoadViolation];
    }
    void
    possibleLoadViolation(bool f)
    {
        instFlags[PossibleLoadViolation] = f;
    }

    /** True if the address hit a external snoop while sitting in the LSQ.
     * If this is true and a older instruction sees it, this instruction must
     * reexecute
     */
    bool hitExternalSnoop() const { return instFlags[HitExternalSnoop]; }
    void hitExternalSnoop(bool f) { instFlags[HitExternalSnoop] = f; }

    /**
     * Returns true if the DTB address translation is being delayed due to a hw
     * page table walk.
     */
    bool
    isTranslationDelayed() const
    {
        return (translationStarted() && !translationCompleted());
    }

  public:
#ifdef DEBUG
    void dumpSNList();
#endif

    /** Renames a destination register to a physical register.  Also records
     *  the previous physical register that the logical register mapped to.
     */
    void
    renameDestReg(int idx, PhysRegIdPtr renamed_dest,
                  PhysRegIdPtr previous_rename)
    {
        regs.renamedDestIdx(idx, renamed_dest);
        regs.prevDestIdx(idx, previous_rename);
        if (renamed_dest->isPinned())
            setPinnedRegsRenamed();
    }

    /** Renames a source logical register to the physical register which
     *  has/will produce that logical register's result.
     *  @todo: add in whether or not the source register is ready.
     */
    void
    renameSrcReg(int idx, PhysRegIdPtr renamed_src)
    {
        regs.renamedSrcIdx(idx, renamed_src);
    }

    /** Dumps out contents of this BaseDynInst. */
    void dump();

    /** Dumps out contents of this BaseDynInst into given string. */
    void dump(std::string &outstring);

    /** Read this CPU's ID. */
    int cpuId() const { return cpu->cpuId(); }

    /** Read this CPU's Socket ID. */
    uint32_t socketId() const { return cpu->socketId(); }

    /** Read this CPU's data requestor ID */
    RequestorID requestorId() const { return cpu->dataRequestorId(); }

    /** Read this context's system-wide ID **/
    ContextID contextId() const { return thread->contextId(); }

    /** Returns the fault type. */
    Fault getFault() const { return fault; }
    /** TODO: This I added for the LSQRequest side to be able to modify the
     * fault. There should be a better mechanism in place. */
    Fault& getFault() { return fault; }

    /** Checks whether or not this instruction has had its branch target
     *  calculated yet.  For now it is not utilized and is hacked to be
     *  always false.
     *  @todo: Actually use this instruction.
     */
    bool doneTargCalc() { return false; }

    /** Set the predicted target of this current instruction. */
    void setPredTarg(const TheISA::PCState &_predPC) { predPC = _predPC; }

    const TheISA::PCState &readPredTarg() { return predPC; }

    /** Returns the predicted PC immediately after the branch. */
    Addr predInstAddr() { return predPC.instAddr(); }

    /** Returns the predicted PC two instructions after the branch */
    Addr predNextInstAddr() { return predPC.nextInstAddr(); }

    /** Returns the predicted micro PC after the branch */
    Addr predMicroPC() { return predPC.microPC(); }

    /** Returns whether the instruction was predicted taken or not. */
    bool readPredTaken() { return instFlags[PredTaken]; }

    void
    setPredTaken(bool predicted_taken)
    {
        instFlags[PredTaken] = predicted_taken;
    }

    /** Returns whether the instruction mispredicted. */
    bool
    mispredicted()
    {
        TheISA::PCState tempPC = pc;
        staticInst->advancePC(tempPC);
        return !(tempPC == predPC);
    }

    //
    //  Instruction types.  Forward checks to StaticInst object.
    //
    bool isNop()          const { return staticInst->isNop(); }
    bool isMemRef()       const { return staticInst->isMemRef(); }
    bool isLoad()         const { return staticInst->isLoad(); }
    bool isStore()        const { return staticInst->isStore(); }
    bool isAtomic()       const { return staticInst->isAtomic(); }
    bool isStoreConditional() const
    { return staticInst->isStoreConditional(); }
    bool isInstPrefetch() const { return staticInst->isInstPrefetch(); }
    bool isDataPrefetch() const { return staticInst->isDataPrefetch(); }
    bool isInteger()      const { return staticInst->isInteger(); }
    bool isFloating()     const { return staticInst->isFloating(); }
    bool isVector()       const { return staticInst->isVector(); }
    bool isControl()      const { return staticInst->isControl(); }
    bool isCall()         const { return staticInst->isCall(); }
    bool isReturn()       const { return staticInst->isReturn(); }
    bool isDirectCtrl()   const { return staticInst->isDirectCtrl(); }
    bool isIndirectCtrl() const { return staticInst->isIndirectCtrl(); }
    bool isCondCtrl()     const { return staticInst->isCondCtrl(); }
    bool isUncondCtrl()   const { return staticInst->isUncondCtrl(); }
    bool isSerializing()  const { return staticInst->isSerializing(); }
    bool
    isSerializeBefore() const
    {
        return staticInst->isSerializeBefore() || status[SerializeBefore];
    }
    bool
    isSerializeAfter() const
    {
        return staticInst->isSerializeAfter() || status[SerializeAfter];
    }
    bool isSquashAfter() const { return staticInst->isSquashAfter(); }
    bool isFullMemBarrier()   const { return staticInst->isFullMemBarrier(); }
    bool isReadBarrier() const { return staticInst->isReadBarrier(); }
    bool isWriteBarrier() const { return staticInst->isWriteBarrier(); }
    bool isNonSpeculative() const { return staticInst->isNonSpeculative(); }
    bool isQuiesce() const { return staticInst->isQuiesce(); }
    bool isUnverifiable() const { return staticInst->isUnverifiable(); }
    bool isSyscall() const { return staticInst->isSyscall(); }
    bool isMacroop() const { return staticInst->isMacroop(); }
    bool isMicroop() const { return staticInst->isMicroop(); }
    bool isDelayedCommit() const { return staticInst->isDelayedCommit(); }
    bool isLastMicroop() const { return staticInst->isLastMicroop(); }
    bool isFirstMicroop() const { return staticInst->isFirstMicroop(); }
    // hardware transactional memory
    bool isHtmStart() const { return staticInst->isHtmStart(); }
    bool isHtmStop() const { return staticInst->isHtmStop(); }
    bool isHtmCancel() const { return staticInst->isHtmCancel(); }
    bool isHtmCmd() const { return staticInst->isHtmCmd(); }

    uint64_t
    getHtmTransactionUid() const override
    {
        assert(instFlags[HtmFromTransaction]);
        return this->htmUid;
    }

    uint64_t
    newHtmTransactionUid() const override
    {
        panic("Not yet implemented\n");
        return 0;
    }

    bool
    inHtmTransactionalState() const override
    {
        return instFlags[HtmFromTransaction];
    }

    uint64_t
    getHtmTransactionalDepth() const override
    {
        if (inHtmTransactionalState())
            return this->htmDepth;
        else
            return 0;
    }

    void
    setHtmTransactionalState(uint64_t htm_uid, uint64_t htm_depth)
    {
        instFlags.set(HtmFromTransaction);
        htmUid = htm_uid;
        htmDepth = htm_depth;
    }

    void
    clearHtmTransactionalState()
    {
        if (inHtmTransactionalState()) {
            DPRINTF(HtmCpu,
                "clearing instuction's transactional state htmUid=%u\n",
                getHtmTransactionUid());

            instFlags.reset(HtmFromTransaction);
            htmUid = -1;
            htmDepth = 0;
        }
    }

    /** Temporarily sets this instruction as a serialize before instruction. */
    void setSerializeBefore() { status.set(SerializeBefore); }

    /** Clears the serializeBefore part of this instruction. */
    void clearSerializeBefore() { status.reset(SerializeBefore); }

    /** Checks if this serializeBefore is only temporarily set. */
    bool isTempSerializeBefore() { return status[SerializeBefore]; }

    /** Temporarily sets this instruction as a serialize after instruction. */
    void setSerializeAfter() { status.set(SerializeAfter); }

    /** Clears the serializeAfter part of this instruction.*/
    void clearSerializeAfter() { status.reset(SerializeAfter); }

    /** Checks if this serializeAfter is only temporarily set. */
    bool isTempSerializeAfter() { return status[SerializeAfter]; }

    /** Sets the serialization part of this instruction as handled. */
    void setSerializeHandled() { status.set(SerializeHandled); }

    /** Checks if the serialization part of this instruction has been
     *  handled.  This does not apply to the temporary serializing
     *  state; it only applies to this instruction's own permanent
     *  serializing state.
     */
    bool isSerializeHandled() { return status[SerializeHandled]; }

    /** Returns the opclass of this instruction. */
    OpClass opClass() const { return staticInst->opClass(); }

    /** Returns the branch target address. */
    TheISA::PCState
    branchTarget() const
    {
        return staticInst->branchTarget(pc);
    }

    /** Returns the number of source registers. */
    size_t numSrcRegs() const { return regs.numSrcs(); }

    /** Returns the number of destination registers. */
    size_t numDestRegs() const { return regs.numDests(); }

    // the following are used to track physical register usage
    // for machines with separate int & FP reg files
    int8_t numFPDestRegs()  const { return staticInst->numFPDestRegs(); }
    int8_t numIntDestRegs() const { return staticInst->numIntDestRegs(); }
    int8_t numCCDestRegs() const { return staticInst->numCCDestRegs(); }
    int8_t numVecDestRegs() const { return staticInst->numVecDestRegs(); }
    int8_t
    numVecElemDestRegs() const
    {
        return staticInst->numVecElemDestRegs();
    }
    int8_t
    numVecPredDestRegs() const
    {
        return staticInst->numVecPredDestRegs();
    }

    /** Returns the logical register index of the i'th destination register. */
    const RegId& destRegIdx(int i) const { return staticInst->destRegIdx(i); }

    /** Returns the logical register index of the i'th source register. */
    const RegId& srcRegIdx(int i) const { return staticInst->srcRegIdx(i); }

    /** Return the size of the instResult queue. */
    uint8_t resultSize() { return instResult.size(); }

    /** Pops a result off the instResult queue.
     * If the result stack is empty, return the default value.
     * */
    InstResult
    popResult(InstResult dflt=InstResult())
    {
        if (!instResult.empty()) {
            InstResult t = instResult.front();
            instResult.pop();
            return t;
        }
        return dflt;
    }

    /** Pushes a result onto the instResult queue. */
    /** @{ */
    /** Scalar result. */
    template<typename T>
    void
    setScalarResult(T &&t)
    {
        if (instFlags[RecordResult]) {
            instResult.push(InstResult(std::forward<T>(t),
                        InstResult::ResultType::Scalar));
        }
    }

    /** Full vector result. */
    template<typename T>
    void
    setVecResult(T &&t)
    {
        if (instFlags[RecordResult]) {
            instResult.push(InstResult(std::forward<T>(t),
                        InstResult::ResultType::VecReg));
        }
    }

    /** Vector element result. */
    template<typename T>
    void
    setVecElemResult(T &&t)
    {
        if (instFlags[RecordResult]) {
            instResult.push(InstResult(std::forward<T>(t),
                        InstResult::ResultType::VecElem));
        }
    }

    /** Predicate result. */
    template<typename T>
    void
    setVecPredResult(T &&t)
    {
        if (instFlags[RecordResult]) {
            instResult.push(InstResult(std::forward<T>(t),
                            InstResult::ResultType::VecPredReg));
        }
    }
    /** @} */

    /** Records that one of the source registers is ready. */
    void markSrcRegReady();

    /** Marks a specific register as ready. */
    void markSrcRegReady(RegIndex src_idx);

    /** Sets this instruction as completed. */
    void setCompleted() { status.set(Completed); }

    /** Returns whether or not this instruction is completed. */
    bool isCompleted() const { return status[Completed]; }

    /** Marks the result as ready. */
    void setResultReady() { status.set(ResultReady); }

    /** Returns whether or not the result is ready. */
    bool isResultReady() const { return status[ResultReady]; }

    /** Sets this instruction as ready to issue. */
    void setCanIssue() { status.set(CanIssue); }

    /** Returns whether or not this instruction is ready to issue. */
    bool readyToIssue() const { return status[CanIssue]; }

    /** Clears this instruction being able to issue. */
    void clearCanIssue() { status.reset(CanIssue); }

    /** Sets this instruction as issued from the IQ. */
    void setIssued() { status.set(Issued); }

    /** Returns whether or not this instruction has issued. */
    bool isIssued() const { return status[Issued]; }

    /** Clears this instruction as being issued. */
    void clearIssued() { status.reset(Issued); }

    /** Sets this instruction as executed. */
    void setExecuted() { status.set(Executed); }

    /** Returns whether or not this instruction has executed. */
    bool isExecuted() const { return status[Executed]; }

    /** Sets this instruction as ready to commit. */
    void setCanCommit() { status.set(CanCommit); }

    /** Clears this instruction as being ready to commit. */
    void clearCanCommit() { status.reset(CanCommit); }

    /** Returns whether or not this instruction is ready to commit. */
    bool readyToCommit() const { return status[CanCommit]; }

    void setAtCommit() { status.set(AtCommit); }

    bool isAtCommit() { return status[AtCommit]; }

    /** Sets this instruction as committed. */
    void setCommitted() { status.set(Committed); }

    /** Returns whether or not this instruction is committed. */
    bool isCommitted() const { return status[Committed]; }

    /** Sets this instruction as squashed. */
    void setSquashed();

    /** Returns whether or not this instruction is squashed. */
    bool isSquashed() const { return status[Squashed]; }

    //Instruction Queue Entry
    //-----------------------
    /** Sets this instruction as a entry the IQ. */
    void setInIQ() { status.set(IqEntry); }

    /** Sets this instruction as a entry the IQ. */
    void clearInIQ() { status.reset(IqEntry); }

    /** Returns whether or not this instruction has issued. */
    bool isInIQ() const { return status[IqEntry]; }

    /** Sets this instruction as squashed in the IQ. */
    void setSquashedInIQ() { status.set(SquashedInIQ); status.set(Squashed);}

    /** Returns whether or not this instruction is squashed in the IQ. */
    bool isSquashedInIQ() const { return status[SquashedInIQ]; }


    //Load / Store Queue Functions
    //-----------------------
    /** Sets this instruction as a entry the LSQ. */
    void setInLSQ() { status.set(LsqEntry); }

    /** Sets this instruction as a entry the LSQ. */
    void removeInLSQ() { status.reset(LsqEntry); }

    /** Returns whether or not this instruction is in the LSQ. */
    bool isInLSQ() const { return status[LsqEntry]; }

    /** Sets this instruction as squashed in the LSQ. */
    void setSquashedInLSQ() { status.set(SquashedInLSQ); status.set(Squashed);}

    /** Returns whether or not this instruction is squashed in the LSQ. */
    bool isSquashedInLSQ() const { return status[SquashedInLSQ]; }


    //Reorder Buffer Functions
    //-----------------------
    /** Sets this instruction as a entry the ROB. */
    void setInROB() { status.set(RobEntry); }

    /** Sets this instruction as a entry the ROB. */
    void clearInROB() { status.reset(RobEntry); }

    /** Returns whether or not this instruction is in the ROB. */
    bool isInROB() const { return status[RobEntry]; }

    /** Sets this instruction as squashed in the ROB. */
    void setSquashedInROB() { status.set(SquashedInROB); }

    /** Returns whether or not this instruction is squashed in the ROB. */
    bool isSquashedInROB() const { return status[SquashedInROB]; }

    /** Returns whether pinned registers are renamed */
    bool isPinnedRegsRenamed() const { return status[PinnedRegsRenamed]; }

    /** Sets the destination registers as renamed */
    void
    setPinnedRegsRenamed()
    {
        assert(!status[PinnedRegsSquashDone]);
        assert(!status[PinnedRegsWritten]);
        status.set(PinnedRegsRenamed);
    }

    /** Returns whether destination registers are written */
    bool isPinnedRegsWritten() const { return status[PinnedRegsWritten]; }

    /** Sets destination registers as written */
    void
    setPinnedRegsWritten()
    {
        assert(!status[PinnedRegsSquashDone]);
        assert(status[PinnedRegsRenamed]);
        status.set(PinnedRegsWritten);
    }

    /** Return whether dest registers' pinning status updated after squash */
    bool
    isPinnedRegsSquashDone() const
    {
        return status[PinnedRegsSquashDone];
    }

    /** Sets dest registers' status updated after squash */
    void
    setPinnedRegsSquashDone()
    {
        assert(!status[PinnedRegsSquashDone]);
        status.set(PinnedRegsSquashDone);
    }

    /** Read the PC state of this instruction. */
    TheISA::PCState pcState() const override { return pc; }

    /** Set the PC state of this instruction. */
    void pcState(const TheISA::PCState &val) override { pc = val; }

    /** Read the PC of this instruction. */
    Addr instAddr() const { return pc.instAddr(); }

    /** Read the PC of the next instruction. */
    Addr nextInstAddr() const { return pc.nextInstAddr(); }

    /**Read the micro PC of this instruction. */
    Addr microPC() const { return pc.microPC(); }

    bool readPredicate() const override { return instFlags[Predicate]; }

    void
    setPredicate(bool val) override
    {
        instFlags[Predicate] = val;

        if (traceData) {
            traceData->setPredicate(val);
        }
    }

    bool
    readMemAccPredicate() const override
    {
        return instFlags[MemAccPredicate];
    }

    void
    setMemAccPredicate(bool val) override
    {
        instFlags[MemAccPredicate] = val;
    }

    /** Sets the thread id. */
    void setTid(ThreadID tid) { threadNumber = tid; }

    /** Sets the pointer to the thread state. */
    void setThreadState(ThreadState *state) { thread = state; }

    /** Returns the thread context. */
    gem5::ThreadContext *tcBase() const override { return thread->getTC(); }

  public:
    /** Is this instruction's memory access strictly ordered? */
    bool strictlyOrdered() const { return instFlags[IsStrictlyOrdered]; }
    void strictlyOrdered(bool so) { instFlags[IsStrictlyOrdered] = so; }

    /** Has this instruction generated a memory request. */
    bool hasRequest() const { return instFlags[ReqMade]; }
    /** Assert this instruction has generated a memory request. */
    void setRequest() { instFlags[ReqMade] = true; }

    /** Returns iterator to this instruction in the list of all insts. */
    ListIt &getInstListIt() { return instListIt; }

    /** Sets iterator for this instruction in the list of all insts. */
    void setInstListIt(ListIt _instListIt) { instListIt = _instListIt; }

  public:
    /** Returns the number of consecutive store conditional failures. */
    unsigned int
    readStCondFailures() const override
    {
        return thread->storeCondFailures;
    }

    /** Sets the number of consecutive store conditional failures. */
    void
    setStCondFailures(unsigned int sc_failures) override
    {
        thread->storeCondFailures = sc_failures;
    }

  public:
    // monitor/mwait funtions
    void
    armMonitor(Addr address) override
    {
        cpu->armMonitor(threadNumber, address);
    }
    bool
    mwait(PacketPtr pkt) override
    {
        return cpu->mwait(threadNumber, pkt);
    }
    void
    mwaitAtomic(gem5::ThreadContext *tc) override
    {
        return cpu->mwaitAtomic(threadNumber, tc, cpu->mmu);
    }
    AddressMonitor *
    getAddrMonitor() override
    {
        return cpu->getCpuAddrMonitor(threadNumber);
    }

  private:
    // hardware transactional memory
    uint64_t htmUid = -1;
    uint64_t htmDepth = 0;

  public:
#if TRACING_ON
    // Value -1 indicates that particular phase
    // hasn't happened (yet).
    /** Tick records used for the pipeline activity viewer. */
    Tick fetchTick = -1;      // instruction fetch is completed.
    int32_t decodeTick = -1;  // instruction enters decode phase
    int32_t renameTick = -1;  // instruction enters rename phase
    int32_t dispatchTick = -1;
    int32_t issueTick = -1;
    int32_t completeTick = -1;
    int32_t commitTick = -1;
    int32_t storeTick = -1;
#endif

    /* Values used by LoadToUse stat */
    Tick firstIssue = -1;
    Tick lastWakeDependents = -1;

    /** Reads a misc. register, including any side-effects the read
     * might have as defined by the architecture.
     */
    RegVal
    readMiscReg(int misc_reg) override
    {
        return this->cpu->readMiscReg(misc_reg, this->threadNumber);
    }

    /** Sets a misc. register, including any side-effects the write
     * might have as defined by the architecture.
     */
    void
    setMiscReg(int misc_reg, RegVal val) override
    {
        /** Writes to misc. registers are recorded and deferred until the
         * commit stage, when updateMiscRegs() is called. First, check if
         * the misc reg has been written before and update its value to be
         * committed instead of making a new entry. If not, make a new
         * entry and record the write.
         */
        for (auto &idx: _destMiscRegIdx) {
            if (idx == misc_reg)
                return;
        }

        _destMiscRegIdx.push_back(misc_reg);
        _destMiscRegVal.push_back(val);
    }

    /** Reads a misc. register, including any side-effects the read
     * might have as defined by the architecture.
     */
    RegVal
    readMiscRegOperand(const StaticInst *si, int idx) override
    {
        const RegId& reg = si->srcRegIdx(idx);
        assert(reg.is(MiscRegClass));
        return this->cpu->readMiscReg(reg.index(), this->threadNumber);
    }

    /** Sets a misc. register, including any side-effects the write
     * might have as defined by the architecture.
     */
    void
    setMiscRegOperand(const StaticInst *si, int idx, RegVal val) override
    {
        const RegId& reg = si->destRegIdx(idx);
        assert(reg.is(MiscRegClass));
        setMiscReg(reg.index(), val);
    }

    /** Called at the commit stage to update the misc. registers. */
    void
    updateMiscRegs()
    {
        // @todo: Pretty convoluted way to avoid squashing from happening when
        // using the TC during an instruction's execution (specifically for
        // instructions that have side-effects that use the TC).  Fix this.
        // See cpu/o3/dyn_inst_impl.hh.
        bool no_squash_from_TC = this->thread->noSquashFromTC;
        this->thread->noSquashFromTC = true;

        for (int i = 0; i < _destMiscRegIdx.size(); i++)
            this->cpu->setMiscReg(
                _destMiscRegIdx[i], _destMiscRegVal[i], this->threadNumber);

        this->thread->noSquashFromTC = no_squash_from_TC;
    }

    void
    forwardOldRegs()
    {

        for (int idx = 0; idx < this->numDestRegs(); idx++) {
            PhysRegIdPtr prev_phys_reg = this->regs.prevDestIdx(idx);
            const RegId& original_dest_reg = this->staticInst->destRegIdx(idx);
            switch (original_dest_reg.classValue()) {
              case IntRegClass:
                this->setIntRegOperand(this->staticInst.get(), idx,
                        this->cpu->readIntReg(prev_phys_reg));
                break;
              case FloatRegClass:
                this->setFloatRegOperandBits(this->staticInst.get(), idx,
                        this->cpu->readFloatReg(prev_phys_reg));
                break;
              case VecRegClass:
                this->setVecRegOperand(this->staticInst.get(), idx,
                        this->cpu->readVecReg(prev_phys_reg));
                break;
              case VecElemClass:
                this->setVecElemOperand(this->staticInst.get(), idx,
                        this->cpu->readVecElem(prev_phys_reg));
                break;
              case VecPredRegClass:
                this->setVecPredRegOperand(this->staticInst.get(), idx,
                        this->cpu->readVecPredReg(prev_phys_reg));
                break;
              case CCRegClass:
                this->setCCRegOperand(this->staticInst.get(), idx,
                        this->cpu->readCCReg(prev_phys_reg));
                break;
              case MiscRegClass:
                // no need to forward misc reg values
                break;
              default:
                panic("Unknown register class: %d",
                        (int)original_dest_reg.classValue());
            }
        }
    }
    /** Traps to handle specified fault. */
    void trap(const Fault &fault);

  public:

    // The register accessor methods provide the index of the
    // instruction's operand (e.g., 0 or 1), not the architectural
    // register index, to simplify the implementation of register
    // renaming.  We find the architectural register index by indexing
    // into the instruction's own operand index table.  Note that a
    // raw pointer to the StaticInst is provided instead of a
    // ref-counted StaticInstPtr to redice overhead.  This is fine as
    // long as these methods don't copy the pointer into any long-term
    // storage (which is pretty hard to imagine they would have reason
    // to do).

    RegVal
    readIntRegOperand(const StaticInst *si, int idx) override
    {
        return this->cpu->readIntReg(this->regs.renamedSrcIdx(idx));
    }

    RegVal
    readFloatRegOperandBits(const StaticInst *si, int idx) override
    {
        return this->cpu->readFloatReg(this->regs.renamedSrcIdx(idx));
    }

    const TheISA::VecRegContainer&
    readVecRegOperand(const StaticInst *si, int idx) const override
    {
        return this->cpu->readVecReg(this->regs.renamedSrcIdx(idx));
    }

    /**
     * Read destination vector register operand for modification.
     */
    TheISA::VecRegContainer&
    getWritableVecRegOperand(const StaticInst *si, int idx) override
    {
        return this->cpu->getWritableVecReg(this->regs.renamedDestIdx(idx));
    }

    TheISA::VecElem
    readVecElemOperand(const StaticInst *si, int idx) const override
    {
        return this->cpu->readVecElem(this->regs.renamedSrcIdx(idx));
    }

    const TheISA::VecPredRegContainer&
    readVecPredRegOperand(const StaticInst *si, int idx) const override
    {
        return this->cpu->readVecPredReg(this->regs.renamedSrcIdx(idx));
    }

    TheISA::VecPredRegContainer&
    getWritableVecPredRegOperand(const StaticInst *si, int idx) override
    {
        return this->cpu->getWritableVecPredReg(
                this->regs.renamedDestIdx(idx));
    }

    RegVal
    readCCRegOperand(const StaticInst *si, int idx) override
    {
        return this->cpu->readCCReg(this->regs.renamedSrcIdx(idx));
    }

    /** @todo: Make results into arrays so they can handle multiple dest
     *  registers.
     */
    void
    setIntRegOperand(const StaticInst *si, int idx, RegVal val) override
    {
        this->cpu->setIntReg(this->regs.renamedDestIdx(idx), val);
        setScalarResult(val);
    }

    void
    setFloatRegOperandBits(const StaticInst *si, int idx, RegVal val) override
    {
        this->cpu->setFloatReg(this->regs.renamedDestIdx(idx), val);
        setScalarResult(val);
    }

    void
    setVecRegOperand(const StaticInst *si, int idx,
                     const TheISA::VecRegContainer& val) override
    {
        this->cpu->setVecReg(this->regs.renamedDestIdx(idx), val);
        setVecResult(val);
    }

    void
    setVecElemOperand(const StaticInst *si, int idx,
            const TheISA::VecElem val) override
    {
        int reg_idx = idx;
        this->cpu->setVecElem(this->regs.renamedDestIdx(reg_idx), val);
        setVecElemResult(val);
    }

    void
    setVecPredRegOperand(const StaticInst *si, int idx,
                         const TheISA::VecPredRegContainer& val) override
    {
        this->cpu->setVecPredReg(this->regs.renamedDestIdx(idx), val);
        setVecPredResult(val);
    }

    void
    setCCRegOperand(const StaticInst *si, int idx, RegVal val) override
    {
        this->cpu->setCCReg(this->regs.renamedDestIdx(idx), val);
        setScalarResult(val);
    }
};

} // namespace o3
} // namespace gem5

#endif // __CPU_O3_DYN_INST_HH__
