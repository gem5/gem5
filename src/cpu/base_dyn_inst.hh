/*
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
 * Copyright (c) 2009 The University of Edinburgh
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
 *          Timothy M. Jones
 */

#ifndef __CPU_BASE_DYN_INST_HH__
#define __CPU_BASE_DYN_INST_HH__

#include <bitset>
#include <list>
#include <string>

#include "arch/faults.hh"
#include "arch/utility.hh"
#include "base/fast_alloc.hh"
#include "base/trace.hh"
#include "config/full_system.hh"
#include "config/the_isa.hh"
#include "cpu/o3/comm.hh"
#include "cpu/exetrace.hh"
#include "cpu/inst_seq.hh"
#include "cpu/op_class.hh"
#include "cpu/static_inst.hh"
#include "cpu/translation.hh"
#include "mem/packet.hh"
#include "sim/byteswap.hh"
#include "sim/system.hh"
#include "sim/tlb.hh"

/**
 * @file
 * Defines a dynamic instruction context.
 */

// Forward declaration.
class StaticInstPtr;

template <class Impl>
class BaseDynInst : public FastAlloc, public RefCounted
{
  public:
    // Typedef for the CPU.
    typedef typename Impl::CPUType ImplCPU;
    typedef typename ImplCPU::ImplState ImplState;

    // Logical register index type.
    typedef TheISA::RegIndex RegIndex;
    // Integer register type.
    typedef TheISA::IntReg IntReg;
    // Floating point register type.
    typedef TheISA::FloatReg FloatReg;

    // The DynInstPtr type.
    typedef typename Impl::DynInstPtr DynInstPtr;

    // The list of instructions iterator type.
    typedef typename std::list<DynInstPtr>::iterator ListIt;

    enum {
        MaxInstSrcRegs = TheISA::MaxInstSrcRegs,        /// Max source regs
        MaxInstDestRegs = TheISA::MaxInstDestRegs,      /// Max dest regs
    };

    /** The StaticInst used by this BaseDynInst. */
    StaticInstPtr staticInst;

    ////////////////////////////////////////////
    //
    // INSTRUCTION EXECUTION
    //
    ////////////////////////////////////////////
    /** InstRecord that tracks this instructions. */
    Trace::InstRecord *traceData;

    void demapPage(Addr vaddr, uint64_t asn)
    {
        cpu->demapPage(vaddr, asn);
    }
    void demapInstPage(Addr vaddr, uint64_t asn)
    {
        cpu->demapPage(vaddr, asn);
    }
    void demapDataPage(Addr vaddr, uint64_t asn)
    {
        cpu->demapPage(vaddr, asn);
    }

    /**
     * Does a read to a given address.
     * @param addr The address to read.
     * @param data The read's data is written into this parameter.
     * @param flags The request's flags.
     * @return Returns any fault due to the read.
     */
    template <class T>
    Fault read(Addr addr, T &data, unsigned flags);

    Fault readBytes(Addr addr, uint8_t *data, unsigned size, unsigned flags);

    /**
     * Does a write to a given address.
     * @param data The data to be written.
     * @param addr The address to write to.
     * @param flags The request's flags.
     * @param res The result of the write (for load locked/store conditionals).
     * @return Returns any fault due to the write.
     */
    template <class T>
    Fault write(T data, Addr addr, unsigned flags, uint64_t *res);

    Fault writeBytes(uint8_t *data, unsigned size,
                     Addr addr, unsigned flags, uint64_t *res);

    /** Splits a request in two if it crosses a dcache block. */
    void splitRequest(RequestPtr req, RequestPtr &sreqLow,
                      RequestPtr &sreqHigh);

    /** Initiate a DTB address translation. */
    void initiateTranslation(RequestPtr req, RequestPtr sreqLow,
                             RequestPtr sreqHigh, uint64_t *res,
                             BaseTLB::Mode mode);

    /** Finish a DTB address translation. */
    void finishTranslation(WholeTranslationState *state);

    /** @todo: Consider making this private. */
  public:
    /** The sequence number of the instruction. */
    InstSeqNum seqNum;

    enum Status {
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
        RecoverInst,             /// Is a recover instruction
        BlockingInst,            /// Is a blocking instruction
        ThreadsyncWait,          /// Is a thread synchronization instruction
        SerializeBefore,         /// Needs to serialize on
                                 /// instructions ahead of it
        SerializeAfter,          /// Needs to serialize instructions behind it
        SerializeHandled,        /// Serialization has been handled
        NumStatus
    };

    /** The status of this BaseDynInst.  Several bits can be set. */
    std::bitset<NumStatus> status;

    /** The thread this instruction is from. */
    ThreadID threadNumber;

    /** data address space ID, for loads & stores. */
    short asid;

    /** How many source registers are ready. */
    unsigned readyRegs;

    /** Pointer to the Impl's CPU object. */
    ImplCPU *cpu;

    /** Pointer to the thread state. */
    ImplState *thread;

    /** The kind of fault this instruction has generated. */
    Fault fault;

    /** Pointer to the data for the memory access. */
    uint8_t *memData;

    /** The effective virtual address (lds & stores only). */
    Addr effAddr;

    /** Is the effective virtual address valid. */
    bool effAddrValid;

    /** The effective physical address. */
    Addr physEffAddr;

    /** Effective virtual address for a copy source. */
    Addr copySrcEffAddr;

    /** Effective physical address for a copy source. */
    Addr copySrcPhysEffAddr;

    /** The memory request flags (from translation). */
    unsigned memReqFlags;

    union Result {
        uint64_t integer;
//        float fp;
        double dbl;
    };

    /** The result of the instruction; assumes for now that there's only one
     *  destination register.
     */
    Result instResult;

    /** Records changes to result? */
    bool recordResult;

    /** Did this instruction execute, or is it predicated false */
    bool predicate;

  protected:
    /** PC state for this instruction. */
    TheISA::PCState pc;

    /** Predicted PC state after this instruction. */
    TheISA::PCState predPC;

    /** If this is a branch that was predicted taken */
    bool predTaken;

  public:

#ifdef DEBUG
    void dumpSNList();
#endif

    /** Whether or not the source register is ready.
     *  @todo: Not sure this should be here vs the derived class.
     */
    bool _readySrcRegIdx[MaxInstSrcRegs];

  protected:
    /** Flattened register index of the destination registers of this
     *  instruction.
     */
    TheISA::RegIndex _flatDestRegIdx[TheISA::MaxInstDestRegs];

    /** Flattened register index of the source registers of this
     *  instruction.
     */
    TheISA::RegIndex _flatSrcRegIdx[TheISA::MaxInstSrcRegs];

    /** Physical register index of the destination registers of this
     *  instruction.
     */
    PhysRegIndex _destRegIdx[TheISA::MaxInstDestRegs];

    /** Physical register index of the source registers of this
     *  instruction.
     */
    PhysRegIndex _srcRegIdx[TheISA::MaxInstSrcRegs];

    /** Physical register index of the previous producers of the
     *  architected destinations.
     */
    PhysRegIndex _prevDestRegIdx[TheISA::MaxInstDestRegs];

  public:

    /** Returns the physical register index of the i'th destination
     *  register.
     */
    PhysRegIndex renamedDestRegIdx(int idx) const
    {
        return _destRegIdx[idx];
    }

    /** Returns the physical register index of the i'th source register. */
    PhysRegIndex renamedSrcRegIdx(int idx) const
    {
        return _srcRegIdx[idx];
    }

    /** Returns the flattened register index of the i'th destination
     *  register.
     */
    TheISA::RegIndex flattenedDestRegIdx(int idx) const
    {
        return _flatDestRegIdx[idx];
    }

    /** Returns the flattened register index of the i'th source register */
    TheISA::RegIndex flattenedSrcRegIdx(int idx) const
    {
        return _flatSrcRegIdx[idx];
    }

    /** Returns the physical register index of the previous physical register
     *  that remapped to the same logical register index.
     */
    PhysRegIndex prevDestRegIdx(int idx) const
    {
        return _prevDestRegIdx[idx];
    }

    /** Renames a destination register to a physical register.  Also records
     *  the previous physical register that the logical register mapped to.
     */
    void renameDestReg(int idx,
                       PhysRegIndex renamed_dest,
                       PhysRegIndex previous_rename)
    {
        _destRegIdx[idx] = renamed_dest;
        _prevDestRegIdx[idx] = previous_rename;
    }

    /** Renames a source logical register to the physical register which
     *  has/will produce that logical register's result.
     *  @todo: add in whether or not the source register is ready.
     */
    void renameSrcReg(int idx, PhysRegIndex renamed_src)
    {
        _srcRegIdx[idx] = renamed_src;
    }

    /** Flattens a source architectural register index into a logical index.
     */
    void flattenSrcReg(int idx, TheISA::RegIndex flattened_src)
    {
        _flatSrcRegIdx[idx] = flattened_src;
    }

    /** Flattens a destination architectural register index into a logical
     * index.
     */
    void flattenDestReg(int idx, TheISA::RegIndex flattened_dest)
    {
        _flatDestRegIdx[idx] = flattened_dest;
    }
    /** BaseDynInst constructor given a binary instruction.
     *  @param staticInst A StaticInstPtr to the underlying instruction.
     *  @param pc The PC state for the instruction.
     *  @param predPC The predicted next PC state for the instruction.
     *  @param seq_num The sequence number of the instruction.
     *  @param cpu Pointer to the instruction's CPU.
     */
    BaseDynInst(StaticInstPtr staticInst, TheISA::PCState pc,
                TheISA::PCState predPC, InstSeqNum seq_num, ImplCPU *cpu);

    /** BaseDynInst constructor given a binary instruction.
     *  @param inst The binary instruction.
     *  @param _pc The PC state for the instruction.
     *  @param _predPC The predicted next PC state for the instruction.
     *  @param seq_num The sequence number of the instruction.
     *  @param cpu Pointer to the instruction's CPU.
     */
    BaseDynInst(TheISA::ExtMachInst inst, TheISA::PCState pc,
                TheISA::PCState predPC, InstSeqNum seq_num, ImplCPU *cpu);

    /** BaseDynInst constructor given a StaticInst pointer.
     *  @param _staticInst The StaticInst for this BaseDynInst.
     */
    BaseDynInst(StaticInstPtr &_staticInst);

    /** BaseDynInst destructor. */
    ~BaseDynInst();

  private:
    /** Function to initialize variables in the constructors. */
    void initVars();

  public:
    /** Dumps out contents of this BaseDynInst. */
    void dump();

    /** Dumps out contents of this BaseDynInst into given string. */
    void dump(std::string &outstring);

    /** Read this CPU's ID. */
    int cpuId() { return cpu->cpuId(); }

    /** Read this context's system-wide ID **/
    int contextId() { return thread->contextId(); }

    /** Returns the fault type. */
    Fault getFault() { return fault; }

    /** Checks whether or not this instruction has had its branch target
     *  calculated yet.  For now it is not utilized and is hacked to be
     *  always false.
     *  @todo: Actually use this instruction.
     */
    bool doneTargCalc() { return false; }

    /** Set the predicted target of this current instruction. */
    void setPredTarg(const TheISA::PCState &_predPC)
    {
        predPC = _predPC;
    }

    const TheISA::PCState &readPredTarg() { return predPC; }

    /** Returns the predicted PC immediately after the branch. */
    Addr predInstAddr() { return predPC.instAddr(); }

    /** Returns the predicted PC two instructions after the branch */
    Addr predNextInstAddr() { return predPC.nextInstAddr(); }

    /** Returns the predicted micro PC after the branch */
    Addr predMicroPC() { return predPC.microPC(); }

    /** Returns whether the instruction was predicted taken or not. */
    bool readPredTaken()
    {
        return predTaken;
    }

    void setPredTaken(bool predicted_taken)
    {
        predTaken = predicted_taken;
    }

    /** Returns whether the instruction mispredicted. */
    bool mispredicted()
    {
        TheISA::PCState tempPC = pc;
        TheISA::advancePC(tempPC, staticInst);
        return !(tempPC == predPC);
    }

    //
    //  Instruction types.  Forward checks to StaticInst object.
    //
    bool isNop()          const { return staticInst->isNop(); }
    bool isMemRef()       const { return staticInst->isMemRef(); }
    bool isLoad()         const { return staticInst->isLoad(); }
    bool isStore()        const { return staticInst->isStore(); }
    bool isStoreConditional() const
    { return staticInst->isStoreConditional(); }
    bool isInstPrefetch() const { return staticInst->isInstPrefetch(); }
    bool isDataPrefetch() const { return staticInst->isDataPrefetch(); }
    bool isCopy()         const { return staticInst->isCopy(); }
    bool isInteger()      const { return staticInst->isInteger(); }
    bool isFloating()     const { return staticInst->isFloating(); }
    bool isControl()      const { return staticInst->isControl(); }
    bool isCall()         const { return staticInst->isCall(); }
    bool isReturn()       const { return staticInst->isReturn(); }
    bool isDirectCtrl()   const { return staticInst->isDirectCtrl(); }
    bool isIndirectCtrl() const { return staticInst->isIndirectCtrl(); }
    bool isCondCtrl()     const { return staticInst->isCondCtrl(); }
    bool isUncondCtrl()   const { return staticInst->isUncondCtrl(); }
    bool isCondDelaySlot() const { return staticInst->isCondDelaySlot(); }
    bool isThreadSync()   const { return staticInst->isThreadSync(); }
    bool isSerializing()  const { return staticInst->isSerializing(); }
    bool isSerializeBefore() const
    { return staticInst->isSerializeBefore() || status[SerializeBefore]; }
    bool isSerializeAfter() const
    { return staticInst->isSerializeAfter() || status[SerializeAfter]; }
    bool isSquashAfter() const { return staticInst->isSquashAfter(); }
    bool isMemBarrier()   const { return staticInst->isMemBarrier(); }
    bool isWriteBarrier() const { return staticInst->isWriteBarrier(); }
    bool isNonSpeculative() const { return staticInst->isNonSpeculative(); }
    bool isQuiesce() const { return staticInst->isQuiesce(); }
    bool isIprAccess() const { return staticInst->isIprAccess(); }
    bool isUnverifiable() const { return staticInst->isUnverifiable(); }
    bool isSyscall() const { return staticInst->isSyscall(); }
    bool isMacroop() const { return staticInst->isMacroop(); }
    bool isMicroop() const { return staticInst->isMicroop(); }
    bool isDelayedCommit() const { return staticInst->isDelayedCommit(); }
    bool isLastMicroop() const { return staticInst->isLastMicroop(); }
    bool isFirstMicroop() const { return staticInst->isFirstMicroop(); }
    bool isMicroBranch() const { return staticInst->isMicroBranch(); }

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
    TheISA::PCState branchTarget() const
    { return staticInst->branchTarget(pc); }

    /** Returns the number of source registers. */
    int8_t numSrcRegs() const { return staticInst->numSrcRegs(); }

    /** Returns the number of destination registers. */
    int8_t numDestRegs() const { return staticInst->numDestRegs(); }

    // the following are used to track physical register usage
    // for machines with separate int & FP reg files
    int8_t numFPDestRegs()  const { return staticInst->numFPDestRegs(); }
    int8_t numIntDestRegs() const { return staticInst->numIntDestRegs(); }

    /** Returns the logical register index of the i'th destination register. */
    RegIndex destRegIdx(int i) const { return staticInst->destRegIdx(i); }

    /** Returns the logical register index of the i'th source register. */
    RegIndex srcRegIdx(int i) const { return staticInst->srcRegIdx(i); }

    /** Returns the result of an integer instruction. */
    uint64_t readIntResult() { return instResult.integer; }

    /** Returns the result of a floating point instruction. */
    float readFloatResult() { return (float)instResult.dbl; }

    /** Returns the result of a floating point (double) instruction. */
    double readDoubleResult() { return instResult.dbl; }

    /** Records an integer register being set to a value. */
    void setIntRegOperand(const StaticInst *si, int idx, uint64_t val)
    {
        if (recordResult)
            instResult.integer = val;
    }

    /** Records an fp register being set to a value. */
    void setFloatRegOperand(const StaticInst *si, int idx, FloatReg val,
                            int width)
    {
        if (recordResult) {
            if (width == 32)
                instResult.dbl = (double)val;
            else if (width == 64)
                instResult.dbl = val;
            else
                panic("Unsupported width!");
        }
    }

    /** Records an fp register being set to a value. */
    void setFloatRegOperand(const StaticInst *si, int idx, FloatReg val)
    {
        if (recordResult)
            instResult.dbl = (double)val;
    }

    /** Records an fp register being set to an integer value. */
    void setFloatRegOperandBits(const StaticInst *si, int idx, uint64_t val,
                                int width)
    {
        if (recordResult)
            instResult.integer = val;
    }

    /** Records an fp register being set to an integer value. */
    void setFloatRegOperandBits(const StaticInst *si, int idx, uint64_t val)
    {
        if (recordResult)
            instResult.integer = val;
    }

    /** Records that one of the source registers is ready. */
    void markSrcRegReady();

    /** Marks a specific register as ready. */
    void markSrcRegReady(RegIndex src_idx);

    /** Returns if a source register is ready. */
    bool isReadySrcRegIdx(int idx) const
    {
        return this->_readySrcRegIdx[idx];
    }

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
    void setSquashed() { status.set(Squashed); }

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
    void setSquashedInLSQ() { status.set(SquashedInLSQ);}

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

    /** Read the PC state of this instruction. */
    const TheISA::PCState pcState() const { return pc; }

    /** Set the PC state of this instruction. */
    const void pcState(const TheISA::PCState &val) { pc = val; }

    /** Read the PC of this instruction. */
    const Addr instAddr() const { return pc.instAddr(); }

    /** Read the PC of the next instruction. */
    const Addr nextInstAddr() const { return pc.nextInstAddr(); }

    /**Read the micro PC of this instruction. */
    const Addr microPC() const { return pc.microPC(); }

    bool readPredicate()
    {
        return predicate;
    }

    void setPredicate(bool val)
    {
        predicate = val;

        if (traceData) {
            traceData->setPredicate(val);
        }
    }

    /** Sets the ASID. */
    void setASID(short addr_space_id) { asid = addr_space_id; }

    /** Sets the thread id. */
    void setTid(ThreadID tid) { threadNumber = tid; }

    /** Sets the pointer to the thread state. */
    void setThreadState(ImplState *state) { thread = state; }

    /** Returns the thread context. */
    ThreadContext *tcBase() { return thread->getTC(); }

  private:
    /** Instruction effective address.
     *  @todo: Consider if this is necessary or not.
     */
    Addr instEffAddr;

    /** Whether or not the effective address calculation is completed.
     *  @todo: Consider if this is necessary or not.
     */
    bool eaCalcDone;

    /** Is this instruction's memory access uncacheable. */
    bool isUncacheable;

    /** Has this instruction generated a memory request. */
    bool reqMade;

  public:
    /** Sets the effective address. */
    void setEA(Addr &ea) { instEffAddr = ea; eaCalcDone = true; }

    /** Returns the effective address. */
    const Addr &getEA() const { return instEffAddr; }

    /** Returns whether or not the eff. addr. calculation has been completed. */
    bool doneEACalc() { return eaCalcDone; }

    /** Returns whether or not the eff. addr. source registers are ready. */
    bool eaSrcsReady();

    /** Whether or not the memory operation is done. */
    bool memOpDone;

    /** Is this instruction's memory access uncacheable. */
    bool uncacheable() { return isUncacheable; }

    /** Has this instruction generated a memory request. */
    bool hasRequest() { return reqMade; }

  public:
    /** Load queue index. */
    int16_t lqIdx;

    /** Store queue index. */
    int16_t sqIdx;

    /** Iterator pointing to this BaseDynInst in the list of all insts. */
    ListIt instListIt;

    /** Returns iterator to this instruction in the list of all insts. */
    ListIt &getInstListIt() { return instListIt; }

    /** Sets iterator for this instruction in the list of all insts. */
    void setInstListIt(ListIt _instListIt) { instListIt = _instListIt; }

  public:
    /** Returns the number of consecutive store conditional failures. */
    unsigned readStCondFailures()
    { return thread->storeCondFailures; }

    /** Sets the number of consecutive store conditional failures. */
    void setStCondFailures(unsigned sc_failures)
    { thread->storeCondFailures = sc_failures; }
};

template<class Impl>
Fault
BaseDynInst<Impl>::readBytes(Addr addr, uint8_t *data,
                             unsigned size, unsigned flags)
{
    reqMade = true;
    Request *req = new Request(asid, addr, size, flags, this->pc.instAddr(),
                               thread->contextId(), threadNumber);

    Request *sreqLow = NULL;
    Request *sreqHigh = NULL;

    // Only split the request if the ISA supports unaligned accesses.
    if (TheISA::HasUnalignedMemAcc) {
        splitRequest(req, sreqLow, sreqHigh);
    }
    initiateTranslation(req, sreqLow, sreqHigh, NULL, BaseTLB::Read);

    if (fault == NoFault) {
        effAddr = req->getVaddr();
        effAddrValid = true;
        fault = cpu->read(req, sreqLow, sreqHigh, data, lqIdx);
    } else {
        // Commit will have to clean up whatever happened.  Set this
        // instruction as executed.
        this->setExecuted();
    }

    if (fault != NoFault) {
        // Return a fixed value to keep simulation deterministic even
        // along misspeculated paths.
        if (data)
            bzero(data, size);
    }

    if (traceData) {
        traceData->setAddr(addr);
    }

    return fault;
}

template<class Impl>
template<class T>
inline Fault
BaseDynInst<Impl>::read(Addr addr, T &data, unsigned flags)
{
    Fault fault = readBytes(addr, (uint8_t *)&data, sizeof(T), flags);

    data = TheISA::gtoh(data);

    if (traceData) {
        traceData->setData(data);
    }

    return fault;
}

template<class Impl>
Fault
BaseDynInst<Impl>::writeBytes(uint8_t *data, unsigned size,
                              Addr addr, unsigned flags, uint64_t *res)
{
    if (traceData) {
        traceData->setAddr(addr);
    }

    reqMade = true;
    Request *req = new Request(asid, addr, size, flags, this->pc.instAddr(),
                               thread->contextId(), threadNumber);

    Request *sreqLow = NULL;
    Request *sreqHigh = NULL;

    // Only split the request if the ISA supports unaligned accesses.
    if (TheISA::HasUnalignedMemAcc) {
        splitRequest(req, sreqLow, sreqHigh);
    }
    initiateTranslation(req, sreqLow, sreqHigh, res, BaseTLB::Write);

    if (fault == NoFault) {
        effAddr = req->getVaddr();
        effAddrValid = true;
        fault = cpu->write(req, sreqLow, sreqHigh, data, sqIdx);
    }

    return fault;
}

template<class Impl>
template<class T>
inline Fault
BaseDynInst<Impl>::write(T data, Addr addr, unsigned flags, uint64_t *res)
{
    if (traceData) {
        traceData->setData(data);
    }
    data = TheISA::htog(data);
    return writeBytes((uint8_t *)&data, sizeof(T), addr, flags, res);
}

template<class Impl>
inline void
BaseDynInst<Impl>::splitRequest(RequestPtr req, RequestPtr &sreqLow,
                                RequestPtr &sreqHigh)
{
    // Check to see if the request crosses the next level block boundary.
    unsigned block_size = cpu->getDcachePort()->peerBlockSize();
    Addr addr = req->getVaddr();
    Addr split_addr = roundDown(addr + req->getSize() - 1, block_size);
    assert(split_addr <= addr || split_addr - addr < block_size);

    // Spans two blocks.
    if (split_addr > addr) {
        req->splitOnVaddr(split_addr, sreqLow, sreqHigh);
    }
}

template<class Impl>
inline void
BaseDynInst<Impl>::initiateTranslation(RequestPtr req, RequestPtr sreqLow,
                                       RequestPtr sreqHigh, uint64_t *res,
                                       BaseTLB::Mode mode)
{
    if (!TheISA::HasUnalignedMemAcc || sreqLow == NULL) {
        WholeTranslationState *state =
            new WholeTranslationState(req, NULL, res, mode);

        // One translation if the request isn't split.
        DataTranslation<BaseDynInst<Impl> > *trans =
            new DataTranslation<BaseDynInst<Impl> >(this, state);
        cpu->dtb->translateTiming(req, thread->getTC(), trans, mode);
    } else {
        WholeTranslationState *state =
            new WholeTranslationState(req, sreqLow, sreqHigh, NULL, res, mode);

        // Two translations when the request is split.
        DataTranslation<BaseDynInst<Impl> > *stransLow =
            new DataTranslation<BaseDynInst<Impl> >(this, state, 0);
        DataTranslation<BaseDynInst<Impl> > *stransHigh =
            new DataTranslation<BaseDynInst<Impl> >(this, state, 1);

        cpu->dtb->translateTiming(sreqLow, thread->getTC(), stransLow, mode);
        cpu->dtb->translateTiming(sreqHigh, thread->getTC(), stransHigh, mode);
    }
}

template<class Impl>
inline void
BaseDynInst<Impl>::finishTranslation(WholeTranslationState *state)
{
    fault = state->getFault();

    if (state->isUncacheable())
        isUncacheable = true;

    if (fault == NoFault) {
        physEffAddr = state->getPaddr();
        memReqFlags = state->getFlags();

        if (state->mainReq->isCondSwap()) {
            assert(state->res);
            state->mainReq->setExtraData(*state->res);
        }

    } else {
        state->deleteReqs();
    }
    delete state;
}

#endif // __CPU_BASE_DYN_INST_HH__
