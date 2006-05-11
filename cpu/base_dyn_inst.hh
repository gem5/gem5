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

#ifndef __CPU_BASE_DYN_INST_HH__
#define __CPU_BASE_DYN_INST_HH__

#include <list>
#include <string>

#include "base/fast_alloc.hh"
#include "base/trace.hh"
#include "config/full_system.hh"
#include "cpu/exetrace.hh"
#include "cpu/inst_seq.hh"
#include "cpu/static_inst.hh"
#include "encumbered/cpu/full/op_class.hh"
#include "mem/functional/memory_control.hh"
#include "sim/system.hh"
/*
#include "encumbered/cpu/full/bpred_update.hh"
#include "encumbered/cpu/full/spec_memory.hh"
#include "encumbered/cpu/full/spec_state.hh"
#include "encumbered/mem/functional/main.hh"
*/

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
    typedef typename Impl::FullCPU FullCPU;
    typedef typename FullCPU::ImplState ImplState;

    // Binary machine instruction type.
    typedef TheISA::MachInst MachInst;
    // Extended machine instruction type
    typedef TheISA::ExtMachInst ExtMachInst;
    // Logical register index type.
    typedef TheISA::RegIndex RegIndex;
    // Integer register index type.
    typedef TheISA::IntReg IntReg;

    // The DynInstPtr type.
    typedef typename Impl::DynInstPtr DynInstPtr;

    // The list of instructions iterator type.
    typedef typename std::list<DynInstPtr>::iterator ListIt;

    enum {
        MaxInstSrcRegs = TheISA::MaxInstSrcRegs,	/// Max source regs
        MaxInstDestRegs = TheISA::MaxInstDestRegs,	/// Max dest regs
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

    /**
     * Does a read to a given address.
     * @param addr The address to read.
     * @param data The read's data is written into this parameter.
     * @param flags The request's flags.
     * @return Returns any fault due to the read.
     */
    template <class T>
    Fault read(Addr addr, T &data, unsigned flags);

    /**
     * Does a write to a given address.
     * @param data The data to be written.
     * @param addr The address to write to.
     * @param flags The request's flags.
     * @param res The result of the write (for load locked/store conditionals).
     * @return Returns any fault due to the write.
     */
    template <class T>
    Fault write(T data, Addr addr, unsigned flags,
                        uint64_t *res);

    void prefetch(Addr addr, unsigned flags);
    void writeHint(Addr addr, int size, unsigned flags);
    Fault copySrcTranslate(Addr src);
    Fault copy(Addr dest);

    /** @todo: Consider making this private. */
  public:
    /** The sequence number of the instruction. */
    InstSeqNum seqNum;

    /** Is the instruction in the IQ */
    bool iqEntry;

    /** Is the instruction in the ROB */
    bool robEntry;

    /** Is the instruction in the LSQ */
    bool lsqEntry;

    /** Is the instruction completed. */
    bool completed;

    /** Is the instruction's result ready. */
    bool resultReady;

    /** Can this instruction issue. */
    bool canIssue;

    /** Has this instruction issued. */
    bool issued;

    /** Has this instruction executed (or made it through execute) yet. */
    bool executed;

    /** Can this instruction commit. */
    bool canCommit;

    /** Is this instruction committed. */
    bool committed;

    /** Is this instruction squashed. */
    bool squashed;

    /** Is this instruction squashed in the instruction queue. */
    bool squashedInIQ;

    /** Is this instruction squashed in the instruction queue. */
    bool squashedInLSQ;

    /** Is this instruction squashed in the instruction queue. */
    bool squashedInROB;

    /** Is this a recover instruction. */
    bool recoverInst;

    /** Is this a thread blocking instruction. */
    bool blockingInst;	/* this inst has called thread_block() */

    /** Is this a thread syncrhonization instruction. */
    bool threadsyncWait;

    /** The thread this instruction is from. */
    short threadNumber;

    /** data address space ID, for loads & stores. */
    short asid;

    /** How many source registers are ready. */
    unsigned readyRegs;

    /** Pointer to the FullCPU object. */
    FullCPU *cpu;

    /** Pointer to the exec context. */
    ImplState *thread;

    /** The kind of fault this instruction has generated. */
    Fault fault;

    /** The memory request. */
    MemReqPtr req;

    /** The effective virtual address (lds & stores only). */
    Addr effAddr;

    /** The effective physical address. */
    Addr physEffAddr;

    /** Effective virtual address for a copy source. */
    Addr copySrcEffAddr;

    /** Effective physical address for a copy source. */
    Addr copySrcPhysEffAddr;

    /** The memory request flags (from translation). */
    unsigned memReqFlags;

    /** The size of the data to be stored. */
    int storeSize;

    /** The data to be stored. */
    IntReg storeData;

    union Result {
        uint64_t integer;
        float fp;
        double dbl;
    };

    /** The result of the instruction; assumes for now that there's only one
     *  destination register.
     */
    Result instResult;

    /** PC of this instruction. */
    Addr PC;

    /** Next non-speculative PC.  It is not filled in at fetch, but rather
     *  once the target of the branch is truly known (either decode or
     *  execute).
     */
    Addr nextPC;

    /** Predicted next PC. */
    Addr predPC;

    /** Count of total number of dynamic instructions. */
    static int instcount;

#ifdef DEBUG
    void dumpSNList();
#endif

    /** Whether or not the source register is ready.
     *  @todo: Not sure this should be here vs the derived class.
     */
    bool _readySrcRegIdx[MaxInstSrcRegs];

  public:
    /** BaseDynInst constructor given a binary instruction.
     *  @param inst The binary instruction.
     *  @param PC The PC of the instruction.
     *  @param pred_PC The predicted next PC.
     *  @param seq_num The sequence number of the instruction.
     *  @param cpu Pointer to the instruction's CPU.
     */
    BaseDynInst(ExtMachInst inst, Addr PC, Addr pred_PC, InstSeqNum seq_num,
                FullCPU *cpu);

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
    /**
     *  @todo: Make this function work; currently it is a dummy function.
     *  @param fault Last fault.
     *  @param cmd Last command.
     *  @param addr Virtual address of access.
     *  @param p Memory accessed.
     *  @param nbytes Access size.
     */
    void
    trace_mem(Fault fault,
              MemCmd cmd,
              Addr addr,
              void *p,
              int nbytes);

    /** Dumps out contents of this BaseDynInst. */
    void dump();

    /** Dumps out contents of this BaseDynInst into given string. */
    void dump(std::string &outstring);

    /** Returns the fault type. */
    Fault getFault() { return fault; }

    /** Checks whether or not this instruction has had its branch target
     *  calculated yet.  For now it is not utilized and is hacked to be
     *  always false.
     *  @todo: Actually use this instruction.
     */
    bool doneTargCalc() { return false; }

    /** Returns the next PC.  This could be the speculative next PC if it is
     *  called prior to the actual branch target being calculated.
     */
    Addr readNextPC() { return nextPC; }

    /** Set the predicted target of this current instruction. */
    void setPredTarg(Addr predicted_PC) { predPC = predicted_PC; }

    /** Returns the predicted target of the branch. */
    Addr readPredTarg() { return predPC; }

    /** Returns whether the instruction was predicted taken or not. */
    bool predTaken() { return predPC != (PC + sizeof(MachInst)); }

    /** Returns whether the instruction mispredicted. */
    bool mispredicted() { return predPC != nextPC; }

    //
    //  Instruction types.  Forward checks to StaticInst object.
    //
    bool isNop()	  const { return staticInst->isNop(); }
    bool isMemRef()    	  const { return staticInst->isMemRef(); }
    bool isLoad()	  const { return staticInst->isLoad(); }
    bool isStore()	  const { return staticInst->isStore(); }
    bool isInstPrefetch() const { return staticInst->isInstPrefetch(); }
    bool isDataPrefetch() const { return staticInst->isDataPrefetch(); }
    bool isCopy()         const { return staticInst->isCopy(); }
    bool isInteger()	  const { return staticInst->isInteger(); }
    bool isFloating()	  const { return staticInst->isFloating(); }
    bool isControl()	  const { return staticInst->isControl(); }
    bool isCall()	  const { return staticInst->isCall(); }
    bool isReturn()	  const { return staticInst->isReturn(); }
    bool isDirectCtrl()	  const { return staticInst->isDirectCtrl(); }
    bool isIndirectCtrl() const { return staticInst->isIndirectCtrl(); }
    bool isCondCtrl()	  const { return staticInst->isCondCtrl(); }
    bool isUncondCtrl()	  const { return staticInst->isUncondCtrl(); }
    bool isThreadSync()   const { return staticInst->isThreadSync(); }
    bool isSerializing()  const { return staticInst->isSerializing(); }
    bool isSerializeBefore() const
    { return staticInst->isSerializeBefore() || serializeBefore; }
    bool isSerializeAfter() const
    { return staticInst->isSerializeAfter() || serializeAfter; }
    bool isMemBarrier()   const { return staticInst->isMemBarrier(); }
    bool isWriteBarrier() const { return staticInst->isWriteBarrier(); }
    bool isNonSpeculative() const { return staticInst->isNonSpeculative(); }
    bool isQuiesce() const { return staticInst->isQuiesce(); }
    bool isUnverifiable() const { return staticInst->isUnverifiable(); }

    /** Temporarily sets this instruction as a serialize before instruction. */
    void setSerializeBefore() { serializeBefore = true; }

    /** Clears the serializeBefore part of this instruction. */
    void clearSerializeBefore() { serializeBefore = false; }

    /** Checks if this serializeBefore is only temporarily set. */
    bool isTempSerializeBefore() { return serializeBefore; }

    /** Tracks if instruction has been externally set as serializeBefore. */
    bool serializeBefore;

    /** Temporarily sets this instruction as a serialize after instruction. */
    void setSerializeAfter() { serializeAfter = true; }

    /** Clears the serializeAfter part of this instruction.*/
    void clearSerializeAfter() { serializeAfter = false; }

    /** Checks if this serializeAfter is only temporarily set. */
    bool isTempSerializeAfter() { return serializeAfter; }

    /** Tracks if instruction has been externally set as serializeAfter. */
    bool serializeAfter;

    /** Checks if the serialization part of this instruction has been
     *  handled.  This does not apply to the temporary serializing
     *  state; it only applies to this instruction's own permanent
     *  serializing state.
     */
    bool isSerializeHandled() { return serializeHandled; }

    /** Sets the serialization part of this instruction as handled. */
    void setSerializeHandled() { serializeHandled = true; }

    /** Whether or not the serialization of this instruction has been handled. */
    bool serializeHandled;

    /** Returns the opclass of this instruction. */
    OpClass opClass() const { return staticInst->opClass(); }

    /** Returns the branch target address. */
    Addr branchTarget() const { return staticInst->branchTarget(PC); }

    /** Returns the number of source registers. */
    int8_t numSrcRegs()	const { return staticInst->numSrcRegs(); }

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
    float readFloatResult() { return instResult.fp; }

    /** Returns the result of a floating point (double) instruction. */
    double readDoubleResult() { return instResult.dbl; }

    void setIntReg(const StaticInst *si, int idx, uint64_t val)
    {
        instResult.integer = val;
    }

    void setFloatRegSingle(const StaticInst *si, int idx, float val)
    {
        instResult.fp = val;
    }

    void setFloatRegDouble(const StaticInst *si, int idx, double val)
    {
        instResult.dbl = val;
    }

    void setFloatRegInt(const StaticInst *si, int idx, uint64_t val)
    {
        instResult.integer = val;
    }

    //Push to .cc file.
    /** Records that one of the source registers is ready. */
    void markSrcRegReady();

    /** Marks a specific register as ready.
     *  @todo: Move this to .cc file.
     */
    void markSrcRegReady(RegIndex src_idx);

    /** Returns if a source register is ready. */
    bool isReadySrcRegIdx(int idx) const
    {
        return this->_readySrcRegIdx[idx];
    }

    /** Sets this instruction as completed. */
    void setCompleted() { completed = true; }

    /** Returns whether or not this instruction is completed. */
    bool isCompleted() const { return completed; }

    void setResultReady() { resultReady = true; }

    bool isResultReady() const { return resultReady; }

    /** Sets this instruction as ready to issue. */
    void setCanIssue() { canIssue = true; }

    /** Returns whether or not this instruction is ready to issue. */
    bool readyToIssue() const { return canIssue; }

    /** Sets this instruction as issued from the IQ. */
    void setIssued() { issued = true; }

    /** Returns whether or not this instruction has issued. */
    bool isIssued() const { return issued; }

    /** Sets this instruction as executed. */
    void setExecuted() { executed = true; }

    /** Returns whether or not this instruction has executed. */
    bool isExecuted() const { return executed; }

    /** Sets this instruction as ready to commit. */
    void setCanCommit() { canCommit = true; }

    /** Clears this instruction as being ready to commit. */
    void clearCanCommit() { canCommit = false; }

    /** Returns whether or not this instruction is ready to commit. */
    bool readyToCommit() const { return canCommit; }

    /** Sets this instruction as committed. */
    void setCommitted() { committed = true; }

    /** Returns whether or not this instruction is committed. */
    bool isCommitted() const { return committed; }

    /** Sets this instruction as squashed. */
    void setSquashed() { squashed = true; }

    /** Returns whether or not this instruction is squashed. */
    bool isSquashed() const { return squashed; }

    //Instruction Queue Entry
    //-----------------------
    /** Sets this instruction as a entry the IQ. */
    void setInIQ() { iqEntry = true; }

    /** Sets this instruction as a entry the IQ. */
    void removeInIQ() { iqEntry = false; }

    /** Sets this instruction as squashed in the IQ. */
    void setSquashedInIQ() { squashedInIQ = true; squashed = true;}

    /** Returns whether or not this instruction is squashed in the IQ. */
    bool isSquashedInIQ() const { return squashedInIQ; }

    /** Returns whether or not this instruction has issued. */
    bool isInIQ() const { return iqEntry; }


    //Load / Store Queue Functions
    //-----------------------
    /** Sets this instruction as a entry the LSQ. */
    void setInLSQ() { lsqEntry = true; }

    /** Sets this instruction as a entry the LSQ. */
    void removeInLSQ() { lsqEntry = false; }

    /** Sets this instruction as squashed in the LSQ. */
    void setSquashedInLSQ() { squashedInLSQ = true;}

    /** Returns whether or not this instruction is squashed in the LSQ. */
    bool isSquashedInLSQ() const { return squashedInLSQ; }

    /** Returns whether or not this instruction is in the LSQ. */
    bool isInLSQ() const { return lsqEntry; }


    //Reorder Buffer Functions
    //-----------------------
    /** Sets this instruction as a entry the ROB. */
    void setInROB() { robEntry = true; }

    /** Sets this instruction as a entry the ROB. */
    void removeInROB() { robEntry = false; }

    /** Sets this instruction as squashed in the ROB. */
    void setSquashedInROB() { squashedInROB = true; }

    /** Returns whether or not this instruction is squashed in the ROB. */
    bool isSquashedInROB() const { return squashedInROB; }

    /** Returns whether or not this instruction is in the ROB. */
    bool isInROB() const { return robEntry; }

    /** Read the PC of this instruction. */
    const Addr readPC() const { return PC; }

    /** Set the next PC of this instruction (its actual target). */
    void setNextPC(uint64_t val)
    {
        nextPC = val;
//        instResult.integer = val;
    }

    void setASID(short addr_space_id) { asid = addr_space_id; }

    void setThread(unsigned tid) { threadNumber = tid; }

    void setState(ImplState *state) { thread = state; }

    /** Returns the exec context.
     *  @todo: Remove this once the ExecContext is no longer used.
     */
    ExecContext *xcBase() { return thread->getXCProxy(); }

  private:
    /** Instruction effective address.
     *  @todo: Consider if this is necessary or not.
     */
    Addr instEffAddr;

    /** Whether or not the effective address calculation is completed.
     *  @todo: Consider if this is necessary or not.
     */
    bool eaCalcDone;

  public:
    /** Sets the effective address. */
    void setEA(Addr &ea) { instEffAddr = ea; eaCalcDone = true; }

    /** Returns the effective address. */
    const Addr &getEA() const { return req->vaddr; }

    /** Returns whether or not the eff. addr. calculation has been completed. */
    bool doneEACalc() { return eaCalcDone; }

    /** Returns whether or not the eff. addr. source registers are ready. */
    bool eaSrcsReady();

    /** Whether or not the memory operation is done. */
    bool memOpDone;

  public:
    /** Load queue index. */
    int16_t lqIdx;

    /** Store queue index. */
    int16_t sqIdx;

    bool reachedCommit;

    /** Iterator pointing to this BaseDynInst in the list of all insts. */
    ListIt instListIt;

    /** Returns iterator to this instruction in the list of all insts. */
    ListIt &getInstListIt() { return instListIt; }

    /** Sets iterator for this instruction in the list of all insts. */
    void setInstListIt(ListIt _instListIt) { instListIt = _instListIt; }
};

template<class Impl>
template<class T>
inline Fault
BaseDynInst<Impl>::read(Addr addr, T &data, unsigned flags)
{
    if (executed) {
        fault = cpu->read(req, data, lqIdx);
        return fault;
    }

    req = new MemReq(addr, thread->getXCProxy(), sizeof(T), flags);
    req->asid = asid;
    req->thread_num = threadNumber;
    req->pc = this->PC;

    if ((req->vaddr & (TheISA::VMPageSize - 1)) + req->size >
        TheISA::VMPageSize) {
        return TheISA::genAlignmentFault();
    }

    fault = cpu->translateDataReadReq(req);

    effAddr = req->vaddr;
    physEffAddr = req->paddr;
    memReqFlags = req->flags;

    if (fault == NoFault) {
#if FULL_SYSTEM
        if (cpu->system->memctrl->badaddr(physEffAddr)) {
            fault = TheISA::genMachineCheckFault();
            data = (T)-1;
            this->setExecuted();
        } else {
            fault = cpu->read(req, data, lqIdx);
        }
#else
        fault = cpu->read(req, data, lqIdx);
#endif
    } else {
        // Return a fixed value to keep simulation deterministic even
        // along misspeculated paths.
        data = (T)-1;

        // Commit will have to clean up whatever happened.  Set this
        // instruction as executed.
        this->setExecuted();
    }

    if (traceData) {
        traceData->setAddr(addr);
        traceData->setData(data);
    }

    return fault;
}

template<class Impl>
template<class T>
inline Fault
BaseDynInst<Impl>::write(T data, Addr addr, unsigned flags, uint64_t *res)
{
    if (traceData) {
        traceData->setAddr(addr);
        traceData->setData(data);
    }

    req = new MemReq(addr, thread->getXCProxy(), sizeof(T), flags);

    req->asid = asid;
    req->thread_num = threadNumber;
    req->pc = this->PC;

    if ((req->vaddr & (TheISA::VMPageSize - 1)) + req->size >
        TheISA::VMPageSize) {
        return TheISA::genAlignmentFault();
    }

    fault = cpu->translateDataWriteReq(req);

    effAddr = req->vaddr;
    physEffAddr = req->paddr;
    memReqFlags = req->flags;

    if (fault == NoFault) {
#if FULL_SYSTEM
        if (cpu->system->memctrl->badaddr(physEffAddr)) {
            fault = TheISA::genMachineCheckFault();
        } else {
            fault = cpu->write(req, data, sqIdx);
        }
#else
        fault = cpu->write(req, data, sqIdx);
#endif
    }

    if (res) {
        // always return some result to keep misspeculated paths
        // (which will ignore faults) deterministic
        *res = (fault == NoFault) ? req->result : 0;
    }

    return fault;
}

#endif // __CPU_BASE_DYN_INST_HH__
