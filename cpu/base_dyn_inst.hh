/*
 * Copyright (c) 2001-2004 The Regents of The University of Michigan
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

#ifndef __BASE_DYN_INST_HH__
#define __BASE_DYN_INST_HH__

#include <vector>
#include <string>

#include "base/fast_alloc.hh"
#include "base/trace.hh"

#include "cpu/static_inst.hh"
#include "cpu/beta_cpu/comm.hh"
#include "cpu/full_cpu/bpred_update.hh"
#include "mem/functional_mem/main_memory.hh"
#include "cpu/full_cpu/spec_memory.hh"
#include "cpu/inst_seq.hh"
#include "cpu/full_cpu/op_class.hh"
#include "cpu/full_cpu/spec_state.hh"

/**
 * @file
 * Defines a dynamic instruction context.
 */

namespace Trace {
    class InstRecord;
};

// Forward declaration.
template <class blah>
class StaticInstPtr;

template <class Impl>
class BaseDynInst : public FastAlloc, public RefCounted
{
  public:
    // Typedef for the CPU.
    typedef typename Impl::FullCPU FullCPU;

    //Typedef to get the ISA.
    typedef typename Impl::ISA ISA;

    /// Binary machine instruction type.
    typedef typename ISA::MachInst MachInst;
    /// Memory address type.
    typedef typename ISA::Addr	   Addr;
    /// Logical register index type.
    typedef typename ISA::RegIndex RegIndex;
    /// Integer register index type.
    typedef typename ISA::IntReg   IntReg;

    enum {
        MaxInstSrcRegs = ISA::MaxInstSrcRegs,	//< Max source regs
        MaxInstDestRegs = ISA::MaxInstDestRegs,	//< Max dest regs
    };

    StaticInstPtr<ISA> staticInst;

    ////////////////////////////////////////////
    //
    // INSTRUCTION EXECUTION
    //
    ////////////////////////////////////////////
    Trace::InstRecord *traceData;

//    void setCPSeq(InstSeqNum seq);

    template <class T>
    Fault read(Addr addr, T &data, unsigned flags);

    template <class T>
    Fault write(T data, Addr addr, unsigned flags,
                        uint64_t *res);


    IntReg *getIntegerRegs(void);
    FunctionalMemory *getMemory(void);

    void prefetch(Addr addr, unsigned flags);
    void writeHint(Addr addr, int size, unsigned flags);
    Fault copySrcTranslate(Addr src);
    Fault copy(Addr dest);

  public:
    /** Is this instruction valid. */
    bool valid;

    /** The sequence number of the instruction. */
    InstSeqNum seqNum;

    /** How many source registers are ready. */
    unsigned readyRegs;

    /** Can this instruction issue. */
    bool canIssue;

    /** Has this instruction issued. */
    bool issued;

    /** Has this instruction executed (or made it through execute) yet. */
    bool executed;

    /** Can this instruction commit. */
    bool canCommit;

    /** Is this instruction squashed. */
    bool squashed;

    /** Is this instruction squashed in the instruction queue. */
    bool squashedInIQ;

    /** Is this a recover instruction. */
    bool recoverInst;

    /** Is this a thread blocking instruction. */
    bool blockingInst;	/* this inst has called thread_block() */

    /** Is this a thread syncrhonization instruction. */
    bool threadsyncWait;

    /** If the BTB missed. */
    bool btbMissed;

    /** The thread this instruction is from. */
    short threadNumber;

    /** If instruction is speculative. */
    short specMode;

    /** data address space ID, for loads & stores. */
    short asid;

    /** Pointer to the FullCPU object. */
    FullCPU *cpu;

    /** Pointer to the exec context.  Will not exist in the final version. */
    ExecContext *xc;

    /** The kind of fault this instruction has generated. */
    Fault fault;

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

    /** Result of this instruction, if an integer. */
    uint64_t intResult;

    /** Result of this instruction, if a float. */
    float floatResult;

    /** Result of this instruction, if a double. */
    double doubleResult;

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

    /** Did this instruction do a spec write? */
    bool specMemWrite;

  private:
    /** Physical register index of the destination registers of this
     *  instruction.
     */
    PhysRegIndex _destRegIdx[MaxInstDestRegs];

    /** Physical register index of the source registers of this
     *  instruction.
     */
    PhysRegIndex _srcRegIdx[MaxInstSrcRegs];

    /** Whether or not the source register is ready. */
    bool _readySrcRegIdx[MaxInstSrcRegs];

    /** Physical register index of the previous producers of the
     *  architected destinations.
     */
    PhysRegIndex _prevDestRegIdx[MaxInstDestRegs];

  public:
    /** BaseDynInst constructor given a binary instruction. */
    BaseDynInst(MachInst inst, Addr PC, Addr Pred_PC, InstSeqNum seq_num,
                FullCPU *cpu);

    /** BaseDynInst constructor given a static inst pointer. */
    BaseDynInst(StaticInstPtr<ISA> &_staticInst);

    /** BaseDynInst destructor. */
    ~BaseDynInst();

#if 0
    Fault
    mem_access(MemCmd cmd,	// Read or Write access cmd
               Addr addr,	// virtual address of access
               void *p,		// input/output buffer
               int nbytes);	// access size
#endif

    void
    trace_mem(Fault fault,      // last fault
              MemCmd cmd,       // last command
              Addr addr,        // virtual address of access
              void *p,          // memory accessed
              int nbytes);      // access size

    /** Dumps out contents of this BaseDynInst. */
    void dump();

    /** Dumps out contents of this BaseDynInst into given string. */
    void dump(std::string &outstring);

    /** Returns the fault type. */
    Fault getFault() { return fault; }

    /** Checks whether or not this instruction has had its branch target
     *  calculated yet.  For now it is not utilized and is hacked to be
     *  always false.
     */
    bool doneTargCalc() { return false; }

    /** Returns the calculated target of the branch. */
    Addr readCalcTarg() { return nextPC; }

    Addr readNextPC() { return nextPC; }

    /** Set the predicted target of this current instruction. */
    void setPredTarg(Addr predicted_PC) { predPC = predicted_PC; }

    /** Returns the predicted target of the branch. */
    Addr readPredTarg() { return predPC; }

    /** Returns whether the instruction was predicted taken or not. */
    bool predTaken() {
//        DPRINTF(FullCPU, "PC: %08p\n", PC);
//        DPRINTF(FullCPU, "predPC: %08p\n", predPC);

        return( predPC != (PC + sizeof(MachInst) ) );
    }

    /** Returns whether the instruction mispredicted. */
    bool mispredicted() { return (predPC != nextPC); }

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
    bool isMemBarrier()   const { return staticInst->isMemBarrier(); }
    bool isWriteBarrier() const { return staticInst->isWriteBarrier(); }
    bool isNonSpeculative() const { return staticInst->isNonSpeculative(); }

    int8_t numSrcRegs()	 const { return staticInst->numSrcRegs(); }
    int8_t numDestRegs() const { return staticInst->numDestRegs(); }

    // the following are used to track physical register usage
    // for machines with separate int & FP reg files
    int8_t numFPDestRegs()  const { return staticInst->numFPDestRegs(); }
    int8_t numIntDestRegs() const { return staticInst->numIntDestRegs(); }

    /** Returns the logical register index of the i'th destination register. */
    RegIndex destRegIdx(int i) const
    {
        return staticInst->destRegIdx(i);
    }

    /** Returns the logical register index of the i'th source register. */
    RegIndex srcRegIdx(int i) const
    {
        return staticInst->srcRegIdx(i);
    }

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

    bool isReadySrcRegIdx(int idx) const
    {
        return _readySrcRegIdx[idx];
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

    //Push to .cc file.
    /** Records that one of the source registers is ready. */
    void markSrcRegReady()
    {
        ++readyRegs;
        if(readyRegs == numSrcRegs()) {
            canIssue = true;
        }
    }

    void markSrcRegReady(RegIndex src_idx)
    {
        ++readyRegs;

        _readySrcRegIdx[src_idx] = 1;

        if(readyRegs == numSrcRegs()) {
            canIssue = true;
        }
    }

    /** Sets this instruction as ready to issue. */
    void setCanIssue() { canIssue = true; }

    /** Returns whether or not this instruction is ready to issue. */
    bool readyToIssue() const { return canIssue; }

    /** Sets this instruction as issued from the IQ. */
    void setIssued() { issued = true; }

    /** Returns whether or not this instruction has issued. */
    bool isIssued() { return issued; }

    /** Sets this instruction as executed. */
    void setExecuted() { executed = true; }

    /** Returns whether or not this instruction has executed. */
    bool isExecuted() { return executed; }

    /** Sets this instruction as ready to commit. */
    void setCanCommit() { canCommit = true; }

    /** Clears this instruction as being ready to commit. */
    void clearCanCommit() { canCommit = false; }

    /** Returns whether or not this instruction is ready to commit. */
    bool readyToCommit() const { return canCommit; }

    /** Sets this instruction as squashed. */
    void setSquashed() { squashed = true; }

    /** Returns whether or not this instruction is squashed. */
    bool isSquashed() const { return squashed; }

    /** Sets this instruction as squashed in the IQ. */
    void setSquashedInIQ() { squashedInIQ = true; }

    /** Returns whether or not this instruction is squashed in the IQ. */
    bool isSquashedInIQ() { return squashedInIQ; }

    /** Returns the opclass of this instruction. */
    OpClass opClass() const { return staticInst->opClass(); }

    /** Returns whether or not the BTB missed. */
    bool btbMiss() const { return btbMissed; }

    /** Returns the branch target address. */
    Addr branchTarget() const { return staticInst->branchTarget(PC); }

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

    uint64_t readIntReg(StaticInst<ISA> *si, int idx)
    {
        return cpu->readIntReg(_srcRegIdx[idx]);
    }

    float readFloatRegSingle(StaticInst<ISA> *si, int idx)
    {
        return cpu->readFloatRegSingle(_srcRegIdx[idx]);
    }

    double readFloatRegDouble(StaticInst<ISA> *si, int idx)
    {
        return cpu->readFloatRegDouble(_srcRegIdx[idx]);
    }

    uint64_t readFloatRegInt(StaticInst<ISA> *si, int idx)
    {
        return cpu->readFloatRegInt(_srcRegIdx[idx]);
    }
    /** @todo: Make results into arrays so they can handle multiple dest
     *  registers.
     */
    void setIntReg(StaticInst<ISA> *si, int idx, uint64_t val)
    {
        cpu->setIntReg(_destRegIdx[idx], val);
        intResult = val;
    }

    void setFloatRegSingle(StaticInst<ISA> *si, int idx, float val)
    {
        cpu->setFloatRegSingle(_destRegIdx[idx], val);
        floatResult = val;
    }

    void setFloatRegDouble(StaticInst<ISA> *si, int idx, double val)
    {
        cpu->setFloatRegDouble(_destRegIdx[idx], val);
        doubleResult = val;
    }

    void setFloatRegInt(StaticInst<ISA> *si, int idx, uint64_t val)
    {
        cpu->setFloatRegInt(_destRegIdx[idx], val);
        intResult = val;
    }

    /** Read the PC of this instruction. */
    Addr readPC() { return PC; }

    /** Set the next PC of this instruction (its actual target). */
    void setNextPC(uint64_t val) { nextPC = val; }

//    bool misspeculating() { return cpu->misspeculating(); }
    ExecContext *xcBase() { return xc; }
};

template<class Impl>
template<class T>
inline Fault
BaseDynInst<Impl>::read(Addr addr, T &data, unsigned flags)
{
    MemReqPtr req = new MemReq(addr, xc, sizeof(T), flags);
    req->asid = asid;

    fault = cpu->translateDataReadReq(req);

    // Record key MemReq parameters so we can generate another one
    // just like it for the timing access without calling translate()
    // again (which might mess up the TLB).
    effAddr = req->vaddr;
    physEffAddr = req->paddr;
    memReqFlags = req->flags;

    /**
     * @todo
     * Replace the disjoint functional memory with a unified one and remove
     * this hack.
     */
#ifndef FULL_SYSTEM
    req->paddr = req->vaddr;
#endif

    if (fault == No_Fault) {
        fault = cpu->read(req, data);
    }
    else {
        // Return a fixed value to keep simulation deterministic even
        // along misspeculated paths.
        data = (T)-1;
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

    storeSize = sizeof(T);
    storeData = data;
    if (specMode)
        specMemWrite = true;

    MemReqPtr req = new MemReq(addr, xc, sizeof(T), flags);

    req->asid = asid;

    fault = cpu->translateDataWriteReq(req);

    // Record key MemReq parameters so we can generate another one
    // just like it for the timing access without calling translate()
    // again (which might mess up the TLB).
    effAddr = req->vaddr;
    physEffAddr = req->paddr;
    memReqFlags = req->flags;

    /**
     * @todo
     * Replace the disjoint functional memory with a unified one and remove
     * this hack.
     */
#ifndef FULL_SYSTEM
    req->paddr = req->vaddr;
#endif

    if (fault == No_Fault) {
        fault = cpu->write(req, data);
    }

    if (res) {
        // always return some result to keep misspeculated paths
        // (which will ignore faults) deterministic
        *res = (fault == No_Fault) ? req->result : 0;
    }

    return fault;
}

#endif // __DYN_INST_HH__
