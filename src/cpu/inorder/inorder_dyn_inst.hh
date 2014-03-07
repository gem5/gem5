/*
 * Copyright (c) 2007 MIPS Technologies, Inc.
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
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
 *          Korey Sewell
 */

#ifndef __CPU_INORDER_DYN_INST_HH__
#define __CPU_INORDER_DYN_INST_HH__

#include <bitset>
#include <list>
#include <string>

#include "arch/isa_traits.hh"
#include "arch/mt.hh"
#include "arch/types.hh"
#include "arch/utility.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "config/the_isa.hh"
#include "cpu/inorder/inorder_trace.hh"
#include "cpu/inorder/pipeline_traits.hh"
#include "cpu/inorder/resource.hh"
#include "cpu/inorder/resource_sked.hh"
#include "cpu/inorder/thread_state.hh"
#include "cpu/exetrace.hh"
#include "cpu/inst_seq.hh"
#include "cpu/op_class.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "debug/InOrderDynInst.hh"
#include "mem/packet.hh"
#include "sim/fault_fwd.hh"
#include "sim/system.hh"

#if THE_ISA == ALPHA_ISA
#include "arch/alpha/ev5.hh"
#endif

/**
 * @file
 * Defines a dynamic instruction context for a inorder CPU model.
 */

// Forward declaration.
class ResourceRequest;
class Packet;

class InOrderDynInst : public RefCounted
{
  public:
    // Binary machine instruction type.
    typedef TheISA::MachInst MachInst;
    // Extended machine instruction type
    typedef TheISA::ExtMachInst ExtMachInst;
    // Logical register index type.
    typedef TheISA::RegIndex RegIndex;
    // Integer register type.
    typedef TheISA::IntReg IntReg;
    // Floating point register type.
    typedef TheISA::FloatReg FloatReg;
    // Floating point register type.
    typedef TheISA::FloatRegBits FloatRegBits;
    // Condition code register type.
    typedef TheISA::CCReg CCReg;
    // Floating point register type.
    typedef TheISA::MiscReg MiscReg;

    typedef short int PhysRegIndex;

    /** The refcounted DynInst pointer to be used.  In most cases this is
     *  what should be used, and not DynInst*.
     */
    typedef RefCountingPtr<InOrderDynInst> DynInstPtr;

    // The list of instructions iterator type.
    typedef std::list<DynInstPtr>::iterator ListIt;

    enum {
        MaxInstSrcRegs = TheISA::MaxInstSrcRegs,        /// Max source regs
        MaxInstDestRegs = TheISA::MaxInstDestRegs       /// Max dest regs
    };

  public:
    /** BaseDynInst constructor given a binary instruction.
     *  @param seq_num The sequence number of the instruction.
     *  @param cpu Pointer to the instruction's CPU.
     *  NOTE: Must set Binary Instrution through Member Function
     */
    InOrderDynInst(InOrderCPU *cpu, InOrderThreadState *state,
                   InstSeqNum seq_num, ThreadID tid, unsigned asid = 0);

    /** InOrderDynInst destructor. */
    ~InOrderDynInst();

  public:
    /** The sequence number of the instruction. */
    InstSeqNum seqNum;

    /** If this instruction is squashing, the number should we squash behind. */
    InstSeqNum squashSeqNum;

    enum Status {
        RegDepMapEntry,          /// Instruction is entered onto the RegDepMap
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
        RemoveList,               /// Is Instruction on Remove List?
        NumStatus
    };

    /** The status of this BaseDynInst.  Several bits can be set. */
    std::bitset<NumStatus> status;

    /** The thread this instruction is from. */
    short threadNumber;

    /** data address space ID, for loads & stores. */
    short asid;

    /** The virtual processor number */
    short virtProcNumber;

    /** The StaticInst used by this BaseDynInst. */
    StaticInstPtr staticInst;

    /** InstRecord that tracks this instructions. */
    Trace::InOrderTraceRecord *traceData;

    /** Pointer to the Impl's CPU object. */
    InOrderCPU *cpu;

    /** Pointer to the thread state. */
    InOrderThreadState *thread;

    /** The kind of fault this instruction has generated. */
    Fault fault;

    /** The memory request. */
    Request *req;

    /** Pointer to the data for the memory access. */
    uint8_t *memData;

    /**  Data used for a store for operation. */
    uint64_t loadData;

    /**  Data used for a store for operation. */
    uint64_t storeData;

    /** List of active resource requests for this instruction */
    std::list<ResourceRequest*> reqList;

    /** The effective virtual address (lds & stores only). */
    Addr effAddr;

    /** The effective physical address. */
    Addr physEffAddr;

    /** The memory request flags (from translation). */
    unsigned memReqFlags;

    /** How many source registers are ready. */
    unsigned readyRegs;

    enum ResultType {
        None,
        Integer,
        Float,
        FloatBits,
        Double
    };

    /** An instruction src/dest has to be one of these types */
    struct InstValue {
        IntReg intVal;
        union {
            FloatReg f;
            FloatRegBits i;
        } fpVal;

        InstValue()
        {
            intVal = 0;
            fpVal.i = 0;
        }
    };

    /** Result of an instruction execution */
    struct InstResult {
        ResultType type;
        InstValue res;
        Tick tick;

        InstResult()
          : type(None), tick(0)
        { }
    };

    /** The source of the instruction; assumes for now that there's only one
     *  destination register.
     */
    InstValue instSrc[MaxInstSrcRegs];

    /** The result of the instruction; assumes for now that there's only one
     *  destination register.
     */
    InstResult instResult[MaxInstDestRegs];

    /** PC of this instruction. */
    TheISA::PCState pc;

    /** Predicted next PC. */
    TheISA::PCState predPC;

    /** Address to get/write data from/to */
    /* Fetching address when inst. starts, Data address for load/store after fetch*/
    Addr memAddr;

    /** Whether or not the source register is ready.
     *  @todo: Not sure this should be here vs the derived class.
     */
    bool _readySrcRegIdx[MaxInstSrcRegs];

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
    PhysRegIndex _destRegIdx[MaxInstDestRegs];

    /** Physical register index of the source registers of this
     *  instruction.
     */
    PhysRegIndex _srcRegIdx[MaxInstSrcRegs];

    /** Physical register index of the previous producers of the
     *  architected destinations.
     */
    PhysRegIndex _prevDestRegIdx[MaxInstDestRegs];

    int nextStage;

  private:
    /** Function to initialize variables in the constructors. */
    void initVars();

  public:
    Tick memTime;

    PacketDataPtr splitMemData;
    RequestPtr splitMemReq;    
    int totalSize;
    int split2ndSize;
    Addr split2ndAddr;
    bool split2ndAccess;
    uint8_t split2ndData;
    PacketDataPtr split2ndDataPtr;
    unsigned split2ndFlags;
    bool splitInst;
    int splitFinishCnt;
    uint64_t *split2ndStoreDataPtr;    
    bool splitInstSked;

    ////////////////////////////////////////////////////////////
    //
    //  BASE INSTRUCTION INFORMATION.
    //
    ////////////////////////////////////////////////////////////
    std::string instName()
    { return (staticInst) ? staticInst->getName() : "undecoded-inst"; }

    void setStaticInst(StaticInstPtr si);

    ExtMachInst getMachInst() { return staticInst->machInst; }

    /** Sets the StaticInst. */
    void setStaticInst(StaticInstPtr &static_inst);

    /** Sets the sequence number. */
    void setSeqNum(InstSeqNum seq_num) { seqNum = seq_num; }

    /** Sets the ASID. */
    void setASID(short addr_space_id) { asid = addr_space_id; }

    /** Reads the thread id. */
    short readTid() { return threadNumber; }

    /** Sets the thread id. */
    void setTid(ThreadID tid) { threadNumber = tid; }

    void setVpn(int id) { virtProcNumber = id; }

    int readVpn() { return virtProcNumber; }

    /** Sets the pointer to the thread state. */
    void setThreadState(InOrderThreadState *state) { thread = state; }

    /** Returns the thread context. */
    ThreadContext *tcBase() { return thread->getTC(); }

    /** Returns the fault type. */
    Fault getFault() { return fault; }

    /** Read this CPU's ID. */
    int cpuId() const;

    /** Read this context's system-wide ID **/
    int contextId() const { return thread->contextId(); }

    ////////////////////////////////////////////////////////////
    //
    //  INSTRUCTION TYPES -  Forward checks to StaticInst object.
    //
    ////////////////////////////////////////////////////////////
    bool isNop()          const { return staticInst->isNop(); }
    bool isMemRef()       const { return staticInst->isMemRef(); }
    bool isLoad()         const { return staticInst->isLoad(); }
    bool isStore()        const { return staticInst->isStore(); }
    bool isStoreConditional() const
    { return staticInst->isStoreConditional(); }
    bool isInstPrefetch() const { return staticInst->isInstPrefetch(); }
    bool isDataPrefetch() const { return staticInst->isDataPrefetch(); }
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
    bool isMemBarrier()   const { return staticInst->isMemBarrier(); }
    bool isWriteBarrier() const { return staticInst->isWriteBarrier(); }
    bool isNonSpeculative() const { return staticInst->isNonSpeculative(); }
    bool isQuiesce() const { return staticInst->isQuiesce(); }
    bool isIprAccess() const { return staticInst->isIprAccess(); }
    bool isUnverifiable() const { return staticInst->isUnverifiable(); }
    bool isSyscall() const
    { return staticInst->isSyscall(); }
    bool isMicroop() const { return staticInst->isMicroop(); }
    bool isLastMicroop() const { return staticInst->isLastMicroop(); }


    /////////////////////////////////////////////
    //
    // RESOURCE SCHEDULING
    //
    /////////////////////////////////////////////
    typedef ThePipeline::RSkedPtr RSkedPtr;
    bool inFrontEnd;

    RSkedPtr frontSked;
    RSkedIt frontSked_end;

    RSkedPtr backSked;
    RSkedIt backSked_end;

    RSkedIt curSkedEntry;

    void setFrontSked(RSkedPtr front_sked)
    {
        frontSked = front_sked;
        frontSked_end.init(frontSked);
        frontSked_end = frontSked->end();
        //DPRINTF(InOrderDynInst, "Set FrontSked End to : %x \n" ,
        //        frontSked_end.getIt()/*, frontSked->end()*/);
        //assert(frontSked_end == frontSked->end());

        // This initializes instruction to be able
        // to walk the resource schedule
        curSkedEntry.init(frontSked);
        curSkedEntry = frontSked->begin();
    }

    void setBackSked(RSkedPtr back_sked)
    {
        backSked = back_sked;
        backSked_end.init(backSked);
        backSked_end = backSked->end();
    }

    void setNextStage(int stage_num) { nextStage = stage_num; }
    int getNextStage() { return nextStage; }

    /** Print Resource Schedule */
    void printSked()
    {
        if (frontSked != NULL) {
            frontSked->print();
        }

        if (backSked != NULL) {
            backSked->print();
        }
    }

    /** Return Next Resource Stage To Be Used */
    int nextResStage()
    {
        assert((inFrontEnd && curSkedEntry != frontSked_end) ||
               (!inFrontEnd && curSkedEntry != backSked_end));

        return curSkedEntry->stageNum;
    }


    /** Return Next Resource To Be Used */
    int nextResource()
    {
        assert((inFrontEnd && curSkedEntry != frontSked_end) ||
               (!inFrontEnd && curSkedEntry != backSked_end));

        return curSkedEntry->resNum;
    }

    /** Finish using a schedule entry, increment to next entry */
    bool finishSkedEntry()
    {
        curSkedEntry++;

        if (inFrontEnd && curSkedEntry == frontSked_end) {
            DPRINTF(InOrderDynInst, "[sn:%i] Switching to "
                  "back end schedule.\n", seqNum);
          assert(backSked != NULL);
          curSkedEntry.init(backSked);
          curSkedEntry = backSked->begin();
          inFrontEnd = false;
        } else if (!inFrontEnd && curSkedEntry == backSked_end) {
            return true;
        }

        DPRINTF(InOrderDynInst, "[sn:%i] Next Stage: %i "
                "Next Resource: %i.\n", seqNum, curSkedEntry->stageNum,
                curSkedEntry->resNum);

        return false;
    }

    /** Release a Resource Request (Currently Unused) */
    void releaseReq(ResourceRequest* req);

    ////////////////////////////////////////////
    //
    // INSTRUCTION EXECUTION
    //
    ////////////////////////////////////////////
    /** Returns the opclass of this instruction. */
    OpClass opClass() const { return staticInst->opClass(); }

    /** Executes the instruction.*/
    Fault execute();

    unsigned curResSlot;

    unsigned getCurResSlot() { return curResSlot; }

    void setCurResSlot(unsigned slot_num) { curResSlot = slot_num; }

    /** Calls a syscall. */
    /** Calls hardware return from error interrupt. */
    Fault hwrei();
    /** Traps to handle specified fault. */
    void trap(Fault fault);
    bool simPalCheck(int palFunc);
    short syscallNum;

    /** Emulates a syscall. */
    void syscall(int64_t callnum);

    ////////////////////////////////////////////////////////////
    //
    // MULTITHREADING INTERFACE TO CPU MODELS
    //
    ////////////////////////////////////////////////////////////
    virtual void deallocateContext(int thread_num);

    ////////////////////////////////////////////////////////////
    //
    //  PROGRAM COUNTERS - PC/NPC/NPC
    //
    ////////////////////////////////////////////////////////////
    /** Read the PC of this instruction. */
    const TheISA::PCState &pcState() const { return pc; }

    /** Sets the PC of this instruction. */
    void pcState(const TheISA::PCState &_pc) { pc = _pc; }

    const Addr instAddr() { return pc.instAddr(); }
    const Addr nextInstAddr() { return pc.nextInstAddr(); }
    const MicroPC microPC() { return pc.microPC(); }

    ////////////////////////////////////////////////////////////
    //
    // BRANCH PREDICTION
    //
    ////////////////////////////////////////////////////////////
    /** Set the predicted target of this current instruction. */
    void setPredTarg(const TheISA::PCState &predictedPC)
    { predPC = predictedPC; }

    /** Returns the predicted target of the branch. */
    TheISA::PCState readPredTarg() { return predPC; }

    /** Returns the predicted PC immediately after the branch. */
    Addr predInstAddr() { return predPC.instAddr(); }

    /** Returns the predicted PC two instructions after the branch */
    Addr predNextInstAddr() { return predPC.nextInstAddr(); }

    /** Returns the predicted micro PC after the branch */
    Addr readPredMicroPC() { return predPC.microPC(); }

    /** Returns whether the instruction was predicted taken or not. */
    bool predTaken() { return predictTaken; }

    /** Returns whether the instruction mispredicted. */
    bool
    mispredicted()
    {
        TheISA::PCState nextPC = pc;
        TheISA::advancePC(nextPC, staticInst);
        return !(nextPC == predPC);
    }

    /** Returns the branch target address. */
    TheISA::PCState branchTarget() const
    { return staticInst->branchTarget(pc); }

    /** Checks whether or not this instruction has had its branch target
     *  calculated yet.  For now it is not utilized and is hacked to be
     *  always false.
     *  @todo: Actually use this instruction.
     */
    bool doneTargCalc() { return false; }

    void setBranchPred(bool prediction) { predictTaken = prediction; }

    int squashingStage;

    bool predictTaken;

    bool procDelaySlotOnMispred;

    void setSquashInfo(unsigned stage_num);

    ////////////////////////////////////////////
    //
    // MEMORY ACCESS
    //
    ////////////////////////////////////////////

    Fault readMem(Addr addr, uint8_t *data, unsigned size, unsigned flags);

    Fault writeMem(uint8_t *data, unsigned size,
                   Addr addr, unsigned flags, uint64_t *res);

    /** Initiates a memory access - Calculate Eff. Addr & Initiate Memory
     *  Access Only valid for memory operations.
     */
    Fault initiateAcc();

    /** Completes a memory access - Only valid for memory operations. */
    Fault completeAcc(Packet *pkt);

    /** Calculates Eff. Addr.  part of a memory instruction.  */
    Fault calcEA();

    /** Read Effective Address from instruction & do memory access */
    Fault memAccess();

    RequestPtr fetchMemReq;
    RequestPtr dataMemReq;

    bool memAddrReady;

    bool validMemAddr()
    { return memAddrReady; }

    void setMemAddr(Addr addr)
    { memAddr = addr; memAddrReady = true;}

    void unsetMemAddr()
    { memAddrReady = false;}

    Addr getMemAddr()
    { return memAddr; }

    /** Sets the effective address. */
    void setEA(Addr &ea) { instEffAddr = ea; eaCalcDone = true; }

    /** Returns the effective address. */
    const Addr &getEA() const { return instEffAddr; }

    /** Returns whether or not the eff. addr. calculation has been completed.*/
    bool doneEACalc() { return eaCalcDone; }

    /** Returns whether or not the eff. addr. source registers are ready.
     *  Assume that src registers 1..n-1 are the ones that the
     *  EA calc depends on.  (i.e. src reg 0 is the source of the data to be
     *  stored)
     */
    bool eaSrcsReady()
    {
        for (int i = 1; i < numSrcRegs(); ++i) {
            if (!_readySrcRegIdx[i])
                return false;
        }

        return true;
    }

    //////////////////////////////////////////////////
    //
    // SOURCE-DESTINATION REGISTER INDEXING
    //
    //////////////////////////////////////////////////
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

    //////////////////////////////////////////////////
    //
    // RENAME/PHYSICAL REGISTER FILE SUPPORT
    //
    //////////////////////////////////////////////////
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

    /** Returns if a source register is ready. */
    bool isReadySrcRegIdx(int idx) const
    {
        return this->_readySrcRegIdx[idx];
    }

    /** Records that one of the source registers is ready. */
    void markSrcRegReady()
    {
        if (++readyRegs == numSrcRegs()) {
            status.set(CanIssue);
        }
    }

    /** Marks a specific register as ready. */
    void markSrcRegReady(RegIndex src_idx)
    {
        _readySrcRegIdx[src_idx] = true;

        markSrcRegReady();
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


    PhysRegIndex readDestRegIdx(int idx)
    {
        return _destRegIdx[idx];
    }

    void setDestRegIdx(int idx, PhysRegIndex dest_idx)
    {
        _destRegIdx[idx] = dest_idx;
    }

    int getDestIdxNum(PhysRegIndex dest_idx)
    {
        for (int i=0; i < staticInst->numDestRegs(); i++) {
            if (_flatDestRegIdx[i] == dest_idx)
                return i;
        }

        return -1;
    }

    PhysRegIndex readSrcRegIdx(int idx)
    {
        return _srcRegIdx[idx];
    }

    void setSrcRegIdx(int idx, PhysRegIndex src_idx)
    {
        _srcRegIdx[idx] = src_idx;
    }

    int getSrcIdxNum(PhysRegIndex src_idx)
    {
        for (int i=0; i < staticInst->numSrcRegs(); i++) {
            if (_srcRegIdx[i] == src_idx)
                return i;
        }

        return -1;
    }

    ////////////////////////////////////////////////////
    //
    // SOURCE-DESTINATION REGISTER VALUES
    //
    ////////////////////////////////////////////////////

    /** Functions that sets an integer or floating point
     *  source register to a value. */
    void setIntSrc(int idx, uint64_t val);
    void setFloatSrc(int idx, FloatReg val);
    void setFloatRegBitsSrc(int idx, TheISA::FloatRegBits val);

    TheISA::IntReg* getIntSrcPtr(int idx) { return &instSrc[idx].intVal; }
    uint64_t readIntSrc(int idx) { return instSrc[idx].intVal; }

    /** These Instructions read a integer/float/misc. source register
     *  value in the instruction. The instruction's execute function will
     *  call these and it is the interface that is used by the ISA descr.
     *  language (which is why the name isnt readIntSrc(...)) Note: That
     *  the source reg. value is set using the setSrcReg() function.
     */
    IntReg readIntRegOperand(const StaticInst *si, int idx, ThreadID tid = 0);
    FloatReg readFloatRegOperand(const StaticInst *si, int idx);
    TheISA::FloatRegBits readFloatRegOperandBits(const StaticInst *si, int idx);
    MiscReg readMiscReg(int misc_reg);
    MiscReg readMiscRegNoEffect(int misc_reg);
    MiscReg readMiscRegOperand(const StaticInst *si, int idx);
    MiscReg readMiscRegOperandNoEffect(const StaticInst *si, int idx);

    /** Returns the result value instruction. */
    ResultType resultType(int idx)
    {
        return instResult[idx].type;
    }

    IntReg readIntResult(int idx)
    {
        return instResult[idx].res.intVal;
    }

    FloatReg readFloatResult(int idx)
    {
       return instResult[idx].res.fpVal.f;
    }

    FloatRegBits readFloatBitsResult(int idx)
    {
       return instResult[idx].res.fpVal.i;
    }

    CCReg readCCResult(int idx)
    {
       return instResult[idx].res.intVal;
    }

    Tick readResultTime(int idx) { return instResult[idx].tick; }

    IntReg* getIntResultPtr(int idx) { return &instResult[idx].res.intVal; }

    /** This is the interface that an instruction will use to write
     *  it's destination register.
     */
    void setIntRegOperand(const StaticInst *si, int idx, IntReg val);
    void setFloatRegOperand(const StaticInst *si, int idx, FloatReg val);
    void setFloatRegOperandBits(const StaticInst *si, int idx,
            TheISA::FloatRegBits val);
    void setCCRegOperand(const StaticInst *si, int idx, CCReg val);
    void setMiscReg(int misc_reg, const MiscReg &val);
    void setMiscRegNoEffect(int misc_reg, const MiscReg &val);
    void setMiscRegOperand(const StaticInst *si, int idx, const MiscReg &val);
    void setMiscRegOperandNoEffect(const StaticInst *si, int idx,
                                   const MiscReg &val);

    virtual uint64_t readRegOtherThread(unsigned idx,
                                        ThreadID tid = InvalidThreadID);
    virtual void setRegOtherThread(unsigned idx, const uint64_t &val,
                                   ThreadID tid = InvalidThreadID);

    /** Returns the number of consecutive store conditional failures. */
    unsigned readStCondFailures()
    { return thread->storeCondFailures; }

    /** Sets the number of consecutive store conditional failures. */
    void setStCondFailures(unsigned sc_failures)
    { thread->storeCondFailures = sc_failures; }

    //////////////////////////////////////////////////////////////
    //
    // INSTRUCTION STATUS FLAGS (READ/SET)
    //
    //////////////////////////////////////////////////////////////
    /** Sets this instruction as entered on the CPU Reg Dep Map */
    void setRegDepEntry() { status.set(RegDepMapEntry); }

    /** Unsets this instruction as entered on the CPU Reg Dep Map */
    void clearRegDepEntry() { status.reset(RegDepMapEntry); }

    /** Returns whether or not the entry is on the CPU Reg Dep Map */
    bool isRegDepEntry() const { return status[RegDepMapEntry]; }

    /** Sets this instruction as entered on the CPU Reg Dep Map */
    void setRemoveList() { status.set(RemoveList); }

    /** Returns whether or not the entry is on the CPU Reg Dep Map */
    bool isRemoveList() const { return status[RemoveList]; }

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

    /** Sets this instruction as issued from the IQ. */
    void setIssued() { status.set(Issued); }

    /** Returns whether or not this instruction has issued. */
    bool isIssued() const { return status[Issued]; }

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
    /** Load queue index. */
    int16_t lqIdx;

    /** Store queue index. */
    int16_t sqIdx;

    /** Iterator pointing to this BaseDynInst in the list of all insts. */
    ListIt instListIt;

    bool onInstList;

    /** Returns iterator to this instruction in the list of all insts. */
    ListIt getInstListIt() { return instListIt; }

    /** Sets iterator for this instruction in the list of all insts. */
    void setInstListIt(ListIt _instListIt) { onInstList = true; instListIt = _instListIt; }

    /** Count of total number of dynamic instructions. */
    static int instcount;

    void resetInstCount();
    
    /** Dumps out contents of this BaseDynInst. */
    void dump();

    /** Dumps out contents of this BaseDynInst into given string. */
    void dump(std::string &outstring);

    //inline int curCount() { return curCount(); }
};


#endif // __CPU_BASE_DYN_INST_HH__
