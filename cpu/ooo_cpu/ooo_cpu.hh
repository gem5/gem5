/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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

#ifndef __CPU_OOO_CPU_OOO_CPU_HH__
#define __CPU_OOO_CPU_OOO_CPU_HH__

#include "base/statistics.hh"
#include "cpu/base_cpu.hh"
#include "cpu/exec_context.hh"
#include "cpu/full_cpu/fu_pool.hh"
#include "cpu/ooo_cpu/ea_list.hh"
#include "cpu/pc_event.hh"
#include "cpu/static_inst.hh"
#include "mem/mem_interface.hh"
#include "sim/eventq.hh"

// forward declarations
#ifdef FULL_SYSTEM
class Processor;
class AlphaITB;
class AlphaDTB;
class PhysicalMemory;

class RemoteGDB;
class GDBListener;

#else

class Process;

#endif // FULL_SYSTEM

class Checkpoint;
class MemInterface;

namespace Trace {
    class InstRecord;
}

/**
 * Declaration of Out-of-Order CPU class.  Basically it is a SimpleCPU with
 * simple out-of-order capabilities added to it.  It is still a 1 CPI machine
 * (?), but is capable of handling cache misses.  Basically it models having
 * a ROB/IQ by only allowing a certain amount of instructions to execute while
 * the cache miss is outstanding.
 */

template <class Impl>
class OoOCPU : public BaseCPU
{
  private:
    typedef typename Impl::DynInst DynInst;
    typedef typename Impl::DynInstPtr DynInstPtr;
    typedef typename Impl::ISA ISA;

  public:
    // main simulation loop (one cycle)
    void tick();

  private:
    struct TickEvent : public Event
    {
        OoOCPU *cpu;
        int width;

        TickEvent(OoOCPU *c, int w);
        void process();
        const char *description();
    };

    TickEvent tickEvent;

    /// Schedule tick event, regardless of its current state.
    void scheduleTickEvent(int delay)
    {
        if (tickEvent.squashed())
            tickEvent.reschedule(curTick + delay);
        else if (!tickEvent.scheduled())
            tickEvent.schedule(curTick + delay);
    }

    /// Unschedule tick event, regardless of its current state.
    void unscheduleTickEvent()
    {
        if (tickEvent.scheduled())
            tickEvent.squash();
    }

  private:
    Trace::InstRecord *traceData;

    template<typename T>
    void trace_data(T data);

  public:
    //
    enum Status {
        Running,
        Idle,
        IcacheMissStall,
        IcacheMissComplete,
        DcacheMissStall,
        SwitchedOut
    };

  private:
    Status _status;

  public:
    void post_interrupt(int int_num, int index);

    void zero_fill_64(Addr addr) {
        static int warned = 0;
        if (!warned) {
            warn ("WH64 is not implemented");
            warned = 1;
        }
    };

    struct Params : public BaseCPU::Params
    {
        MemInterface *icache_interface;
        MemInterface *dcache_interface;
        int width;
#ifdef FULL_SYSTEM
        AlphaITB *itb;
        AlphaDTB *dtb;
        FunctionalMemory *mem;
#else
        Process *process;
#endif
        int issueWidth;
    };

    OoOCPU(Params *params);

    virtual ~OoOCPU();

  private:
    void copyFromXC();

  public:
    // execution context
    ExecContext *xc;

    void switchOut();
    void takeOverFrom(BaseCPU *oldCPU);

#ifdef FULL_SYSTEM
    Addr dbg_vtophys(Addr addr);

    bool interval_stats;
#endif

    // L1 instruction cache
    MemInterface *icacheInterface;

    // L1 data cache
    MemInterface *dcacheInterface;

    FuncUnitPool *fuPool;

    // Refcounted pointer to the one memory request.
    MemReqPtr cacheMemReq;

    class ICacheCompletionEvent : public Event
    {
      private:
        OoOCPU *cpu;

      public:
        ICacheCompletionEvent(OoOCPU *_cpu);

        virtual void process();
        virtual const char *description();
    };

    // Will need to create a cache completion event upon any memory miss.
    ICacheCompletionEvent iCacheCompletionEvent;

    class DCacheCompletionEvent : public Event
    {
      private:
        OoOCPU *cpu;
        DynInstPtr inst;

      public:
        DCacheCompletionEvent(OoOCPU *_cpu, DynInstPtr &_inst);

        virtual void process();
        virtual const char *description();
    };

    friend class DCacheCompletionEvent;

    Status status() const { return _status; }

    virtual void activateContext(int thread_num, int delay);
    virtual void suspendContext(int thread_num);
    virtual void deallocateContext(int thread_num);
    virtual void haltContext(int thread_num);

    // statistics
    virtual void regStats();
    virtual void resetStats();

    // number of simulated instructions
    Counter numInst;
    Counter startNumInst;
    Stats::Scalar<> numInsts;

    virtual Counter totalInstructions() const
    {
        return numInst - startNumInst;
    }

    // number of simulated memory references
    Stats::Scalar<> numMemRefs;

    // number of simulated loads
    Counter numLoad;
    Counter startNumLoad;

    // number of idle cycles
    Stats::Average<> notIdleFraction;
    Stats::Formula idleFraction;

    // number of cycles stalled for I-cache misses
    Stats::Scalar<> icacheStallCycles;
    Counter lastIcacheStall;

    // number of cycles stalled for D-cache misses
    Stats::Scalar<> dcacheStallCycles;
    Counter lastDcacheStall;

    void processICacheCompletion();

    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);

#ifdef FULL_SYSTEM
    bool validInstAddr(Addr addr) { return true; }
    bool validDataAddr(Addr addr) { return true; }
    int getInstAsid() { return xc->regs.instAsid(); }
    int getDataAsid() { return xc->regs.dataAsid(); }

    Fault translateInstReq(MemReqPtr &req)
    {
        return itb->translate(req);
    }

    Fault translateDataReadReq(MemReqPtr &req)
    {
        return dtb->translate(req, false);
    }

    Fault translateDataWriteReq(MemReqPtr &req)
    {
        return dtb->translate(req, true);
    }

#else
    bool validInstAddr(Addr addr)
    { return xc->validInstAddr(addr); }

    bool validDataAddr(Addr addr)
    { return xc->validDataAddr(addr); }

    int getInstAsid() { return xc->asid; }
    int getDataAsid() { return xc->asid; }

    Fault dummyTranslation(MemReqPtr &req)
    {
#if 0
        assert((req->vaddr >> 48 & 0xffff) == 0);
#endif

        // put the asid in the upper 16 bits of the paddr
        req->paddr = req->vaddr & ~((Addr)0xffff << sizeof(Addr) * 8 - 16);
        req->paddr = req->paddr | (Addr)req->asid << sizeof(Addr) * 8 - 16;
        return No_Fault;
    }
    Fault translateInstReq(MemReqPtr &req)
    {
        return dummyTranslation(req);
    }
    Fault translateDataReadReq(MemReqPtr &req)
    {
        return dummyTranslation(req);
    }
    Fault translateDataWriteReq(MemReqPtr &req)
    {
        return dummyTranslation(req);
    }

#endif

    template <class T>
    Fault read(Addr addr, T &data, unsigned flags, DynInstPtr inst);

    template <class T>
    Fault write(T data, Addr addr, unsigned flags,
                uint64_t *res, DynInstPtr inst);

    void prefetch(Addr addr, unsigned flags)
    {
        // need to do this...
    }

    void writeHint(Addr addr, int size, unsigned flags)
    {
        // need to do this...
    }

    Fault copySrcTranslate(Addr src);

    Fault copy(Addr dest);

  private:
    bool executeInst(DynInstPtr &inst);

    void renameInst(DynInstPtr &inst);

    void addInst(DynInstPtr &inst);

    void commitHeadInst();

    bool grabInst();

    Fault fetchCacheLine();

    InstSeqNum getAndIncrementInstSeq();

    bool ambigMemAddr;

  private:
    InstSeqNum globalSeqNum;

    DynInstPtr renameTable[ISA::TotalNumRegs];
    DynInstPtr commitTable[ISA::TotalNumRegs];

    // Might need a table of the shadow registers as well.
#ifdef FULL_SYSTEM
    DynInstPtr palShadowTable[ISA::NumIntRegs];
#endif

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

    // In the OoO case these shouldn't read from the XC but rather from the
    // rename table of DynInsts.  Also these likely shouldn't be called very
    // often, other than when adding things into the xc during say a syscall.

    uint64_t readIntReg(StaticInst<TheISA> *si, int idx)
    {
        return xc->readIntReg(si->srcRegIdx(idx));
    }

    float readFloatRegSingle(StaticInst<TheISA> *si, int idx)
    {
        int reg_idx = si->srcRegIdx(idx) - TheISA::FP_Base_DepTag;
        return xc->readFloatRegSingle(reg_idx);
    }

    double readFloatRegDouble(StaticInst<TheISA> *si, int idx)
    {
        int reg_idx = si->srcRegIdx(idx) - TheISA::FP_Base_DepTag;
        return xc->readFloatRegDouble(reg_idx);
    }

    uint64_t readFloatRegInt(StaticInst<TheISA> *si, int idx)
    {
        int reg_idx = si->srcRegIdx(idx) - TheISA::FP_Base_DepTag;
        return xc->readFloatRegInt(reg_idx);
    }

    void setIntReg(StaticInst<TheISA> *si, int idx, uint64_t val)
    {
        xc->setIntReg(si->destRegIdx(idx), val);
    }

    void setFloatRegSingle(StaticInst<TheISA> *si, int idx, float val)
    {
        int reg_idx = si->destRegIdx(idx) - TheISA::FP_Base_DepTag;
        xc->setFloatRegSingle(reg_idx, val);
    }

    void setFloatRegDouble(StaticInst<TheISA> *si, int idx, double val)
    {
        int reg_idx = si->destRegIdx(idx) - TheISA::FP_Base_DepTag;
        xc->setFloatRegDouble(reg_idx, val);
    }

    void setFloatRegInt(StaticInst<TheISA> *si, int idx, uint64_t val)
    {
        int reg_idx = si->destRegIdx(idx) - TheISA::FP_Base_DepTag;
        xc->setFloatRegInt(reg_idx, val);
    }

    uint64_t readPC() { return PC; }
    void setNextPC(Addr val) { nextPC = val; }

  private:
    Addr PC;
    Addr nextPC;

    unsigned issueWidth;

    bool fetchRedirExcp;
    bool fetchRedirBranch;

    /** Mask to get a cache block's address. */
    Addr cacheBlkMask;

    unsigned cacheBlkSize;

    Addr cacheBlkPC;

    /** The cache line being fetched. */
    uint8_t *cacheData;

  protected:
    bool cacheBlkValid;

  private:

    // Align an address (typically a PC) to the start of an I-cache block.
    // We fold in the PISA 64- to 32-bit conversion here as well.
    Addr icacheBlockAlignPC(Addr addr)
    {
        addr = ISA::realPCToFetchPC(addr);
        return (addr & ~(cacheBlkMask));
    }

    unsigned instSize;

    // ROB tracking stuff.
    DynInstPtr robHeadPtr;
    DynInstPtr robTailPtr;
    unsigned robInsts;

    // List of outstanding EA instructions.
  protected:
    EAList eaList;

  public:
    void branchToTarget(Addr val)
    {
        if (!fetchRedirExcp) {
            fetchRedirBranch = true;
            PC = val;
        }
    }

    // ISA stuff:
    uint64_t readUniq() { return xc->readUniq(); }
    void setUniq(uint64_t val) { xc->setUniq(val); }

    uint64_t readFpcr() { return xc->readFpcr(); }
    void setFpcr(uint64_t val) { xc->setFpcr(val); }

#ifdef FULL_SYSTEM
    uint64_t readIpr(int idx, Fault &fault) { return xc->readIpr(idx, fault); }
    Fault setIpr(int idx, uint64_t val) { return xc->setIpr(idx, val); }
    Fault hwrei() { return xc->hwrei(); }
    int readIntrFlag() { return xc->readIntrFlag(); }
    void setIntrFlag(int val) { xc->setIntrFlag(val); }
    bool inPalMode() { return xc->inPalMode(); }
    void ev5_trap(Fault fault) { xc->ev5_trap(fault); }
    bool simPalCheck(int palFunc) { return xc->simPalCheck(palFunc); }
#else
    void syscall() { xc->syscall(); }
#endif

    ExecContext *xcBase() { return xc; }
};


// precise architected memory state accessor macros
template <class Impl>
template <class T>
Fault
OoOCPU<Impl>::read(Addr addr, T &data, unsigned flags, DynInstPtr inst)
{
    MemReqPtr readReq = new MemReq();
    readReq->xc = xc;
    readReq->asid = 0;
    readReq->data = new uint8_t[64];

    readReq->reset(addr, sizeof(T), flags);

    // translate to physical address - This might be an ISA impl call
    Fault fault = translateDataReadReq(readReq);

    // do functional access
    if (fault == No_Fault)
        fault = xc->mem->read(readReq, data);
#if 0
    if (traceData) {
        traceData->setAddr(addr);
        if (fault == No_Fault)
            traceData->setData(data);
    }
#endif

    // if we have a cache, do cache access too
    if (fault == No_Fault && dcacheInterface) {
        readReq->cmd = Read;
        readReq->completionEvent = NULL;
        readReq->time = curTick;
        /*MemAccessResult result = */dcacheInterface->access(readReq);

        if (dcacheInterface->doEvents()) {
            readReq->completionEvent = new DCacheCompletionEvent(this, inst);
            lastDcacheStall = curTick;
            unscheduleTickEvent();
            _status = DcacheMissStall;
        }
    }

    if (!dcacheInterface && (readReq->flags & UNCACHEABLE))
        recordEvent("Uncached Read");

    return fault;
}

template <class Impl>
template <class T>
Fault
OoOCPU<Impl>::write(T data, Addr addr, unsigned flags,
                    uint64_t *res, DynInstPtr inst)
{
    MemReqPtr writeReq = new MemReq();
    writeReq->xc = xc;
    writeReq->asid = 0;
    writeReq->data = new uint8_t[64];

#if 0
    if (traceData) {
        traceData->setAddr(addr);
        traceData->setData(data);
    }
#endif

    writeReq->reset(addr, sizeof(T), flags);

    // translate to physical address
    Fault fault = xc->translateDataWriteReq(writeReq);

    // do functional access
    if (fault == No_Fault)
        fault = xc->write(writeReq, data);

    if (fault == No_Fault && dcacheInterface) {
        writeReq->cmd = Write;
        memcpy(writeReq->data,(uint8_t *)&data,writeReq->size);
        writeReq->completionEvent = NULL;
        writeReq->time = curTick;
        /*MemAccessResult result = */dcacheInterface->access(writeReq);

        if (dcacheInterface->doEvents()) {
            writeReq->completionEvent = new DCacheCompletionEvent(this, inst);
            lastDcacheStall = curTick;
            unscheduleTickEvent();
            _status = DcacheMissStall;
        }
    }

    if (res && (fault == No_Fault))
        *res = writeReq->result;

    if (!dcacheInterface && (writeReq->flags & UNCACHEABLE))
        recordEvent("Uncached Write");

    return fault;
}


#endif // __CPU_OOO_CPU_OOO_CPU_HH__
