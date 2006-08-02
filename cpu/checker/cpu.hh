/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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

#ifndef __CPU_CHECKER_CPU_HH__
#define __CPU_CHECKER_CPU_HH__

#include <list>
#include <queue>
#include <map>

#include "base/statistics.hh"
#include "config/full_system.hh"
#include "cpu/base.hh"
#include "cpu/base_dyn_inst.hh"
#include "cpu/cpu_exec_context.hh"
#include "cpu/pc_event.hh"
#include "cpu/static_inst.hh"
#include "sim/eventq.hh"

// forward declarations
#if FULL_SYSTEM
class Processor;
class AlphaITB;
class AlphaDTB;
class PhysicalMemory;

class RemoteGDB;
class GDBListener;

#else

class Process;

#endif // FULL_SYSTEM
template <class>
class BaseDynInst;
class ExecContext;
class MemInterface;
class Checkpoint;
class Sampler;

/**
 * CheckerCPU class.  Dynamically verifies instructions as they are
 * completed by making sure that the instruction and its results match
 * the independent execution of the benchmark inside the checker.  The
 * checker verifies instructions in order, regardless of the order in
 * which instructions complete.  There are certain results that can
 * not be verified, specifically the result of a store conditional or
 * the values of uncached accesses.  In these cases, and with
 * instructions marked as "IsUnverifiable", the checker assumes that
 * the value from the main CPU's execution is correct and simply
 * copies that value.  It provides a CheckerExecContext (see
 * checker/exec_context.hh) that provides hooks for updating the
 * Checker's state through any ExecContext accesses.  This allows the
 * checker to be able to correctly verify instructions, even with
 * external accesses to the ExecContext that change state.
 */
class CheckerCPU : public BaseCPU
{
  protected:
    typedef TheISA::MachInst MachInst;
    typedef TheISA::MiscReg MiscReg;
  public:
    virtual void init();

    struct Params : public BaseCPU::Params
    {
#if FULL_SYSTEM
        AlphaITB *itb;
        AlphaDTB *dtb;
        FunctionalMemory *mem;
#else
        Process *process;
#endif
        bool exitOnError;
        bool updateOnError;
    };

  public:
    CheckerCPU(Params *p);
    virtual ~CheckerCPU();

    void setMemory(FunctionalMemory *mem);

    FunctionalMemory *memPtr;

#if FULL_SYSTEM
    void setSystem(System *system);

    System *systemPtr;
#endif
  public:
    // execution context
    CPUExecContext *cpuXC;

    ExecContext *xcProxy;

    AlphaITB *itb;
    AlphaDTB *dtb;

#if FULL_SYSTEM
    Addr dbg_vtophys(Addr addr);
#endif

    union Result {
        uint64_t integer;
        float fp;
        double dbl;
    };

    Result result;

    // current instruction
    MachInst machInst;

    // Refcounted pointer to the one memory request.
    MemReqPtr memReq;

    StaticInstPtr curStaticInst;

    // number of simulated instructions
    Counter numInst;
    Counter startNumInst;

    std::queue<int> miscRegIdxs;

    virtual Counter totalInstructions() const
    {
        return numInst - startNumInst;
    }

    // number of simulated loads
    Counter numLoad;
    Counter startNumLoad;

    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);

    template <class T>
    Fault read(Addr addr, T &data, unsigned flags);

    template <class T>
    Fault write(T data, Addr addr, unsigned flags, uint64_t *res);

    // These functions are only used in CPU models that split
    // effective address computation from the actual memory access.
    void setEA(Addr EA) { panic("SimpleCPU::setEA() not implemented\n"); }
    Addr getEA() 	{ panic("SimpleCPU::getEA() not implemented\n"); }

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

    uint64_t readIntReg(const StaticInst *si, int idx)
    {
        return cpuXC->readIntReg(si->srcRegIdx(idx));
    }

    float readFloatRegSingle(const StaticInst *si, int idx)
    {
        int reg_idx = si->srcRegIdx(idx) - TheISA::FP_Base_DepTag;
        return cpuXC->readFloatRegSingle(reg_idx);
    }

    double readFloatRegDouble(const StaticInst *si, int idx)
    {
        int reg_idx = si->srcRegIdx(idx) - TheISA::FP_Base_DepTag;
        return cpuXC->readFloatRegDouble(reg_idx);
    }

    uint64_t readFloatRegInt(const StaticInst *si, int idx)
    {
        int reg_idx = si->srcRegIdx(idx) - TheISA::FP_Base_DepTag;
        return cpuXC->readFloatRegInt(reg_idx);
    }

    void setIntReg(const StaticInst *si, int idx, uint64_t val)
    {
        cpuXC->setIntReg(si->destRegIdx(idx), val);
        result.integer = val;
    }

    void setFloatRegSingle(const StaticInst *si, int idx, float val)
    {
        int reg_idx = si->destRegIdx(idx) - TheISA::FP_Base_DepTag;
        cpuXC->setFloatRegSingle(reg_idx, val);
        result.fp = val;
    }

    void setFloatRegDouble(const StaticInst *si, int idx, double val)
    {
        int reg_idx = si->destRegIdx(idx) - TheISA::FP_Base_DepTag;
        cpuXC->setFloatRegDouble(reg_idx, val);
        result.dbl = val;
    }

    void setFloatRegInt(const StaticInst *si, int idx, uint64_t val)
    {
        int reg_idx = si->destRegIdx(idx) - TheISA::FP_Base_DepTag;
        cpuXC->setFloatRegInt(reg_idx, val);
        result.integer = val;
    }

    uint64_t readPC() { return cpuXC->readPC(); }
    void setNextPC(uint64_t val) {
        cpuXC->setNextPC(val);
    }

    MiscReg readMiscReg(int misc_reg)
    {
        return cpuXC->readMiscReg(misc_reg);
    }

    MiscReg readMiscRegWithEffect(int misc_reg, Fault &fault)
    {
        return cpuXC->readMiscRegWithEffect(misc_reg, fault);
    }

    Fault setMiscReg(int misc_reg, const MiscReg &val)
    {
        result.integer = val;
        miscRegIdxs.push(misc_reg);
        return cpuXC->setMiscReg(misc_reg, val);
    }

    Fault setMiscRegWithEffect(int misc_reg, const MiscReg &val)
    {
        miscRegIdxs.push(misc_reg);
        return cpuXC->setMiscRegWithEffect(misc_reg, val);
    }

    void recordPCChange(uint64_t val) { changedPC = true; }
    void recordNextPCChange(uint64_t val) { changedNextPC = true; }

    bool translateInstReq(MemReqPtr &req);
    void translateDataWriteReq(MemReqPtr &req);
    void translateDataReadReq(MemReqPtr &req);

#if FULL_SYSTEM
    Fault hwrei() { return cpuXC->hwrei(); }
    int readIntrFlag() { return cpuXC->readIntrFlag(); }
    void setIntrFlag(int val) { cpuXC->setIntrFlag(val); }
    bool inPalMode() { return cpuXC->inPalMode(); }
    void ev5_trap(Fault fault) { fault->invoke(xcProxy); }
    bool simPalCheck(int palFunc) { return cpuXC->simPalCheck(palFunc); }
#else
    // Assume that the normal CPU's call to syscall was successful.
    // The checker's state would have already been updated by the syscall.
    void syscall() { }
#endif

    virtual void handleError() = 0;

    bool checkFlags(MemReqPtr &req);

    ExecContext *xcBase() { return xcProxy; }
    CPUExecContext *cpuXCBase() { return cpuXC; }

    Result unverifiedResult;
    MemReqPtr unverifiedReq;

    bool changedPC;
    bool willChangePC;
    uint64_t newPC;
    bool changedNextPC;
    bool exitOnError;
    bool updateOnError;

    InstSeqNum youngestSN;
};

/**
 * Templated Checker class.  This Checker class is templated on the
 * DynInstPtr of the instruction type that will be verified.  Proper
 * template instantiations of the Checker must be placed at the bottom
 * of checker/cpu.cc.
 */
template <class DynInstPtr>
class Checker : public CheckerCPU
{
  public:
    Checker(Params *p)
        : CheckerCPU(p), updateThisCycle(false), unverifiedInst(NULL)
    { }

    void switchOut(Sampler *s);
    void takeOverFrom(BaseCPU *oldCPU);

    void tick(DynInstPtr &inst);

    void validateInst(DynInstPtr &inst);
    void validateExecution(DynInstPtr &inst);
    void validateState();

    virtual void handleError()
    {
        if (exitOnError)
            panic("Checker found error!");
        else if (updateOnError)
            updateThisCycle = true;
    }

    bool updateThisCycle;

    DynInstPtr unverifiedInst;

    std::list<DynInstPtr> instList;
    typedef typename std::list<DynInstPtr>::iterator InstListIt;
    void dumpInsts();
};

#endif // __CPU_CHECKER_CPU_HH__
