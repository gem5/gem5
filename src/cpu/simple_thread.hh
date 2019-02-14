/*
 * Copyright (c) 2011-2012, 2016-2018 ARM Limited
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
 * Copyright (c) 2001-2006 The Regents of The University of Michigan
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
 * Authors: Steve Reinhardt
 *          Nathan Binkert
 */

#ifndef __CPU_SIMPLE_THREAD_HH__
#define __CPU_SIMPLE_THREAD_HH__

#include "arch/decoder.hh"
#include "arch/generic/tlb.hh"
#include "arch/isa.hh"
#include "arch/isa_traits.hh"
#include "arch/registers.hh"
#include "arch/types.hh"
#include "base/types.hh"
#include "config/the_isa.hh"
#include "cpu/thread_context.hh"
#include "cpu/thread_state.hh"
#include "debug/CCRegs.hh"
#include "debug/FloatRegs.hh"
#include "debug/IntRegs.hh"
#include "debug/VecPredRegs.hh"
#include "debug/VecRegs.hh"
#include "mem/page_table.hh"
#include "mem/request.hh"
#include "sim/byteswap.hh"
#include "sim/eventq.hh"
#include "sim/process.hh"
#include "sim/serialize.hh"
#include "sim/system.hh"

class BaseCPU;
class CheckerCPU;

class FunctionProfile;
class ProfileNode;

namespace TheISA {
    namespace Kernel {
        class Statistics;
    }
}

/**
 * The SimpleThread object provides a combination of the ThreadState
 * object and the ThreadContext interface. It implements the
 * ThreadContext interface so that a ProxyThreadContext class can be
 * made using SimpleThread as the template parameter (see
 * thread_context.hh). It adds to the ThreadState object by adding all
 * the objects needed for simple functional execution, including a
 * simple architectural register file, and pointers to the ITB and DTB
 * in full system mode. For CPU models that do not need more advanced
 * ways to hold state (i.e. a separate physical register file, or
 * separate fetch and commit PC's), this SimpleThread class provides
 * all the necessary state for full architecture-level functional
 * simulation.  See the AtomicSimpleCPU or TimingSimpleCPU for
 * examples.
 */

class SimpleThread : public ThreadState
{
  protected:
    typedef TheISA::MachInst MachInst;
    using VecRegContainer = TheISA::VecRegContainer;
    using VecElem = TheISA::VecElem;
    using VecPredRegContainer = TheISA::VecPredRegContainer;
  public:
    typedef ThreadContext::Status Status;

  protected:
    RegVal floatRegs[TheISA::NumFloatRegs];
    RegVal intRegs[TheISA::NumIntRegs];
    VecRegContainer vecRegs[TheISA::NumVecRegs];
    VecPredRegContainer vecPredRegs[TheISA::NumVecPredRegs];
#ifdef ISA_HAS_CC_REGS
    RegVal ccRegs[TheISA::NumCCRegs];
#endif
    TheISA::ISA *const isa;    // one "instance" of the current ISA.

    TheISA::PCState _pcState;

    /** Did this instruction execute or is it predicated false */
    bool predicate;

  public:
    std::string name() const
    {
        return csprintf("%s.[tid:%i]", baseCpu->name(), tc->threadId());
    }

    ProxyThreadContext<SimpleThread> *tc;

    System *system;

    BaseTLB *itb;
    BaseTLB *dtb;

    TheISA::Decoder decoder;

    // constructor: initialize SimpleThread from given process structure
    // FS
    SimpleThread(BaseCPU *_cpu, int _thread_num, System *_system,
                 BaseTLB *_itb, BaseTLB *_dtb, TheISA::ISA *_isa,
                 bool use_kernel_stats = true);
    // SE
    SimpleThread(BaseCPU *_cpu, int _thread_num, System *_system,
                 Process *_process, BaseTLB *_itb, BaseTLB *_dtb,
                 TheISA::ISA *_isa);

    virtual ~SimpleThread();

    virtual void takeOverFrom(ThreadContext *oldContext);

    void regStats(const std::string &name);

    void copyState(ThreadContext *oldContext);

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
    void startup();

    /***************************************************************
     *  SimpleThread functions to provide CPU with access to various
     *  state.
     **************************************************************/

    /** Returns the pointer to this SimpleThread's ThreadContext. Used
     *  when a ThreadContext must be passed to objects outside of the
     *  CPU.
     */
    ThreadContext *getTC() { return tc; }

    void demapPage(Addr vaddr, uint64_t asn)
    {
        itb->demapPage(vaddr, asn);
        dtb->demapPage(vaddr, asn);
    }

    void demapInstPage(Addr vaddr, uint64_t asn)
    {
        itb->demapPage(vaddr, asn);
    }

    void demapDataPage(Addr vaddr, uint64_t asn)
    {
        dtb->demapPage(vaddr, asn);
    }

    void dumpFuncProfile();

    Fault hwrei();

    bool simPalCheck(int palFunc);

    /*******************************************
     * ThreadContext interface functions.
     ******************************************/

    BaseCPU *getCpuPtr() { return baseCpu; }

    BaseTLB *getITBPtr() { return itb; }

    BaseTLB *getDTBPtr() { return dtb; }

    CheckerCPU *getCheckerCpuPtr() { return NULL; }

    TheISA::ISA *getIsaPtr() { return isa; }

    TheISA::Decoder *getDecoderPtr() { return &decoder; }

    System *getSystemPtr() { return system; }

    Status status() const { return _status; }

    void setStatus(Status newStatus) { _status = newStatus; }

    /// Set the status to Active.
    void activate();

    /// Set the status to Suspended.
    void suspend();

    /// Set the status to Halted.
    void halt();

    void copyArchRegs(ThreadContext *tc);

    void clearArchRegs()
    {
        _pcState = 0;
        memset(intRegs, 0, sizeof(intRegs));
        memset(floatRegs, 0, sizeof(floatRegs));
        for (int i = 0; i < TheISA::NumVecRegs; i++) {
            vecRegs[i].zero();
        }
        for (int i = 0; i < TheISA::NumVecPredRegs; i++) {
            vecPredRegs[i].reset();
        }
#ifdef ISA_HAS_CC_REGS
        memset(ccRegs, 0, sizeof(ccRegs));
#endif
        isa->clear();
    }

    //
    // New accessors for new decoder.
    //
    RegVal
    readIntReg(int reg_idx)
    {
        int flatIndex = isa->flattenIntIndex(reg_idx);
        assert(flatIndex < TheISA::NumIntRegs);
        uint64_t regVal(readIntRegFlat(flatIndex));
        DPRINTF(IntRegs, "Reading int reg %d (%d) as %#x.\n",
                reg_idx, flatIndex, regVal);
        return regVal;
    }

    RegVal
    readFloatReg(int reg_idx)
    {
        int flatIndex = isa->flattenFloatIndex(reg_idx);
        assert(flatIndex < TheISA::NumFloatRegs);
        RegVal regVal(readFloatRegFlat(flatIndex));
        DPRINTF(FloatRegs, "Reading float reg %d (%d) bits as %#x.\n",
                reg_idx, flatIndex, regVal);
        return regVal;
    }

    const VecRegContainer&
    readVecReg(const RegId& reg) const
    {
        int flatIndex = isa->flattenVecIndex(reg.index());
        assert(flatIndex < TheISA::NumVecRegs);
        const VecRegContainer& regVal = readVecRegFlat(flatIndex);
        DPRINTF(VecRegs, "Reading vector reg %d (%d) as %s.\n",
                reg.index(), flatIndex, regVal.print());
        return regVal;
    }

    VecRegContainer&
    getWritableVecReg(const RegId& reg)
    {
        int flatIndex = isa->flattenVecIndex(reg.index());
        assert(flatIndex < TheISA::NumVecRegs);
        VecRegContainer& regVal = getWritableVecRegFlat(flatIndex);
        DPRINTF(VecRegs, "Reading vector reg %d (%d) as %s for modify.\n",
                reg.index(), flatIndex, regVal.print());
        return regVal;
    }

    /** Vector Register Lane Interfaces. */
    /** @{ */
    /** Reads source vector <T> operand. */
    template <typename T>
    VecLaneT<T, true>
    readVecLane(const RegId& reg) const
    {
        int flatIndex = isa->flattenVecIndex(reg.index());
        assert(flatIndex < TheISA::NumVecRegs);
        auto regVal = readVecLaneFlat<T>(flatIndex, reg.elemIndex());
        DPRINTF(VecRegs, "Reading vector lane %d (%d)[%d] as %lx.\n",
                reg.index(), flatIndex, reg.elemIndex(), regVal);
        return regVal;
    }

    /** Reads source vector 8bit operand. */
    virtual ConstVecLane8
    readVec8BitLaneReg(const RegId& reg) const
    { return readVecLane<uint8_t>(reg); }

    /** Reads source vector 16bit operand. */
    virtual ConstVecLane16
    readVec16BitLaneReg(const RegId& reg) const
    { return readVecLane<uint16_t>(reg); }

    /** Reads source vector 32bit operand. */
    virtual ConstVecLane32
    readVec32BitLaneReg(const RegId& reg) const
    { return readVecLane<uint32_t>(reg); }

    /** Reads source vector 64bit operand. */
    virtual ConstVecLane64
    readVec64BitLaneReg(const RegId& reg) const
    { return readVecLane<uint64_t>(reg); }

    /** Write a lane of the destination vector register. */
    template <typename LD>
    void setVecLaneT(const RegId& reg, const LD& val)
    {
        int flatIndex = isa->flattenVecIndex(reg.index());
        assert(flatIndex < TheISA::NumVecRegs);
        setVecLaneFlat(flatIndex, reg.elemIndex(), val);
        DPRINTF(VecRegs, "Reading vector lane %d (%d)[%d] to %lx.\n",
                reg.index(), flatIndex, reg.elemIndex(), val);
    }
    virtual void setVecLane(const RegId& reg,
            const LaneData<LaneSize::Byte>& val)
    { return setVecLaneT(reg, val); }
    virtual void setVecLane(const RegId& reg,
            const LaneData<LaneSize::TwoByte>& val)
    { return setVecLaneT(reg, val); }
    virtual void setVecLane(const RegId& reg,
            const LaneData<LaneSize::FourByte>& val)
    { return setVecLaneT(reg, val); }
    virtual void setVecLane(const RegId& reg,
            const LaneData<LaneSize::EightByte>& val)
    { return setVecLaneT(reg, val); }
    /** @} */

    const VecElem& readVecElem(const RegId& reg) const
    {
        int flatIndex = isa->flattenVecElemIndex(reg.index());
        assert(flatIndex < TheISA::NumVecRegs);
        const VecElem& regVal = readVecElemFlat(flatIndex, reg.elemIndex());
        DPRINTF(VecRegs, "Reading element %d of vector reg %d (%d) as"
                " %#x.\n", reg.elemIndex(), reg.index(), flatIndex, regVal);
        return regVal;
    }

    const VecPredRegContainer&
    readVecPredReg(const RegId& reg) const
    {
        int flatIndex = isa->flattenVecPredIndex(reg.index());
        assert(flatIndex < TheISA::NumVecPredRegs);
        const VecPredRegContainer& regVal = readVecPredRegFlat(flatIndex);
        DPRINTF(VecPredRegs, "Reading predicate reg %d (%d) as %s.\n",
                reg.index(), flatIndex, regVal.print());
        return regVal;
    }

    VecPredRegContainer&
    getWritableVecPredReg(const RegId& reg)
    {
        int flatIndex = isa->flattenVecPredIndex(reg.index());
        assert(flatIndex < TheISA::NumVecPredRegs);
        VecPredRegContainer& regVal = getWritableVecPredRegFlat(flatIndex);
        DPRINTF(VecPredRegs,
                "Reading predicate reg %d (%d) as %s for modify.\n",
                reg.index(), flatIndex, regVal.print());
        return regVal;
    }

    RegVal
    readCCReg(int reg_idx)
    {
#ifdef ISA_HAS_CC_REGS
        int flatIndex = isa->flattenCCIndex(reg_idx);
        assert(0 <= flatIndex);
        assert(flatIndex < TheISA::NumCCRegs);
        uint64_t regVal(readCCRegFlat(flatIndex));
        DPRINTF(CCRegs, "Reading CC reg %d (%d) as %#x.\n",
                reg_idx, flatIndex, regVal);
        return regVal;
#else
        panic("Tried to read a CC register.");
        return 0;
#endif
    }

    void
    setIntReg(int reg_idx, RegVal val)
    {
        int flatIndex = isa->flattenIntIndex(reg_idx);
        assert(flatIndex < TheISA::NumIntRegs);
        DPRINTF(IntRegs, "Setting int reg %d (%d) to %#x.\n",
                reg_idx, flatIndex, val);
        setIntRegFlat(flatIndex, val);
    }

    void
    setFloatReg(int reg_idx, RegVal val)
    {
        int flatIndex = isa->flattenFloatIndex(reg_idx);
        assert(flatIndex < TheISA::NumFloatRegs);
        // XXX: Fix array out of bounds compiler error for gem5.fast
        // when checkercpu enabled
        if (flatIndex < TheISA::NumFloatRegs)
            setFloatRegFlat(flatIndex, val);
        DPRINTF(FloatRegs, "Setting float reg %d (%d) bits to %#x.\n",
                reg_idx, flatIndex, val);
    }

    void
    setVecReg(const RegId& reg, const VecRegContainer& val)
    {
        int flatIndex = isa->flattenVecIndex(reg.index());
        assert(flatIndex < TheISA::NumVecRegs);
        setVecRegFlat(flatIndex, val);
        DPRINTF(VecRegs, "Setting vector reg %d (%d) to %s.\n",
                reg.index(), flatIndex, val.print());
    }

    void
    setVecElem(const RegId& reg, const VecElem& val)
    {
        int flatIndex = isa->flattenVecElemIndex(reg.index());
        assert(flatIndex < TheISA::NumVecRegs);
        setVecElemFlat(flatIndex, reg.elemIndex(), val);
        DPRINTF(VecRegs, "Setting element %d of vector reg %d (%d) to"
                " %#x.\n", reg.elemIndex(), reg.index(), flatIndex, val);
    }

    void
    setVecPredReg(const RegId& reg, const VecPredRegContainer& val)
    {
        int flatIndex = isa->flattenVecPredIndex(reg.index());
        assert(flatIndex < TheISA::NumVecPredRegs);
        setVecPredRegFlat(flatIndex, val);
        DPRINTF(VecPredRegs, "Setting predicate reg %d (%d) to %s.\n",
                reg.index(), flatIndex, val.print());
    }

    void
    setCCReg(int reg_idx, RegVal val)
    {
#ifdef ISA_HAS_CC_REGS
        int flatIndex = isa->flattenCCIndex(reg_idx);
        assert(flatIndex < TheISA::NumCCRegs);
        DPRINTF(CCRegs, "Setting CC reg %d (%d) to %#x.\n",
                reg_idx, flatIndex, val);
        setCCRegFlat(flatIndex, val);
#else
        panic("Tried to set a CC register.");
#endif
    }

    TheISA::PCState
    pcState()
    {
        return _pcState;
    }

    void
    pcState(const TheISA::PCState &val)
    {
        _pcState = val;
    }

    void
    pcStateNoRecord(const TheISA::PCState &val)
    {
        _pcState = val;
    }

    Addr
    instAddr()
    {
        return _pcState.instAddr();
    }

    Addr
    nextInstAddr()
    {
        return _pcState.nextInstAddr();
    }

    void
    setNPC(Addr val)
    {
        _pcState.setNPC(val);
    }

    MicroPC
    microPC()
    {
        return _pcState.microPC();
    }

    bool readPredicate()
    {
        return predicate;
    }

    void setPredicate(bool val)
    {
        predicate = val;
    }

    RegVal
    readMiscRegNoEffect(int misc_reg, ThreadID tid=0) const
    {
        return isa->readMiscRegNoEffect(misc_reg);
    }

    RegVal
    readMiscReg(int misc_reg, ThreadID tid=0)
    {
        return isa->readMiscReg(misc_reg, tc);
    }

    void
    setMiscRegNoEffect(int misc_reg, RegVal val, ThreadID tid = 0)
    {
        return isa->setMiscRegNoEffect(misc_reg, val);
    }

    void
    setMiscReg(int misc_reg, RegVal val, ThreadID tid = 0)
    {
        return isa->setMiscReg(misc_reg, val, tc);
    }

    RegId
    flattenRegId(const RegId& regId) const
    {
        return isa->flattenRegId(regId);
    }

    unsigned readStCondFailures() { return storeCondFailures; }

    void setStCondFailures(unsigned sc_failures)
    { storeCondFailures = sc_failures; }

    void
    syscall(int64_t callnum, Fault *fault)
    {
        process->syscall(callnum, tc, fault);
    }

    RegVal readIntRegFlat(int idx) { return intRegs[idx]; }
    void setIntRegFlat(int idx, RegVal val) { intRegs[idx] = val; }

    RegVal readFloatRegFlat(int idx) { return floatRegs[idx]; }
    void setFloatRegFlat(int idx, RegVal val) { floatRegs[idx] = val; }

    const VecRegContainer &
    readVecRegFlat(const RegIndex& reg) const
    {
        return vecRegs[reg];
    }

    VecRegContainer &
    getWritableVecRegFlat(const RegIndex& reg)
    {
        return vecRegs[reg];
    }

    void
    setVecRegFlat(const RegIndex& reg, const VecRegContainer& val)
    {
        vecRegs[reg] = val;
    }

    template <typename T>
    VecLaneT<T, true>
    readVecLaneFlat(const RegIndex& reg, int lId) const
    {
        return vecRegs[reg].laneView<T>(lId);
    }

    template <typename LD>
    void
    setVecLaneFlat(const RegIndex& reg, int lId, const LD& val)
    {
        vecRegs[reg].laneView<typename LD::UnderlyingType>(lId) = val;
    }

    const VecElem &
    readVecElemFlat(const RegIndex& reg, const ElemIndex& elemIndex) const
    {
        return vecRegs[reg].as<TheISA::VecElem>()[elemIndex];
    }

    void
    setVecElemFlat(const RegIndex& reg, const ElemIndex& elemIndex,
                   const VecElem val)
    {
        vecRegs[reg].as<TheISA::VecElem>()[elemIndex] = val;
    }

    const VecPredRegContainer& readVecPredRegFlat(const RegIndex& reg) const
    {
        return vecPredRegs[reg];
    }

    VecPredRegContainer& getWritableVecPredRegFlat(const RegIndex& reg)
    {
        return vecPredRegs[reg];
    }

    void setVecPredRegFlat(const RegIndex& reg, const VecPredRegContainer& val)
    {
        vecPredRegs[reg] = val;
    }

#ifdef ISA_HAS_CC_REGS
    RegVal readCCRegFlat(int idx) { return ccRegs[idx]; }
    void setCCRegFlat(int idx, RegVal val) { ccRegs[idx] = val; }
#else
    RegVal readCCRegFlat(int idx)
    { panic("readCCRegFlat w/no CC regs!\n"); }

    void setCCRegFlat(int idx, RegVal val)
    { panic("setCCRegFlat w/no CC regs!\n"); }
#endif
};


#endif // __CPU_CPU_EXEC_CONTEXT_HH__
