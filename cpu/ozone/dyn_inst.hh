/*
 * Copyright (c) 2005 The Regents of The University of Michigan
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

#ifndef __CPU_OZONE_DYN_INST_HH__
#define __CPU_OZONE_DYN_INST_HH__

#include "arch/isa_traits.hh"
#include "config/full_system.hh"
#include "cpu/base_dyn_inst.hh"
#include "cpu/ozone/cpu.hh"   // MUST include this
#include "cpu/inst_seq.hh"
#include "cpu/ozone/simple_impl.hh" // Would be nice to not have to include this
#include "cpu/ozone/ozone_impl.hh"

#include <list>
#include <vector>

template <class Impl>
class OzoneDynInst : public BaseDynInst<Impl>
{
  public:
    // Typedefs
    typedef typename Impl::FullCPU FullCPU;

    typedef typename FullCPU::ImplState ImplState;

    // Typedef for DynInstPtr.  This is really just a RefCountingPtr<OoODynInst>.
    typedef typename Impl::DynInstPtr DynInstPtr;

//    typedef typename Impl::BranchPred::BPredInfo BPredInfo;

    typedef TheISA::ExtMachInst ExtMachInst;
    typedef TheISA::MachInst MachInst;
    typedef TheISA::MiscReg MiscReg;
    typedef typename std::list<DynInstPtr>::iterator ListIt;

    // Note that this is duplicated from the BaseDynInst class; I'm simply not
    // sure the enum would carry through so I could use it in array
    // declarations in this class.
    enum {
        MaxInstSrcRegs = TheISA::MaxInstSrcRegs,
        MaxInstDestRegs = TheISA::MaxInstDestRegs
    };

    OzoneDynInst(FullCPU *cpu);

    OzoneDynInst(ExtMachInst inst, Addr PC, Addr Pred_PC,
                 InstSeqNum seq_num, FullCPU *cpu);

    OzoneDynInst(StaticInstPtr inst);

    ~OzoneDynInst();

    void setSrcInst(DynInstPtr &newSrcInst, int regIdx)
    { srcInsts[regIdx] = newSrcInst; }

    bool srcInstReady(int regIdx);

    void setPrevDestInst(DynInstPtr &oldDestInst, int regIdx)
    { prevDestInst[regIdx] = oldDestInst; }

    DynInstPtr &getPrevDestInst(int regIdx)
    { return prevDestInst[regIdx]; }

    void addDependent(DynInstPtr &dependent_inst);

    std::vector<DynInstPtr> &getDependents() { return dependents; }

    void wakeDependents();

//    void setBPredInfo(const BPredInfo &bp_info) { bpInfo = bp_info; }

//    BPredInfo &getBPredInfo() { return bpInfo; }

//    OzoneXC *thread;

  private:
    void initInstPtrs();

    std::vector<DynInstPtr> dependents;

    /** The instruction that produces the value of the source registers.  These
     *  may be NULL if the value has already been read from the source
     *  instruction.
     */
    DynInstPtr srcInsts[MaxInstSrcRegs];

    /**
     *  Previous rename instruction for this destination.
     */
    DynInstPtr prevDestInst[MaxInstSrcRegs];

//    BPredInfo bpInfo;

  public:

    Fault initiateAcc();

    Fault completeAcc();
/*
    template <class T>
    Fault read(Addr addr, T &data, unsigned flags);

    template <class T>
    Fault write(T data, Addr addr, unsigned flags, uint64_t *res);
*/
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
        return srcInsts[idx]->readIntResult();
    }

    float readFloatRegSingle(const StaticInst *si, int idx)
    {
        return srcInsts[idx]->readFloatResult();
    }

    double readFloatRegDouble(const StaticInst *si, int idx)
    {
        return srcInsts[idx]->readDoubleResult();
    }

    uint64_t readFloatRegInt(const StaticInst *si, int idx)
    {
        return srcInsts[idx]->readIntResult();
    }

    /** @todo: Make results into arrays so they can handle multiple dest
     *  registers.
     */
    void setIntReg(const StaticInst *si, int idx, uint64_t val)
    {
        this->instResult.integer = val;
    }

    void setFloatRegSingle(const StaticInst *si, int idx, float val)
    {
        this->instResult.fp = val;
    }

    void setFloatRegDouble(const StaticInst *si, int idx, double val)
    {
        this->instResult.dbl = val;
    }

    void setFloatRegInt(const StaticInst *si, int idx, uint64_t val)
    {
        this->instResult.integer = val;
    }

    void setIntResult(uint64_t result) { this->instResult.integer = result; }
    void setDoubleResult(double result) { this->instResult.dbl = result; }

    bool srcsReady();
    bool eaSrcsReady();

    Fault execute();

    Fault executeEAComp()
    { return NoFault; }

    Fault executeMemAcc()
    { return this->staticInst->memAccInst()->execute(this, this->traceData); }

    void clearDependents();

  public:
    // ISA stuff
    MiscReg readMiscReg(int misc_reg);

    MiscReg readMiscRegWithEffect(int misc_reg, Fault &fault);

    Fault setMiscReg(int misc_reg, const MiscReg &val);

    Fault setMiscRegWithEffect(int misc_reg, const MiscReg &val);

#if FULL_SYSTEM
    Fault hwrei();
    int readIntrFlag();
    void setIntrFlag(int val);
    bool inPalMode();
    void trap(Fault fault);
    bool simPalCheck(int palFunc);
#else
    void syscall();
#endif

    ListIt iqIt;
    bool iqItValid;
};

/*
template<class Impl>
template<class T>
inline Fault
OzoneDynInst<Impl>::read(Addr addr, T &data, unsigned flags)
{
    Fault fault = this->cpu->read(addr, data, flags, this);

    if (this->traceData) {
        this->traceData->setAddr(addr);
        this->traceData->setData(data);
    }

    return fault;
}

template<class Impl>
template<class T>
inline Fault
OzoneDynInst<Impl>::write(T data, Addr addr, unsigned flags, uint64_t *res)
{
    Fault fault = this->cpu->write(data, addr, flags, res, this);

    this->storeSize = sizeof(T);
    this->storeData = data;

    if (this->traceData) {
        this->traceData->setAddr(addr);
        this->traceData->setData(data);
    }

    return fault;
}
*/
#endif // __CPU_OZONE_DYN_INST_HH__
