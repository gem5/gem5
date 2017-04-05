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
 *
 * Authors: Kevin Lim
 */

#ifndef __CPU_O3_DYN_INST_HH__
#define __CPU_O3_DYN_INST_HH__

#include <array>

#include "arch/isa_traits.hh"
#include "config/the_isa.hh"
#include "cpu/o3/cpu.hh"
#include "cpu/o3/isa_specific.hh"
#include "cpu/base_dyn_inst.hh"
#include "cpu/inst_seq.hh"
#include "cpu/reg_class.hh"

class Packet;

template <class Impl>
class BaseO3DynInst : public BaseDynInst<Impl>
{
  public:
    /** Typedef for the CPU. */
    typedef typename Impl::O3CPU O3CPU;

    /** Binary machine instruction type. */
    typedef TheISA::MachInst MachInst;
    /** Extended machine instruction type. */
    typedef TheISA::ExtMachInst ExtMachInst;
    /** Register types. */
    typedef TheISA::IntReg   IntReg;
    typedef TheISA::FloatReg FloatReg;
    typedef TheISA::FloatRegBits FloatRegBits;
    typedef TheISA::CCReg   CCReg;
    using VecRegContainer = TheISA::VecRegContainer;
    using VecElem = TheISA::VecElem;
    static constexpr auto NumVecElemPerVecReg = TheISA::NumVecElemPerVecReg;

    /** Misc register type. */
    typedef TheISA::MiscReg  MiscReg;

    enum {
        MaxInstSrcRegs = TheISA::MaxInstSrcRegs,        //< Max source regs
        MaxInstDestRegs = TheISA::MaxInstDestRegs       //< Max dest regs
    };

  public:
    /** BaseDynInst constructor given a binary instruction. */
    BaseO3DynInst(const StaticInstPtr &staticInst, const StaticInstPtr
            &macroop, TheISA::PCState pc, TheISA::PCState predPC,
            InstSeqNum seq_num, O3CPU *cpu);

    /** BaseDynInst constructor given a static inst pointer. */
    BaseO3DynInst(const StaticInstPtr &_staticInst,
                  const StaticInstPtr &_macroop);

    ~BaseO3DynInst();

    /** Executes the instruction.*/
    Fault execute();

    /** Initiates the access.  Only valid for memory operations. */
    Fault initiateAcc();

    /** Completes the access.  Only valid for memory operations. */
    Fault completeAcc(PacketPtr pkt);

  private:
    /** Initializes variables. */
    void initVars();

  protected:
    /** Explicitation of dependent names. */
    using BaseDynInst<Impl>::cpu;
    using BaseDynInst<Impl>::_srcRegIdx;
    using BaseDynInst<Impl>::_destRegIdx;

    /** Values to be written to the destination misc. registers. */
    std::array<MiscReg, TheISA::MaxMiscDestRegs> _destMiscRegVal;

    /** Indexes of the destination misc. registers. They are needed to defer
     * the write accesses to the misc. registers until the commit stage, when
     * the instruction is out of its speculative state.
     */
    std::array<short, TheISA::MaxMiscDestRegs> _destMiscRegIdx;

    /** Number of destination misc. registers. */
    uint8_t _numDestMiscRegs;


  public:
#if TRACING_ON
    /** Tick records used for the pipeline activity viewer. */
    Tick fetchTick;      // instruction fetch is completed.
    int32_t decodeTick;  // instruction enters decode phase
    int32_t renameTick;  // instruction enters rename phase
    int32_t dispatchTick;
    int32_t issueTick;
    int32_t completeTick;
    int32_t commitTick;
    int32_t storeTick;
#endif

    /** Reads a misc. register, including any side-effects the read
     * might have as defined by the architecture.
     */
    MiscReg readMiscReg(int misc_reg)
    {
        return this->cpu->readMiscReg(misc_reg, this->threadNumber);
    }

    /** Sets a misc. register, including any side-effects the write
     * might have as defined by the architecture.
     */
    void setMiscReg(int misc_reg, const MiscReg &val)
    {
        /** Writes to misc. registers are recorded and deferred until the
         * commit stage, when updateMiscRegs() is called. First, check if
         * the misc reg has been written before and update its value to be
         * committed instead of making a new entry. If not, make a new
         * entry and record the write.
         */
        for (int idx = 0; idx < _numDestMiscRegs; idx++) {
            if (_destMiscRegIdx[idx] == misc_reg) {
               _destMiscRegVal[idx] = val;
               return;
            }
        }

        assert(_numDestMiscRegs < TheISA::MaxMiscDestRegs);
        _destMiscRegIdx[_numDestMiscRegs] = misc_reg;
        _destMiscRegVal[_numDestMiscRegs] = val;
        _numDestMiscRegs++;
    }

    /** Reads a misc. register, including any side-effects the read
     * might have as defined by the architecture.
     */
    TheISA::MiscReg readMiscRegOperand(const StaticInst *si, int idx)
    {
        const RegId& reg = si->srcRegIdx(idx);
        assert(reg.isMiscReg());
        return this->cpu->readMiscReg(reg.index(), this->threadNumber);
    }

    /** Sets a misc. register, including any side-effects the write
     * might have as defined by the architecture.
     */
    void setMiscRegOperand(const StaticInst *si, int idx,
                                     const MiscReg &val)
    {
        const RegId& reg = si->destRegIdx(idx);
        assert(reg.isMiscReg());
        setMiscReg(reg.index(), val);
    }

    /** Called at the commit stage to update the misc. registers. */
    void updateMiscRegs()
    {
        // @todo: Pretty convoluted way to avoid squashing from happening when
        // using the TC during an instruction's execution (specifically for
        // instructions that have side-effects that use the TC).  Fix this.
        // See cpu/o3/dyn_inst_impl.hh.
        bool no_squash_from_TC = this->thread->noSquashFromTC;
        this->thread->noSquashFromTC = true;

        for (int i = 0; i < _numDestMiscRegs; i++)
            this->cpu->setMiscReg(
                _destMiscRegIdx[i], _destMiscRegVal[i], this->threadNumber);

        this->thread->noSquashFromTC = no_squash_from_TC;
    }

    void forwardOldRegs()
    {

        for (int idx = 0; idx < this->numDestRegs(); idx++) {
            PhysRegIdPtr prev_phys_reg = this->prevDestRegIdx(idx);
            const RegId& original_dest_reg =
                this->staticInst->destRegIdx(idx);
            switch (original_dest_reg.classValue()) {
              case IntRegClass:
                this->setIntRegOperand(this->staticInst.get(), idx,
                               this->cpu->readIntReg(prev_phys_reg));
                break;
              case FloatRegClass:
                this->setFloatRegOperandBits(this->staticInst.get(), idx,
                               this->cpu->readFloatRegBits(prev_phys_reg));
                break;
              case VecRegClass:
                this->setVecRegOperand(this->staticInst.get(), idx,
                               this->cpu->readVecReg(prev_phys_reg));
                break;
              case VecElemClass:
                this->setVecElemOperand(this->staticInst.get(), idx,
                               this->cpu->readVecElem(prev_phys_reg));
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
    /** Calls hardware return from error interrupt. */
    Fault hwrei();
    /** Traps to handle specified fault. */
    void trap(const Fault &fault);
    bool simPalCheck(int palFunc);

    /** Emulates a syscall. */
    void syscall(int64_t callnum, Fault *fault);

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

    IntReg readIntRegOperand(const StaticInst *si, int idx)
    {
        return this->cpu->readIntReg(this->_srcRegIdx[idx]);
    }

    FloatReg readFloatRegOperand(const StaticInst *si, int idx)
    {
        return this->cpu->readFloatReg(this->_srcRegIdx[idx]);
    }

    FloatRegBits readFloatRegOperandBits(const StaticInst *si, int idx)
    {
        return this->cpu->readFloatRegBits(this->_srcRegIdx[idx]);
    }

    const VecRegContainer&
    readVecRegOperand(const StaticInst *si, int idx) const
    {
        return this->cpu->readVecReg(this->_srcRegIdx[idx]);
    }

    /**
     * Read destination vector register operand for modification.
     */
    VecRegContainer&
    getWritableVecRegOperand(const StaticInst *si, int idx)
    {
        return this->cpu->getWritableVecReg(this->_destRegIdx[idx]);
    }

    /** Vector Register Lane Interfaces. */
    /** @{ */
    /** Reads source vector 8bit operand. */
    ConstVecLane8
    readVec8BitLaneOperand(const StaticInst *si, int idx) const
    {
        return cpu->template readVecLane<uint8_t>(_srcRegIdx[idx]);
    }

    /** Reads source vector 16bit operand. */
    ConstVecLane16
    readVec16BitLaneOperand(const StaticInst *si, int idx) const
    {
        return cpu->template readVecLane<uint16_t>(_srcRegIdx[idx]);
    }

    /** Reads source vector 32bit operand. */
    ConstVecLane32
    readVec32BitLaneOperand(const StaticInst *si, int idx) const
    {
        return cpu->template readVecLane<uint32_t>(_srcRegIdx[idx]);
    }

    /** Reads source vector 64bit operand. */
    ConstVecLane64
    readVec64BitLaneOperand(const StaticInst *si, int idx) const
    {
        return cpu->template readVecLane<uint64_t>(_srcRegIdx[idx]);
    }

    /** Write a lane of the destination vector operand. */
    template <typename LD>
    void
    setVecLaneOperandT(const StaticInst *si, int idx, const LD& val)
    {
        return cpu->template setVecLane(_destRegIdx[idx], val);
    }
    virtual void
    setVecLaneOperand(const StaticInst *si, int idx,
            const LaneData<LaneSize::Byte>& val)
    {
        return setVecLaneOperandT(si, idx, val);
    }
    virtual void
    setVecLaneOperand(const StaticInst *si, int idx,
            const LaneData<LaneSize::TwoByte>& val)
    {
        return setVecLaneOperandT(si, idx, val);
    }
    virtual void
    setVecLaneOperand(const StaticInst *si, int idx,
            const LaneData<LaneSize::FourByte>& val)
    {
        return setVecLaneOperandT(si, idx, val);
    }
    virtual void
    setVecLaneOperand(const StaticInst *si, int idx,
            const LaneData<LaneSize::EightByte>& val)
    {
        return setVecLaneOperandT(si, idx, val);
    }
    /** @} */

    VecElem readVecElemOperand(const StaticInst *si, int idx) const
    {
        return this->cpu->readVecElem(this->_srcRegIdx[idx]);
    }

    CCReg readCCRegOperand(const StaticInst *si, int idx)
    {
        return this->cpu->readCCReg(this->_srcRegIdx[idx]);
    }

    /** @todo: Make results into arrays so they can handle multiple dest
     *  registers.
     */
    void setIntRegOperand(const StaticInst *si, int idx, IntReg val)
    {
        this->cpu->setIntReg(this->_destRegIdx[idx], val);
        BaseDynInst<Impl>::setIntRegOperand(si, idx, val);
    }

    void setFloatRegOperand(const StaticInst *si, int idx, FloatReg val)
    {
        this->cpu->setFloatReg(this->_destRegIdx[idx], val);
        BaseDynInst<Impl>::setFloatRegOperand(si, idx, val);
    }

    void setFloatRegOperandBits(const StaticInst *si, int idx,
                                FloatRegBits val)
    {
        this->cpu->setFloatRegBits(this->_destRegIdx[idx], val);
        BaseDynInst<Impl>::setFloatRegOperandBits(si, idx, val);
    }

    void
    setVecRegOperand(const StaticInst *si, int idx,
                     const VecRegContainer& val)
    {
        this->cpu->setVecReg(this->_destRegIdx[idx], val);
        BaseDynInst<Impl>::setVecRegOperand(si, idx, val);
    }

    void setVecElemOperand(const StaticInst *si, int idx,
                           const VecElem val)
    {
        int reg_idx = idx;
        this->cpu->setVecElem(this->_destRegIdx[reg_idx], val);
        BaseDynInst<Impl>::setVecElemOperand(si, idx, val);
    }

    void setCCRegOperand(const StaticInst *si, int idx, CCReg val)
    {
        this->cpu->setCCReg(this->_destRegIdx[idx], val);
        BaseDynInst<Impl>::setCCRegOperand(si, idx, val);
    }

#if THE_ISA == MIPS_ISA
    MiscReg readRegOtherThread(const RegId& misc_reg, ThreadID tid)
    {
        panic("MIPS MT not defined for O3 CPU.\n");
        return 0;
    }

    void setRegOtherThread(const RegId& misc_reg, MiscReg val, ThreadID tid)
    {
        panic("MIPS MT not defined for O3 CPU.\n");
    }
#endif

  public:
    /** Calculates EA part of a memory instruction. Currently unused,
     * though it may be useful in the future if we want to split
     * memory operations into EA calculation and memory access parts.
     */
    Fault calcEA()
    {
        return this->staticInst->eaCompInst()->execute(this, this->traceData);
    }

    /** Does the memory access part of a memory instruction. Currently unused,
     * though it may be useful in the future if we want to split
     * memory operations into EA calculation and memory access parts.
     */
    Fault memAccess()
    {
        return this->staticInst->memAccInst()->execute(this, this->traceData);
    }
};

#endif // __CPU_O3_ALPHA_DYN_INST_HH__

