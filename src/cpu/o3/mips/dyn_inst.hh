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
 *
 * Authors: Kevin Lim
 *          Korey Sewell
 */

#ifndef __CPU_O3_MIPS_DYN_INST_HH__
#define __CPU_O3_MIPS_DYN_INST_HH__

#include "arch/isa_traits.hh"
#include "cpu/base_dyn_inst.hh"
#include "cpu/inst_seq.hh"
#include "cpu/o3/mips/cpu.hh"
#include "cpu/o3/mips/impl.hh"

class Packet;

/**
 * Mostly implementation & ISA specific MipsDynInst. As with most
 * other classes in the new CPU model, it is templated on the Impl to
 * allow for passing in of all types, such as the CPU type and the ISA
 * type. The MipsDynInst serves as the primary interface to the CPU
 * for instructions that are executing.
 */
template <class Impl>
class MipsDynInst : public BaseDynInst<Impl>
{
  public:
    /** Typedef for the CPU. */
    typedef typename Impl::O3CPU O3CPU;

    /** Logical register index type. */
    typedef TheISA::RegIndex RegIndex;
    /** Integer register index type. */
    typedef TheISA::IntReg   IntReg;
    typedef TheISA::FloatReg FloatReg;
    typedef TheISA::FloatRegBits FloatRegBits;
    /** Misc register index type. */
    typedef TheISA::MiscReg  MiscReg;

    enum {
        MaxInstSrcRegs = TheISA::MaxInstSrcRegs,	//< Max source regs
        MaxInstDestRegs = TheISA::MaxInstDestRegs,	//< Max dest regs
    };

  public:
    /** BaseDynInst constructor given a binary instruction. */
    MipsDynInst(ExtMachInst inst, Addr PC, Addr Pred_PC, InstSeqNum seq_num,
                 O3CPU *cpu);

    /** BaseDynInst constructor given a static inst pointer. */
    MipsDynInst(StaticInstPtr &_staticInst);

    /** Executes the instruction.*/
    Fault execute();

    /** Initiates the access.  Only valid for memory operations. */
    Fault initiateAcc();

    /** Completes the access.  Only valid for memory operations. */
    Fault completeAcc(PacketPtr pkt);

  private:
    /** Initializes variables. */
    void initVars();

  public:
    /** Reads a miscellaneous register. */
    MiscReg readMiscReg(int misc_reg)
    {
        return this->cpu->readMiscReg(misc_reg, this->threadNumber);
    }

    /** Reads a misc. register, including any side-effects the read
     * might have as defined by the architecture.
     */
    MiscReg readMiscRegWithEffect(int misc_reg)
    {
        return this->cpu->readMiscRegWithEffect(misc_reg, this->threadNumber);
    }

    /** Sets a misc. register. */
    void setMiscReg(int misc_reg, const MiscReg &val)
    {
        this->instResult.integer = val;
        this->cpu->setMiscReg(misc_reg, val, this->threadNumber);
    }

    /** Sets a misc. register, including any side-effects the write
     * might have as defined by the architecture.
     */
    void setMiscRegWithEffect(int misc_reg, const MiscReg &val)
    {
        return this->cpu->setMiscRegWithEffect(misc_reg, val,
                                               this->threadNumber);
    }

    /** Calls a syscall. */
    void syscall(int64_t callnum);

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

    uint64_t readIntRegOperand(const StaticInst *si, int idx)
    {
        return this->cpu->readIntReg(this->_srcRegIdx[idx]);
    }

    FloatReg readFloatRegOperand(const StaticInst *si, int idx, int width)
    {
        return this->cpu->readFloatReg(this->_srcRegIdx[idx], width);
    }

    FloatReg readFloatRegOperand(const StaticInst *si, int idx)
    {
        return this->cpu->readFloatReg(this->_srcRegIdx[idx]);
    }

    FloatRegBits readFloatRegOperandBits(const StaticInst *si, int idx,
                                         int width)
    {
        return this->cpu->readFloatRegBits(this->_srcRegIdx[idx], width);
    }

    FloatRegBits readFloatRegOperandBits(const StaticInst *si, int idx)
    {
        return this->cpu->readFloatRegBits(this->_srcRegIdx[idx]);
    }

    /** @todo: Make results into arrays so they can handle multiple dest
     *  registers.
     */
    void setIntRegOperand(const StaticInst *si, int idx, uint64_t val)
    {
        this->cpu->setIntReg(this->_destRegIdx[idx], val);
        BaseDynInst<Impl>::setIntRegOperand(si, idx, val);
    }

    void setFloatRegOperand(const StaticInst *si, int idx, FloatReg val,
                            int width)
    {
        this->cpu->setFloatReg(this->_destRegIdx[idx], val, width);
        BaseDynInst<Impl>::setFloatRegOperand(si, idx, val, width);
    }

    void setFloatRegOperand(const StaticInst *si, int idx, FloatReg val)
    {
        this->cpu->setFloatReg(this->_destRegIdx[idx], val);
        BaseDynInst<Impl>::setFloatRegOperand(si, idx, val);
    }

    void setFloatRegOperandBits(const StaticInst *si, int idx,
                                FloatRegBits val, int width)
    {
        this->cpu->setFloatRegBits(this->_destRegIdx[idx], val, width);
        BaseDynInst<Impl>::setFloatRegOperandBits(si, idx, val);
    }

    void setFloatRegOperandBits(const StaticInst *si, int idx,
                                FloatRegBits val)
    {
        this->cpu->setFloatRegBits(this->_destRegIdx[idx], val);
        BaseDynInst<Impl>::setFloatRegOperandBits(si, idx, val);
    }

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

#endif // __CPU_O3_MIPS_DYN_INST_HH__

