/*
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

#ifndef __CPU_O3_ALPHA_DYN_INST_HH__
#define __CPU_O3_ALPHA_DYN_INST_HH__

#include "arch/isa_traits.hh"
#include "cpu/base_dyn_inst.hh"
#include "cpu/inst_seq.hh"
#include "cpu/o3/alpha_cpu.hh"
#include "cpu/o3/alpha_impl.hh"

class Packet;

/**
 * Mostly implementation & ISA specific AlphaDynInst. As with most
 * other classes in the new CPU model, it is templated on the Impl to
 * allow for passing in of all types, such as the CPU type and the ISA
 * type. The AlphaDynInst serves as the primary interface to the CPU
 * for instructions that are executing.
 */
template <class Impl>
class AlphaDynInst : public BaseDynInst<Impl>
{
  public:
    /** Typedef for the CPU. */
    typedef typename Impl::O3CPU O3CPU;

    /** Binary machine instruction type. */
    typedef TheISA::MachInst MachInst;
    /** Extended machine instruction type. */
    typedef TheISA::ExtMachInst ExtMachInst;
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
    AlphaDynInst(ExtMachInst inst, Addr PC, Addr Pred_PC, InstSeqNum seq_num,
                 O3CPU *cpu);

    /** BaseDynInst constructor given a static inst pointer. */
    AlphaDynInst(StaticInstPtr &_staticInst);

    /** Executes the instruction.*/
    Fault execute();

    /** Initiates the access.  Only valid for memory operations. */
    Fault initiateAcc();

    /** Completes the access.  Only valid for memory operations. */
    Fault completeAcc(Packet *pkt);

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
    MiscReg readMiscRegWithEffect(int misc_reg, Fault &fault)
    {
        return this->cpu->readMiscRegWithEffect(misc_reg, fault,
                                                this->threadNumber);
    }

    /** Sets a misc. register. */
    Fault setMiscReg(int misc_reg, const MiscReg &val)
    {
        this->instResult.integer = val;
        return this->cpu->setMiscReg(misc_reg, val, this->threadNumber);
    }

    /** Sets a misc. register, including any side-effects the write
     * might have as defined by the architecture.
     */
    Fault setMiscRegWithEffect(int misc_reg, const MiscReg &val)
    {
        return this->cpu->setMiscRegWithEffect(misc_reg, val,
                                               this->threadNumber);
    }

#if FULL_SYSTEM
    /** Calls hardware return from error interrupt. */
    Fault hwrei();
    /** Reads interrupt flag. */
    int readIntrFlag();
    /** Sets interrupt flag. */
    void setIntrFlag(int val);
    /** Checks if system is in PAL mode. */
    bool inPalMode();
    /** Traps to handle specified fault. */
    void trap(Fault fault);
    bool simPalCheck(int palFunc);
#else
    /** Calls a syscall. */
    void syscall(int64_t callnum);
#endif

  private:
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

    uint64_t readIntReg(const StaticInst *si, int idx)
    {
        return this->cpu->readIntReg(_srcRegIdx[idx]);
    }

    FloatReg readFloatReg(const StaticInst *si, int idx, int width)
    {
        return this->cpu->readFloatReg(_srcRegIdx[idx], width);
    }

    FloatReg readFloatReg(const StaticInst *si, int idx)
    {
        return this->cpu->readFloatReg(_srcRegIdx[idx]);
    }

    FloatRegBits readFloatRegBits(const StaticInst *si, int idx, int width)
    {
        return this->cpu->readFloatRegBits(_srcRegIdx[idx], width);
    }

    FloatRegBits readFloatRegBits(const StaticInst *si, int idx)
    {
        return this->cpu->readFloatRegBits(_srcRegIdx[idx]);
    }

    /** @todo: Make results into arrays so they can handle multiple dest
     *  registers.
     */
    void setIntReg(const StaticInst *si, int idx, uint64_t val)
    {
        this->cpu->setIntReg(_destRegIdx[idx], val);
        BaseDynInst<Impl>::setIntReg(si, idx, val);
    }

    void setFloatReg(const StaticInst *si, int idx, FloatReg val, int width)
    {
        this->cpu->setFloatReg(_destRegIdx[idx], val, width);
        BaseDynInst<Impl>::setFloatReg(si, idx, val, width);
    }

    void setFloatReg(const StaticInst *si, int idx, FloatReg val)
    {
        this->cpu->setFloatReg(_destRegIdx[idx], val);
        BaseDynInst<Impl>::setFloatReg(si, idx, val);
    }

    void setFloatRegBits(const StaticInst *si, int idx,
            FloatRegBits val, int width)
    {
        this->cpu->setFloatRegBits(_destRegIdx[idx], val, width);
        BaseDynInst<Impl>::setFloatRegBits(si, idx, val);
    }

    void setFloatRegBits(const StaticInst *si, int idx, FloatRegBits val)
    {
        this->cpu->setFloatRegBits(_destRegIdx[idx], val);
        BaseDynInst<Impl>::setFloatRegBits(si, idx, val);
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

