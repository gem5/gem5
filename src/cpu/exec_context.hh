/*
 * Copyright (c) 2014, 2016-2018, 2020 ARM Limited
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
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * Copyright (c) 2015 Advanced Micro Devices, Inc.
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

#ifndef __CPU_EXEC_CONTEXT_HH__
#define __CPU_EXEC_CONTEXT_HH__

#include "arch/registers.hh"
#include "base/types.hh"
#include "config/the_isa.hh"
#include "cpu/base.hh"
#include "cpu/reg_class.hh"
#include "cpu/static_inst_fwd.hh"
#include "cpu/translation.hh"
#include "mem/request.hh"

/**
 * The ExecContext is an abstract base class the provides the
 * interface used by the ISA to manipulate the state of the CPU model.
 *
 * Register accessor methods in this class typically provide the index
 * of the instruction's operand (e.g., 0 or 1), not the architectural
 * register index, to simplify the implementation of register
 * renaming.  The architectural register index can be found by
 * indexing into the instruction's own operand index table.
 *
 * @note The methods in this class typically take a raw pointer to the
 * StaticInst is provided instead of a ref-counted StaticInstPtr to
 * reduce overhead as an argument. This is fine as long as the
 * implementation doesn't copy the pointer into any long-term storage
 * (which is pretty hard to imagine they would have reason to do).
 */
class ExecContext {
  public:
    typedef TheISA::PCState PCState;

    using VecRegContainer = TheISA::VecRegContainer;
    using VecElem = TheISA::VecElem;
    using VecPredRegContainer = TheISA::VecPredRegContainer;

  public:
    /**
     * @{
     * @name Integer Register Interfaces
     *
     */

    /** Reads an integer register. */
    virtual RegVal readIntRegOperand(const StaticInst *si, int idx) = 0;

    /** Sets an integer register to a value. */
    virtual void setIntRegOperand(const StaticInst *si,
                                  int idx, RegVal val) = 0;

    /** @} */


    /**
     * @{
     * @name Floating Point Register Interfaces
     */

    /** Reads a floating point register in its binary format, instead
     * of by value. */
    virtual RegVal readFloatRegOperandBits(const StaticInst *si, int idx) = 0;

    /** Sets the bits of a floating point register of single width
     * to a binary value. */
    virtual void setFloatRegOperandBits(const StaticInst *si,
                                        int idx, RegVal val) = 0;

    /** @} */

    /** Vector Register Interfaces. */
    /** @{ */
    /** Reads source vector register operand. */
    virtual const VecRegContainer&
    readVecRegOperand(const StaticInst *si, int idx) const = 0;

    /** Gets destination vector register operand for modification. */
    virtual VecRegContainer&
    getWritableVecRegOperand(const StaticInst *si, int idx) = 0;

    /** Sets a destination vector register operand to a value. */
    virtual void
    setVecRegOperand(const StaticInst *si, int idx,
                     const VecRegContainer& val) = 0;
    /** @} */

    /** Vector Register Lane Interfaces. */
    /** @{ */
    /** Reads source vector 8bit operand. */
    virtual ConstVecLane8
    readVec8BitLaneOperand(const StaticInst *si, int idx) const = 0;

    /** Reads source vector 16bit operand. */
    virtual ConstVecLane16
    readVec16BitLaneOperand(const StaticInst *si, int idx) const = 0;

    /** Reads source vector 32bit operand. */
    virtual ConstVecLane32
    readVec32BitLaneOperand(const StaticInst *si, int idx) const = 0;

    /** Reads source vector 64bit operand. */
    virtual ConstVecLane64
    readVec64BitLaneOperand(const StaticInst *si, int idx) const = 0;

    /** Write a lane of the destination vector operand. */
    /** @{ */
    virtual void setVecLaneOperand(const StaticInst *si, int idx,
            const LaneData<LaneSize::Byte>& val) = 0;
    virtual void setVecLaneOperand(const StaticInst *si, int idx,
            const LaneData<LaneSize::TwoByte>& val) = 0;
    virtual void setVecLaneOperand(const StaticInst *si, int idx,
            const LaneData<LaneSize::FourByte>& val) = 0;
    virtual void setVecLaneOperand(const StaticInst *si, int idx,
            const LaneData<LaneSize::EightByte>& val) = 0;
    /** @} */

    /** Vector Elem Interfaces. */
    /** @{ */
    /** Reads an element of a vector register. */
    virtual VecElem readVecElemOperand(const StaticInst *si,
                                        int idx) const = 0;

    /** Sets a vector register to a value. */
    virtual void setVecElemOperand(const StaticInst *si, int idx,
                                   const VecElem val) = 0;
    /** @} */

    /** Predicate registers interface. */
    /** @{ */
    /** Reads source predicate register operand. */
    virtual const VecPredRegContainer&
    readVecPredRegOperand(const StaticInst *si, int idx) const = 0;

    /** Gets destination predicate register operand for modification. */
    virtual VecPredRegContainer&
    getWritableVecPredRegOperand(const StaticInst *si, int idx) = 0;

    /** Sets a destination predicate register operand to a value. */
    virtual void
    setVecPredRegOperand(const StaticInst *si, int idx,
                         const VecPredRegContainer& val) = 0;
    /** @} */

    /**
     * @{
     * @name Condition Code Registers
     */
    virtual RegVal readCCRegOperand(const StaticInst *si, int idx) = 0;
    virtual void setCCRegOperand(
            const StaticInst *si, int idx, RegVal val) = 0;
    /** @} */

    /**
     * @{
     * @name Misc Register Interfaces
     */
    virtual RegVal readMiscRegOperand(const StaticInst *si, int idx) = 0;
    virtual void setMiscRegOperand(const StaticInst *si,
                                   int idx, RegVal val) = 0;

    /**
     * Reads a miscellaneous register, handling any architectural
     * side effects due to reading that register.
     */
    virtual RegVal readMiscReg(int misc_reg) = 0;

    /**
     * Sets a miscellaneous register, handling any architectural
     * side effects due to writing that register.
     */
    virtual void setMiscReg(int misc_reg, RegVal val) = 0;

    /** @} */

    /**
     * @{
     * @name PC Control
     */
    virtual PCState pcState() const = 0;
    virtual void pcState(const PCState &val) = 0;
    /** @} */

    /**
     * @{
     * @name Memory Interface
     */
    /**
     * Perform an atomic memory read operation.  Must be overridden
     * for exec contexts that support atomic memory mode.  Not pure
     * virtual since exec contexts that only support timing memory
     * mode need not override (though in that case this function
     * should never be called).
     */
    virtual Fault readMem(Addr addr, uint8_t *data, unsigned int size,
            Request::Flags flags,
            const std::vector<bool>& byte_enable = std::vector<bool>())
    {
        panic("ExecContext::readMem() should be overridden\n");
    }

    /**
     * Initiate a timing memory read operation.  Must be overridden
     * for exec contexts that support timing memory mode.  Not pure
     * virtual since exec contexts that only support atomic memory
     * mode need not override (though in that case this function
     * should never be called).
     */
    virtual Fault initiateMemRead(Addr addr, unsigned int size,
            Request::Flags flags,
            const std::vector<bool>& byte_enable = std::vector<bool>())
    {
        panic("ExecContext::initiateMemRead() should be overridden\n");
    }

    /**
     * For atomic-mode contexts, perform an atomic memory write operation.
     * For timing-mode contexts, initiate a timing memory write operation.
     */
    virtual Fault writeMem(uint8_t *data, unsigned int size, Addr addr,
                           Request::Flags flags, uint64_t *res,
                           const std::vector<bool>& byte_enable =
                               std::vector<bool>()) = 0;

    /**
     * For atomic-mode contexts, perform an atomic AMO (a.k.a., Atomic
     * Read-Modify-Write Memory Operation)
     */
    virtual Fault amoMem(Addr addr, uint8_t *data, unsigned int size,
                         Request::Flags flags,
                         AtomicOpFunctorPtr amo_op)
    {
        panic("ExecContext::amoMem() should be overridden\n");
    }

    /**
     * For timing-mode contexts, initiate an atomic AMO (atomic
     * read-modify-write memory operation)
     */
    virtual Fault initiateMemAMO(Addr addr, unsigned int size,
                                 Request::Flags flags,
                                 AtomicOpFunctorPtr amo_op)
    {
        panic("ExecContext::initiateMemAMO() should be overridden\n");
    }

    /**
     * Sets the number of consecutive store conditional failures.
     */
    virtual void setStCondFailures(unsigned int sc_failures) = 0;

    /**
     * Returns the number of consecutive store conditional failures.
     */
    virtual unsigned int readStCondFailures() const = 0;

    /** @} */

    /**
     * @{
     * @name SysCall Emulation Interfaces
     */

    /**
     * Executes a syscall.
     */
    virtual void syscall(Fault *fault) = 0;

    /** @} */

    /** Returns a pointer to the ThreadContext. */
    virtual ThreadContext *tcBase() const = 0;

    /**
     * @{
     * @name ARM-Specific Interfaces
     */

    virtual bool readPredicate() const = 0;
    virtual void setPredicate(bool val) = 0;
    virtual bool readMemAccPredicate() const = 0;
    virtual void setMemAccPredicate(bool val) = 0;

    /** @} */

    /**
     * @{
     * @name X86-Specific Interfaces
     */

    /**
     * Invalidate a page in the DTLB <i>and</i> ITLB.
     */
    virtual void demapPage(Addr vaddr, uint64_t asn) = 0;
    virtual void armMonitor(Addr address) = 0;
    virtual bool mwait(PacketPtr pkt) = 0;
    virtual void mwaitAtomic(ThreadContext *tc) = 0;
    virtual AddressMonitor *getAddrMonitor() = 0;

    /** @} */
};

#endif // __CPU_EXEC_CONTEXT_HH__
