/*
 * Copyright (c) 2014 ARM Limited
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
 *          Andreas Sandberg
 */

#ifndef __CPU_EXEC_CONTEXT_HH__
#define __CPU_EXEC_CONTEXT_HH__

#include "arch/registers.hh"
#include "base/types.hh"
#include "config/the_isa.hh"
#include "cpu/base.hh"
#include "cpu/static_inst_fwd.hh"
#include "cpu/translation.hh"

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
    typedef TheISA::IntReg IntReg;
    typedef TheISA::PCState PCState;
    typedef TheISA::FloatReg FloatReg;
    typedef TheISA::FloatRegBits FloatRegBits;
    typedef TheISA::MiscReg MiscReg;

    typedef TheISA::CCReg CCReg;

  public:
    /**
     * @{
     * @name Integer Register Interfaces
     *
     */

    /** Reads an integer register. */
    virtual IntReg readIntRegOperand(const StaticInst *si, int idx) = 0;

    /** Sets an integer register to a value. */
    virtual void setIntRegOperand(const StaticInst *si,
                                  int idx, IntReg val) = 0;

    /** @} */


    /**
     * @{
     * @name Floating Point Register Interfaces
     */

    /** Reads a floating point register of single register width. */
    virtual FloatReg readFloatRegOperand(const StaticInst *si, int idx) = 0;

    /** Reads a floating point register in its binary format, instead
     * of by value. */
    virtual FloatRegBits readFloatRegOperandBits(const StaticInst *si,
                                                 int idx) = 0;

    /** Sets a floating point register of single width to a value. */
    virtual void setFloatRegOperand(const StaticInst *si,
                                    int idx, FloatReg val) = 0;

    /** Sets the bits of a floating point register of single width
     * to a binary value. */
    virtual void setFloatRegOperandBits(const StaticInst *si,
                                        int idx, FloatRegBits val) = 0;

    /** @} */

    /**
     * @{
     * @name Condition Code Registers
     */
    virtual CCReg readCCRegOperand(const StaticInst *si, int idx) = 0;
    virtual void setCCRegOperand(const StaticInst *si, int idx, CCReg val) = 0;
    /** @} */

    /**
     * @{
     * @name Misc Register Interfaces
     */
    virtual MiscReg readMiscRegOperand(const StaticInst *si, int idx) = 0;
    virtual void setMiscRegOperand(const StaticInst *si,
                                   int idx, const MiscReg &val) = 0;

    /**
     * Reads a miscellaneous register, handling any architectural
     * side effects due to reading that register.
     */
    virtual MiscReg readMiscReg(int misc_reg) = 0;

    /**
     * Sets a miscellaneous register, handling any architectural
     * side effects due to writing that register.
     */
    virtual void setMiscReg(int misc_reg, const MiscReg &val) = 0;

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
     * Record the effective address of the instruction.
     *
     * @note Only valid for memory ops.
     */
    virtual void setEA(Addr EA) = 0;
    /**
     * Get the effective address of the instruction.
     *
     * @note Only valid for memory ops.
     */
    virtual Addr getEA() const = 0;

    virtual Fault readMem(Addr addr, uint8_t *data, unsigned int size,
                          unsigned int flags) = 0;

    virtual Fault writeMem(uint8_t *data, unsigned int size, Addr addr,
                           unsigned int flags, uint64_t *res) = 0;

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
     * Executes a syscall specified by the callnum.
     */
    virtual void syscall(int64_t callnum) = 0;

    /** @} */

    /** Returns a pointer to the ThreadContext. */
    virtual ThreadContext *tcBase() = 0;

    /**
     * @{
     * @name Alpha-Specific Interfaces
     */

    /**
     * Somewhat Alpha-specific function that handles returning from an
     * error or interrupt.
     */
    virtual Fault hwrei() = 0;

    /**
     * Check for special simulator handling of specific PAL calls.  If
     * return value is false, actual PAL call will be suppressed.
     */
    virtual bool simPalCheck(int palFunc) = 0;

    /** @} */

    /**
     * @{
     * @name ARM-Specific Interfaces
     */

    virtual bool readPredicate() = 0;
    virtual void setPredicate(bool val) = 0;

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

    /**
     * @{
     * @name MIPS-Specific Interfaces
     */

#if THE_ISA == MIPS_ISA
    virtual MiscReg readRegOtherThread(int regIdx,
                                       ThreadID tid = InvalidThreadID) = 0;
    virtual void setRegOtherThread(int regIdx, MiscReg val,
                                   ThreadID tid = InvalidThreadID) = 0;
#endif

    /** @} */
};

#endif // __CPU_EXEC_CONTEXT_HH__
