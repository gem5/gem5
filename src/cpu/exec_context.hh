/*
 * Copyright (c) 2014, 2016-2018, 2020-2021 ARM Limited
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

#include "base/types.hh"
#include "cpu/base.hh"
#include "cpu/reg_class.hh"
#include "cpu/static_inst_fwd.hh"
#include "cpu/translation.hh"
#include "mem/request.hh"

namespace gem5
{

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
class ExecContext
{
  public:

    virtual RegVal getRegOperand(const StaticInst *si, int idx) = 0;
    virtual void getRegOperand(const StaticInst *si, int idx, void *val) = 0;
    virtual void *getWritableRegOperand(const StaticInst *si, int idx) = 0;
    virtual void setRegOperand(const StaticInst *si, int idx, RegVal val) = 0;
    virtual void setRegOperand(const StaticInst *si, int idx,
            const void *val) = 0;

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
    virtual const PCStateBase &pcState() const = 0;
    virtual void pcState(const PCStateBase &val) = 0;
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
    virtual Fault
    readMem(Addr addr, uint8_t *data, unsigned int size,
            Request::Flags flags, const std::vector<bool>& byte_enable)
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
    virtual Fault
    initiateMemRead(Addr addr, unsigned int size,
            Request::Flags flags, const std::vector<bool>& byte_enable)
    {
        panic("ExecContext::initiateMemRead() should be overridden\n");
    }

    /**
     * Initiate a memory management command with no valid address.
     * Currently, these instructions need to bypass squashing in the O3 model
     * Examples include HTM commands and TLBI commands.
     * e.g. tell Ruby we're starting/stopping a HTM transaction,
     *      or tell Ruby to issue a TLBI operation
     */
    virtual Fault initiateMemMgmtCmd(Request::Flags flags) = 0;

    /**
     * For atomic-mode contexts, perform an atomic memory write operation.
     * For timing-mode contexts, initiate a timing memory write operation.
     */
    virtual Fault writeMem(uint8_t *data, unsigned int size, Addr addr,
                           Request::Flags flags, uint64_t *res,
                           const std::vector<bool>& byte_enable) = 0;

    /**
     * For atomic-mode contexts, perform an atomic AMO (a.k.a., Atomic
     * Read-Modify-Write Memory Operation)
     */
    virtual Fault
    amoMem(Addr addr, uint8_t *data, unsigned int size,
            Request::Flags flags, AtomicOpFunctorPtr amo_op)
    {
        panic("ExecContext::amoMem() should be overridden\n");
    }

    /**
     * For timing-mode contexts, initiate an atomic AMO (atomic
     * read-modify-write memory operation)
     */
    virtual Fault
    initiateMemAMO(Addr addr, unsigned int size, Request::Flags flags,
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

    // hardware transactional memory
    virtual uint64_t newHtmTransactionUid() const = 0;
    virtual uint64_t getHtmTransactionUid() const = 0;
    virtual bool inHtmTransactionalState() const = 0;
    virtual uint64_t getHtmTransactionalDepth() const = 0;

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

} // namespace gem5

#endif // __CPU_EXEC_CONTEXT_HH__
