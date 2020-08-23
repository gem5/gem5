/*
 * Copyright (c) 2003-2004 The Regents of The University of Michigan
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

#ifndef __SPARC_PROCESS_HH__
#define __SPARC_PROCESS_HH__

#include <memory>
#include <string>
#include <vector>

#include "arch/sparc/isa_traits.hh"
#include "arch/sparc/miscregs.hh"
#include "base/loader/object_file.hh"
#include "mem/page_table.hh"
#include "sim/byteswap.hh"
#include "sim/process.hh"
#include "sim/syscall_abi.hh"

class SparcProcess : public Process
{
  protected:

    const Addr StackBias;

    // The locations of the fill and spill handlers
    Addr fillStart, spillStart;

    SparcProcess(ProcessParams * params, ::Loader::ObjectFile *objFile,
                 Addr _StackBias);

    void initState() override;

    template<class IntType>
    void argsInit(int pageSize);

  public:

    // Handles traps which request services from the operating system
    virtual void handleTrap(int trapNum, ThreadContext *tc);

    Addr readFillStart() { return fillStart; }
    Addr readSpillStart() { return spillStart; }

    virtual void flushWindows(ThreadContext *tc) = 0;

    struct SyscallABI
    {
        static const std::vector<int> ArgumentRegs;
    };
};

namespace GuestABI
{

template <typename ABI>
struct Result<ABI, SyscallReturn,
    typename std::enable_if<std::is_base_of<
        SparcProcess::SyscallABI, ABI>::value>::type>
{
    static void
    store(ThreadContext *tc, const SyscallReturn &ret)
    {
        if (ret.suppressed() || ret.needsRetry())
            return;

        // check for error condition.  SPARC syscall convention is to
        // indicate success/failure in reg the carry bit of the ccr
        // and put the return value itself in the standard return value reg.
        SparcISA::PSTATE pstate =
            tc->readMiscRegNoEffect(SparcISA::MISCREG_PSTATE);
        SparcISA::CCR ccr = tc->readIntReg(SparcISA::INTREG_CCR);
        RegVal val;
        if (ret.successful()) {
            ccr.xcc.c = ccr.icc.c = 0;
            val = ret.returnValue();
        } else {
            ccr.xcc.c = ccr.icc.c = 1;
            val = ret.errnoValue();
        }
        tc->setIntReg(SparcISA::INTREG_CCR, ccr);
        if (pstate.am)
            val = bits(val, 31, 0);
        tc->setIntReg(SparcISA::ReturnValueReg, val);
        if (ret.count() == 2)
            tc->setIntReg(SparcISA::SyscallPseudoReturnReg, ret.value2());
    }
};

} // namespace GuestABI

class Sparc32Process : public SparcProcess
{
  protected:

    Sparc32Process(ProcessParams * params, ::Loader::ObjectFile *objFile)
        : SparcProcess(params, objFile, 0)
    {
        Addr brk_point = image.maxAddr();
        brk_point = roundUp(brk_point, SparcISA::PageBytes);

        // Reserve 8M for main stack.
        Addr max_stack_size = 8 * 1024 * 1024;

        // Set up stack. On SPARC Linux, stack goes from the top of memory
        // downward, less the hole for the kernel address space.
        Addr stack_base = 0xf0000000ULL;

        // Set pointer for next thread stack.
        Addr next_thread_stack_base = stack_base - max_stack_size;

        // Set up region for mmaps.
        Addr mmap_end = 0x70000000;

        memState = std::make_shared<MemState>(this, brk_point, stack_base,
                                              max_stack_size,
                                              next_thread_stack_base,
                                              mmap_end);
    }

    void initState() override;

  public:

    void argsInit(int intSize, int pageSize);

    void flushWindows(ThreadContext *tc) override;

    struct SyscallABI : public GenericSyscallABI32,
                        public SparcProcess::SyscallABI
    {};
};

namespace GuestABI
{

template <typename Arg>
struct Argument<Sparc32Process::SyscallABI, Arg,
    typename std::enable_if<
        Sparc32Process::SyscallABI::IsWide<Arg>::value>::type>
{
    using ABI = Sparc32Process::SyscallABI;

    static Arg
    get(ThreadContext *tc, typename ABI::State &state)
    {
        panic_if(state + 1 >= ABI::ArgumentRegs.size(),
                "Ran out of syscall argument registers.");
        auto high = ABI::ArgumentRegs[state++];
        auto low = ABI::ArgumentRegs[state++];
        return (Arg)ABI::mergeRegs(tc, low, high);
    }
};

} // namespace GuestABI

class Sparc64Process : public SparcProcess
{
  protected:

    Sparc64Process(ProcessParams * params, ::Loader::ObjectFile *objFile)
        : SparcProcess(params, objFile, 2047)
    {
        Addr brk_point = image.maxAddr();
        brk_point = roundUp(brk_point, SparcISA::PageBytes);

        Addr max_stack_size = 8 * 1024 * 1024;

        // Set up stack. On SPARC Linux, stack goes from the top of memory
        // downward, less the hole for the kernel address space.
        Addr stack_base = 0x80000000000ULL;

        // Set pointer for next thread stack.  Reserve 8M for main stack.
        Addr next_thread_stack_base = stack_base - max_stack_size;

        // Set up region for mmaps.
        Addr mmap_end = 0xfffff80000000000ULL;

        memState = std::make_shared<MemState>(this, brk_point, stack_base,
                                              max_stack_size,
                                              next_thread_stack_base,
                                              mmap_end);
    }

    void initState() override;

  public:

    void argsInit(int intSize, int pageSize);

    void flushWindows(ThreadContext *tc) override;

    struct SyscallABI : public GenericSyscallABI64,
                        public SparcProcess::SyscallABI
    {};
};

#endif // __SPARC_PROCESS_HH__
