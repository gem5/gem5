/*
 * Copyright 2020 Google Inc.
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

#ifndef __ARCH_SPARC_SE_WORKLOAD_HH__
#define __ARCH_SPARC_SE_WORKLOAD_HH__

#include <vector>

#include "arch/sparc/regs/int.hh"
#include "arch/sparc/regs/misc.hh"
#include "arch/sparc/remote_gdb.hh"
#include "base/loader/object_file.hh"
#include "cpu/thread_context.hh"
#include "params/SparcSEWorkload.hh"
#include "sim/se_workload.hh"
#include "sim/syscall_abi.hh"

namespace gem5
{

namespace SparcISA
{

class SEWorkload : public gem5::SEWorkload
{
  public:
    PARAMS(SparcSEWorkload);
    using gem5::SEWorkload::SEWorkload;

    void
    setSystem(System *sys) override
    {
        gem5::SEWorkload::setSystem(sys);
        gdb =
            BaseRemoteGDB::build<RemoteGDB>(params().remote_gdb_port, system);
    }

    virtual void handleTrap(ThreadContext *tc, int trapNum);
    virtual void flushWindows(ThreadContext *tc);

    bool is64(ThreadContext *tc);

    struct BaseSyscallABI
    {
        static const std::vector<RegId> ArgumentRegs;
    };

    struct SyscallABI32 : public GenericSyscallABI32, public BaseSyscallABI
    {
    };

    struct SyscallABI64 : public GenericSyscallABI64, public BaseSyscallABI
    {
    };
};

} // namespace SparcISA

namespace guest_abi
{

template <typename ABI>
struct Result<ABI, SyscallReturn,
              typename std::enable_if_t<std::is_base_of_v<
                  SparcISA::SEWorkload::BaseSyscallABI, ABI>>>
{
    static void
    store(ThreadContext *tc, const SyscallReturn &ret)
    {
        // check for error condition.  SPARC syscall convention is to
        // indicate success/failure in reg the carry bit of the ccr
        // and put the return value itself in the standard return value reg.
        SparcISA::PSTATE pstate =
            tc->readMiscRegNoEffect(SparcISA::MISCREG_PSTATE);
        SparcISA::CCR ccr = tc->getReg(SparcISA::int_reg::Ccr);
        RegVal val;
        if (ret.successful()) {
            ccr.xcc.c = ccr.icc.c = 0;
            val = ret.returnValue();
        } else {
            ccr.xcc.c = ccr.icc.c = 1;
            val = ret.errnoValue();
        }
        tc->setReg(SparcISA::int_reg::Ccr, ccr);
        if (pstate.am)
            val = bits(val, 31, 0);
        tc->setReg(SparcISA::ReturnValueReg, val);
        if (ret.count() == 2)
            tc->setReg(SparcISA::SyscallPseudoReturnReg, ret.value2());
    }
};

template <typename Arg>
struct Argument<SparcISA::SEWorkload::SyscallABI32, Arg,
                typename std::enable_if_t<
                    std::is_integral_v<Arg> &&
                    SparcISA::SEWorkload::SyscallABI32::IsWideV<Arg>>>
{
    using ABI = SparcISA::SEWorkload::SyscallABI32;

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

} // namespace guest_abi
} // namespace gem5

#endif // __ARCH_SPARC_SE_WORKLOAD_HH__
