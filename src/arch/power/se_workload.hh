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

#ifndef __ARCH_POWER_SE_WORKLOAD_HH__
#define __ARCH_POWER_SE_WORKLOAD_HH__

#include "arch/power/regs/int.hh"
#include "arch/power/regs/misc.hh"
#include "arch/power/remote_gdb.hh"
#include "params/PowerSEWorkload.hh"
#include "sim/se_workload.hh"
#include "sim/syscall_abi.hh"
#include "sim/syscall_desc.hh"

namespace gem5
{

namespace PowerISA
{

class SEWorkload : public gem5::SEWorkload
{
  public:
    using Params = PowerSEWorkloadParams;
    SEWorkload(const Params &p, Addr page_shift) :
        gem5::SEWorkload(p, page_shift)
    {}

    void
    setSystem(System *sys) override
    {
        gem5::SEWorkload::setSystem(sys);
        gdb = BaseRemoteGDB::build<RemoteGDB>(system);
    }

    loader::Arch getArch() const override { return loader::Power; }

    struct SyscallABI : public GenericSyscallABI64
    {
        static const std::vector<RegId> ArgumentRegs;
    };
};

} // namespace PowerISA

GEM5_DEPRECATED_NAMESPACE(GuestABI, guest_abi);
namespace guest_abi
{

template <>
struct Result<PowerISA::SEWorkload::SyscallABI, SyscallReturn>
{
    static void
    store(ThreadContext *tc, const SyscallReturn &ret)
    {
        PowerISA::Cr cr = tc->getReg(PowerISA::int_reg::Cr);
        if (ret.successful()) {
            cr.cr0.so = 0;
        } else {
            cr.cr0.so = 1;
        }
        tc->setReg(PowerISA::int_reg::Cr, cr);
        tc->setReg(PowerISA::ReturnValueReg, ret.encodedValue());
    }
};

} // namespace guest_abi
} // namespace gem5

#endif // __ARCH_POWER_SE_WORKLOAD_HH__
