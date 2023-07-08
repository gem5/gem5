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

#ifndef __ARCH_RISCV_SE_WORKLOAD_HH__
#define __ARCH_RISCV_SE_WORKLOAD_HH__

#include "arch/riscv/reg_abi.hh"
#include "arch/riscv/regs/int.hh"
#include "arch/riscv/remote_gdb.hh"
#include "params/RiscvSEWorkload.hh"
#include "sim/se_workload.hh"
#include "sim/syscall_abi.hh"

namespace gem5
{

namespace RiscvISA
{

class SEWorkload : public gem5::SEWorkload
{
  public:
    PARAMS(RiscvSEWorkload);

    SEWorkload(const Params &p, Addr page_shift) :
        gem5::SEWorkload(p, page_shift)
    {}

    void
    setSystem(System *sys) override
    {
        gem5::SEWorkload::setSystem(sys);
        gdb = BaseRemoteGDB::build<RemoteGDB>(
                params().remote_gdb_port, system);
    }

    loader::Arch getArch() const override { return loader::Riscv64; }

    using SyscallABI64 = RegABI64;
    using SyscallABI32 = RegABI32;
};

} // namespace RiscvISA

namespace guest_abi
{

template <>
struct Result<RiscvISA::SEWorkload::SyscallABI64, SyscallReturn>
{
    static void
    store(ThreadContext *tc, const SyscallReturn &ret)
    {
        if (ret.successful()) {
            // no error
            tc->setReg(RiscvISA::ReturnValueReg, ret.returnValue());
        } else {
            // got an error, return details
            tc->setReg(RiscvISA::ReturnValueReg, ret.encodedValue());
        }
    }
};

template <>
struct Result<RiscvISA::SEWorkload::SyscallABI32, SyscallReturn>
{
    static void
    store(ThreadContext *tc, const SyscallReturn &ret)
    {
        if (ret.successful()) {
            // no error
            tc->setReg(RiscvISA::ReturnValueReg, sext<32>(ret.returnValue()));
        } else {
            // got an error, return details
            tc->setReg(RiscvISA::ReturnValueReg, sext<32>(ret.encodedValue()));
        }
    }
};

} // namespace guest_abi
} // namespace gem5

#endif // __ARCH_RISCV_SE_WORKLOAD_HH__
