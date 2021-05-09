/*
 * Copyright 2015 Ruslan Bukin <br@bsdpad.com>
 *
 * This software was developed by the University of Cambridge Computer
 * Laboratory as part of the CTSRD Project, with support from the UK Higher
 * Education Innovation Fund (HEIF).
 *
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

#ifndef __ARCH_ARM_FREEBSD_SE_WORKLOAD_HH__
#define __ARCH_ARM_FREEBSD_SE_WORKLOAD_HH__

#include "arch/arm/freebsd/freebsd.hh"
#include "arch/arm/regs/cc.hh"
#include "arch/arm/regs/int.hh"
#include "arch/arm/se_workload.hh"
#include "params/ArmEmuFreebsd.hh"
#include "sim/syscall_desc.hh"

namespace gem5
{

namespace ArmISA
{

class EmuFreebsd : public SEWorkload
{
  public:
    using Params = ArmEmuFreebsdParams;

    EmuFreebsd(const Params &p) : SEWorkload(p) {}

    struct BaseSyscallABI {};
    struct SyscallABI32 : public SEWorkload::SyscallABI32,
                          public BaseSyscallABI
    {};
    struct SyscallABI64 : public SEWorkload::SyscallABI64,
                          public BaseSyscallABI
    {};

    void syscall(ThreadContext *tc) override;
};

} // namespace ArmISA

GEM5_DEPRECATED_NAMESPACE(GuestABI, guest_abi);
namespace guest_abi
{

template <typename ABI>
struct Result<ABI, SyscallReturn,
    typename std::enable_if_t<std::is_base_of<
        ArmISA::EmuFreebsd::BaseSyscallABI, ABI>::value>>
{
    static void
    store(ThreadContext *tc, const SyscallReturn &ret)
    {
        if (ret.suppressed() || ret.needsRetry())
            return;

        RegVal val;
        if (ret.successful()) {
            tc->setCCReg(ArmISA::CCREG_C, 0);
            val = ret.returnValue();
        } else {
            tc->setCCReg(ArmISA::CCREG_C, 1);
            val = ret.encodedValue();
        }
        tc->setIntReg(ArmISA::ReturnValueReg, val);
        if (ret.count() > 1)
            tc->setIntReg(ArmISA::SyscallPseudoReturnReg, ret.value2());
    }
};

} // namespace guest_abi
} // namespace gem5

#endif // __ARCH_ARM_FREEBSD_SE_WORKLOAD_HH__
