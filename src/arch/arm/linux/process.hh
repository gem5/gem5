/*
 * Copyright (c) 2011-2012 ARM Limited
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
 * Copyright (c) 2007-2008 The Florida State University
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

#ifndef __ARM_LINUX_PROCESS_HH__
#define __ARM_LINUX_PROCESS_HH__

#include <vector>

#include "arch/arm/process.hh"

class ArmLinuxProcessBits
{
  public:
    struct SyscallABI {};
};

namespace GuestABI
{

template <typename ABI>
struct Result<ABI, SyscallReturn,
    typename std::enable_if<std::is_base_of<
        ArmLinuxProcessBits::SyscallABI, ABI>::value>::type>
{
    static void
    store(ThreadContext *tc, const SyscallReturn &ret)
    {
        if (ret.suppressed() || ret.needsRetry())
            return;

        tc->setIntReg(ArmISA::ReturnValueReg, ret.encodedValue());
        if (ret.count() > 1)
            tc->setIntReg(ArmISA::SyscallPseudoReturnReg, ret.value2());
    }
};

} // namespace GuestABI

/// A process with emulated Arm/Linux syscalls.
class ArmLinuxProcess32 : public ArmProcess32, public ArmLinuxProcessBits
{
  public:
    ArmLinuxProcess32(ProcessParams * params, ::Loader::ObjectFile *objFile,
                      ::Loader::Arch _arch);

    void initState() override;

    void syscall(ThreadContext *tc) override;

    /// A page to hold "kernel" provided functions. The name might be wrong.
    static const Addr commPage;

    struct SyscallABI : public ArmProcess32::SyscallABI,
                        public ArmLinuxProcessBits::SyscallABI
    {};
};

/// A process with emulated Arm/Linux syscalls.
class ArmLinuxProcess64 : public ArmProcess64, public ArmLinuxProcessBits
{
  public:
    ArmLinuxProcess64(ProcessParams * params, ::Loader::ObjectFile *objFile,
                      ::Loader::Arch _arch);

    void initState() override;
    void syscall(ThreadContext *tc) override;

    struct SyscallABI : public ArmProcess64::SyscallABI,
                        public ArmLinuxProcessBits::SyscallABI
    {};
};

#endif // __ARM_LINUX_PROCESS_HH__
