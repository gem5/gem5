/*
 * Copyright 2003-2004 The Regents of The University of Michigan
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

#ifndef __ARCH_SPARC_LINUX_SE_WORKLOAD_HH__
#define __ARCH_SPARC_LINUX_SE_WORKLOAD_HH__

#include "arch/sparc/linux/linux.hh"
#include "arch/sparc/se_workload.hh"
#include "params/SparcEmuLinux.hh"
#include "sim/syscall_desc.hh"

namespace gem5
{

namespace SparcISA
{

class EmuLinux : public SEWorkload
{
  protected:
    /// 64 bit syscall descriptors, indexed by call number.
    static SyscallDescTable<SEWorkload::SyscallABI64> syscallDescs;

    /// 32 bit compatibility syscall descriptors, indexed by call number.
    static SyscallDescTable<SEWorkload::SyscallABI32> syscall32Descs;

    void syscall64(ThreadContext *tc);
    void syscall32(ThreadContext *tc);

  public:
    using Params = SparcEmuLinuxParams;

    EmuLinux(const Params &p);

    loader::Arch
    getArch() const override
    {
        return loader::SPARC64;
    }

    ByteOrder
    byteOrder() const override
    {
        return ByteOrder::big;
    }

    void handleTrap(ThreadContext *tc, int trapNum) override;
    void syscall(ThreadContext *tc) override;
};

} // namespace SparcISA
} // namespace gem5

#endif // __ARCH_SPARC_LINUX_SE_WORKLOAD_HH__
