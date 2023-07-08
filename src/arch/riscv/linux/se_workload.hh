/*
 * Copyright 2004 The Regents of The University of Michigan
 * Copyright 2016 The University of Virginia
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

#ifndef __ARCH_RISCV_LINUX_SE_WORKLOAD_HH__
#define __ARCH_RISCV_LINUX_SE_WORKLOAD_HH__

#include "arch/riscv/linux/linux.hh"
#include "arch/riscv/page_size.hh"
#include "arch/riscv/se_workload.hh"
#include "params/RiscvEmuLinux.hh"
#include "sim/syscall_desc.hh"

namespace gem5
{

namespace RiscvISA
{

class EmuLinux : public SEWorkload
{
  protected:

    /// 64 bit syscall descriptors, indexed by call number.
    static SyscallDescTable<SEWorkload::SyscallABI64> syscallDescs64;

    /// 32 bit syscall descriptors, indexed by call number.
    static SyscallDescTable<SEWorkload::SyscallABI32> syscallDescs32;

  public:
    using Params = RiscvEmuLinuxParams;

    EmuLinux(const Params &p) : SEWorkload(p, PageShift) {}

    ByteOrder byteOrder() const override { return ByteOrder::little; }

    void syscall(ThreadContext *tc) override;
};

} // namespace RiscvISA
} // namespace gem5

#endif // __ARCH_RISCV_LINUX_SE_WORKLOAD_HH__
