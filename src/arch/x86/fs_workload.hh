/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
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

#ifndef __ARCH_X86_FS_WORKLOAD_HH__
#define __ARCH_X86_FS_WORKLOAD_HH__

#include <string>
#include <vector>

#include "arch/x86/regs/misc.hh"
#include "arch/x86/regs/segment.hh"
#include "arch/x86/remote_gdb.hh"
#include "base/types.hh"
#include "cpu/thread_context.hh"
#include "params/X86FsWorkload.hh"
#include "sim/kernel_workload.hh"

namespace gem5
{

namespace X86ISA
{

namespace smbios
{

class SMBiosTable;

} // namespace smbios

namespace intelmp
{

class FloatingPointer;
class ConfigTable;

} // namespace intelmp

void installSegDesc(ThreadContext *tc, int seg, SegDescriptor desc,
                    bool longmode);

class FsWorkload : public KernelWorkload
{
  public:
    PARAMS(X86FsWorkload);
    FsWorkload(const Params &p);

  public:
    void initState() override;

    void
    setSystem(System *sys) override
    {
        KernelWorkload::setSystem(sys);
        gdb =
            BaseRemoteGDB::build<RemoteGDB>(params().remote_gdb_port, system);
    }

    ByteOrder
    byteOrder() const override
    {
        return ByteOrder::little;
    }

  protected:
    smbios::SMBiosTable *smbiosTable;
    intelmp::FloatingPointer *mpFloatingPointer;
    intelmp::ConfigTable *mpConfigTable;
    ACPI::RSDP *rsdp;

    void writeOutSMBiosTable(Addr header, Addr &headerSize, Addr &tableSize,
                             Addr table = 0);

    void writeOutMPTable(Addr fp, Addr &fpSize, Addr &tableSize,
                         Addr table = 0);

    void writeOutACPITables(Addr begin, Addr &size);

  private:
    bool enable_osxsave;
};

} // namespace X86ISA
} // namespace gem5

#endif // __ARCH_X86_FS_WORKLOAD_HH__
