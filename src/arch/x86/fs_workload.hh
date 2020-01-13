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
#include "base/types.hh"
#include "cpu/thread_context.hh"
#include "params/X86FsWorkload.hh"
#include "sim/kernel_workload.hh"

namespace X86ISA
{

namespace SMBios
{

class SMBiosTable;

} // namespace SMBios
namespace IntelMP
{

class FloatingPointer;
class ConfigTable;

} // namespace IntelMP

/* memory mappings for KVMCpu in SE mode */
const Addr syscallCodeVirtAddr = 0xffff800000000000;
const Addr GDTVirtAddr = 0xffff800000001000;
const Addr IDTVirtAddr = 0xffff800000002000;
const Addr TSSVirtAddr = 0xffff800000003000;
const Addr TSSPhysAddr = 0x63000;
const Addr ISTVirtAddr = 0xffff800000004000;
const Addr PFHandlerVirtAddr = 0xffff800000005000;
const Addr MMIORegionVirtAddr = 0xffffc90000000000;
const Addr MMIORegionPhysAddr = 0xffff0000;

void installSegDesc(ThreadContext *tc, SegmentRegIndex seg,
                    SegDescriptor desc, bool longmode);

class FsWorkload : public KernelWorkload
{
  public:
    typedef X86FsWorkloadParams Params;
    FsWorkload(Params *p);

  public:
    void initState() override;

  protected:

    SMBios::SMBiosTable *smbiosTable;
    IntelMP::FloatingPointer *mpFloatingPointer;
    IntelMP::ConfigTable *mpConfigTable;
    ACPI::RSDP *rsdp;

    void writeOutSMBiosTable(Addr header,
            Addr &headerSize, Addr &tableSize, Addr table=0);

    void writeOutMPTable(Addr fp,
            Addr &fpSize, Addr &tableSize, Addr table=0);

    const Params *params() const { return (const Params *)&_params; }
};

} // namespace X86ISA

#endif // __ARCH_X86_FS_WORKLOAD_HH__
