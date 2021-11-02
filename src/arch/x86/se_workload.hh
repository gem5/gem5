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

#ifndef __ARCH_X86_SE_WORKLOAD_HH__
#define __ARCH_X86_SE_WORKLOAD_HH__

#include "base/types.hh"

namespace gem5
{

namespace X86ISA
{

/* memory mappings for KVMCpu in SE mode */
const Addr syscallCodeVirtAddr = 0xffff800000000000;
const Addr GDTVirtAddr = 0xffff800000001000;
const Addr IDTVirtAddr = 0xffff800000002000;
const Addr TSSVirtAddr = 0xffff800000003000;
const Addr TSSPhysAddr = 0x63000;
const Addr ISTVirtAddr = 0xffff800000004000;
const Addr PFHandlerVirtAddr = 0xffff800000005000;
const Addr MMIORegionVirtAddr = 0xffffc90000000000;

} // namespace X86ISA
} // namespace gem5

#endif // __ARCH_X86_SE_WORKLOAD_HH__
