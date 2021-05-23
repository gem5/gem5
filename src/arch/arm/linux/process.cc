/*
 * Copyright (c) 2010-2013, 2015, 2020 ARM Limited
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
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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

#include "arch/arm/linux/process.hh"

#include <sys/syscall.h>

#include "arch/arm/linux/linux.hh"
#include "arch/arm/page_size.hh"
#include "base/loader/object_file.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "kern/linux/linux.hh"
#include "mem/se_translating_port_proxy.hh"
#include "sim/process.hh"
#include "sim/syscall_desc.hh"
#include "sim/syscall_emul.hh"
#include "sim/system.hh"

namespace gem5
{

using namespace ArmISA;

const Addr ArmLinuxProcess32::commPage = 0xffff0000;

void
ArmLinuxProcess32::initState()
{
    ArmProcess32::initState();
    allocateMem(commPage, PageBytes);
    ThreadContext *tc = system->threads[contextIds[0]];

    uint8_t swiNeg1[] = {
        0xff, 0xff, 0xff, 0xef  // swi -1
    };

    SETranslatingPortProxy proxy(tc);
    // Fill this page with swi -1 so we'll no if we land in it somewhere.
    for (Addr addr = 0; addr < PageBytes; addr += sizeof(swiNeg1)) {
        proxy.writeBlob(commPage + addr, swiNeg1, sizeof(swiNeg1));
    }

    uint8_t memory_barrier[] =
    {
        0x5f, 0xf0, 0x7f, 0xf5, // dmb
        0x0e, 0xf0, 0xa0, 0xe1  // return
    };
    proxy.writeBlob(commPage + 0x0fa0, memory_barrier, sizeof(memory_barrier));

    uint8_t cmpxchg[] =
    {
        0x9f, 0x3f, 0x92, 0xe1,  // ldrex    r3, [r2]
        0x00, 0x30, 0x53, 0xe0,  // subs     r3, r3, r0
        0x91, 0x3f, 0x82, 0x01,  // strexeq  r3, r1, [r2]
        0x01, 0x00, 0x33, 0x03,  // teqeq    r3, #1
        0xfa, 0xff, 0xff, 0x0a,  // beq 1b
        0x00, 0x00, 0x73, 0xe2,  // rsbs r0, r3, #0
        0x5f, 0xf0, 0x7f, 0xf5,  // dmb
        0x0e, 0xf0, 0xa0, 0xe1   // return
    };
    proxy.writeBlob(commPage + 0x0fc0, cmpxchg, sizeof(cmpxchg));

    uint8_t get_tls[] =
    {
                                // read user read-only thread id register
        0x70, 0x0f, 0x1d, 0xee, // mrc p15, 0, r0, c13, c0, 3
        0x0e, 0xf0, 0xa0, 0xe1  // return
    };
    proxy.writeBlob(commPage + 0x0fe0, get_tls, sizeof(get_tls));
}

void
ArmLinuxProcess64::initState()
{
    ArmProcess64::initState();
    // The 64 bit equivalent of the comm page would be set up here.
}

} // namespace gem5
