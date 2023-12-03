/*
 * Copyright (c) 2007-2008 The Hewlett-Packard Development Company
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

#include "arch/x86/linux/fs_workload.hh"

#include "arch/x86/regs/int.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "mem/port_proxy.hh"
#include "params/X86FsLinux.hh"
#include "sim/byteswap.hh"
#include "sim/system.hh"

namespace gem5
{

namespace X86ISA
{

FsLinux::FsLinux(const Params &p)
    : X86ISA::FsWorkload(p), e820Table(p.e820_table)
{}

void
FsLinux::initState()
{
    X86ISA::FsWorkload::initState();

    auto phys_proxy = system->physProxy;

    // The location of the real mode data structure.
    const Addr realModeData = 0x90200;

    /*
     * Deal with the command line stuff.
     */

    // A buffer to store the command line.
    const Addr commandLineBuff = 0x90000;
    // A pointer to the commandLineBuff stored in the real mode data.
    const Addr commandLinePointer = realModeData + 0x228;

    panic_if(commandLine.length() + 1 > realModeData - commandLineBuff,
             "Command line \"%s\" is longer than %d characters.", commandLine,
             realModeData - commandLineBuff - 1);
    phys_proxy.writeString(commandLineBuff, commandLine.c_str());

    // Generate a pointer of the right size and endianness to put into
    // commandLinePointer.
    uint32_t guestCommandLineBuff = htole((uint32_t)commandLineBuff);
    phys_proxy.writeBlob(commandLinePointer, &guestCommandLineBuff,
                         sizeof(guestCommandLineBuff));

    /*
     * Screen Info.
     */

    // We'll skip on this for now because it's only needed for framebuffers,
    // something we don't support at the moment.

    /*
     * EDID info
     */

    // Skipping for now.

    /*
     * Saved video mode
     */

    // Skipping for now.

    /*
     * Loader type.
     */

    // Skipping for now.

    /*
     * E820 memory map
     */

    // A pointer to the number of E820 entries there are.
    const Addr e820MapNrPointer = realModeData + 0x1e8;

    // A pointer to the buffer for E820 entries.
    const Addr e820MapPointer = realModeData + 0x2d0;

    e820Table->writeTo(phys_proxy, e820MapNrPointer, e820MapPointer);

    /*
     * Pass the location of the real mode data structure to the kernel
     * using register %esi. We'll use %rsi which should be equivalent.
     */
    system->threads[0]->setReg(int_reg::Rsi, realModeData);
}

} // namespace X86ISA
} // namespace gem5
