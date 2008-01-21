/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
 *
 * Redistribution and use of this software in source and binary forms,
 * with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * The software must be used only for Non-Commercial Use which means any
 * use which is NOT directed to receiving any direct monetary
 * compensation for, or commercial advantage from such use.  Illustrative
 * examples of non-commercial use are academic research, personal study,
 * teaching, education and corporate research & development.
 * Illustrative examples of commercial use are distributing products for
 * commercial advantage and providing services using the software for
 * commercial advantage.
 *
 * If you wish to use this software or functionality therein that may be
 * covered by patents for commercial use, please contact:
 *     Director of Intellectual Property Licensing
 *     Office of Strategy and Technology
 *     Hewlett-Packard Company
 *     1501 Page Mill Road
 *     Palo Alto, California  94304
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.  Redistributions
 * in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.  Neither the name of
 * the COPYRIGHT HOLDER(s), HEWLETT-PACKARD COMPANY, nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.  No right of
 * sublicense is granted herewith.  Derivatives of the software and
 * output created using the software may be prepared, but only for
 * Non-Commercial Uses.  Derivatives of the software may be shared with
 * others provided: (i) the others agree to abide by the list of
 * conditions herein which includes the Non-Commercial Use restrictions;
 * and (ii) such Derivatives of the software include the above copyright
 * notice to acknowledge the contribution from this software where
 * applicable, this list of conditions and the disclaimer below.
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
 *
 * Authors: Gabe Black
 */

#include "arch/x86/intregs.hh"
#include "arch/x86/linux/system.hh"
#include "arch/vtophys.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "mem/physical.hh"
#include "params/LinuxX86System.hh"


using namespace LittleEndianGuest;
using namespace X86ISA;

LinuxX86System::LinuxX86System(Params *p)
    : X86System(p), commandLine(p->boot_osflags)
{
}

LinuxX86System::~LinuxX86System()
{
}

void
LinuxX86System::startup()
{
    X86System::startup();

    // The location of the real mode data structure.
    const Addr realModeData = 0x90200;

    // A port to write to memory.
    FunctionalPort * physPort = threadContexts[0]->getPhysPort();

    /*
     * Deal with the command line stuff.
     */

    // A buffer to store the command line.
    const Addr commandLineBuff = 0x90000;
    // A pointer to the commandLineBuff stored in the real mode data.
    const Addr commandLinePointer = realModeData + 0x228;

    if (commandLine.length() + 1 > realModeData - commandLineBuff)
        panic("Command line \"%s\" is longer than %d characters.\n",
                commandLine, realModeData - commandLineBuff - 1);
    physPort->writeBlob(commandLineBuff,
            (uint8_t *)commandLine.c_str(), commandLine.length() + 1);

    // Generate a pointer of the right size and endianness to put into
    // commandLinePointer.
    uint32_t guestCommandLineBuff =
        X86ISA::htog((uint32_t)commandLineBuff);
    physPort->writeBlob(commandLinePointer,
            (uint8_t *)&guestCommandLineBuff, sizeof(guestCommandLineBuff));

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

    struct e820Entry
    {
        Addr addr;
        Addr size;
        uint32_t type;
    };

    // The size is computed this way to ensure no padding sneaks in.
    int e820EntrySize =
        sizeof(e820Entry().addr) +
        sizeof(e820Entry().size) +
        sizeof(e820Entry().type);

    // I'm not sure what these should actually be. On a real machine they
    // would be generated by the BIOS, and they need to reflect the regions
    // which are actually available/reserved. These values are copied from
    // my development machine.
    e820Entry e820Map[] = {
        {ULL(0x0), ULL(0x9d400), 1},
        {ULL(0x9d400), ULL(0xa0000) - ULL(0x9d400), 2},
        {ULL(0xe8000), ULL(0x100000) - ULL(0xe8000), 2},
        {ULL(0x100000), ULL(0xcfff9300) - ULL(0x100000), 1},
        {ULL(0xcfff9300), ULL(0xd0000000) - ULL(0xcfff9300), 2},
        {ULL(0xfec00000), ULL(0x100000000) - ULL(0xfec00000), 2}
    };

    uint8_t e820Nr = sizeof(e820Map) / sizeof(e820Entry);

    // Make sure the number of entries isn't bigger than what the kernel
    // would be capable of providing.
    assert(e820Nr <= 128);

    uint8_t guestE820Nr = X86ISA::htog(e820Nr);
    physPort->writeBlob(e820MapNrPointer,
            (uint8_t *)&guestE820Nr, sizeof(guestE820Nr));

    for (int i = 0; i < e820Nr; i++) {
        e820Entry guestE820Entry;
        guestE820Entry.addr = X86ISA::htog(e820Map[i].addr);
        guestE820Entry.size = X86ISA::htog(e820Map[i].size);
        guestE820Entry.type = X86ISA::htog(e820Map[i].type);
        physPort->writeBlob(e820MapPointer + e820EntrySize * i,
                (uint8_t *)&guestE820Entry.addr,
                sizeof(guestE820Entry.addr));
        physPort->writeBlob(
                e820MapPointer + e820EntrySize * i +
                    sizeof(guestE820Entry.addr),
                (uint8_t *)&guestE820Entry.size,
                sizeof(guestE820Entry.size));
        physPort->writeBlob(
                e820MapPointer + e820EntrySize * i +
                    sizeof(guestE820Entry.addr) +
                    sizeof(guestE820Entry.size),
                (uint8_t *)&guestE820Entry.type,
                sizeof(guestE820Entry.type));
    }

    /*
     * Pass the location of the real mode data structure to the kernel
     * using register %esi. We'll use %rsi which should be equivalent.
     */
    threadContexts[0]->setIntReg(INTREG_RSI, realModeData);
}

LinuxX86System *
LinuxX86SystemParams::create()
{
    return new LinuxX86System(this);
}
