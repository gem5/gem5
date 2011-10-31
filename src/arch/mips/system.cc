/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * Copyright (c) 2007 MIPS Technologies, Inc.
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
 *
 * Authors: Ali Saidi
 *          Nathan Binkert
 *          Jaidev Patwardhan
 */

#include "arch/mips/system.hh"
#include "arch/vtophys.hh"
#include "base/loader/hex_file.hh"
#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "base/trace.hh"
#include "mem/physical.hh"
#include "mem/vport.hh"
#include "params/MipsSystem.hh"
#include "sim/byteswap.hh"

using namespace LittleEndianGuest;

MipsSystem::MipsSystem(Params *p) : System(p)
{
    if (p->bare_iron == true) {
        hexFile = new HexFile(params()->hex_file_name);
        if (!hexFile->loadSections(functionalPort))
            panic("Could not load hex file\n");
    }

    Addr addr = 0;

    consoleSymtab = new SymbolTable;


    /**
     * Load the console code into memory
     */
    //    Load Console Code
    console = createObjectFile(params()->console);

    warn("console code is located at: %s\n", params()->console);

    if (console == NULL)
        fatal("Could not load console file %s", params()->console);
    //Load program sections into memory
    console->loadSections(functionalPort, loadAddrMask);

    //load symbols
    if (!console->loadGlobalSymbols(consoleSymtab))
        panic("could not load console symbols\n");

    if (!console->loadGlobalSymbols(debugSymbolTable))
        panic("could not load console symbols\n");


#ifndef NDEBUG
    consolePanicEvent = addConsoleFuncEvent<BreakPCEvent>("panic");
#endif

    /**
     * Copy the osflags (kernel arguments) into the consoles
     * memory. (Presently Linux does not use the console service
     * routine to get these command line arguments, but Tru64 and
     * others do.)
     */
    if (consoleSymtab->findAddress("env_booted_osflags", addr)) {
        warn("writing addr starting from %#x", addr);
        virtPort->writeBlob(addr, (uint8_t*)params()->boot_osflags.c_str(),
                strlen(params()->boot_osflags.c_str()));
    }

    /**
     * Set the hardware reset parameter block system type and revision
     * information to Tsunami.
     */
    if (consoleSymtab->findAddress("m5_rpb", addr)) {
        uint64_t data;
        data = htog(params()->system_type);
        virtPort->write(addr + 0x50, data);
        data = htog(params()->system_rev);
        virtPort->write(addr + 0x58, data);
    } else {
        panic("could not find hwrpb\n");
    }
}

MipsSystem::~MipsSystem()
{
}

Addr
MipsSystem::fixFuncEventAddr(Addr addr)
{
    return addr;
}

void
MipsSystem::setMipsAccess(Addr access)
{}

bool
MipsSystem::breakpoint()
{
    return 0;
}

void
MipsSystem::serialize(std::ostream &os)
{
    System::serialize(os);
}


void
MipsSystem::unserialize(Checkpoint *cp, const std::string &section)
{
    System::unserialize(cp,section);
}

MipsSystem *
MipsSystemParams::create()
{
    return new MipsSystem(this);
}

