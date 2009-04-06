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
#include "base/remote_gdb.hh"
#include "base/loader/object_file.hh"
#include "base/loader/hex_file.hh"
#include "base/loader/symtab.hh"
#include "base/trace.hh"
#include "mem/physical.hh"
#include "params/MipsSystem.hh"
#include "sim/byteswap.hh"


using namespace LittleEndianGuest;

MipsSystem::MipsSystem(Params *p)
    : System(p)
{

#if FULL_SYSTEM
    if (p->bare_iron == true) {
        hexFile = new HexFile(params()->hex_file_name);
        if(!hexFile->loadSections(&functionalPort,MipsISA::LoadAddrMask))
            panic("Could not load hex file\n");
    }

    Addr addr = 0;
    /* Comment out old Alpha Based Code

     Don't need the console before we start looking at booting linux */


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
     console->loadSections(&functionalPort, MipsISA::LoadAddrMask);

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
        cout << "-" << endl;
        virtPort.writeBlob(addr, (uint8_t*)params()->boot_osflags.c_str(),
                strlen(params()->boot_osflags.c_str()));
    }

    /**
     * Set the hardware reset parameter block system type and revision
     * information to Tsunami.
     */
    if (consoleSymtab->findAddress("m5_rpb", addr)) {
        uint64_t data;
        data = htog(params()->system_type);
        virtPort.write(addr+0x50, data);
        data = htog(params()->system_rev);
        virtPort.write(addr+0x58, data);
    } else
        panic("could not find hwrpb\n");
#endif
}

MipsSystem::~MipsSystem()
{
}
#if FULL_SYSTEM
/**
 * This function fixes up addresses that are used to match PCs for
 * hooking simulator events on to target function executions.
 *
 * Mips binaries may have multiple global offset table (GOT)
 * sections.  A function that uses the GOT starts with a
 * two-instruction prolog which sets the global pointer (gp == r29) to
 * the appropriate GOT section.  The proper gp value is calculated
 * based on the function address, which must be passed by the caller
 * in the procedure value register (pv aka t12 == r27).  This sequence
 * looks like the following:
 *
 *                      opcode Ra Rb offset
 *      ldah gp,X(pv)     09   29 27   X
 *      lda  gp,Y(gp)     08   29 29   Y
 *
 * for some constant offsets X and Y.  The catch is that the linker
 * (or maybe even the compiler, I'm not sure) may recognize that the
 * caller and callee are using the same GOT section, making this
 * prolog redundant, and modify the call target to skip these
 * instructions.  If we check for execution of the first instruction
 * of a function (the one the symbol points to) to detect when to skip
 * it, we'll miss all these modified calls.  It might work to
 * unconditionally check for the third instruction, but not all
 * functions have this prolog, and there's some chance that those
 * first two instructions could have undesired consequences.  So we do
 * the Right Thing and pattern-match the first two instructions of the
 * function to decide where to patch.
 *
 * Eventually this code should be moved into an ISA-specific file.
 */

Addr
MipsSystem::fixFuncEventAddr(Addr addr)
{
  /*
    // mask for just the opcode, Ra, and Rb fields (not the offset)
    const uint32_t inst_mask = 0xffff0000;
    // ldah gp,X(pv): opcode 9, Ra = 29, Rb = 27
    const uint32_t gp_ldah_pattern = (9 << 26) | (29 << 21) | (27 << 16);
    // lda  gp,Y(gp): opcode 8, Ra = 29, rb = 29
    const uint32_t gp_lda_pattern  = (8 << 26) | (29 << 21) | (29 << 16);

    uint32_t i1 = virtPort.read<uint32_t>(addr);
    uint32_t i2 = virtPort.read<uint32_t>(addr + sizeof(MipsISA::MachInst));

    if ((i1 & inst_mask) == gp_ldah_pattern &&
        (i2 & inst_mask) == gp_lda_pattern) {
        Addr new_addr = addr + 2* sizeof(MipsISA::MachInst);
        DPRINTF(Loader, "fixFuncEventAddr: %p -> %p", addr, new_addr);
        return new_addr;
    } else {
        return addr;
        }*/
  return addr;
}


void
MipsSystem::setMipsAccess(Addr access)
{
    Addr addr = 0;
    if (consoleSymtab->findAddress("m5MipsAccess", addr)) {
      //        virtPort.write(addr, htog(AlphaISA::Phys2K0Seg(access)));
    } else
    panic("could not find m5MipsAccess\n");
    }

#endif

bool
MipsSystem::breakpoint()
{
  return 0;
  //    return remoteGDB[0]->trap(MIPS_KENTRY_INT);
}

void
MipsSystem::serialize(std::ostream &os)
{
    System::serialize(os);
    //    consoleSymtab->serialize("console_symtab", os);
}


void
MipsSystem::unserialize(Checkpoint *cp, const std::string &section)
{
    System::unserialize(cp,section);
    //    consoleSymtab->unserialize("console_symtab", cp, section);
}

MipsSystem *
MipsSystemParams::create()
{
    return new MipsSystem(this);
}

