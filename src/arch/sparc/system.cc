/*
 * Copyright (c) 2002-2006 The Regents of The University of Michigan
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
 */

#include "arch/sparc/system.hh"
#include "arch/vtophys.hh"
#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "base/trace.hh"
#include "mem/physical.hh"
#include "params/SparcSystem.hh"
#include "sim/byteswap.hh"

using namespace BigEndianGuest;

SparcSystem::SparcSystem(Params *p)
    : System(p), sysTick(0)
{
    resetSymtab = new SymbolTable;
    hypervisorSymtab = new SymbolTable;
    openbootSymtab = new SymbolTable;
    nvramSymtab = new SymbolTable;
    hypervisorDescSymtab = new SymbolTable;
    partitionDescSymtab = new SymbolTable;

    /**
     * Load the boot code, and hypervisor into memory.
     */
    // Read the reset binary
    reset = createObjectFile(params()->reset_bin, true);
    if (reset == NULL)
        fatal("Could not load reset binary %s", params()->reset_bin);

    // Read the openboot binary
    openboot = createObjectFile(params()->openboot_bin, true);
    if (openboot == NULL)
        fatal("Could not load openboot bianry %s", params()->openboot_bin);

    // Read the hypervisor binary
    hypervisor = createObjectFile(params()->hypervisor_bin, true);
    if (hypervisor == NULL)
        fatal("Could not load hypervisor binary %s", params()->hypervisor_bin);

    // Read the nvram image
    nvram = createObjectFile(params()->nvram_bin, true);
    if (nvram == NULL)
        fatal("Could not load nvram image %s", params()->nvram_bin);

    // Read the hypervisor description image
    hypervisor_desc = createObjectFile(params()->hypervisor_desc_bin, true);
    if (hypervisor_desc == NULL)
        fatal("Could not load hypervisor description image %s",
                params()->hypervisor_desc_bin);

    // Read the partition description image
    partition_desc = createObjectFile(params()->partition_desc_bin, true);
    if (partition_desc == NULL)
        fatal("Could not load partition description image %s",
                params()->partition_desc_bin);

    // load symbols
    if (!reset->loadGlobalSymbols(resetSymtab))
        panic("could not load reset symbols\n");

    if (!openboot->loadGlobalSymbols(openbootSymtab))
        panic("could not load openboot symbols\n");

    if (!hypervisor->loadLocalSymbols(hypervisorSymtab))
        panic("could not load hypervisor symbols\n");

    if (!nvram->loadLocalSymbols(nvramSymtab))
        panic("could not load nvram symbols\n");

    if (!hypervisor_desc->loadLocalSymbols(hypervisorDescSymtab))
        panic("could not load hypervisor description symbols\n");

    if (!partition_desc->loadLocalSymbols(partitionDescSymtab))
        panic("could not load partition description symbols\n");

    // load symbols into debug table
    if (!reset->loadGlobalSymbols(debugSymbolTable))
        panic("could not load reset symbols\n");

    if (!openboot->loadGlobalSymbols(debugSymbolTable))
        panic("could not load openboot symbols\n");

    if (!hypervisor->loadLocalSymbols(debugSymbolTable))
        panic("could not load hypervisor symbols\n");

    // Strip off the rom address so when the hypervisor is copied into memory we
    // have symbols still
    if (!hypervisor->loadLocalSymbols(debugSymbolTable, 0, 0, 0xFFFFFF))
        panic("could not load hypervisor symbols\n");

    if (!nvram->loadGlobalSymbols(debugSymbolTable))
        panic("could not load reset symbols\n");

    if (!hypervisor_desc->loadGlobalSymbols(debugSymbolTable))
        panic("could not load hypervisor description symbols\n");

    if (!partition_desc->loadLocalSymbols(debugSymbolTable))
        panic("could not load partition description symbols\n");

}

void
SparcSystem::initState()
{
    // Call the initialisation of the super class
    System::initState();

    // Load reset binary into memory
    reset->setTextBase(params()->reset_addr);
    reset->loadSections(physProxy);
    // Load the openboot binary
    openboot->setTextBase(params()->openboot_addr);
    openboot->loadSections(physProxy);
    // Load the hypervisor binary
    hypervisor->setTextBase(params()->hypervisor_addr);
    hypervisor->loadSections(physProxy);
    // Load the nvram image
    nvram->setTextBase(params()->nvram_addr);
    nvram->loadSections(physProxy);
    // Load the hypervisor description image
    hypervisor_desc->setTextBase(params()->hypervisor_desc_addr);
    hypervisor_desc->loadSections(physProxy);
    // Load the partition description image
    partition_desc->setTextBase(params()->partition_desc_addr);
    partition_desc->loadSections(physProxy);


    // @todo any fixup code over writing data in binaries on setting break
    // events on functions should happen here.

}

SparcSystem::~SparcSystem()
{
    delete resetSymtab;
    delete hypervisorSymtab;
    delete openbootSymtab;
    delete nvramSymtab;
    delete hypervisorDescSymtab;
    delete partitionDescSymtab;
    delete reset;
    delete openboot;
    delete hypervisor;
    delete nvram;
    delete hypervisor_desc;
    delete partition_desc;
}

void
SparcSystem::serializeSymtab(CheckpointOut &cp) const
{
    resetSymtab->serialize("reset_symtab", cp);
    hypervisorSymtab->serialize("hypervisor_symtab", cp);
    openbootSymtab->serialize("openboot_symtab", cp);
    nvramSymtab->serialize("nvram_symtab", cp);
    hypervisorDescSymtab->serialize("hypervisor_desc_symtab", cp);
    partitionDescSymtab->serialize("partition_desc_symtab", cp);
}


void
SparcSystem::unserializeSymtab(CheckpointIn &cp)
{
    resetSymtab->unserialize("reset_symtab", cp);
    hypervisorSymtab->unserialize("hypervisor_symtab", cp);
    openbootSymtab->unserialize("openboot_symtab", cp);
    nvramSymtab->unserialize("nvram_symtab", cp);
    hypervisorDescSymtab->unserialize("hypervisor_desc_symtab", cp);
    partitionDescSymtab->unserialize("partition_desc_symtab", cp);
}

SparcSystem *
SparcSystemParams::create()
{
    return new SparcSystem(this);
}
