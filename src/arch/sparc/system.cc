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

#include "arch/sparc/faults.hh"
#include "arch/vtophys.hh"
#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "base/trace.hh"
#include "mem/physical.hh"
#include "params/SparcSystem.hh"
#include "sim/byteswap.hh"

namespace
{

ObjectFile *
loadFirmwareImage(const std::string &fname, const std::string &name)
{
    ObjectFile *obj = createObjectFile(fname, true);
    fatal_if(!obj, "Could not load %s %s.", name, fname);
    return obj;
}

} // anonymous namespace

SparcSystem::SparcSystem(Params *p)
    : System(p), sysTick(0)
{
    resetSymtab = new SymbolTable;
    hypervisorSymtab = new SymbolTable;
    openbootSymtab = new SymbolTable;
    nvramSymtab = new SymbolTable;
    hypervisorDescSymtab = new SymbolTable;
    partitionDescSymtab = new SymbolTable;

    reset = loadFirmwareImage(params()->reset_bin, "reset binary");
    openboot = loadFirmwareImage(params()->openboot_bin, "openboot binary");
    hypervisor = loadFirmwareImage(
            params()->hypervisor_bin, "hypervisor binary");
    nvram = loadFirmwareImage(params()->nvram_bin, "nvram image");
    hypervisor_desc = loadFirmwareImage(
            params()->hypervisor_desc_bin, "hypervisor description image");
    partition_desc = loadFirmwareImage(
            params()->partition_desc_bin, "partition description image");

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

namespace
{

void
writeFirmwareImage(ObjectFile *obj, Addr addr, const PortProxy &proxy)
{
    MemoryImage image = obj->buildImage();

    // If the entry point isn't somewhere in the image, we assume we need to
    // move where it's loaded so that it is.
    if (addr < image.minAddr() || addr >= image.maxAddr()) {
        // Move the image by the difference between the expected entry address,
        // and the entry point in the object file.
        image.offset(addr - obj->entryPoint());
    }

    image.write(proxy);
}

} // anonymous namespace

void
SparcSystem::initState()
{
    // Call the initialisation of the super class
    System::initState();

    writeFirmwareImage(reset, params()->reset_addr, physProxy);
    writeFirmwareImage(openboot, params()->openboot_addr, physProxy);
    writeFirmwareImage(hypervisor, params()->hypervisor_addr, physProxy);
    writeFirmwareImage(nvram, params()->nvram_addr, physProxy);
    writeFirmwareImage(
            hypervisor_desc, params()->hypervisor_desc_addr, physProxy);
    writeFirmwareImage(
            partition_desc, params()->partition_desc_addr, physProxy);

    // @todo any fixup code over writing data in binaries on setting break
    // events on functions should happen here.

    if (threadContexts.empty())
        return;

    // Other CPUs will get activated by IPIs.
    auto *tc = threadContexts[0];
    SparcISA::PowerOnReset().invoke(tc);
    tc->activate();
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
