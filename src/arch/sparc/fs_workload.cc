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
 */

#include "arch/sparc/fs_workload.hh"

#include "arch/sparc/faults.hh"
#include "arch/vtophys.hh"
#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "base/trace.hh"
#include "mem/physical.hh"
#include "params/SparcFsWorkload.hh"
#include "sim/byteswap.hh"
#include "sim/system.hh"

namespace
{

ObjectFile *
loadFirmwareImage(const std::string &fname, const std::string &name)
{
    ObjectFile *obj = createObjectFile(fname, true);
    fatal_if(!obj, "Could not load %s %s.", name, fname);
    return obj;
}

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

namespace SparcISA
{

FsWorkload::FsWorkload(Params *p) : OsKernel(*p)
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
    panic_if(!reset->loadGlobalSymbols(resetSymtab),
             "could not load reset symbols");

    panic_if(!openboot->loadGlobalSymbols(openbootSymtab),
             "could not load openboot symbols");

    panic_if(!hypervisor->loadLocalSymbols(hypervisorSymtab),
             "could not load hypervisor symbols");

    panic_if(!nvram->loadLocalSymbols(nvramSymtab),
             "could not load nvram symbols");

    panic_if(!hypervisor_desc->loadLocalSymbols(hypervisorDescSymtab),
             "could not load hypervisor description symbols");

    panic_if(!partition_desc->loadLocalSymbols(partitionDescSymtab),
             "could not load partition description symbols");

    // load symbols into debug table
    panic_if(!reset->loadGlobalSymbols(debugSymbolTable),
             "could not load reset symbols");

    panic_if(!openboot->loadGlobalSymbols(debugSymbolTable),
             "could not load openboot symbols");

    panic_if(!hypervisor->loadLocalSymbols(debugSymbolTable),
             "could not load hypervisor symbols");

    // Strip off the rom address so when the hypervisor is copied into memory
    // we have symbols still
    panic_if(!hypervisor->loadLocalSymbols(debugSymbolTable, 0, 0, 0xFFFFFF),
             "could not load hypervisor symbols");

    panic_if(!nvram->loadGlobalSymbols(debugSymbolTable),
             "could not load reset symbols");

    panic_if(!hypervisor_desc->loadGlobalSymbols(debugSymbolTable),
             "could not load hypervisor description symbols");

    panic_if(!partition_desc->loadLocalSymbols(debugSymbolTable),
             "could not load partition description symbols");

}

void
FsWorkload::initState()
{
    OsKernel::initState();

    if (system->threadContexts.empty())
        return;

    // Other CPUs will get activated by IPIs.
    auto *tc = system->threadContexts[0];
    SparcISA::PowerOnReset().invoke(tc);
    tc->activate();

    auto phys_proxy = system->physProxy;

    writeFirmwareImage(reset, params()->reset_addr, phys_proxy);
    writeFirmwareImage(openboot, params()->openboot_addr, phys_proxy);
    writeFirmwareImage(hypervisor, params()->hypervisor_addr, phys_proxy);
    writeFirmwareImage(nvram, params()->nvram_addr, phys_proxy);
    writeFirmwareImage(
            hypervisor_desc, params()->hypervisor_desc_addr, phys_proxy);
    writeFirmwareImage(
            partition_desc, params()->partition_desc_addr, phys_proxy);
}

FsWorkload::~FsWorkload()
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
FsWorkload::serializeSymtab(CheckpointOut &cp) const
{
    resetSymtab->serialize("reset_symtab", cp);
    hypervisorSymtab->serialize("hypervisor_symtab", cp);
    openbootSymtab->serialize("openboot_symtab", cp);
    nvramSymtab->serialize("nvram_symtab", cp);
    hypervisorDescSymtab->serialize("hypervisor_desc_symtab", cp);
    partitionDescSymtab->serialize("partition_desc_symtab", cp);
}


void
FsWorkload::unserializeSymtab(CheckpointIn &cp)
{
    resetSymtab->unserialize("reset_symtab", cp);
    hypervisorSymtab->unserialize("hypervisor_symtab", cp);
    openbootSymtab->unserialize("openboot_symtab", cp);
    nvramSymtab->unserialize("nvram_symtab", cp);
    hypervisorDescSymtab->unserialize("hypervisor_desc_symtab", cp);
    partitionDescSymtab->unserialize("partition_desc_symtab", cp);
}

} // namespace SparcISA

SparcISA::FsWorkload *
SparcFsWorkloadParams::create()
{
    return new SparcISA::FsWorkload(this);
}
