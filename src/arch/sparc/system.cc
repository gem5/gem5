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
#include "base/remote_gdb.hh"
#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "base/trace.hh"
#include "mem/physical.hh"
#include "sim/byteswap.hh"
#include "sim/builder.hh"


using namespace BigEndianGuest;

SparcSystem::SparcSystem(Params *p)
    : System(p), sysTick(0),funcRomPort(p->name + "-fport")

{
    resetSymtab = new SymbolTable;
    hypervisorSymtab = new SymbolTable;
    openbootSymtab = new SymbolTable;

    Port *rom_port;
    rom_port = params()->rom->getPort("functional");
    funcRomPort.setPeer(rom_port);
    rom_port->setPeer(&funcRomPort);

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


    // Load reset binary into memory
    reset->setTextBase(params()->reset_addr);
    reset->loadSections(&funcRomPort);
    // Load the openboot binary
    openboot->setTextBase(params()->openboot_addr);
    openboot->loadSections(&funcRomPort);
    // Load the hypervisor binary
    hypervisor->setTextBase(params()->hypervisor_addr);
    hypervisor->loadSections(&funcRomPort);

    // load symbols
    if (!reset->loadGlobalSymbols(resetSymtab))
        panic("could not load reset symbols\n");

    if (!openboot->loadGlobalSymbols(openbootSymtab))
        panic("could not load openboot symbols\n");

    if (!hypervisor->loadLocalSymbols(hypervisorSymtab))
        panic("could not load hypervisor symbols\n");

    // load symbols into debug table
    if (!reset->loadGlobalSymbols(debugSymbolTable))
        panic("could not load reset symbols\n");

    if (!openboot->loadGlobalSymbols(debugSymbolTable))
        panic("could not load openboot symbols\n");

    if (!hypervisor->loadLocalSymbols(debugSymbolTable))
        panic("could not load hypervisor symbols\n");


    // @todo any fixup code over writing data in binaries on setting break
    // events on functions should happen here.

}

SparcSystem::~SparcSystem()
{
    delete resetSymtab;
    delete hypervisorSymtab;
    delete openbootSymtab;
    delete reset;
    delete openboot;
    delete hypervisor;
}

bool
SparcSystem::breakpoint()
{
    panic("Need to implement");
}

void
SparcSystem::serialize(std::ostream &os)
{
    System::serialize(os);
    resetSymtab->serialize("reset_symtab", os);
    hypervisorSymtab->serialize("hypervisor_symtab", os);
    openbootSymtab->serialize("openboot_symtab", os);
}


void
SparcSystem::unserialize(Checkpoint *cp, const std::string &section)
{
    System::unserialize(cp,section);
    resetSymtab->unserialize("reset_symtab", cp, section);
    hypervisorSymtab->unserialize("hypervisor_symtab", cp, section);
    openbootSymtab->unserialize("openboot_symtab", cp, section);
}


BEGIN_DECLARE_SIM_OBJECT_PARAMS(SparcSystem)

    SimObjectParam<PhysicalMemory *> physmem;
    SimObjectParam<PhysicalMemory *> rom;
    SimpleEnumParam<System::MemoryMode> mem_mode;

    Param<Addr> reset_addr;
    Param<Addr> hypervisor_addr;
    Param<Addr> openboot_addr;

    Param<std::string> kernel;
    Param<std::string> reset_bin;
    Param<std::string> hypervisor_bin;
    Param<std::string> openboot_bin;

    Param<Tick> boot_cpu_frequency;
    Param<std::string> boot_osflags;
    Param<std::string> readfile;
    Param<unsigned int> init_param;

END_DECLARE_SIM_OBJECT_PARAMS(SparcSystem)

BEGIN_INIT_SIM_OBJECT_PARAMS(SparcSystem)

    INIT_PARAM(physmem, "phsyical memory"),
    INIT_PARAM(rom, "ROM for boot code"),
    INIT_ENUM_PARAM(mem_mode, "Memory Mode, (1=atomic, 2=timing)",
            System::MemoryModeStrings),

    INIT_PARAM(reset_addr, "Address that reset should be loaded at"),
    INIT_PARAM(hypervisor_addr, "Address that hypervisor should be loaded at"),
    INIT_PARAM(openboot_addr, "Address that openboot should be loaded at"),

    INIT_PARAM(kernel, "file that contains the kernel code"),
    INIT_PARAM(reset_bin, "file that contains the reset code"),
    INIT_PARAM(hypervisor_bin, "file that contains the hypervisor code"),
    INIT_PARAM(openboot_bin, "file that contains the openboot code"),
    INIT_PARAM(boot_cpu_frequency, "Frequency of the boot CPU"),
    INIT_PARAM_DFLT(boot_osflags, "flags to pass to the kernel during boot",
                    "a"),
    INIT_PARAM_DFLT(readfile, "file to read startup script from", ""),
    INIT_PARAM_DFLT(init_param, "numerical value to pass into simulator", 0)

END_INIT_SIM_OBJECT_PARAMS(SparcSystem)

CREATE_SIM_OBJECT(SparcSystem)
{
    SparcSystem::Params *p = new SparcSystem::Params;
    p->name = getInstanceName();
    p->boot_cpu_frequency = boot_cpu_frequency;
    p->physmem = physmem;
    p->rom = rom;
    p->mem_mode = mem_mode;
    p->kernel_path = kernel;
    p->reset_addr = reset_addr;
    p->hypervisor_addr = hypervisor_addr;
    p->openboot_addr = openboot_addr;
    p->reset_bin = reset_bin;
    p->hypervisor_bin = hypervisor_bin;
    p->openboot_bin = openboot_bin;
    p->boot_osflags = boot_osflags;
    p->init_param = init_param;
    p->readfile = readfile;
    return new SparcSystem(p);
}

REGISTER_SIM_OBJECT("SparcSystem", SparcSystem)


