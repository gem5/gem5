/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

#include "exec_context.hh"
#include "object_file.hh"
#include "memory_control.hh"
#include "physical_memory.hh"
#include "symtab.hh"
#include "remote_gdb.hh"
#include "vtophys.hh"
#include "system.hh"
#include "trace.hh"

using namespace std;

vector<System *> System::systemList;

int System::numSystemsRunning = 0;

System::System(const std::string _name,
               MemoryController *_memCtrl,
               PhysicalMemory *_physmem,
               const std::string &kernel_path,
               const std::string &console_path,
               const std::string &palcode,
               const std::string &boot_osflags)
    : SimObject(_name),
      kernel_panic_event(&pcEventQueue, "kernel panic"),
      console_panic_event(&pcEventQueue, "console panic"),
      badaddr_event(&pcEventQueue, "badaddr"),
      skip_power_state(&pcEventQueue, "tl_v48_capture_power_state"),
      skip_scavenge_boot(&pcEventQueue, "pmap_scavenge_boot"),
      printf_event(&pcEventQueue, "printf"),
      debug_printf_event(&pcEventQueue, "debug_printf", false),
      debug_printfr_event(&pcEventQueue, "debug_printfr", true),
      dump_mbuf_event(&pcEventQueue, "dump_mbuf"),
      memCtrl(_memCtrl),
      physmem(_physmem),
      remoteGDB(NULL),
      gdbListen(NULL)
{
    kernelSymtab = new SymbolTable;
    consoleSymtab = new SymbolTable;

    ObjectFile *kernel = createObjectFile(kernel_path);
    if (kernel == NULL)
        fatal("Could not load kernel file %s", kernel_path);

    ObjectFile *console = createObjectFile(console_path);
    if (console == NULL)
        fatal("Could not load console file %s", console_path);

    if (!kernel->loadGlobalSymbols(kernelSymtab))
        panic("could not load kernel symbols\n");

    if (!console->loadGlobalSymbols(consoleSymtab))
        panic("could not load console symbols\n");

    // Load pal file
    ObjectFile *pal = createObjectFile(palcode);
    if (pal == NULL)
        fatal("Could not load PALcode file %s", palcode);
    pal->loadSections(physmem, true);

    // copy of initial reg file contents
    initRegs = new RegFile;
    memset(initRegs, 0, sizeof(RegFile));

    // Load console file
    console->loadSections(physmem, true);

    // Load kernel file
    kernel->loadSections(physmem, true);
    kernelStart = kernel->textBase();
    kernelEnd = kernel->bssBase() + kernel->bssSize();
    kernelEntry = kernel->entryPoint();

    DPRINTF(Loader, "Kernel start = %#x\n"
            "Kernel end   = %#x\n"
            "Kernel entry = %#x\n",
            kernelStart, kernelEnd, kernelEntry);

    // Setup kernel boot parameters
    initRegs->pc = 0x4001;
    initRegs->npc = initRegs->pc + sizeof(MachInst);

    DPRINTF(Loader, "Kernel loaded...\n");

#ifdef FULL_SYSTEM
    Addr addr = 0;

    for(int i = 0; i < 12/*MAX_CPUS*/; i++)
        xc_array[i] = (ExecContext *) 0;

    num_cpus = 0;

    if (kernelSymtab->findAddress("enable_async_printf", addr)) {
        Addr paddr = vtophys(physmem, addr);
        uint8_t *enable_async_printf =
            physmem->dma_addr(paddr, sizeof(uint32_t));

        if (enable_async_printf)
            *(uint32_t *)enable_async_printf = 0;
    }

    if (consoleSymtab->findAddress("env_booted_osflags", addr)) {
        Addr paddr = vtophys(physmem, addr);
        char *osflags = (char *)physmem->dma_addr(paddr, sizeof(uint32_t));

        if (osflags)
            strcpy(osflags, boot_osflags.c_str());
    }

    if (kernelSymtab->findAddress("panic", addr))
        kernel_panic_event.schedule(addr);
    else
        panic("could not find kernel symbol \'panic\'");

    if (consoleSymtab->findAddress("panic", addr))
        console_panic_event.schedule(addr);

    if (kernelSymtab->findAddress("badaddr", addr))
        badaddr_event.schedule(addr);
    else
        panic("could not find kernel symbol \'badaddr\'");

    if (kernelSymtab->findAddress("tl_v48_capture_power_state", addr))
        skip_power_state.schedule(addr);

    if (kernelSymtab->findAddress("pmap_scavenge_boot", addr))
        skip_scavenge_boot.schedule(addr);

#if TRACING_ON
    if (kernelSymtab->findAddress("printf", addr))
        printf_event.schedule(addr);

    if (kernelSymtab->findAddress("m5printf", addr))
        debug_printf_event.schedule(addr);

    if (kernelSymtab->findAddress("m5printfr", addr))
        debug_printfr_event.schedule(addr);

    if (kernelSymtab->findAddress("m5_dump_mbuf", addr))
        dump_mbuf_event.schedule(addr);
#endif

#endif

    // add self to global system list
    systemList.push_back(this);

    numSystemsRunning++;
}


System::~System()
{
    delete kernelSymtab;
    delete consoleSymtab;
    delete initRegs;
}


void
System::initBootContext(ExecContext *xc)
{
    xc->regs = *initRegs;

    remoteGDB = new RemoteGDB(this, xc);
    gdbListen = new GDBListener(remoteGDB, 7000);
    gdbListen->listen();

    // Reset the system
    //
    TheISA::init(physmem, &xc->regs);
}


void
System::registerExecContext(ExecContext *xc)
{
    if (num_cpus == 12/*MAX_CPUS*/)
        panic("Too many CPU's\n");
    xc_array[xc->cpu_id] = xc;
    num_cpus++;
}


void
System::printSystems()
{
    vector<System *>::iterator i = systemList.begin();
    vector<System *>::iterator end = systemList.end();
    for (; i != end; ++i) {
        System *sys = *i;
        cerr << "System " << sys->name() << ": " << hex << sys << endl;
    }
}


extern "C"
void
printSystems()
{
    System::printSystems();
}


BEGIN_DECLARE_SIM_OBJECT_PARAMS(System)

    SimObjectParam<MemoryController *> mem_ctl;
    SimObjectParam<PhysicalMemory *> physmem;

    Param<string> kernel_code;
    Param<string> console_code;
    Param<string> pal_code;
    Param<string> boot_osflags;

END_DECLARE_SIM_OBJECT_PARAMS(System)

BEGIN_INIT_SIM_OBJECT_PARAMS(System)

    INIT_PARAM(mem_ctl, "memory controller"),
    INIT_PARAM(physmem, "phsyical memory"),
    INIT_PARAM(kernel_code, "file that contains the kernel code"),
    INIT_PARAM(console_code, "file that contains the console code"),
    INIT_PARAM(pal_code, "file that contains palcode"),
    INIT_PARAM_DFLT(boot_osflags, "flags to pass to the kernel during boot",
                    "a")

END_INIT_SIM_OBJECT_PARAMS(System)


CREATE_SIM_OBJECT(System)
{
    System *sys = new System(getInstanceName(), mem_ctl, physmem,
                             kernel_code, console_code, pal_code,
                             boot_osflags);

    return sys;
}

REGISTER_SIM_OBJECT("System", System)
