/*
 * Copyright (c) 2002-2004 The Regents of The University of Michigan
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

#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "base/remote_gdb.hh"
#include "cpu/exec_context.hh"
#include "kern/kernel_stats.hh"
#include "mem/functional_mem/memory_control.hh"
#include "mem/functional_mem/physical_memory.hh"
#include "targetarch/vtophys.hh"
#include "sim/param.hh"
#include "sim/system.hh"
#include "base/trace.hh"

using namespace std;

vector<System *> System::systemList;

int System::numSystemsRunning = 0;

System::System(Params *p)
    : SimObject(p->name), memctrl(p->memctrl), physmem(p->physmem),
      init_param(p->init_param), params(p)
{
    // add self to global system list
    systemList.push_back(this);

    kernelSymtab = new SymbolTable;
    consoleSymtab = new SymbolTable;
    palSymtab = new SymbolTable;
    debugSymbolTable = new SymbolTable;

    /**
     * Load the kernel, pal, and console code into memory
     */
    // Load kernel code
    kernel = createObjectFile(params->kernel_path);
    if (kernel == NULL)
        fatal("Could not load kernel file %s", params->kernel_path);

    // Load Console Code
    console = createObjectFile(params->console_path);
    if (console == NULL)
        fatal("Could not load console file %s", params->console_path);

    // Load pal file
    pal = createObjectFile(params->palcode);
    if (pal == NULL)
        fatal("Could not load PALcode file %s", params->palcode);


    // Load program sections into memory
    pal->loadSections(physmem, true);
    console->loadSections(physmem, true);
    kernel->loadSections(physmem, true);

    // setup entry points
    kernelStart = kernel->textBase();
    kernelEnd = kernel->bssBase() + kernel->bssSize();
    kernelEntry = kernel->entryPoint();

    // load symbols
    if (!kernel->loadGlobalSymbols(kernelSymtab))
        panic("could not load kernel symbols\n");

    if (!kernel->loadLocalSymbols(kernelSymtab))
        panic("could not load kernel local symbols\n");

    if (!console->loadGlobalSymbols(consoleSymtab))
        panic("could not load console symbols\n");

    if (!pal->loadGlobalSymbols(palSymtab))
        panic("could not load pal symbols\n");

    if (!pal->loadLocalSymbols(palSymtab))
        panic("could not load pal symbols\n");

    if (!kernel->loadGlobalSymbols(debugSymbolTable))
        panic("could not load kernel symbols\n");

    if (!kernel->loadLocalSymbols(debugSymbolTable))
        panic("could not load kernel local symbols\n");

    if (!console->loadGlobalSymbols(debugSymbolTable))
        panic("could not load console symbols\n");

    if (!pal->loadGlobalSymbols(debugSymbolTable))
        panic("could not load pal symbols\n");

    if (!pal->loadLocalSymbols(debugSymbolTable))
        panic("could not load pal symbols\n");


    DPRINTF(Loader, "Kernel start = %#x\n", kernelStart);
    DPRINTF(Loader, "Kernel end   = %#x\n", kernelEnd);
    DPRINTF(Loader, "Kernel entry = %#x\n", kernelEntry);
    DPRINTF(Loader, "Kernel loaded...\n");

    Addr addr = 0;
#ifdef DEBUG
    consolePanicEvent = new BreakPCEvent(&pcEventQueue, "console panic");
    if (consoleSymtab->findAddress("panic", addr))
        consolePanicEvent->schedule(addr);
#endif

    /**
     * Copy the osflags (kernel arguments) into the consoles
     * memory. (Presently Linux does not use the console service
     * routine to get these command line arguments, but Tru64 and
     * others do.)
     */
    if (consoleSymtab->findAddress("env_booted_osflags", addr)) {
        Addr paddr = vtophys(physmem, addr);
        char *osflags = (char *)physmem->dma_addr(paddr, sizeof(uint32_t));

        if (osflags)
              strcpy(osflags, params->boot_osflags.c_str());
    }

    /**
     * Set the hardware reset parameter block system type and revision
     * information to Tsunami.
     */
    if (consoleSymtab->findAddress("xxm_rpb", addr)) {
        Addr paddr = vtophys(physmem, addr);
        char *hwrpb = (char *)physmem->dma_addr(paddr, sizeof(uint64_t));

        if (!hwrpb)
            panic("could not translate hwrpb addr\n");

        *(uint64_t*)(hwrpb+0x50) = htoa(params->system_type);
        *(uint64_t*)(hwrpb+0x58) = htoa(params->system_rev);
    } else
        panic("could not find hwrpb\n");

    // increment the number of running systms
    numSystemsRunning++;

    kernelBinning = new Kernel::Binning(this);
}

System::~System()
{
    delete kernelSymtab;
    delete consoleSymtab;
    delete kernel;
    delete console;
    delete pal;

    delete kernelBinning;

#ifdef DEBUG
    delete consolePanicEvent;
#endif
}

bool
System::breakpoint()
{
    return remoteGDB[0]->trap(ALPHA_KENTRY_INT);
}

int
System::registerExecContext(ExecContext *xc)
{
    int xcIndex = execContexts.size();
    execContexts.push_back(xc);

    RemoteGDB *rgdb = new RemoteGDB(this, xc);
    GDBListener *gdbl = new GDBListener(rgdb, 7000 + xcIndex);
    gdbl->listen();
    /**
     * Uncommenting this line waits for a remote debugger to connect
     * to the simulator before continuing.
     */
    //gdbl->accept();

    if (remoteGDB.size() <= xcIndex) {
        remoteGDB.resize(xcIndex+1);
    }

    remoteGDB[xcIndex] = rgdb;

    return xcIndex;
}

void
System::startup()
{
    if (!execContexts.empty()) {
        // activate with zero delay so that we start ticking right
        // away on cycle 0
        execContexts[0]->activate(0);
    }
}

void
System::replaceExecContext(ExecContext *xc, int xcIndex)
{
    if (xcIndex >= execContexts.size()) {
        panic("replaceExecContext: bad xcIndex, %d >= %d\n",
              xcIndex, execContexts.size());
    }

    execContexts[xcIndex] = xc;
    remoteGDB[xcIndex]->replaceExecContext(xc);
}

void
System::regStats()
{
    kernelBinning->regStats(name() + ".kern");
}

void
System::serialize(ostream &os)
{
    kernelBinning->serialize(os);
}


void
System::unserialize(Checkpoint *cp, const string &section)
{
    kernelBinning->unserialize(cp, section);
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

DEFINE_SIM_OBJECT_CLASS_NAME("System", System)

