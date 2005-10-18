/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
#include "mem/functional/memory_control.hh"
#include "mem/functional/physical.hh"
#include "targetarch/vtophys.hh"
#include "sim/builder.hh"
#include "sim/system.hh"
#include "base/trace.hh"

using namespace std;

vector<System *> System::systemList;

int System::numSystemsRunning = 0;

System::System(Params *p)
    : SimObject(p->name), memctrl(p->memctrl), physmem(p->physmem),
      init_param(p->init_param), numcpus(0), params(p)
{
    // add self to global system list
    systemList.push_back(this);

    kernelSymtab = new SymbolTable;
    consoleSymtab = new SymbolTable;
    palSymtab = new SymbolTable;
    allSymtab = new SymbolTable;
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

    if (!kernel->loadGlobalSymbols(allSymtab))
        panic("could not load kernel symbols\n");

    if (!kernel->loadLocalSymbols(allSymtab))
        panic("could not load kernel local symbols\n");

    if (!console->loadGlobalSymbols(allSymtab))
        panic("could not load console symbols\n");

    if (!pal->loadGlobalSymbols(allSymtab))
        panic("could not load pal symbols\n");

    if (!pal->loadLocalSymbols(allSymtab))
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
    consolePanicEvent = addConsoleFuncEvent<BreakPCEvent>("panic");
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
    if (consoleSymtab->findAddress("m5_rpb", addr)) {
        Addr paddr = vtophys(physmem, addr);
        char *hwrpb = (char *)physmem->dma_addr(paddr, sizeof(uint64_t));

        if (!hwrpb)
            panic("could not translate hwrpb addr\n");

        *(uint64_t*)(hwrpb+0x50) = htog(params->system_type);
        *(uint64_t*)(hwrpb+0x58) = htog(params->system_rev);
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


/**
 * This function fixes up addresses that are used to match PCs for
 * hooking simulator events on to target function executions.
 *
 * Alpha binaries may have multiple global offset table (GOT)
 * sections.  A function that uses the GOT starts with a
 * two-instruction prolog which sets the global pointer (gp == r29) to
 * the appropriate GOT section.  The proper gp value is calculated
 * based on the function address, which must be passed by the caller
 * in the procedure value register (pv aka t12 == r27).  This sequence
 * looks like the following:
 *
 *			opcode Ra Rb offset
 *	ldah gp,X(pv)     09   29 27   X
 *	lda  gp,Y(gp)     08   29 29   Y
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
System::fixFuncEventAddr(Addr addr)
{
    // mask for just the opcode, Ra, and Rb fields (not the offset)
    const uint32_t inst_mask = 0xffff0000;
    // ldah gp,X(pv): opcode 9, Ra = 29, Rb = 27
    const uint32_t gp_ldah_pattern = (9 << 26) | (29 << 21) | (27 << 16);
    // lda  gp,Y(gp): opcode 8, Ra = 29, rb = 29
    const uint32_t gp_lda_pattern  = (8 << 26) | (29 << 21) | (29 << 16);
    // instruction size
    const int sz = sizeof(uint32_t);

    Addr paddr = vtophys(physmem, addr);
    uint32_t i1 = *(uint32_t *)physmem->dma_addr(paddr, sz);
    uint32_t i2 = *(uint32_t *)physmem->dma_addr(paddr+sz, sz);

    if ((i1 & inst_mask) == gp_ldah_pattern &&
        (i2 & inst_mask) == gp_lda_pattern) {
        Addr new_addr = addr + 2*sz;
        DPRINTF(Loader, "fixFuncEventAddr: %p -> %p", addr, new_addr);
        return new_addr;
    } else {
        return addr;
    }
}


void
System::setAlphaAccess(Addr access)
{
    Addr addr = 0;
    if (consoleSymtab->findAddress("m5AlphaAccess", addr)) {
        Addr paddr = vtophys(physmem, addr);
        uint64_t *m5AlphaAccess =
            (uint64_t *)physmem->dma_addr(paddr, sizeof(uint64_t));

        if (!m5AlphaAccess)
            panic("could not translate m5AlphaAccess addr\n");

        *m5AlphaAccess = htog(EV5::Phys2K0Seg(access));
    } else
        panic("could not find m5AlphaAccess\n");
}


bool
System::breakpoint()
{
    return remoteGDB[0]->trap(ALPHA_KENTRY_INT);
}

int rgdb_wait = -1;

int
System::registerExecContext(ExecContext *xc, int id)
{
    if (id == -1) {
        for (id = 0; id < execContexts.size(); id++) {
            if (!execContexts[id])
                break;
        }
    }

    if (execContexts.size() <= id)
        execContexts.resize(id + 1);

    if (execContexts[id])
        panic("Cannot have two CPUs with the same id (%d)\n", id);

    execContexts[id] = xc;
    numcpus++;

    RemoteGDB *rgdb = new RemoteGDB(this, xc);
    GDBListener *gdbl = new GDBListener(rgdb, 7000 + id);
    gdbl->listen();
    /**
     * Uncommenting this line waits for a remote debugger to connect
     * to the simulator before continuing.
     */
    if (rgdb_wait != -1 && rgdb_wait == id)
        gdbl->accept();

    if (remoteGDB.size() <= id) {
        remoteGDB.resize(id + 1);
    }

    remoteGDB[id] = rgdb;

    return id;
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
System::replaceExecContext(ExecContext *xc, int id)
{
    if (id >= execContexts.size()) {
        panic("replaceExecContext: bad id, %d >= %d\n",
              id, execContexts.size());
    }

    execContexts[id] = xc;
    remoteGDB[id]->replaceExecContext(xc);
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

BEGIN_DECLARE_SIM_OBJECT_PARAMS(System)

    Param<Tick> boot_cpu_frequency;
    SimObjectParam<MemoryController *> memctrl;
    SimObjectParam<PhysicalMemory *> physmem;

    Param<string> kernel;
    Param<string> console;
    Param<string> pal;

    Param<string> boot_osflags;
    Param<string> readfile;
    Param<unsigned int> init_param;

    Param<uint64_t> system_type;
    Param<uint64_t> system_rev;

    Param<bool> bin;
    VectorParam<string> binned_fns;
    Param<bool> bin_int;

END_DECLARE_SIM_OBJECT_PARAMS(System)

BEGIN_INIT_SIM_OBJECT_PARAMS(System)

    INIT_PARAM(boot_cpu_frequency, "Frequency of the boot CPU"),
    INIT_PARAM(memctrl, "memory controller"),
    INIT_PARAM(physmem, "phsyical memory"),
    INIT_PARAM(kernel, "file that contains the kernel code"),
    INIT_PARAM(console, "file that contains the console code"),
    INIT_PARAM(pal, "file that contains palcode"),
    INIT_PARAM_DFLT(boot_osflags, "flags to pass to the kernel during boot",
                    "a"),
    INIT_PARAM_DFLT(readfile, "file to read startup script from", ""),
    INIT_PARAM_DFLT(init_param, "numerical value to pass into simulator", 0),
    INIT_PARAM_DFLT(system_type, "Type of system we are emulating", 34),
    INIT_PARAM_DFLT(system_rev, "Revision of system we are emulating", 1<<10),
    INIT_PARAM_DFLT(bin, "is this system to be binned", false),
    INIT_PARAM(binned_fns, "functions to be broken down and binned"),
    INIT_PARAM_DFLT(bin_int, "is interrupt code binned seperately?", true)

END_INIT_SIM_OBJECT_PARAMS(System)

CREATE_SIM_OBJECT(System)
{
    System::Params *p = new System::Params;
    p->name = getInstanceName();
    p->boot_cpu_frequency = boot_cpu_frequency;
    p->memctrl = memctrl;
    p->physmem = physmem;
    p->kernel_path = kernel;
    p->console_path = console;
    p->palcode = pal;
    p->boot_osflags = boot_osflags;
    p->init_param = init_param;
    p->readfile = readfile;
    p->system_type = system_type;
    p->system_rev = system_rev;
    p->bin = bin;
    p->binned_fns = binned_fns;
    p->bin_int = bin_int;
    return new System(p);
}

REGISTER_SIM_OBJECT("System", System)

