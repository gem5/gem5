/*
 * Copyright (c) 2011 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2003-2006 The Regents of The University of Michigan
 * Copyright (c) 2011 Regents of the University of California
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
 * Authors: Steve Reinhardt
 *          Lisa Hsu
 *          Nathan Binkert
 *          Ali Saidi
 *          Rick Strong
 */

#include "arch/isa_traits.hh"
#include "arch/remote_gdb.hh"
#include "arch/utility.hh"
#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "base/trace.hh"
#include "config/full_system.hh"
#include "config/the_isa.hh"
#include "cpu/thread_context.hh"
#include "debug/Loader.hh"
#include "debug/WorkItems.hh"
#include "mem/mem_object.hh"
#include "mem/physical.hh"
#include "sim/byteswap.hh"
#include "sim/debug.hh"
#include "sim/system.hh"

#if FULL_SYSTEM
#include "arch/vtophys.hh"
#include "kern/kernel_stats.hh"
#include "mem/vport.hh"
#else
#include "params/System.hh"
#endif

using namespace std;
using namespace TheISA;

vector<System *> System::systemList;

int System::numSystemsRunning = 0;

System::System(Params *p)
    : SimObject(p), physmem(p->physmem), _numContexts(0),
#if FULL_SYSTEM
      init_param(p->init_param),
      loadAddrMask(p->load_addr_mask),
#else
      pagePtr(0),
      nextPID(0),
#endif
      memoryMode(p->mem_mode),
      workItemsBegin(0),
      workItemsEnd(0),
      numWorkIds(p->num_work_ids),
      _params(p),
      totalNumInsts(0),
      instEventQueue("system instruction-based event queue")
{
    // add self to global system list
    systemList.push_back(this);

    /** Keep track of all memories we can execute code out of
     * in our system
     */
    for (int x = 0; x < p->memories.size(); x++) {
        if (!p->memories[x])
            continue;
        memRanges.push_back(RangeSize(p->memories[x]->start(),
                                      p->memories[x]->size()));
    }

#if FULL_SYSTEM
    kernelSymtab = new SymbolTable;
    if (!debugSymbolTable)
        debugSymbolTable = new SymbolTable;


    /**
     * Get a functional port to memory
     */
    Port *mem_port;
    functionalPort = new FunctionalPort(name() + "-fport");
    mem_port = physmem->getPort("functional");
    functionalPort->setPeer(mem_port);
    mem_port->setPeer(functionalPort);

    virtPort = new VirtualPort(name() + "-fport");
    mem_port = physmem->getPort("functional");
    virtPort->setPeer(mem_port);
    mem_port->setPeer(virtPort);


    /**
     * Load the kernel code into memory
     */
    if (params()->kernel == "") {
        inform("No kernel set for full system simulation. Assuming you know what"
                " you're doing...\n");
    } else {
        // Load kernel code
        kernel = createObjectFile(params()->kernel);
        inform("kernel located at: %s", params()->kernel);

        if (kernel == NULL)
            fatal("Could not load kernel file %s", params()->kernel);

        // Load program sections into memory
        kernel->loadSections(functionalPort, loadAddrMask);

        // setup entry points
        kernelStart = kernel->textBase();
        kernelEnd = kernel->bssBase() + kernel->bssSize();
        kernelEntry = kernel->entryPoint();

        // load symbols
        if (!kernel->loadGlobalSymbols(kernelSymtab))
            fatal("could not load kernel symbols\n");

        if (!kernel->loadLocalSymbols(kernelSymtab))
            fatal("could not load kernel local symbols\n");

        if (!kernel->loadGlobalSymbols(debugSymbolTable))
            fatal("could not load kernel symbols\n");

        if (!kernel->loadLocalSymbols(debugSymbolTable))
            fatal("could not load kernel local symbols\n");

        DPRINTF(Loader, "Kernel start = %#x\n", kernelStart);
        DPRINTF(Loader, "Kernel end   = %#x\n", kernelEnd);
        DPRINTF(Loader, "Kernel entry = %#x\n", kernelEntry);
        DPRINTF(Loader, "Kernel loaded...\n");
    }
#endif // FULL_SYSTEM

    // increment the number of running systms
    numSystemsRunning++;

    activeCpus.clear();
}

System::~System()
{
#if FULL_SYSTEM
    delete kernelSymtab;
    delete kernel;
#else
    panic("System::fixFuncEventAddr needs to be rewritten "
          "to work with syscall emulation");
#endif // FULL_SYSTEM}

    for (uint32_t j = 0; j < numWorkIds; j++)
        delete workItemStats[j];
}

void
System::setMemoryMode(Enums::MemoryMode mode)
{
    assert(getState() == Drained);
    memoryMode = mode;
}

bool System::breakpoint()
{
    if (remoteGDB.size())
        return remoteGDB[0]->breakpoint();
    return false;
}

/**
 * Setting rgdb_wait to a positive integer waits for a remote debugger to
 * connect to that context ID before continuing.  This should really
   be a parameter on the CPU object or something...
 */
int rgdb_wait = -1;

int
System::registerThreadContext(ThreadContext *tc, int assigned)
{
    int id;
    if (assigned == -1) {
        for (id = 0; id < threadContexts.size(); id++) {
            if (!threadContexts[id])
                break;
        }

        if (threadContexts.size() <= id)
            threadContexts.resize(id + 1);
    } else {
        if (threadContexts.size() <= assigned)
            threadContexts.resize(assigned + 1);
        id = assigned;
    }

    if (threadContexts[id])
        fatal("Cannot have two CPUs with the same id (%d)\n", id);

    threadContexts[id] = tc;
    _numContexts++;

    int port = getRemoteGDBPort();
    if (port) {
        RemoteGDB *rgdb = new RemoteGDB(this, tc);
        GDBListener *gdbl = new GDBListener(rgdb, port + id);
        gdbl->listen();

        if (rgdb_wait != -1 && rgdb_wait == id)
            gdbl->accept();

        if (remoteGDB.size() <= id) {
            remoteGDB.resize(id + 1);
        }

        remoteGDB[id] = rgdb;
    }

    activeCpus.push_back(false);

    return id;
}

int
System::numRunningContexts()
{
    int running = 0;
    for (int i = 0; i < _numContexts; ++i) {
        if (threadContexts[i]->status() != ThreadContext::Halted)
            ++running;
    }
    return running;
}

void
System::initState()
{
#if FULL_SYSTEM
    int i;
    for (i = 0; i < threadContexts.size(); i++)
        TheISA::startupCPU(threadContexts[i], i);
#endif
}

void
System::replaceThreadContext(ThreadContext *tc, int context_id)
{
    if (context_id >= threadContexts.size()) {
        panic("replaceThreadContext: bad id, %d >= %d\n",
              context_id, threadContexts.size());
    }

    threadContexts[context_id] = tc;
    if (context_id < remoteGDB.size())
        remoteGDB[context_id]->replaceThreadContext(tc);
}

#if !FULL_SYSTEM
Addr
System::allocPhysPages(int npages)
{
    Addr return_addr = pagePtr << LogVMPageSize;
    pagePtr += npages;
    if (return_addr >= physmem->size())
        fatal("Out of memory, please increase size of physical memory.");
    return return_addr;
}

Addr
System::memSize()
{
    return physmem->size();
}

Addr
System::freeMemSize()
{
   return physmem->size() - (pagePtr << LogVMPageSize);
}

#endif

bool
System::isMemory(const Addr addr) const
{
    std::list<Range<Addr> >::const_iterator i;
    for (i = memRanges.begin(); i != memRanges.end(); i++) {
        if (*i == addr)
            return true;
    }
    return false;
}

void
System::resume()
{
    SimObject::resume();
    totalNumInsts = 0;
}

void
System::serialize(ostream &os)
{
#if FULL_SYSTEM
    kernelSymtab->serialize("kernel_symtab", os);
#else // !FULL_SYSTEM
    SERIALIZE_SCALAR(pagePtr);
    SERIALIZE_SCALAR(nextPID);
#endif
}


void
System::unserialize(Checkpoint *cp, const string &section)
{
#if FULL_SYSTEM
    kernelSymtab->unserialize("kernel_symtab", cp, section);
#else // !FULL_SYSTEM
    UNSERIALIZE_SCALAR(pagePtr);
    UNSERIALIZE_SCALAR(nextPID);
#endif
}

void
System::regStats()
{
    for (uint32_t j = 0; j < numWorkIds ; j++) {
        workItemStats[j] = new Stats::Histogram();
        stringstream namestr;
        ccprintf(namestr, "work_item_type%d", j);
        workItemStats[j]->init(20)
                         .name(name() + "." + namestr.str())
                         .desc("Run time stat for" + namestr.str())
                         .prereq(*workItemStats[j]);
    }
}

void
System::workItemEnd(uint32_t tid, uint32_t workid)
{
    std::pair<uint32_t,uint32_t> p(tid, workid);
    if (!lastWorkItemStarted.count(p))
        return;

    Tick samp = curTick() - lastWorkItemStarted[p];
    DPRINTF(WorkItems, "Work item end: %d\t%d\t%lld\n", tid, workid, samp);

    if (workid >= numWorkIds)
        fatal("Got workid greater than specified in system configuration\n");

    workItemStats[workid]->sample(samp);
    lastWorkItemStarted.erase(p);
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

void
printSystems()
{
    System::printSystems();
}

const char *System::MemoryModeStrings[3] = {"invalid", "atomic",
    "timing"};

#if !FULL_SYSTEM

System *
SystemParams::create()
{
    return new System(this);
}

#endif
