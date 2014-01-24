/*
 * Copyright (c) 2011-2013 ARM Limited
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
#include "base/str.hh"
#include "base/trace.hh"
#include "config/the_isa.hh"
#include "cpu/thread_context.hh"
#include "debug/Loader.hh"
#include "debug/WorkItems.hh"
#include "kern/kernel_stats.hh"
#include "mem/abstract_mem.hh"
#include "mem/physical.hh"
#include "params/System.hh"
#include "sim/byteswap.hh"
#include "sim/debug.hh"
#include "sim/full_system.hh"
#include "sim/system.hh"

using namespace std;
using namespace TheISA;

vector<System *> System::systemList;

int System::numSystemsRunning = 0;

System::System(Params *p)
    : MemObject(p), _systemPort("system_port", this),
      _numContexts(0),
      pagePtr(0),
      init_param(p->init_param),
      physProxy(_systemPort, p->cache_line_size),
      loadAddrMask(p->load_addr_mask),
      loadAddrOffset(p->load_offset),
      nextPID(0),
      physmem(name() + ".physmem", p->memories),
      memoryMode(p->mem_mode),
      _cacheLineSize(p->cache_line_size),
      workItemsBegin(0),
      workItemsEnd(0),
      numWorkIds(p->num_work_ids),
      _params(p),
      totalNumInsts(0),
      instEventQueue("system instruction-based event queue")
{
    // add self to global system list
    systemList.push_back(this);

    if (FullSystem) {
        kernelSymtab = new SymbolTable;
        if (!debugSymbolTable)
            debugSymbolTable = new SymbolTable;
    }

    // check if the cache line size is a value known to work
    if (!(_cacheLineSize == 16 || _cacheLineSize == 32 ||
          _cacheLineSize == 64 || _cacheLineSize == 128))
        warn_once("Cache line size is neither 16, 32, 64 nor 128 bytes.\n");

    // Get the generic system master IDs
    MasterID tmp_id M5_VAR_USED;
    tmp_id = getMasterId("writebacks");
    assert(tmp_id == Request::wbMasterId);
    tmp_id = getMasterId("functional");
    assert(tmp_id == Request::funcMasterId);
    tmp_id = getMasterId("interrupt");
    assert(tmp_id == Request::intMasterId);

    if (FullSystem) {
        if (params()->kernel == "") {
            inform("No kernel set for full system simulation. "
                   "Assuming you know what you're doing\n");

            kernel = NULL;
        } else {
            // Get the kernel code
            kernel = createObjectFile(params()->kernel);
            inform("kernel located at: %s", params()->kernel);

            if (kernel == NULL)
                fatal("Could not load kernel file %s", params()->kernel);

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

            // Loading only needs to happen once and after memory system is
            // connected so it will happen in initState()
        }
    }

    // increment the number of running systms
    numSystemsRunning++;

    // Set back pointers to the system in all memories
    for (int x = 0; x < params()->memories.size(); x++)
        params()->memories[x]->system(this);
}

System::~System()
{
    delete kernelSymtab;
    delete kernel;

    for (uint32_t j = 0; j < numWorkIds; j++)
        delete workItemStats[j];
}

void
System::init()
{
    // check that the system port is connected
    if (!_systemPort.isConnected())
        panic("System port on %s is not connected.\n", name());
}

BaseMasterPort&
System::getMasterPort(const std::string &if_name, PortID idx)
{
    // no need to distinguish at the moment (besides checking)
    return _systemPort;
}

void
System::setMemoryMode(Enums::MemoryMode mode)
{
    assert(getDrainState() == Drainable::Drained);
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

#if THE_ISA != NULL_ISA
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
#endif

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
    if (FullSystem) {
        for (int i = 0; i < threadContexts.size(); i++)
            TheISA::startupCPU(threadContexts[i], i);
        // Moved from the constructor to here since it relies on the
        // address map being resolved in the interconnect
        /**
         * Load the kernel code into memory
         */
        if (params()->kernel != "")  {
            // Validate kernel mapping before loading binary
            if (!(isMemAddr((kernelStart & loadAddrMask) + loadAddrOffset) &&
                     isMemAddr((kernelEnd & loadAddrMask) + loadAddrOffset))) {
                fatal("Kernel is mapped to invalid location (not memory). "
                      "kernelStart 0x(%x) - kernelEnd 0x(%x) %#x:%#x\n", kernelStart,
                      kernelEnd, (kernelStart & loadAddrMask) + loadAddrOffset,
                      (kernelEnd & loadAddrMask) + loadAddrOffset);
            }
            // Load program sections into memory
            kernel->loadSections(physProxy, loadAddrMask, loadAddrOffset);

            DPRINTF(Loader, "Kernel start = %#x\n", kernelStart);
            DPRINTF(Loader, "Kernel end   = %#x\n", kernelEnd);
            DPRINTF(Loader, "Kernel entry = %#x\n", kernelEntry);
            DPRINTF(Loader, "Kernel loaded...\n");
        }
    }

    activeCpus.clear();
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

Addr
System::allocPhysPages(int npages)
{
    Addr return_addr = pagePtr << LogVMPageSize;
    pagePtr += npages;
    if ((pagePtr << LogVMPageSize) > physmem.totalSize())
        fatal("Out of memory, please increase size of physical memory.");
    return return_addr;
}

Addr
System::memSize() const
{
    return physmem.totalSize();
}

Addr
System::freeMemSize() const
{
   return physmem.totalSize() - (pagePtr << LogVMPageSize);
}

bool
System::isMemAddr(Addr addr) const
{
    return physmem.isMemAddr(addr);
}

unsigned int
System::drain(DrainManager *dm)
{
    setDrainState(Drainable::Drained);
    return 0;
}

void
System::drainResume()
{
    Drainable::drainResume();
    totalNumInsts = 0;
}

void
System::serialize(ostream &os)
{
    if (FullSystem)
        kernelSymtab->serialize("kernel_symtab", os);
    SERIALIZE_SCALAR(pagePtr);
    SERIALIZE_SCALAR(nextPID);
    serializeSymtab(os);

    // also serialize the memories in the system
    nameOut(os, csprintf("%s.physmem", name()));
    physmem.serialize(os);
}


void
System::unserialize(Checkpoint *cp, const string &section)
{
    if (FullSystem)
        kernelSymtab->unserialize("kernel_symtab", cp, section);
    UNSERIALIZE_SCALAR(pagePtr);
    UNSERIALIZE_SCALAR(nextPID);
    unserializeSymtab(cp, section);

    // also unserialize the memories in the system
    physmem.unserialize(cp, csprintf("%s.physmem", name()));
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

MasterID
System::getMasterId(std::string master_name)
{
    // strip off system name if the string starts with it
    if (startswith(master_name, name()))
        master_name = master_name.erase(0, name().size() + 1);

    // CPUs in switch_cpus ask for ids again after switching
    for (int i = 0; i < masterIds.size(); i++) {
        if (masterIds[i] == master_name) {
            return i;
        }
    }

    // Verify that the statistics haven't been enabled yet
    // Otherwise objects will have sized their stat buckets and
    // they will be too small

    if (Stats::enabled())
        fatal("Can't request a masterId after regStats(). \
                You must do so in init().\n");

    masterIds.push_back(master_name);

    return masterIds.size() - 1;
}

std::string
System::getMasterName(MasterID master_id)
{
    if (master_id >= masterIds.size())
        fatal("Invalid master_id passed to getMasterName()\n");

    return masterIds[master_id];
}

const char *System::MemoryModeStrings[4] = {"invalid", "atomic", "timing",
                                            "atomic_noncaching"};

System *
SystemParams::create()
{
    return new System(this);
}
