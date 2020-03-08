/*
 * Copyright (c) 2011-2014,2017-2019 ARM Limited
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
 */

#include "sim/system.hh"

#include <algorithm>

#include "arch/remote_gdb.hh"
#include "arch/utility.hh"
#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "base/str.hh"
#include "base/trace.hh"
#include "config/use_kvm.hh"
#if USE_KVM
#include "cpu/kvm/base.hh"
#include "cpu/kvm/vm.hh"
#endif
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/Loader.hh"
#include "debug/Quiesce.hh"
#include "debug/WorkItems.hh"
#include "mem/abstract_mem.hh"
#include "mem/physical.hh"
#include "params/System.hh"
#include "sim/byteswap.hh"
#include "sim/debug.hh"
#include "sim/full_system.hh"
#include "sim/redirect_path.hh"

using namespace std;
using namespace TheISA;

vector<System *> System::systemList;

void
System::Threads::Thread::resume()
{
#   if THE_ISA != NULL_ISA
    DPRINTFS(Quiesce, context->getCpuPtr(), "activating\n");
    context->activate();
#   endif
}

std::string
System::Threads::Thread::name() const
{
    assert(context);
    return csprintf("%s.threads[%d]", context->getSystemPtr()->name(),
            context->contextId());
}

void
System::Threads::Thread::quiesce() const
{
    context->suspend();
    auto *workload = context->getSystemPtr()->workload;
    if (workload)
        workload->recordQuiesce();
}

ContextID
System::Threads::insert(ThreadContext *tc, ContextID id)
{
    if (id == InvalidContextID) {
        for (id = 0; id < size(); id++) {
            if (!threads[id].context)
                break;
        }
    }

    if (id >= size())
        threads.resize(id + 1);

    fatal_if(threads[id].context,
            "Cannot have two thread contexts with the same id (%d).", id);

    auto *sys = tc->getSystemPtr();

    auto &t = thread(id);
    t.context = tc;
    // Look up this thread again on resume, in case the threads vector has
    // been reallocated.
    t.resumeEvent = new EventFunctionWrapper(
            [this, id](){ thread(id).resume(); }, sys->name());
#   if THE_ISA != NULL_ISA
    int port = getRemoteGDBPort();
    if (port) {
        t.gdb = new RemoteGDB(sys, tc, port + id);
        t.gdb->listen();
    }
#   endif

    return id;
}

void
System::Threads::replace(ThreadContext *tc, ContextID id)
{
    auto &t = thread(id);
    panic_if(!t.context, "Can't replace a context which doesn't exist.");
    if (t.gdb)
        t.gdb->replaceThreadContext(tc);
#   if THE_ISA != NULL_ISA
    if (t.resumeEvent->scheduled()) {
        Tick when = t.resumeEvent->when();
        t.context->getCpuPtr()->deschedule(t.resumeEvent);
        tc->getCpuPtr()->schedule(t.resumeEvent, when);
    }
#   endif
    t.context = tc;
}

ThreadContext *
System::Threads::findFree()
{
    for (auto &thread: threads) {
        if (thread.context->status() == ThreadContext::Halted)
            return thread.context;
    }
    return nullptr;
}

int
System::Threads::numRunning() const
{
    int count = 0;
    for (auto &thread: threads) {
        auto status = thread.context->status();
        if (status != ThreadContext::Halted &&
                status != ThreadContext::Halting) {
            count++;
        }
    }
    return count;
}

void
System::Threads::quiesce(ContextID id)
{
    auto &t = thread(id);
#   if THE_ISA != NULL_ISA
    BaseCPU M5_VAR_USED *cpu = t.context->getCpuPtr();
    DPRINTFS(Quiesce, cpu, "quiesce()\n");
#   endif
    t.quiesce();
}

void
System::Threads::quiesceTick(ContextID id, Tick when)
{
#   if THE_ISA != NULL_ISA
    auto &t = thread(id);
    BaseCPU *cpu = t.context->getCpuPtr();

    DPRINTFS(Quiesce, cpu, "quiesceTick until %u\n", when);
    t.quiesce();

    cpu->reschedule(t.resumeEvent, when, true);
#   endif
}

int System::numSystemsRunning = 0;

System::System(Params *p)
    : SimObject(p), _systemPort("system_port", this),
      multiThread(p->multi_thread),
      pagePtr(0),
      init_param(p->init_param),
      physProxy(_systemPort, p->cache_line_size),
      workload(p->workload),
#if USE_KVM
      kvmVM(p->kvm_vm),
#else
      kvmVM(nullptr),
#endif
      physmem(name() + ".physmem", p->memories, p->mmap_using_noreserve,
              p->shared_backstore),
      memoryMode(p->mem_mode),
      _cacheLineSize(p->cache_line_size),
      workItemsBegin(0),
      workItemsEnd(0),
      numWorkIds(p->num_work_ids),
      thermalModel(p->thermal_model),
      _params(p),
      _m5opRange(p->m5ops_base ?
                 RangeSize(p->m5ops_base, 0x10000) :
                 AddrRange(1, 0)), // Create an empty range if disabled
      totalNumInsts(0),
      redirectPaths(p->redirect_paths)
{
    if (workload)
        workload->system = this;

    // add self to global system list
    systemList.push_back(this);

#if USE_KVM
    if (kvmVM) {
        kvmVM->setSystem(this);
    }
#endif

    // check if the cache line size is a value known to work
    if (!(_cacheLineSize == 16 || _cacheLineSize == 32 ||
          _cacheLineSize == 64 || _cacheLineSize == 128))
        warn_once("Cache line size is neither 16, 32, 64 nor 128 bytes.\n");

    // Get the generic system master IDs
    MasterID tmp_id M5_VAR_USED;
    tmp_id = getMasterId(this, "writebacks");
    assert(tmp_id == Request::wbMasterId);
    tmp_id = getMasterId(this, "functional");
    assert(tmp_id == Request::funcMasterId);
    tmp_id = getMasterId(this, "interrupt");
    assert(tmp_id == Request::intMasterId);

    // increment the number of running systems
    numSystemsRunning++;

    // Set back pointers to the system in all memories
    for (int x = 0; x < params()->memories.size(); x++)
        params()->memories[x]->system(this);
}

System::~System()
{
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

void
System::startup()
{
    SimObject::startup();

    // Now that we're about to start simulation, wait for GDB connections if
    // requested.
#if THE_ISA != NULL_ISA
    for (int i = 0; i < threads.size(); i++) {
        auto *gdb = threads.thread(i).gdb;
        auto *cpu = threads[i]->getCpuPtr();
        if (gdb && cpu->waitForRemoteGDB()) {
            inform("%s: Waiting for a remote GDB connection on port %d.",
                   cpu->name(), gdb->port());
            gdb->connect();
        }
    }
#endif
}

Port &
System::getPort(const std::string &if_name, PortID idx)
{
    // no need to distinguish at the moment (besides checking)
    return _systemPort;
}

void
System::setMemoryMode(Enums::MemoryMode mode)
{
    assert(drainState() == DrainState::Drained);
    memoryMode = mode;
}

bool System::breakpoint()
{
    if (!threads.size())
        return false;
    auto *gdb = threads.thread(0).gdb;
    if (!gdb)
        return false;
    return gdb->breakpoint();
}

ContextID
System::registerThreadContext(ThreadContext *tc, ContextID assigned)
{
    ContextID id = threads.insert(tc, assigned);

    for (auto *e: liveEvents)
        tc->schedule(e);

    return id;
}

bool
System::schedule(PCEvent *event)
{
    bool all = true;
    liveEvents.push_back(event);
    for (auto *tc: threads)
        all = tc->schedule(event) && all;
    return all;
}

bool
System::remove(PCEvent *event)
{
    bool all = true;
    liveEvents.remove(event);
    for (auto *tc: threads)
        all = tc->remove(event) && all;
    return all;
}

void
System::replaceThreadContext(ThreadContext *tc, ContextID context_id)
{
    auto *otc = threads[context_id];
    threads.replace(tc, context_id);

    for (auto *e: liveEvents) {
        otc->remove(e);
        tc->schedule(e);
    }
}

bool
System::validKvmEnvironment() const
{
#if USE_KVM
    if (threads.empty())
        return false;

    for (auto *tc: threads) {
        if (!dynamic_cast<BaseKvmCPU *>(tc->getCpuPtr()))
            return false;
    }

    return true;
#else
    return false;
#endif
}

Addr
System::allocPhysPages(int npages)
{
    Addr return_addr = pagePtr << PageShift;
    pagePtr += npages;

    Addr next_return_addr = pagePtr << PageShift;

    if (_m5opRange.contains(next_return_addr)) {
        warn("Reached m5ops MMIO region\n");
        return_addr = 0xffffffff;
        pagePtr = 0xffffffff >> PageShift;
    }

    if ((pagePtr << PageShift) > physmem.totalSize())
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
   return physmem.totalSize() - (pagePtr << PageShift);
}

bool
System::isMemAddr(Addr addr) const
{
    return physmem.isMemAddr(addr);
}

void
System::addDeviceMemory(MasterID masterId, AbstractMemory *deviceMemory)
{
    if (!deviceMemMap.count(masterId)) {
        deviceMemMap.insert(std::make_pair(masterId, deviceMemory));
    }
}

bool
System::isDeviceMemAddr(PacketPtr pkt) const
{
    const MasterID& mid = pkt->masterId();

    return (deviceMemMap.count(mid) &&
            deviceMemMap.at(mid)->getAddrRange().contains(pkt->getAddr()));
}

AbstractMemory *
System::getDeviceMemory(MasterID mid) const
{
    panic_if(!deviceMemMap.count(mid),
             "No device memory found for MasterID %d\n", mid);
    return deviceMemMap.at(mid);
}

void
System::drainResume()
{
    totalNumInsts = 0;
}

void
System::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(pagePtr);

    for (auto &t: threads.threads) {
        Tick when = 0;
        if (t.resumeEvent && t.resumeEvent->scheduled())
            when = t.resumeEvent->when();
        ContextID id = t.context->contextId();
        paramOut(cp, csprintf("quiesceEndTick_%d", id), when);
    }

    // also serialize the memories in the system
    physmem.serializeSection(cp, "physmem");
}


void
System::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(pagePtr);

    for (auto &t: threads.threads) {
        Tick when = 0;
        ContextID id = t.context->contextId();
        if (!optParamIn(cp, csprintf("quiesceEndTick_%d", id), when) ||
                !when || !t.resumeEvent) {
            continue;
        }
#       if THE_ISA != NULL_ISA
        t.context->getCpuPtr()->schedule(t.resumeEvent, when);
#       endif
    }

    // also unserialize the memories in the system
    physmem.unserializeSection(cp, "physmem");
}

void
System::regStats()
{
    SimObject::regStats();

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
    ios::fmtflags flags(cerr.flags());

    vector<System *>::iterator i = systemList.begin();
    vector<System *>::iterator end = systemList.end();
    for (; i != end; ++i) {
        System *sys = *i;
        cerr << "System " << sys->name() << ": " << hex << sys << endl;
    }

    cerr.flags(flags);
}

void
printSystems()
{
    System::printSystems();
}

std::string
System::stripSystemName(const std::string& master_name) const
{
    if (startswith(master_name, name())) {
        return master_name.substr(name().size());
    } else {
        return master_name;
    }
}

MasterID
System::lookupMasterId(const SimObject* obj) const
{
    MasterID id = Request::invldMasterId;

    // number of occurrences of the SimObject pointer
    // in the master list.
    auto obj_number = 0;

    for (int i = 0; i < masters.size(); i++) {
        if (masters[i].obj == obj) {
            id = i;
            obj_number++;
        }
    }

    fatal_if(obj_number > 1,
        "Cannot lookup MasterID by SimObject pointer: "
        "More than one master is sharing the same SimObject\n");

    return id;
}

MasterID
System::lookupMasterId(const std::string& master_name) const
{
    std::string name = stripSystemName(master_name);

    for (int i = 0; i < masters.size(); i++) {
        if (masters[i].masterName == name) {
            return i;
        }
    }

    return Request::invldMasterId;
}

MasterID
System::getGlobalMasterId(const std::string& master_name)
{
    return _getMasterId(nullptr, master_name);
}

MasterID
System::getMasterId(const SimObject* master, std::string submaster)
{
    auto master_name = leafMasterName(master, submaster);
    return _getMasterId(master, master_name);
}

MasterID
System::_getMasterId(const SimObject* master, const std::string& master_name)
{
    std::string name = stripSystemName(master_name);

    // CPUs in switch_cpus ask for ids again after switching
    for (int i = 0; i < masters.size(); i++) {
        if (masters[i].masterName == name) {
            return i;
        }
    }

    // Verify that the statistics haven't been enabled yet
    // Otherwise objects will have sized their stat buckets and
    // they will be too small

    if (Stats::enabled()) {
        fatal("Can't request a masterId after regStats(). "
                "You must do so in init().\n");
    }

    // Generate a new MasterID incrementally
    MasterID master_id = masters.size();

    // Append the new Master metadata to the group of system Masters.
    masters.emplace_back(master, name, master_id);

    return masters.back().masterId;
}

std::string
System::leafMasterName(const SimObject* master, const std::string& submaster)
{
    if (submaster.empty()) {
        return master->name();
    } else {
        // Get the full master name by appending the submaster name to
        // the root SimObject master name
        return master->name() + "." + submaster;
    }
}

std::string
System::getMasterName(MasterID master_id)
{
    if (master_id >= masters.size())
        fatal("Invalid master_id passed to getMasterName()\n");

    const auto& master_info = masters[master_id];
    return master_info.masterName;
}

System *
SystemParams::create()
{
    return new System(this);
}
