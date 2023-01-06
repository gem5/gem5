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

#include "base/compiler.hh"
#include "base/cprintf.hh"
#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "base/str.hh"
#include "base/trace.hh"
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
#include "sim/redirect_path.hh"
#include "sim/serialize_handlers.hh"

namespace gem5
{

std::vector<System *> System::systemList;

void
System::Threads::Thread::resume()
{
    DPRINTFS(Quiesce, context->getCpuPtr(), "activating\n");
    context->activate();
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
    context->getSystemPtr()->workload->recordQuiesce();
}

void
System::Threads::insert(ThreadContext *tc)
{
    ContextID id = size();
    tc->setContextId(id);

    auto &t = threads.emplace_back();
    t.context = tc;
    // Look up this thread again on resume, in case the threads vector has
    // been reallocated.
    t.resumeEvent = new EventFunctionWrapper(
            [this, id](){ thread(id).resume(); },
            tc->getSystemPtr()->name());
}

void
System::Threads::replace(ThreadContext *tc, ContextID id)
{
    auto &t = thread(id);
    panic_if(!t.context, "Can't replace a context which doesn't exist.");
    if (t.resumeEvent->scheduled()) {
        Tick when = t.resumeEvent->when();
        t.context->getCpuPtr()->deschedule(t.resumeEvent);
        tc->getCpuPtr()->schedule(t.resumeEvent, when);
    }
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
    [[maybe_unused]] BaseCPU *cpu = t.context->getCpuPtr();
    DPRINTFS(Quiesce, cpu, "quiesce()\n");
    t.quiesce();
}

void
System::Threads::quiesceTick(ContextID id, Tick when)
{
    auto &t = thread(id);
    BaseCPU *cpu = t.context->getCpuPtr();

    DPRINTFS(Quiesce, cpu, "quiesceTick until %u\n", when);
    t.quiesce();

    cpu->reschedule(t.resumeEvent, when, true);
}

int System::numSystemsRunning = 0;

System::System(const Params &p)
    : SimObject(p), _systemPort("system_port"),
      multiThread(p.multi_thread),
      init_param(p.init_param),
      physProxy(_systemPort, p.cache_line_size),
      workload(p.workload),
      physmem(name() + ".physmem", p.memories, p.mmap_using_noreserve,
              p.shared_backstore, p.auto_unlink_shared_backstore),
      ShadowRomRanges(p.shadow_rom_ranges.begin(),
                      p.shadow_rom_ranges.end()),
      memoryMode(p.mem_mode),
      _cacheLineSize(p.cache_line_size),
      numWorkIds(p.num_work_ids),
      thermalModel(p.thermal_model),
      _m5opRange(p.m5ops_base ?
                 RangeSize(p.m5ops_base, 0x10000) :
                 AddrRange(1, 0)), // Create an empty range if disabled
      redirectPaths(p.redirect_paths)
{
    panic_if(!workload, "No workload set for system %s "
            "(could use StubWorkload?).", name());
    workload->setSystem(this);

    // add self to global system list
    systemList.push_back(this);

    // check if the cache line size is a value known to work
    if (_cacheLineSize != 16 && _cacheLineSize != 32 &&
        _cacheLineSize != 64 && _cacheLineSize != 128) {
        warn_once("Cache line size is neither 16, 32, 64 nor 128 bytes.\n");
    }

    // Get the generic system requestor IDs
    [[maybe_unused]] RequestorID tmp_id;
    tmp_id = getRequestorId(this, "writebacks");
    assert(tmp_id == Request::wbRequestorId);
    tmp_id = getRequestorId(this, "functional");
    assert(tmp_id == Request::funcRequestorId);
    tmp_id = getRequestorId(this, "interrupt");
    assert(tmp_id == Request::intRequestorId);

    // increment the number of running systems
    numSystemsRunning++;

    // Set back pointers to the system in all memories
    for (int x = 0; x < params().memories.size(); x++)
        params().memories[x]->system(this);
}

System::~System()
{
    for (uint32_t j = 0; j < numWorkIds; j++)
        delete workItemStats[j];
}

Port &
System::getPort(const std::string &if_name, PortID idx)
{
    // no need to distinguish at the moment (besides checking)
    return _systemPort;
}

void
System::setMemoryMode(enums::MemoryMode mode)
{
    assert(drainState() == DrainState::Drained);
    memoryMode = mode;
}

void
System::registerThreadContext(ThreadContext *tc)
{
    threads.insert(tc);

    workload->registerThreadContext(tc);

    for (auto *e: liveEvents)
        tc->schedule(e);
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

    workload->replaceThreadContext(tc);

    for (auto *e: liveEvents) {
        otc->remove(e);
        tc->schedule(e);
    }
}

Addr
System::memSize() const
{
    return physmem.totalSize();
}

bool
System::isMemAddr(Addr addr) const
{
    return physmem.isMemAddr(addr);
}

void
System::addDeviceMemory(RequestorID requestor_id,
    memory::AbstractMemory *deviceMemory)
{
    deviceMemMap[requestor_id].push_back(deviceMemory);
}

bool
System::isDeviceMemAddr(const PacketPtr& pkt) const
{
    if (!deviceMemMap.count(pkt->requestorId())) {
        return false;
    }

    return (getDeviceMemory(pkt) != nullptr);
}

memory::AbstractMemory *
System::getDeviceMemory(const PacketPtr& pkt) const
{
    const RequestorID& rid = pkt->requestorId();

    panic_if(!deviceMemMap.count(rid),
             "No device memory found for Requestor %d\n", rid);

    for (auto& mem : deviceMemMap.at(rid)) {
        if (pkt->getAddrRange().isSubset(mem->getAddrRange())) {
            return mem;
        }
    }

    return nullptr;
}

void
System::serialize(CheckpointOut &cp) const
{
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
    for (auto &t: threads.threads) {
        Tick when = 0;
        ContextID id = t.context->contextId();
        if (!optParamIn(cp, csprintf("quiesceEndTick_%d", id), when) ||
                !when || !t.resumeEvent) {
            continue;
        }
        t.context->getCpuPtr()->schedule(t.resumeEvent, when);
    }

    // also unserialize the memories in the system
    physmem.unserializeSection(cp, "physmem");
}

void
System::regStats()
{
    SimObject::regStats();

    for (uint32_t j = 0; j < numWorkIds ; j++) {
        workItemStats[j] = new statistics::Histogram(this);
        std::stringstream namestr;
        ccprintf(namestr, "work_item_type%d", j);
        workItemStats[j]->init(20)
                         .name(namestr.str())
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

bool
System::trapToGdb(GDBSignal signal, ContextID ctx_id) const
{
    return workload->trapToGdb(signal, ctx_id);
}

void
System::printSystems()
{
    std::ios::fmtflags flags(std::cerr.flags());

    std::vector<System *>::iterator i = systemList.begin();
    std::vector<System *>::iterator end = systemList.end();
    for (; i != end; ++i) {
        System *sys = *i;
        std::cerr << "System " << sys->name() << ": " << std::hex << sys
                  << std::endl;
    }

    std::cerr.flags(flags);
}

void
printSystems()
{
    System::printSystems();
}

std::string
System::stripSystemName(const std::string& requestor_name) const
{
    if (startswith(requestor_name, name())) {
        return requestor_name.substr(name().size() + 1);
    } else {
        return requestor_name;
    }
}

RequestorID
System::lookupRequestorId(const SimObject* obj) const
{
    RequestorID id = Request::invldRequestorId;

    // number of occurrences of the SimObject pointer
    // in the requestor list.
    auto obj_number = 0;

    for (int i = 0; i < requestors.size(); i++) {
        if (requestors[i].obj == obj) {
            id = i;
            obj_number++;
        }
    }

    fatal_if(obj_number > 1,
        "Cannot lookup RequestorID by SimObject pointer: "
        "More than one requestor is sharing the same SimObject\n");

    return id;
}

RequestorID
System::lookupRequestorId(const std::string& requestor_name) const
{
    std::string name = stripSystemName(requestor_name);

    for (int i = 0; i < requestors.size(); i++) {
        if (requestors[i].req_name == name) {
            return i;
        }
    }

    return Request::invldRequestorId;
}

RequestorID
System::getGlobalRequestorId(const std::string& requestor_name)
{
    return _getRequestorId(nullptr, requestor_name);
}

RequestorID
System::getRequestorId(const SimObject* requestor, std::string subrequestor)
{
    auto requestor_name = leafRequestorName(requestor, subrequestor);
    return _getRequestorId(requestor, requestor_name);
}

RequestorID
System::_getRequestorId(const SimObject* requestor,
                     const std::string& requestor_name)
{
    std::string name = stripSystemName(requestor_name);

    // CPUs in switch_cpus ask for ids again after switching
    for (int i = 0; i < requestors.size(); i++) {
        if (requestors[i].req_name == name) {
            return i;
        }
    }

    // Verify that the statistics haven't been enabled yet
    // Otherwise objects will have sized their stat buckets and
    // they will be too small

    if (statistics::enabled()) {
        fatal("Can't request a requestorId after regStats(). "
                "You must do so in init().\n");
    }

    // Generate a new RequestorID incrementally
    RequestorID requestor_id = requestors.size();

    // Append the new Requestor metadata to the group of system Requestors.
    requestors.emplace_back(requestor, name, requestor_id);

    return requestors.back().id;
}

std::string
System::leafRequestorName(const SimObject* requestor,
                       const std::string& subrequestor)
{
    if (subrequestor.empty()) {
        return requestor->name();
    } else {
        // Get the full requestor name by appending the subrequestor name to
        // the root SimObject requestor name
        return requestor->name() + "." + subrequestor;
    }
}

std::string
System::getRequestorName(RequestorID requestor_id)
{
    if (requestor_id >= requestors.size())
        fatal("Invalid requestor_id passed to getRequestorName()\n");

    const auto& requestor_info = requestors[requestor_id];
    return requestor_info.req_name;
}

} // namespace gem5
