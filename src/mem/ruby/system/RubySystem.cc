/*
 * Copyright (c) 2019,2021 ARM Limited
 * All rights reserved.
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
 * Copyright (c) 1999-2011 Mark D. Hill and David A. Wood
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

#include "mem/ruby/system/RubySystem.hh"

#include <fcntl.h>
#include <zlib.h>

#include <cstdio>
#include <list>

#include "base/compiler.hh"
#include "base/intmath.hh"
#include "base/statistics.hh"
#include "debug/RubyCacheTrace.hh"
#include "debug/RubySystem.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/network/Network.hh"
#include "mem/ruby/system/DMASequencer.hh"
#include "mem/ruby/system/Sequencer.hh"
#include "mem/simple_mem.hh"
#include "sim/eventq.hh"
#include "sim/simulate.hh"
#include "sim/system.hh"

namespace gem5
{

namespace ruby
{

bool RubySystem::m_randomization;
uint32_t RubySystem::m_block_size_bytes;
uint32_t RubySystem::m_block_size_bits;
uint32_t RubySystem::m_memory_size_bits;
bool RubySystem::m_warmup_enabled = false;
// To look forward to allowing multiple RubySystem instances, track the number
// of RubySystems that need to be warmed up on checkpoint restore.
unsigned RubySystem::m_systems_to_warmup = 0;
bool RubySystem::m_cooldown_enabled = false;

RubySystem::RubySystem(const Params &p)
    : ClockedObject(p), m_access_backing_store(p.access_backing_store),
      m_cache_recorder(NULL)
{
    m_randomization = p.randomization;

    m_block_size_bytes = p.block_size_bytes;
    assert(isPowerOf2(m_block_size_bytes));
    m_block_size_bits = floorLog2(m_block_size_bytes);
    m_memory_size_bits = p.memory_size_bits;

    // Resize to the size of different machine types
    m_abstract_controls.resize(MachineType_NUM);

    // Collate the statistics before they are printed.
    statistics::registerDumpCallback([this]() { collateStats(); });
    // Create the profiler
    m_profiler = new Profiler(p, this);
    m_phys_mem = p.phys_mem;
}

void
RubySystem::registerNetwork(Network* network_ptr)
{
    m_networks.emplace_back(network_ptr);
}

void
RubySystem::registerAbstractController(AbstractController* cntrl)
{
    m_abs_cntrl_vec.push_back(cntrl);

    MachineID id = cntrl->getMachineID();
    m_abstract_controls[id.getType()][id.getNum()] = cntrl;
}

void
RubySystem::registerMachineID(const MachineID& mach_id, Network* network)
{
    int network_id = -1;
    for (int idx = 0; idx < m_networks.size(); ++idx) {
        if (m_networks[idx].get() == network) {
            network_id = idx;
        }
    }

    fatal_if(network_id < 0, "Could not add MachineID %s. Network not found",
             MachineIDToString(mach_id).c_str());

    machineToNetwork.insert(std::make_pair(mach_id, network_id));
}

// This registers all requestor IDs in the system for functional reads. This
// should be called in init() since requestor IDs are obtained in a SimObject's
// constructor and there are functional reads/writes between init() and
// startup().
void
RubySystem::registerRequestorIDs()
{
    // Create the map for RequestorID to network node. This is done in init()
    // because all RequestorIDs must be obtained in the constructor and
    // AbstractControllers are registered in their constructor. This is done
    // in two steps: (1) Add all of the AbstractControllers. Since we don't
    // have a mapping of RequestorID to MachineID this is the easiest way to
    // filter out AbstractControllers from non-Ruby requestors. (2) Go through
    // the system's list of RequestorIDs and add missing RequestorIDs to
    // network 0 (the default).
    for (auto& cntrl : m_abs_cntrl_vec) {
        RequestorID id = cntrl->getRequestorId();
        MachineID mach_id = cntrl->getMachineID();

        // These are setup in Network constructor and should exist
        fatal_if(!machineToNetwork.count(mach_id),
                 "No machineID %s. Does not belong to a Ruby network?",
                 MachineIDToString(mach_id).c_str());

        auto network_id = machineToNetwork[mach_id];
        requestorToNetwork.insert(std::make_pair(id, network_id));

        // Create helper vectors for each network to iterate over.
        netCntrls[network_id].push_back(cntrl);
    }

    // Default all other requestor IDs to network 0
    for (auto id = 0; id < params().system->maxRequestors(); ++id) {
        if (!requestorToNetwork.count(id)) {
            requestorToNetwork.insert(std::make_pair(id, 0));
        }
    }
}

RubySystem::~RubySystem()
{
    delete m_profiler;
}

void
RubySystem::makeCacheRecorder(uint8_t *uncompressed_trace,
                              uint64_t cache_trace_size,
                              uint64_t block_size_bytes)
{
    std::vector<RubyPort*> ruby_port_map;
    RubyPort* ruby_port_ptr = NULL;

    for (int cntrl = 0; cntrl < m_abs_cntrl_vec.size(); cntrl++) {
        if (m_abs_cntrl_vec[cntrl]->getGPUCoalescer() != NULL) {
            ruby_port_map.push_back(
                    (RubyPort*)m_abs_cntrl_vec[cntrl]->getGPUCoalescer());
        } else {
            ruby_port_map.push_back(
                    (RubyPort*)m_abs_cntrl_vec[cntrl]->getCPUSequencer());
        }

        if (ruby_port_ptr == NULL) {
            ruby_port_ptr = ruby_port_map[cntrl];
        }
    }

    assert(ruby_port_ptr != NULL);

    for (int cntrl = 0; cntrl < m_abs_cntrl_vec.size(); cntrl++) {
        if (ruby_port_map[cntrl] == NULL) {
            ruby_port_map[cntrl] = ruby_port_ptr;
        } else {
            ruby_port_ptr = ruby_port_map[cntrl];
        }

    }

    // Remove the old CacheRecorder if it's still hanging about.
    if (m_cache_recorder != NULL) {
        delete m_cache_recorder;
    }

    // Create the CacheRecorder and record the cache trace
    m_cache_recorder = new CacheRecorder(uncompressed_trace, cache_trace_size,
                                         ruby_port_map,
                                         block_size_bytes);
}

void
RubySystem::memWriteback()
{
    m_cooldown_enabled = true;

    // Make the trace so we know what to write back.
    DPRINTF(RubyCacheTrace, "Recording Cache Trace\n");
    makeCacheRecorder(NULL, 0, getBlockSizeBytes());
    for (int cntrl = 0; cntrl < m_abs_cntrl_vec.size(); cntrl++) {
        m_abs_cntrl_vec[cntrl]->recordCacheTrace(cntrl, m_cache_recorder);
    }
    DPRINTF(RubyCacheTrace, "Cache Trace Complete\n");

    // If there is no dirty block, we don't need to flush the cache
    if (m_cache_recorder->getNumRecords() == 0)
    {
        m_cooldown_enabled = false;
        return;
    }

    // save the current tick value
    Tick curtick_original = curTick();
    DPRINTF(RubyCacheTrace, "Recording current tick %ld\n", curtick_original);

    // Deschedule all prior events on the event queue, but record the tick they
    // were scheduled at so they can be restored correctly later.
    std::list<std::pair<Event*, Tick> > original_events;
    while (!eventq->empty()) {
        Event *curr_head = eventq->getHead();
        if (curr_head->isAutoDelete()) {
            DPRINTF(RubyCacheTrace, "Event %s auto-deletes when descheduled,"
                    " not recording\n", curr_head->name());
        } else {
            original_events.push_back(
                    std::make_pair(curr_head, curr_head->when()));
        }
        eventq->deschedule(curr_head);
    }

    // Schedule an event to start cache cooldown
    DPRINTF(RubyCacheTrace, "Starting cache flush\n");
    enqueueRubyEvent(curTick());
    simulate();
    DPRINTF(RubyCacheTrace, "Cache flush complete\n");

    // Deschedule any events left on the event queue.
    while (!eventq->empty()) {
        eventq->deschedule(eventq->getHead());
    }

    // Restore curTick
    setCurTick(curtick_original);

    // Restore all events that were originally on the event queue.  This is
    // done after setting curTick back to its original value so that events do
    // not seem to be scheduled in the past.
    while (!original_events.empty()) {
        std::pair<Event*, Tick> event = original_events.back();
        eventq->schedule(event.first, event.second);
        original_events.pop_back();
    }

    // No longer flushing back to memory.
    m_cooldown_enabled = false;

    // There are several issues with continuing simulation after calling
    // memWriteback() at the moment, that stem from taking events off the
    // queue, simulating again, and then putting them back on, whilst
    // pretending that no time has passed.  One is that some events will have
    // been deleted, so can't be put back.  Another is that any object
    // recording the tick something happens may end up storing a tick in the
    // future.  A simple warning here alerts the user that things may not work
    // as expected.
    warn_once("Ruby memory writeback is experimental.  Continuing simulation "
              "afterwards may not always work as intended.");

    // Keep the cache recorder around so that we can dump the trace if a
    // checkpoint is immediately taken.
}

void
RubySystem::writeCompressedTrace(uint8_t *raw_data, std::string filename,
                                 uint64_t uncompressed_trace_size)
{
    // Create the checkpoint file for the memory
    std::string thefile = CheckpointIn::dir() + "/" + filename.c_str();

    int fd = creat(thefile.c_str(), 0664);
    if (fd < 0) {
        perror("creat");
        fatal("Can't open memory trace file '%s'\n", filename);
    }

    gzFile compressedMemory = gzdopen(fd, "wb");
    if (compressedMemory == NULL)
        fatal("Insufficient memory to allocate compression state for %s\n",
              filename);

    if (gzwrite(compressedMemory, raw_data, uncompressed_trace_size) !=
        uncompressed_trace_size) {
        fatal("Write failed on memory trace file '%s'\n", filename);
    }

    if (gzclose(compressedMemory)) {
        fatal("Close failed on memory trace file '%s'\n", filename);
    }
    delete[] raw_data;
}

void
RubySystem::serialize(CheckpointOut &cp) const
{
    // Store the cache-block size, so we are able to restore on systems
    // with a different cache-block size. CacheRecorder depends on the
    // correct cache-block size upon unserializing.
    uint64_t block_size_bytes = getBlockSizeBytes();
    SERIALIZE_SCALAR(block_size_bytes);

    // Check that there's a valid trace to use.  If not, then memory won't
    // be up-to-date and the simulation will probably fail when restoring
    // from the checkpoint.
    if (m_cache_recorder == NULL) {
        fatal("Call memWriteback() before serialize() to create"
                "ruby trace");
    }

    // Aggregate the trace entries together into a single array
    uint8_t *raw_data = new uint8_t[4096];
    uint64_t cache_trace_size = m_cache_recorder->aggregateRecords(
                                                        &raw_data, 4096);
    std::string cache_trace_file = name() + ".cache.gz";
    writeCompressedTrace(raw_data, cache_trace_file, cache_trace_size);

    SERIALIZE_SCALAR(cache_trace_file);
    SERIALIZE_SCALAR(cache_trace_size);
}

void
RubySystem::drainResume()
{
    // Delete the cache recorder if it was created in memWriteback()
    // to checkpoint the current cache state.
    if (m_cache_recorder) {
        delete m_cache_recorder;
        m_cache_recorder = NULL;
    }
}

void
RubySystem::readCompressedTrace(std::string filename, uint8_t *&raw_data,
                                uint64_t &uncompressed_trace_size)
{
    // Read the trace file
    gzFile compressedTrace;

    // trace file
    int fd = open(filename.c_str(), O_RDONLY);
    if (fd < 0) {
        perror("open");
        fatal("Unable to open trace file %s", filename);
    }

    compressedTrace = gzdopen(fd, "rb");
    if (compressedTrace == NULL) {
        fatal("Insufficient memory to allocate compression state for %s\n",
              filename);
    }

    raw_data = new uint8_t[uncompressed_trace_size];
    if (gzread(compressedTrace, raw_data, uncompressed_trace_size) <
            uncompressed_trace_size) {
        fatal("Unable to read complete trace from file %s\n", filename);
    }

    if (gzclose(compressedTrace)) {
        fatal("Failed to close cache trace file '%s'\n", filename);
    }
}

void
RubySystem::unserialize(CheckpointIn &cp)
{
    uint8_t *uncompressed_trace = NULL;

    // This value should be set to the checkpoint-system's block-size.
    // Optional, as checkpoints without it can be run if the
    // checkpoint-system's block-size == current block-size.
    uint64_t block_size_bytes = getBlockSizeBytes();
    UNSERIALIZE_OPT_SCALAR(block_size_bytes);

    std::string cache_trace_file;
    uint64_t cache_trace_size = 0;

    UNSERIALIZE_SCALAR(cache_trace_file);
    UNSERIALIZE_SCALAR(cache_trace_size);
    cache_trace_file = cp.getCptDir() + "/" + cache_trace_file;

    readCompressedTrace(cache_trace_file, uncompressed_trace,
                        cache_trace_size);
    m_warmup_enabled = true;
    m_systems_to_warmup++;

    // Create the cache recorder that will hang around until startup.
    makeCacheRecorder(uncompressed_trace, cache_trace_size, block_size_bytes);
}

void
RubySystem::init()
{
    registerRequestorIDs();
}

void
RubySystem::startup()
{

    // Ruby restores state from a checkpoint by resetting the clock to 0 and
    // playing the requests that can possibly re-generate the cache state.
    // The clock value is set to the actual checkpointed value once all the
    // requests have been executed.
    //
    // This way of restoring state is pretty finicky. For example, if a
    // Ruby component reads time before the state has been restored, it would
    // cache this value and hence its clock would not be reset to 0, when
    // Ruby resets the global clock. This can potentially result in a
    // deadlock.
    //
    // The solution is that no Ruby component should read time before the
    // simulation starts. And then one also needs to hope that the time
    // Ruby finishes restoring the state is less than the time when the
    // state was checkpointed.

    if (m_warmup_enabled) {
        DPRINTF(RubyCacheTrace, "Starting ruby cache warmup\n");
        // save the current tick value
        Tick curtick_original = curTick();
        // save the event queue head
        Event* eventq_head = eventq->replaceHead(NULL);
        // save the exit event pointer
        GlobalSimLoopExitEvent *original_simulate_limit_event = nullptr;
        original_simulate_limit_event = simulate_limit_event;
        // set curTick to 0 and reset Ruby System's clock
        setCurTick(0);
        resetClock();

        // Schedule an event to start cache warmup
        enqueueRubyEvent(curTick());
        simulate();

        delete m_cache_recorder;
        m_cache_recorder = NULL;
        m_systems_to_warmup--;
        if (m_systems_to_warmup == 0) {
            m_warmup_enabled = false;
        }

        // Restore eventq head
        eventq->replaceHead(eventq_head);
        // Restore exit event pointer
        simulate_limit_event = original_simulate_limit_event;
        // Restore curTick and Ruby System's clock
        setCurTick(curtick_original);
        resetClock();
    }

    resetStats();
}

void
RubySystem::processRubyEvent()
{
    if (getWarmupEnabled()) {
        m_cache_recorder->enqueueNextFetchRequest();
    } else if (getCooldownEnabled()) {
        m_cache_recorder->enqueueNextFlushRequest();
    }
}

void
RubySystem::resetStats()
{
    m_start_cycle = curCycle();
    for (auto& network : m_networks) {
        network->resetStats();
    }
    ClockedObject::resetStats();
}

#ifndef PARTIAL_FUNC_READS
bool
RubySystem::functionalRead(PacketPtr pkt)
{
    Addr address(pkt->getAddr());
    Addr line_address = makeLineAddress(address);

    AccessPermission access_perm = AccessPermission_NotPresent;

    DPRINTF(RubySystem, "Functional Read request for %#x\n", address);

    unsigned int num_ro = 0;
    unsigned int num_rw = 0;
    unsigned int num_busy = 0;
    unsigned int num_maybe_stale = 0;
    unsigned int num_backing_store = 0;
    unsigned int num_invalid = 0;

    // Only send functional requests within the same network.
    assert(requestorToNetwork.count(pkt->requestorId()));
    int request_net_id = requestorToNetwork[pkt->requestorId()];
    assert(netCntrls.count(request_net_id));

    AbstractController *ctrl_ro = nullptr;
    AbstractController *ctrl_rw = nullptr;
    AbstractController *ctrl_backing_store = nullptr;

    // In this loop we count the number of controllers that have the given
    // address in read only, read write and busy states.
    for (auto& cntrl : netCntrls[request_net_id]) {
        access_perm = cntrl-> getAccessPermission(line_address);
        if (access_perm == AccessPermission_Read_Only){
            num_ro++;
            if (ctrl_ro == nullptr) ctrl_ro = cntrl;
        }
        else if (access_perm == AccessPermission_Read_Write){
            num_rw++;
            if (ctrl_rw == nullptr) ctrl_rw = cntrl;
        }
        else if (access_perm == AccessPermission_Busy)
            num_busy++;
        else if (access_perm == AccessPermission_Maybe_Stale)
            num_maybe_stale++;
        else if (access_perm == AccessPermission_Backing_Store) {
            // See RubySlicc_Exports.sm for details, but Backing_Store is meant
            // to represent blocks in memory *for Broadcast/Snooping protocols*,
            // where memory has no idea whether it has an exclusive copy of data
            // or not.
            num_backing_store++;
            if (ctrl_backing_store == nullptr)
                ctrl_backing_store = cntrl;
        }
        else if (access_perm == AccessPermission_Invalid ||
                 access_perm == AccessPermission_NotPresent)
            num_invalid++;
    }

    // This if case is meant to capture what happens in a Broadcast/Snoop
    // protocol where the block does not exist in the cache hierarchy. You
    // only want to read from the Backing_Store memory if there is no copy in
    // the cache hierarchy, otherwise you want to try to read the RO or RW
    // copies existing in the cache hierarchy (covered by the else statement).
    // The reason is because the Backing_Store memory could easily be stale, if
    // there are copies floating around the cache hierarchy, so you want to read
    // it only if it's not in the cache hierarchy at all.
    int num_controllers = netCntrls[request_net_id].size();
    if (num_invalid == (num_controllers - 1) && num_backing_store == 1) {
        DPRINTF(RubySystem, "only copy in Backing_Store memory, read from it\n");
        ctrl_backing_store->functionalRead(line_address, pkt);
        return true;
    } else if (num_ro > 0 || num_rw >= 1) {
        if (num_rw > 1) {
            // We iterate over the vector of abstract controllers, and return
            // the first copy found. If we have more than one cache with block
            // in writable permission, the first one found would be returned.
            warn("More than one Abstract Controller with RW permission for "
                 "addr: %#x on cacheline: %#x.", address, line_address);
        }
        // In Broadcast/Snoop protocols, this covers if you know the block
        // exists somewhere in the caching hierarchy, then you want to read any
        // valid RO or RW block.  In directory protocols, same thing, you want
        // to read any valid readable copy of the block.
        DPRINTF(RubySystem, "num_maybe_stale=%d, num_busy = %d, num_ro = %d, "
                            "num_rw = %d\n",
                num_maybe_stale, num_busy, num_ro, num_rw);
        // Use the copy from the controller with read/write permission (if
        // any), otherwise use get the first read only found
        if (ctrl_rw) {
            ctrl_rw->functionalRead(line_address, pkt);
        } else {
            assert(ctrl_ro);
            ctrl_ro->functionalRead(line_address, pkt);
        }
        return true;
    } else if ((num_busy + num_maybe_stale) > 0) {
        // No controller has a valid copy of the block, but a transient or
        // stale state indicates a valid copy should be in transit in the
        // network or in a message buffer waiting to be handled
        DPRINTF(RubySystem, "Controllers functionalRead lookup "
                            "(num_maybe_stale=%d, num_busy = %d)\n",
                num_maybe_stale, num_busy);
        for (auto& cntrl : netCntrls[request_net_id]) {
            if (cntrl->functionalReadBuffers(pkt))
                return true;
        }
        DPRINTF(RubySystem, "Network functionalRead lookup "
                            "(num_maybe_stale=%d, num_busy = %d)\n",
                num_maybe_stale, num_busy);
        for (auto& network : m_networks) {
            if (network->functionalRead(pkt))
                return true;
        }
    }

    return false;
}
#else
bool
RubySystem::functionalRead(PacketPtr pkt)
{
    Addr address(pkt->getAddr());
    Addr line_address = makeLineAddress(address);

    DPRINTF(RubySystem, "Functional Read request for %#x\n", address);

    std::vector<AbstractController*> ctrl_ro;
    std::vector<AbstractController*> ctrl_busy;
    std::vector<AbstractController*> ctrl_others;
    AbstractController *ctrl_rw = nullptr;
    AbstractController *ctrl_bs = nullptr;

    // Build lists of controllers that have line
    for (auto ctrl : m_abs_cntrl_vec) {
        switch(ctrl->getAccessPermission(line_address)) {
            case AccessPermission_Read_Only:
                ctrl_ro.push_back(ctrl);
                break;
            case AccessPermission_Busy:
                ctrl_busy.push_back(ctrl);
                break;
            case AccessPermission_Read_Write:
                assert(ctrl_rw == nullptr);
                ctrl_rw = ctrl;
                break;
            case AccessPermission_Backing_Store:
                assert(ctrl_bs == nullptr);
                ctrl_bs = ctrl;
                break;
            case AccessPermission_Backing_Store_Busy:
                assert(ctrl_bs == nullptr);
                ctrl_bs = ctrl;
                ctrl_busy.push_back(ctrl);
                break;
            default:
                ctrl_others.push_back(ctrl);
                break;
        }
    }

    DPRINTF(RubySystem, "num_ro=%d, num_busy=%d , has_rw=%d, "
                        "backing_store=%d\n",
                ctrl_ro.size(), ctrl_busy.size(),
                ctrl_rw != nullptr, ctrl_bs != nullptr);

    // Issue functional reads to all controllers found in a stable state
    // until we get a full copy of the line
    WriteMask bytes;
    if (ctrl_rw != nullptr) {
        ctrl_rw->functionalRead(line_address, pkt, bytes);
        // if a RW controllter has the full line that's all uptodate
        if (bytes.isFull())
            return true;
    }

    // Get data from RO and BS
    for (auto ctrl : ctrl_ro)
        ctrl->functionalRead(line_address, pkt, bytes);

    if (ctrl_bs)
        ctrl_bs->functionalRead(line_address, pkt, bytes);

    // if there is any busy controller or bytes still not set, then a partial
    // and/or dirty copy of the line might be in a message buffer or the
    // network
    if (!ctrl_busy.empty() || !bytes.isFull()) {
        DPRINTF(RubySystem, "Reading from remaining controllers, "
                            "buffers and networks\n");
        if (ctrl_rw != nullptr)
            ctrl_rw->functionalReadBuffers(pkt, bytes);
        for (auto ctrl : ctrl_ro)
            ctrl->functionalReadBuffers(pkt, bytes);
        if (ctrl_bs != nullptr)
            ctrl_bs->functionalReadBuffers(pkt, bytes);
        for (auto ctrl : ctrl_busy) {
            ctrl->functionalRead(line_address, pkt, bytes);
            ctrl->functionalReadBuffers(pkt, bytes);
        }
        for (auto& network : m_networks) {
            network->functionalRead(pkt, bytes);
        }
        for (auto ctrl : ctrl_others) {
            ctrl->functionalRead(line_address, pkt, bytes);
            ctrl->functionalReadBuffers(pkt, bytes);
        }
    }
    // we either got the full line or couldn't find anything at this point
    panic_if(!(bytes.isFull() || bytes.isEmpty()),
            "Inconsistent state on functional read for %#x %s\n",
            address, bytes);

    return bytes.isFull();
}
#endif

// The function searches through all the buffers that exist in different
// cache, directory and memory controllers, and in the network components
// and writes the data portion of those that hold the address specified
// in the packet.
bool
RubySystem::functionalWrite(PacketPtr pkt)
{
    Addr addr(pkt->getAddr());
    Addr line_addr = makeLineAddress(addr);
    AccessPermission access_perm = AccessPermission_NotPresent;

    DPRINTF(RubySystem, "Functional Write request for %#x\n", addr);

    [[maybe_unused]] uint32_t num_functional_writes = 0;

    // Only send functional requests within the same network.
    assert(requestorToNetwork.count(pkt->requestorId()));
    int request_net_id = requestorToNetwork[pkt->requestorId()];
    assert(netCntrls.count(request_net_id));

    for (auto& cntrl : netCntrls[request_net_id]) {
        num_functional_writes += cntrl->functionalWriteBuffers(pkt);

        access_perm = cntrl->getAccessPermission(line_addr);
        if (access_perm != AccessPermission_Invalid &&
            access_perm != AccessPermission_NotPresent) {
            num_functional_writes +=
                cntrl->functionalWrite(line_addr, pkt);
        }

        // Also updates requests pending in any sequencer associated
        // with the controller
        if (cntrl->getCPUSequencer()) {
            num_functional_writes +=
                cntrl->getCPUSequencer()->functionalWrite(pkt);
        }
        if (cntrl->getDMASequencer()) {
            num_functional_writes +=
                cntrl->getDMASequencer()->functionalWrite(pkt);
        }
    }

    for (auto& network : m_networks) {
        num_functional_writes += network->functionalWrite(pkt);
    }
    DPRINTF(RubySystem, "Messages written = %u\n", num_functional_writes);

    return true;
}

} // namespace ruby
} // namespace gem5
