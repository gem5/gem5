/*
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

#include <fcntl.h>
#include <zlib.h>

#include <cstdio>

#include "base/intmath.hh"
#include "base/statistics.hh"
#include "debug/RubyCacheTrace.hh"
#include "debug/RubySystem.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/network/Network.hh"
#include "mem/ruby/system/System.hh"
#include "sim/eventq.hh"
#include "sim/simulate.hh"

using namespace std;

int RubySystem::m_random_seed;
bool RubySystem::m_randomization;
uint32_t RubySystem::m_block_size_bytes;
uint32_t RubySystem::m_block_size_bits;
uint64_t RubySystem::m_memory_size_bytes;
uint32_t RubySystem::m_memory_size_bits;

RubySystem::RubySystem(const Params *p)
    : ClockedObject(p)
{
    if (g_system_ptr != NULL)
        fatal("Only one RubySystem object currently allowed.\n");

    m_random_seed = p->random_seed;
    srandom(m_random_seed);
    m_randomization = p->randomization;

    m_block_size_bytes = p->block_size_bytes;
    assert(isPowerOf2(m_block_size_bytes));
    m_block_size_bits = floorLog2(m_block_size_bytes);

    m_memory_size_bytes = p->mem_size;
    if (m_memory_size_bytes == 0) {
        m_memory_size_bits = 0;
    } else {
        m_memory_size_bits = ceilLog2(m_memory_size_bytes);
    }

    if (p->no_mem_vec) {
        m_mem_vec = NULL;
    } else {
        m_mem_vec = new MemoryVector;
        m_mem_vec->resize(m_memory_size_bytes);
    }

    m_warmup_enabled = false;
    m_cooldown_enabled = false;

    // Setup the global variables used in Ruby
    g_system_ptr = this;

    // Resize to the size of different machine types
    g_abs_controls.resize(MachineType_NUM);

    // Collate the statistics before they are printed.
    Stats::registerDumpCallback(new RubyStatsCallback(this));
    // Create the profiler
    m_profiler = new Profiler(p);
}

void
RubySystem::registerNetwork(Network* network_ptr)
{
  m_network = network_ptr;
}

void
RubySystem::registerAbstractController(AbstractController* cntrl)
{
  m_abs_cntrl_vec.push_back(cntrl);

  MachineID id = cntrl->getMachineID();
  g_abs_controls[id.getType()][id.getNum()] = cntrl;
}

void
RubySystem::registerSparseMemory(SparseMemory* s)
{
    m_sparse_memory_vector.push_back(s);
}

void
RubySystem::registerMemController(MemoryControl *mc) {
    m_memory_controller_vec.push_back(mc);
}

RubySystem::~RubySystem()
{
    delete m_network;
    delete m_profiler;
    if (m_mem_vec)
        delete m_mem_vec;
}

void
RubySystem::writeCompressedTrace(uint8_t *raw_data, string filename,
                                 uint64 uncompressed_trace_size)
{
    // Create the checkpoint file for the memory
    string thefile = Checkpoint::dir() + "/" + filename.c_str();

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
    delete raw_data;
}

void
RubySystem::serialize(std::ostream &os)
{
    m_cooldown_enabled = true;

    vector<Sequencer*> sequencer_map;
    Sequencer* sequencer_ptr = NULL;
    int cntrl_id = -1;


    for (int cntrl = 0; cntrl < m_abs_cntrl_vec.size(); cntrl++) {
        sequencer_map.push_back(m_abs_cntrl_vec[cntrl]->getSequencer());
        if (sequencer_ptr == NULL) {
            sequencer_ptr = sequencer_map[cntrl];
            cntrl_id = cntrl;
        }
    }

    assert(sequencer_ptr != NULL);

    for (int cntrl = 0; cntrl < m_abs_cntrl_vec.size(); cntrl++) {
        if (sequencer_map[cntrl] == NULL) {
            sequencer_map[cntrl] = sequencer_ptr;
        }
    }

    DPRINTF(RubyCacheTrace, "Recording Cache Trace\n");
    // Create the CacheRecorder and record the cache trace
    m_cache_recorder = new CacheRecorder(NULL, 0, sequencer_map);

    for (int cntrl = 0; cntrl < m_abs_cntrl_vec.size(); cntrl++) {
        m_abs_cntrl_vec[cntrl]->recordCacheTrace(cntrl, m_cache_recorder);
    }

    DPRINTF(RubyCacheTrace, "Cache Trace Complete\n");
    // save the current tick value
    Tick curtick_original = curTick();
    // save the event queue head
    Event* eventq_head = eventq->replaceHead(NULL);
    DPRINTF(RubyCacheTrace, "Recording current tick %ld and event queue\n",
            curtick_original);

    // Schedule an event to start cache cooldown
    DPRINTF(RubyCacheTrace, "Starting cache flush\n");
    enqueueRubyEvent(curTick());
    simulate();
    DPRINTF(RubyCacheTrace, "Cache flush complete\n");

    // Restore eventq head
    eventq_head = eventq->replaceHead(eventq_head);
    // Restore curTick
    setCurTick(curtick_original);

    uint8_t *raw_data = NULL;

    if (m_mem_vec != NULL) {
        uint64 memory_trace_size = m_mem_vec->collatePages(raw_data);

        string memory_trace_file = name() + ".memory.gz";
        writeCompressedTrace(raw_data, memory_trace_file,
                             memory_trace_size);

        SERIALIZE_SCALAR(memory_trace_file);
        SERIALIZE_SCALAR(memory_trace_size);

    } else {
        for (int i = 0; i < m_sparse_memory_vector.size(); ++i) {
            m_sparse_memory_vector[i]->recordBlocks(cntrl_id,
                                                    m_cache_recorder);
        }
    }

    // Aggergate the trace entries together into a single array
    raw_data = new uint8_t[4096];
    uint64 cache_trace_size = m_cache_recorder->aggregateRecords(&raw_data,
                                                                 4096);
    string cache_trace_file = name() + ".cache.gz";
    writeCompressedTrace(raw_data, cache_trace_file, cache_trace_size);

    SERIALIZE_SCALAR(cache_trace_file);
    SERIALIZE_SCALAR(cache_trace_size);

    m_cooldown_enabled = false;
}

void
RubySystem::readCompressedTrace(string filename, uint8_t *&raw_data,
                                uint64& uncompressed_trace_size)
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
RubySystem::unserialize(Checkpoint *cp, const string &section)
{
    uint8_t *uncompressed_trace = NULL;

    if (m_mem_vec != NULL) {
        string memory_trace_file;
        uint64 memory_trace_size = 0;

        UNSERIALIZE_SCALAR(memory_trace_file);
        UNSERIALIZE_SCALAR(memory_trace_size);
        memory_trace_file = cp->cptDir + "/" + memory_trace_file;

        readCompressedTrace(memory_trace_file, uncompressed_trace,
                            memory_trace_size);
        m_mem_vec->populatePages(uncompressed_trace);

        delete [] uncompressed_trace;
        uncompressed_trace = NULL;
    }

    string cache_trace_file;
    uint64 cache_trace_size = 0;

    UNSERIALIZE_SCALAR(cache_trace_file);
    UNSERIALIZE_SCALAR(cache_trace_size);
    cache_trace_file = cp->cptDir + "/" + cache_trace_file;

    readCompressedTrace(cache_trace_file, uncompressed_trace,
                        cache_trace_size);
    m_warmup_enabled = true;

    vector<Sequencer*> sequencer_map;
    Sequencer* t = NULL;
    for (int cntrl = 0; cntrl < m_abs_cntrl_vec.size(); cntrl++) {
        sequencer_map.push_back(m_abs_cntrl_vec[cntrl]->getSequencer());
        if (t == NULL) t = sequencer_map[cntrl];
    }

    assert(t != NULL);

    for (int cntrl = 0; cntrl < m_abs_cntrl_vec.size(); cntrl++) {
        if (sequencer_map[cntrl] == NULL) {
            sequencer_map[cntrl] = t;
        }
    }

    m_cache_recorder = new CacheRecorder(uncompressed_trace, cache_trace_size,
                                         sequencer_map);
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
        // save the current tick value
        Tick curtick_original = curTick();
        // save the event queue head
        Event* eventq_head = eventq->replaceHead(NULL);
        // set curTick to 0 and reset Ruby System's clock
        setCurTick(0);
        resetClock();

        // Schedule an event to start cache warmup
        enqueueRubyEvent(curTick());
        simulate();

        delete m_cache_recorder;
        m_cache_recorder = NULL;
        m_warmup_enabled = false;

        // reset DRAM so that it's not waiting for events on the old event
        // queue
        for (int i = 0; i < m_memory_controller_vec.size(); ++i) {
            m_memory_controller_vec[i]->reset();
        }

        // Restore eventq head
        eventq_head = eventq->replaceHead(eventq_head);
        // Restore curTick and Ruby System's clock
        setCurTick(curtick_original);
        resetClock();
    }

    resetStats();
}

void
RubySystem::RubyEvent::process()
{
    if (ruby_system->m_warmup_enabled) {
        ruby_system->m_cache_recorder->enqueueNextFetchRequest();
    }  else if (ruby_system->m_cooldown_enabled) {
        ruby_system->m_cache_recorder->enqueueNextFlushRequest();
    }
}

void
RubySystem::resetStats()
{
    g_ruby_start = curCycle();
}

bool
RubySystem::functionalRead(PacketPtr pkt)
{
    Address address(pkt->getAddr());
    Address line_address(address);
    line_address.makeLineAddress();

    AccessPermission access_perm = AccessPermission_NotPresent;
    int num_controllers = m_abs_cntrl_vec.size();

    DPRINTF(RubySystem, "Functional Read request for %s\n",address);

    unsigned int num_ro = 0;
    unsigned int num_rw = 0;
    unsigned int num_busy = 0;
    unsigned int num_backing_store = 0;
    unsigned int num_invalid = 0;

    // In this loop we count the number of controllers that have the given
    // address in read only, read write and busy states.
    for (unsigned int i = 0; i < num_controllers; ++i) {
        access_perm = m_abs_cntrl_vec[i]-> getAccessPermission(line_address);
        if (access_perm == AccessPermission_Read_Only)
            num_ro++;
        else if (access_perm == AccessPermission_Read_Write)
            num_rw++;
        else if (access_perm == AccessPermission_Busy)
            num_busy++;
        else if (access_perm == AccessPermission_Backing_Store)
            // See RubySlicc_Exports.sm for details, but Backing_Store is meant
            // to represent blocks in memory *for Broadcast/Snooping protocols*,
            // where memory has no idea whether it has an exclusive copy of data
            // or not.
            num_backing_store++;
        else if (access_perm == AccessPermission_Invalid ||
                 access_perm == AccessPermission_NotPresent)
            num_invalid++;
    }
    assert(num_rw <= 1);

    uint8_t *data = pkt->getPtr<uint8_t>(true);
    unsigned int size_in_bytes = pkt->getSize();
    unsigned startByte = address.getAddress() - line_address.getAddress();

    // This if case is meant to capture what happens in a Broadcast/Snoop
    // protocol where the block does not exist in the cache hierarchy. You
    // only want to read from the Backing_Store memory if there is no copy in
    // the cache hierarchy, otherwise you want to try to read the RO or RW
    // copies existing in the cache hierarchy (covered by the else statement).
    // The reason is because the Backing_Store memory could easily be stale, if
    // there are copies floating around the cache hierarchy, so you want to read
    // it only if it's not in the cache hierarchy at all.
    if (num_invalid == (num_controllers - 1) &&
            num_backing_store == 1) {
        DPRINTF(RubySystem, "only copy in Backing_Store memory, read from it\n");
        for (unsigned int i = 0; i < num_controllers; ++i) {
            access_perm = m_abs_cntrl_vec[i]->getAccessPermission(line_address);
            if (access_perm == AccessPermission_Backing_Store) {
                DataBlock& block = m_abs_cntrl_vec[i]->
                    getDataBlock(line_address);

                DPRINTF(RubySystem, "reading from %s block %s\n",
                        m_abs_cntrl_vec[i]->name(), block);
                for (unsigned j = 0; j < size_in_bytes; ++j) {
                    data[j] = block.getByte(j + startByte);
                }
                return true;
            }
        }
    } else if (num_ro > 0 || num_rw == 1) {
        // In Broadcast/Snoop protocols, this covers if you know the block
        // exists somewhere in the caching hierarchy, then you want to read any
        // valid RO or RW block.  In directory protocols, same thing, you want
        // to read any valid readable copy of the block.
        DPRINTF(RubySystem, "num_busy = %d, num_ro = %d, num_rw = %d\n",
                num_busy, num_ro, num_rw);
        // In this loop, we try to figure which controller has a read only or
        // a read write copy of the given address. Any valid copy would suffice
        // for a functional read.
        for (unsigned int i = 0;i < num_controllers;++i) {
            access_perm = m_abs_cntrl_vec[i]->getAccessPermission(line_address);
            if (access_perm == AccessPermission_Read_Only ||
                access_perm == AccessPermission_Read_Write) {
                DataBlock& block = m_abs_cntrl_vec[i]->
                    getDataBlock(line_address);

                DPRINTF(RubySystem, "reading from %s block %s\n",
                        m_abs_cntrl_vec[i]->name(), block);
                for (unsigned j = 0; j < size_in_bytes; ++j) {
                    data[j] = block.getByte(j + startByte);
                }
                return true;
            }
        }
    }

    return false;
}

// The function searches through all the buffers that exist in different
// cache, directory and memory controllers, and in the network components
// and writes the data portion of those that hold the address specified
// in the packet.
bool
RubySystem::functionalWrite(PacketPtr pkt)
{
    Address addr(pkt->getAddr());
    Address line_addr = line_address(addr);
    AccessPermission access_perm = AccessPermission_NotPresent;
    int num_controllers = m_abs_cntrl_vec.size();

    DPRINTF(RubySystem, "Functional Write request for %s\n",addr);

    uint8_t *data = pkt->getPtr<uint8_t>(true);
    unsigned int size_in_bytes = pkt->getSize();
    unsigned startByte = addr.getAddress() - line_addr.getAddress();

    uint32_t M5_VAR_USED num_functional_writes = 0;

    for (unsigned int i = 0; i < num_controllers;++i) {
        num_functional_writes +=
            m_abs_cntrl_vec[i]->functionalWriteBuffers(pkt);

        access_perm = m_abs_cntrl_vec[i]->getAccessPermission(line_addr);
        if (access_perm != AccessPermission_Invalid &&
            access_perm != AccessPermission_NotPresent) {

            num_functional_writes++;

            DataBlock& block = m_abs_cntrl_vec[i]->getDataBlock(line_addr);
            DPRINTF(RubySystem, "%s\n",block);
            for (unsigned j = 0; j < size_in_bytes; ++j) {
              block.setByte(j + startByte, data[j]);
            }
            DPRINTF(RubySystem, "%s\n",block);
        }
    }

    for (unsigned int i = 0; i < m_memory_controller_vec.size() ;++i) {
        num_functional_writes +=
            m_memory_controller_vec[i]->functionalWriteBuffers(pkt);
    }

    num_functional_writes += m_network->functionalWrite(pkt);
    DPRINTF(RubySystem, "Messages written = %u\n", num_functional_writes);

    return true;
}

#ifdef CHECK_COHERENCE
// This code will check for cases if the given cache block is exclusive in
// one node and shared in another-- a coherence violation
//
// To use, the SLICC specification must call sequencer.checkCoherence(address)
// when the controller changes to a state with new permissions.  Do this
// in setState.  The SLICC spec must also define methods "isBlockShared"
// and "isBlockExclusive" that are specific to that protocol
//
void
RubySystem::checkGlobalCoherenceInvariant(const Address& addr)
{
#if 0
    NodeID exclusive = -1;
    bool sharedDetected = false;
    NodeID lastShared = -1;

    for (int i = 0; i < m_chip_vector.size(); i++) {
        if (m_chip_vector[i]->isBlockExclusive(addr)) {
            if (exclusive != -1) {
                // coherence violation
                WARN_EXPR(exclusive);
                WARN_EXPR(m_chip_vector[i]->getID());
                WARN_EXPR(addr);
                WARN_EXPR(getTime());
                ERROR_MSG("Coherence Violation Detected -- 2 exclusive chips");
            } else if (sharedDetected) {
                WARN_EXPR(lastShared);
                WARN_EXPR(m_chip_vector[i]->getID());
                WARN_EXPR(addr);
                WARN_EXPR(getTime());
                ERROR_MSG("Coherence Violation Detected -- exclusive chip with >=1 shared");
            } else {
                exclusive = m_chip_vector[i]->getID();
            }
        } else if (m_chip_vector[i]->isBlockShared(addr)) {
            sharedDetected = true;
            lastShared = m_chip_vector[i]->getID();

            if (exclusive != -1) {
                WARN_EXPR(lastShared);
                WARN_EXPR(exclusive);
                WARN_EXPR(addr);
                WARN_EXPR(getTime());
                ERROR_MSG("Coherence Violation Detected -- exclusive chip with >=1 shared");
            }
        }
    }
#endif
}
#endif

RubySystem *
RubySystemParams::create()
{
    return new RubySystem(this);
}
