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
#include "base/output.hh"
#include "debug/RubyCacheTrace.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/network/Network.hh"
#include "mem/ruby/profiler/Profiler.hh"
#include "mem/ruby/system/System.hh"
#include "sim/eventq.hh"
#include "sim/simulate.hh"

using namespace std;

int RubySystem::m_random_seed;
bool RubySystem::m_randomization;
Tick RubySystem::m_clock;
int RubySystem::m_block_size_bytes;
int RubySystem::m_block_size_bits;
uint64 RubySystem::m_memory_size_bytes;
int RubySystem::m_memory_size_bits;

Network* RubySystem::m_network_ptr;
Profiler* RubySystem::m_profiler_ptr;
MemoryVector* RubySystem::m_mem_vec_ptr;

RubySystem::RubySystem(const Params *p)
    : SimObject(p)
{
    if (g_system_ptr != NULL)
        fatal("Only one RubySystem object currently allowed.\n");

    m_random_seed = p->random_seed;
    srandom(m_random_seed);
    m_randomization = p->randomization;
    m_clock = p->clock;

    m_block_size_bytes = p->block_size_bytes;
    assert(isPowerOf2(m_block_size_bytes));
    m_block_size_bits = floorLog2(m_block_size_bytes);

    m_memory_size_bytes = p->mem_size;
    if (m_memory_size_bytes == 0) {
        m_memory_size_bits = 0;
    } else {
        m_memory_size_bits = floorLog2(m_memory_size_bytes);
    }

    g_eventQueue_ptr = new RubyEventQueue(p->eventq, m_clock);
    g_system_ptr = this;
    if (p->no_mem_vec) {
        m_mem_vec_ptr = NULL;
    } else {
        m_mem_vec_ptr = new MemoryVector;
        m_mem_vec_ptr->resize(m_memory_size_bytes);
    }

    //
    // Print ruby configuration and stats at exit
    //
    RubyExitCallback* rubyExitCB = new RubyExitCallback(p->stats_filename);
    registerExitCallback(rubyExitCB);
    m_warmup_enabled = false;
    m_cooldown_enabled = false;
}

void
RubySystem::init()
{
    m_profiler_ptr->clearStats();
}

void
RubySystem::registerNetwork(Network* network_ptr)
{
  m_network_ptr = network_ptr;
}

void
RubySystem::registerProfiler(Profiler* profiler_ptr)
{
  m_profiler_ptr = profiler_ptr;
}

void
RubySystem::registerAbstractController(AbstractController* cntrl)
{
  m_abs_cntrl_vec.push_back(cntrl);
}

void
RubySystem::registerSparseMemory(SparseMemory* s)
{
    m_sparse_memory_vector.push_back(s);
}

RubySystem::~RubySystem()
{
    delete m_network_ptr;
    delete m_profiler_ptr;
    if (m_mem_vec_ptr)
        delete m_mem_vec_ptr;
}

void
RubySystem::printSystemConfig(ostream & out)
{
    out << "RubySystem config:" << endl
        << "  random_seed: " << m_random_seed << endl
        << "  randomization: " << m_randomization << endl
        << "  cycle_period: " << m_clock << endl
        << "  block_size_bytes: " << m_block_size_bytes << endl
        << "  block_size_bits: " << m_block_size_bits << endl
        << "  memory_size_bytes: " << m_memory_size_bytes << endl
        << "  memory_size_bits: " << m_memory_size_bits << endl;
}

void
RubySystem::printConfig(ostream& out)
{
    out << "\n================ Begin RubySystem Configuration Print ================\n\n";
    printSystemConfig(out);
    m_network_ptr->printConfig(out);
    m_profiler_ptr->printConfig(out);
    out << "\n================ End RubySystem Configuration Print ================\n\n";
}

void
RubySystem::printStats(ostream& out)
{
    const time_t T = time(NULL);
    tm *localTime = localtime(&T);
    char buf[100];
    strftime(buf, 100, "%b/%d/%Y %H:%M:%S", localTime);

    out << "Real time: " << buf << endl;

    m_profiler_ptr->printStats(out);
    m_network_ptr->printStats(out);
}

void
RubySystem::writeCompressedTrace(uint8* raw_data, string filename,
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
    curTick(curtick_original);

    uint8* raw_data = NULL;

    if (m_mem_vec_ptr != NULL) {
        uint64 memory_trace_size = m_mem_vec_ptr->collatePages(raw_data);

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
RubySystem::readCompressedTrace(string filename, uint8*& raw_data,
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
    //
    // The main purpose for clearing stats in the unserialize process is so
    // that the profiler can correctly set its start time to the unserialized
    // value of curTick()
    //
    clearStats();
    uint8* uncompressed_trace = NULL;

    if (m_mem_vec_ptr != NULL) {
        string memory_trace_file;
        uint64 memory_trace_size = 0;

        UNSERIALIZE_SCALAR(memory_trace_file);
        UNSERIALIZE_SCALAR(memory_trace_size);
        memory_trace_file = cp->cptDir + "/" + memory_trace_file;

        readCompressedTrace(memory_trace_file, uncompressed_trace,
                            memory_trace_size);
        m_mem_vec_ptr->populatePages(uncompressed_trace);

        delete uncompressed_trace;
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
        if(t == NULL) t = sequencer_map[cntrl];
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
    if (m_warmup_enabled) {
        // save the current tick value
        Tick curtick_original = curTick();
        // save the event queue head
        Event* eventq_head = eventq->replaceHead(NULL);
        // set curTick to 0
        curTick(0);

        // Schedule an event to start cache warmup
        enqueueRubyEvent(curTick());
        simulate();

        delete m_cache_recorder;
        m_cache_recorder = NULL;
        m_warmup_enabled = false;
        // Restore eventq head
        eventq_head = eventq->replaceHead(eventq_head);
        // Restore curTick
        curTick(curtick_original);
    }
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
RubySystem::clearStats() const
{
    m_profiler_ptr->clearStats();
    m_network_ptr->clearStats();
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
                WARN_EXPR(g_eventQueue_ptr->getTime());
                ERROR_MSG("Coherence Violation Detected -- 2 exclusive chips");
            } else if (sharedDetected) {
                WARN_EXPR(lastShared);
                WARN_EXPR(m_chip_vector[i]->getID());
                WARN_EXPR(addr);
                WARN_EXPR(g_eventQueue_ptr->getTime());
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
                WARN_EXPR(g_eventQueue_ptr->getTime());
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

/**
 * virtual process function that is invoked when the callback
 * queue is executed.
 */
void
RubyExitCallback::process()
{
    std::ostream *os = simout.create(stats_filename);
    RubySystem::printConfig(*os);
    *os << endl;
    RubySystem::printStats(*os);
}
