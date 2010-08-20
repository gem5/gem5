/*
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
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

#include "base/intmath.hh"
#include "base/output.hh"
#include "mem/ruby/buffers/MessageBuffer.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/network/Network.hh"
#include "mem/ruby/profiler/Profiler.hh"
#include "mem/ruby/recorder/Tracer.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include "mem/ruby/system/MemoryVector.hh"
#include "mem/ruby/system/System.hh"

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
Tracer* RubySystem::m_tracer_ptr;
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

    m_network_ptr = p->network;
    g_debug_ptr = p->debug;
    m_profiler_ptr = p->profiler;
    m_tracer_ptr = p->tracer;

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
}

void
RubySystem::init()
{
    m_profiler_ptr->clearStats();
}

RubySystem::~RubySystem()
{
    delete m_network_ptr;
    delete m_profiler_ptr;
    delete m_tracer_ptr;
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
RubySystem::serialize(std::ostream &os)
{

}

void
RubySystem::unserialize(Checkpoint *cp, const string &section)
{
    //
    // The main purpose for clearing stats in the unserialize process is so
    // that the profiler can correctly set its start time to the unserialized
    // value of curTick
    //
    clearStats();
}

void
RubySystem::clearStats() const
{
    m_profiler_ptr->clearStats();
    m_network_ptr->clearStats();
}

void
RubySystem::recordCacheContents(CacheRecorder& tr) const
{
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
