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

/*
 * Contains all of the various parts of the system we are simulating.
 * Performs allocation, deallocation, and setup of all the major
 * components of the system
 */

#ifndef __MEM_RUBY_SYSTEM_SYSTEM_HH__
#define __MEM_RUBY_SYSTEM_SYSTEM_HH__

#include "base/callback.hh"
#include "mem/ruby/common/Global.hh"
#include "mem/ruby/eventqueue/RubyEventQueue.hh"
#include "mem/ruby/system/RubyPort.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include "params/RubySystem.hh"
#include "sim/sim_object.hh"

class AbstractController;
class CacheRecorder;
class MemoryVector;
class Network;
class Profiler;
class Tracer;

/*
 * This defines the number of longs (32-bits on 32 bit machines,
 * 64-bit on 64-bit AMD machines) to use to hold the set...
 * the default is 4, allowing 128 or 256 different members
 * of the set.
 *
 * This should never need to be changed for correctness reasons,
 * though increasing it will increase performance for larger
 * set sizes at the cost of a (much) larger memory footprint
 *
 */
const int NUMBER_WORDS_PER_SET = 1;

class RubySystem : public SimObject
{
  public:
    typedef RubySystemParams Params;
    RubySystem(const Params *p);
    ~RubySystem();

    // config accessors
    static int getRandomSeed() { return m_random_seed; }
    static int getRandomization() { return m_randomization; }
    static int getBlockSizeBytes() { return m_block_size_bytes; }
    static int getBlockSizeBits() { return m_block_size_bits; }
    static uint64 getMemorySizeBytes() { return m_memory_size_bytes; }
    static int getMemorySizeBits() { return m_memory_size_bits; }

    // Public Methods
    static Network*
    getNetwork()
    {
        assert(m_network_ptr != NULL);
        return m_network_ptr;
    }

    static RubyEventQueue*
    getEventQueue()
    {
        return g_eventQueue_ptr;
    }

    Profiler*
    getProfiler()
    {
        assert(m_profiler_ptr != NULL);
        return m_profiler_ptr;
    }

    static Tracer*
    getTracer()
    {
        assert(m_tracer_ptr != NULL);
        return m_tracer_ptr;
    }

    static MemoryVector*
    getMemoryVector()
    {
        assert(m_mem_vec_ptr != NULL);
        return m_mem_vec_ptr;
    }

    void recordCacheContents(CacheRecorder& tr) const;
    static void printConfig(std::ostream& out);
    static void printStats(std::ostream& out);
    void clearStats() const;

    uint64 getInstructionCount(int thread) { return 1; }
    static uint64
    getCycleCount(int thread)
    {
        return g_eventQueue_ptr->getTime();
    }

    void print(std::ostream& out) const;

    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);

    void registerNetwork(Network*);
    void registerProfiler(Profiler*);
    void registerTracer(Tracer*);
    void registerAbstractController(AbstractController*);

  private:
    // Private copy constructor and assignment operator
    RubySystem(const RubySystem& obj);
    RubySystem& operator=(const RubySystem& obj);

    void init();

    static void printSystemConfig(std::ostream& out);

  private:
    // configuration parameters
    static int m_random_seed;
    static bool m_randomization;
    static Tick m_clock;
    static int m_block_size_bytes;
    static int m_block_size_bits;
    static uint64 m_memory_size_bytes;
    static int m_memory_size_bits;

    static Network* m_network_ptr;

  public:
    static Profiler* m_profiler_ptr;
    static Tracer* m_tracer_ptr;
    static MemoryVector* m_mem_vec_ptr;
    std::vector<AbstractController*> m_abs_cntrl_vec;
};

inline std::ostream&
operator<<(std::ostream& out, const RubySystem& obj)
{
    //obj.print(out);
    out << std::flush;
    return out;
}

class RubyExitCallback : public Callback
{
  private:
    std::string stats_filename;

  public:
    virtual ~RubyExitCallback() {}

    RubyExitCallback(const std::string& _stats_filename)
    {
        stats_filename = _stats_filename;
    }

    virtual void process();
};

#endif // __MEM_RUBY_SYSTEM_SYSTEM_HH__
