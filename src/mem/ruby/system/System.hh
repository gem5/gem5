/*
 * Copyright (c) 1999-2012 Mark D. Hill and David A. Wood
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
#include "mem/ruby/recorder/CacheRecorder.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include "mem/ruby/system/MemoryVector.hh"
#include "mem/ruby/system/SparseMemory.hh"
#include "params/RubySystem.hh"
#include "sim/sim_object.hh"

class Network;
class Profiler;

class RubySystem : public SimObject
{
  public:
    class RubyEvent : public Event
    {
      public:
        RubyEvent(RubySystem* _ruby_system)
        {
            ruby_system = _ruby_system;
        }
      private:
        void process();

        RubySystem* ruby_system;
    };

    friend class RubyEvent;

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

    static MemoryVector*
    getMemoryVector()
    {
        assert(m_mem_vec_ptr != NULL);
        return m_mem_vec_ptr;
    }

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

    void serialize(std::ostream &os);
    void unserialize(Checkpoint *cp, const std::string &section);
    void process();
    void startup();

    void registerNetwork(Network*);
    void registerProfiler(Profiler*);
    void registerAbstractController(AbstractController*);
    void registerSparseMemory(SparseMemory*);

    bool eventQueueEmpty() { return eventq->empty(); }
    void enqueueRubyEvent(Tick tick)
    {
        RubyEvent* e = new RubyEvent(this);
        schedule(e, tick);
    }

  private:
    // Private copy constructor and assignment operator
    RubySystem(const RubySystem& obj);
    RubySystem& operator=(const RubySystem& obj);

    void init();

    static void printSystemConfig(std::ostream& out);
    void readCompressedTrace(std::string filename,
                             uint8*& raw_data,
                             uint64& uncompressed_trace_size);
    void writeCompressedTrace(uint8* raw_data, std::string file,
                              uint64 uncompressed_trace_size);

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
    static MemoryVector* m_mem_vec_ptr;
    std::vector<AbstractController*> m_abs_cntrl_vec;
    bool m_warmup_enabled;
    bool m_cooldown_enabled;
    CacheRecorder* m_cache_recorder;
    std::vector<SparseMemory*> m_sparse_memory_vector;
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
