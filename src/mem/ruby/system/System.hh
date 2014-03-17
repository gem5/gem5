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
#include "base/output.hh"
#include "mem/packet.hh"
#include "mem/ruby/profiler/Profiler.hh"
#include "mem/ruby/recorder/CacheRecorder.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include "mem/ruby/system/MemoryControl.hh"
#include "mem/ruby/system/MemoryVector.hh"
#include "mem/ruby/system/SparseMemory.hh"
#include "params/RubySystem.hh"
#include "sim/clocked_object.hh"

class Network;

class RubySystem : public ClockedObject
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
    static uint32_t getBlockSizeBytes() { return m_block_size_bytes; }
    static uint32_t getBlockSizeBits() { return m_block_size_bits; }
    static uint64_t getMemorySizeBytes() { return m_memory_size_bytes; }
    static uint32_t getMemorySizeBits() { return m_memory_size_bits; }

    // Public Methods
    Profiler*
    getProfiler()
    {
        assert(m_profiler != NULL);
        return m_profiler;
    }

    MemoryVector*
    getMemoryVector()
    {
        assert(m_mem_vec != NULL);
        return m_mem_vec;
    }

    void regStats() { m_profiler->regStats(name()); }
    void collateStats() { m_profiler->collateStats(); }
    void resetStats();

    void serialize(std::ostream &os);
    void unserialize(Checkpoint *cp, const std::string &section);
    void process();
    void startup();
    bool functionalRead(Packet *ptr);
    bool functionalWrite(Packet *ptr);

    void registerNetwork(Network*);
    void registerAbstractController(AbstractController*);
    void registerSparseMemory(SparseMemory*);
    void registerMemController(MemoryControl *mc);

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

    void readCompressedTrace(std::string filename,
                             uint8_t *&raw_data,
                             uint64& uncompressed_trace_size);
    void writeCompressedTrace(uint8_t *raw_data, std::string file,
                              uint64 uncompressed_trace_size);

  private:
    // configuration parameters
    static int m_random_seed;
    static bool m_randomization;
    static uint32_t m_block_size_bytes;
    static uint32_t m_block_size_bits;
    static uint64_t m_memory_size_bytes;
    static uint32_t m_memory_size_bits;

    Network* m_network;
    std::vector<MemoryControl *> m_memory_controller_vec;
    std::vector<AbstractController *> m_abs_cntrl_vec;

  public:
    Profiler* m_profiler;
    MemoryVector* m_mem_vec;
    bool m_warmup_enabled;
    bool m_cooldown_enabled;
    CacheRecorder* m_cache_recorder;
    std::vector<SparseMemory*> m_sparse_memory_vector;
};

class RubyStatsCallback : public Callback
{
  private:
    RubySystem *ruby_system;

  public:
    virtual ~RubyStatsCallback() {}
    RubyStatsCallback(RubySystem *system) : ruby_system(system) {}
    void process() { ruby_system->collateStats(); }
};

#endif // __MEM_RUBY_SYSTEM_SYSTEM_HH__
