
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
 * System.hh
 *
 * Description: Contains all of the various parts of the system we are
 * simulating.  Performs allocation, deallocation, and setup of all
 * the major components of the system
 *
 * $Id$
 *
 */

#ifndef SYSTEM_H
#define SYSTEM_H

#include "mem/ruby/system/RubyPort.hh"
#include "mem/ruby/common/Global.hh"
#include "mem/gems_common/Vector.hh"
#include "mem/ruby/eventqueue/RubyEventQueue.hh"
#include <map>
#include "sim/sim_object.hh"
#include "params/RubySystem.hh"
#include "base/callback.hh"

class Profiler;
class Network;
class CacheRecorder;
class Tracer;
class Sequencer;
class DMASequencer;
class MemoryVector;
class AbstractController;
class MessageBuffer;
class CacheMemory;
class DirectoryMemory;
class Topology;
class MemoryControl;

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
const int NUMBER_WORDS_PER_SET = 4;


struct RubyObjConf {
  string type;
  string name;
  vector<string> argv;
  RubyObjConf(string _type, string _name, vector<string> _argv)
    : type(_type), name(_name), argv(_argv)
  {}
};

class RubySystem : public SimObject {
public:
    typedef RubySystemParams Params;
    RubySystem(const Params *p);
  // Destructor
  ~RubySystem();

  // config accessors
  static int getRandomSeed() { return m_random_seed; }
  static int getRandomization() { return m_randomization; }
  static int getBlockSizeBytes() { return m_block_size_bytes; }
  static int getBlockSizeBits() { return m_block_size_bits; }
  static uint64 getMemorySizeBytes() { return m_memory_size_bytes; }
  static int getMemorySizeBits() { return m_memory_size_bits; }

  // Public Methods
  static RubyPort* getPortOnly(const string & name) {
    assert(m_ports.count(name) == 1); return m_ports[name]; }
  static RubyPort* getPort(const string & name, void (*hit_callback)(int64_t)) {
    if (m_ports.count(name) != 1){
      cerr << "Port " << name << " has " << m_ports.count(name) << " instances" << endl;
    }
    assert(m_ports.count(name) == 1); 
    m_ports[name]->registerHitCallback(hit_callback); 
    return m_ports[name]; 
  }
  static Network* getNetwork() { assert(m_network_ptr != NULL); return m_network_ptr; }
  static Topology* getTopology(const string & name) { assert(m_topologies.count(name) == 1); return m_topologies[name]; }
  static CacheMemory* getCache(const string & name) { assert(m_caches.count(name) == 1); return m_caches[name]; }
  static DirectoryMemory* getDirectory(const string & name) { assert(m_directories.count(name) == 1); return m_directories[name]; }
  static MemoryControl* getMemoryControl(const string & name) { assert(m_memorycontrols.count(name) == 1); return m_memorycontrols[name]; }
  static Sequencer* getSequencer(const string & name) { assert(m_sequencers.count(name) == 1); return m_sequencers[name]; }
  static DMASequencer* getDMASequencer(const string & name) { assert(m_dma_sequencers.count(name) == 1); return m_dma_sequencers[name]; }
  static AbstractController* getController(const string & name) { assert(m_controllers.count(name) == 1); return m_controllers[name]; }

  static RubyEventQueue* getEventQueue() { return g_eventQueue_ptr; }

  Profiler* getProfiler() {assert(m_profiler_ptr != NULL); return m_profiler_ptr; }
  static Tracer* getTracer() { assert(m_tracer_ptr != NULL); return m_tracer_ptr; }
  static MemoryVector* getMemoryVector() { assert(m_mem_vec_ptr != NULL); return m_mem_vec_ptr;}

  void recordCacheContents(CacheRecorder& tr) const;
  static void printConfig(ostream& out);
  static void printStats(ostream& out);
  void clearStats() const;

  uint64 getInstructionCount(int thread) { return 1; }
  static uint64 getCycleCount(int thread) { return g_eventQueue_ptr->getTime(); }

  void print(ostream& out) const;
  /*
#ifdef CHECK_COHERENCE
  void checkGlobalCoherenceInvariant(const Address& addr);
#endif
  */

private:
  // Constructors
  RubySystem(const vector <RubyObjConf> & cfg_file);

  // Private copy constructor and assignment operator
  RubySystem(const RubySystem& obj);
  RubySystem& operator=(const RubySystem& obj);

  void init();

  static void printSystemConfig(ostream& out);

private:
  // configuration parameters
  static int m_random_seed;
  static bool m_randomization;
  static Tick m_clock;
  static int m_block_size_bytes;
  static int m_block_size_bits;
  static uint64 m_memory_size_bytes;
  static int m_memory_size_bits;

  // Data Members (m_ prefix)
  static Network* m_network_ptr;
  static map< string, Topology* > m_topologies;
  static map< string, RubyPort* > m_ports;
  static map< string, CacheMemory* > m_caches;
  static map< string, DirectoryMemory* > m_directories;
  static map< string, Sequencer* > m_sequencers;
  static map< string, DMASequencer* > m_dma_sequencers;
  static map< string, AbstractController* > m_controllers;
  static map< string, MemoryControl* > m_memorycontrols;

  //added by SS
  //static map< string, Tracer* > m_tracers;

public:
  static Profiler* m_profiler_ptr;
  static Tracer* m_tracer_ptr;
  static MemoryVector* m_mem_vec_ptr;
};

// Output operator declaration
ostream& operator<<(ostream& out, const RubySystem& obj);

// ******************* Definitions *******************

// Output operator definition
inline
ostream& operator<<(ostream& out, const RubySystem& obj)
{
//  obj.print(out);
  out << flush;
  return out;
}

class RubyExitCallback : public Callback
{
  private:
    string stats_filename;

  public:
    /**
     * virtualize the destructor to make sure that the correct one
     * gets called.
     */

    virtual ~RubyExitCallback() {}

    RubyExitCallback(const string& _stats_filename)
    {
      stats_filename = _stats_filename;
    }

    virtual void process();
};

#endif //SYSTEM_H



