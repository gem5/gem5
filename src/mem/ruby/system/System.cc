
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
 * RubySystem.cc
 *
 * Description: See System.hh
 *
 * $Id$
 *
 */


#include "mem/ruby/system/System.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/profiler/Profiler.hh"
#include "mem/ruby/network/Network.hh"
#include "mem/ruby/recorder/Tracer.hh"
#include "mem/protocol/Protocol.hh"
#include "mem/ruby/buffers/MessageBuffer.hh"
#include "mem/ruby/system/Sequencer.hh"
#include "mem/ruby/system/DMASequencer.hh"
#include "mem/ruby/system/MemoryVector.hh"
#include "mem/protocol/ControllerFactory.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include "mem/ruby/system/CacheMemory.hh"
#include "mem/ruby/system/DirectoryMemory.hh"
#include "mem/ruby/network/simple/Topology.hh"
#include "mem/ruby/network/simple/SimpleNetwork.hh"
#include "mem/ruby/system/RubyPort.hh"
#include "mem/ruby/network/garnet-flexible-pipeline/GarnetNetwork.hh"
#include "mem/ruby/network/garnet-fixed-pipeline/GarnetNetwork_d.hh"
#include "mem/ruby/system/MemoryControl.hh"

int RubySystem::m_random_seed;
bool RubySystem::m_randomization;
int RubySystem::m_tech_nm;
int RubySystem::m_freq_mhz;
int RubySystem::m_block_size_bytes;
int RubySystem::m_block_size_bits;
uint64 RubySystem::m_memory_size_bytes;
int RubySystem::m_memory_size_bits;

map< string, RubyPort* > RubySystem::m_ports;
map< string, CacheMemory* > RubySystem::m_caches;
map< string, DirectoryMemory* > RubySystem::m_directories;
map< string, Sequencer* > RubySystem::m_sequencers;
map< string, DMASequencer* > RubySystem::m_dma_sequencers;
map< string, AbstractController* > RubySystem::m_controllers;
map< string, MemoryControl* > RubySystem::m_memorycontrols;


Network* RubySystem::m_network_ptr;
map< string, Topology*> RubySystem::m_topologies;
Profiler* RubySystem::m_profiler_ptr;
Tracer* RubySystem::m_tracer_ptr;

MemoryVector* RubySystem::m_mem_vec_ptr;


RubySystem* RubySystem::create(const vector <RubyObjConf> & sys_conf)
{
  if (g_system_ptr == NULL)
    return new RubySystem(sys_conf);
  return g_system_ptr;
}

void RubySystem::init(const vector<string> & argv)
{
  for (size_t i=0; i < argv.size(); i+=2) {
    if (argv[i] == "random_seed") {
      m_random_seed = atoi(argv[i+1].c_str());
      srandom(m_random_seed);
    } else if (argv[i] == "randomization") {
      m_randomization = string_to_bool(argv[i+1]);
    } else if (argv[i] == "tech_nm") {
      m_tech_nm = atoi(argv[i+1].c_str());
    } else if (argv[i] == "freq_mhz") {
      m_freq_mhz = atoi(argv[i+1].c_str());
    } else if (argv[i] == "block_size_bytes") {
      m_block_size_bytes = atoi(argv[i+1].c_str());
      assert(is_power_of_2(m_block_size_bytes));
      m_block_size_bits = log_int(m_block_size_bytes);
    } else if (argv[i] == "debug") {

    } else if (argv[i] == "tracer") {

    } else if (argv[i] == "profiler") {

  //  } else if (argv[i] == "MI_example") {

    } else {
      cerr << "Error: Unknown RubySystem config parameter -- " << argv[i] << endl;
     assert(0);
    }
  }
}

RubySystem::RubySystem(const vector <RubyObjConf> & sys_conf)
{
  //  DEBUG_MSG(SYSTEM_COMP, MedPrio,"initializing");

  for (size_t i=0;i<sys_conf.size(); i++) {
    const string & type = sys_conf[i].type;
    const string & name = sys_conf[i].name;
    const vector<string> & argv = sys_conf[i].argv;
    if (type == "System") {
      init(argv);  // initialize system-wide variables before doing anything else!
    } else if (type == "Debug") {
      g_debug_ptr = new Debug(name, argv);
    }
  }

  assert( g_debug_ptr != NULL);
  g_eventQueue_ptr = new RubyEventQueue;
  g_system_ptr = this;
  m_mem_vec_ptr = new MemoryVector;

  /* object contruction is broken into two steps (Constructor and init) to avoid cyclic dependencies
   *  e.g. a sequencer needs a pointer to a controller and a controller needs a pointer to a sequencer
   */

  vector<string> memory_control_names;

  for (size_t i=0;i<sys_conf.size(); i++) {
    const string & type = sys_conf[i].type;
    const string & name = sys_conf[i].name;
    if (type == "System" || type == "Debug")
      continue;
    else if (type == "SetAssociativeCache")
      m_caches[name] = new CacheMemory(name);
    else if (type == "DirectoryMemory")
      m_directories[name] = new DirectoryMemory(name);
    else if (type == "Sequencer") {
      m_sequencers[name] = new Sequencer(name);
      m_ports[name] = m_sequencers[name];
    } else if (type == "DMASequencer") {
      m_dma_sequencers[name] = new DMASequencer(name);
      m_ports[name] = m_dma_sequencers[name];
    } else if (type == "Topology") {
      assert(m_topologies.size() == 0); // only one toplogy at a time is supported right now
      m_topologies[name] = new Topology(name);
    } else if (type == "SimpleNetwork") {
      assert(m_network_ptr == NULL); // only one network at a time is supported right now
      m_network_ptr = new SimpleNetwork(name);
    } else if (type.find("generated") == 0) {
      string controller_type = type.substr(10);
      m_controllers[name] = ControllerFactory::createController(controller_type, name);
//      printf ("ss: generated %s \n", controller_type);
//added by SS
    } else if (type == "Tracer") {
      //m_tracers[name] = new Tracer(name);
      m_tracer_ptr = new Tracer(name);
    } else if (type == "Profiler") {
      m_profiler_ptr = new Profiler(name);
    } else if (type == "GarnetNetwork") {
      assert(m_network_ptr == NULL); // only one network at a time is supported right now
      m_network_ptr = new GarnetNetwork(name);
    } else if (type == "GarnetNetwork_d") {
      assert(m_network_ptr == NULL); // only one network at a time is supported right now
      m_network_ptr = new GarnetNetwork_d(name);
    } else if (type == "MemoryControl") {
      m_memorycontrols[name] = new MemoryControl(name);
      memory_control_names.push_back (name);
    } else {
      cerr << "Error: Unknown object type -- " << type << endl;
      assert(0);
    }
  }

  for (size_t i=0;i<sys_conf.size(); i++) {
    string type = sys_conf[i].type;
    string name = sys_conf[i].name;
    const vector<string> & argv = sys_conf[i].argv;
    if (type == "Topology")
      m_topologies[name]->init(argv);
  }

  for (size_t i=0;i<sys_conf.size(); i++) {
    string type = sys_conf[i].type;
    string name = sys_conf[i].name;
    const vector<string> & argv = sys_conf[i].argv;
    if (type == "SimpleNetwork" || type == "GarnetNetwork" || type == "GarnetNetwork_d"){
      m_network_ptr->init(argv);
    }
  }

  for (size_t i=0;i<sys_conf.size(); i++) {
    string type = sys_conf[i].type;
    string name = sys_conf[i].name;
    const vector<string> & argv = sys_conf[i].argv;
    if (type == "MemoryControl" ){
      m_memorycontrols[name]->init(argv);
    }
  }

  for (size_t i=0;i<sys_conf.size(); i++) {
    string type = sys_conf[i].type;
    string name = sys_conf[i].name;
    const vector<string> & argv = sys_conf[i].argv;
    if (type == "System" || type == "Debug")
      continue;
    else if (type == "SetAssociativeCache")
      m_caches[name]->init(argv);
    else if (type == "DirectoryMemory")
      m_directories[name]->init(argv);
    else if (type == "MemoryControl")
      continue;
    else if (type == "Sequencer")
      m_sequencers[name]->init(argv);
    else if (type == "DMASequencer")
      m_dma_sequencers[name]->init(argv);
    else if (type == "Topology")
      continue;
    else if (type == "SimpleNetwork" || type == "GarnetNetwork" || type == "GarnetNetwork_d")
      continue;
    else if (type.find("generated") == 0) {
      string controller_type = type.substr(11);
      m_controllers[name]->init(m_network_ptr, argv);
    }
//added by SS
    else if (type == "Tracer")
      //m_tracers[name]->init(argv);
      m_tracer_ptr->init(argv);
    else if (type == "Profiler")
      m_profiler_ptr->init(argv, memory_control_names);
//    else if (type == "MI_example"){
//    }
    else
      assert(0);
  }

//  m_profiler_ptr = new Profiler;

  // calculate system-wide parameters
  m_memory_size_bytes = 0;
  DirectoryMemory* prev = NULL;
  for (map< string, DirectoryMemory*>::const_iterator it = m_directories.begin();
       it != m_directories.end(); it++) {
    if (prev != NULL)
      assert((*it).second->getSize() == prev->getSize()); // must be equal for proper address mapping
    m_memory_size_bytes += (*it).second->getSize();
    prev = (*it).second;
  }
  m_mem_vec_ptr->setSize(m_memory_size_bytes);
  m_memory_size_bits = log_int(m_memory_size_bytes);

//  m_tracer_ptr = new Tracer;
  DEBUG_MSG(SYSTEM_COMP, MedPrio,"finished initializing");
  DEBUG_NEWLINE(SYSTEM_COMP, MedPrio);
}

RubySystem::~RubySystem()
{
  /*
  for (int i=0; i < MachineType_base_level(MachineType_NUM); i++) {
    for (int j=0; j < RubyConfig::getNumberOfControllersPerType(i); j++ ) {
      delete m_controllers[i][j];
    }
  }
  delete m_network_ptr;
  delete m_profiler_ptr;
  delete m_tracer_ptr;
  */
}

void RubySystem::printSystemConfig(ostream & out)
{
  out << "RubySystem config:" << endl;
  out << "  random_seed: " << m_random_seed << endl;
  out << "  randomization: " << m_randomization << endl;
  out << "  tech_nm: " << m_tech_nm << endl;
  out << "  freq_mhz: " << m_freq_mhz << endl;
  out << "  block_size_bytes: " << m_block_size_bytes << endl;
  out << "  block_size_bits: " << m_block_size_bits << endl;
  out << "  memory_size_bytes: " << m_memory_size_bytes << endl;
  out << "  memory_size_bits: " << m_memory_size_bits << endl;

}

void RubySystem::printConfig(ostream& out)
{
  out << "\n================ Begin RubySystem Configuration Print ================\n\n";
  //  RubyConfig::printConfiguration(out);
  //  out << endl;
  printSystemConfig(out);
  for (map<string, AbstractController*>::const_iterator it = m_controllers.begin();
       it != m_controllers.end(); it++) {
    (*it).second->printConfig(out);
  }
  for (map<string, CacheMemory*>::const_iterator it = m_caches.begin();
       it != m_caches.end(); it++) {
    (*it).second->printConfig(out);
  }
  DirectoryMemory::printGlobalConfig(out);
  for (map<string, DirectoryMemory*>::const_iterator it = m_directories.begin();
       it != m_directories.end(); it++) {
    (*it).second->printConfig(out);
  }
  for (map<string, Sequencer*>::const_iterator it = m_sequencers.begin();
       it != m_sequencers.end(); it++) {
    (*it).second->printConfig(out);
  }

  m_network_ptr->printConfig(out);
  m_profiler_ptr->printConfig(out);

  out << "\n================ End RubySystem Configuration Print ================\n\n";
}

void RubySystem::printStats(ostream& out)
{

  const time_t T = time(NULL);
  tm *localTime = localtime(&T);
  char buf[100];
  strftime(buf, 100, "%b/%d/%Y %H:%M:%S", localTime);

  out << "Real time: " << buf << endl;

  m_profiler_ptr->printStats(out);
  m_network_ptr->printStats(out);
  for (map<string, AbstractController*>::const_iterator it = m_controllers.begin();
       it != m_controllers.end(); it++) {
    (*it).second->printStats(out);
  }
}

void RubySystem::clearStats() const
{
  /*
  m_profiler_ptr->clearStats();
  for (int i=0; i<m_rubyRequestQueues.size(); i++)
    for (int j=0;j<m_rubyRequestQueues[i].size(); j++)
      m_rubyRequestQueues[i][j]->clearStats();
  m_network_ptr->clearStats();
  for (int i=0; i < MachineType_base_level(MachineType_NUM); i++)
    m_controllers[i][0]->clearStats();
  */
}

void RubySystem::recordCacheContents(CacheRecorder& tr) const
{
  /*
    for (int i = 0; i < m_chip_vector.size(); i++) {
    for (int m_version = 0; m_version < RubyConfig::numberOfProcsPerChip(); m_version++) {
      if (Protocol::m_TwoLevelCache) {
        m_chip_vector[i]->m_L1Cache_L1IcacheMemory_vec[m_version]->setAsInstructionCache(true);
        m_chip_vector[i]->m_L1Cache_L1DcacheMemory_vec[m_version]->setAsInstructionCache(false);
      } else {
        m_chip_vector[i]->m_L1Cache_cacheMemory_vec[m_version]->setAsInstructionCache(false);
      }
    }
    m_chip_vector[i]->recordCacheContents(tr);
  }
  */
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
void RubySystem::checkGlobalCoherenceInvariant(const Address& addr  )  {
  /*
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
      }
      else if (sharedDetected) {
        WARN_EXPR(lastShared);
        WARN_EXPR(m_chip_vector[i]->getID());
        WARN_EXPR(addr);
        WARN_EXPR(g_eventQueue_ptr->getTime());
        ERROR_MSG("Coherence Violation Detected -- exclusive chip with >=1 shared");
      }
      else {
        exclusive = m_chip_vector[i]->getID();
      }
    }
    else if (m_chip_vector[i]->isBlockShared(addr)) {
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
  */
}
#endif




