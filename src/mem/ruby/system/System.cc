
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
 * RubySystem.C
 *
 * Description: See RubySystem.h
 *
 * $Id$
 *
 */


#include "System.hh"
#include "Profiler.hh"
#include "Network.hh"
#include "Tester.hh"
#include "SyntheticDriver.hh"
#include "DeterministicDriver.hh"
#include "Chip.hh"
//#include "Tracer.hh"
#include "Protocol.hh"
//#include "XactIsolationChecker.hh"  // gem5:Arka for decomissioning of log_tm
//#include "XactCommitArbiter.hh"
//#include "XactVisualizer.hh"
#include "M5Driver.hh"

RubySystem::RubySystem()
{
  DEBUG_MSG(SYSTEM_COMP, MedPrio,"initializing");

  m_driver_ptr = NULL;
  m_profiler_ptr = new Profiler;

  // NETWORK INITIALIZATION
  // create the network by calling a function that calls new
  m_network_ptr = Network::createNetwork(RubyConfig::numberOfChips());

  DEBUG_MSG(SYSTEM_COMP, MedPrio,"Constructed network");

  // CHIP INITIALIZATION
  m_chip_vector.setSize(RubyConfig::numberOfChips());// create the vector of pointers to processors
  for(int i=0; i<RubyConfig::numberOfChips(); i++) { // for each chip
    // create the chip
    m_chip_vector[i] = new Chip(i, m_network_ptr);
    DEBUG_MSG(SYSTEM_COMP, MedPrio,"Constructed a chip");
  }

  // These must be after the chips are constructed

#if 0
  if (!g_SIMICS) {
    if (g_SYNTHETIC_DRIVER && !g_DETERMINISTIC_DRIVER) {
      m_driver_ptr = new SyntheticDriver(this);
    } else if (!g_SYNTHETIC_DRIVER && g_DETERMINISTIC_DRIVER) {
      m_driver_ptr = new DeterministicDriver(this);
    } else if (g_SYNTHETIC_DRIVER && g_DETERMINISTIC_DRIVER) {
      ERROR_MSG("SYNTHETIC and DETERMINISTIC DRIVERS are exclusive and cannot be both enabled");
    } else {
      // normally make tester object, otherwise make an opal interface object.
      if (!OpalInterface::isOpalLoaded()) {
        m_driver_ptr = new Tester(this);
      } else {
        m_driver_ptr = new OpalInterface(this);
      }
    }
  } else {
    // detect if opal is loaded or not
    if (OpalInterface::isOpalLoaded()) {
      m_driver_ptr = new OpalInterface(this);
    } else {
      assert(0);
      /* Need to allocate a driver here */
      // m_driver_ptr = new SimicsDriver(this);
    }
  }
#endif

    if (g_SYNTHETIC_DRIVER && !g_DETERMINISTIC_DRIVER) {
      cerr << "Creating Synthetic Driver" << endl;
      m_driver_ptr = new SyntheticDriver(this);
    } else if (!g_SYNTHETIC_DRIVER && g_DETERMINISTIC_DRIVER) {
      cerr << "Creating Deterministic Driver" << endl;
      m_driver_ptr = new DeterministicDriver(this);
    } else {
      cerr << "Creating M5 Driver" << endl;
      m_driver_ptr = new M5Driver(this);
    }
 /*  gem5:Binkert for decomissiong of tracer
     m_tracer_ptr = new Tracer;
 */

 /*  gem5:Arka for decomissiong of log_tm
  if (XACT_MEMORY) {
    m_xact_isolation_checker = new XactIsolationChecker;
    m_xact_commit_arbiter    = new XactCommitArbiter;
    m_xact_visualizer        = new XactVisualizer;
  }
*/
  DEBUG_MSG(SYSTEM_COMP, MedPrio,"finished initializing");
  DEBUG_NEWLINE(SYSTEM_COMP, MedPrio);

}

RubySystem::~RubySystem()
{
  for (int i = 0; i < m_chip_vector.size(); i++) {
    delete m_chip_vector[i];
  }
  delete m_driver_ptr;
  delete m_network_ptr;
  delete m_profiler_ptr;
 /*  gem5:Binkert for decomissiong of tracer
     delete m_tracer_ptr;
 */
}

void RubySystem::printConfig(ostream& out) const
{
  out << "\n================ Begin RubySystem Configuration Print ================\n\n";
  RubyConfig::printConfiguration(out);
  out << endl;
  getChip(0)->printConfig(out);
  m_network_ptr->printConfig(out);
  m_driver_ptr->printConfig(out);
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
  for(int i=0; i<RubyConfig::numberOfChips(); i++) { // for each chip
    for(int p=0; p<RubyConfig::numberOfProcsPerChip(); p++) {
      m_chip_vector[i]->m_L1Cache_mandatoryQueue_vec[p]->printStats(out);
    }
  }
  m_network_ptr->printStats(out);
  m_driver_ptr->printStats(out);
  Chip::printStats(out);
}

void RubySystem::clearStats() const
{
  m_profiler_ptr->clearStats();
  m_network_ptr->clearStats();
  m_driver_ptr->clearStats();
  Chip::clearStats();
  for(int i=0; i<RubyConfig::numberOfChips(); i++) { // for each chip
    for(int p=0; p<RubyConfig::numberOfProcsPerChip(); p++) {
      m_chip_vector[i]->m_L1Cache_mandatoryQueue_vec[p]->clearStats();
    }
  }
}

void RubySystem::recordCacheContents(CacheRecorder& tr) const
{
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
}
#endif




