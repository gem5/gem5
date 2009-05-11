
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
 * RubySystem.h
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

#include "mem/ruby/common/Global.hh"
#include "mem/gems_common/Vector.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/config/RubyConfig.hh"
#include "mem/protocol/MachineType.hh"
#include "mem/ruby/slicc_interface/AbstractChip.hh"

class Profiler;
class Network;
class Driver;
class CacheRecorder;
class Tracer;
class Sequencer;
class XactIsolationChecker;
class XactCommitArbiter;
class XactVisualizer;
class TransactionInterfaceManager;

class RubySystem {
public:
  // Constructors
  RubySystem();

  // Destructor
  ~RubySystem();

  // Public Methods
  int getNumProcessors() { return RubyConfig::numberOfProcessors(); }
  int getNumMemories() { return RubyConfig::numberOfMemories(); }
  Profiler* getProfiler() { return m_profiler_ptr; }
  Driver* getDriver() { assert(m_driver_ptr != NULL); return m_driver_ptr; }
  Tracer* getTracer() { assert(m_tracer_ptr != NULL); return m_tracer_ptr; }
  Network* getNetwork() { assert(m_network_ptr != NULL); return m_network_ptr; }
  XactIsolationChecker* getXactIsolationChecker() { assert(m_xact_isolation_checker!= NULL); return m_xact_isolation_checker;}
  XactCommitArbiter* getXactCommitArbiter() { assert(m_xact_commit_arbiter!= NULL); return m_xact_commit_arbiter;}
  XactVisualizer*    getXactVisualizer() { assert(m_xact_visualizer!= NULL); return m_xact_visualizer;}

  AbstractChip* getChip(int chipNumber) const { assert(m_chip_vector[chipNumber] != NULL); return m_chip_vector[chipNumber];}
  Sequencer* getSequencer(int procNumber) const {
    assert(procNumber < RubyConfig::numberOfProcessors());
    return m_chip_vector[procNumber/RubyConfig::numberOfProcsPerChip()]->getSequencer(procNumber%RubyConfig::numberOfProcsPerChip());
  }
  TransactionInterfaceManager* getTransactionInterfaceManager(int procNumber) const {
    return m_chip_vector[procNumber/RubyConfig::numberOfProcsPerChip()]->getTransactionInterfaceManager(procNumber%RubyConfig::numberOfProcsPerChip());
  }
  void recordCacheContents(CacheRecorder& tr) const;
  void printConfig(ostream& out) const;
  void printStats(ostream& out);
  void clearStats() const;

  void print(ostream& out) const;
#ifdef CHECK_COHERENCE
  void checkGlobalCoherenceInvariant(const Address& addr);
#endif

private:
  // Private Methods

  // Private copy constructor and assignment operator
  RubySystem(const RubySystem& obj);
  RubySystem& operator=(const RubySystem& obj);

  // Data Members (m_ prefix)
  Network* m_network_ptr;
  Vector<AbstractChip*> m_chip_vector;
  Profiler* m_profiler_ptr;
  Driver* m_driver_ptr;
  Tracer* m_tracer_ptr;
  XactIsolationChecker *m_xact_isolation_checker;
  XactCommitArbiter    *m_xact_commit_arbiter;
  XactVisualizer       *m_xact_visualizer;
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

#endif //SYSTEM_H



