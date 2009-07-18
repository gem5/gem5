
/*
    Copyright (C) 1999-2008 by Mark D. Hill and David A. Wood for the
    Wisconsin Multifacet Project.  Contact: gems@cs.wisc.edu
    http://www.cs.wisc.edu/gems/

    --------------------------------------------------------------------

    This file is part of the Ruby Multiprocessor Memory System Simulator, 
    a component of the Multifacet GEMS (General Execution-driven 
    Multiprocessor Simulator) software toolset originally developed at 
    the University of Wisconsin-Madison.

    Ruby was originally developed primarily by Milo Martin and Daniel
    Sorin with contributions from Ross Dickson, Carl Mauer, and Manoj
    Plakal.

    Substantial further development of Multifacet GEMS at the
    University of Wisconsin was performed by Alaa Alameldeen, Brad
    Beckmann, Jayaram Bobba, Ross Dickson, Dan Gibson, Pacia Harper,
    Derek Hower, Milo Martin, Michael Marty, Carl Mauer, Michelle Moravan,
    Kevin Moore, Andrew Phelps, Manoj Plakal, Daniel Sorin, Haris Volos, 
    Min Xu, and Luke Yen.
    --------------------------------------------------------------------

    If your use of this software contributes to a published paper, we
    request that you (1) cite our summary paper that appears on our
    website (http://www.cs.wisc.edu/gems/) and (2) e-mail a citation
    for your published paper to gems@cs.wisc.edu.

    If you redistribute derivatives of this software, we request that
    you notify us and either (1) ask people to register with us at our
    website (http://www.cs.wisc.edu/gems/) or (2) collect registration
    information and periodically send it to us.

    --------------------------------------------------------------------

    Multifacet GEMS is free software; you can redistribute it and/or
    modify it under the terms of version 2 of the GNU General Public
    License as published by the Free Software Foundation.

    Multifacet GEMS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with the Multifacet GEMS; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
    02111-1307, USA

    The GNU General Public License is contained in the file LICENSE.

### END HEADER ###
*/

/*
 * $Id$
 *
 * Description: 
 *
 */

#ifndef DETERMINISTICDRIVER_H
#define DETERMINISTICDRIVER_H
#include <map>
#include "mem/ruby/common/Global.hh"
#include "mem/ruby/tester/Tester_Globals.hh"
#include "mem/ruby/common/Histogram.hh"            // includes global, but doesn't use anything, so it should be fine
#include "mem/protocol/CacheRequestType.hh"     // includes global, but doesn't use anything, so it should be fine
#include "mem/ruby/common/Address.hh"       // we redefined the address
#include "mem/ruby/tester/DetermGETXGenerator.hh"  // this is our file
#include "mem/ruby/tester/DetermSeriesGETSGenerator.hh"  // this is our file
#include "mem/ruby/tester/DetermInvGenerator.hh"  // this is our file
#include "mem/ruby/libruby.hh"
#include "mem/ruby/common/Driver.hh"
#include "mem/ruby/common/Consumer.hh"
#include "mem/ruby/eventqueue/RubyEventQueue.hh"
#include "mem/protocol/SpecifiedGeneratorType.hh"

//class DMAGenerator;

class DeterministicDriver : public Driver, public Consumer {
public:
  friend class DetermGETXGenerator;
  friend class DetermSeriesGETSGenerator;
  friend class DetermInvGenerator;
  // Constructors
  DeterministicDriver(string generator_type, int num_completions, int num_procs, Time g_think_time, Time g_wait_time, int g_tester_length);

  // Destructor
  ~DeterministicDriver();
  
  // Public Methods
  void go();
  bool isStoreReady(NodeID node);
  bool isLoadReady(NodeID node);
  bool isStoreReady(NodeID node, Address addr);
  bool isLoadReady(NodeID node, Address addr);
  void loadCompleted(NodeID node, Address addr);
  void storeCompleted(NodeID node, Address addr);
  Address getNextLoadAddr(NodeID node);
  Address getNextStoreAddr(NodeID node);
  int getLoadsCompleted() { return m_loads_completed; }
  int getStoresCompleted() { return m_stores_completed; }

  void reportDone();
  void recordLoadLatency(Time time);
  void recordStoreLatency(Time time);

//  void dmaHitCallback();
  void hitCallback(int64_t request_id);
  void wakeup();
  void printStats(ostream& out) const;
  void clearStats() {}
  void printConfig(ostream& out) const {}

  void print(ostream& out) const;
  // Public copy constructor and assignment operator
  DeterministicDriver(const DeterministicDriver& obj);
  DeterministicDriver& operator=(const DeterministicDriver& obj);

private:
  // Private Methods

  Address getNextAddr(NodeID node, Vector<NodeID> addr_vector);
  bool isAddrReady(NodeID node, Vector<NodeID> addr_vector);
  bool isAddrReady(NodeID node, Vector<NodeID> addr_vector, Address addr);
  void setNextAddr(NodeID node, Address addr, Vector<NodeID>& addr_vector);

  
  // Data Members (m_ prefix)
  Vector<Time> m_last_progress_vector;
  Vector<SpecifiedGenerator*> m_generator_vector;
  //DMAGenerator* m_dma_generator;
  Vector<NodeID> m_load_vector;  // Processor last to load the addr
  Vector<NodeID> m_store_vector;  // Processor last to store the addr

  int last_proc;
  int m_done_counter;
  int m_loads_completed;
  int m_stores_completed;
  // enforces the previous node to have a certain # of completions
  // before next node starts
  
  map <int64_t, pair <int, Address> > requests;
  Time m_think_time;
  Time m_wait_time;
  int m_tester_length;
  int m_num_procs;
  RubyEventQueue * eventQueue;
  int m_numCompletionsPerNode;  

  Histogram m_load_latency;
  Histogram m_store_latency;
  Time m_finish_time;
  Time m_last_issue;
};

// Output operator declaration
ostream& operator<<(ostream& out, const DeterministicDriver& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline 
ostream& operator<<(ostream& out, const DeterministicDriver& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //DETERMINISTICDRIVER_H
