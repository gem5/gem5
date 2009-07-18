
/*
    Copyright (C) 1999-2005 by Mark D. Hill and David A. Wood for the
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
    Beckmann, Ross Dickson, Pacia Harper, Milo Martin, Michael Marty,
    Carl Mauer, Kevin Moore, Manoj Plakal, Daniel Sorin, Min Xu, and
    Luke Yen.

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
 * Description: The driver interface between racey pseudo threads and ruby
 *              memory timing simulator.
 *
 */

#ifndef RACEYDRIVER_H
#define RACEYDRIVER_H

#include "mem/ruby/common/Global.hh"
#include "mem/ruby/tester/Tester_Globals.hh"
#include "mem/ruby/common/Driver.hh"
#include "mem/ruby/tester/RaceyPseudoThread.hh"
#include <map>
#include "mem/ruby/libruby.hh"

#define g_DEADLOCK_THRESHOLD 5000


struct address_data {
  Address address;
  uint8_t * data;
};

class RaceyDriver : public Driver, public Consumer {
public:
  friend class RaceyPseudoThread;
  // Constructors
  RaceyDriver(int num_procs, int tester_length);

  // Destructor
  ~RaceyDriver();

  // Public Methods
  int runningThreads();
  void registerThread0Wakeup();
  void joinThread();
  bool Thread0Initialized() {
    return m_racey_pseudo_threads[0]->getInitializedState();
  };

  void hitCallback(int64_t request_id);
  void wakeup();
  void printStats(ostream& out) const;
  void clearStats() {}
  void printConfig(ostream& out) const {}
  void go();

  integer_t getInstructionCount(int procID) const;

  // save/load cpu states
  void saveCPUStates(int cpu_id, string filename) {
    m_racey_pseudo_threads[cpu_id]->saveCPUStates(filename);
  };
  void loadCPUStates(int cpu_id, string filename) {
    m_racey_pseudo_threads[cpu_id]->loadCPUStates(filename);
  };

  // reset IC
  void resetIC(int cpu_id) {
    m_racey_pseudo_threads[cpu_id]->resetIC();
  }

  void print(ostream& out) const;
  
private:

  // Private copy constructor and assignment operator
  RaceyDriver(const RaceyDriver& obj);
  RaceyDriver& operator=(const RaceyDriver& obj);
  
  // Data Members (m_ prefix)
  Vector<RaceyPseudoThread*> m_racey_pseudo_threads;
  int m_done_counter;
  bool m_wakeup_thread0;
  Time m_finish_time;
  map <int64_t, pair <int, struct address_data> > requests;
  RubyEventQueue * eventQueue;
  int m_num_procs;
  int m_tester_length;
};

// Output operator declaration
ostream& operator<<(ostream& out, const RaceyDriver& obj);

// ******************* Definitions *******************

// Output operator definition
extern inline 
ostream& operator<<(ostream& out, const RaceyDriver& obj)
{
  obj.print(out);
  out << flush;
  return out;
}

#endif //RACEYDRIVER_H

