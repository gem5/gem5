
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
 */

// This Deterministic Generator generates GETX requests for all nodes in the system
// The GETX requests are generated one at a time in round-robin fashion 0...1...2...etc.

#include "mem/ruby/tester/DetermGETXGenerator.hh"
#include "mem/protocol/DetermGETXGeneratorStatus.hh"
#include "mem/ruby/tester/DeterministicDriver.hh"
#include "mem/ruby/tester/Tester_Globals.hh"
#include "mem/ruby/common/Global.hh"
#include "mem/ruby/tester/SpecifiedGenerator.hh"
//#include "DMAController.hh"
#include "mem/ruby/libruby.hh"


DetermGETXGenerator::DetermGETXGenerator(NodeID node, DeterministicDriver * driver)
{
  m_status = DetermGETXGeneratorStatus_Thinking;
  m_last_transition = 0;
  counter = 0;
  m_node = node;
  m_address = Address(1);  // initialize to null value
  m_counter = 0;
  issued_load = false;
  parent_driver = driver;
  // don't know exactly when this node needs to request so just guess randomly
  parent_driver->eventQueue->scheduleEvent(this, 1+(random() % 200));
}

DetermGETXGenerator::~DetermGETXGenerator()
{
}

void DetermGETXGenerator::wakeup()
{
  DEBUG_EXPR(TESTER_COMP, MedPrio, m_node);
  DEBUG_EXPR(TESTER_COMP, MedPrio, m_status);

  // determine if this node is next for the GETX round robin request
  if (m_status == DetermGETXGeneratorStatus_Thinking) {
    if (parent_driver->isStoreReady(m_node)) {
      if (!issued_load) {
        pickAddress();
      }
      m_status = DetermGETXGeneratorStatus_Store_Pending;  // Store Pending
      m_last_transition = parent_driver->eventQueue->getTime();
      initiateStore();  // GETX
    } else { // I'll check again later
      parent_driver->eventQueue->scheduleEvent(this, thinkTime());
    }
  } else {
    WARN_EXPR(m_status);
    ERROR_MSG("Invalid status");
  }

}

void DetermGETXGenerator::performCallback(NodeID proc, Address address)
{
  assert(proc == m_node);
  assert(address == m_address);  

  DEBUG_EXPR(TESTER_COMP, LowPrio, proc);
  DEBUG_EXPR(TESTER_COMP, LowPrio, m_status);
  DEBUG_EXPR(TESTER_COMP, LowPrio, address);

  if (m_status == DetermGETXGeneratorStatus_Store_Pending) { 
    parent_driver->recordStoreLatency(parent_driver->eventQueue->getTime() - m_last_transition);
    parent_driver->storeCompleted(m_node, address);  // advance the store queue

    m_counter++;
    if (m_counter < parent_driver->m_tester_length) {
      m_status = DetermGETXGeneratorStatus_Thinking;
      m_last_transition = parent_driver->eventQueue->getTime();
      parent_driver->eventQueue->scheduleEvent(this, waitTime());
    } else {
      parent_driver->reportDone();
      m_status = DetermGETXGeneratorStatus_Done;
      m_last_transition = parent_driver->eventQueue->getTime();
    } 

  } else {
    WARN_EXPR(m_status);
    ERROR_MSG("Invalid status");
  }
}

int DetermGETXGenerator::thinkTime() const
{
  return parent_driver->m_think_time;
}

int DetermGETXGenerator::waitTime() const
{
  return parent_driver->m_wait_time;
}

void DetermGETXGenerator::pickAddress()
{
  assert(m_status == DetermGETXGeneratorStatus_Thinking);

  m_address = parent_driver->getNextStoreAddr(m_node);
}

void DetermGETXGenerator::initiateStore()
{
  DEBUG_MSG(TESTER_COMP, MedPrio, "initiating Store");
  uint8_t *write_data = new uint8_t[64];
  for(int i=0; i < 64; i++) {
      write_data[i] = m_node;
  }

  char name [] = "Sequencer_";
  char port_name [13];
  sprintf(port_name, "%s%d", name, m_node);
  int64_t request_id;
  if (counter%10 == 0) { 
    if (!issued_load) {
      cerr << m_node << " RMW_Read to address: " << m_address.getAddress() << endl << flush;
      request_id = libruby_issue_request(libruby_get_port_by_name(port_name), RubyRequest(m_address.getAddress(), write_data, 64, 0, RubyRequestType_RMW_Read, RubyAccessMode_Supervisor));
      issued_load = true;
    }
    else {
      cerr << m_node << " RMW_Write to address: " << m_address.getAddress() << endl << flush;
      request_id = libruby_issue_request(libruby_get_port_by_name(port_name), RubyRequest(m_address.getAddress(), write_data, 64, 0, RubyRequestType_RMW_Write, RubyAccessMode_Supervisor));
      issued_load = false;
      counter++;
    }
  }
  else {
      cerr << m_node << " ST to address: " << m_address.getAddress() << endl << flush;
      request_id = libruby_issue_request(libruby_get_port_by_name(port_name), RubyRequest(m_address.getAddress(), write_data, 64, 0, RubyRequestType_ST, RubyAccessMode_Supervisor));
      counter++;
  }

  // delete [] write_data;

  ASSERT(parent_driver->requests.find(request_id) == parent_driver->requests.end()); 
  parent_driver->requests.insert(make_pair(request_id, make_pair(m_node, m_address)));
}

void DetermGETXGenerator::print(ostream& out) const
{
}

