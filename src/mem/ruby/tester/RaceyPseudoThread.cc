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
 * Description: see RaceyPseudoThread.h
 */

#include "mem/ruby/tester/RaceyPseudoThread.hh"
#include "mem/ruby/tester/RaceyDriver.hh"
#include "gzstream.hh"

RaceyPseudoThread::RaceyPseudoThread(NodeID id, RaceyDriver& driver)
  : m_driver(driver), m_proc_id(id) {

  resetIC(); // IC contains the committed instruction number
  m_last_progress = 0;
  m_done = false;
  m_stop = 0;
  m_driver.eventQueue->scheduleEvent(this, 1);
}

RaceyPseudoThread::~RaceyPseudoThread() {
}

void RaceyPseudoThread::checkForDeadlock() {
  Time current_time = m_driver.eventQueue->getTime();
  if(!m_done && (current_time - m_last_progress) > g_DEADLOCK_THRESHOLD) {
    panic("Deadlock detected: m_proc_id: %d m_ic_counter: %d "
          "m_last_progress: %d\n",
          m_proc_id, m_ic_counter, m_last_progress);
  }
}

void RaceyPseudoThread::performCallback(int proc, Address address, uint8_t * data ) {
  assert(proc == m_proc_id);

  DPRINTF(RubyTester, "proc: %d, address: %s\n", proc, address);

  m_last_progress = m_driver.eventQueue->getTime();

  if(m_read) {
    int b0, b1, b2, b3;
    b0 = data[0]; b0 <<= 0;
    b1 = data[1]; b1 <<= 8;
    b2 = data[2]; b2 <<= 16;
    b3 = data[3]; b3 <<= 24;
    m_value = b0 | b1 | b2 | b3;
  } else {
    char b0, b1, b2, b3;
    b0 = (m_value>>0)&0xFF; data[0] = b0;
    b1 = (m_value>>8)&0xFF; data[1] = b1;
    b2 = (m_value>>16)&0xFF; data[2] = b2;
    b3 = (m_value>>24)&0xFF; data[3] = b3;
  }

  // schedule wakeup for next requests in next cycle
  m_driver.eventQueue->scheduleEvent(this, 1);

  // new instruction
  m_ic_counter++;

}

void RaceyPseudoThread::wakeup() {
  // for debug
  if(m_stop != 0) {
    cout << m_proc_id << " " << m_stop << ((m_read)?  " read ":" written ") << m_value;
    if(0) cout << " [" << m_driver.eventQueue->getTime() << "]";
    cout << endl;
  }

  assert(!m_done);

  // Note, this function can not have ANY local variable!

  switch(m_stop) {
  case 0:
    break;
  case 1:
    goto L1;
  case 2:
    goto L2;
  case 3:
    goto L3;
  case 4:
    goto L4;
  case 5:
    goto L5;
  case 6:
    goto L6;
  case 7:
    goto L7;
  case 8:
    goto L8;
  case 9:
    goto L9;
  case 10:
    goto L10;
  default:
    fatal("RaceyPseudoThread: Bad context point %u!", m_stop);
  }

  //
  // initialization
  //
  if(m_proc_id == 0) {
    for(m_looper = 0; m_looper < m_driver.m_num_procs; m_looper++) {
      store_sig(m_looper, m_looper+1);
      m_stop = 6; return;
L6:   {};
    }
    for(m_looper = 0; m_looper < M_ELEM; m_looper++) {
      store_m(m_looper, M_ELEM-m_looper);
      m_stop = 7; return;
L7:   {};
    }

    // init done
    m_initialized = true;
  } else {
    // other processors
    if(!m_driver.Thread0Initialized()) {
      // wait for processors 0
      m_driver.eventQueue->scheduleEvent(this, 1);
      return;
    }
  }

  cout << "Thread " << m_proc_id << " started in parallel phase" << endl;

  //
  // main thread body
  //
  for(m_looper = 0 ; m_looper < m_driver.m_tester_length; m_looper++) {
    /* m_value = */ load_sig(m_proc_id);
    m_stop = 1; return;
L1: {};
    m_num = m_value;
    m_index1 = m_num%M_ELEM;
    /* m_value = */ load_m(m_index1);
    m_stop = 2; return;
L2: {};
    m_num = mix(m_num, m_value);
    m_index2 = m_num%M_ELEM;
    /* m_value = */ load_m(m_index2);
    m_stop = 3; return;
L3: {};
    m_num = mix(m_num, m_value);
    store_m(m_index2, m_num);
    m_stop = 4; return;
L4: {};
    store_sig(m_proc_id, m_num);
    m_stop = 5; return;
L5: {};
  } // end for

  //
  // compute final sig
  //
  if(m_proc_id == 0) {
    // wait for other threads
    while (m_driver.runningThreads() > 1) {
      m_driver.registerThread0Wakeup();
      m_stop = 10; return;
L10: {};
    }

    /* m_value = */ load_sig(0);
    m_stop = 8; return;
L8: {};
    m_final_sig = m_value;
    for(m_looper = 1; m_looper < m_driver.m_num_procs; m_looper++) {
      /* m_value = */ load_sig(m_looper);
      m_stop = 9; return;
L9:   {};
      m_final_sig = mix(m_value, m_final_sig);
    }
  } // processors 0

  // done
  m_driver.joinThread();
  m_done = true;
}

void RaceyPseudoThread::load_sig(unsigned index) {
  cout << m_proc_id << " : load_sig " << index << endl;

  m_read = true;
  // timestamp, threadid, action, and logical address are used only by transactional memory, should be augmented
  uint8_t * read_data = new uint8_t[4];

  char name [] = "Sequencer_";
  char port_name [13];
  sprintf(port_name, "%s%d", name, m_proc_id);

  // pc is zero, problem?
  int64_t request_id = libruby_issue_request(libruby_get_port_by_name(port_name), RubyRequest(sig(index), read_data, 4, 0, RubyRequestType_LD, RubyAccessMode_User));
  
  ASSERT(m_driver.requests.find(request_id) == m_driver.requests.end());

  struct address_data request_data;
  request_data.address = Address(sig(index));
  request_data.data = read_data;
  m_driver.requests.insert(make_pair(request_id, make_pair(m_proc_id, request_data)));
  
  /*sequencer()->makeRequest(CacheMsg(Address(sig(index)), Address(sig(index)), CacheRequestType_LD,
                                    Address(physical_address_t(1)),
                                    AccessModeType_UserMode, 4,
                                    PrefetchBit_No, 0, Address(0), 
                                    0, 0 , false)); */
}

void RaceyPseudoThread::load_m(unsigned index) {
  // cout << m_proc_id << " : load_m " << index << endl;

  m_read = true;
  uint8_t * read_data = new uint8_t[4];

  char name [] = "Sequencer_";
  char port_name [13];
  sprintf(port_name, "%s%d", name, m_proc_id);

  // pc is zero, problem?
  int64_t request_id = libruby_issue_request(libruby_get_port_by_name(port_name), RubyRequest(m(index), read_data, 4, 0, RubyRequestType_LD, RubyAccessMode_User));
  
  ASSERT(m_driver.requests.find(request_id) == m_driver.requests.end());

  struct address_data request_data;
  request_data.address = Address(m(index));
  request_data.data = read_data;
  m_driver.requests.insert(make_pair(request_id, make_pair(m_proc_id, request_data)));
  
  /*sequencer()->makeRequest(CacheMsg(Address(m(index)), Address(m(index)), CacheRequestType_LD,
                                    Address(physical_address_t(1)),
                                    AccessModeType_UserMode, 4,
                                    PrefetchBit_No, 0, Address(0), 
                                    0, 0, false)); */
}

void RaceyPseudoThread::store_sig(unsigned index, unsigned value) {
  cout << m_proc_id << " : store_sig " << index << " " << value << endl;

  m_read = false;
  m_value = value;
  uint8_t * write_data = new uint8_t[4];


  memcpy(write_data, &value, 4);

  char name [] = "Sequencer_";
  char port_name [13];
  sprintf(port_name, "%s%d", name, m_proc_id);

  // pc is zero, problem?
  int64_t request_id = libruby_issue_request(libruby_get_port_by_name(port_name), RubyRequest(sig(index), write_data, 4, 0, RubyRequestType_ST, RubyAccessMode_User));
  
  ASSERT(m_driver.requests.find(request_id) == m_driver.requests.end());

  struct address_data request_data;
  request_data.address = Address(sig(index));
  request_data.data = write_data;
  m_driver.requests.insert(make_pair(request_id, make_pair(m_proc_id, request_data)));  

  /*sequencer()->makeRequest(CacheMsg(Address(sig(index)), Address(sig(index)), CacheRequestType_ST,
                                    Address(physical_address_t(1)),
                                    AccessModeType_UserMode, 4,
                                    PrefetchBit_No, 0, Address(0),
                                    0, 0, false)); */
}

void RaceyPseudoThread::store_m(unsigned index, unsigned value) {
  //cout << m_proc_id << " : store_m " << index << endl;

  m_read = false;
  m_value = value;
  uint8_t * write_data = new uint8_t[4];
  memcpy(write_data, &value, 4);

  char name [] = "Sequencer_";
  char port_name [13];
  sprintf(port_name, "%s%d", name, m_proc_id);

  // pc is zero, problem?
  int64_t request_id = libruby_issue_request(libruby_get_port_by_name(port_name), RubyRequest(m(index), write_data, 4, 0, RubyRequestType_ST, RubyAccessMode_User));
  
  ASSERT(m_driver.requests.find(request_id) == m_driver.requests.end());

  struct address_data request_data;
  request_data.address = Address(m(index));
  request_data.data = write_data;
  m_driver.requests.insert(make_pair(request_id, make_pair(m_proc_id, request_data)));  

  /*sequencer()->makeRequest(CacheMsg(Address(m(index)), Address(m(index)), CacheRequestType_ST,
                                    Address(physical_address_t(1)),
                                    AccessModeType_UserMode, 4,
                                    PrefetchBit_No, 0, Address(0),
                                    0, 0, false)); */
}

// Save and Load context of a thread
void RaceyPseudoThread::saveCPUStates(string filename) {
  ogzstream out(filename.c_str());
  out.write((char*)&m_looper, sizeof(int));
  out.write((char*)&m_num, sizeof(unsigned));
  out.write((char*)&m_index1, sizeof(unsigned));
  out.write((char*)&m_index2, sizeof(unsigned));
  out.write((char*)&m_stop, sizeof(unsigned));
  out.close();
}

void RaceyPseudoThread::loadCPUStates(string filename) {
  igzstream out(filename.c_str());
  out.read((char*)&m_looper, sizeof(int));
  out.read((char*)&m_num, sizeof(unsigned));
  out.read((char*)&m_index1, sizeof(unsigned));
  out.read((char*)&m_index2, sizeof(unsigned));
  out.read((char*)&m_stop, sizeof(unsigned));
  out.close();
}

void RaceyPseudoThread::print(ostream& out) const {
  out << "[Racey Pseudo Thread: " << m_proc_id << "]" << endl;
}

