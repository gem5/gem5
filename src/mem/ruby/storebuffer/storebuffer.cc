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

/*------------------------------------------------------------------------*/
/* Includes                                                                             */
/*------------------------------------------------------------------------*/

#include <map>

#include "mem/ruby/storebuffer/hfa.hh"
#include "mem/ruby/storebuffer/storebuffer.hh"
#include "mem/ruby/common/Global.hh"

#if RUBY_TSO_CHECKER
#include "TsoChecker.hh"
#endif

#define SYSTEM_EXIT ASSERT(0)


// global map of request id_s to map them back to storebuffer pointers
map <uint64_t, StoreBuffer *> request_map;

#if RUBY_TSO_CHECKER
Tso::TsoChecker * g_tsoChecker;
#endif

void hit(int64_t id) {
  if (request_map.find(id) == request_map.end()) {
    ERROR_OUT("Request ID not found in the map");
    DEBUG_EXPR(STOREBUFFER_COMP, MedPrio, id);
    ASSERT(0);
  }
  else {
    request_map[id]->complete(id);
    request_map.erase(id);
  }
}


//*****************************************************************************************
StoreBuffer::StoreBuffer(uint32 id, uint32 block_bits, int storebuffer_size) {
#if RUBY_TSO_CHECKER
  if (id == 0) {
    g_tsoChecker = new Tso::TsoChecker();
    g_tsoChecker->init(64);
  }
#endif
  iseq = 0;
  tso_iseq = 0;
  char name [] = "Sequencer_";
  char port_name [13];
  sprintf(port_name, "%s%d", name, id);
  m_port = libruby_get_port(port_name, hit);
  m_hit_callback = NULL;
  ASSERT(storebuffer_size >= 0);
  m_storebuffer_size = storebuffer_size;
  m_id = id;
  m_block_size = 1 << block_bits;
  m_block_mask = ~(m_block_size - 1);
  m_buffer_size = 0;
  m_use_storebuffer = false;
  m_storebuffer_full = false;
  m_storebuffer_flushing = false;
  m_stalled_issue = true;
  if(m_storebuffer_size > 0){
    m_use_storebuffer = true;
  }

  #ifdef DEBUG_WRITE_BUFFER
      DEBUG_OUT("*******storebuffer_t::Using Write Buffer? %d\n",m_use_storebuffer);
  #endif
}

//******************************************************************************************
StoreBuffer::~StoreBuffer(){
#if RUBY_TSO_CHECKER
  if (m_id == 0) {
    delete g_tsoChecker;
  }
#endif
}

//*****************************************************************************************************
void StoreBuffer::registerHitCallback(void (*hit_callback)(int64_t request_id)) {
  assert(m_hit_callback == NULL); // can't assign hit_callback twice
  m_hit_callback = hit_callback;
}


//*****************************************************************************************************
void StoreBuffer::addToStoreBuffer(struct RubyRequest request){
  if(m_use_storebuffer){
    #ifdef DEBUG_WRITE_BUFFER
       DEBUG_OUT("\n***StoreBuffer: addToStoreBuffer BEGIN, contents:\n");
       DEBUG_OUT("\n");
    #endif

    #ifdef DEBUG_WRITE_BUFFER
       DEBUG_OUT("\t INSERTING new request\n");
    #endif


    buffer.push_front(SBEntry(request, NULL));

    m_buffer_size++;

    if (m_buffer_size >= m_storebuffer_size) {
      m_storebuffer_full = true;
    }
    else if (m_stalled_issue) {
      m_stalled_issue = false;
      issueNextStore();
    }

    iseq++;

     #ifdef DEBUG_WRITE_BUFFER
       DEBUG_OUT("***StoreBuffer: addToStoreBuffer END, contents:\n");
       DEBUG_OUT("\n");
     #endif
  }  //end if(m_use_storebuffer)
  else {
    // make request to libruby
    uint64_t id = libruby_issue_request(m_port, request);
    if (request_map.find(id) != request_map.end()) {
      ERROR_OUT("Request ID is already in the map");
      DEBUG_EXPR(STOREBUFFER_COMP, MedPrio, id);
      ASSERT(0);
    }
    else {
      request_map.insert(make_pair(id, this));
      outstanding_requests.insert(make_pair(id, request));
    }
  }
}


//*****************************************************************************************************
// Return value of -2 indicates that the load request was satisfied by the store buffer
// Return value of -3 indicates a partial match, so the load has to retry until NO_MATCH
// Alternatively we could satisfy the partial match, but tso gets complicated and more races
//*****************************************************************************************************
int64_t StoreBuffer::handleLoad(struct RubyRequest request) {
  if (m_use_storebuffer) {
    load_match match = checkForLoadHit(request);
    if (match == FULL_MATCH) {
      // fill data
      returnMatchedData(request);
      iseq++;
      return -2;
    }
    else if (match == NO_MATCH) {
      // make request to libruby and return the id
      uint64_t id = libruby_issue_request(m_port, request);
      if (request_map.find(id) != request_map.end()) {
        ERROR_OUT("Request ID is already in the map");
        DEBUG_EXPR(STOREBUFFER_COMP, MedPrio, id);
        ASSERT(0);
      }
      else {
        request_map.insert(make_pair(id, this));
        outstanding_requests.insert(make_pair(id, request));
      }
      iseq++;
      return id;
    }
    else { // partial match
      return -3;
    }
  }
  else {
    // make a request to ruby
    return libruby_issue_request(m_port, request);
  }
}


//*****************************************************************************************************
// This function will fill the data array if any match is found
//*****************************************************************************************************
load_match StoreBuffer::checkForLoadHit(struct RubyRequest request) {
  if (m_use_storebuffer) {
    physical_address_t physical_address = request.paddr;
    int len = request.len;

    uint8_t * data = new uint8_t[64];
    memset(data, 0, 64);
    for (int i = physical_address%64; i < len; i++) {
      data[i] = 1;
    }

    bool found = false;
    physical_address_t lineaddr = physical_address & m_block_mask;

    // iterate over the buffer looking for hits
    for (deque<struct SBEntry>::iterator it = buffer.begin(); it != buffer.end(); it++) {
      if ((it->m_request.paddr & m_block_mask) == lineaddr) {
        found = true;
        for (int i = it->m_request.paddr%64; i < it->m_request.len; i++) {
          data[i] = 0;
        }
      }
    }

    // if any matching entry is found, determine if all the requested bytes have been matched
    if (found) {
      ASSERT(m_buffer_size > 0);
      int unmatched_bytes = 0;
      for (int i = physical_address%64; i < len; i++) {
        unmatched_bytes = unmatched_bytes + data[i];
      }
      if (unmatched_bytes == 0) {
        delete data;
        return FULL_MATCH;
      }
      else {
        delete data;
        return PARTIAL_MATCH;
      }
    }
    else {
      delete data;
      return NO_MATCH;
    }
  } // end of if (m_use_storebuffer)
  else {
    // this function should never be called if we are not using a store buffer
    ERROR_OUT("checkForLoadHit called while write buffer is not in use");
    ASSERT(0);
  }
}


//***************************************************************************************************
void StoreBuffer::returnMatchedData(struct RubyRequest request) {
  if (m_use_storebuffer) {

    uint8_t * data = new uint8_t[64];
    memset(data, 0, 64);
    uint8_t * written = new uint8_t[64];
    memset(written, 0, 64);

    physical_address_t physical_address = request.paddr;
    int len = request.len;

    ASSERT(checkForLoadHit(request) != NO_MATCH);
    physical_address_t lineaddr = physical_address & m_block_mask;
    bool found = false;
#if RUBY_TSO_CHECKER
    Tso::TsoCheckerCmd * cmd;
#endif
    deque<struct SBEntry>::iterator satisfying_store;
    for (deque<struct SBEntry>::iterator it = buffer.begin(); it != buffer.end(); it++) {
      if ((it->m_request.paddr & m_block_mask) == lineaddr) {
        if (!found) {
          found = true;
#if RUBY_TSO_CHECKER
          satisfying_store = it;
          cmd = new Tso::TsoCheckerCmd(m_id, // this thread id
                            iseq, // instruction sequence
                            ITYPE_LOAD, // is a store
                            MEM_LOAD_DATA, // commit
                            request.paddr, // the address
                            NULL, // and data
                            request.len,  // and len
                            DSRC_STB, // shouldn't matter
                            libruby_get_time(), // macc: for store macc and time are the same and it
                            0, // gobs
                            0);
#endif
        }
        uint8_t * dataPtr = it->m_request.data;
        int offset = it->m_request.paddr%64;
        for (int i = offset; i < it->m_request.len; i++) {
          if (!written[i]) { // don't overwrite data with earlier data
            data[i] = dataPtr[i-offset];
            written[i] = 1;
          }
        }
      }
    }

    int i = physical_address%64;
    for (int j = 0; (i < physical_address%64 + len) && (j < len); i++, j++) {
      if (written[i]) {
        request.data[j] = data[i];
      }
    }

#if RUBY_TSO_CHECKER
    uint64_t tso_data = 0;
    memcpy(&tso_data, request.data, request.len);
    cmd->setData(tso_data);

    Tso::TsoCheckerCmd * adjust_cmd = satisfying_store->m_next_ptr;
    if (adjust_cmd == NULL) {
      adjust_cmd = cmd;
    }
    else {
      while (adjust_cmd->getNext() != NULL) {
        adjust_cmd = adjust_cmd->getNext();
      }
      adjust_cmd->setNext(cmd);
    }
#endif

    delete data;
    delete written;
  }
  else {
    ERROR_OUT("returnMatchedData called while write buffer is not in use");
    ASSERT(0);
  }
}


//******************************************************************************************
void StoreBuffer::flushStoreBuffer(){
  if (m_use_storebuffer) {
     #ifdef DEBUG_WRITE_BUFFER
         DEBUG_OUT("\n***StoreBuffer: flushStoreBuffer BEGIN, contents:\n");
         DEBUG_OUT("\n");
     #endif

     if(m_buffer_size > 0) {
       m_storebuffer_flushing = true; // indicate that we are flushing
     }
     else {
       m_storebuffer_flushing = false;
       return;
     }
  }
  else {
    // do nothing
    return;
  }
}

//****************************************************************************************
void StoreBuffer::issueNextStore() {
  SBEntry request = buffer.back();
  uint64_t id = libruby_issue_request(m_port, request.m_request);
  if (request_map.find(id) != request_map.end()) {
    assert(0);
  }
  else {
    request_map.insert(make_pair(id, this));
    outstanding_requests.insert(make_pair(id, request.m_request));
  }
}

//****************************************************************************************
void StoreBuffer::complete(uint64_t id) {
  if (m_use_storebuffer) {
    ASSERT(outstanding_requests.find(id) != outstanding_requests.end());
    physical_address_t physical_address = outstanding_requests.find(id)->second.paddr;
    RubyRequestType type = outstanding_requests.find(id)->second.type;
#ifdef DEBUG_WRITE_BUFFER
    DEBUG_OUT("\n***StoreBuffer: complete BEGIN, contents:\n");
    DEBUG_OUT("\n");
#endif

    if (type == RubyRequestType_ST) {
      physical_address_t lineaddr = physical_address & m_block_mask;

      //Note fastpath hits are handled like regular requests - they must remove the WB entry!
      if ( lineaddr != physical_address ) {
        ERROR_OUT("error: StoreBuffer: ruby returns pa 0x%0llx which is not a cache line: 0x%0llx\n", physical_address, lineaddr );
      }

      SBEntry from_buffer = buffer.back();
      if (((from_buffer.m_request.paddr & m_block_mask) == lineaddr) && (from_buffer.m_request.type == type)) {
        buffer.pop_back();
        m_buffer_size--;
        ASSERT(m_buffer_size >= 0);

#if RUBY_TSO_CHECKER
        int len = outstanding_requests.find(id)->second.len;
        uint64_t data = 0;
        memcpy(&data, from_buffer.m_request.data, 4);

        cerr << m_id << " INSERTING STORE" << endl << flush;
        // add to the tsoChecker
        g_tsoChecker->input(m_id, // this thread id
                          (id & ISEQ_MASK), // instruction sequence
                          ITYPE_STORE, // is a store
                          MEM_STORE_COMMIT, // commit
                          physical_address, // the address
                          data, // and data
                          len,  // and len
                          DSRC_STB, // shouldn't matter
                          libruby_get_time(), // macc
                          libruby_get_time(), // gobs
                          libruby_get_time()); // time
        tso_iseq++;

        // also add the loads that are satisfied by this store
        if (from_buffer.m_next_ptr != NULL) {
          from_buffer.m_next_ptr->setGobs(libruby_get_time());
          g_tsoChecker->input(*(from_buffer.m_next_ptr));
          cerr << m_id << " INSERTING LOAD for STORE: " << from_buffer.m_next_ptr->getIseq()  << endl << flush;
          tso_iseq++;
          Tso::TsoCheckerCmd * to_input = from_buffer.m_next_ptr->getNext();
          while (to_input != NULL) {
            if (to_input->getGobs() == 0) {
              to_input->setGobs(libruby_get_time());
            }
            cerr << m_id << " INSERTING LOAD iseq for STORE: " << to_input->getIseq() << endl << flush;
            g_tsoChecker->input(*to_input);
            tso_iseq++;
            to_input = to_input->getNext();
          }
        }
#endif
        // schedule the next request
        if (m_buffer_size > 0) {
          issueNextStore();
        }
        else if (m_buffer_size == 0) {
          m_storebuffer_flushing = false;
          m_stalled_issue = true;
        }

        m_storebuffer_full = false;

      }
      else {
        ERROR_OUT("[%d] error: StoreBuffer: at complete, address 0x%0llx not found.\n", m_id, lineaddr);
        ERROR_OUT("StoreBuffer:: complete FAILS\n");
        ASSERT(0);
      }

#ifdef DEBUG_WRITE_BUFFER
      DEBUG_OUT("***StoreBuffer: complete END, contents:\n");
      DEBUG_OUT("\n");
#endif
    } // end if (type == ST)
    else if (type == RubyRequestType_LD) {
#if RUBY_TSO_CHECKER
      RubyRequest request = outstanding_requests.find(id)->second;
      uint64_t data = 0;
      memcpy(&data, request.data, request.len);

      // add to the tsoChecker if in order, otherwise, find a place to put ourselves
      if ((id & ISEQ_MASK) == tso_iseq) {
        tso_iseq++;
        cerr << m_id << " INSERTING LOAD" << endl << flush;
        g_tsoChecker->input(m_id, // this thread id
                          (id & ISEQ_MASK), // instruction sequence
                          ITYPE_LOAD, // is a store
                          MEM_LOAD_DATA, // commit
                          request.paddr, // the address
                          data, // and data
                          request.len,  // and len
                          DSRC_L2_MEMORY, // shouldn't matter  DSRC_L1
                          libruby_get_time(), // macc: for store macc and time are the same and it
                          libruby_get_time(), // macc
                          libruby_get_time()); // time
      }
      else {
        Tso::TsoCheckerCmd * cmd;
        cmd = new Tso::TsoCheckerCmd(m_id, // this thread id
                          (id & ISEQ_MASK), // instruction sequence
                          ITYPE_LOAD, // is a store
                          MEM_LOAD_DATA, // commit
                          request.paddr, // the address
                          data, // and data
                          request.len,  // and len
                          DSRC_L2_MEMORY, // shouldn't matter  DSRC_L1
                          libruby_get_time(), // macc: for store macc and time are the same and it
                          libruby_get_time(), // macc
                          libruby_get_time()); // time
        insertTsoLL(cmd);
      }
#endif
      m_hit_callback(id);
    }

    // LD, ST or FETCH hit callback
    outstanding_requests.erase(id);

  } // end if(m_use_storebuffer)
  else {
    m_hit_callback(id);
  }
}

#if RUBY_TSO_CHECKER
void StoreBuffer::insertTsoLL(Tso::TsoCheckerCmd * cmd) {
  uint64_t count = cmd->getIseq();
  Tso::TsoCheckerCmd * current = NULL;
  Tso::TsoCheckerCmd * previous = NULL;
  deque<struct SBEntry>::reverse_iterator iter;
  bool found = false;
  for (iter = buffer.rbegin(); iter != buffer.rend(); ++ iter) {
    if (iter->m_next_ptr != NULL) {
      current = iter->m_next_ptr->getNext(); // initalize both to the beginning of the linked list
      previous = current;
      while (current != NULL) {
        if (current->getIseq() > count) {
          found = true;
          break;
        }
        previous = current;
        current = current->getNext();
      }
    }
    // break out if found a match, iterator should still point to the right SBEntry
    if (found) {
      break;
    }
  }

  // will insert at the end if not found
  if (!found) {
    buffer.front().m_next_ptr = cmd;
  }
  else if (current == previous) {
    cerr << "INSERTING " << count << " BEFORE: " << iter->m_next_ptr->getIseq();
    Tso::TsoCheckerCmd * temp = iter->m_next_ptr;
    iter->m_next_ptr = cmd;
    cmd->setNext(temp);
  }
  else {
    cerr << "INSERTING " << count << " BETWEEN: " << previous->getIseq() << " AND " << current->getIseq();
    cmd->setNext(current);
    previous->setNext(cmd);
  }
}
#endif

//***************************************************************************************************
void StoreBuffer::print( void )
{
  DEBUG_OUT("[%d] StoreBuffer: Total entries: %d Outstanding: %d\n", m_id, m_buffer_size);

  if(m_use_storebuffer){
  }
  else{
    DEBUG_OUT("\t WRITE BUFFER NOT USED\n");
  }
}




