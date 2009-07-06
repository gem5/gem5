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

#ifndef _STOREBUFFER_H_
#define _STOREBUFFER_H_

#include <map>
#include <deque>

#include "config/ruby_tso_checker.hh"
#include "mem/ruby/storebuffer/hfa.hh"
#include "mem/ruby/libruby.hh"

#if RUBY_TSO_CHECKER
#include "TsoCheckerCmd.hh"
#endif

/**
 *  Status for write buffer accesses. The Write buffer can hit in fastpath, be full, or
 *     successfully enqueue the store request
 */
enum storebuffer_status_t { WB_FULL, WB_OK, WB_FLUSHING };

/**
 *  Status of a load match
 */
enum load_match { NO_MATCH, PARTIAL_MATCH, FULL_MATCH };

struct SBEntry {
  struct RubyRequest m_request;
#if RUBY_TSO_CHECKER
  Tso::TsoCheckerCmd * m_next_ptr;
#endif
  SBEntry(struct RubyRequest request, void * ptr)
      : m_request(request)
    {
#if RUBY_TSO_CHECKER
        m_next_ptr = (Tso::TsoCheckerCmd*) ptr;
#endif
    }
};

class StoreBuffer {
 public:
  ///Constructor
  /// Note that the size of the Write Buffer is determined by the WRITE_BUFFER_SIZE config parameter
  StoreBuffer(uint32 id, uint32 block_bits, int storebuffer_size);

  /// Register hitcallback back to CPU
  void registerHitCallback(void (*hit_callback)(int64_t request_id));

  /// Destructor
  ~StoreBuffer();

  ///Adds a store entry to the write buffer
  void addToStoreBuffer(struct RubyRequest request);

  ///Flushes the entire write buffer
  void flushStoreBuffer();

  ///A pseq object calls this when Ruby completes our request
  void complete(uint64_t);

  /// Returns ID. If ID == -2, HIT, else it's an ID to wait on
  int64_t handleLoad(struct RubyRequest request);

  /// Used by all load insts to check whether it hits to any entry in the WB. If so, the WB is flushed
  load_match checkForLoadHit(struct RubyRequest request);

  /// Used to fill the load in case of FULL_MATCH
  void returnMatchedData(struct RubyRequest request);

  /// Issue next store in line
  void issueNextStore();

  /// prints out the contents of the Write Buffer
  void print();

#if RUBY_TSO_CHECKER
  /// if load completes before store, insert correctly to be issued to TSOChecker
  void insertTsoLL(Tso::TsoCheckerCmd * cmd);
#endif

  /// Returns flag indicating whether we are using the write buffer
  bool useStoreBuffer() { return m_use_storebuffer; }

  bool storeBufferFull() { return m_storebuffer_full; }

  bool storeBufferFlushing() { return m_storebuffer_flushing; }

 private:
  /// id of this write buffer (one per sequencer object)
  uint32          m_id;

  /// number of bytes in cacheline
  uint32          m_block_size;

  /// the size of the write buffer
  uint32          m_storebuffer_size;

  /// mask to strip off non-cache line bits
  pa_t            m_block_mask;

  /// list of store requests in the write buffer
  deque <struct SBEntry> buffer;

  /// the current length of the write buffer
  uint32          m_buffer_size;

  /// whether we want to simulate the write buffer or not:
  bool            m_use_storebuffer;

  /// indicates whether the write buffer is full or not
  bool            m_storebuffer_full;

  /// indicates that we are currently flushing the write buffer
  bool            m_storebuffer_flushing;

  /// indicates that automatic issue is stalled and the next store to be added should issue itself
  bool            m_stalled_issue;

  /// RubyPort to make requests to
  RubyPortHandle  m_port;

  /// HitCallback to CPU
  void            (*m_hit_callback)(int64_t);

  /// Map the request id to rubyrequest
  map<uint64_t, struct RubyRequest> outstanding_requests;

  /// current instruction counter
  uint64_t iseq;


  /// input into tso counter
  uint64_t tso_iseq;
};

#endif
