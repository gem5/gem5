/*
 * Copyright (c) 2009 Mark D. Hill and David A. Wood
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

#ifndef __MEM_RUBY_LIBRUBY_HH__
#define __MEM_RUBY_LIBRUBY_HH__

#include <ostream>

#include "base/types.hh"
#include "mem/packet.hh"

typedef void* RubyPortHandle;
enum RubyRequestType {
  RubyRequestType_NULL,
  RubyRequestType_IFETCH,
  RubyRequestType_LD,
  RubyRequestType_ST,
  RubyRequestType_Load_Linked,
  RubyRequestType_Store_Conditional,
  RubyRequestType_RMW_Read,
  RubyRequestType_RMW_Write,
  RubyRequestType_Locked_RMW_Read,
  RubyRequestType_Locked_RMW_Write,
  RubyRequestType_NUM
};

enum RubyAccessMode {
  RubyAccessMode_User,
  RubyAccessMode_Supervisor,
  RubyAccessMode_Device
};

struct RubyRequest
{
    uint64_t paddr;
    uint8_t* data;
    int len;
    uint64_t pc;
    RubyRequestType type;
    RubyAccessMode access_mode;
    PacketPtr pkt;
    unsigned proc_id;

    RubyRequest() {}
    RubyRequest(uint64_t _paddr,
                uint8_t* _data,
                int _len,
                uint64_t _pc,
                RubyRequestType _type,
                RubyAccessMode _access_mode,
                PacketPtr _pkt,
                unsigned _proc_id = 100)
        : paddr(_paddr),
          data(_data),
          len(_len),
          pc(_pc),
          type(_type),
          access_mode(_access_mode),
          pkt(_pkt),
          proc_id(_proc_id)
    {}
};

std::ostream& operator<<(std::ostream& out, const RubyRequest& obj);
std::ostream& operator<<(std::ostream& out, const RubyRequestType& obj);

/**
 * Initialize the system.  cfg_file is a Ruby-lang configuration script
 */
void libruby_init(const char* cfg_file);

/**
 * Tear down a configured system.  Must be invoked after a call to
 * libruby_init.
 */
void libruby_destroy();

/**
 * Print the last error encountered by ruby.  Currently unimplemented.
 */
const char* libruby_last_error();

/**
 *  Retrieve a handle to a RubyPort object, identified by name in the
 *  configuration.  You also pass in the callback function you want
 *  this port to use when a request completes.  Only one handle to a
 *  port is allowed at a time.
 */
RubyPortHandle libruby_get_port(const char* name,
                                void (*hit_callback)(int64_t access_id));

/**
 *  Retrieve a handle to a RubyPort object, identified by name in the
 *  configuration.
 */
RubyPortHandle libruby_get_port_by_name(const char* name);

/**
 * issue_request returns a unique access_id to identify the ruby
 * transaction. This access_id is later returned to the caller via
 * hit_callback (passed to libruby_get_port)
 */
int64_t libruby_issue_request(RubyPortHandle p, struct RubyRequest request);

/**
 * writes data directly into Ruby's data array.  Note that this
 * ignores caches, and should be considered incoherent after
 * simulation starts.
 */
void libruby_write_ram(uint64_t paddr, uint8_t *data, int len);

/**
 * reads data directory from Ruby's data array.  Note that this
 * ignores caches, and should be considered incoherent after
 * simulation starts
 */
void libruby_read_ram(uint64_t paddr, uint8_t *data, int len);

/**
 * tick the system n cycles.  Eventually, will return the number of
 * cycles until the next event, but for now it always returns 0
 */
int libruby_tick(int n);

/**
 *  self explainitory
 */
void libruby_print_config(std::ostream &out);

/**
 * self explainitory
 */
void libruby_print_stats(std::ostream &out);

/**
 * does not return until done
 */
void libruby_playback_trace(char *trace_filename);

/*
 * enables the tracer and opens the trace file
 */
void libruby_start_tracing(char *record_filename);

/*
 * closes the trace file
 */
void libruby_stop_tracing();

/**
 * get time
 */
uint64_t libruby_get_time();

#endif // __MEM_RUBY_LIBRUBY_HH__
