
#ifndef LIBRUBY_H
#define LIBRUBY_H

#include <stdint.h>
#include <ostream>

typedef void* RubyPortHandle;
enum RubyRequestType {
  RubyRequestType_NULL,
  RubyRequestType_IFETCH,
  RubyRequestType_LD,
  RubyRequestType_ST,
  RubyRequestType_Locked_Read,
  RubyRequestType_Locked_Write,
  RubyRequestType_RMW_Read,
  RubyRequestType_RMW_Write,
  RubyRequestType_NUM
};

enum RubyAccessMode {
  RubyAccessMode_User,
  RubyAccessMode_Supervisor,
  RubyAccessMode_Device
};

struct RubyRequest {
  uint64_t paddr;
  uint8_t* data;
  int len;
  uint64_t pc;
  RubyRequestType type;
  RubyAccessMode access_mode;

  RubyRequest() {}
  RubyRequest(uint64_t _paddr, uint8_t* _data, int _len, uint64_t _pc, RubyRequestType _type, RubyAccessMode _access_mode)
    : paddr(_paddr), data(_data), len(_len), pc(_pc), type(_type), access_mode(_access_mode)
  {}
};

/**
 * Initialize the system.  cfg_file is a Ruby-lang configuration script
 */
void libruby_init(const char* cfg_file);

/**
 * Tear down a configured system.  Must be invoked after a call to libruby_init.
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
RubyPortHandle libruby_get_port(const char* name, void (*hit_callback)(int64_t access_id));

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
void libruby_write_ram(uint64_t paddr, uint8_t * data, int len);

/**
 * reads data directory from Ruby's data array.  Note that this
 * ignores caches, and should be considered incoherent after
 * simulation starts
 */
void libruby_read_ram(uint64_t paddr, uint8_t * data, int len);

/**
 * tick the system n cycles.  Eventually, will return the number of
 * cycles until the next event, but for now it always returns 0
 */
int libruby_tick(int n);

/**
 *  self explainitory
 */
void libruby_print_config(std::ostream & out);

/**
 * self explainitory
 */
void libruby_print_stats(std::ostream & out);

/**
 * does not return until done
 */  
void libruby_playback_trace(char * trace_filename);

/*
 * enables the tracer and opens the trace file
 */ 
void libruby_start_tracing(char * record_filename);

/*
 * closes the trace file
 */ 
void libruby_stop_tracing();

/**
 * get time
 */
uint64_t libruby_get_time();
#endif
