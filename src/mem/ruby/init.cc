
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
 * init.C
 *
 * Description: See init.h
 *
 * $Id$
 *
 */

#include "Global.hh"
#include "EventQueue.hh"
#include "System.hh"
#include "Debug.hh"
#include "Profiler.hh"
#include "Tester.hh"
#include "OpalInterface.hh"
#include "init.hh"
#include "interface.hh"

#ifdef CONTIGUOUS_ADDRESSES
#include "ContiguousAddressTranslator.hh"

/* Declared in interface.C */
extern ContiguousAddressTranslator * g_p_ca_translator;

#endif // #ifdef CONTIGUOUS_ADDRESSES

using namespace std;
#include <string>
#include <map>
#include <stdlib.h>

// Maurice
// extern "C" {
// #include "simics/api.hh"
// };

#include "FakeSimicsDataTypes.hh"

#include "confio.hh"
#include "initvar.hh"

// A generated file containing the default parameters in string form
// The defaults are stored in the variable global_default_param
#include "default_param.hh"

attr_value_t ruby_session_get( void *id, void *obj,
                               attr_value_t *idx ) {
  attr_value_t ret;

  // all session attributes default to return invalid
  ret.kind = Sim_Val_Invalid;
  return ret;
}

set_error_t ruby_session_set( void *id, void *obj,
                              attr_value_t *val, attr_value_t *idx ) {
  const char *command = (const char *) id;
  // Add new ruby commands to this function

#if 0 // Eventually add these commands back in
  if (!strcmp(command, "dump-stats" ) ) {
    char* filename = (char*) val->u.string;
    if(strcmp(filename, "")){
      ruby_dump_stats(filename);
    } else {
      ruby_dump_stats(NULL);
    }
    return Sim_Set_Ok;
  } else if (!strcmp(command, "dump-short-stats" ) ) {
    char* filename = (char*) val->u.string;
    if(strcmp(filename, "")){
      ruby_dump_short_stats(filename);
    } else {
      ruby_dump_short_stats(NULL);
    }
    return Sim_Set_Ok;
  } else if (!strcmp(command, "periodic-stats-file" ) ) {
    char* filename = (char*) val->u.string;
    ruby_set_periodic_stats_file(filename);
    return Sim_Set_Ok;
  } else if (!strcmp(command, "periodic-stats-interval" ) ) {
    int interval = val->u.integer;
    ruby_set_periodic_stats_interval(interval);
    return Sim_Set_Ok;
  } else if (!strcmp(command, "clear-stats" ) ) {
    ruby_clear_stats();
    return Sim_Set_Ok;
  } else if (!strcmp(command, "debug-verb" ) ) {
    char* new_verbosity = (char*) val->u.string;
    ruby_change_debug_verbosity(new_verbosity);
    return Sim_Set_Ok;
  } else if (!strcmp(command, "debug-filter" ) ) {
    char* new_debug_filter = (char*) val->u.string;
    ruby_change_debug_filter(new_debug_filter);
    return Sim_Set_Ok;
  } else if (!strcmp(command, "debug-output-file" ) ) {
    char* new_filename = (char*) val->u.string;
    ruby_set_debug_output_file(new_filename);
    return Sim_Set_Ok;
  } else if (!strcmp(command, "debug-start-time" ) ) {
    char* new_start_time = (char*) val->u.string;
    ruby_set_debug_start_time(new_start_time);
    return Sim_Set_Ok;
  } else if (!strcmp(command, "load-caches" ) ) {
    char* filename = (char*) val->u.string;
    ruby_load_caches(filename);
    return Sim_Set_Ok;
  } else if (!strcmp(command, "save-caches" ) ) {
    char* filename = (char*) val->u.string;
    ruby_save_caches(filename);
    return Sim_Set_Ok;
  } else if (!strcmp(command, "dump-cache" ) ) {
    int cpuNumber = val->u.integer;
    ruby_dump_cache(cpuNumber);
    return Sim_Set_Ok;
  } else if (!strcmp(command, "dump-cache-data" ) ) {
    int   cpuNumber = val->u.list.vector[0].u.integer;
    char *filename  = (char*) val->u.list.vector[1].u.string;
    ruby_dump_cache_data( cpuNumber, filename );
    return Sim_Set_Ok;
  } else if (!strcmp(command, "tracer-output-file" ) ) {
    char* new_filename = (char*) val->u.string;
    ruby_set_tracer_output_file(new_filename);
    return Sim_Set_Ok;
  } else if (!strcmp(command, "xact-visualizer-file" ) ) {
    char* new_filename = (char*) val->u.string;
    ruby_xact_visualizer_file(new_filename);
    return Sim_Set_Ok;
  }
  fprintf( stderr, "error: unrecognized command: %s\n", command );
#endif
  return Sim_Set_Illegal_Value;
}

static  initvar_t  *ruby_initvar_obj = NULL;

//***************************************************************************
static void init_generate_values( void )
{
  /* update generated values, based on input configuration */
}

//***************************************************************************
void init_variables( void )
{
  // allocate the "variable initialization" package
  ruby_initvar_obj = new initvar_t( "ruby", "../../../ruby/",
                                    global_default_param,
                                    &init_simulator,
                                    &init_generate_values,
                                    &ruby_session_get,
                                    &ruby_session_set );
}

void init_simulator()
{
  // Set things to NULL to make sure we don't de-reference them
  // without a seg. fault.
  g_system_ptr = NULL;
  g_debug_ptr = NULL;
  g_eventQueue_ptr = NULL;

  cout << "Ruby Timing Mode" << endl;

  if (g_SIMICS) {
    // LUKE - if we don't set the default SMT threads in condor scripts,
    //        set it now
    if(g_NUM_SMT_THREADS == 0){
      g_NUM_SMT_THREADS = 1;
    }
   if(g_NUM_PROCESSORS == 0){
     //only set to default if value not set in condor scripts
     //  Account for SMT systems also
      g_NUM_PROCESSORS = SIMICS_number_processors()/g_NUM_SMT_THREADS;
    }
  }

  RubyConfig::init();

  g_debug_ptr = new Debug( DEBUG_FILTER_STRING,
                           DEBUG_VERBOSITY_STRING,
                           DEBUG_START_TIME,
                           DEBUG_OUTPUT_FILENAME );

  cout << "Creating event queue..." << endl;
  g_eventQueue_ptr = new EventQueue;
  cout << "Creating event queue done" << endl;

  cout << "Creating system..." << endl;
  cout << "  Processors: " << RubyConfig::numberOfProcessors() << endl;

  g_system_ptr = new RubySystem;
  cout << "Creating system done" << endl;

  // if opal is loaded, its static interface object (inst) will be non-null,
  // and the opal object needs to be notified that ruby is now loaded.
  // "1" indicates a load and should be replaced with an enumerated type.
  if (OpalInterface::inst != NULL) {
    OpalInterface::inst->notify( 1 );
  }

#ifdef CONTIGUOUS_ADDRESSES
  if(g_SIMICS) {
    cout << "Establishing Contiguous Address Space Mappings..." << flush;
    g_p_ca_translator = new ContiguousAddressTranslator();
    assert(g_p_ca_translator!=NULL);
    if(g_p_ca_translator->AddressesAreContiguous()) {
      cout << "Physical Memory Addresses are already contiguous." << endl;
      delete g_p_ca_translator;
      g_p_ca_translator = NULL;
    } else {
      cout << "Done." << endl;
    }
  } else {
    g_p_ca_translator = NULL;
  }
#endif // #ifdef CONTIGUOUS_ADDRESSES

  cout << "Ruby initialization complete" << endl;
}

void init_opal_interface( mf_ruby_api_t *api )
{
  OpalInterface::installInterface( api );
}

int init_use_snoop()
{
  if (g_SIMICS) {
    // The "snoop interface" defined by simics allows ruby to see store
    // data values (from simics). If DATA_BLOCK is defined, we are tracking
    // data, so we need to install the snoop interface.
    return ((DATA_BLOCK == true) || (XACT_MEMORY));
  } else {
    return (0);
  }
}

void destroy_simulator()
{
  cout << "Deleting system..." << endl;
  delete g_system_ptr;
  cout << "Deleting system done" << endl;

  cout << "Deleting event queue..." << endl;
  delete g_eventQueue_ptr;
  cout << "Deleting event queue done" << endl;

  delete g_debug_ptr;
}

/*-------------------------------------------------------------------------+
 | DG: These are the external load and unload hooks that will be called by |
 | M5 in phase 1 integration, and possibly afterwards, too.                |
 +-------------------------------------------------------------------------*/

extern "C"
int OnLoadRuby() {
  init_variables();
  return 0;
}

extern "C"
int OnInitRuby() {
  init_simulator();
  return 0;
}

extern "C"
int OnUnloadRuby() {
  destroy_simulator();
  return 0;
}

