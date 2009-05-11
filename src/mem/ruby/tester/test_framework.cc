
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
 * $Id$
 *
 */

#include "protocol_name.hh"
#include "test_framework.hh"
#include "System.hh"
#include "OpalInterface.hh"
#include "init.hh"
#include "Tester.hh"
#include "EventQueue.hh"
#include "getopt.hh"
#include "Network.hh"
#include "CacheRecorder.hh"
#include "Tracer.hh"

using namespace std;
#include <string>
#include <map>

// Maurice
// extern "C" {
// #include "simics/api.hh"
// };

#include "confio.hh"
#include "initvar.hh"

// A generated file containing the default tester parameters in string form
// The defaults are stored in the variables
// global_default_param and global_default_tester_param
#include "default_param.hh"
#include "tester_param.hh"

static void parseOptions(int argc, char **argv);
static void usageInstructions();
static void checkArg(char ch);
static void tester_record_cache();
static void tester_playback_trace();
static void tester_initialize(int argc, char **argv);
static void tester_destroy();

static string trace_filename;
char * my_default_param;
initvar_t * my_initvar;

void tester_main(int argc, char **argv)
{
  tester_initialize(argc, argv);

  if (trace_filename != "") {
    // playback a trace (for multicast-mask prediction)
    tester_playback_trace();
  } else {
    // test code to create a trace
    if (!(g_SYNTHETIC_DRIVER || g_DETERMINISTIC_DRIVER) && trace_filename == "") {
      g_system_ptr->getTracer()->startTrace("ruby.trace.gz");
      g_eventQueue_ptr->triggerEvents(g_eventQueue_ptr->getTime() + 10000);
      g_system_ptr->getTracer()->stopTrace();
    }

    g_eventQueue_ptr->triggerAllEvents();

    // This call is placed here to make sure the cache dump code doesn't fall victim to code rot
    if (!(g_SYNTHETIC_DRIVER || g_DETERMINISTIC_DRIVER)) {
      tester_record_cache();
    }
  }
  tester_destroy();
}

static void tester_allocate( void )
{
  init_simulator();
}

static void tester_generate_values( void )
{
}

void tester_initialize(int argc, char **argv)
{
  int   param_len = strlen( global_default_param ) + strlen( global_default_tester_param ) + 1;
  char *default_param = (char *) malloc( sizeof(char) * param_len );
  my_default_param = default_param;
  strcpy( default_param, global_default_param );
  strcat( default_param, global_default_tester_param );

  // when the initvar object is created, it reads the configuration default
  //   -for the tester, the configuration defaults in config/tester.defaults

  /** note: default_param is included twice in the tester:
   *       -once in init.C
   *       -again in this file
   */
  initvar_t *ruby_initvar = new initvar_t( "ruby", "../../../ruby/",
                                           default_param,
                                           &tester_allocate,
                                           &tester_generate_values,
                                           NULL,
                                           NULL );
  my_initvar = ruby_initvar;
  ruby_initvar->checkInitialization();
  parseOptions(argc, argv);

  ruby_initvar->allocate();

  g_system_ptr->printConfig(cout);
  cout << "Testing clear stats...";
  g_system_ptr->clearStats();
  cout << "Done." << endl;
  //free( default_param );
  //delete ruby_initvar;
}

void tester_destroy()
{
  g_system_ptr->printStats(cout);
  g_debug_ptr->closeDebugOutputFile();

  free(my_default_param);
  delete my_initvar;
  // Clean up
  destroy_simulator();
  cerr << "Success: " << CURRENT_PROTOCOL << endl;
}

void tester_install_opal(mf_opal_api_t* opal_api, mf_ruby_api_t* ruby_api)
{
  // initialize our api interface
  OpalInterface::installInterface(ruby_api);

  // update the OpalInterface object to point to opal's interface
  ((OpalInterface *) g_system_ptr->getDriver())->setOpalInterface(opal_api);
}

void tester_record_cache()
{
  cout << "Testing recording of cache contents" << endl;
  CacheRecorder recorder;
  g_system_ptr->recordCacheContents(recorder);
  int written = recorder.dumpRecords("ruby.caches.gz");
  int read = Tracer::playbackTrace("ruby.caches.gz");
  assert(read == written);
  cout << "Testing recording of cache contents completed" << endl;
}

void tester_playback_trace()
{
  assert(trace_filename != "");
  cout << "Reading trace from file '" << trace_filename << "'..." << endl;
  int read = Tracer::playbackTrace(trace_filename);
  cout << "(" << read << " requests read)" << endl;
  if (read == 0) {
    ERROR_MSG("Zero items read from tracefile.");
  }
}

// ************************************************************************
// *** Functions for parsing the command line parameters for the tester ***
// ************************************************************************

static struct option const long_options[] =
{
  {"help", no_argument, NULL, 'h'},
  {"processors", required_argument, NULL, 'p'},
  {"length", required_argument, NULL, 'l'},
  {"random", required_argument, NULL, 'r'},
  {"trace_input", required_argument, NULL, 'z'},
  {"component", required_argument, NULL, 'c'},
  {"verbosity", required_argument, NULL, 'v'},
  {"debug_output_file", required_argument, NULL, 'o'},
  {"start", required_argument, NULL, 's'},
  {"bandwidth", required_argument, NULL, 'b'},
  {"threshold", required_argument, NULL, 't'},
  {"think_time", required_argument, NULL, 'k'},
  {"locks", required_argument, NULL, 'q'},
  {"network", required_argument, NULL, 'n'},
  {"procs_per_chip", required_argument, NULL, 'a'},
  {"l2_caches", required_argument, NULL, 'e'},
  {"memories", required_argument, NULL, 'm'},
  {NULL, 0, NULL, 0}
};

static void parseOptions(int argc, char **argv)
{
  cout << "Parsing command line arguments:" << endl;

  // construct the short arguments string
  int counter = 0;
  string short_options;
  while (long_options[counter].name != NULL) {
    short_options += char(long_options[counter].val);
    if (long_options[counter].has_arg == required_argument) {
      short_options += char(':');
    }
    counter++;
  }

  char c;
  /* Parse command line options.  */
  bool error;
  while ((c = getopt_long (argc, argv, short_options.c_str(), long_options, (int *) 0)) != EOF) {
    switch (c) {
    case 0:
      break;

    case 'c':
      checkArg(c);
      cout << "  component filter string = " << optarg << endl;
      error = Debug::checkFilterString( optarg );
      if (error) {
        usageInstructions();
      }
      DEBUG_FILTER_STRING = strdup( optarg );
      break;

    case 'h':
      usageInstructions();
      break;

    case 'v':
      checkArg(c);
      cout << "  verbosity string = " << optarg << endl;
      error = Debug::checkVerbosityString(optarg);
      if (error) {
        usageInstructions();
      }
      DEBUG_VERBOSITY_STRING = strdup( optarg );
      break;

    case 'r': {
      checkArg(c);
      if (string(optarg) == "random") {
        g_RANDOM_SEED = time(NULL);
      } else {
        g_RANDOM_SEED = atoi(optarg);
        if (g_RANDOM_SEED == 0) {
          usageInstructions();
        }
      }
      break;
    }

    case 'l': {
      checkArg(c);
      g_tester_length = atoi(optarg);
      cout << "  length of run = " << g_tester_length << endl;
      if (g_tester_length == 0) {
        usageInstructions();
      }
      break;
    }

    case 'q': {
      checkArg(c);
      g_synthetic_locks = atoi(optarg);
      cout << "  locks in synthetic workload = " << g_synthetic_locks << endl;
      if (g_synthetic_locks == 0) {
        usageInstructions();
      }
      break;
    }

    case 'p': {
      checkArg(c);
      g_NUM_PROCESSORS = atoi(optarg);
      break;
    }

    case 'a': {
      checkArg(c);
      g_PROCS_PER_CHIP = atoi(optarg);
      cout << "  g_PROCS_PER_CHIP: " << g_PROCS_PER_CHIP << endl;
      break;
    }

    case 'e': {
      checkArg(c);
      g_NUM_L2_BANKS = atoi(optarg);
      cout << "  g_NUM_L2_BANKS: " << g_NUM_L2_BANKS << endl;
      break;
    }

    case 'm': {
      checkArg(c);
      g_NUM_MEMORIES = atoi(optarg);
      cout << "  g_NUM_MEMORIES: " << g_NUM_MEMORIES << endl;
      break;
    }

    case 's': {
      checkArg(c);
      long long start_time = atoll(optarg);
      cout << "  debug start cycle = " << start_time << endl;
      if (start_time == 0) {
        usageInstructions();
      }
      DEBUG_START_TIME = start_time;
      break;
    }

    case 'b': {
      checkArg(c);
      int bandwidth = atoi(optarg);
      cout << "  bandwidth per link (MB/sec) = " << bandwidth << endl;
      g_endpoint_bandwidth = bandwidth;
      if (bandwidth == 0) {
        usageInstructions();
      }
      break;
    }

    case 't': {
      checkArg(c);
      g_bash_bandwidth_adaptive_threshold = atof(optarg);
      if ((g_bash_bandwidth_adaptive_threshold > 1.1) || (g_bash_bandwidth_adaptive_threshold < -0.1)) {
        cerr << "Error: Bandwidth adaptive threshold must be between 0.0 and 1.0" << endl;
        usageInstructions();
      }

      break;
    }

    case 'k': {
      checkArg(c);
      g_think_time = atoi(optarg);
      break;
    }

    case 'o':
      checkArg(c);
      cout << "  output file = " << optarg << endl;
      DEBUG_OUTPUT_FILENAME = strdup( optarg );
      break;

    case 'z':
      checkArg(c);
      trace_filename = string(optarg);
      cout << "  tracefile = " << trace_filename << endl;
      break;

      case 'n':
        checkArg(c);
        cout << "  topology = " << string(optarg) << endl;
        g_NETWORK_TOPOLOGY = strdup(optarg);
        break;

    default:
      cerr << "parameter '" << c << "' unknown" << endl;
      usageInstructions();
    }
  }

  if ((trace_filename != "") || (g_tester_length != 0)) {
    if ((trace_filename != "") && (g_tester_length != 0)) {
      cerr << "Error: both a run length (-l) and a trace file (-z) have been specified." << endl;
      usageInstructions();
    }
  } else {
    cerr << "Error: either run length (-l) must be > 0 or a trace file (-z) must be specified." << endl;
    usageInstructions();
  }
}

static void usageInstructions()
{
  cerr << endl;
  cerr << "Options:" << endl;

  // print options
  int counter = 0;
  while (long_options[counter].name != NULL) {
    cerr << "  -" << char(long_options[counter].val);
    if (long_options[counter].has_arg == required_argument) {
      cerr << " <arg>";
    }
    cerr << "  --" << long_options[counter].name;
    if (long_options[counter].has_arg == required_argument) {
      cerr << " <arg>";
    }
    cerr << endl;
    counter++;
  }

  cerr << "Option --processors (-p) is required." << endl;
  cerr << "Either option --length (-l) or --trace_input (-z) must be specified." << endl;
  cerr << endl;
  g_debug_ptr->usageInstructions();
  cerr << endl;

  exit(1);
}

static void checkArg(char ch)
{
  if (optarg == NULL) {
    cerr << "Error: parameter '" << ch << "' missing required argument" << endl;
    usageInstructions();
  }
}
