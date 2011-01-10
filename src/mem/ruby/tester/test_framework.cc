
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

using namespace std;

#include "base/misc.hh"
#include "mem/ruby/tester/test_framework.hh"
#include "mem/protocol/protocol_name.hh"
#include "getopt.hh"
#include "mem/ruby/tester/DeterministicDriver.hh"
#include "mem/ruby/tester/RaceyDriver.hh"
#include "mem/ruby/common/Driver.hh"
#include "mem/ruby/recorder/Tracer.hh"


#include <string>
#include <map>
#include <iostream>
#include <assert.h>
#include <vector>
#include <string>
#include <sstream>
#include <sys/wait.h>

#include "mem/ruby/libruby.hh"

// FIXME: should really make this a class if can't figure out how to make a function to get the ruby parameter
static void set_defaults();
static void parseOptions(int argc, char **argv);
static void usageInstructions();
static void checkArg(char ch);
static void tester_initialize(int argc, char **argv);
static void tester_playback_trace();
static void tester_destroy();
static void hit_callback(int64_t request_id);

// Tester variables
string driver_type;
string generator_type;
Driver * m_driver_ptr;
int g_tester_length;
int num_completions;
Time g_think_time;
Time g_wait_time;
int num_procs;
static string trace_filename;
bool replaying;

void tester_main(int argc, char **argv)
{
  tester_initialize(argc, argv);
  
  if (trace_filename != "") {
    // playback a trace (for multicast-mask prediction)
    replaying = true;
    tester_playback_trace();
  }


  tester_destroy();
}

vector<string> tokenizeMyString(string str, string delims)
{
  vector<string> tokens;
  char* pch;
  char* tmp;
  const char* c_delims = delims.c_str();
  tmp = new char[str.length()+1];
  strcpy(tmp, str.c_str());
  pch = strtok(tmp, c_delims);
  while (pch != NULL) {
    tokens.push_back(string(pch));
    pch = strtok(NULL, c_delims);
  }
  delete [] tmp;
  return tokens;
}


vector<string> getPorts(const char* cfg_script, int cfg_script_argc, char* cfg_script_argv[])
{
  stringstream cfg_output;

  // first we execute the Ruby-lang configuration script
  int fd[2];
  int pid;
  if (pipe(fd) == -1) {
    perror("Error Creating Pipe");
    exit(EXIT_FAILURE);
  }
  
  pid = fork();
  if (pid == -1){
    perror("Error forking");
    exit(EXIT_FAILURE);
  }

  if (!pid) {
    // child
    close(fd[0]); // close the read end of the pipe
    // replace stdout with the write pipe
    if (dup2(fd[1], STDOUT_FILENO) == -1) {
      perror("Error redirecting stdout");
      exit(EXIT_FAILURE);
    }
#define QUOTE_MACRO(x, y) QUOTE_TXT(x,y)
#define QUOTE_TXT(x, y) #x y
    if (execlp("ruby", "ruby", "-I", QUOTE_MACRO(GEMS_ROOT, "/ruby/config"), QUOTE_MACRO(GEMS_ROOT, "/tests/list_ports.rb"), cfg_script, NULL)) {
      perror("execlp");
      exit(EXIT_FAILURE);
    }
  } else {
    close(fd[1]);   

    int child_status;
    if (wait(&child_status) == -1) {
      perror("wait");
      exit(EXIT_FAILURE);
    }
    if (child_status != EXIT_SUCCESS) {
      exit(EXIT_FAILURE);
    }
    
    char buf[100];
    int bytes_read;
    while( (bytes_read = read(fd[0], buf, 100)) > 0 ) {
      for (int i=0;i<bytes_read;i++) {
	cfg_output << buf[i];
      }
    }
    assert(bytes_read == 0);
    close(fd[0]);
  }
  string line;
  getline(cfg_output, line);

  return tokenizeMyString(line, " ");
}



void tester_initialize(int argc, char **argv)
{
  const char* cfg_file = argv[1];

  set_defaults();
  parseOptions(argc, argv);

  libruby_init(cfg_file);
  libruby_print_config(std::cout);

  vector<string> port_names = getPorts(cfg_file, 0, NULL);
  vector<RubyPortHandle> ports;

  for (vector<string>::const_iterator it = port_names.begin(); it != port_names.end(); it++)
    ports.push_back(libruby_get_port((*it).c_str(), hit_callback));
  
  if (driver_type == "Deterministic") {
    m_driver_ptr = new DeterministicDriver(generator_type, num_completions, num_procs, g_think_time, g_wait_time, g_tester_length);
  }
  else if (driver_type == "Racey") {
    m_driver_ptr = new RaceyDriver(num_procs, g_tester_length);
  }
 /* else if (driver_type == "Synthetic") {
    m_driver_ptr = new SyntheticDriver();
  }
  }*/

  if (trace_filename == "") {
    m_driver_ptr->go();
  }
}

void tester_playback_trace()
{
  replaying = true;
  assert(trace_filename != "");
  cout << "Reading trace from file '" << trace_filename << "'..." << endl;
  Tracer * replayer = new Tracer("noname");
  int read = replayer->playbackTrace(trace_filename);
  cout << "(" << read << " requests read)" << endl;
  if (read == 0) {
    fatal("Zero items read from tracefile.");
  }
}

void tester_destroy()
{
  m_driver_ptr->printStats(cout);
  libruby_destroy();
  cerr << "Success: " << CURRENT_PROTOCOL << endl;
}


void hit_callback(int64_t request_id)
{
  if (!replaying) {
    m_driver_ptr->hitCallback(request_id); 
  }
}

// ************************************************************************
// *** Functions for parsing the command line parameters for the tester ***
// ************************************************************************



static struct option const long_options[] =
{
  {"help", no_argument, NULL, 'h'},
  {"number of processors", required_argument, NULL, 'p'},
  {"test run length", required_argument, NULL, 'l'},
  {"generator think time", required_argument, NULL, 'k'},
  {"generator wait time", required_argument, NULL, 'w'},
  {"driver type", required_argument, NULL, 'd'},
  {"generator type", required_argument, NULL, 'g'},
  {"num completions before pass", required_argument, NULL, 'n'},
  {"test tracer", required_argument, NULL, 'z'},
  {NULL, 0, NULL, 0}
};


// This is awkward and temporary, need the defaults config, and also need functions to
// just lookup a parameter in the configuration file
// Ideally the default values are set by libruby_init and then a function is provided to 
// set values at run-time
static void set_defaults() {
  replaying = false;
  g_tester_length = 0;
  g_think_time = 10;
  g_wait_time = 10;
  
  num_procs = 1;
  trace_filename = "";
  num_completions = 1;
  driver_type = "Deterministic";
  generator_type = "DetermSeriesGETSGenerator";
}

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
  while ((c = getopt_long (argc, argv, short_options.c_str(), long_options, (int *) 0)) != EOF) {
    switch (c) {
    case 0:
      break;
    case 'h':
      usageInstructions();
      break;
    case 'p':
      checkArg(c);
      cout << "  number of processors = " << optarg << endl;
      num_procs = atoi( optarg );
      break;
    case 'l': {
      checkArg(c);
      g_tester_length = atoi(optarg);
      cout << "  length of run = " << g_tester_length << endl;
      if (g_tester_length == 0) {
        usageInstructions();
      }
      break;
    }
    case 'k': {
      checkArg(c);
      g_think_time = atoi(optarg);
      break;
    }
    case 'w': {
      checkArg(c);
      g_wait_time = atoi(optarg);
      break;
    }
    case 'd':
      checkArg(c);
      cout << "  driver type = " << optarg << endl;
      driver_type = strdup( optarg );
      break;
    case 'g':
      checkArg(c);
      cout << "  generator type = " << optarg << endl;
      generator_type = strdup( optarg );
      break;
    case 'n':
      checkArg(c);
      cout << "  num completions before pass = " << optarg << endl;
      num_completions = atoi( optarg );
      break;
   case 'z': 
      checkArg(c);
      trace_filename = string(optarg);
      cout << "  tracefile = " << trace_filename << endl;
      break;
     
    default:
      cerr << "parameter '" << c << "' unknown" << endl;
      usageInstructions();
    }
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
