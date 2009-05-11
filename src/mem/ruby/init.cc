
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

#include "mem/ruby/common/Global.hh"
#include "mem/ruby/eventqueue/RubyEventQueue.hh"
#include "mem/ruby/system/System.hh"
#include "mem/ruby/common/Debug.hh"
#include "mem/ruby/common/Driver.hh"
#include "mem/ruby/profiler/Profiler.hh"
#include "mem/ruby/tester/Tester.hh"
#include "mem/ruby/init.hh"

using namespace std;
#include <string>
#include <map>
#include <stdlib.h>

#include "mem/gems_common/ioutil/confio.hh"
#include "mem/gems_common/ioutil/initvar.hh"

// A generated file containing the default parameters in string form
// The defaults are stored in the variable global_default_param
#include "mem/ruby/default_param.hh"

static initvar_t *ruby_initvar_obj = NULL;

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
                                    &init_generate_values);
}


 /*
void init_variables(const char* config_str )
{
  // allocate the "variable initialization" package
  ruby_initvar_obj = new initvar_t( "ruby", "../../../ruby/",
                                    config_str,
                                    &init_simulator,
                                    &init_generate_values );
}
 */

void init_simulator()
{
    // Set things to NULL to make sure we don't de-reference them
    // without a seg. fault.
    g_system_ptr = NULL;
    g_debug_ptr = NULL;
    g_eventQueue_ptr = NULL;

    cout << "Ruby Timing Mode" << endl;


    g_debug_ptr = new Debug( DEBUG_FILTER_STRING,
                             DEBUG_VERBOSITY_STRING,
                             DEBUG_START_TIME,
                             DEBUG_OUTPUT_FILENAME );
    RubyConfig::init();

    cout << "Creating event queue..." << endl;
    g_eventQueue_ptr = new RubyEventQueue;
    cout << "Creating event queue done" << endl;

    cout << "Creating system..." << endl;
    cout << "  Processors: " << RubyConfig::numberOfProcessors() << endl;

    g_system_ptr = new RubySystem;
    cout << "Creating system done" << endl;

    cout << "Ruby initialization complete" << endl;
}

void init_simulator(Driver* _driver)
{
    // Set things to NULL to make sure we don't de-reference them
    // without a seg. fault.
    g_system_ptr = NULL;
    g_debug_ptr = NULL;
    g_eventQueue_ptr = NULL;

    cout << "Ruby Timing Mode" << endl;


    g_debug_ptr = new Debug( DEBUG_FILTER_STRING,
                             DEBUG_VERBOSITY_STRING,
                             DEBUG_START_TIME,
                             DEBUG_OUTPUT_FILENAME );
    RubyConfig::init();

    cout << "Creating event queue..." << endl;
    g_eventQueue_ptr = new RubyEventQueue;
    cout << "Creating event queue done" << endl;

    cout << "Creating system..." << endl;
    cout << "  Processors: " << RubyConfig::numberOfProcessors() << endl;

    g_system_ptr = new RubySystem(_driver);
    cout << "Creating system done" << endl;

    cout << "Ruby initialization complete" << endl;
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

//dsm: superfluous
/*extern "C"
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
}*/

/* I have to put it somewhere for now */
void tester_main(int argc, char **argv) {
    std::cout << __FILE__ << "(" << __LINE__ << "): Not implemented." << std::endl;
}
