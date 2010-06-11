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

#include <fstream>
#include <stdarg.h>

#include "base/misc.hh"
#include "mem/ruby/common/Debug.hh"
#include "mem/ruby/common/Global.hh"
#include "mem/ruby/eventqueue/RubyEventQueue.hh"

using namespace std;

class Debug;
extern Debug* g_debug_ptr;
ostream *debug_cout_ptr;

bool Debug::m_protocol_trace = false;
struct DebugComponentData
{
    const char *desc;
    const char ch;
};

// component character list
DebugComponentData debugComponents[] =
{
    {"System",            's' },
    {"Node",              'N' },
    {"Queue",             'q' },
    {"Event Queue",       'e' },
    {"Network",           'n' },
    {"Sequencer",         'S' },
    {"Tester",            't' },
    {"Generated",         'g' },
    {"SLICC",             'l' },
    {"Network Queues",    'Q' },
    {"Time",              'T' },
    {"Network Internals", 'i' },
    {"Store Buffer",      'b' },
    {"Cache",             'c' },
    {"Predictor",         'p' },
    {"Memory",            'M' },
};

extern "C" void changeDebugVerbosity(VerbosityLevel vb);
extern "C" void changeDebugFilter(int filter);

void
changeDebugVerbosity(VerbosityLevel vb)
{
    g_debug_ptr->setVerbosity(vb);
}

void
changeDebugFilter(int filter)
{
    g_debug_ptr->setFilter(filter);
}

Debug::Debug(const Params *p)
    : SimObject(p)
{
    clearFilter();
    debug_cout_ptr = &cout;

    setFilterString(p->filter_string.c_str());
    setVerbosityString(p->verbosity_string.c_str());
    setDebugOutputFile(p->output_filename.c_str());
    m_starting_cycle = p->start_time;
    m_protocol_trace = p->protocol_trace;
    g_debug_ptr = this;
}

Debug::~Debug()
{
}

void
Debug::printVerbosity(ostream& out) const
{
    switch (getVerbosity()) {
      case No_Verb:
        out << "verbosity = No_Verb" << endl;
        break;
      case Low_Verb:
        out << "verbosity = Low_Verb" << endl;
        break;
      case Med_Verb:
        out << "verbosity = Med_Verb" << endl;
        break;
      case High_Verb:
        out << "verbosity = High_Verb" << endl;
        break;
      default:
        out << "verbosity = unknown" << endl;
    }
}

bool
Debug::validDebug(int module, PriorityLevel priority)
{
    int local_module = (1 << module);
    if (m_filter & local_module) {
        if (g_eventQueue_ptr == NULL ||
            g_eventQueue_ptr->getTime() >= m_starting_cycle) {
            switch (m_verbosityLevel) {
              case No_Verb:
                return false;
              case Low_Verb:
                return (priority == HighPrio);
              case Med_Verb:
                return (priority == HighPrio || priority == MedPrio);
              case High_Verb:
                return true;
            }
        }
    }
    return false;
}

void
Debug::setDebugTime(Time t)
{
    m_starting_cycle = t;
}

void
Debug::setVerbosity(VerbosityLevel vb)
{
    m_verbosityLevel = vb;
}

void
Debug::setFilter(int filter)
{
    m_filter = filter;
}

bool
Debug::setVerbosityString(const char *verb_str)
{
    string verb = verb_str ? verb_str : "";
    if (verb == "none") {
        setVerbosity(No_Verb);
    } else if (verb == "low") {
        setVerbosity(Low_Verb);
    } else if (verb == "med") {
        setVerbosity(Med_Verb);
    } else if (verb == "high") {
        setVerbosity(High_Verb);
    } else {
        cerr << "Error: unrecognized verbosity (use none, low, med, high): "
             << verb << endl;
        return true; // error
    }
    return false; // no error
}

bool
Debug::checkFilter(char ch)
{
    for (int i = 0; i < NUMBER_OF_COMPS; i++) {
        // Look at all components to find a character match
        if (debugComponents[i].ch == ch) {
            // We found a match - return no error
            return false; // no error
        }
    }
    return true; // error
}

bool
Debug::checkFilterString(const char *filter_str)
{
    if (filter_str == NULL) {
        cerr << "Error: unrecognized component filter: NULL" << endl;
        return true; // error
    }

    // check for default filter ("none") before reporting RUBY_DEBUG error
    if (string(filter_str) == "none") {
        return false; // no error
    }

    if (RUBY_DEBUG == false) {
        cerr << "Error: User specified set of debug components, but the "
             << "RUBY_DEBUG compile-time flag is false." << endl
             << "Solution: Re-compile with RUBY_DEBUG set to true." << endl;
        return true; // error
    }

    if (string(filter_str) == "all") {
        return false; // no error
    }

    // scan string checking each character
    for (unsigned int i = 0; i < strlen(filter_str); i++) {
        bool unrecognized = checkFilter(filter_str[i]);
        if (unrecognized == true) {
            return true; // error
        }
    }
    return false; // no error
}

bool
Debug::setFilterString(const char *filter_str)
{
    if (checkFilterString(filter_str)) {
        return true; // error
    }

    if (string(filter_str) == "all" ) {
        allFilter();
    } else if (string(filter_str) == "none") {
        clearFilter();
    } else {
        // scan string adding to bit mask for each component which is present
        for (unsigned int i = 0; i < strlen(filter_str); i++) {
            bool error = addFilter( filter_str[i] );
            if (error) {
                return true; // error
            }
        }
    }
    return false; // no error
}

bool
Debug::addFilter(char ch)
{
    for (int i = 0; i < NUMBER_OF_COMPS; i++) {
        // Look at all components to find a character match
        if (debugComponents[i].ch == ch) {
            // We found a match - update the filter bit mask
            cout << "  Debug: Adding to filter: '" << ch << "' ("
                 << debugComponents[i].desc << ")" << endl;
            m_filter |= (1 << i);
            return false; // no error
        }
    }

    // We didn't find the character
    cerr << "Error: unrecognized component filter: " << ch << endl;
    usageInstructions();
    return true; // error
}

void
Debug::clearFilter()
{
    m_filter = 0;
}

void Debug::allFilter()
{
    m_filter = ~0;
}

void
Debug::usageInstructions(void)
{
    cerr << "Debug components: " << endl;
    for (int i = 0; i < NUMBER_OF_COMPS; i++) {
        cerr << "  " << debugComponents[i].ch << ": "
             << debugComponents[i].desc << endl;
    }
}

void
Debug::print(ostream& out) const
{
    out << "[Debug]" << endl;
}

void
Debug::setDebugOutputFile (const char *filename)
{
    if (filename == NULL || !strcmp(filename, "none")) {
        debug_cout_ptr = &cout;
        return;
    }

    if (m_fout.is_open()) {
        m_fout.close();
    }
    m_fout.open(filename, ios::out);
    if (!m_fout.is_open()) {
        cerr << "setDebugOutputFile: can't open file " << filename << endl;
    } else {
        debug_cout_ptr = &m_fout;
    }
}

void
Debug::closeDebugOutputFile ()
{
    if (m_fout.is_open()) {
        m_fout.close ();
        debug_cout_ptr = &cout;
    }
}

void
Debug::debugMsg( const char *fmt, ...)
{
    va_list  args;

    // you could check validDebug() here before printing the message
    va_start(args, fmt);
    vfprintf(stdout, fmt, args);
    va_end(args);
}

Debug *
RubyDebugParams::create()
{
    return new Debug(this);
}
