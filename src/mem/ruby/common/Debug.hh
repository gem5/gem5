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

#ifndef __MEM_RUBY_COMMON_DEBUG_HH__
#define __MEM_RUBY_COMMON_DEBUG_HH__

#include <unistd.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "mem/ruby/common/Global.hh"
#include "sim/sim_object.hh"

#include "params/RubyDebug.hh"

extern std::ostream * debug_cout_ptr;

// component enumeration
enum DebugComponents
{
    SYSTEM_COMP,
    NODE_COMP,
    QUEUE_COMP,
    EVENTQUEUE_COMP,
    NETWORK_COMP,
    SEQUENCER_COMP,
    TESTER_COMP,
    GENERATED_COMP,
    SLICC_COMP,
    NETWORKQUEUE_COMP,
    TIME_COMP,
    NETWORK_INTERNALS_COMP,
    STOREBUFFER_COMP,
    CACHE_COMP,
    PREDICTOR_COMP,
    MEMORY_COMP,
    NUMBER_OF_COMPS
};

enum PriorityLevel {HighPrio, MedPrio, LowPrio};
enum VerbosityLevel {No_Verb, Low_Verb, Med_Verb, High_Verb};

class Debug : public SimObject
{
  public:
    typedef RubyDebugParams Params;
    Debug(const Params *p);
    ~Debug();

    static bool getProtocolTrace() { return m_protocol_trace; }
    bool validDebug(int module, PriorityLevel priority);
    void printVerbosity(std::ostream& out) const;
    void setVerbosity(VerbosityLevel vb);
    bool setVerbosityString(const char *);
    VerbosityLevel getVerbosity() const { return m_verbosityLevel; }
    void setFilter(int);
    static bool checkFilter( char);
    static bool checkFilterString(const char *);
    bool setFilterString(const char *);
    void setDebugTime(Time);
    Time getDebugTime() const { return m_starting_cycle; }
    bool addFilter(char);
    void clearFilter();
    void allFilter();
    void print(std::ostream& out) const;
    /* old school debugging "vararg": sends messages to screen and log */
    void debugMsg(const char *fmt, ...);

    void setDebugOutputFile (const char * filename);
    void closeDebugOutputFile ();
    static void usageInstructions(void);

  private:
    // Private copy constructor and assignment operator
    Debug(const Debug& obj);
    Debug& operator=(const Debug& obj);

    static bool m_protocol_trace;
    VerbosityLevel m_verbosityLevel;
    int m_filter;
    Time m_starting_cycle;

    std::fstream m_fout;
};

inline std::ostream&
operator<<(std::ostream& out, const Debug& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

const bool ERROR_MESSAGE_FLAG = true;
const bool WARNING_MESSAGE_FLAG = true;

#ifdef RUBY_NO_ASSERT
const bool ASSERT_FLAG = false;
#else
const bool ASSERT_FLAG = true;
#endif

#undef assert
#define assert(EXPR) ASSERT(EXPR)
#undef ASSERT
#define ASSERT(EXPR) do {                                               \
    using namespace std;                                                \
    if (ASSERT_FLAG) {                                                  \
        if (!(EXPR)) {                                                  \
            cerr << "failed assertion '"                                \
                 << #EXPR << "' at fn "                                 \
                 << __PRETTY_FUNCTION__ << " in "                       \
                 << __FILE__ << ":"                                     \
                 << __LINE__ << endl << flush;                          \
            (*debug_cout_ptr) << "failed assertion '"                   \
                               << #EXPR << "' at fn "                   \
                               << __PRETTY_FUNCTION__ << " in "         \
                               << __FILE__ << ":"                       \
                               << __LINE__ << endl << flush;            \
            if (isatty(STDIN_FILENO)) {                                 \
                cerr << "At this point you might want to attach a debug to " \
                     << "the running and get to the" << endl            \
                     << "crash site; otherwise press enter to continue" \
                     << endl                                            \
                     << "PID: " << getpid()                             \
                     << endl << flush;                                  \
                char c;                                                 \
                cin.get(c);                                             \
            }                                                           \
            abort();                                                    \
        }                                                               \
    }                                                                   \
} while (0)

#define BREAK(X) do {                                   \
    using namespace std;                                \
    cerr << "breakpoint '"                              \
         << #X << "' reached at fn "                    \
         << __PRETTY_FUNCTION__ << " in "               \
         << __FILE__ << ":"                             \
         << __LINE__ << endl << flush;                  \
    if(isatty(STDIN_FILENO)) {                          \
        cerr << "press enter to continue" << endl;      \
        cerr << "PID: " << getpid();                    \
        cerr << endl << flush;                          \
        char c;                                         \
        cin.get(c);                                     \
    }                                                   \
} while (0)

#define ERROR_MSG(MESSAGE) do {                                 \
    using namespace std;                                        \
    if (ERROR_MESSAGE_FLAG) {                                   \
        cerr << "Fatal Error: in fn "                           \
             << __PRETTY_FUNCTION__ << " in "                   \
             << __FILE__ << ":"                                 \
             << __LINE__ << ": "                                \
             << (MESSAGE) << endl << flush;                     \
        (* debug_cout_ptr) << "Fatal Error: in fn "             \
                           << __PRETTY_FUNCTION__ << " in "     \
                           << __FILE__ << ":"                   \
                           << __LINE__ << ": "                  \
                           << (MESSAGE) << endl << flush;       \
        abort();                                                \
    }                                                           \
} while(0)

#define WARN_MSG(MESSAGE) do {                                  \
    using namespace std;                                        \
    if (WARNING_MESSAGE_FLAG) {                                 \
        cerr << "Warning: in fn "                               \
             << __PRETTY_FUNCTION__ << " in "                   \
             << __FILE__ << ":"                                 \
             << __LINE__ << ": "                                \
             << (MESSAGE) << endl << flush;                     \
        (* debug_cout_ptr) << "Warning: in fn "                 \
                           << __PRETTY_FUNCTION__ << " in "     \
                           << __FILE__ << ":"                   \
                           << __LINE__ << ": "                  \
                           << (MESSAGE) << endl << flush;       \
    }                                                           \
} while (0)

#define WARN_EXPR(EXPR) do {                                    \
    using namespace std;                                        \
    if (WARNING_MESSAGE_FLAG) {                                 \
        cerr << "Warning: in fn "                               \
             << __PRETTY_FUNCTION__ << " in "                   \
             << __FILE__ << ":"                                 \
             << __LINE__ << ": "                                \
             << #EXPR << " is "                                 \
             << (EXPR) << endl << flush;                        \
        (* debug_cout_ptr) << "Warning: in fn "                 \
                           << __PRETTY_FUNCTION__ << " in "     \
                           << __FILE__ << ":"                   \
                           << __LINE__ << ": "                  \
                           << #EXPR << " is "                   \
                           << (EXPR) << endl << flush;          \
    }                                                           \
} while (0)

#define ERROR_OUT( rest... ) do {               \
    using namespace std;                        \
    if (ERROR_MESSAGE_FLAG) {                   \
        cout << "error: in fn "                 \
             << __PRETTY_FUNCTION__ << " in "   \
             << __FILE__ << ":"                 \
             << __LINE__ << ": ";               \
        g_debug_ptr->debugMsg(rest);            \
    }                                           \
} while (0)

#endif // __MEM_RUBY_COMMON_DEBUG_HH__

