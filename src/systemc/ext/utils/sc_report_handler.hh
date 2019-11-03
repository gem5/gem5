/*
 * Copyright 2018 Google, Inc.
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
 *
 * Authors: Gabe Black
 */

#ifndef __SYSTEMC_EXT_UTIL_SC_REPORT_HANDLER_HH__
#define __SYSTEMC_EXT_UTIL_SC_REPORT_HANDLER_HH__

#include <string>

#include "messages.hh"
#include "sc_report.hh" // for sc_severity

namespace sc_core
{

typedef unsigned sc_actions;

enum
{
    SC_UNSPECIFIED = 0x0000,
    SC_DO_NOTHING = 0x0001,
    SC_THROW = 0x0002,
    SC_LOG = 0x0004,
    SC_DISPLAY = 0x0008,
    SC_CACHE_REPORT = 0x0010,
    SC_INTERRUPT = 0x0020,
    SC_STOP = 0x0040,
    SC_ABORT = 0x0080,

    // The spec says these should be macros, but that breaks the build for the
    // regression tests since they refer to, for instance,
    // sc_core::SC_DEFAULT_INFO_ACTIONS.
    SC_DEFAULT_INFO_ACTIONS = SC_LOG | SC_DISPLAY,
    SC_DEFAULT_WARNING_ACTIONS = SC_LOG | SC_DISPLAY,
    SC_DEFAULT_ERROR_ACTIONS = SC_LOG | SC_CACHE_REPORT | SC_THROW,
    SC_DEFAULT_FATAL_ACTIONS = SC_LOG | SC_DISPLAY | SC_CACHE_REPORT | SC_ABORT
};

typedef void (*sc_report_handler_proc)(const sc_report &, const sc_actions &);

class sc_report_handler
{
  public:
    static void report(sc_severity, const char *msg_type, const char *msg,
                       const char *file, int line);
    static void report(sc_severity, const char *msg_type, const char *msg,
                       int verbosity, const char *file, int line);

    // Deprecated
    static void report(sc_severity, int id, const char *msg, const char *file,
                       int line);

    static sc_actions set_actions(sc_severity, sc_actions=SC_UNSPECIFIED);
    static sc_actions set_actions(const char *msg_type,
                                  sc_actions=SC_UNSPECIFIED);
    static sc_actions set_actions(const char *msg_type, sc_severity,
                                  sc_actions=SC_UNSPECIFIED);

    static int stop_after(sc_severity, int limit=-1);
    static int stop_after(const char *msg_type, int limit=-1);
    static int stop_after(const char *msg_type, sc_severity, int limit=-1);

    static int get_count(sc_severity);
    static int get_count(const char *msg_type);
    static int get_count(const char *msg_type, sc_severity);

    // Nonstandard
    // In the spec, these aren't listed as static functions. They are static in
    // the Accellera implementation and are used as such in the tests.
    static int set_verbosity_level(int);
    static int get_verbosity_level();

    static sc_actions suppress(sc_actions);
    static sc_actions suppress();
    static sc_actions force(sc_actions);
    static sc_actions force();

    static sc_actions set_catch_actions(sc_actions);
    static sc_actions get_catch_actions();

    static void set_handler(sc_report_handler_proc);
    static void default_handler(const sc_report &, const sc_actions &);
    static sc_actions get_new_action_id();

    static sc_report *get_cached_report();
    static void clear_cached_report();

    static bool set_log_file_name(const char *);
    static const char *get_log_file_name();
};

#define SC_REPORT_INFO_VERB(msg_type, msg, verbosity) \
        ::sc_core::sc_report_handler::report( \
            ::sc_core::SC_INFO, msg_type, msg, verbosity, __FILE__, __LINE__)

#define SC_REPORT_INFO(msg_type, msg) \
        ::sc_core::sc_report_handler::report( \
            ::sc_core::SC_INFO, msg_type, msg, __FILE__, __LINE__)

#define SC_REPORT_WARNING(msg_type, msg) \
        ::sc_core::sc_report_handler::report( \
            ::sc_core::SC_WARNING, msg_type, msg, __FILE__, __LINE__)

#define SC_REPORT_ERROR(msg_type, msg) \
        ::sc_core::sc_report_handler::report( \
            ::sc_core::SC_ERROR, msg_type, msg, __FILE__, __LINE__)

#define SC_REPORT_FATAL(msg_type, msg) \
        ::sc_core::sc_report_handler::report( \
            ::sc_core::SC_FATAL, msg_type, msg, __FILE__, __LINE__)

#define sc_assert(expr) \
        ((void)((expr) ? 0 : (SC_REPORT_FATAL( \
            ::sc_core::SC_ID_ASSERTION_FAILED_, #expr), 0)))

void sc_interrupt_here(const char *msg_type, sc_severity);
void sc_stop_here(const char *msg_type, sc_severity);

// Nonstandard
// From Accellera, "not documented, but available".
const std::string sc_report_compose_message(const sc_report &);
bool sc_report_close_default_log();

} // namespace sc_core

#endif  //__SYSTEMC_EXT_UTIL_SC_REPORT_HANDLER_HH__
