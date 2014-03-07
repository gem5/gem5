/*
 * Copyright (c) 2014 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 *
 * Authors: Nathan Binkert
 *          Dave Greene
 */

#ifndef __BASE_MISC_HH__
#define __BASE_MISC_HH__

#include "base/compiler.hh"
#include "base/cprintf.hh"
#include "base/varargs.hh"

#if defined(__SUNPRO_CC)
#define __FUNCTION__ "how to fix me?"
#endif

// General exit message, these functions will never return and will
// either abort() if code is < 0 or exit with the code if >= 0
void __exit_message(const char *prefix, int code,
    const char *func, const char *file, int line,
    const char *format, CPRINTF_DECLARATION) M5_ATTR_NORETURN;

void __exit_message(const char *prefix, int code,
    const char *func, const char *file, int line,
    const std::string &format, CPRINTF_DECLARATION) M5_ATTR_NORETURN;

inline void
__exit_message(const char *prefix, int code,
    const char *func, const char *file, int line,
    const std::string& format, CPRINTF_DEFINITION)
{
    __exit_message(prefix, code, func, file, line, format.c_str(),
                   VARARGS_ALLARGS);
}

M5_PRAGMA_NORETURN(__exit_message)
#define exit_message(prefix, code, ...)                            \
    __exit_message(prefix, code, __FUNCTION__, __FILE__, __LINE__, \
                   __VA_ARGS__)

//
// This implements a cprintf based panic() function.  panic() should
// be called when something happens that should never ever happen
// regardless of what the user does (i.e., an acutal m5 bug).  panic()
// calls abort which can dump core or enter the debugger.
//
//
#define panic(...) exit_message("panic", -1, __VA_ARGS__)

//
// This implements a cprintf based fatal() function.  fatal() should
// be called when the simulation cannot continue due to some condition
// that is the user's fault (bad configuration, invalid arguments,
// etc.) and not a simulator bug.  fatal() calls  abort() like
// panic() does.
//
#define fatal(...) exit_message("fatal", -1, __VA_ARGS__)

/**
 * Conditional panic macro that checks the supplied condition and only panics
 * if the condition is true and allows the programmer to specify diagnostic
 * printout.  Useful to replace if + panic, or if + print + assert, etc.
 *
 * @param cond Condition that is checked; if true -> panic
 * @param ...  Printf-based format string with arguments, extends printout.
 */
#define panic_if(cond, ...) \
    do { \
        if ((cond)) \
            exit_message("panic condition "#cond" occurred", -1, __VA_ARGS__); \
    } while (0)


/**
 * Conditional fatal macro that checks the supplied condition and only causes a
 * fatal error if the condition is true and allows the programmer to specify
 * diagnostic printout.  Useful to replace if + fatal, or if + print + assert,
 * etc.
 *
 * @param cond Condition that is checked; if true -> fatal
 * @param ...  Printf-based format string with arguments, extends printout.
 */
#define fatal_if(cond, ...) \
    do { \
        if ((cond)) \
            exit_message("fatal condition "#cond" occurred", 1, __VA_ARGS__); \
    } while (0)


void
__base_message(std::ostream &stream, const char *prefix, bool verbose,
          const char *func, const char *file, int line,
          const char *format, CPRINTF_DECLARATION);

inline void
__base_message(std::ostream &stream, const char *prefix, bool verbose,
          const char *func, const char *file, int line,
          const std::string &format, CPRINTF_DECLARATION)
{
    __base_message(stream, prefix, verbose, func, file, line, format.c_str(),
              VARARGS_ALLARGS);
}

#define base_message(stream, prefix, verbose, ...)                      \
    __base_message(stream, prefix, verbose, __FUNCTION__, __FILE__, __LINE__, \
                   __VA_ARGS__)

// Only print the message the first time this expression is
// encountered.  i.e.  This doesn't check the string itself and
// prevent duplicate strings, this prevents the statement from
// happening more than once. So, even if the arguments change and that
// would have resulted in a different message thoes messages would be
// supressed.
#define base_message_once(...) do {                     \
        static bool once = false;                       \
        if (!once) {                                    \
            base_message(__VA_ARGS__);                  \
            once = true;                                \
        }                                               \
    } while (0)

#define cond_message(cond, ...) do {            \
        if (cond)                               \
            base_message(__VA_ARGS__);          \
    } while (0)

#define cond_message_once(cond, ...) do {               \
        static bool once = false;                       \
        if (!once && cond) {                            \
            base_message(__VA_ARGS__);                  \
            once = true;                                \
        }                                               \
    } while (0)


extern bool want_warn, warn_verbose;
extern bool want_info, info_verbose;
extern bool want_hack, hack_verbose;

#define warn(...) \
    cond_message(want_warn, std::cerr, "warn", warn_verbose, __VA_ARGS__)
#define inform(...) \
    cond_message(want_info, std::cout, "info", info_verbose, __VA_ARGS__)
#define hack(...) \
    cond_message(want_hack, std::cerr, "hack", hack_verbose, __VA_ARGS__)

#define warn_once(...) \
    cond_message_once(want_warn, std::cerr, "warn", warn_verbose, __VA_ARGS__)
#define inform_once(...) \
    cond_message_once(want_info, std::cout, "info", info_verbose, __VA_ARGS__)
#define hack_once(...) \
    cond_message_once(want_hack, std::cerr, "hack", hack_verbose, __VA_ARGS__)

/**
 * The chatty assert macro will function like a normal assert, but will allow the
 * specification of additional, helpful material to aid debugging why the
 * assertion actually failed.  Like the normal assertion, the chatty_assert
 * will not be active in fast builds.
 *
 * @param cond Condition that is checked; if false -> assert
 * @param ...  Printf-based format string with arguments, extends printout.
 */
#ifdef NDEBUG
#define chatty_assert(cond, ...)
#else //!NDEBUG
#define chatty_assert(cond, ...)                                                \
    do {                                                                        \
        if (!(cond)) {                                                          \
            base_message(std::cerr, "assert("#cond") failing", 1, __VA_ARGS__); \
            assert(cond);                                                       \
        }                                                                       \
    } while (0)
#endif // NDEBUG
#endif // __BASE_MISC_HH__
