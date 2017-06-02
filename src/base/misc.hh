/*
 * Copyright (c) 2014, 2017 ARM Limited
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
 *          Andreas Sandberg
 */

#ifndef __BASE_MISC_HH__
#define __BASE_MISC_HH__

#include <cassert>
#include <cstdlib>
#include <iostream>
#include <utility>

#include "base/compiler.hh"
#include "base/cprintf.hh"

#if defined(__SUNPRO_CC)
#define __FUNCTION__ "how to fix me?"
#endif

class Logger
{
  public:
    enum LogLevel {
        PANIC = 0,
        FATAL,
        WARN,
        INFO,
        HACK,
        NUM_LOG_LEVELS,
    };

    /**
     * Set the active log level.
     *
     * All levels that are lower or equal to the selected log level
     * will be activated.
     *
     * @param ll Maximum log level to print
     */
    static void setLevel(LogLevel ll);

    /**
     * Get a Logger corresponding to a specific log level
     *
     * @param ll Log level to access
     * @return Reference to the requested logger
     */
    static Logger &get(LogLevel ll);

  public:
    Logger(std::ostream &stream, const char *prefix);
    virtual ~Logger() {};

    template<typename ...Args> void
    print(const char *func, const char *file, int line,
          const char *format, const Args &...args)
    {
        if (!enabled)
            return;

        if (prefix)
            stream << prefix << ": ";
        ccprintf(stream, format, args...);

        printEpilogue(func, file, line, format);
    }

    template<typename ...Args> void
    print(const char *func, const char *file, int line,
          const std::string &format, const Args &...args)
    {
        print(func, file, line, format.c_str(), args...);
    }

  protected:
    virtual void printEpilogue(const char *func, const char *file, int line,
                               const char *format);

  public:
    bool enabled;
    bool verbose;

  protected:
    std::ostream &stream;
    const char *prefix;
};

class ExitLogger : public Logger
{
  public:
    using Logger::Logger;

    void printEpilogue(const char *func, const char *file, int line,
                       const char *format) override;
};

#define exit_message(logger, code, ...)                                 \
    do {                                                                \
        logger.print(__FUNCTION__, __FILE__, __LINE__, __VA_ARGS__);    \
        if (code < 0)                                                   \
            ::abort();                                                  \
        else                                                            \
            ::exit(code);                                               \
    } while (0)

//
// This implements a cprintf based panic() function.  panic() should
// be called when something happens that should never ever happen
// regardless of what the user does (i.e., an acutal m5 bug).  panic()
// calls abort which can dump core or enter the debugger.
//
//
#define panic(...) exit_message(::Logger::get(::Logger::PANIC), -1, \
                                __VA_ARGS__)

//
// This implements a cprintf based fatal() function.  fatal() should
// be called when the simulation cannot continue due to some condition
// that is the user's fault (bad configuration, invalid arguments,
// etc.) and not a simulator bug.  fatal() calls  abort() like
// panic() does.
//
#define fatal(...) exit_message(::Logger::get(::Logger::FATAL), 1, \
                                __VA_ARGS__)

/**
 * Conditional panic macro that checks the supplied condition and only panics
 * if the condition is true and allows the programmer to specify diagnostic
 * printout.  Useful to replace if + panic, or if + print + assert, etc.
 *
 * @param cond Condition that is checked; if true -> panic
 * @param ...  Printf-based format string with arguments, extends printout.
 */
#define panic_if(cond, ...)                                  \
    do {                                                     \
        if ((cond)) {                                        \
            panic("panic condition " # cond " occurred: %s", \
                  csprintf(__VA_ARGS__));                    \
        }                                                    \
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
#define fatal_if(cond, ...)                                     \
    do {                                                        \
        if ((cond)) {                                           \
            fatal("fatal condition " # cond " occurred: %s",    \
                  csprintf(__VA_ARGS__));                       \
        }                                                       \
    } while (0)


#define base_message(logger, ...)                                       \
    logger.print(__FUNCTION__, __FILE__, __LINE__, __VA_ARGS__)

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


#define warn(...) \
    base_message(::Logger::get(::Logger::WARN), __VA_ARGS__)
#define inform(...) \
    base_message(::Logger::get(::Logger::INFO), __VA_ARGS__)
#define hack(...) \
    base_message(::Logger::get(::Logger::HACK), __VA_ARGS__)

#define warn_once(...) \
    base_message_once(::Logger::get(::Logger::WARN), __VA_ARGS__)
#define inform_once(...) \
    base_message_once(::Logger::get(::Logger::INFO), __VA_ARGS__)
#define hack_once(...) \
    base_message_once(::Logger::get(::Logger::HACK), __VA_ARGS__)

/**
 * Conditional warning macro that checks the supplied condition and
 * only prints a warning if the condition is true. Useful to replace
 * if + warn.
 *
 * @param cond Condition that is checked; if true -> warn
 * @param ...  Printf-based format string with arguments, extends printout.
 */
#define warn_if(cond, ...) \
    do { \
        if ((cond)) \
            warn(__VA_ARGS__); \
    } while (0)

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
#define chatty_assert(cond, ...)                                        \
    do {                                                                \
        if (!(cond))                                                    \
            panic("assert(" # cond ") failed: %s", csprintf(__VA_ARGS__)); \
    } while (0)
#endif // NDEBUG
#endif // __BASE_MISC_HH__
