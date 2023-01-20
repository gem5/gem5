/*
 * Copyright (c) 2014, 2017, 2019 ARM Limited
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
 */

#ifndef __BASE_LOGGING_HH__
#define __BASE_LOGGING_HH__

#include <cassert>
#include <sstream>
#include <utility>

#include "base/compiler.hh"
#include "base/cprintf.hh"

namespace gem5
{

class Logger
{
  public:

    /**
     * Get a Logger for the specified type of message.
     */
    static Logger &getPanic();
    static Logger &getFatal();
    static Logger &getWarn();
    static Logger &getInfo();
    static Logger &getHack();

    enum LogLevel
    {
        PANIC, FATAL, WARN, INFO, HACK,
        NUM_LOG_LEVELS,
    };

    static void
    setLevel(LogLevel ll)
    {
        getPanic().enabled = (ll >= PANIC);
        getFatal().enabled = (ll >= FATAL);
        getWarn().enabled = (ll >= WARN);
        getInfo().enabled = (ll >= INFO);
        getHack().enabled = (ll >= HACK);
    }

    struct Loc
    {
        Loc(const char *file, int line) : file(file), line(line) {}
        const char *file;
        int line;
    };

    Logger(const char *prefix) : enabled(true), prefix(prefix)
    {
        assert(prefix);
    }

    virtual ~Logger() {};

    template<typename ...Args> void
    print(const Loc &loc, const char *format, const Args &...args)
    {
        std::stringstream ss;
        ccprintf(ss, format, args...);
        const std::string str = ss.str();

        std::stringstream ss_formatted;
        ss_formatted << prefix << str;
        if (str.length() && str.back() != '\n' && str.back() != '\r')
            ss_formatted << std::endl;
        if (!enabled)
            return;
        log(loc, ss_formatted.str());
    }

    template<typename ...Args> void
    print(const Loc &loc, const std::string &format, const Args &...args)
    {
        print(loc, format.c_str(), args...);
    }

    /**
     * This helper is necessary since noreturn isn't inherited by virtual
     * functions, and gcc will get mad if a function calls panic and then
     * doesn't return.
     */
    [[noreturn]] void exit_helper() { exit(); ::abort(); }

  protected:
    bool enabled;

    /** Generates the log message. By default it is sent to cerr. */
    virtual void
    log(const Loc &loc, std::string s)
    {
        std::cerr << loc.file << ":" << loc.line << ": " << s;
    }

    virtual void exit() { /* Fall through to the abort in exit_helper. */ }

    const char *prefix;
};

#define base_message(logger, ...)                                       \
    [&log = logger](const auto&... args) {                              \
        log.print(::gem5::Logger::Loc(__FILE__, __LINE__), args...);    \
    }(__VA_ARGS__)

/*
 * Only print the message the first time this expression is
 * encountered.  i.e.  This doesn't check the string itself and
 * prevent duplicate strings, this prevents the statement from
 * happening more than once. So, even if the arguments change and that
 * would have resulted in a different message thoes messages would be
 * supressed.
 */
#define base_message_once(logger, ...)          \
    [&log = logger](const auto&... args) {      \
        static bool once{false};                \
        if (GEM5_UNLIKELY(!once)) {             \
            once = true;                        \
            base_message(log, args...);         \
        }                                       \
    }(__VA_ARGS__)

/*
 * logger.exit_helper() can't be called inside the lambda for now as the
 * lambda's operator() can't be [[noreturn]]. As a result, exit_message and it'
 * s derivative cannot be used in functions without also specifying a return
 * value, which is inconvenient if not impossible.
 */

#define exit_message(logger, ...)               \
    (                                           \
        [&log = logger](const auto&... args) {  \
            base_message(log, args...);         \
        }(__VA_ARGS__),                         \
        logger.exit_helper()                    \
    )

/**
 * This implements a cprintf based panic() function.  panic() should
 * be called when something happens that should never ever happen
 * regardless of what the user does (i.e., an acutal m5 bug).  panic()
 * might call abort which can dump core or enter the debugger.
 *
 * \def panic(...)
 *
 * @ingroup api_logger
 */
#define panic(...) exit_message(::gem5::Logger::getPanic(), __VA_ARGS__)

/**
 * This implements a cprintf based fatal() function.  fatal() should
 * be called when the simulation cannot continue due to some condition
 * that is the user's fault (bad configuration, invalid arguments,
 * etc.) and not a simulator bug.  fatal() might call exit, unlike panic().
 *
 * \def fatal(...)
 *
 * @ingroup api_logger
 */
#define fatal(...) exit_message(::gem5::Logger::getFatal(), __VA_ARGS__)

/**
 * Conditional panic macro that checks the supplied condition and only panics
 * if the condition is true and allows the programmer to specify diagnostic
 * printout.  Useful to replace if + panic, or if + print + assert, etc.
 *
 * @param cond Condition that is checked; if true -> panic
 * @param ...  Printf-based format string with arguments, extends printout.
 *
 * \def panic_if(...)
 *
 * @ingroup api_logger
 */
#define panic_if(cond, ...)                             \
    (                                                   \
    GEM5_UNLIKELY(static_cast<bool>(cond)) ?            \
    panic("panic condition " # cond " occurred: %s",    \
        ::gem5::csprintf(__VA_ARGS__)) :                \
    void(0)                                             \
    )


/**
 * Conditional fatal macro that checks the supplied condition and only causes a
 * fatal error if the condition is true and allows the programmer to specify
 * diagnostic printout.  Useful to replace if + fatal, or if + print + assert,
 * etc.
 *
 * @param cond Condition that is checked; if true -> fatal
 * @param ...  Printf-based format string with arguments, extends printout.
 *
 * \def fatal_if(...)
 *
 * @ingroup api_logger
 */
#define fatal_if(cond, ...)                             \
    (                                                   \
    GEM5_UNLIKELY(static_cast<bool>(cond)) ?            \
    fatal("fatal condition " # cond " occurred: %s",    \
        ::gem5::csprintf(__VA_ARGS__)) :                \
    void(0)                                             \
    )


/**
 * \def warn(...)
 * \def inform(...)
 * \def hack(...)
 * \def warn_once(...)
 * \def inform_once(...)
 * \def hack_once(...)
 *
 * @ingroup api_logger
 * @{
 */
#define warn(...) base_message(::gem5::Logger::getWarn(), __VA_ARGS__)
#define inform(...) base_message(::gem5::Logger::getInfo(), __VA_ARGS__)
#define hack(...) base_message(::gem5::Logger::getHack(), __VA_ARGS__)

#define warn_once(...) \
    base_message_once(::gem5::Logger::getWarn(), __VA_ARGS__)
#define inform_once(...) \
    base_message_once(::gem5::Logger::getInfo(), __VA_ARGS__)
#define hack_once(...) \
    base_message_once(::gem5::Logger::getHack(), __VA_ARGS__)
/** @} */ // end of api_logger

/**
 *
 * Conditional warning macro that checks the supplied condition and
 * only prints a warning if the condition is true. Useful to replace
 * if + warn.
 *
 * @param cond Condition that is checked; if true -> warn
 * @param ...  Printf-based format string with arguments, extends printout.
 *
 * \def warn_if(cond, ...)
 * \def warn_if_once(cond, ...)
 *
 * @ingroup api_logger
 * @{
 */
#define warn_if(cond, ...)      \
    (                           \
    static_cast<bool>(cond) ?   \
    warn(__VA_ARGS__) :         \
    void(0)                     \
    )

#define warn_if_once(cond, ...) \
    (                           \
    static_cast<bool>(cond) ?   \
    warn_once(__VA_ARGS__) :    \
    void(0)                     \
    )

/** @} */ // end of api_logger

#ifdef NDEBUG
#define NDEBUG_DEFINED 1
#else
#define NDEBUG_DEFINED 0
#endif

/**
 * The assert macro will function like a normal assert, but will use panic
 * instead of straight abort(). This allows to perform some cleaning up in
 * ExitLogger::exit() before calling abort().
 *
 * @param cond Condition that is checked; if false -> panic
 * @param ...  Printf-based format string with arguments, extends printout.
 *
 * \def gem5_assert(cond, ...)
 *
 * @ingroup api_logger
 */
#define gem5_assert(cond, ...)                                  \
    (                                                           \
    GEM5_UNLIKELY(NDEBUG_DEFINED || static_cast<bool>(cond)) ?  \
    void(0) :                                                   \
    [](const auto&... args) {                                   \
        auto msg = [&]{                                         \
            if constexpr (sizeof...(args) == 0) return "";      \
            else return std::string(": ") + csprintf(args...);  \
        };                                                      \
        panic("assert(" #cond ") failed%s", msg());             \
    }(__VA_ARGS__)                                              \
    )

/** @} */ // end of api_logger

#define chatty_assert(...)                                                   \
    (                                                                        \
        gem5_assert(args...),                                                \
        GEM5_DEPRECATED_MACRO(chatty_assert, {}, "Please use gem5_assert()") \
    )

} // namespace gem5
#endif // __BASE_LOGGING_HH__
