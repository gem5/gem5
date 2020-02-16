/*
 * Copyright (c) 2014, 2019 ARM Limited
 * All rights reserved
 *
 * Copyright (c) 2001-2006 The Regents of The University of Michigan
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

#ifndef __BASE_TRACE_HH__
#define __BASE_TRACE_HH__

#include <string>

#include "base/cprintf.hh"
#include "base/debug.hh"
#include "base/match.hh"
#include "base/types.hh"
#include "sim/core.hh"

namespace Trace {

/** Debug logging base class.  Handles formatting and outputting
 *  time/name/message messages */
class Logger
{
  protected:
    /** Name match for objects to ignore */
    ObjectMatch ignore;

  public:
    /** Log a single message */
    template <typename ...Args>
    void dprintf(Tick when, const std::string &name, const char *fmt,
                 const Args &...args)
    {
        dprintf_flag(when, name, "", fmt, args...);
    }

    /** Log a single message with a flag prefix. */
    template <typename ...Args>
    void dprintf_flag(Tick when, const std::string &name,
            const std::string &flag,
            const char *fmt, const Args &...args)
    {
        if (!name.empty() && ignore.match(name))
            return;
        std::ostringstream line;
        ccprintf(line, fmt, args...);
        logMessage(when, name, flag, line.str());
    }

    /** Dump a block of data of length len */
    void dump(Tick when, const std::string &name,
            const void *d, int len, const std::string &flag);

    /** Log formatted message */
    virtual void logMessage(Tick when, const std::string &name,
            const std::string &flag, const std::string &message) = 0;

    /** Return an ostream that can be used to send messages to
     *  the 'same place' as formatted logMessage messages.  This
     *  can be implemented to use a logger's underlying ostream,
     *  to provide an ostream which formats the output in some
     *  way, or just set to one of std::cout, std::cerr */
    virtual std::ostream &getOstream() = 0;

    /** Set objects to ignore */
    void setIgnore(ObjectMatch &ignore_) { ignore = ignore_; }

    /** Add objects to ignore */
    void addIgnore(const ObjectMatch &ignore_) { ignore.add(ignore_); }

    virtual ~Logger() { }
};

/** Logging wrapper for ostreams with the format:
 *  <when>: <name>: <message-body> */
class OstreamLogger : public Logger
{
  protected:
    std::ostream &stream;

  public:
    OstreamLogger(std::ostream &stream_) : stream(stream_)
    { }

    void logMessage(Tick when, const std::string &name,
            const std::string &flag, const std::string &message) override;

    std::ostream &getOstream() override { return stream; }
};

/** Get the current global debug logger.  This takes ownership of the given
 *  logger which should be allocated using 'new' */
Logger *getDebugLogger();

/** Get the ostream from the current global logger */
std::ostream &output();

/** Delete the current global logger and assign a new one */
void setDebugLogger(Logger *logger);

/** Enable/disable debug logging */
void enable();
void disable();

} // namespace Trace

// This silly little class allows us to wrap a string in a functor
// object so that we can give a name() that DPRINTF will like
struct StringWrap
{
    std::string str;
    StringWrap(const std::string &s) : str(s) {}
    const std::string &operator()() const { return str; }
};

// Return the global context name "global".  This function gets called when
// the DPRINTF macros are used in a context without a visible name() function
const std::string &name();

// Interface for things with names. (cf. SimObject but without other
// functionality).  This is useful when using DPRINTF
class Named
{
  protected:
    const std::string _name;

  public:
    Named(const std::string &name_) : _name(name_) { }

  public:
    const std::string &name() const { return _name; }
};

//
// DPRINTF is a debugging trace facility that allows one to
// selectively enable tracing statements.  To use DPRINTF, there must
// be a function or functor called name() that returns a const
// std::string & in the current scope.
//
// If you desire that the automatic printing not occur, use DPRINTFR
// (R for raw)
//

#if TRACING_ON

#define DTRACE(x) (Debug::x)

#define DDUMP(x, data, count) do {               \
    using namespace Debug;                       \
    if (DTRACE(x))                               \
        Trace::getDebugLogger()->dump(           \
            curTick(), name(), data, count, #x); \
} while (0)

#define DPRINTF(x, ...) do {                     \
    using namespace Debug;                       \
    if (DTRACE(x)) {                             \
        Trace::getDebugLogger()->dprintf_flag(   \
            curTick(), name(), #x, __VA_ARGS__); \
    }                                            \
} while (0)

#define DPRINTFS(x, s, ...) do {                        \
    using namespace Debug;                              \
    if (DTRACE(x)) {                                    \
        Trace::getDebugLogger()->dprintf_flag(          \
                curTick(), s->name(), #x, __VA_ARGS__); \
    }                                                   \
} while (0)

#define DPRINTFR(x, ...) do {                          \
    using namespace Debug;                             \
    if (DTRACE(x)) {                                   \
        Trace::getDebugLogger()->dprintf_flag(         \
            (Tick)-1, std::string(), #x, __VA_ARGS__); \
    }                                                  \
} while (0)

#define DDUMPN(data, count) do {                                       \
    Trace::getDebugLogger()->dump(curTick(), name(), data, count);     \
} while (0)

#define DPRINTFN(...) do {                                             \
    Trace::getDebugLogger()->dprintf(curTick(), name(), __VA_ARGS__);  \
} while (0)

#define DPRINTFNR(...) do {                                                 \
    Trace::getDebugLogger()->dprintf((Tick)-1, std::string(), __VA_ARGS__); \
} while (0)

#define DPRINTF_UNCONDITIONAL(x, ...) do {    \
    Trace::getDebugLogger()->dprintf_flag(    \
        curTick(), name(), #x, __VA_ARGS__);  \
} while (0)

#else // !TRACING_ON

#define DTRACE(x) (false)
#define DDUMP(x, data, count) do {} while (0)
#define DPRINTF(x, ...) do {} while (0)
#define DPRINTFS(x, ...) do {} while (0)
#define DPRINTFR(...) do {} while (0)
#define DDUMPN(data, count) do {} while (0)
#define DPRINTFN(...) do {} while (0)
#define DPRINTFNR(...) do {} while (0)
#define DPRINTF_UNCONDITIONAL(x, ...) do {} while (0)

#endif  // TRACING_ON

#endif // __BASE_TRACE_HH__
