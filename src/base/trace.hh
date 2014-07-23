/*
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
 *
 * Authors: Nathan Binkert
 *          Steve Reinhardt
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

using Debug::SimpleFlag;
using Debug::CompoundFlag;

std::ostream &output();
void setOutput(const std::string &filename);

extern bool enabled;
bool changeFlag(const char *str, bool value);
void dumpStatus();

extern ObjectMatch ignore;
extern const std::string DefaultName;

void dprintf(Tick when, const std::string &name, const char *format,
             CPRINTF_DECLARATION);
void dump(Tick when, const std::string &name, const void *data, int len);

} // namespace Trace

// This silly little class allows us to wrap a string in a functor
// object so that we can give a name() that DPRINTF will like
struct StringWrap
{
    std::string str;
    StringWrap(const std::string &s) : str(s) {}
    const std::string &operator()() const { return str; }
};

inline const std::string &name() { return Trace::DefaultName; }

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

#define DTRACE(x) ((Debug::x) && Trace::enabled)

#define DDUMP(x, data, count) do {                              \
    using namespace Debug;                                      \
    if (DTRACE(x))                                              \
        Trace::dump(curTick(), name(), data, count);              \
} while (0)

#define DPRINTF(x, ...) do {                                    \
    using namespace Debug;                                      \
    if (DTRACE(x))                                              \
        Trace::dprintf(curTick(), name(), __VA_ARGS__);           \
} while (0)

#define DPRINTFS(x, s, ...) do {                                \
    using namespace Debug;                                      \
    if (DTRACE(x))                                              \
        Trace::dprintf(curTick(), s->name(), __VA_ARGS__);      \
} while (0)

#define DPRINTFR(x, ...) do {                                   \
    using namespace Debug;                                      \
    if (DTRACE(x))                                              \
        Trace::dprintf((Tick)-1, std::string(), __VA_ARGS__);   \
} while (0)

#define DDUMPN(data, count) do {                                \
    Trace::dump(curTick(), name(), data, count);                  \
} while (0)

#define DPRINTFN(...) do {                                      \
    Trace::dprintf(curTick(), name(), __VA_ARGS__);               \
} while (0)

#define DPRINTFNR(...) do {                                     \
    Trace::dprintf((Tick)-1, string(), __VA_ARGS__);            \
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

#endif  // TRACING_ON

#endif // __BASE_TRACE_HH__
