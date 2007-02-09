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

#include <vector>

#include "base/cprintf.hh"
#include "base/match.hh"
#include "sim/host.hh"
#include "sim/root.hh"

#include "base/traceflags.hh"

namespace Trace {

    typedef std::vector<bool> FlagVec;

    extern FlagVec flags;

    extern std::ostream *dprintf_stream;

    inline bool
    IsOn(int t)
    {
        return flags[t];
    }

    extern bool enabled;

    void dump(const uint8_t *data, int count);

    class Record
    {
      protected:
        Tick cycle;

        Record(Tick _cycle)
            : cycle(_cycle)
        {
        }

      public:
        virtual ~Record() {}

        virtual void dump(std::ostream &) = 0;
    };

    class PrintfRecord : public Record
    {
      private:
        const std::string &name;
        const char *format;
        CPrintfArgsList args;

      public:
        PrintfRecord(Tick cycle, const std::string &_name, const char *_format,
                     CPRINTF_DECLARATION)
            : Record(cycle), name(_name), format(_format),
              args(VARARGS_ALLARGS)
        {
        }

        virtual ~PrintfRecord();

        virtual void dump(std::ostream &);
    };

    class DataRecord : public Record
    {
      private:
        const std::string &name;
        uint8_t *data;
        int len;

      public:
        DataRecord(Tick cycle, const std::string &name,
                   const void *_data, int _len);
        virtual ~DataRecord();

        virtual void dump(std::ostream &);
    };

    class Log
    {
      private:
        int	 size;		// number of records in log
        Record **buffer;	// array of 'size' Record ptrs (circular buf)
        Record **nextRecPtr;	// next slot to use in buffer
        Record **wrapRecPtr;	// &buffer[size], for quick wrap check

      public:
        Log();
        ~Log();

        void init(int _size);

        void append(Record *);	// append trace record to log
        void dump(std::ostream &);	// dump contents to stream
    };

    extern Log theLog;

    extern ObjectMatch ignore;

    inline void
    dprintf(Tick when, const std::string &name, const char *format,
            CPRINTF_DECLARATION)
    {
        if (!name.empty() && ignore.match(name))
            return;

        theLog.append(new Trace::PrintfRecord(when, name, format,
                                              VARARGS_ALLARGS));
    }

    inline void
    dataDump(Tick when, const std::string &name, const void *data, int len)
    {
        theLog.append(new Trace::DataRecord(when, name, data, len));
    }

    extern const std::string DefaultName;

};

inline std::ostream &
DebugOut()
{
    return *Trace::dprintf_stream;
}

// This silly little class allows us to wrap a string in a functor
// object so that we can give a name() that DPRINTF will like
struct StringWrap
{
    std::string str;
    StringWrap(const std::string &s) : str(s) {}
    const std::string &operator()() const { return str; }
};

inline const std::string &name() { return Trace::DefaultName; }

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

#define DTRACE(x) (Trace::IsOn(Trace::x) && Trace::enabled)

#define DDUMP(x, data, count) do {                              \
    if (DTRACE(x))                                              \
        Trace::dataDump(curTick, name(), data, count);          \
} while (0)

#define DPRINTF(x, ...) do {                                    \
    if (DTRACE(x))                                              \
        Trace::dprintf(curTick, name(), __VA_ARGS__);           \
} while (0)

#define DPRINTFR(x, ...) do {                                   \
    if (DTRACE(x))                                              \
        Trace::dprintf((Tick)-1, std::string(), __VA_ARGS__);   \
} while (0)

#define DPRINTFN(...) do {                                      \
    Trace::dprintf(curTick, name(), __VA_ARGS__);               \
} while (0)

#define DPRINTFNR(...) do {                                     \
    Trace::dprintf((Tick)-1, string(), __VA_ARGS__);            \
} while (0)

#else // !TRACING_ON

#define DTRACE(x) (false)
#define DPRINTF(x, ...) do {} while (0)
#define DPRINTFR(...) do {} while (0)
#define DPRINTFN(...) do {} while (0)
#define DPRINTFNR(...) do {} while (0)
#define DDUMP(x, data, count) do {} while (0)

#endif	// TRACING_ON

#endif // __BASE_TRACE_HH__
