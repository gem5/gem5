/*
 * Copyright (c) 2001-2004 The Regents of The University of Michigan
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

#ifndef __TRACE_HH__
#define __TRACE_HH__

#include <vector>

#include "base/cprintf.hh"
#include "sim/host.hh"
#include "sim/universe.hh"

#ifndef TRACING_ON
#ifndef NDEBUG
#define TRACING_ON	1
#else
#define TRACING_ON	0
#endif
#endif

#include "base/traceflags.hh"

namespace Trace {

    typedef std::vector<bool> FlagVec;

    extern FlagVec flags;

#if TRACING_ON
    const bool On				= true;
#else
    const bool On				= false;
#endif

    inline bool
    IsOn(int t)
    {
        return flags[t];

    }

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
        const char *format;
        const std::string &name;
        cp::ArgList &args;

      public:
        PrintfRecord(const char *_format, cp::ArgList &_args,
                     Tick cycle, const std::string &_name)
            : Record(cycle), format(_format), name(_name), args(_args)
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

    extern int dprintf_ignore_size;

    bool
    dprintf_ignore_name(const std::string &name);

    inline void
    dprintf(const char *format, cp::ArgList &args, Tick cycle,
            const std::string &name)
    {
        if (!dprintf_ignore_size || name.empty() || !dprintf_ignore_name(name))
            theLog.append(new Trace::PrintfRecord(format, args, cycle, name));
    }

    inline void
    dataDump(Tick cycle, const std::string &name, const void *data, int len)
    {
        theLog.append(new Trace::DataRecord(cycle, name, data, len));
    }

    extern const std::string DefaultName;
};

inline const std::string &name() { return Trace::DefaultName; }

std::ostream &DebugOut();

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

#define DTRACE(x) (Trace::IsOn(Trace::x))

#define DCOUT(x) if (Trace::IsOn(Trace::x)) DebugOut()

#define DDUMP(x, data, count) \
do { \
    if (Trace::IsOn(Trace::x)) \
        Trace::dataDump(curTick, name(), data, count);	\
} while (0)

#define __dprintf(cycle, name, format, args...) \
    Trace::dprintf(format, (*(new cp::ArgList), args), cycle, name)

#define DPRINTF(x, args...) \
do { \
    if (Trace::IsOn(Trace::x)) \
        __dprintf(curTick, name(), args, cp::ArgListNull()); \
} while (0)

#define DPRINTFR(x, args...) \
do { \
    if (Trace::IsOn(Trace::x)) \
        __dprintf((Tick)-1, string(), args, cp::ArgListNull()); \
} while (0)

#define DPRINTFN(args...) \
do { \
    __dprintf(curTick, name(), args, cp::ArgListNull()); \
} while (0)

#define DPRINTFNR(args...) \
do { \
    __dprintf((Tick)-1, string(), args, cp::ArgListNull()); \
} while (0)

#else // !TRACING_ON

#define DTRACE(x) (false)
#define DCOUT(x) if (0) DebugOut()
#define DPRINTF(x, args...) do {} while (0)
#define DPRINTFR(args...) do {} while (0)
#define DPRINTFN(args...) do {} while (0)
#define DPRINTFNR(args...) do {} while (0)
#define DDUMP(x, data, count) do {} while (0)

#endif	// TRACING_ON

#endif // __TRACE_HH__
