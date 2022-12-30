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

#include "base/trace.hh"

#include <cctype>
#include <fstream>
#include <iostream>
#include <sstream>

#include "base/atomicio.hh"
#include "base/logging.hh"
#include "base/output.hh"
#include "base/str.hh"
#include "debug/FmtFlag.hh"
#include "debug/FmtStackTrace.hh"
#include "debug/FmtTicksOff.hh"
#include "sim/backtrace.hh"

const std::string &
name()
{
    static const std::string default_name("global");

    return default_name;
}

namespace gem5
{

namespace trace
{

// This variable holds the output logger for debug information.  Other
// than setting up/redirecting this logger, do *NOT* reference this
// directly

Logger *debug_logger = NULL;

Logger *
getDebugLogger()
{
    /* Set a default logger to cerr when no other logger is set */
    if (!debug_logger)
        debug_logger = new OstreamLogger(std::cerr);

    return debug_logger;
}

std::ostream &
output()
{
    return getDebugLogger()->getOstream();
}

void
setDebugLogger(Logger *logger)
{
    if (!logger)
        warn("Trying to set debug logger to NULL\n");
    else
        debug_logger = logger;
}

void
enable()
{
    debug::Flag::globalEnable();
}

void
disable()
{
    debug::Flag::globalDisable();
}

ObjectMatch ignore;


void
Logger::dump(Tick when, const std::string &name,
         const void *d, int len, const std::string &flag)
{
    if (!name.empty() && ignore.match(name))
        return;

    const char *data = static_cast<const char *>(d);
    int c, i, j;

    for (i = 0; i < len; i += 16) {
        std::ostringstream line;

        ccprintf(line, "%08x  ", i);
        c = len - i;
        if (c > 16) c = 16;

        for (j = 0; j < c; j++) {
            ccprintf(line, "%02x ", data[i + j] & 0xff);
            if ((j & 0xf) == 7 && j > 0)
                ccprintf(line, " ");
        }

        for (; j < 16; j++)
            ccprintf(line, "   ");
        ccprintf(line, "  ");

        for (j = 0; j < c; j++) {
            int ch = data[i + j] & 0x7f;
            ccprintf(line, "%c", (char)(isprint(ch) ? ch : ' '));
        }

        ccprintf(line, "\n");
        logMessage(when, name, flag, line.str());

        if (c < 16)
            break;
    }
}

void
OstreamLogger::logMessage(Tick when, const std::string &name,
        const std::string &flag, const std::string &message)
{
    if (!name.empty() && ignore.match(name))
        return;

    if (!debug::FmtTicksOff && (when != MaxTick))
        ccprintf(stream, "%7d: ", when);

    if (debug::FmtFlag && !flag.empty())
        stream << flag << ": ";

    if (!name.empty())
        stream << name << ": ";

    stream << message;
    stream.flush();

    if (debug::FmtStackTrace) {
        print_backtrace();
        STATIC_ERR("\n");
    }
}

} // namespace trace
} // namespace gem5
