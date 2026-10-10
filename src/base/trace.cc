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

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iostream>
#include <syncstream>

#include "base/atomicio.hh"
#include "base/logging.hh"
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

std::string
Logger::getDumpRow(const void *d, int idx, int len)
{
    const char *data = static_cast<const char *>(d);
    const int c = std::min(len - idx, 16);

    std::string row = csprintf("%08x  ", idx);

    for (int j = 0; j < c; j++) {
        row += csprintf("%02x ", data[j] & 0xff);
        if (j == 7) {
            row += ' ';
        }
    }

    row.append(3 * (16 - c), ' ');
    row += "  ";

    for (int j = 0; j < c; j++) {
        const int ch = data[j] & 0x7f;
        row += isprint(ch) ? static_cast<char>(ch) : ' ';
    }

    row += '\n';

    return row;
}

void
Logger::dump(Tick when, const std::string &name,
         const void *d, int len, const std::string &flag)
{
    if (!isEnabled(name)) {
        return;
    }

    const char *data = static_cast<const char *>(d);

    for (int i = 0; i < len; i += 16) {
        logMessage(when, name, flag, getDumpRow(data + i, i, len));
    }
}

void
OstreamLogger::dump(Tick when, const std::string &name, const void *d, int len,
                    const std::string &flag)
{
    if (!isEnabled(name)) {
        return;
    }

    const char *data = static_cast<const char *>(d);
    std::string dump;

    for (int i = 0; i < len; i += 16) {
        dump +=
            getMessagePrefix(when, name, flag) + getDumpRow(data + i, i, len);
    }

    saveMessage(dump);

    if (debug::FmtStackTrace) {
        print_backtrace();
        STATIC_ERR("\n");
    }
}

void
OstreamLogger::logMessage(Tick when, const std::string &name,
        const std::string &flag, const std::string &message)
{
    if (!isEnabled(name)) {
        return;
    }

    saveMessage(getMessagePrefix(when, name, flag) + message);

    if (debug::FmtStackTrace) {
        print_backtrace();
        STATIC_ERR("\n");
    }
}

void
OstreamLogger::saveMessage(const std::string &message)
{
    // Emit the whole message atomically so that concurrent event queue
    // threads cannot interleave output or race on stream state.
    std::osyncstream syncstream(stream);
    syncstream << message << std::flush_emit;
}

std::string
OstreamLogger::getMessagePrefix(Tick when, const std::string &name,
                                const std::string &flag)
{
    std::string prefix;

    if (!debug::FmtTicksOff && (when != MaxTick)) {
        prefix += csprintf("%7d: ", when);
    }

    if (debug::FmtFlag && !flag.empty()) {
        prefix += flag + ": ";
    }

    if (!name.empty()) {
        prefix += name + ": ";
    }

    return prefix;
}

} // namespace trace
} // namespace gem5
