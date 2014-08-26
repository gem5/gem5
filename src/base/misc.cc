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
 *          Andreas Sandberg
 */

#include <cstdlib>
#include <cstring>
#include <string>

#include "base/cprintf.hh"
#include "base/hostinfo.hh"
#include "base/misc.hh"
#include "base/output.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "sim/core.hh"

using namespace std;

bool want_warn = true;
bool want_info = true;
bool want_hack = true;

bool warn_verbose = false;
bool info_verbose = false;
bool hack_verbose = false;

static void
newline_if_needed(std::ostream &stream, const char *format)
{
    const size_t format_len(strlen(format));

    switch (format_len ? format[format_len - 1] : '\0') {
      case '\n':
      case '\r':
        break;
      default:
        stream << std::endl;
    }
}

void
__exit_epilogue(int code,
                const char *func, const char *file, int line,
                const char *format)
{
    newline_if_needed(std::cerr, format);

    ccprintf(std::cerr,
             " @ tick %d\n"
             "[%s:%s, line %d]\n"
             "Memory Usage: %ld KBytes\n",
             curTick(), func, file, line, memUsage());

    if (code < 0)
        abort();
    else
        exit(code);
}

void
__base_message_epilogue(std::ostream &stream, bool verbose,
                        const char *func, const char *file, int line,
                        const char *format)
{
    newline_if_needed(stream, format);

    if (verbose) {
        ccprintf(stream, " @ cycle %d\n[%s:%s, line %d]\n",
                 curTick(), func, file, line);
    }
}
