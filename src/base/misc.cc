/*
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
 */

#include <cstdlib>
#include <iostream>
#include <string>

#include "base/cprintf.hh"
#include "base/hostinfo.hh"
#include "base/misc.hh"
#include "base/output.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "base/varargs.hh"
#include "sim/core.hh"

using namespace std;

bool want_warn = true;
bool want_info = true;
bool want_hack = true;

bool warn_verbose = false;
bool info_verbose = false;
bool hack_verbose = false;

void
__exit_message(const char *prefix, int code,
    const char *func, const char *file, int line,
    const char *fmt, CPRINTF_DEFINITION)
{
    CPrintfArgsList args(VARARGS_ALLARGS);

    string format = prefix;
    format += ": ";
    format += fmt;
    switch (format[format.size() - 1]) {
      case '\n':
      case '\r':
        break;
      default:
        format += "\n";
    }

    format += " @ tick %d\n[%s:%s, line %d]\n";
    format += "Memory Usage: %ld KBytes\n";

    args.push_back(curTick());
    args.push_back(func);
    args.push_back(file);
    args.push_back(line);
    args.push_back(memUsage());

    ccprintf(cerr, format.c_str(), args);

    if (code < 0)
        abort();
    else
        exit(code);
}

void
__base_message(std::ostream &stream, const char *prefix, bool verbose,
    const char *func, const char *file, int line,
    const char *fmt, CPRINTF_DEFINITION)
{
    CPrintfArgsList args(VARARGS_ALLARGS);

    string format = prefix;
    format += ": ";
    format += fmt;
    switch (format[format.size() - 1]) {
      case '\n':
      case '\r':
        break;
      default:
        format += "\n";
    }

    if (verbose) {
        format += " @ cycle %d\n[%s:%s, line %d]\n";
        args.push_back(curTick());
        args.push_back(func);
        args.push_back(file);
        args.push_back(line);
    }

    ccprintf(stream, format.c_str(), args);
}
