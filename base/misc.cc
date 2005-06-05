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
 */

#include <iostream>
#include <string>

#include "base/cprintf.hh"
#include "base/hostinfo.hh"
#include "base/misc.hh"
#include "base/output.hh"
#include "base/trace.hh"
#include "sim/host.hh"
#include "sim/root.hh"

using namespace std;

void
__panic(const string &format, cp::ArgList &args, const char *func,
        const char *file, int line)
{
    string fmt = "panic: " + format;
    switch (fmt[fmt.size() - 1]) {
      case '\n':
      case '\r':
        break;
      default:
        fmt += "\n";
    }

    fmt += " @ cycle %d\n[%s:%s, line %d]\n";

    args.append(curTick);
    args.append(func);
    args.append(file);
    args.append(line);
    args.dump(cerr, fmt);

    delete &args;

    abort();
}

void
__fatal(const string &format, cp::ArgList &args, const char *func,
        const char *file, int line)
{
    string fmt = "fatal: " + format;

    switch (fmt[fmt.size() - 1]) {
      case '\n':
      case '\r':
        break;
      default:
        fmt += "\n";
    }

    fmt += " @ cycle %d\n[%s:%s, line %d]\n";
    fmt += "Memory Usage: %ld KBytes\n";

    args.append(curTick);
    args.append(func);
    args.append(file);
    args.append(line);
    args.append(memUsage());
    args.dump(cerr, fmt);

    delete &args;

    exit(1);
}

void
__warn(const string &format, cp::ArgList &args, const char *func,
       const char *file, int line)
{
    string fmt = "warn: " + format;

    switch (fmt[fmt.size() - 1]) {
      case '\n':
      case '\r':
        break;
      default:
        fmt += "\n";
    }

#ifdef VERBOSE_WARN
    fmt += " @ cycle %d\n[%s:%s, line %d]\n";
    args.append(curTick);
    args.append(func);
    args.append(file);
    args.append(line);
#endif

    args.dump(cerr, fmt);
    if (simout.isFile(*outputStream))
        args.dump(*outputStream, fmt);

    delete &args;
}
