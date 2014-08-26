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

#include <cctype>
#include <fstream>
#include <iostream>
#include <string>

#include "base/misc.hh"
#include "base/output.hh"
#include "base/str.hh"
#include "base/trace.hh"

using namespace std;

namespace Trace {

const string DefaultName("global");
bool enabled = false;

//
// This variable holds the output stream for debug information.  Other
// than setting up/redirecting this stream, do *NOT* reference this
// directly; use DebugOut() (see below) to access this stream for
// output.
//
ostream *dprintf_stream = &cerr;
ostream &
output()
{
    return *dprintf_stream;
}

void
setOutput(const string &filename)
{
    dprintf_stream = simout.find(filename);
    if (!dprintf_stream)
        dprintf_stream = simout.create(filename);
}

ObjectMatch ignore;


bool
__dprintf_prologue(Tick when, const std::string &name)
{
    if (!name.empty() && ignore.match(name))
        return false;

    std::ostream &os = *dprintf_stream;

    if (when != MaxTick)
        ccprintf(os, "%7d: ", when);

    if (!name.empty())
        os << name << ": ";

    return true;
}

void
dump(Tick when, const std::string &name, const void *d, int len)
{
    const char *data = static_cast<const char *>(d);
    std::ostream &os = *dprintf_stream;
    int c, i, j;

    for (i = 0; i < len; i += 16) {
        if (!__dprintf_prologue(when, name))
            return;

        ccprintf(os, "%08x  ", i);
        c = len - i;
        if (c > 16) c = 16;

        for (j = 0; j < c; j++) {
            ccprintf(os, "%02x ", data[i + j] & 0xff);
            if ((j & 0xf) == 7 && j > 0)
                ccprintf(os, " ");
        }

        for (; j < 16; j++)
            ccprintf(os, "   ");
        ccprintf(os, "  ");

        for (j = 0; j < c; j++) {
            int ch = data[i + j] & 0x7f;
            ccprintf(os, "%c", (char)(isprint(ch) ? ch : ' '));
        }

        ccprintf(os, "\n");

        if (c < 16)
            break;
    }
}

} // namespace Trace
