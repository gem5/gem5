/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

#include <ctype.h>
#include <fstream>
#include <iostream>
#include <list>
#include <string>
#include <vector>

#include "base/misc.hh"
#include "base/trace.hh"
#include "base/str.hh"

using namespace std;

namespace Trace {
const string DefaultName("global");
FlagVec flags(NumFlags, false);

//
// This variable holds the output stream for debug information.  Other
// than setting up/redirecting this stream, do *NOT* reference this
// directly; use DebugOut() (see below) to access this stream for
// output.
//
ostream *dprintf_stream = &cerr;

int dprintf_ignore_size;
vector<string> dprintf_ignore;
vector<vector<string> > ignore_tokens;
vector<int> ignore_size;

bool
dprintf_ignore_name(const string &name)
{
    vector<string> name_tokens;
    tokenize(name_tokens, name, '.');
    int ntsize = name_tokens.size();

    for (int i = 0; i < dprintf_ignore_size; ++i) {
        bool match = true;
        int jstop = ignore_size[i];
        for (int j = 0; j < jstop; ++j) {
            if (j >= ntsize)
                break;

            const string &ignore = ignore_tokens[i][j];
            if (ignore != "*" && ignore != name_tokens[j]) {
                match = false;
                break;
            }
        }

        if (match == true)
            return true;
    }

    return false;
}


Log theLog;

Log::Log()
{
    size = 0;
    buffer = NULL;
}


void
Log::init(int _size)
{
    if (buffer != NULL) {
        fatal("Trace::Log::init called twice!");
    }

    size = _size;

    buffer = new (Record *)[size];

    for (int i = 0; i < size; ++i) {
        buffer[i] = NULL;
    }

    nextRecPtr = &buffer[0];
    wrapRecPtr = &buffer[size];
}


Log::~Log()
{
    for (int i = 0; i < size; ++i) {
        delete buffer[i];
    }

    delete [] buffer;
}


void
Log::append(Record *rec)
{
    // dump record to output stream if there's one open
    if (dprintf_stream != NULL) {
        rec->dump(*dprintf_stream);
    }

    // no buffering: justget rid of it now
    if (buffer == NULL) {
        delete rec;
        return;
    }

    Record *oldRec = *nextRecPtr;

    if (oldRec != NULL) {
        // log has wrapped: overwrite
        delete oldRec;
    }

    *nextRecPtr = rec;

    if (++nextRecPtr == wrapRecPtr) {
        nextRecPtr = &buffer[0];
    }
}


void
Log::dump(ostream &os)
{
    if (buffer == NULL) {
        return;
    }

    Record **bufPtr = nextRecPtr;

    if (*bufPtr == NULL) {
        // next record slot is empty: log must not be full yet.
        // start dumping from beginning of buffer
        bufPtr = buffer;
    }

    do {
        Record *rec = *bufPtr;

        rec->dump(os);

        if (++bufPtr == wrapRecPtr) {
            bufPtr = &buffer[0];
        }
    } while (bufPtr != nextRecPtr);
}

PrintfRecord::~PrintfRecord()
{
    delete &args;
}


void
PrintfRecord::dump(ostream &os)
{
    string fmt = "";

    if (!name.empty()) {
        fmt = "%s: " + fmt;
        args.prepend(name);
    }

    if (cycle != (Tick)-1) {
        fmt = "%7d: " + fmt;
        args.prepend(cycle);
    }

    fmt += format;

    args.dump(os, fmt);
    os.flush();
}



RawDataRecord::RawDataRecord(Tick _cycle,
                                    const uint8_t *_data, int _len)
    : Record(_cycle), len(_len)
{
    data = new uint8_t[len];
    memcpy(data, _data, len);
}


RawDataRecord::~RawDataRecord()
{
    delete [] data;
}


void
RawDataRecord::dump(ostream &os)
{
    int c, i, j;

    for (i = 0; i < len; i += 16) {
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
            ccprintf(os,
                     "%c", (char)(isprint(ch) ? ch : ' '));
        }

        ccprintf(os, "\n");

        if (c < 16)
            break;
    }
}
} // namespace Trace

//
// Returns the current output stream for debug information.  As a
// wrapper around Trace::dprintf_stream, this handles cases where debug
// information is generated in the process of parsing .ini options,
// before we process the option that sets up the debug output stream
// itself.
//
std::ostream &
DebugOut()
{
    return *Trace::dprintf_stream;
}

/////////////////////////////////////////////
//
// C-linkage functions for invoking from gdb
//
/////////////////////////////////////////////

//
// Dump trace buffer to specified file (cout if NULL)
//
extern "C"
void
dumpTrace(const char *filename)
{
    if (filename != NULL) {
        ofstream out(filename);
        Trace::theLog.dump(out);
        out.close();
    }
    else {
        Trace::theLog.dump(cout);
    }
}


//
// Turn on/off trace output to cerr.  Typically used when trace output
// is only going to circular buffer, but you want to see what's being
// sent there as you step through some code in gdb.  This uses the
// same facility as the "trace to file" feature, and will print error
// messages rather than clobbering an existing ostream pointer.
//
extern "C"
void
echoTrace(bool on)
{
    if (on) {
        if (Trace::dprintf_stream != NULL) {
            cerr << "Already echoing trace to a file... go do a 'tail -f'"
                 << " on that file instead." << endl;
        } else {
            Trace::dprintf_stream = &cerr;
        }
    } else {
        if (Trace::dprintf_stream != &cerr) {
            cerr << "Not echoing trace to cerr." << endl;
        } else {
            Trace::dprintf_stream = NULL;
        }
    }
}
