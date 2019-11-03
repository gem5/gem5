/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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

#include "base/debug.hh"

#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <csignal>

#include "base/cprintf.hh"
#include "base/logging.hh"

using namespace std;

namespace Debug {

//
// This function will cause the process to signal itself with a
// SIGTRAP which is ignored if not in gdb, but will cause the debugger
// to break if in gdb.
//
void
breakpoint()
{
#ifndef NDEBUG
    kill(getpid(), SIGTRAP);
#else
    cprintf("Debug::breakpoint suppressed, compiled with NDEBUG\n");
#endif
}

//
// Flags for debugging purposes.  Primarily for trace.hh
//
int allFlagsVersion = 0;
FlagsMap &
allFlags()
{
    static FlagsMap flags;
    return flags;
}

bool SimpleFlag::_active = false;

Flag *
findFlag(const std::string &name)
{
    FlagsMap::iterator i = allFlags().find(name);
    if (i == allFlags().end())
        return NULL;
    return i->second;
}

Flag::Flag(const char *name, const char *desc)
    : _name(name), _desc(desc)
{
    pair<FlagsMap::iterator, bool> result =
        allFlags().insert(make_pair(name, this));

    if (!result.second)
        panic("Flag %s already defined!", name);

    ++allFlagsVersion;
}

Flag::~Flag()
{
    // should find and remove flag.
}

void
SimpleFlag::enableAll()
{
    _active = true;
    for (auto& i : allFlags())
        i.second->sync();
}

void
SimpleFlag::disableAll()
{
    _active = false;
    for (auto& i : allFlags())
        i.second->sync();
}

void
CompoundFlag::enable()
{
    for (auto& k : _kids)
        k->enable();
}

void
CompoundFlag::disable()
{
    for (auto& k : _kids)
        k->disable();
}

struct AllFlags : public Flag
{
    AllFlags()
        : Flag("All", "All Flags")
    {}

    void
    enable()
    {
        FlagsMap::iterator i = allFlags().begin();
        FlagsMap::iterator end = allFlags().end();
        for (; i != end; ++i)
            if (i->second != this)
                i->second->enable();
    }

    void
    disable()
    {
        FlagsMap::iterator i = allFlags().begin();
        FlagsMap::iterator end = allFlags().end();
        for (; i != end; ++i)
            if (i->second != this)
                i->second->disable();
    }
};

AllFlags theAllFlags;
Flag *const All = &theAllFlags;

bool
changeFlag(const char *s, bool value)
{
    Flag *f = findFlag(s);
    if (!f)
        return false;

    if (value)
        f->enable();
    else
        f->disable();

    return true;
}

} // namespace Debug

// add a set of functions that can easily be invoked from gdb
void
setDebugFlag(const char *string)
{
    Debug::changeFlag(string, true);
}

void
clearDebugFlag(const char *string)
{
    Debug::changeFlag(string, false);
}

void
dumpDebugFlags()
{
    using namespace Debug;
    FlagsMap::iterator i = allFlags().begin();
    FlagsMap::iterator end = allFlags().end();
    for (; i != end; ++i) {
        SimpleFlag *f = dynamic_cast<SimpleFlag *>(i->second);
        if (f && f->status())
            cprintf("%s\n", f->name());
    }
}
