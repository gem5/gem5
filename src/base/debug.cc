/*
 * Copyright (c) 2020 ARM Limited
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
 */

#include "base/debug.hh"

#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <csignal>

#include "base/cprintf.hh"
#include "base/logging.hh"

namespace gem5
{

namespace debug
{

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
    cprintf("debug::breakpoint suppressed, compiled with NDEBUG\n");
#endif
}

// Used to check the freshness of cached views of all flags.
FlagsMap &
allFlags()
{
    // Ensure that the special "All" compound debug flag has been created,
    // and avoid infinite recursion.
    static bool done = false;
    if (!done) {
        done = true;
        AllFlagsFlag::instance();
    }
    static FlagsMap flags;
    return flags;
}

bool Flag::_globalEnable = false;

Flag *
findFlag(const std::string &name)
{
    FlagsMap::iterator i = allFlags().find(name);
    if (i == allFlags().end()) {
        return NULL;
    }
    return i->second;
}

Flag::Flag(const char *name, const char *desc)
    : _name(name), _desc(desc)
{
    std::pair<FlagsMap::iterator, bool> result =
        allFlags().insert(std::make_pair(name, this));

    panic_if(!result.second, "Flag %s already defined!", name);

    sync();
}

Flag::~Flag()
{
    allFlags().erase(name());
}

void
Flag::globalEnable()
{
    _globalEnable = true;
    for (auto& i : allFlags())
        i.second->sync();
}

void
Flag::globalDisable()
{
    _globalEnable = false;
    for (auto& i : allFlags())
        i.second->sync();
}

SimpleFlag::SimpleFlag(const char *name, const char *desc, bool is_format)
  : Flag(name, desc), _isFormat(is_format)
{
    // Add non-format flags to the special "All" compound flag.
    if (!isFormat())
        AllFlagsFlag::instance().add(this);
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

AllFlagsFlag::AllFlagsFlag() : CompoundFlag("All",
        "Controls all debug flags. It should not be used within C++ code.", {})
{}

void
AllFlagsFlag::add(SimpleFlag *flag)
{
    ++_version;
    _kids.push_back(flag);
}

int AllFlagsFlag::_version = 0;

AllFlagsFlag &
AllFlagsFlag::instance()
{
    static AllFlagsFlag flag;
    return flag;
}

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

} // namespace debug

// add a set of functions that can easily be invoked from gdb
void
setDebugFlag(const char *string)
{
    debug::changeFlag(string, true);
}

void
clearDebugFlag(const char *string)
{
    debug::changeFlag(string, false);
}

void
dumpDebugFlags(std::ostream &os)
{
    using namespace debug;
    FlagsMap::iterator i = allFlags().begin();
    FlagsMap::iterator end = allFlags().end();
    for (; i != end; ++i) {
        SimpleFlag *f = dynamic_cast<SimpleFlag *>(i->second);
        if (f && f->tracing())
            ccprintf(os, "%s\n", f->name());
    }
}

} // namespace gem5
