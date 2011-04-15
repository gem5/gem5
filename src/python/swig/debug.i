/*
 * Copyright (c) 2006 The Regents of The University of Michigan
 * Copyright (c) 2010 The Hewlett-Packard Development Company
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

%module(package="m5.internal") debug

%{
#include <cassert>
#include <map>
#include <string>
#include <vector>

#include "base/debug.hh"
#include "base/types.hh"
#include "sim/debug.hh"

using namespace std;

typedef map<string, Debug::Flag *> FlagsMap;
typedef vector<Debug::Flag *> FlagsVec;

namespace Debug {
extern int allFlagsVersion;
FlagsMap &allFlags();
}

inline int
getAllFlagsVersion()
{
    return Debug::allFlagsVersion;
}

inline FlagsVec
getAllFlags()
{
    FlagsMap &flagsMap = Debug::allFlags();

    FlagsVec flags(flagsMap.size());

    int index = 0;
    FlagsMap::iterator i = flagsMap.begin();
    FlagsMap::iterator end = flagsMap.end();
    for (; i != end; ++i) {
        assert(index < flags.size());
        flags[index++] = i->second;
    }

    return flags;
}

%}

%ignore Debug::SimpleFlag::operator!;

%include <std_string.i>
%include <std_vector.i>
%include <stdint.i>

%include "base/debug.hh"
%include "base/types.hh"
%include "sim/debug.hh"

%template(AllFlags) std::vector<Debug::Flag *>;

int getAllFlagsVersion();
std::vector<Debug::Flag *> getAllFlags();
