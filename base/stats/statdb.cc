/*
 * Copyright (c) 2003-2004 The Regents of The University of Michigan
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

#include "base/misc.hh"
#include "base/trace.hh"
#include "base/statistics.hh"
#include "base/stats/bin.hh"
#include "base/stats/statdb.hh"

using namespace std;

namespace Statistics {
namespace Database {

StatData *
find(void *stat)
{
    stat_map_t::const_iterator i = map().find(stat);

    if (i == map().end())
        return NULL;

    return (*i).second;
}

void
regBin(MainBin *bin, const std::string &_name)
{
    bins().push_back(bin);
    DPRINTF(Stats, "registering %s\n", _name);
}

void
regStat(void *stat, StatData *data)
{
    if (map().find(stat) != map().end())
        panic("shouldn't register stat twice!");

    stats().push_back(data);

#ifndef NDEBUG
    pair<stat_map_t::iterator, bool> result =
#endif
        map().insert(make_pair(stat, data));
    assert(result.second && "this should never fail");
    assert(map().find(stat) != map().end());
}

void
regPrint(void *stat)
{
    StatData *data = find(stat);
    assert(data);
    data->flags |= print;
}

TheDatabase &db()
{
    static TheDatabase db;
    return db;
}

/* namespace Database */ }
/* namespace Statistics */ }
