/*
 * Copyright (c) 2021 Daniel R. Carvalho
 * Copyright (c) 2019 Arm Limited
 * All rights reserved.
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

#include "base/stats/info.hh"

#include <cctype>

#include "base/cprintf.hh"
#include "base/debug.hh"
#include "base/logging.hh"
#include "base/stats/storage.hh"
#include "base/str.hh"

namespace gem5
{

namespace statistics
{

std::string Info::separatorString = "::";

int Info::id_count = 0;

int debug_break_id = -1;

NameMapType &
nameMap()
{
    static NameMapType the_map;
    return the_map;
}

Info::Info()
    : flags(none), precision(-1), prereq(0), storageParams()
{
    id = id_count++;
    if (debug_break_id >= 0 and debug_break_id == id)
        debug::breakpoint();
}

Info::~Info()
{
}

StorageParams const*
Info::getStorageParams() const
{
    return storageParams.get();
}

void
Info::setStorageParams(const StorageParams *const params)
{
    return storageParams.reset(params);
}

bool
validateStatName(const std::string &name)
{
    if (name.empty())
        return false;

    std::vector<std::string> vec;
    tokenize(vec, name, '.', false);
    std::vector<std::string>::const_iterator item = vec.begin();
    while (item != vec.end()) {
        if (item->empty())
            return false;

        std::string::const_iterator c = item->begin();

        // The first character is different
        if (!isalpha(*c) && *c != '_')
            return false;

        // The rest of the characters have different rules.
        while (++c != item->end()) {
            if (!isalnum(*c) && *c != '_')
                return false;
        }

        ++item;
    }

    return true;
}

void
Info::setName(const std::string &name, bool old_style)
{
    if (!validateStatName(name))
        panic("invalid stat name '%s'", name);

    // We only register the stat with the nameMap() if we are using
    // old-style stats without a parent group. New-style stats should
    // be unique since their names should correspond to a member
    // variable.
    if (old_style) {
        auto p = nameMap().insert(make_pair(name, this));

        panic_if(!p.second, "same statistic name used twice! name=%s\n", name);
    }

    this->name = name;
}

bool
Info::less(Info *stat1, Info *stat2)
{
    const std::string &name1 = stat1->name;
    const std::string &name2 = stat2->name;

    std::vector<std::string> v1;
    std::vector<std::string> v2;

    tokenize(v1, name1, '.');
    tokenize(v2, name2, '.');

    size_type last = std::min(v1.size(), v2.size()) - 1;
    for (off_type i = 0; i < last; ++i)
        if (v1[i] != v2[i])
            return v1[i] < v2[i];

    // Special compare for last element.
    if (v1[last] == v2[last])
        return v1.size() < v2.size();
    else
        return v1[last] < v2[last];

    return false;
}

bool
Info::baseCheck() const
{
    if (!(flags & statistics::init)) {
        panic("this is stat number %d\n"
              "Not all stats have been initialized.\n"
              "You may need to add <ParentClass>::regStats() to a"
              " new SimObject's regStats() function. Name: %s",
              id, name);
        return false;
    }

    if ((flags & display) && name.empty()) {
        panic("all printable stats must be named");
        return false;
    }

    return true;
}

void
Info::enable()
{
}

void
VectorInfo::enable()
{
    size_type s = size();
    if (subnames.size() < s)
        subnames.resize(s);
    if (subdescs.size() < s)
        subdescs.resize(s);
}

void
VectorDistInfo::enable()
{
    size_type s = size();
    if (subnames.size() < s)
        subnames.resize(s);
    if (subdescs.size() < s)
        subdescs.resize(s);
}

void
Vector2dInfo::enable()
{
    if (subnames.size() < x)
        subnames.resize(x);
    if (subdescs.size() < x)
        subdescs.resize(x);
    if (y_subnames.size() < y)
        y_subnames.resize(y);
}

} // namespace statistics
} // namespace gem5
