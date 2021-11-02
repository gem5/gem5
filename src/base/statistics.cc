/*
 * Copyright (c) 2019-2020 Arm Limited
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

#include "base/statistics.hh"

#include <cassert>
#include <list>
#include <map>
#include <string>
#include <utility>

#include "base/callback.hh"
#include "base/logging.hh"
#include "sim/root.hh"

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(Stats, statistics);
namespace statistics
{

// We wrap these in a function to make sure they're built in time.
std::list<Info *> &
statsList()
{
    static std::list<Info *> the_list;
    return the_list;
}

MapType &
statsMap()
{
    static MapType the_map;
    return the_map;
}

void
InfoAccess::setInfo(Group *parent, Info *info)
{
    panic_if(statsMap().find(this) != statsMap().end() ||
             _info != nullptr,
             "shouldn't register stat twice!");

    // New-style stats are reachable through the hierarchy and
    // shouldn't be added to the global lists.
    if (parent) {
        _info = info;
        return;
    }

    statsList().push_back(info);

#ifndef NDEBUG
    std::pair<MapType::iterator, bool> result =
#endif
        statsMap().insert(std::make_pair(this, info));
    assert(result.second && "this should never fail");
    assert(statsMap().find(this) != statsMap().end());
}

void
InfoAccess::setParams(const StorageParams *params)
{
    info()->setStorageParams(params);
}

void
InfoAccess::setInit()
{
    info()->flags.set(init);
}

Info *
InfoAccess::info()
{
    if (newStyleStats()) {
        // New-style stats
        return _info;
    } else {
        // Legacy stats
        MapType::const_iterator i = statsMap().find(this);
        assert(i != statsMap().end());
        return (*i).second;
    }
}

const Info *
InfoAccess::info() const
{
    if (newStyleStats()) {
        // New-style stats
        return _info;
    } else {
        // Legacy stats
        MapType::const_iterator i = statsMap().find(this);
        assert(i != statsMap().end());
        return (*i).second;
    }
}

bool
InfoAccess::newStyleStats() const
{
    return _info != nullptr;
}

Formula::Formula(Group *parent, const char *name, const char *desc)
    : DataWrapVec<Formula, FormulaInfoProxy>(
            parent, name, units::Unspecified::get(), desc)

{
}

Formula::Formula(Group *parent, const char *name, const units::Base *unit,
                 const char *desc)
    : DataWrapVec<Formula, FormulaInfoProxy>(parent, name, unit, desc)
{
}

Formula::Formula(Group *parent, const char *name, const char *desc,
                 const Temp &r)
    : DataWrapVec<Formula, FormulaInfoProxy>(
            parent, name, units::Unspecified::get(), desc)
{
    *this = r;
}

Formula::Formula(Group *parent, const char *name, const units::Base *unit,
                 const char *desc, const Temp &r)
    : DataWrapVec<Formula, FormulaInfoProxy>(parent, name, unit, desc)
{
    *this = r;
}

const Formula &
Formula::operator=(const Temp &r)
{
    assert(!root && "Can't change formulas");
    root = r.getNodePtr();
    setInit();
    assert(size());
    return *this;
}

const Formula &
Formula::operator+=(Temp r)
{
    if (root)
        root = NodePtr(new BinaryNode<std::plus<Result> >(root, r));
    else {
        root = r.getNodePtr();
        setInit();
    }

    assert(size());
    return *this;
}

const Formula &
Formula::operator/=(Temp r)
{
    assert (root);
    root = NodePtr(new BinaryNode<std::divides<Result> >(root, r));

    assert(size());
    return *this;
}


void
Formula::result(VResult &vec) const
{
    if (root)
        vec = root->result();
}

Result
Formula::total() const
{
    return root ? root->total() : 0.0;
}

size_type
Formula::size() const
{
    if (!root)
        return 0;
    else
        return root->size();
}

void
Formula::reset()
{
}

bool
Formula::zero() const
{
    VResult vec;
    result(vec);
    for (VResult::size_type i = 0; i < vec.size(); ++i)
        if (vec[i] != 0.0)
            return false;
    return true;
}

std::string
Formula::str() const
{
    return root ? root->str() : "";
}

Handler resetHandler = NULL;
Handler dumpHandler = NULL;

void
registerHandlers(Handler reset_handler, Handler dump_handler)
{
    resetHandler = reset_handler;
    dumpHandler = dump_handler;
}

CallbackQueue dumpQueue;
CallbackQueue resetQueue;

void
processResetQueue()
{
    resetQueue.process();
}

void
processDumpQueue()
{
    dumpQueue.process();
}

void
registerResetCallback(const std::function<void()> &callback)
{
    resetQueue.push_back(callback);
}

bool _enabled = false;

bool
enabled()
{
    return _enabled;
}

void
enable()
{
    if (_enabled)
        fatal("Stats are already enabled");

    _enabled = true;
}

void
dump()
{
    if (dumpHandler)
        dumpHandler();
    else
        fatal("No registered statistics::dump handler");
}

void
reset()
{
    if (resetHandler)
        resetHandler();
    else
        fatal("No registered statistics::reset handler");
}

const Info *
resolve(const std::string &name)
{
    const auto &it = nameMap().find(name);
    if (it != nameMap().cend()) {
        return it->second;
    } else {
        return Root::root()->resolveStat(name);
    }
}

void
registerDumpCallback(const std::function<void()> &callback)
{
    dumpQueue.push_back(callback);
}

} // namespace statistics

void
debugDumpStats()
{
    statistics::dump();
}

} // namespace gem5
