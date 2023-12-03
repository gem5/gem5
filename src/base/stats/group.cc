/*
 * Copyright (c) 2019, 2020 Arm Limited
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

#include "base/stats/group.hh"

#include "base/compiler.hh"
#include "base/logging.hh"
#include "base/named.hh"
#include "base/stats/info.hh"
#include "base/trace.hh"
#include "debug/Stats.hh"

namespace gem5
{

namespace statistics
{

Group::Group(Group *parent, const char *name) : mergedParent(nullptr)
{
    if (parent && name) {
        parent->addStatGroup(name, this);
    } else if (parent && !name) {
        parent->mergeStatGroup(this);
    }
}

Group::~Group() {}

void
Group::regStats()
{
    for (auto &g : mergedStatGroups)
        g->regStats();

    for (auto &g : statGroups) {
        if (debug::Stats) {
            [[maybe_unused]] const Named *named =
                dynamic_cast<const Named *>(this);
            DPRINTF(Stats, "%s: regStats in group %s\n",
                    named ? named->name() : "?", g.first);
        }
        g.second->regStats();
    }
}

void
Group::resetStats()
{
    for (auto &s : stats)
        s->reset();

    for (auto &g : mergedStatGroups)
        g->resetStats();

    for (auto &g : statGroups)
        g.second->resetStats();
}

void
Group::preDumpStats()
{
    for (auto &g : mergedStatGroups)
        g->preDumpStats();

    for (auto &g : statGroups)
        g.second->preDumpStats();
}

void
Group::addStat(statistics::Info *info)
{
    stats.push_back(info);
    if (mergedParent)
        mergedParent->addStat(info);
}

void
Group::addStatGroup(const char *name, Group *block)
{
    panic_if(!block, "Can't add null stat group %s", name);
    panic_if(block == this, "Stat group can't be added to itself");
    panic_if(statGroups.find(name) != statGroups.end(),
             "Stats of the same group share the same name `%s`.\n", name);

    statGroups[name] = block;
}

const Info *
Group::resolveStat(std::string name) const
{
    auto pos = name.find(".");
    if (pos == std::string::npos) {
        // look for the stat in this group
        for (auto &info : stats) {
            if (info->name == name) {
                return info;
            }
        }
    } else {
        // look for the stat in subgroups
        const std::string gname = name.substr(0, pos);
        for (auto &g : statGroups) {
            if (g.first == gname) {
                return g.second->resolveStat(name.substr(pos + 1));
            }
        }
    }

    // finally look for the stat in groups that have been merged
    for (auto &g : mergedStatGroups) {
        auto info = g->resolveStat(name);
        if (info) {
            return info;
        }
    }

    return nullptr;
}

void
Group::mergeStatGroup(Group *block)
{
    panic_if(!block, "No stat block provided");
    panic_if(block->mergedParent,
             "Stat group already merged into another group");
    panic_if(block == this, "Stat group can't merge with itself");

    // Track the new stat group
    mergedStatGroups.push_back(block);

    // We might not have seen stats that were associated with the
    // child group before it was merged, so add them here.
    for (auto &s : block->stats)
        addStat(s);

    // Setup the parent pointer so the child know that it needs to
    // register new stats with the parent.
    block->mergedParent = this;
}

const std::map<std::string, Group *> &
Group::getStatGroups() const
{
    return statGroups;
}

const std::vector<Info *> &
Group::getStats() const
{
    return stats;
}

} // namespace statistics
} // namespace gem5
