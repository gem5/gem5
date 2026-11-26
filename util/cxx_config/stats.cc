/*
 * Copyright (c) 2014 ARM Limited
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

/**
 * @file
 *
 *  C++-only configuration stats handling example
 *
 *  Register with: gem5::statistics::registerHandlers(statsReset, statsDump)
 */

#include <functional>
#include <iostream>
#include <string>

#include "base/statistics.hh"
#include "sim/root.hh"
#include "stats.hh"

namespace CxxConfig
{

namespace
{

void
visitStats(gem5::statistics::Group &group, const std::string &prefix,
           const std::function<void(gem5::statistics::Info *,
                                    const std::string &)> &visit)
{
    for (auto *stat : group.getStats()) {
        visit(stat, prefix + stat->name);
    }
    for (const auto &[name, child] : group.getStatGroups()) {
        visitStats(*child, prefix + name + ".", visit);
    }
}

} // anonymous namespace

void
statsPrepare()
{
    visitStats(*gem5::Root::root(), "",
               [](auto *stat, const auto &) { stat->prepare(); });
}

void
statsDump()
{
    std::cerr << "Stats dump\n";
    gem5::statistics::processDumpQueue();
    gem5::Root::root()->preDumpStats();
    statsPrepare();

    visitStats(*gem5::Root::root(), "", [](auto *stat, const auto &name) {
        if (!stat->flags.isSet(gem5::statistics::display)) {
            return;
        }
        if (auto *scalar =
                dynamic_cast<gem5::statistics::ScalarInfo *>(stat)) {
            std::cerr << "SCALAR " << name << ' ' << scalar->value() << '\n';
        } else if (auto *vector =
                       dynamic_cast<gem5::statistics::VectorInfo *>(stat)) {
            const auto results = vector->value();
            for (size_t index = 0; index < results.size(); ++index) {
                std::cerr << "VECTOR " << name << '[' << index << "] "
                          << results[index] << '\n';
            }
            std::cerr << "VTOTAL " << name << ' ' << vector->total() << '\n';
        } else {
            std::cerr << "?????? " << name << '\n';
        }
    });
}

void
statsReset()
{
    std::cerr << "Stats reset\n";
    gem5::Root::root()->resetStats();
    gem5::statistics::processResetQueue();
}

void
statsEnable()
{
    visitStats(*gem5::Root::root(), "",
               [](auto *stat, const auto &) { stat->enable(); });
}

} // namespace CxxConfig
