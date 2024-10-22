/*
 * Copyright (c) 2024 The Regents of the University of California
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

#include "test_objects/stat_tester.hh"

#include "base/stats/group.hh"

namespace gem5
{

void
ScalarStatTester::setStats()
{
    stats.scalar = params.value;
}

ScalarStatTester::ScalarStatTesterStats::ScalarStatTesterStats(
    statistics::Group *parent,
    const ScalarStatTesterParams &params
) : statistics::Group(parent),
    scalar(this,
        params.name.c_str(),
        statistics::units::Count::get(),
        params.description.c_str()
    )
{
}

void
VectorStatTester::setStats()
{
    for (int i = 0; i < params.values.size(); i++)
    {
        stats.vector[i] = (params.values[i]);
    }
}

VectorStatTester::VectorStatTesterStats::VectorStatTesterStats(
    statistics::Group *parent,
    const VectorStatTesterParams &params
) : statistics::Group(parent),
    vector(this,
        params.name.c_str(),
        statistics::units::Count::get(),
        params.description.c_str()
    )
{
    vector.init(params.values.size());
    for (int i = 0; i < params.values.size(); i++)
    {
        if (params.subnames.size() > i) {
            vector.subname(i, params.subnames[i]);
        } else {
            vector.subname(i, std::to_string(i));
        }
        if (params.subdescs.size() > i) {
            vector.subdesc(i, params.subdescs[i]);
        }
    }
}

} // namespace gem5
