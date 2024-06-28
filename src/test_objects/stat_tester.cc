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

#include <set>

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

void
Vector2dStatTester::setStats()
{
    for (int i = 0; i < params.x_size; i++)
    {
        for (int j = 0; j < params.y_size; j++)
        {
            stats.vector2d[i][j] = (params.values[j + i * params.y_size]);
        }
    }
}

Vector2dStatTester::Vector2dStatTesterStats::Vector2dStatTesterStats(
    statistics::Group *parent,
    const Vector2dStatTesterParams &params
) : statistics::Group(parent),
    vector2d(this,
        params.name.c_str(),
        statistics::units::Count::get(),
        params.description.c_str()
    )
{
    vector2d.init(params.x_size, params.y_size);

    assert(params.x_size * params.y_size == params.values.size());

    for (int i = 0; i < params.x_size; i++)
    {
        if (params.subnames.size() > i) {
            vector2d.subname(i, params.subnames[i]);
        } else {
            vector2d.subname(i, std::to_string(i));
        }
        if (params.subdescs.size() > i) {
            vector2d.subdesc(i, params.subdescs[i]);
        }
    }
    for (int j = 0; j < params.y_size; j++)
    {
        if (params.ysubnames.size() > j) {
            vector2d.ysubname(j, params.ysubnames[j]);
        } else {
            vector2d.ysubname(j, std::to_string(j));
        }

    }

}

void
SparseHistStatTester::setStats()
{
    for (auto sample : params.samples) {
        stats.sparse_histogram.sample(sample);
    }
}

SparseHistStatTester::SparseHistStatTesterStats::SparseHistStatTesterStats(
    statistics::Group *parent,
    const SparseHistStatTesterParams &params
) : statistics::Group(parent),
    sparse_histogram(
        this,
        params.name.c_str(),
        statistics::units::Count::get(),
        params.description.c_str()
    )
{
    sparse_histogram.init(
        (std::set(params.samples.begin(), params.samples.end())).size()
    );
}

} // namespace gem5
