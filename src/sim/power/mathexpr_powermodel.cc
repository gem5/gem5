/*
 * Copyright (c) 2016 ARM Limited
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
 *
 * Authors: David Guillen Fandos
 */

#include "sim/power/mathexpr_powermodel.hh"

#include "base/statistics.hh"
#include "params/MathExprPowerModel.hh"
#include "sim/mathexpr.hh"
#include "sim/power/thermal_model.hh"
#include "sim/sim_object.hh"

MathExprPowerModel::MathExprPowerModel(const Params *p)
    : PowerModelState(p), dyn_expr(p->dyn), st_expr(p->st)
{
    // Calculate the name of the object we belong to
    std::vector<std::string> path;
    tokenize(path, name(), '.', true);
    // It's something like xyz.power_model.pm2
    assert(path.size() > 2);
    for (unsigned i = 0; i < path.size() - 2; i++)
        basename += path[i] + ".";
}

void
MathExprPowerModel::startup()
{
    // Create a map with stats and pointers for quick access
    // Has to be done here, since we need access to the statsList
    for (auto & i: Stats::statsList())
        if (i->name.find(basename) == 0)
            stats_map[i->name.substr(basename.size())] = i;
}

double
MathExprPowerModel::getStatValue(const std::string &name) const
{
    using namespace Stats;

    // Automatic variables:
    if (name == "temp")
        return _temp;

    // Try to cast the stat, only these are supported right now
    Info *info = stats_map.at(name);

    ScalarInfo *si = dynamic_cast<ScalarInfo*>(info);
    if (si)
        return si->value();
    FormulaInfo *fi = dynamic_cast<FormulaInfo*>(info);
    if (fi)
        return fi->total();

    panic("Unknown stat type!\n");
}

void
MathExprPowerModel::regStats()
{
    PowerModelState::regStats();
}

MathExprPowerModel*
MathExprPowerModelParams::create()
{
    return new MathExprPowerModel(this);
}
