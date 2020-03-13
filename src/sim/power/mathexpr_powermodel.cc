/*
 * Copyright (c) 2016-2017, 2020 ARM Limited
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

#include "sim/power/mathexpr_powermodel.hh"

#include <string>

#include "base/statistics.hh"
#include "params/MathExprPowerModel.hh"
#include "sim/mathexpr.hh"
#include "sim/power/thermal_model.hh"
#include "sim/sim_object.hh"

MathExprPowerModel::MathExprPowerModel(const Params *p)
    : PowerModelState(p), dyn_expr(p->dyn), st_expr(p->st)
{
}

void
MathExprPowerModel::startup()
{
    for (auto expr: {dyn_expr, st_expr}) {
        std::vector<std::string> vars = expr.getVariables();

        for (auto var: vars) {
            // Automatic variables:
            if (var == "temp" || var == "voltage" || var == "clock_period") {
                continue;
            }

            auto *info = Stats::resolve(var);
            fatal_if(!info, "Failed to evaluate %s in expression:\n%s\n",
                     var, expr.toStr());
            statsMap[var] = info;
        }
    }
}

double
MathExprPowerModel::eval(const MathExpr &expr) const
{
    return expr.eval(
        std::bind(&MathExprPowerModel::getStatValue,
                  this, std::placeholders::_1)
        );
}

double
MathExprPowerModel::getStatValue(const std::string &name) const
{
    using namespace Stats;

    // Automatic variables:
    if (name == "temp") {
        return _temp;
    } else if (name == "voltage") {
        return clocked_object->voltage();
    } else if (name=="clock_period") {
        return clocked_object->clockPeriod();
    }

    const auto it = statsMap.find(name);
    assert(it != statsMap.cend());
    const Info *info = it->second;

    // Try to cast the stat, only these are supported right now
    auto si = dynamic_cast<const ScalarInfo *>(info);
    if (si)
        return si->value();
    auto fi = dynamic_cast<const FormulaInfo *>(info);
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
