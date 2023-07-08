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

#ifndef __SIM_MATHEXPR_POWERMODEL_PM_HH__
#define __SIM_MATHEXPR_POWERMODEL_PM_HH__

#include <unordered_map>

#include "params/MathExprPowerModel.hh"
#include "sim/mathexpr.hh"
#include "sim/power/power_model.hh"

namespace gem5
{

namespace statistics
{
    class Info;
}

/**
 * A Equation power model. The power is represented as a combination
 * of some stats and automatic variables (like temperature).
 */
class MathExprPowerModel : public PowerModelState
{
  public:

    typedef MathExprPowerModelParams Params;
    MathExprPowerModel(const Params &p);

    /**
     * Get the dynamic power consumption.
     *
     * @return Power (Watts) consumed by this object (dynamic component)
     */
    double getDynamicPower() const override { return eval(dyn_expr); }

    /**
     * Get the static power consumption.
     *
     * @return Power (Watts) consumed by this object (static component)
     */
    double getStaticPower() const override { return eval(st_expr); }

    /**
     * Get the value for a variable (maps to a stat)
     *
     * @param name Name of the variable to retrieve the value from
     *
     * @return Power (Watts) consumed by this object (static component)
     */
    double getStatValue(const std::string & name) const;

    void startup() override;
    void regStats() override;

  private:
    /**
     * Evaluate an expression in the context of this object, fatal if
     * evaluation fails.
     *
     * @param expr Expression to evaluate
     * @return Value of expression.
     */
    double eval(const MathExpr &expr) const;

    // Math expressions for dynamic and static power
    MathExpr dyn_expr, st_expr;

    // Map that contains relevant stats for this power model
    std::unordered_map<std::string, const statistics::Info*> statsMap;
};

} // namespace gem5

#endif
