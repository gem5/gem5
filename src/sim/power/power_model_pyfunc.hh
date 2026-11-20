/*
 * Copyright (c) 2026 - The Unviersity of Wisconsin
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

#ifndef __SIM_POWERMODEL_PYFUNC_PM_HH__
#define __SIM_POWERMODEL_PYFUNC_PM_HH__

#include <optional>

#include "base/compiler.hh"
#include "base/statistics.hh"
#include "params/PowerModelPyFunc.hh"
#include "python/pybind11/pybind.hh"
#include "sim/eventq.hh"
#include "sim/power/power_model.hh"

namespace gem5
{

class GEM5_LOCAL PowerModelPyFunc : public PowerModelState
{
  private:
    pybind11::function st_func;
    pybind11::function dyn_func;

    pybind11::object on_sample_start;
    pybind11::object power_log;
    const std::string power_log_file;
    std::optional<Tick> last_log_tick;
    mutable bool preparing_log = false;

    const Cycles pwr_interval;
    const bool auto_start;

    bool sampling = false;
    bool in_interval = false;
    Tick last_sample_tick = 0;
    Tick sample_duration = 0;

    // Repeated non-interval queries at one tick share a Python evaluation.
    mutable std::optional<Tick> dyn_memo_tick;
    mutable std::optional<Tick> st_memo_tick;
    mutable double dyn_memo = 0.0;
    mutable double st_memo = 0.0;

    EventFunctionWrapper intervalEvent;

    bool powerLoggingEnabled() const;
    void logPower(double dyn, double st, double temp_k) const;
    void startSampleWindow();
    void sample();
    void powerAtInterval();
    double callPy(const pybind11::function &func) const;
    double memoized(const pybind11::function &func,
                    std::optional<Tick> &memo_tick, double &memo) const;
    void clearMemo();

    struct PowerModelPyFuncStats : public statistics::Group
    {
        PowerModelPyFuncStats(statistics::Group *parent);
        void sample(double dyn, double st);

        statistics::Histogram powerDist;
        statistics::Histogram dynamicPowerDist;
        statistics::Histogram staticPowerDist;
        statistics::Scalar powerMin;
        statistics::Scalar powerMax;
        statistics::Scalar dynamicPowerMin;
        statistics::Scalar dynamicPowerMax;
        statistics::Scalar staticPowerMin;
        statistics::Scalar staticPowerMax;
    } stats;

  public:
    PARAMS(PowerModelPyFunc);
    PowerModelPyFunc(const Params &p);

    void startup() override;

    double getDynamicPower() const override;
    double getStaticPower() const override;
    void preDumpStats() override;
    void resetStats() override;
    void setTemperature(Temperature temp) override;

    void startSampling();
    void stopSampling();

    bool
    inPowerAtInterval() const
    {
        return in_interval;
    }
    double getSampleDurationSeconds() const;
};

} // namespace gem5

#endif // __SIM_POWERMODEL_PYFUNC_PM_HH__
