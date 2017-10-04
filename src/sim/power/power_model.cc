/*
 * Copyright (c) 2016-2018 ARM Limited
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

#include "sim/power/power_model.hh"

#include "base/statistics.hh"
#include "params/PowerModel.hh"
#include "params/PowerModelState.hh"
#include "sim/clocked_object.hh"
#include "sim/sub_system.hh"

PowerModelState::PowerModelState(const Params *p)
    : SimObject(p), _temp(0), clocked_object(NULL)
{
}

PowerModel::PowerModel(const Params *p)
    : SimObject(p), states_pm(p->pm), subsystem(p->subsystem),
      clocked_object(NULL), power_model_type(p->pm_type)
{
    panic_if(subsystem == NULL,
             "Subsystem is NULL! This is not acceptable for a PowerModel!\n");
    subsystem->registerPowerProducer(this);
    // The temperature passed here will be overwritten, if there is
    // a thermal model present
    for (auto & pms: states_pm){
        pms->setTemperature(p->ambient_temp);
    }

}

void
PowerModel::setClockedObject(ClockedObject * clkobj)
{
    this->clocked_object = clkobj;

    for (auto & pms: states_pm)
        pms->setClockedObject(clkobj);
}

void
PowerModel::thermalUpdateCallback(const double & temp)
{
    for (auto & pms: states_pm)
        pms->setTemperature(temp);
}

void
PowerModel::regProbePoints()
{
    thermalListener.reset(new ThermalProbeListener (
        *this, this->subsystem->getProbeManager(), "thermalUpdate"
    ));
}

PowerModel*
PowerModelParams::create()
{
    return new PowerModel(this);
}

double
PowerModel::getDynamicPower() const
{
    assert(clocked_object);

    if (power_model_type == Enums::PMType::Static) {
        // This power model only collects static data
        return 0;
    }
    std::vector<double> w = clocked_object->powerState->getWeights();

    // Same number of states (excluding UNDEFINED)
    assert(w.size() - 1 == states_pm.size());

    // Make sure we have no UNDEFINED state
    warn_if(w[Enums::PwrState::UNDEFINED] > 0,
        "SimObject in UNDEFINED power state! Power figures might be wrong!\n");

    double power = 0;
    for (unsigned i = 0; i < states_pm.size(); i++)
        if (w[i + 1] > 0.0f)
            power += states_pm[i]->getDynamicPower() * w[i + 1];

    return power;
}

double
PowerModel::getStaticPower() const
{
    assert(clocked_object);

    std::vector<double> w = clocked_object->powerState->getWeights();

    if (power_model_type == Enums::PMType::Dynamic) {
        // This power model only collects dynamic data
        return 0;
    }

    // Same number of states (excluding UNDEFINED)
    assert(w.size() - 1 == states_pm.size());

    // Make sure we have no UNDEFINED state
    if (w[0] > 0)
        warn("SimObject in UNDEFINED power state! "
             "Power figures might be wrong!\n");

    // We have N+1 states, being state #0 the default 'UNDEFINED' state
    double power = 0;
    for (unsigned i = 0; i < states_pm.size(); i++)
        // Don't evaluate power if the object hasn't been in that state
        // This fixes issues with NaNs and similar.
        if (w[i + 1] > 0.0f)
            power += states_pm[i]->getStaticPower() * w[i + 1];

    return power;
}
