/*
 * Copyright (c) 2016, 2018, 2021 ARM Limited
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

#ifndef __SIM_POWER_POWER_MODEL_HH__
#define __SIM_POWER_POWER_MODEL_HH__

#include "base/statistics.hh"
#include "base/temperature.hh"
#include "enums/PMType.hh"
#include "params/PowerModel.hh"
#include "params/PowerModelState.hh"
#include "sim/probe/probe.hh"

namespace gem5
{

class SimObject;
class ClockedObject;

/**
 * A PowerModelState is an abstract class used as interface to get power
 * figures out of SimObjects
 */
class PowerModelState : public SimObject
{
  public:

    typedef PowerModelStateParams Params;
    PowerModelState(const Params &p);

    /**
     * Get the dynamic power consumption.
     *
     * @return Power (Watts) consumed by this object (dynamic component)
     */
    virtual double getDynamicPower() const = 0;

    /**
     * Get the static power consumption.
     *
     * @return Power (Watts) consumed by this object (static component)
     */
    virtual double getStaticPower() const = 0;

    /**
     * Temperature update.
     *
     * @param temp Current temperature of the HW part
     */
    virtual void setTemperature(Temperature temp) { _temp = temp; }

    void setClockedObject(ClockedObject * clkobj) {
        clocked_object = clkobj;
    }

  protected:

    /** Current temperature */
    Temperature _temp;

    /** The clocked object we belong to */
    ClockedObject * clocked_object;

    statistics::Value dynamicPower, staticPower;
};

/**
 * @sa \ref gem5PowerModel "gem5 Power Model"
 *
 * A PowerModel is a class containing a power model for a SimObject.
 * The PM describes the power consumption for every power state.
 */
class PowerModel : public SimObject
{
  public:

    typedef PowerModelParams Params;
    PowerModel(const Params &p);

    /**
     * Get the dynamic power consumption.
     *
     * @return Power (Watts) consumed by this object (dynamic component)
     */
    double getDynamicPower() const;

    /**
     * Get the static power consumption.
     *
     * @return Power (Watts) consumed by this object (static component)
     */
    double getStaticPower() const;

    void setClockedObject(ClockedObject *clkobj);

    virtual void regProbePoints();

    void thermalUpdateCallback(const Temperature &temp);

  protected:
    /** Listener class to catch thermal events */
    class ThermalProbeListener : public ProbeListenerArgBase<Temperature>
    {
      public:
        ThermalProbeListener(PowerModel &_pm, ProbeManager *pm,
                      const std::string &name)
            : ProbeListenerArgBase(pm, name), pm(_pm) {}

        void notify(const Temperature &temp)
        {
            pm.thermalUpdateCallback(temp);
        }

      protected:
        PowerModel &pm;
    };

    /** Actual power models (one per power state) */
    std::vector<PowerModelState*> states_pm;

    /** Listener to catch temperature changes in the SubSystem */
    std::unique_ptr<ThermalProbeListener> thermalListener;

    /** The subsystem this power model belongs to */
    SubSystem * subsystem;

    /** The clocked object we belong to */
    ClockedObject * clocked_object;

    /** The type of power model - collects all power, static or dynamic only */
    enums::PMType power_model_type;

    statistics::Value dynamicPower, staticPower;
};

} // namespace gem5

#endif
