/*
 * Copyright (c) 2015, 2021 Arm Limited
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

#ifndef __SIM_THERMAL_DOMAIN_HH__
#define __SIM_THERMAL_DOMAIN_HH__

#include <vector>

#include "base/statistics.hh"
#include "base/temperature.hh"
#include "params/ThermalDomain.hh"
#include "sim/power/thermal_entity.hh"
#include "sim/power/thermal_node.hh"
#include "sim/sim_object.hh"
#include "sim/sub_system.hh"

namespace gem5
{

template <class T> class ProbePointArg;

/**
 * A ThermalDomain is used to group objects under that operate under
 * the same temperature. Theses domains can be tied together using thermal
 * models (such as RC) to model temperature across components.
 */
class ThermalDomain : public SimObject, public ThermalEntity
{
  public:

    typedef ThermalDomainParams Params;
    ThermalDomain(const Params &p);

    /**
     * Get the startup temperature.
     *
     * @return Initial temperature of the domain
     */
    Temperature initialTemperature() const { return _initTemperature; }

    /**
     * Get the current temperature.
     *
     * @return current temperature of the domain
     */
    Temperature currentTemperature() const;

    /** Set/Get circuit node associated with this domain */
    void setNode(ThermalNode * n) { node = n; }
    ThermalNode * getNode() const { return node; }

    /** Get nodal equation imposed by this node */
    LinearEquation getEquation(ThermalNode * tn, unsigned n,
                               double step) const override;

    /**
      *  Emit a temperature update through probe points interface
      */
    void emitUpdate();

    /**
      *  Set the SubSystem reference we belong to
      */
    void setSubSystem(SubSystem * ss);

  private:
    const Temperature _initTemperature;
    ThermalNode * node;
    SubSystem * subsystem;

    /** Stat for reporting voltage of the domain */
    statistics::Value currentTemp;

    /** Probe to signal for temperature changes in this domain */
    ProbePointArg<Temperature> *ppThermalUpdate;

};

} // namespace gem5

#endif
