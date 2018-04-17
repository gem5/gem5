/*
 * Copyright (c) 2015 ARM Limited
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

#ifndef __SIM_THERMAL_MODEL_HH__
#define __SIM_THERMAL_MODEL_HH__

#include <vector>

#include "params/ThermalCapacitor.hh"
#include "params/ThermalModel.hh"
#include "params/ThermalReference.hh"
#include "params/ThermalResistor.hh"
#include "sim/clocked_object.hh"
#include "sim/power/thermal_domain.hh"
#include "sim/power/thermal_entity.hh"
#include "sim/power/thermal_node.hh"
#include "sim/sim_object.hh"


/**
 * A ThermalResistor is used to model a thermal resistance between two
 * thermal domains. This domains can be either a reference (fixed temp.) or
 * a heat producer (power source).
 */
class ThermalResistor : public SimObject, public ThermalEntity
{
  public:
    typedef ThermalResistorParams Params;
    ThermalResistor(const Params *p);

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    void setNodes(ThermalNode * n1, ThermalNode * n2) {
        node1 = n1;
        node2 = n2;
    }

    LinearEquation getEquation(ThermalNode * tn, unsigned n,
                               double step) const override;

  private:
    /* Resistance value in K/W */
    double _resistance;
    /* Nodes connected to the resistor */
    ThermalNode * node1, * node2;
};

/**
 * A ThermalCapacitor is used to model a thermal capacitance between two
 * thermal domains. This domains can be either a reference (fixed temp.) or
 * a heat producer (power source).
 */
class ThermalCapacitor : public SimObject, public ThermalEntity
{
  public:
    typedef ThermalCapacitorParams Params;
    ThermalCapacitor(const Params *p);

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    LinearEquation getEquation(ThermalNode * tn, unsigned n,
                               double step) const override;

    void setNodes(ThermalNode * n1, ThermalNode * n2) {
        node1 = n1;
        node2 = n2;
    }

  private:
    /* Capacitance value in J/K */
    double _capacitance;
    /* Nodes connected to the resistor */
    ThermalNode * node1, * node2;
};

/**
 * A ThermalReference is a thermal domain with fixed temperature.
 * It's the homologue to the voltage source in a circuit.
 */
class ThermalReference : public SimObject, public ThermalEntity
{
  public:
    typedef ThermalReferenceParams Params;
    ThermalReference(const Params *p);

    void setNode(ThermalNode * n) {
        node = n;
    }

    LinearEquation getEquation(ThermalNode * tn, unsigned n,
                               double step) const override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    /* Fixed temperature value in centigrate degrees */
    double _temperature;
    /* Nodes connected to the resistor */
    ThermalNode * node;
};


/**
 * @sa \ref gem5PowerModel "gem5 Thermal Model"
 *
 * A ThermalModel is the element which ties all thermal objects
 * together and provides the thermal solver to the system.
 * It is reponsible for updating temperature for all Thermal
 * Domains over time by reading power from simobjects.
 */
class ThermalModel : public ClockedObject
{
  public:
    typedef ThermalModelParams Params;
    ThermalModel(const Params *p);

    void addDomain(ThermalDomain * d);
    void addReference(ThermalReference * r);
    void addCapacitor(ThermalCapacitor * c);
    void addResistor(ThermalResistor * r);

    void addNode(ThermalNode * n) { nodes.push_back(n); }

    double getTemp() const;

    void startup() override;
    void doStep();

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
  private:

    /* Keep track of all components used for the thermal model */
    std::vector <ThermalDomain *> domains;
    std::vector <ThermalReference *> references;
    std::vector <ThermalCapacitor *> capacitors;
    std::vector <ThermalResistor *> resistors;

    std::vector <ThermalEntity *> entities;

    /* Keep a list of the instantiated nodes */
    std::vector <ThermalNode*> nodes;
    std::vector <ThermalNode*> eq_nodes;

    /** Stepping event to update the model values */
    EventFunctionWrapper stepEvent;

    /** Step in seconds for thermal updates */
    double _step;

};

#endif
