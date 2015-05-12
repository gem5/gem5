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

#include "sim/power/thermal_model.hh"

#include "base/statistics.hh"
#include "params/ThermalCapacitor.hh"
#include "params/ThermalNode.hh"
#include "params/ThermalReference.hh"
#include "params/ThermalResistor.hh"
#include "sim/clocked_object.hh"
#include "sim/linear_solver.hh"
#include "sim/power/thermal_domain.hh"
#include "sim/sim_object.hh"

/**
 * ThermalNode
 */
ThermalNode::ThermalNode(const Params *p)
    : SimObject(p), id(-1), isref(false), temp(0.0f)
{
}

ThermalNode *
ThermalNodeParams::create()
{
    return new ThermalNode(this);
}

/**
 * ThermalReference
 */
ThermalReference::ThermalReference(const Params *p)
    : SimObject(p), _temperature(p->temperature), node(NULL)
{
}

ThermalReference *
ThermalReferenceParams::create()
{
    return new ThermalReference(this);
}

void
ThermalReference::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(_temperature);
}

void
ThermalReference::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(_temperature);
}

LinearEquation
ThermalReference::getEquation(ThermalNode * n, unsigned nnodes,
                              double step) const {
    // Just return an empty equation
    return LinearEquation(nnodes);
}

/**
 * ThermalResistor
 */
ThermalResistor::ThermalResistor(const Params *p)
    : SimObject(p), _resistance(p->resistance), node1(NULL), node2(NULL)
{
}

ThermalResistor *
ThermalResistorParams::create()
{
    return new ThermalResistor(this);
}

void
ThermalResistor::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(_resistance);
}

void
ThermalResistor::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(_resistance);
}

LinearEquation
ThermalResistor::getEquation(ThermalNode * n, unsigned nnodes,
                             double step) const
{
    // i[n] = (Vn2 - Vn1)/R
    LinearEquation eq(nnodes);

    if (n != node1 && n != node2)
        return eq;

    if (node1->isref)
        eq[eq.cnt()] += -node1->temp / _resistance;
    else
        eq[node1->id] += -1.0f / _resistance;

    if (node2->isref)
        eq[eq.cnt()] += node2->temp / _resistance;
    else
        eq[node2->id] += 1.0f / _resistance;

    // We've assumed n was node1, reverse if necessary
    if (n == node2)
        eq *= -1.0f;

    return eq;
}

/**
 * ThermalCapacitor
 */
ThermalCapacitor::ThermalCapacitor(const Params *p)
    : SimObject(p), _capacitance(p->capacitance), node1(NULL), node2(NULL)
{
}

ThermalCapacitor *
ThermalCapacitorParams::create()
{
    return new ThermalCapacitor(this);
}

void
ThermalCapacitor::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(_capacitance);
}

void
ThermalCapacitor::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(_capacitance);
}

LinearEquation
ThermalCapacitor::getEquation(ThermalNode * n, unsigned nnodes,
                              double step) const
{
    // i(t) = C * d(Vn2 - Vn1)/dt
    // i[n] = C/step * (Vn2 - Vn1 - Vn2[n-1] + Vn1[n-1])
    LinearEquation eq(nnodes);

    if (n != node1 && n != node2)
        return eq;

    eq[eq.cnt()] += _capacitance / step * (node1->temp - node2->temp);

    if (node1->isref)
        eq[eq.cnt()] += _capacitance / step * (-node1->temp);
    else
        eq[node1->id] += -1.0f * _capacitance / step;

    if (node2->isref)
        eq[eq.cnt()] += _capacitance / step * (node2->temp);
    else
        eq[node2->id] += 1.0f * _capacitance / step;

    // We've assumed n was node1, reverse if necessary
    if (n == node2)
        eq *= -1.0f;

    return eq;
}

/**
 * ThermalModel
 */
ThermalModel::ThermalModel(const Params *p)
    : ClockedObject(p), stepEvent(this), _step(p->step)
{
}

ThermalModel *
ThermalModelParams::create()
{
    return new ThermalModel(this);
}

void
ThermalModel::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(_step);
}

void
ThermalModel::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(_step);
}

void
ThermalModel::doStep()
{
    // Calculate new temperatures!
    // For each node in the system, create the kirchhoff nodal equation
    LinearSystem ls(eq_nodes.size());
    for (unsigned i = 0; i < eq_nodes.size(); i++) {
        auto n = eq_nodes[i];
        LinearEquation node_equation (eq_nodes.size());
        for (auto e : entities) {
            LinearEquation eq = e->getEquation(n, eq_nodes.size(), _step);
            node_equation = node_equation + eq;
        }
        ls[i] = node_equation;
    }

    // Get temperatures for this iteration
    std::vector <double> temps = ls.solve();
    for (unsigned i = 0; i < eq_nodes.size(); i++)
        eq_nodes[i]->temp = temps[i];

    // Schedule next computation
    schedule(stepEvent, curTick() + SimClock::Int::s * _step);

    // Notify everybody
    for (auto dom : domains)
        dom->emitUpdate();
}

void
ThermalModel::startup()
{
    // Look for nodes connected to voltage references, these
    // can be just set to the reference value (no nodal equation)
    for (auto ref : references) {
        ref->node->temp = ref->_temperature;
        ref->node->isref = true;
    }
    // Setup the initial temperatures
    for (auto dom : domains)
        dom->getNode()->temp = dom->initialTemperature();

    // Create a list of unknown temperature nodes
    for (auto n : nodes) {
        bool found = false;
        for (auto ref : references)
            if (ref->node == n) {
                found = true;
                break;
            }
        if (!found)
            eq_nodes.push_back(n);
    }

    // Assign each node an ID
    for (unsigned i = 0; i < eq_nodes.size(); i++)
        eq_nodes[i]->id = i;

    // Schedule first thermal update
    schedule(stepEvent, curTick() + SimClock::Int::s * _step);
}

void ThermalModel::addDomain(ThermalDomain * d) {
    domains.push_back(d);
    entities.push_back(d);
}
void ThermalModel::addReference(ThermalReference * r) {
    references.push_back(r);
    entities.push_back(r);
}
void ThermalModel::addCapacitor(ThermalCapacitor * c) {
    capacitors.push_back(c);
    entities.push_back(c);
}
void ThermalModel::addResistor(ThermalResistor * r) {
    resistors.push_back(r);
    entities.push_back(r);
}

double ThermalModel::getTemp() const {
    // Just pick the highest temperature
    double temp = 0;
    for (auto & n : eq_nodes)
        temp = std::max(temp, n->temp);
    return temp;
}
