/* Copyright (c) 2012 Massachusetts Institute of Technology
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "model/electrical/MultiplexerCrossbar.h"

#include <vector>
#include <cmath>

#include "model/PortInfo.h"
#include "model/EventInfo.h"
#include "model/TransitionInfo.h"
#include "model/timing_graph/ElectricalNet.h"
#include "model/electrical/Multiplexer.h"

namespace DSENT
{
    using std::ceil;
    using std::vector;

    MultiplexerCrossbar::MultiplexerCrossbar(const String& instance_name_, const TechModel* tech_model_)
        : ElectricalModel(instance_name_, tech_model_)
    {
        initParameters();
        initProperties();
    }

    MultiplexerCrossbar::~MultiplexerCrossbar()
    {}

    void MultiplexerCrossbar::initParameters()
    {
        addParameterName("NumberInputs");
        addParameterName("NumberOutputs");
        addParameterName("NumberBits");
        addParameterName("BitDuplicate", "TRUE");
        return;
    }

    void MultiplexerCrossbar::initProperties()
    {
        return;
    }

    MultiplexerCrossbar* MultiplexerCrossbar::clone() const
    {
        // TODO
        return NULL;
    }

    void MultiplexerCrossbar::constructModel()
    {
        // Get Parameters
        unsigned int number_inputs = getParameter("NumberInputs").toUInt();
        unsigned int number_outputs = getParameter("NumberOutputs").toUInt();
        unsigned int number_bits = getParameter("NumberBits").toUInt();
        bool bit_duplicate = getParameter("BitDuplicate").toBool();

        ASSERT(number_inputs > 0, "[Error] " + getInstanceName() + " -> Number of inputs must be > 0!");
        ASSERT(number_outputs > 0, "[Error] " + getInstanceName() + " -> Number of outputs must be > 0!");
        ASSERT(number_bits > 0, "[Error] " + getInstanceName() + " -> Number of bits must be > 0!");

        unsigned int number_selects = (unsigned int)ceil(log2((double)number_inputs));
        getGenProperties()->set("NumberSelectsPerPort", number_selects);

        // Construct electrical ports and nets
        // Create input ports
        for(unsigned int i = 0; i < number_inputs; ++i)
        {
            createInputPort("In" + (String)i, makeNetIndex(0, number_bits-1));
        }
        // Create select signals
        for(unsigned int i = 0; i < number_outputs; ++i)
        {
            for(unsigned int j = 0; j < number_selects; ++j)
            {
                createInputPort(String::format("Sel%d_%d", i, j));
            }
        }
        // Create output ports
        for(unsigned int i = 0; i < number_outputs; ++i)
        {
            createOutputPort("Out" + (String)i, makeNetIndex(0, number_bits-1));
        }

        // Create energy, power, and area results
        addAreaResult(new AtomicResult("CrossbarWire"));
        addAreaResult(new AtomicResult("CrossbarFill"));
        createElectricalResults();
        getEventInfo("Idle")->setStaticTransitionInfos();
        createElectricalEventResult("Multicast0");
        getEventInfo("Multicast0")->setStaticTransitionInfos();
        for(unsigned int i = 1; i <= number_outputs; ++i)
        {
            createElectricalEventResult("Multicast" + (String)i);
            EventInfo* event_info = getEventInfo("Multicast" + (String)i);
            // Assuming that In0 is sending to Out0, Out1, ..., Outi
            // and other input ports are static
            for(unsigned int j = 1; j < number_inputs; ++j)
            {
                event_info->setStaticTransitionInfo("In" + (String)j);
            }
            for(unsigned int j = i; j < number_outputs; ++j)
            {
                for(unsigned int k = 0; k < number_selects; ++k)
                {
                    event_info->setStaticTransitionInfo(String::format("Sel%d_%d", j, k));
                }
            }
        }
        createElectricalEventResult("Crossbar");

        // Initiate multiplexers
        vector<String> mux_names(number_outputs, "");
        vector<Multiplexer*> muxs(number_outputs, NULL);
        for(unsigned int i = 0; i < number_outputs; ++i)
        {
            mux_names[i] = "Mux" + (String)i;
            muxs[i] = new Multiplexer(mux_names[i], getTechModel());
            muxs[i]->setParameter("NumberInputs", number_inputs);
            muxs[i]->setParameter("NumberBits", number_bits);
            muxs[i]->setParameter("BitDuplicate", bit_duplicate);
            muxs[i]->construct();
        }

        // Connect inputs and outputs to multiplexers
        for(unsigned int i = 0; i < number_outputs; ++i)
        {
            // Connect inputs
            for(unsigned int j = 0; j < number_inputs; ++j)
            {
                portConnect(muxs[i], "In" + (String)j, "In" + (String)j, makeNetIndex(0, number_bits-1));
            }

            // Connect select signals
            for(unsigned int j = 0; j < number_selects; ++j)
            {
                portConnect(muxs[i], "Sel" + (String)j, String::format("Sel%d_%d", i, j));
            }

            // Connect outputs
            portConnect(muxs[i], "Out", "Out" + (String)i, makeNetIndex(0, number_bits-1));
        }

        // Add area, power, and event results for each mux
        for(unsigned int i = 0; i < number_outputs; ++i)
        {
            addSubInstances(muxs[i], 1.0);
            addElectricalSubResults(muxs[i], 1.0);
            for(unsigned int j = 0; j <= number_outputs; ++j)
            {
                getEventResult("Multicast" + (String)j)->addSubResult(muxs[i]->getEventResult("Mux"), mux_names[i], 1.0);
            }
            getEventResult("Crossbar")->addSubResult(muxs[i]->getEventResult("Mux"), mux_names[i], 1.0);
        }

        // Estimate wiring area
        const String& crossbar_wire_layer = "Intermediate";
        addElectricalWireSubResult(crossbar_wire_layer, getAreaResult("CrossbarWire"), "Self", 1.0);
        double wire_width = getTechModel()->get("Wire->" + crossbar_wire_layer + "->MinWidth").toDouble();
        double wire_spacing = getTechModel()->get("Wire->" + crossbar_wire_layer + "->MinSpacing").toDouble();
        double wire_pitch = wire_width + wire_spacing;
        double wire_area = (number_bits * number_inputs * wire_pitch) * (number_bits * number_outputs * wire_pitch);
        getAreaResult("CrossbarWire")->setValue(wire_area);

        // Add filler area
        getAreaResult("Active")->addSubResult(getAreaResult("CrossbarFill"), "Self", 1.0);
        return;
    }

    void MultiplexerCrossbar::updateModel()
    {
        // Update all sub instances
        Model::updateModel();

        // Update filler area
        // Total Active area = max(stdcell active area, wiring area);
        double wire_area = getAreaResult("CrossbarWire")->calculateSum();
        double active_area = getAreaResult("Active")->calculateSum();
        double fill_area = 0.0;
        if(active_area < wire_area)
        {
            fill_area = wire_area - active_area;
        }
        getAreaResult("CrossbarFill")->setValue(fill_area);
        return;
    }

    void MultiplexerCrossbar::propagateTransitionInfo()
    {
        // The only thing can be updated are the input probabilities
        const unsigned int number_inputs = getParameter("NumberInputs").toUInt();
        const unsigned int number_outputs = getParameter("NumberOutputs").toUInt();

        const unsigned int number_selects = getGenProperties()->get("NumberSelectsPerPort").toUInt();

        for(unsigned int i = 0; i < number_outputs; ++i)
        {
            ElectricalModel* muxi = (ElectricalModel*)getSubInstance("Mux" + (String)i);
            for(unsigned int j = 0; j < number_inputs; ++j)
            {
                propagatePortTransitionInfo(muxi, "In" + (String)j, "In" + (String)j);
            }
            for(unsigned int j = 0; j < number_selects; ++j)
            {
                propagatePortTransitionInfo(muxi, "Sel" + (String)j, String::format("Sel%d_%d", i, j));
            }
            muxi->use();

            // Set output probability
            propagatePortTransitionInfo("Out" + (String)i, muxi, "Out");
        }

        return;
    }

} // namespace DSENT

