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

#include "model/electrical/RepeatedLink.h"

#include "model/PortInfo.h"
#include "model/EventInfo.h"
#include "model/TransitionInfo.h"
#include "model/std_cells/StdCellLib.h"
#include "model/std_cells/StdCell.h"
#include "model/timing_graph/ElectricalTimingTree.h"
#include "model/timing_graph/ElectricalTimingNode.h"
#include "model/timing_graph/ElectricalNet.h"
#include "model/timing_graph/ElectricalDriver.h"
#include "model/timing_graph/ElectricalDelay.h"
#include "model/timing_graph/ElectricalLoad.h"

namespace DSENT
{
    RepeatedLink::RepeatedLink(const String& instance_name_, const TechModel* tech_model_)
        : ElectricalModel(instance_name_, tech_model_)
    {
        m_repeater_ = NULL;
        m_repeater_load_ = NULL;
        m_timing_tree_ = NULL;

        initParameters();
        initProperties();
    }

    RepeatedLink::~RepeatedLink()
    {
        delete m_repeater_;
        delete m_repeater_load_;
        delete m_timing_tree_;
    }

    void RepeatedLink::initParameters()
    {
        addParameterName("NumberBits");
        addParameterName("WireLayer");
        addParameterName("WireWidthMultiplier", 1.0);
        addParameterName("WireSpacingMultiplier", 1.0);
        return;
    }

    void RepeatedLink::initProperties()
    {
        addPropertyName("WireLength");
        addPropertyName("Delay");
        addPropertyName("IsKeepParity", "TRUE");
        return;
    }

    RepeatedLink* RepeatedLink::clone() const
    {
        // TODO
        return NULL;
    }

    void RepeatedLink::constructModel()
    {
        // Get parameters
        unsigned int number_bits = getParameter("NumberBits").toUInt();
        const String& wire_layer = getParameter("WireLayer");
        double wire_width_multiplier = getParameter("WireWidthMultiplier").toDouble();
        double wire_spacing_multiplier = getParameter("WireSpacingMultiplier").toDouble();

        ASSERT(number_bits > 0, "[Error] " + getInstanceName() + 
                " -> Number of bits must be > 0!");
        ASSERT(getTechModel()->isWireLayerExist(wire_layer), "[Error] " + getInstanceName() + 
                " -> Wire layer does not exist!");
        ASSERT(wire_width_multiplier >= 1.0, "[Error] " + getInstanceName() + 
                " -> Wire width multiplier must be >= 1.0!");
        ASSERT(wire_spacing_multiplier >= 1.0, "[Error] " + getInstanceName() +
                " -> Wire spacing multiplier must be >= 1.0!");

        double wire_min_width = getTechModel()->get("Wire->" + wire_layer + "->MinWidth").toDouble();
        double wire_min_spacing = getTechModel()->get("Wire->" + wire_layer + "->MinSpacing").toDouble();

        double wire_width = wire_min_width * wire_width_multiplier;
        double wire_spacing = wire_min_spacing * wire_spacing_multiplier;

        double wire_cap_per_len = getTechModel()->calculateWireCapacitance(wire_layer, wire_width, wire_spacing, 1.0);
        double wire_res_per_len = getTechModel()->calculateWireResistance(wire_layer, wire_width, 1.0);

        getGenProperties()->set("WireWidth", wire_width);
        getGenProperties()->set("WireSpacing", wire_spacing);
        getGenProperties()->set("WireCapacitancePerLength", wire_cap_per_len);
        getGenProperties()->set("WireResistancePerLength", wire_res_per_len);

        // Create ports
        createInputPort("In", makeNetIndex(0, number_bits-1));
        createOutputPort("Out", makeNetIndex(0, number_bits-1));

        // Create area, power, and event results
        createElectricalAtomicResults();        
        createElectricalEventAtomicResult("Send");

        // Create connections
        // Since the length is not set yet, we only to virtual fan-in and virtual fan-out
        createNet("InTmp");
        createNet("OutTmp");
        assignVirtualFanin("InTmp", "In");
        assignVirtualFanout("Out", "OutTmp");

        // Build Electrical Connectivity
        createLoad("In_Cap");
        createDelay("In_to_Out_delay");
        createDriver("Out_Ron", false); // Indicate this driver is not sizable

        ElectricalLoad* in_cap = getLoad("In_Cap");
        ElectricalDelay* in_to_out_delay = getDelay("In_to_Out_delay");
        ElectricalDriver* out_ron = getDriver("Out_Ron");

        getNet("InTmp")->addDownstreamNode(in_cap);
        in_cap->addDownstreamNode(in_to_out_delay);
        in_to_out_delay->addDownstreamNode(out_ron);
        out_ron->addDownstreamNode(getNet("OutTmp"));

        // Init a repeater and a load to mimic a segment of a repeated link
        m_repeater_ = getTechModel()->getStdCellLib()->createStdCell("INV", "Repeater");
        m_repeater_->construct();
        m_repeater_load_ = new ElectricalLoad("RepeaterIn_Cap", this);
        // Make path repeater_ -> repeater_load_
        // to catch the repeater's input/output cap and ensure only one inverter delay
        // is added
        m_repeater_->getNet("Y")->addDownstreamNode(m_repeater_load_);
        // Init a timing object to calculate delay
        m_timing_tree_ = new ElectricalTimingTree("RepeatedLink", this);
        m_timing_tree_->performCritPathExtract(m_repeater_->getNet("A"));
        return;
    }

    void RepeatedLink::updateModel()
    {
        unsigned int number_bits = getParameter("NumberBits").toUInt();

        // Get properties
        double wire_length = getProperty("WireLength").toDouble();
        double required_delay = getProperty("Delay").toDouble();
        bool isKeepParity = getProperty("IsKeepParity").toBool();

        ASSERT(wire_length >= 0, "[Error] " + getInstanceName() +
                " -> Wire length must be >= 0!");
        ASSERT(required_delay >= 0, "[Error] " + getInstanceName() + 
                " -> Required delay must be >= 0!");

        const String& wire_layer = getParameter("WireLayer");
        double wire_width = getGenProperties()->get("WireWidth").toDouble();
        double wire_spacing = getGenProperties()->get("WireSpacing").toDouble();

        // Calculate the total wire cap and total wire res
        double wire_cap_per_len = getGenProperties()->get("WireCapacitancePerLength").toDouble();
        double wire_res_per_len = getGenProperties()->get("WireResistancePerLength").toDouble();
        double total_wire_cap = wire_cap_per_len * wire_length;
        double total_wire_res = wire_res_per_len * wire_length;

        m_repeater_->update();

        unsigned int increment_segments = (isKeepParity)? 2:1;
        unsigned int number_segments = increment_segments;
        double delay;
        m_repeater_->setMinDrivingStrength();
        m_repeater_->getNet("Y")->setDistributedCap(total_wire_cap / number_segments);
        m_repeater_->getNet("Y")->setDistributedRes(total_wire_res / number_segments);
        m_repeater_load_->setLoadCap(m_repeater_->getNet("A")->getTotalDownstreamCap());
        m_timing_tree_->performCritPathExtract(m_repeater_->getNet("A"));
        delay = m_timing_tree_->calculateCritPathDelay(m_repeater_->getNet("A")) * number_segments;

        // If everything is 0, use number_segments min-sized repeater
        if(wire_length != 0)
        {
            // Set the initial number of segments based on isKeepParity
            double last_min_size_delay = 0;
            unsigned int iteration = 0;

            // First set the repeater to the minimum driving strength
            last_min_size_delay = delay;

            Log::printLine(getInstanceName() + " -> Beginning Repeater Insertion");

            while(required_delay < delay)
            {
                Log::printLine(getInstanceName() + " -> Repeater Insertion Iteration " + (String)iteration + 
                        ": Required delay = " + (String)required_delay + 
                        ", Delay = " + (String)delay + 
                        ", Slack = " + (String)(required_delay - delay) + 
                        ", Number of repeaters = " + (String)number_segments);

                // Size up if timing is not met
                while(required_delay < delay)
                {
                    if(m_repeater_->hasMaxDrivingStrength())
                    {
                        break;
                    }
                    m_repeater_->increaseDrivingStrength();
                    m_repeater_load_->setLoadCap(m_repeater_->getNet("A")->getTotalDownstreamCap());
                    m_timing_tree_->performCritPathExtract(m_repeater_->getNet("A"));
                    delay = m_timing_tree_->calculateCritPathDelay(m_repeater_->getNet("A")) * number_segments;

                    iteration++;
                    Log::printLine(getInstanceName() + " -> Slack: " + (String)(required_delay - delay));
                }
                // Increase number of segments if timing is not met
                if(required_delay < delay)
                {
                    number_segments += increment_segments;
                    m_repeater_->setMinDrivingStrength();
                    m_repeater_->getNet("Y")->setDistributedCap(total_wire_cap / number_segments);
                    m_repeater_->getNet("Y")->setDistributedRes(total_wire_res / number_segments);
                    m_repeater_load_->setLoadCap(m_repeater_->getNet("A")->getTotalDownstreamCap());
                    m_timing_tree_->performCritPathExtract(m_repeater_->getNet("A"));
                    delay = m_timing_tree_->calculateCritPathDelay(m_repeater_->getNet("A")) * number_segments;

                    // Abort if adding more min sized repeaters does not decrease the delay
                    if(delay > last_min_size_delay)
                    {
                        break;
                    }
                    last_min_size_delay = delay;
                }
            }
            Log::printLine(getInstanceName() + " -> Repeater Insertion Ended after Iteration: " + (String)iteration + 
                    ": Required delay = " + (String)required_delay + 
                    ", Delay = " + (String)delay + 
                    ", Slack = " + (String)(required_delay - delay) + 
                    ", Number of repeaters = " + (String)number_segments);

            // Print a warning if the timing is not met
            if(required_delay < delay)
            {
                const String& warning_msg = "[Warning] " + getInstanceName() + " -> Timing not met" + 
                    ": Required delay = " + (String)required_delay + 
                    ", Delay = " + (String)delay + 
                    ", Slack = " + (String)(required_delay - delay) +
                    ", Number of repeaters = " + (String)number_segments;
                Log::printLine(std::cerr, warning_msg);
            }
        }

        // Update electrical interfaces
        getLoad("In_Cap")->setLoadCap(m_repeater_->getNet("A")->getTotalDownstreamCap());
        getDelay("In_to_Out_delay")->setDelay(delay);
        getDriver("Out_Ron")->setOutputRes(m_repeater_->getDriver("Y_Ron")->getOutputRes() + (total_wire_res / number_segments));

        getGenProperties()->set("NumberSegments", number_segments);

        // Update area, power results
        resetElectricalAtomicResults();
        addElecticalAtomicResultValues(m_repeater_, number_segments * number_bits);
        double wire_area = wire_length * (wire_width + wire_spacing) * number_bits;
        addElecticalWireAtomicResultValue(wire_layer, wire_area);

        return;
    }

    void RepeatedLink::useModel()
    {
        // Update the transition information for the modeled repeater
        // Since we only modeled one repeater. So the transition information for 0->0 and 1->1 
        // is averaged out
        const TransitionInfo& trans_In = getInputPort("In")->getTransitionInfo();
        double average_static_transition = (trans_In.getNumberTransitions00() + trans_In.getNumberTransitions11()) / 2.0;
        TransitionInfo mod_trans_In(average_static_transition, trans_In.getNumberTransitions01(), average_static_transition);
        m_repeater_->getInputPort("A")->setTransitionInfo(mod_trans_In);
        m_repeater_->use();

        // Get parameters
        unsigned int number_bits = getParameter("NumberBits").toUInt();
        unsigned int number_segments = getGenProperties()->get("NumberSegments").toUInt();

        // Propagate the transition information
        propagateTransitionInfo();

        // Update leakage power
        double power = 0.0;
        power += m_repeater_->getNddPowerResult("Leakage")->calculateSum() * number_segments * number_bits;
        getNddPowerResult("Leakage")->setValue(power);

        // Update event result
        double energy = 0.0;
        energy += m_repeater_->getEventResult("INV")->calculateSum() * number_segments * number_bits;
        getEventResult("Send")->setValue(energy);

        return;
    }

    void RepeatedLink::propagateTransitionInfo()
    {
        unsigned int number_segments = getGenProperties()->get("NumberSegments");

        if((number_segments % 2) == 0)
        {
            propagatePortTransitionInfo("Out", "In");
        }
        else
        {
            const TransitionInfo& trans_In = getInputPort("In")->getTransitionInfo();
            TransitionInfo trans_Out(trans_In.getNumberTransitions11(), trans_In.getNumberTransitions01(), trans_In.getNumberTransitions00());
            getOutputPort("Out")->setTransitionInfo(trans_Out);
        }
        return;
    }

} // namespace DSENT

