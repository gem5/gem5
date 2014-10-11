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

#include "model/electrical/router/Router.h"

#include <cmath>
#include <vector>

#include "model/PortInfo.h"
#include "model/EventInfo.h"
#include "model/TransitionInfo.h"
#include "model/ModelGen.h"
#include "model/std_cells/StdCellLib.h"
#include "model/std_cells/StdCell.h"
#include "model/electrical/router/RouterInputPort.h"
#include "model/electrical/router/RouterSwitchAllocator.h"
#include "model/timing_graph/ElectricalNet.h"

namespace DSENT
{
    using std::sqrt;
    using std::vector;

    using LibUtil::castStringVector;
    using LibUtil::vectorToString;

    Router::Router(const String& instance_name_, const TechModel* tech_model_)
        : ElectricalModel(instance_name_, tech_model_)
    {
        initParameters();
        initProperties();
    }

    Router::~Router()
    {}

    void Router::initParameters()
    {
        addParameterName("NumberInputPorts");
        addParameterName("NumberOutputPorts");
        addParameterName("NumberBitsPerFlit");
        addParameterName("NumberVirtualNetworks");
        addParameterName("NumberVirtualChannelsPerVirtualNetwork");
        addParameterName("NumberBuffersPerVirtualChannel");
        // Spec for input port
        addParameterName("InputPort->BufferModel");
        // Spec for crossbar
        addParameterName("CrossbarModel");
        // Spec for switch allocator
        addParameterName("SwitchAllocator->ArbiterModel");
        // Spec for clock tree
        addParameterName("ClockTreeModel");
        addParameterName("ClockTree->NumberLevels");
        addParameterName("ClockTree->WireLayer");
        addParameterName("ClockTree->WireWidthMultiplier");
        addParameterName("ClockTree->WireSpacingMultiplier", 3.0);
        return;
    }

    void Router::initProperties()
    {
        return;
    }

    Router* Router::clone() const
    {
        // TODO
        return NULL;
    }

    void Router::constructModel()
    {
        // Get parameters
        unsigned int number_input_ports = getParameter("NumberInputPorts").toUInt();
        unsigned int number_output_ports = getParameter("NumberOutputPorts").toUInt();
        unsigned int number_bits_per_flit = getParameter("NumberBitsPerFlit").toUInt();

        ASSERT(number_input_ports > 0, "[Error] " + getInstanceName() + 
                " -> Number of input ports must be > 0!");
        ASSERT(number_output_ports > 0, "[Error] " + getInstanceName() + 
                " -> Number of output ports must be > 0!");
        ASSERT(number_bits_per_flit > 0, "[Error] " + getInstanceName() + 
                " -> Number of bits per buffer must be > 0!");

        // Create ports
        createInputPort("CK");
        for(unsigned int i = 0; i < number_input_ports; ++i)
        {
            createInputPort("FlitIn" + (String)i, makeNetIndex(0, number_bits_per_flit-1));
        }
        for(unsigned int i = 0; i < number_output_ports; ++i)
        {
            createOutputPort("FlitOut" + (String)i, makeNetIndex(0, number_bits_per_flit-1));
        }

        // Create area, power, event results
        createElectricalResults();
        getEventInfo("Idle")->setStaticTransitionInfos();
        getEventInfo("Idle")->setTransitionInfo("CK", TransitionInfo(0.0, 1.0, 0.0));

        createElectricalEventResult("ReadBuffer");
        getEventInfo("ReadBuffer")->setTransitionInfo("CK", TransitionInfo(0.0, 1.0, 0.0));
        createElectricalEventResult("WriteBuffer");
        getEventInfo("WriteBuffer")->setTransitionInfo("CK", TransitionInfo(0.0, 1.0, 0.0));
        for(unsigned int i = 1; i <= number_output_ports; ++i)
        {
            createElectricalEventResult("TraverseCrossbar->Multicast" + (String)i);
            getEventInfo("TraverseCrossbar->Multicast" + (String)i)->setTransitionInfo("CK", TransitionInfo(0.0, 1.0, 0.0));
        }
        createElectricalEventResult("ArbitrateSwitch->ArbitrateStage1");
        createElectricalEventResult("ArbitrateSwitch->ArbitrateStage2");
        createElectricalEventResult("DistributeClock");
        getEventInfo("DistributeClock")->setTransitionInfo("CK", TransitionInfo(0.0, 1.0, 0.0));

        // Create intermediate nets
        createNet("PipelineReg0_In");
        createNet("PipelineReg0_Out");
        createNet("PipelineReg1_In");
        createNet("PipelineReg1_Out");
        for(unsigned int i = 0; i < number_output_ports; ++i)
        {
            createNet("PipelineReg2_In" + (String)i);
            createNet("PipelineReg2_Out" + (String)i);
        }

        createRouterInputPort();
        createSwitchAllocator();
        createVirtualChannelAllocator();
        createCrossbar();
        createClockTree();
        createPipelineReg();

        // Get generated numbers
        unsigned int number_crossbar_selects = getGenProperties()->get("Crossbar->NumberSelects");

        // Add write buffer event
        getEventResult("WriteBuffer")->addSubResult(getSubInstance("PipelineReg0")->getEventResult("DFFD"), "PipelineReg0", number_bits_per_flit);
        getEventResult("WriteBuffer")->addSubResult(getSubInstance("PipelineReg0")->getEventResult("DFFQ"), "PipelineReg0", number_bits_per_flit);
        getEventResult("WriteBuffer")->addSubResult(getSubInstance("PipelineReg0")->getEventResult("CK"), "PipelineReg0", number_bits_per_flit);
        getEventResult("WriteBuffer")->addSubResult(getSubInstance("InputPort")->getEventResult("WriteBuffer"), "InputPort", 1.0);

        // Add read buffer event
        getEventResult("ReadBuffer")->addSubResult(getSubInstance("InputPort")->getEventResult("ReadBuffer"), "InputPort", 1.0);
        getEventResult("ReadBuffer")->addSubResult(getSubInstance("PipelineReg1")->getEventResult("DFFD"), "PipelineReg1", number_bits_per_flit);
        getEventResult("ReadBuffer")->addSubResult(getSubInstance("PipelineReg1")->getEventResult("DFFQ"), "PipelineReg1", number_bits_per_flit);
        getEventResult("ReadBuffer")->addSubResult(getSubInstance("PipelineReg1")->getEventResult("CK"), "PipelineReg1", number_bits_per_flit);

        // Add crossbar traversal event
        for(unsigned int i = 1; i <= number_output_ports; ++i)
        {
            Result* traverse_crossbar_event = getEventResult("TraverseCrossbar->Multicast" + (String)i);
            traverse_crossbar_event->addSubResult(getSubInstance("Crossbar_Sel_DFF")->getEventResult("DFFD"), "Crossbar_Sel_DFF", number_crossbar_selects);
            traverse_crossbar_event->addSubResult(getSubInstance("Crossbar_Sel_DFF")->getEventResult("DFFQ"), "Crossbar_Sel_DFF", number_crossbar_selects);
            traverse_crossbar_event->addSubResult(getSubInstance("Crossbar_Sel_DFF")->getEventResult("CK"), "Crossbar_Sel_DFF", number_crossbar_selects);
            traverse_crossbar_event->addSubResult(getSubInstance("Crossbar")->getEventResult("Multicast" + (String)i), "Crossbar", 1.0);
            for(unsigned int j = 0; j < i; ++j)
            {
                traverse_crossbar_event->addSubResult(getSubInstance("PipelineReg2_" + (String)j)->getEventResult("DFFD"), "PipelineReg2_" + (String)j, number_bits_per_flit);
                traverse_crossbar_event->addSubResult(getSubInstance("PipelineReg2_" + (String)j)->getEventResult("DFFQ"), "PipelineReg2_" + (String)j, number_bits_per_flit);
                traverse_crossbar_event->addSubResult(getSubInstance("PipelineReg2_" + (String)j)->getEventResult("CK"), "PipelineReg2_" + (String)j, number_bits_per_flit);
            }
        }

        // Add stage1 allocator arbitrate
        Result* arb_sw_stage1_event = getEventResult("ArbitrateSwitch->ArbitrateStage1");
        arb_sw_stage1_event->addSubResult(getSubInstance("SwitchAllocator")->getEventResult("ArbitrateStage1"), "SwitchAllocator", 1.0);

        // Add stage2 allocator arbitrate
        Result* arb_sw_stage2_event = getEventResult("ArbitrateSwitch->ArbitrateStage2");
        arb_sw_stage2_event->addSubResult(getSubInstance("SwitchAllocator")->getEventResult("ArbitrateStage2"), "SwitchAllocator", 1.0);

        // Add CK event
        getEventResult("DistributeClock")->addSubResult(getSubInstance("ClockTree")->getEventResult("Send"), "ClockTree", 1.0);
        return;
    }

    void Router::updateModel()
    {
        // Get parameters
        unsigned int number_output_ports = getParameter("NumberOutputPorts").toUInt();

        // Update other components
        getSubInstance("PipelineReg0")->update();
        getSubInstance("InputPort")->update();
        getSubInstance("PipelineReg1")->update();
        getSubInstance("Crossbar_Sel_DFF")->update();
        getSubInstance("Crossbar")->update();
        for(unsigned int i = 0; i < number_output_ports; ++i)
        {
            getSubInstance("PipelineReg2_" + (String)i)->update();
        }
        getSubInstance("SwitchAllocator")->update();

        // Update clock tree
        double total_clock_tree_cap = getNet("CK")->getTotalDownstreamCap();
        double router_area = getAreaResult("Active")->calculateSum();
        Model* clock_tree = getSubInstance("ClockTree");
        clock_tree->setProperty("SitePitch", sqrt(router_area));
        clock_tree->setProperty("TotalLoadCapPerBit", total_clock_tree_cap);
        clock_tree->update();

        return;
    }

    void Router::propagateTransitionInfo()
    {
        // Update probability
        unsigned int number_output_ports = getParameter("NumberOutputPorts");

        // Current event
        const String& current_event = getGenProperties()->get("UseModelEvent");

        ElectricalModel* pipeline_reg0 = (ElectricalModel*)getSubInstance("PipelineReg0");
        propagatePortTransitionInfo(pipeline_reg0, "D", "FlitIn0");
        propagatePortTransitionInfo(pipeline_reg0, "CK", "CK");
        pipeline_reg0->use();

        ElectricalModel* input_port = (ElectricalModel*)getSubInstance("InputPort");
        propagatePortTransitionInfo(input_port, "FlitIn", pipeline_reg0, "Q");
        propagatePortTransitionInfo(input_port, "CK", "CK");
        input_port->getGenProperties()->set("UseModelEvent", "ReadWrite");
        input_port->use();

        ElectricalModel* pipeline_reg1 = (ElectricalModel*)getSubInstance("PipelineReg1");
        propagatePortTransitionInfo(pipeline_reg1, "D", "FlitIn0");
        propagatePortTransitionInfo(pipeline_reg1, "CK", "CK");
        pipeline_reg1->use();

        ElectricalModel* crossbar_sel_dff = (ElectricalModel*)getSubInstance("Crossbar_Sel_DFF");
        assignPortTransitionInfo(crossbar_sel_dff, "D", TransitionInfo());
        propagatePortTransitionInfo(crossbar_sel_dff, "CK", "CK");
        crossbar_sel_dff->use();

        ElectricalModel* crossbar = (ElectricalModel*)getSubInstance("Crossbar");
        bool is_crossbar_event = false;
        for(unsigned int i = 1; i <= number_output_ports; ++i)
        {
            if(current_event == ("TraverseCrossbar->Multicast" + (String)i))
            {
                is_crossbar_event = true;
                // Assume the flit is sent from port 0 to port 0~i-1
                // Apply default transition info
                crossbar->applyTransitionInfo("Multicast" + (String)i);
                // Overwrite transition info
                propagatePortTransitionInfo(crossbar, "In0", "FlitIn0");
                break;
            }
        }
        if(is_crossbar_event == false)
        {
            crossbar->applyTransitionInfo("Multicast1");
            propagatePortTransitionInfo(crossbar, "In0", "FlitIn0");
        }
        crossbar->use();

        vector<ElectricalModel*> pipeline_reg2s(number_output_ports, NULL);
        for(unsigned int i = 0; i < number_output_ports; ++i)
        {
            pipeline_reg2s[i] = (ElectricalModel*)getSubInstance("PipelineReg2_" + (String)i);
            propagatePortTransitionInfo(pipeline_reg2s[i], "D", "FlitIn0");
            propagatePortTransitionInfo(pipeline_reg2s[i], "CK", "CK");
            pipeline_reg2s[i]->use();
        }

        ElectricalModel* sw_allocator = (ElectricalModel*)getSubInstance("SwitchAllocator");
        if(current_event == "ArbitrateSwitch->ArbitrateStage1")
        {
            sw_allocator->applyTransitionInfo("ArbitrateStage1");
        }
        else if(current_event == "ArbitrateSwitch->ArbitrateStage2")
        {
            sw_allocator->applyTransitionInfo("ArbitrateStage2");
        }
        else
        {
            sw_allocator->applyTransitionInfo("Idle");
        }
        sw_allocator->use();

        ElectricalModel* clock_tree = (ElectricalModel*)getSubInstance("ClockTree");
        propagatePortTransitionInfo(clock_tree, "In", "CK");
        clock_tree->use();
        return;
    }

    void Router::createRouterInputPort()
    {
        // Get parameters
        unsigned int number_input_ports = getParameter("NumberInputPorts").toUInt();
        unsigned int number_vns = getParameter("NumberVirtualNetworks").toUInt();
        const String& number_vcs_per_vn = getParameter("NumberVirtualChannelsPerVirtualNetwork");
        const String& number_bufs_per_vc = getParameter("NumberBuffersPerVirtualChannel");
        unsigned int number_bits_per_flit = getParameter("NumberBitsPerFlit").toUInt();
        const String& buffer_model = getParameter("InputPort->BufferModel");

        // Init input port model
        const String& input_port_name = "InputPort";
        RouterInputPort* input_port = new RouterInputPort(input_port_name, getTechModel());
        input_port->setParameter("NumberVirtualNetworks", number_vns);
        input_port->setParameter("NumberVirtualChannelsPerVirtualNetwork", number_vcs_per_vn);
        input_port->setParameter("NumberBuffersPerVirtualChannel", number_bufs_per_vc);
        input_port->setParameter("NumberBitsPerFlit", number_bits_per_flit);
        input_port->setParameter("BufferModel", buffer_model);
        input_port->construct();

        unsigned int number_input_port_outputs = input_port->getGenProperties()->get("NumberOutputs");
        unsigned int number_input_port_addr_bits = input_port->getGenProperties()->get("NumberAddressBits");
        getGenProperties()->set("InputPort->NumberOutputs", number_input_port_outputs);
        getGenProperties()->set("InputPort->NumberAddressBits", number_input_port_addr_bits);

        unsigned int total_number_vcs = input_port->getGenProperties()->get("TotalNumberVirtualChannels");
        getGenProperties()->set("TotalNumberVirtualChannels", total_number_vcs);

        // Add the instance and the results
        addSubInstances(input_port, number_input_ports);
        addElectricalSubResults(input_port, number_input_ports);

        // Create connections
        createNet("InputPort_In", makeNetIndex(0, number_bits_per_flit-1));
        createNet("InputPort_Out", makeNetIndex(0, number_bits_per_flit-1));

        assignVirtualFanout("InputPort_In", "PipelineReg0_Out");
        portConnect(input_port, "FlitIn", "InputPort_In");
        portConnect(input_port, "CK", "CK");
        portConnect(input_port, "FlitOut", "InputPort_Out");
        assignVirtualFanin("PipelineReg1_In", "InputPort_Out");

        return;
    }

    void Router::createVirtualChannelAllocator()
    {}

    void Router::createSwitchAllocator()
    {
        // Get parameters
        unsigned int number_input_ports = getParameter("NumberInputPorts").toUInt();
        unsigned int number_output_ports = getParameter("NumberOutputPorts").toUInt();
        unsigned int total_number_vcs = getGenProperties()->get("TotalNumberVirtualChannels").toUInt();
        const String& arb_model = getParameter("SwitchAllocator->ArbiterModel");

        // Init switch allocator model
        const String& sw_allocator_name = "SwitchAllocator";
        RouterSwitchAllocator* sw_allocator = new RouterSwitchAllocator(sw_allocator_name, getTechModel());
        sw_allocator->setParameter("NumberInputPorts", number_input_ports);
        sw_allocator->setParameter("NumberOutputPorts", number_output_ports);
        sw_allocator->setParameter("TotalNumberVirtualChannels", total_number_vcs);
        sw_allocator->setParameter("ArbiterModel", arb_model);
        sw_allocator->construct();

        // Add the instance and the results
        addSubInstances(sw_allocator, 1.0);
        addElectricalSubResults(sw_allocator, 1.0);

        // Create connections (currently connect CK only)
        portConnect(sw_allocator, "CK", "CK");
        return;
    }

    void Router::createCrossbar()
    {
        // Get parameters
        const String& crossbar_model = getParameter("CrossbarModel");
        unsigned int number_input_ports = getParameter("NumberInputPorts").toUInt();
        unsigned int number_output_ports = getParameter("NumberOutputPorts").toUInt();
        unsigned int number_bits_per_flit = getParameter("NumberBitsPerFlit").toUInt();
        unsigned int number_input_port_outputs = getGenProperties()->get("InputPort->NumberOutputs").toUInt();

        unsigned int number_crossbar_inputs = number_input_port_outputs * number_input_ports;
        unsigned int number_crossbar_outputs = number_output_ports;
        getGenProperties()->set("Crossbar->NumberInputs", number_crossbar_inputs);
        getGenProperties()->set("Crossbar->NumberOutputs", number_crossbar_outputs);

        // Init crossbar model
        const String& crossbar_name = "Crossbar";
        ElectricalModel* crossbar = ModelGen::createCrossbar(crossbar_model, crossbar_name, getTechModel());
        crossbar->setParameter("NumberInputs", number_crossbar_inputs);
        crossbar->setParameter("NumberOutputs", number_crossbar_outputs);
        crossbar->setParameter("NumberBits", number_bits_per_flit);
        crossbar->setParameter("BitDuplicate", "TRUE");
        crossbar->construct();

        unsigned int number_crossbar_selects = crossbar->getGenProperties()->get("NumberSelectsPerPort");
        getGenProperties()->set("Crossbar->NumberSelects", number_crossbar_selects);

        // Init DFF for crossbar selections
        const String& crossbar_sel_dff_name = "Crossbar_Sel_DFF";
        StdCell* crossbar_sel_dff = getTechModel()->getStdCellLib()->createStdCell("DFFQ", crossbar_sel_dff_name);
        crossbar_sel_dff->construct();

        // Add instances and results
        addSubInstances(crossbar, 1.0);
        addElectricalSubResults(crossbar, 1.0);

        addSubInstances(crossbar_sel_dff, number_crossbar_outputs * number_crossbar_selects);
        addElectricalSubResults(crossbar_sel_dff, number_crossbar_outputs * number_crossbar_selects);

        // Create connections
        createNet("Crossbar_Sel_DFF_Out");
        for(unsigned int i = 0; i < number_crossbar_outputs; ++i)
        {
            for(unsigned int j = 0; j < number_crossbar_selects; ++j)
            {
                createNet(String::format("Crossbar_Sel%d_%d", i, j));
            }
            createNet("Crossbar_Out" + (String)i, makeNetIndex(0, number_bits_per_flit-1));
        }
        for(unsigned int i = 0; i < number_crossbar_inputs; ++i)
        {
            createNet("Crossbar_In" + (String)i, makeNetIndex(0, number_bits_per_flit-1));
        }

        for(unsigned int i = 0; i < number_crossbar_selects; ++i)
        {
            portConnect(crossbar_sel_dff, "CK", "CK");
        }
        portConnect(crossbar_sel_dff, "Q", "Crossbar_Sel_DFF_Out");
        for(unsigned int i = 0; i < number_crossbar_inputs; ++i)
        {
            assignVirtualFanout("Crossbar_In" + (String)i, "PipelineReg1_Out");
            portConnect(crossbar, "In" + (String)i, "Crossbar_In" + (String)i);
        }
        for(unsigned int i = 0; i < number_crossbar_outputs; ++i)
        {
            for(unsigned int j = 0; j < number_crossbar_selects; ++j)
            {
                assignVirtualFanout(String::format("Crossbar_Sel%d_%d", i, j), "Crossbar_Sel_DFF_Out");
                portConnect(crossbar, String::format("Sel%d_%d", i, j), String::format("Crossbar_Sel%d_%d", i, j));
            }
            portConnect(crossbar, "Out" + (String)i, "Crossbar_Out" + (String)i);
            assignVirtualFanin("PipelineReg2_In" + (String)i, "Crossbar_Out" + (String)i);
        }

        return;
    }

    void Router::createPipelineReg()
    {
        // Get parameters
        unsigned int number_input_ports = getParameter("NumberInputPorts").toUInt();
        unsigned int number_output_ports = getParameter("NumberOutputPorts").toUInt();
        unsigned int number_bits_per_flit = getParameter("NumberBitsPerFlit").toUInt();
        unsigned int number_crossbar_inputs = getGenProperties()->get("Crossbar->NumberInputs");

        // Init pipeline reg model
        // First stage: from router input to input port
        const String& pipeline_reg0_name = "PipelineReg0";
        StdCell* pipeline_reg0 = getTechModel()->getStdCellLib()->createStdCell("DFFQ", pipeline_reg0_name);
        pipeline_reg0->construct();
        // Second stage: from input port to crossbar
        const String& pipeline_reg1_name = "PipelineReg1";
        StdCell* pipeline_reg1 = getTechModel()->getStdCellLib()->createStdCell("DFFQ", pipeline_reg1_name);
        pipeline_reg1->construct();

        // Third stage: from crossbar to router output
        vector<StdCell*> pipeline_reg2s(number_output_ports, (StdCell*)NULL);
        vector<String> pipeline_reg2_names(number_output_ports, "");
        for(unsigned int i = 0; i < number_output_ports; ++i)
        {
            pipeline_reg2_names[i] = "PipelineReg2_" + (String)i;
            pipeline_reg2s[i] = getTechModel()->getStdCellLib()->createStdCell("DFFQ", pipeline_reg2_names[i]);
            pipeline_reg2s[i]->construct();
        }

        // Add instances and results
        addSubInstances(pipeline_reg0, number_input_ports * number_bits_per_flit);
        addElectricalSubResults(pipeline_reg0, number_input_ports * number_bits_per_flit);

        addSubInstances(pipeline_reg1, number_crossbar_inputs * number_bits_per_flit);
        addElectricalSubResults(pipeline_reg1, number_crossbar_inputs * number_bits_per_flit);

        for(unsigned int i = 0; i < number_output_ports; ++i)
        {
            addSubInstances(pipeline_reg2s[i], number_bits_per_flit);
            addElectricalSubResults(pipeline_reg2s[i], number_bits_per_flit);
        }

        // Create data connections
        for(unsigned int i = 0; i < number_input_ports; ++i)
        {
            assignVirtualFanin("PipelineReg0_In", "FlitIn" + (String)i);
        }
        portConnect(pipeline_reg0, "D", "PipelineReg0_In");
        portConnect(pipeline_reg0, "Q", "PipelineReg0_Out");
        portConnect(pipeline_reg1, "D", "PipelineReg1_In");
        portConnect(pipeline_reg1, "Q", "PipelineReg1_Out");
        for(unsigned int i = 0; i < number_output_ports; ++i)
        {
            portConnect(pipeline_reg2s[i], "D", "PipelineReg2_In" + (String)i);
            portConnect(pipeline_reg2s[i], "Q", "PipelineReg2_Out" + (String)i);
            assignVirtualFanout("FlitOut" + (String)i, "PipelineReg2_Out" + (String)i);
        }

        // Create CK connections
        for(unsigned int n = 0; n < number_bits_per_flit; ++n)
        {
            for(unsigned int i = 0; i < number_input_ports; ++i)
            {
                portConnect(pipeline_reg0, "CK", "CK");
            }
            for(unsigned int i = 0; i < number_crossbar_inputs; ++i)
            {
                portConnect(pipeline_reg1, "CK", "CK");
            }
            for(unsigned int i = 0; i < number_output_ports; ++i)
            {
                portConnect(pipeline_reg2s[i], "CK", "CK");
            }
        }
        return;
    }

    void Router::createClockTree()
    {
        // Get parameters
        const String& clock_tree_model = getParameter("ClockTreeModel");
        const String& clock_tree_number_levels = getParameter("ClockTree->NumberLevels");
        const String& clock_tree_wire_layer = getParameter("ClockTree->WireLayer");
        const String& clock_tree_wire_width_multiplier = getParameter("ClockTree->WireWidthMultiplier");
        const String& clock_tree_wire_spacing_multiplier = getParameter("ClockTree->WireSpacingMultiplier");

        // Init clock tree model
        const String& clock_tree_name = "ClockTree";
        ElectricalModel* clock_tree = (ElectricalModel*)ModelGen::createModel(clock_tree_model, clock_tree_name, getTechModel());
        clock_tree->setParameter("NumberLevels", clock_tree_number_levels);
        clock_tree->setParameter("NumberBits", 1);
        clock_tree->setParameter("WireLayer", clock_tree_wire_layer);
        clock_tree->setParameter("WireWidthMultiplier", clock_tree_wire_width_multiplier);
        clock_tree->setParameter("WireSpacingMultiplier", clock_tree_wire_spacing_multiplier);
        clock_tree->construct();

        // Add instances and results
        addSubInstances(clock_tree, 1.0);
        addElectricalSubResults(clock_tree, 1.0);

        return;
    }
} // namespace DSENT

