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

#include "model/network/PhotonicClos.h"

#include <cmath>

#include "model/ModelGen.h"
#include "model/timing_graph/ElectricalTimingTree.h"
#include "model/timing_graph/ElectricalNet.h"

namespace DSENT
{
    using std::sqrt;

    PhotonicClos::PhotonicClos(const String& instance_name_, const TechModel* tech_model_)
        : ElectricalModel(instance_name_, tech_model_)
    {
        initParameters();
        initProperties();
    }

    PhotonicClos::~PhotonicClos()
    {}

    void PhotonicClos::initParameters()
    {
        // Clock Frequency
        addParameterName("Frequency");
        // Physical Parameters
        addParameterName("NumberInputSites");
        addParameterName("NumberOutputSites");
        addParameterName("NumberBitsPerFlit");
        // Number of each type of routers
        addParameterName("NumberIngressRouters");
        addParameterName("NumberMiddleRouters");
        addParameterName("NumberEgressRouters");
        // Optical link parameters
        addParameterName("SWSR->LinkDataRate");
        addParameterName("SWSR->LaserType");
        addParameterName("SWSR->RingTuningMethod");
        // Router parameters
        addParameterName("Router->NumberVirtualNetworks");
        addParameterName("Router->NumberVirtualChannelsPerVirtualNetwork");
        addParameterName("Router->NumberBuffersPerVirtualChannel");
        addParameterName("Router->InputPort->BufferModel");
        addParameterName("Router->CrossbarModel");
        addParameterName("Router->SwitchAllocator->ArbiterModel");
        addParameterName("Router->ClockTreeModel");
        addParameterName("Router->ClockTree->NumberLevels");
        addParameterName("Router->ClockTree->WireLayer");
        addParameterName("Router->ClockTree->WireWidthMultiplier");
        addParameterName("Router->ClockTree->WireSpacingMultiplier", 3.0);
        // Link parameters
        addParameterName("Link->WireLayer");
        addParameterName("Link->WireWidthMultiplier");
        addParameterName("Link->WireSpacingMultiplier");
        return;
    }

    void PhotonicClos::initProperties()
    {
        addPropertyName("InputSitePitch");
        addPropertyName("OutputSitePitch");
		addPropertyName("SWSR->OptUtil", 1.0);
        return;
    }

    PhotonicClos* PhotonicClos::clone() const
    {
        // TODO
        return NULL;
    }

    void PhotonicClos::constructModel()
    {
        // Get input parameters
        double clock_freq = getParameter("Frequency");
        unsigned int number_input_sites = getParameter("NumberInputSites").toUInt();
        unsigned int number_output_sites = getParameter("NumberOutputSites").toUInt();
        unsigned int number_bits_per_flit = getParameter("NumberBitsPerFlit").toUInt();
        unsigned int number_ingress_routers = getParameter("NumberIngressRouters").toUInt();
        unsigned int number_middle_routers = getParameter("NumberMiddleRouters").toUInt();
        unsigned int number_egress_routers = getParameter("NumberEgressRouters").toUInt();

        ASSERT(clock_freq > 0, "[Error] " + getInstanceName() +
                " -> Clock frequency must be > 0!");
        ASSERT(number_input_sites > 0, "[Error] " + getInstanceName() + 
                " -> Number of input sites must be > 0!");
        ASSERT(number_output_sites > 0, "[Error] " + getInstanceName() + 
                " -> Number of output sites must be > 0!");
        ASSERT(number_bits_per_flit > 0, "[Error] " + getInstanceName() + 
                " -> Number of bits per flit must be > 0!");
        ASSERT(number_ingress_routers > 0, "[Error] " + getInstanceName() +
                " -> Number of ingress routers must be > 0!");
        ASSERT(number_middle_routers > 0, "[Error] " + getInstanceName() + 
                " -> Number of middle routers must be > 0!");
        ASSERT(number_egress_routers > 0, "[Error] " + getInstanceName() +
                " -> Number of egress routers must be > 0!");

        // Get input parameters that will be forwarded to the sub instances
        const String& swsr_link_data_rate = getParameter("SWSR->LinkDataRate");
        const String& swsr_laser_type = getParameter("SWSR->LaserType");
        const String& swsr_ring_tuning_method = getParameter("SWSR->RingTuningMethod");
        const String& router_number_vns = getParameter("Router->NumberVirtualNetworks");
        const String& router_number_vcs_per_vn = getParameter("Router->NumberVirtualChannelsPerVirtualNetwork");
        const String& router_number_bufs_per_vc = getParameter("Router->NumberBuffersPerVirtualChannel");
        const String& router_buffer_model = getParameter("Router->InputPort->BufferModel");
        const String& router_crossbar_model = getParameter("Router->CrossbarModel");
        const String& link_wire_layer = getParameter("Link->WireLayer");
        const String& link_wire_width_multiplier = getParameter("Link->WireWidthMultiplier");
        const String& link_wire_spacing_multiplier = getParameter("Link->WireSpacingMultiplier");

        // Calculate properties from input parameters
        unsigned int ingress_router_number_input_ports = number_input_sites / number_ingress_routers;
        unsigned int ingress_router_number_output_ports = number_middle_routers;
        unsigned int middle_router_number_input_ports = number_ingress_routers;
        unsigned int middle_router_number_output_ports = number_egress_routers;
        unsigned int egress_router_number_input_ports = number_middle_routers;
        unsigned int egress_router_number_output_ports = number_output_sites / number_egress_routers;
        unsigned int number_input_to_ingress_links = number_input_sites;
        unsigned int number_ingress_to_middle_links = number_ingress_routers * number_middle_routers;
        unsigned int number_middle_to_egress_links = number_middle_routers * number_egress_routers;
        unsigned int number_egress_to_output_links = number_output_sites;

        getGenProperties()->set("NumberInputSitesPerIngressRouter", ingress_router_number_input_ports);
        getGenProperties()->set("NumberOutputSitesPerEgressRouter", egress_router_number_output_ports);
        getGenProperties()->set("IngressRouter->NumberInputPorts", ingress_router_number_input_ports);
        getGenProperties()->set("IngressRouter->NumberOutputPorts", ingress_router_number_output_ports);
        getGenProperties()->set("MiddleRouter->NumberInputPorts", middle_router_number_input_ports);
        getGenProperties()->set("MiddleRouter->NumberOutputPorts", middle_router_number_output_ports);
        getGenProperties()->set("EgressRouter->NumberInputPorts", egress_router_number_input_ports);
        getGenProperties()->set("EgressRouter->NumberOutputPorts", egress_router_number_output_ports);

        // Create ports
        createInputPort("CK");

        // Init ingress router
        ElectricalModel* ingress_router = (ElectricalModel*)ModelGen::createModel("Router", "IngressRouter", getTechModel());
        ingress_router->setParameter("NumberInputPorts", ingress_router_number_input_ports);
        ingress_router->setParameter("NumberOutputPorts", ingress_router_number_output_ports);
        ingress_router->setParameter("NumberVirtualNetworks", router_number_vns);
        ingress_router->setParameter("NumberVirtualChannelsPerVirtualNetwork", router_number_vcs_per_vn);
        ingress_router->setParameter("NumberBuffersPerVirtualChannel", router_number_bufs_per_vc);
        ingress_router->setParameter("NumberBitsPerFlit", number_bits_per_flit);
        ingress_router->setParameter("InputPort->BufferModel", router_buffer_model);
        ingress_router->setParameter("CrossbarModel", router_crossbar_model);
        ingress_router->setParameter("SwitchAllocator->ArbiterModel", getParameter("Router->SwitchAllocator->ArbiterModel"));
        ingress_router->setParameter("ClockTreeModel", getParameter("Router->ClockTreeModel"));
        ingress_router->setParameter("ClockTree->NumberLevels", getParameter("Router->ClockTree->NumberLevels"));
        ingress_router->setParameter("ClockTree->WireLayer", getParameter("Router->ClockTree->WireLayer"));
        ingress_router->setParameter("ClockTree->WireWidthMultiplier", getParameter("Router->ClockTree->WireWidthMultiplier"));
        ingress_router->setParameter("ClockTree->WireSpacingMultiplier", getParameter("Router->ClockTree->WireSpacingMultiplier"));
        ingress_router->construct();
        // Init middle routers
        ElectricalModel* middle_router = (ElectricalModel*)ModelGen::createModel("Router", "MiddleRouter", getTechModel());
        middle_router->setParameter("NumberInputPorts", middle_router_number_input_ports);
        middle_router->setParameter("NumberOutputPorts", middle_router_number_output_ports);
        middle_router->setParameter("NumberVirtualNetworks", router_number_vns);
        middle_router->setParameter("NumberVirtualChannelsPerVirtualNetwork", router_number_vcs_per_vn);
        middle_router->setParameter("NumberBuffersPerVirtualChannel", router_number_bufs_per_vc);
        middle_router->setParameter("NumberBitsPerFlit", number_bits_per_flit);
        middle_router->setParameter("InputPort->BufferModel", router_buffer_model);
        middle_router->setParameter("CrossbarModel", router_crossbar_model);
        middle_router->setParameter("SwitchAllocator->ArbiterModel", getParameter("Router->SwitchAllocator->ArbiterModel"));
        middle_router->setParameter("ClockTreeModel", getParameter("Router->ClockTreeModel"));
        middle_router->setParameter("ClockTree->NumberLevels", getParameter("Router->ClockTree->NumberLevels"));
        middle_router->setParameter("ClockTree->WireLayer", getParameter("Router->ClockTree->WireLayer"));
        middle_router->setParameter("ClockTree->WireWidthMultiplier", getParameter("Router->ClockTree->WireWidthMultiplier"));
        middle_router->setParameter("ClockTree->WireSpacingMultiplier", getParameter("Router->ClockTree->WireSpacingMultiplier"));
        middle_router->construct();
        // Init egress routers
        ElectricalModel* egress_router = (ElectricalModel*)ModelGen::createModel("Router", "EgressRouter", getTechModel());
        egress_router->setParameter("NumberInputPorts", egress_router_number_input_ports);
        egress_router->setParameter("NumberOutputPorts", egress_router_number_output_ports);
        egress_router->setParameter("NumberVirtualNetworks", router_number_vns);
        egress_router->setParameter("NumberVirtualChannelsPerVirtualNetwork", router_number_vcs_per_vn);
        egress_router->setParameter("NumberBuffersPerVirtualChannel", router_number_bufs_per_vc);
        egress_router->setParameter("NumberBitsPerFlit", number_bits_per_flit);
        egress_router->setParameter("InputPort->BufferModel", router_buffer_model);
        egress_router->setParameter("CrossbarModel", router_crossbar_model);
        egress_router->setParameter("SwitchAllocator->ArbiterModel", getParameter("Router->SwitchAllocator->ArbiterModel"));
        egress_router->setParameter("ClockTreeModel", getParameter("Router->ClockTreeModel"));
        egress_router->setParameter("ClockTree->NumberLevels", getParameter("Router->ClockTree->NumberLevels"));
        egress_router->setParameter("ClockTree->WireLayer", getParameter("Router->ClockTree->WireLayer"));
        egress_router->setParameter("ClockTree->WireWidthMultiplier", getParameter("Router->ClockTree->WireWidthMultiplier"));
        egress_router->setParameter("ClockTree->WireSpacingMultiplier", getParameter("Router->ClockTree->WireSpacingMultiplier"));
        egress_router->construct();
        // Init input to ingress link
        ElectricalModel* input_to_ingress_link = (ElectricalModel*)ModelGen::createModel("RepeatedLink", "InputToIngressLink", getTechModel());
        input_to_ingress_link->setParameter("NumberBits", number_bits_per_flit);
        input_to_ingress_link->setParameter("WireLayer", link_wire_layer);
        input_to_ingress_link->setParameter("WireWidthMultiplier", link_wire_width_multiplier);
        input_to_ingress_link->setParameter("WireSpacingMultiplier", link_wire_spacing_multiplier);
        input_to_ingress_link->construct();
        // Init ingress to middle link
        ElectricalModel* ingress_to_middle_link = (ElectricalModel*)ModelGen::createModel("SWSRLink", "IngressToMiddleLink", getTechModel());
        ingress_to_middle_link->setParameter("NumberBits", number_bits_per_flit);
        ingress_to_middle_link->setParameter("CoreDataRate", clock_freq);
        ingress_to_middle_link->setParameter("LinkDataRate", swsr_link_data_rate);
        ingress_to_middle_link->setParameter("LaserType", swsr_laser_type);
        ingress_to_middle_link->setParameter("RingTuningMethod", swsr_ring_tuning_method);
        ingress_to_middle_link->construct();
        // Init middle to egress link
        ElectricalModel* middle_to_egress_link = (ElectricalModel*)ModelGen::createModel("SWSRLink", "MiddleToEgressLink", getTechModel());
        middle_to_egress_link->setParameter("NumberBits", number_bits_per_flit);
        middle_to_egress_link->setParameter("CoreDataRate", clock_freq);
        middle_to_egress_link->setParameter("LinkDataRate", swsr_link_data_rate);
        middle_to_egress_link->setParameter("LaserType", swsr_laser_type);
        middle_to_egress_link->setParameter("RingTuningMethod", swsr_ring_tuning_method);
        middle_to_egress_link->construct();
        // Init egress to output link
        ElectricalModel* egress_to_output_link = (ElectricalModel*)ModelGen::createModel("RepeatedLink", "EgressToOutputLink", getTechModel());
        egress_to_output_link->setParameter("NumberBits", number_bits_per_flit);
        egress_to_output_link->setParameter("WireLayer", link_wire_layer);
        egress_to_output_link->setParameter("WireWidthMultiplier", link_wire_width_multiplier);
        egress_to_output_link->setParameter("WireSpacingMultiplier", link_wire_spacing_multiplier);
        egress_to_output_link->construct();

        // Connect ports
        createNet("InputToIngressLink_Out", makeNetIndex(0, number_bits_per_flit - 1));
        createNet("InputToIngressLink_In", makeNetIndex(0, number_bits_per_flit - 1));
        portConnect(input_to_ingress_link, "In", "InputToIngressLink_In");
        portConnect(input_to_ingress_link, "Out", "InputToIngressLink_Out");

        createNet("IngressToMiddleLink_In", makeNetIndex(0, number_bits_per_flit - 1));
        createNet("IngressToMiddleLink_Out", makeNetIndex(0, number_bits_per_flit - 1));
        portConnect(ingress_to_middle_link, "In", "IngressToMiddleLink_In");
        portConnect(ingress_to_middle_link, "Out", "IngressToMiddleLink_Out");

        createNet("MiddleToEgressLink_In", makeNetIndex(0, number_bits_per_flit - 1));
        createNet("MiddleToEgressLink_Out", makeNetIndex(0, number_bits_per_flit - 1));
        portConnect(middle_to_egress_link, "In", "MiddleToEgressLink_In");
        portConnect(middle_to_egress_link, "Out", "MiddleToEgressLink_Out");

        createNet("EgressToOutputLink_In", makeNetIndex(0, number_bits_per_flit - 1));
        createNet("EgressToOutputLink_Out", makeNetIndex(0, number_bits_per_flit - 1));
        portConnect(egress_to_output_link, "In", "EgressToOutputLink_In");
        portConnect(egress_to_output_link, "Out", "EgressToOutputLink_Out");

        portConnect(ingress_router, "CK", "CK");
        for(unsigned int i = 0; i < ingress_router_number_input_ports; ++i)
        {
            createNet("IngressRouter_In" + (String)i, makeNetIndex(0, number_bits_per_flit-1));
            for (unsigned int j = 0; j < number_bits_per_flit; ++j)
                assignVirtualFanout("IngressRouter_In" + (String)i, makeNetIndex(j), "InputToIngressLink_Out", makeNetIndex(j));
            portConnect(ingress_router, "FlitIn" + (String)i, "IngressRouter_In" + (String)i);
        }
        for(unsigned int i = 0; i < ingress_router_number_output_ports; ++i)
        {
            // VFI
            portConnect(ingress_router, "FlitOut" + (String)i, "IngressToMiddleLink_In");
        }
        portConnect(middle_router, "CK", "CK");
        for(unsigned int i = 0; i < middle_router_number_input_ports; ++i)
        {
            createNet("MiddleRouter_In" + (String)i, makeNetIndex(0, number_bits_per_flit-1));
            for (unsigned int j = 0; j < number_bits_per_flit; ++j)
                assignVirtualFanout("MiddleRouter_In" + (String)i, makeNetIndex(j), "IngressToMiddleLink_Out", makeNetIndex(j));
            portConnect(middle_router, "FlitIn" + (String)i, "MiddleRouter_In" + (String)i);
        }
        for(unsigned int i = 0; i < middle_router_number_output_ports; ++i)
        {
            // VFI
            portConnect(middle_router, "FlitOut" + (String)i, "MiddleToEgressLink_In");
        }        
        portConnect(egress_router, "CK", "CK");
        for(unsigned int i = 0; i < egress_router_number_input_ports; ++i)
        {
            createNet("EgressRouter_In" + (String)i, makeNetIndex(0, number_bits_per_flit-1));
            for (unsigned int j = 0; j < number_bits_per_flit; ++j)
                assignVirtualFanout("EgressRouter_In" + (String)i, makeNetIndex(j), "MiddleToEgressLink_Out", makeNetIndex(j));
            portConnect(egress_router, "FlitIn" + (String)i, "EgressRouter_In" + (String)i);
        }
        for(unsigned int i = 0; i < egress_router_number_output_ports; ++i)
        {
            // VFI
            portConnect(egress_router, "FlitOut" + (String)i, "EgressToOutputLink_In");
        }

        // Create area, power, and event results
        createElectricalResults();
        createElectricalEventResult("AvgUnicast");
        createElectricalEventResult("AvgBroadcast");
        addNddPowerResult(new Result("Laser"));
        addNddPowerResult(new Result("RingTuning"));
        addAreaResult(new Result("Photonic"));

        // Add all instances
        addSubInstances(ingress_router, number_ingress_routers);
        addElectricalSubResults(ingress_router, number_ingress_routers);
        addSubInstances(middle_router, number_middle_routers);
        addElectricalSubResults(middle_router, number_middle_routers);
        addSubInstances(egress_router, number_egress_routers);
        addElectricalSubResults(egress_router, number_egress_routers);
        addSubInstances(input_to_ingress_link, number_input_to_ingress_links);
        addElectricalSubResults(input_to_ingress_link, number_input_to_ingress_links);
        addSubInstances(ingress_to_middle_link, number_ingress_to_middle_links);
        addElectricalSubResults(ingress_to_middle_link, number_ingress_to_middle_links);
        getAreaResult("Photonic")->addSubResult(ingress_to_middle_link->getAreaResult("Photonic"), "IngressToMiddleLink", number_ingress_to_middle_links);
        getNddPowerResult("Laser")->addSubResult(ingress_to_middle_link->getNddPowerResult("Laser"), "IngressToMiddleLink", number_ingress_to_middle_links);
        getNddPowerResult("RingTuning")->addSubResult(ingress_to_middle_link->getNddPowerResult("RingTuning"), "IngressToMiddleLink", number_ingress_to_middle_links);
        addSubInstances(middle_to_egress_link, number_middle_to_egress_links);
        addElectricalSubResults(middle_to_egress_link, number_middle_to_egress_links);
        getAreaResult("Photonic")->addSubResult(middle_to_egress_link->getAreaResult("Photonic"), "MiddletoEgressLink", number_middle_to_egress_links);
        getNddPowerResult("Laser")->addSubResult(middle_to_egress_link->getNddPowerResult("Laser"), "MiddleToEgressLink", number_middle_to_egress_links);        
        getNddPowerResult("RingTuning")->addSubResult(middle_to_egress_link->getNddPowerResult("RingTuning"), "MiddleToEgressLink", number_middle_to_egress_links);        
        addSubInstances(egress_to_output_link, number_egress_to_output_links);
        addElectricalSubResults(egress_to_output_link, number_egress_to_output_links);

        // Update unicast event
        Result* avg_unicast_event = getEventResult("AvgUnicast");
        avg_unicast_event->addSubResult(input_to_ingress_link->getEventResult("Send"), "InputToIngressLink", 1.0);
        if(ingress_router->hasEventResult("WriteBuffer"))
        {
            avg_unicast_event->addSubResult(ingress_router->getEventResult("WriteBuffer"), "IngressRouter", 1.0);
        }
        if(ingress_router->hasEventResult("ReadBuffer"))
        {
            avg_unicast_event->addSubResult(ingress_router->getEventResult("ReadBuffer"), "IngressRouter", 1.0);
        }
        avg_unicast_event->addSubResult(ingress_router->getEventResult("TraverseCrossbar->Multicast1"), "IngressRouter", 1.0);
        avg_unicast_event->addSubResult(ingress_to_middle_link->getEventResult("Send"), "IngressToMiddleLink", 1.0);
        if(middle_router->hasEventResult("WriteBuffer"))
        {
            avg_unicast_event->addSubResult(middle_router->getEventResult("WriteBuffer"), "MiddleRouter", 1.0);
        }
        if(middle_router->hasEventResult("ReadBuffer"))
        {
            avg_unicast_event->addSubResult(middle_router->getEventResult("ReadBuffer"), "MiddleRouter", 1.0);
        }
        avg_unicast_event->addSubResult(middle_router->getEventResult("TraverseCrossbar->Multicast1"), "MiddleRouter", 1.0);
        avg_unicast_event->addSubResult(middle_to_egress_link->getEventResult("Send"), "MiddleToEgressLink", 1.0);
        if(egress_router->hasEventResult("WriteBuffer"))
        {
            avg_unicast_event->addSubResult(egress_router->getEventResult("WriteBuffer"), "EgressRouter", 1.0);
        }
        if(egress_router->hasEventResult("ReadBuffer"))
        {
            avg_unicast_event->addSubResult(egress_router->getEventResult("ReadBuffer"), "EgressRouter", 1.0);
        }
        avg_unicast_event->addSubResult(egress_router->getEventResult("TraverseCrossbar->Multicast1"), "EgressRouter", 1.0);
        avg_unicast_event->addSubResult(egress_to_output_link->getEventResult("Send"), "EgressToOutputLink", 1.0);

        // Update broadcast event
        Result* avg_broadcast_event = getEventResult("AvgBroadcast");
        avg_broadcast_event->addSubResult(input_to_ingress_link->getEventResult("Send"), "InputToIngressLink", 1.0);
        if(ingress_router->hasEventResult("WriteBuffer"))
        {
            avg_broadcast_event->addSubResult(ingress_router->getEventResult("WriteBuffer"), "IngressRouter", 1.0);
        }
        if(ingress_router->hasEventResult("ReadBuffer"))
        {
            avg_broadcast_event->addSubResult(ingress_router->getEventResult("ReadBuffer"), "IngressRouter", 1.0);
        }
        avg_broadcast_event->addSubResult(ingress_router->getEventResult("TraverseCrossbar->Multicast1"), "IngressRouter", 1.0);
        avg_broadcast_event->addSubResult(ingress_to_middle_link->getEventResult("Send"), "IngressToMiddleLink", 1.0);
        if(middle_router->hasEventResult("WriteBuffer"))
        {
            avg_broadcast_event->addSubResult(middle_router->getEventResult("WriteBuffer"), "MiddleRouter", 1.0);
        }
        if(middle_router->hasEventResult("ReadBuffer"))
        {
            avg_broadcast_event->addSubResult(middle_router->getEventResult("ReadBuffer"), "MiddleRouter", 1.0);
        }
        avg_broadcast_event->addSubResult(middle_router->getEventResult("TraverseCrossbar->Multicast1"), "MiddleRouter", 1.0);
        avg_broadcast_event->addSubResult(middle_to_egress_link->getEventResult("Send"), "MiddleToEgressLink", number_egress_routers);
        if(egress_router->hasEventResult("WriteBuffer"))
        {
            avg_broadcast_event->addSubResult(egress_router->getEventResult("WriteBuffer"), "EgressRouter", number_egress_routers);
        }
        if(egress_router->hasEventResult("ReadBuffer"))
        {
            avg_broadcast_event->addSubResult(egress_router->getEventResult("ReadBuffer"), "EgressRouter", number_egress_routers);
        }
        avg_broadcast_event->addSubResult(egress_router->getEventResult("TraverseCrossbar->Multicast" + (String)number_egress_routers), "EgressRouter", 1.0);
        avg_broadcast_event->addSubResult(egress_to_output_link->getEventResult("Send"), "EgressToOutputLink", number_output_sites);
        return;
    }

    void PhotonicClos::updateModel()
    {
        // Assumes waveguide runs adjacent to ingress and egress routers
        // Assumes input sites belonging to each ingress router are centered around the ingress router
        // Assumes middle routers are distributed around the chip adjacent to the main waveguide
        // Assumes output sites belonging to each egress router are centered around the egress router

        // Get properties
        double input_site_pitch = getProperty("InputSitePitch").toDouble();
        double output_site_pitch = getProperty("OutputSitePitch").toDouble();
        double clock_freq = getParameter("Frequency");
		const double swsr_opt_util = getProperty("SWSR->OptUtil");

        ASSERT(input_site_pitch > 0, "[Error] " + getInstanceName() + 
                " -> Input site pitch must be > 0!");
        ASSERT(output_site_pitch > 0, "[Error] " + getInstanceName() + 
                " -> Output site pitch must be > 0!");

        unsigned int number_input_sites_per_ingress_router = getGenProperties()->get("NumberInputSitesPerIngressRouter");
        unsigned int number_ingress_routers = getParameter("NumberIngressRouters");
        unsigned int number_output_sites_per_egress_router = getGenProperties()->get("NumberOutputSitesPerEgressRouter");
        unsigned int number_egress_routers = getParameter("NumberEgressRouters");
        double delay = 1.0 / clock_freq;

        //Calculate the length of the waveguide
        double input_to_ingress_link_length = input_site_pitch * (sqrt(number_input_sites_per_ingress_router) - 1.0);
        double input_to_ingress_link_delay = delay * 0.8;
        double ingress_to_middle_link_length = input_site_pitch * (sqrt(number_input_sites_per_ingress_router) * number_ingress_routers);
        double middle_to_egress_link_length = output_site_pitch * (sqrt(number_output_sites_per_egress_router) * number_egress_routers);
        double egress_to_output_link_length = output_site_pitch * (sqrt(number_output_sites_per_egress_router) - 1.0);
        double egress_to_output_link_delay = delay * 0.8;
        double ingress_router_delay = delay;
        double middle_router_delay = delay;
        double egress_router_delay = delay;

        Model* input_to_ingress_link = getSubInstance("InputToIngressLink");
        input_to_ingress_link->setProperty("WireLength", input_to_ingress_link_length);
        input_to_ingress_link->setProperty("Delay", input_to_ingress_link_delay);
        input_to_ingress_link->setProperty("IsKeepParity", "TRUE");
        input_to_ingress_link->update();

        Model* ingress_to_middle_link = getSubInstance("IngressToMiddleLink");
        ingress_to_middle_link->setProperty("Length", ingress_to_middle_link_length);
		ingress_to_middle_link->setProperty("OptUtil", swsr_opt_util);
        ingress_to_middle_link->update();

        Model* middle_to_egress_link = getSubInstance("MiddleToEgressLink");
        middle_to_egress_link->setProperty("Length", middle_to_egress_link_length);
		middle_to_egress_link->setProperty("OptUtil", swsr_opt_util);
        middle_to_egress_link->update();

        Model* egress_to_output_link = getSubInstance("EgressToOutputLink");
        egress_to_output_link->setProperty("WireLength", egress_to_output_link_length);
        egress_to_output_link->setProperty("Delay", egress_to_output_link_delay);
        egress_to_output_link->setProperty("IsKeepParity", "TRUE");
        egress_to_output_link->update();

        ElectricalModel* ingress_router = (ElectricalModel*)getSubInstance("IngressRouter");
        ingress_router->update();
        ElectricalTimingTree ingress_router_timing_tree("IngressRouter", ingress_router);
        ingress_router_timing_tree.performTimingOpt(ingress_router->getNet("CK"), ingress_router_delay);

        ElectricalModel* middle_router = (ElectricalModel*)getSubInstance("MiddleRouter");
        middle_router->update();
        ElectricalTimingTree middle_router_timing_tree("MiddleRouter", middle_router);
        middle_router_timing_tree.performTimingOpt(middle_router->getNet("CK"), middle_router_delay);

        ElectricalModel* egress_router = (ElectricalModel*)getSubInstance("EgressRouter");
        egress_router->update();
        ElectricalTimingTree egress_router_timing_tree("EgressRouter", egress_router);
        egress_router_timing_tree.performTimingOpt(egress_router->getNet("CK"), egress_router_delay);

        return;
    }
    
    void PhotonicClos::propagateTransitionInfo()
    {
        // Get parameters
        double clock_freq = getParameter("Frequency");
        double swsr_link_data_rate = getParameter("SWSR->LinkDataRate");

        // Get properties
        unsigned int ingress_router_number_input_ports = getGenProperties()->get("IngressRouter->NumberInputPorts");
        unsigned int middle_router_number_input_ports = getGenProperties()->get("MiddleRouter->NumberInputPorts");
        unsigned int egress_router_number_input_ports = getGenProperties()->get("EgressRouter->NumberInputPorts");

        ElectricalModel* input_to_ingress_link = (ElectricalModel*)getSubInstance("InputToIngressLink");
        assignPortTransitionInfo(input_to_ingress_link, "In", TransitionInfo(0.25, 0.25, 0.25));
        input_to_ingress_link->use();

        ElectricalModel* ingress_to_middle_link = (ElectricalModel*)getSubInstance("IngressToMiddleLink");
        assignPortTransitionInfo(ingress_to_middle_link, "LinkCK", TransitionInfo(0.0, (double) clock_freq / (swsr_link_data_rate * 2.0), 0.0));
        assignPortTransitionInfo(ingress_to_middle_link, "In", TransitionInfo(0.25, 0.25, 0.25));
        ingress_to_middle_link->use();

        ElectricalModel* middle_to_egress_link = (ElectricalModel*)getSubInstance("MiddleToEgressLink");
        assignPortTransitionInfo(middle_to_egress_link, "LinkCK", TransitionInfo(0.0, (double) clock_freq / (swsr_link_data_rate * 2.0), 0.0));
        assignPortTransitionInfo(middle_to_egress_link, "In", TransitionInfo(0.25, 0.25, 0.25));
        middle_to_egress_link->use();

        ElectricalModel* egress_to_output_link = (ElectricalModel*)getSubInstance("EgressToOutputLink");
        assignPortTransitionInfo(egress_to_output_link, "In", TransitionInfo(0.25, 0.25, 0.25));
        egress_to_output_link->use();

        ElectricalModel* ingress_router = (ElectricalModel*)getSubInstance("IngressRouter");
        for(unsigned int i = 0; i < ingress_router_number_input_ports; ++i)
        {
            assignPortTransitionInfo(ingress_router, "FlitIn" + (String)i, TransitionInfo(0.25, 0.25, 0.25));
        }
        assignPortTransitionInfo(ingress_router, "CK", TransitionInfo(0.0, 1.0, 0.0));
        ingress_router->getGenProperties()->set("UseModelEvent", "");
        ingress_router->use();

        ElectricalModel* middle_router = (ElectricalModel*)getSubInstance("MiddleRouter");
        for(unsigned int i = 0; i < middle_router_number_input_ports; ++i)
        {
            assignPortTransitionInfo(middle_router, "FlitIn" + (String)i, TransitionInfo(0.25, 0.25, 0.25));
        }
        assignPortTransitionInfo(middle_router, "CK", TransitionInfo(0.0, 1.0, 0.0));
        middle_router->getGenProperties()->set("UseModelEvent", "");
        middle_router->use();

        ElectricalModel* egress_router = (ElectricalModel*)getSubInstance("EgressRouter");
        for(unsigned int i = 0; i < egress_router_number_input_ports; ++i)
        {
            assignPortTransitionInfo(egress_router, "FlitIn" + (String)i, TransitionInfo(0.25, 0.25, 0.25));
        }
        assignPortTransitionInfo(egress_router, "CK", TransitionInfo(0.0, 1.0, 0.0));
        egress_router->getGenProperties()->set("UseModelEvent", "");
        egress_router->use();

        return;
    }
} // namespace DSENT

