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

#include "model/network/ElectricalMesh.h"

#include <cmath>

#include "model/PortInfo.h"
#include "model/EventInfo.h"
#include "model/TransitionInfo.h"
#include "model/ModelGen.h"
#include "model/std_cells/StdCellLib.h"
#include "model/timing_graph/ElectricalTimingTree.h"
#include "model/timing_graph/ElectricalNet.h"

namespace DSENT
{
    using std::sqrt;

    ElectricalMesh::ElectricalMesh(const String& instance_name_, const TechModel* tech_model_)
        : ElectricalModel(instance_name_, tech_model_)
    {
        initParameters();
        initProperties();
    }

    ElectricalMesh::~ElectricalMesh()
    {}

    void ElectricalMesh::initParameters()
    {
        // Clock Frequency
        addParameterName("Frequency");
        // Physical Parameters
        addParameterName("NumberSites");
        addParameterName("NumberBitsPerFlit");
        // Concentration factor
        addParameterName("NumberSitesPerRouter");
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

    void ElectricalMesh::initProperties()
    {
        addPropertyName("SitePitch");
        return;
    }

    ElectricalMesh* ElectricalMesh::clone() const
    {
        // TODO
        return NULL;
    }

    void ElectricalMesh::constructModel()
    {
        // Get input paramters
        unsigned int number_sites = getParameter("NumberSites").toUInt();
        unsigned int number_bits_per_flit = getParameter("NumberBitsPerFlit").toUInt();
        unsigned int number_sites_per_router = getParameter("NumberSitesPerRouter").toUInt();

        ASSERT(number_sites > 0, "[Error] " + getInstanceName() + 
                " -> Number of sites must be > 0!");
        ASSERT(number_bits_per_flit > 0, "[Error] " + getInstanceName() + 
                " -> Number of bits per flit must be > 0!");
        ASSERT(number_sites_per_router > 0, "[Error] " + getInstanceName() + 
                " -> Number of sites per router must be > 0!");

        // Get input parameters that will be forwarded to the sub instances
        const String& router_number_vns = getParameter("Router->NumberVirtualNetworks");
        const String& router_number_vcs_per_vn = getParameter("Router->NumberVirtualChannelsPerVirtualNetwork");
        const String& router_number_bufs_per_vc = getParameter("Router->NumberBuffersPerVirtualChannel");
        const String& link_wire_layer = getParameter("Link->WireLayer");
        const String& link_wire_width_multiplier = getParameter("Link->WireWidthMultiplier");
        const String& link_wire_spacing_multiplier = getParameter("Link->WireSpacingMultiplier");

        // Calculate properties from input parameters
        unsigned int number_routers = number_sites / number_sites_per_router;
        unsigned int number_router_to_router_links = 4 * number_routers;
        unsigned int number_router_to_site_links = 2 * number_sites;
        unsigned int router_number_input_ports = 4 + number_sites_per_router;
        unsigned int router_number_output_ports = 4 + number_sites_per_router;

        getGenProperties()->set("NumberRouters", number_routers);
        getGenProperties()->set("NumberRouterToRouterLinks", number_router_to_router_links);
        getGenProperties()->set("NumberRouterToSiteLinks", number_router_to_site_links);
        getGenProperties()->set("Router->NumberInputPorts", router_number_input_ports);
        getGenProperties()->set("Router->NumberOutputPorts", router_number_output_ports);

        // Create ports
        createInputPort("CK");

        // Init mesh routers
        ElectricalModel* router = (ElectricalModel*)ModelGen::createModel("Router", "MeshRouter", getTechModel());
        router->setParameter("NumberInputPorts", router_number_input_ports);
        router->setParameter("NumberOutputPorts", router_number_output_ports);
        router->setParameter("NumberVirtualNetworks", router_number_vns);
        router->setParameter("NumberVirtualChannelsPerVirtualNetwork", router_number_vcs_per_vn);
        router->setParameter("NumberBuffersPerVirtualChannel", router_number_bufs_per_vc);
        router->setParameter("NumberBitsPerFlit", number_bits_per_flit);
        router->setParameter("InputPort->BufferModel", getParameter("Router->InputPort->BufferModel"));
        router->setParameter("CrossbarModel", getParameter("Router->CrossbarModel"));
        router->setParameter("SwitchAllocator->ArbiterModel", getParameter("Router->SwitchAllocator->ArbiterModel"));
        router->setParameter("ClockTreeModel", getParameter("Router->ClockTreeModel"));
        router->setParameter("ClockTree->NumberLevels", getParameter("Router->ClockTree->NumberLevels"));
        router->setParameter("ClockTree->WireLayer", getParameter("Router->ClockTree->WireLayer"));
        router->setParameter("ClockTree->WireWidthMultiplier", getParameter("Router->ClockTree->WireWidthMultiplier"));
        router->setParameter("ClockTree->WireSpacingMultiplier", getParameter("Router->ClockTree->WireSpacingMultiplier"));
        router->construct();

        // Init router to router links
        ElectricalModel* rr_link = (ElectricalModel*)ModelGen::createModel("RepeatedLink", "RouterToRouterLink", getTechModel());
        rr_link->setParameter("NumberBits", number_bits_per_flit);
        rr_link->setParameter("WireLayer", link_wire_layer);
        rr_link->setParameter("WireWidthMultiplier", link_wire_width_multiplier);
        rr_link->setParameter("WireSpacingMultiplier", link_wire_spacing_multiplier);
        rr_link->construct();

        // Init router to site links
        ElectricalModel* rs_link = (ElectricalModel*)ModelGen::createModel("RepeatedLink", "RouterToSiteLink", getTechModel());
        rs_link->setParameter("NumberBits", number_bits_per_flit);
        rs_link->setParameter("WireLayer", link_wire_layer);
        rs_link->setParameter("WireWidthMultiplier", link_wire_width_multiplier);
        rs_link->setParameter("WireSpacingMultiplier", link_wire_spacing_multiplier);
        rs_link->construct();

        // Connect ports
        createNet("RR_Link_Out", makeNetIndex(0, number_bits_per_flit-1));
        createNet("RR_Link_In", makeNetIndex(0, number_bits_per_flit-1));
        portConnect(rr_link, "In", "RR_Link_In");
        portConnect(rr_link, "Out", "RR_Link_Out");

        createNet("RS_Link_Out", makeNetIndex(0, number_bits_per_flit-1));
        createNet("RS_Link_In", makeNetIndex(0, number_bits_per_flit-1));
        portConnect(rs_link, "In", "RS_Link_In");
        portConnect(rs_link, "Out", "RS_Link_Out");

        portConnect(router, "CK", "CK");
        for(unsigned int i = 0; i < router_number_input_ports; ++i)
        {
            createNet("Router_In" + (String)i, makeNetIndex(0, number_bits_per_flit-1));
            portConnect(router, "FlitIn" + (String)i, "Router_In" + (String)i);
        }
        for(unsigned int i = 0; i < router_number_output_ports; ++i)
        {
            createNet("Router_Out" + (String)i, makeNetIndex(0, number_bits_per_flit-1));
            portConnect(router, "FlitOut" + (String)i, "Router_Out" + (String)i);
        }
        for(unsigned int i = 0; i < number_bits_per_flit; ++i)
        {
            for(unsigned int j = 0; j < 4; ++j)
            {
                assignVirtualFanout("Router_In" + (String)j, makeNetIndex(i), "RR_Link_Out", makeNetIndex(i));
                assignVirtualFanin("RR_Link_In", makeNetIndex(i), "Router_Out" + (String)j, makeNetIndex(i));
            }
            for(unsigned int j = 4; j < router_number_input_ports; ++j)
            {
                assignVirtualFanout("Router_In" + (String)j, makeNetIndex(i), "RS_Link_Out", makeNetIndex(i));
                assignVirtualFanin("RS_Link_In", makeNetIndex(i), "Router_Out" + (String)j, makeNetIndex(i));
            }
        }

        // Create area, power and event results
        createElectricalResults();
        createElectricalEventResult("AvgUnicast");
        createElectricalEventResult("AvgBroadcast");

        // Add all instances
        addSubInstances(router, number_routers);
        addElectricalSubResults(router, number_routers);
        addSubInstances(rr_link, number_router_to_router_links);
        addElectricalSubResults(rr_link, number_router_to_router_links);
        addSubInstances(rs_link, number_router_to_site_links);
        addElectricalSubResults(rs_link, number_router_to_site_links);

        double number_routers_per_side = sqrt(number_routers);

        // Update unicast event
        double avg_number_unicast_hop = 2.0 * number_routers_per_side / 3.0;
        double avg_number_unicast_rr_links_traveled = avg_number_unicast_hop;
        double avg_number_unicast_rs_links_traveled = 2.0;
        double avg_number_unicast_router_traveled = avg_number_unicast_hop + 1.0;
        Result* avg_unicast_flit = getEventResult("AvgUnicast");
        avg_unicast_flit->addSubResult(rr_link->getEventResult("Send"), "RouterToRouterLink", avg_number_unicast_rr_links_traveled);
        avg_unicast_flit->addSubResult(rs_link->getEventResult("Send"), "RouterToSiteLink", avg_number_unicast_rs_links_traveled);
        if(router->hasEventResult("WriteBuffer"))
        {
            avg_unicast_flit->addSubResult(router->getEventResult("WriteBuffer"), "MeshRouter", avg_number_unicast_router_traveled);
        }
        if(router->hasEventResult("ReadBuffer"))
        {
            avg_unicast_flit->addSubResult(router->getEventResult("ReadBuffer"), "MeshRouter", avg_number_unicast_router_traveled);
        }
        avg_unicast_flit->addSubResult(router->getEventResult("TraverseCrossbar->Multicast1"), "MeshRouter", avg_number_unicast_router_traveled);

        // Update broadcast event
        double avg_number_broadcast_rr_links_traveled = (number_routers_per_side - 1.0) * number_routers_per_side + number_routers_per_side - 1.0;
        double avg_number_broadcast_rs_links_traveled = number_sites;
        double avg_number_broadcast_router_crossbar_traveled = number_routers * (number_sites_per_router + 1.0) - 2.0;
        Result* avg_broadcast_flit = getEventResult("AvgBroadcast");
        avg_broadcast_flit->addSubResult(rr_link->getEventResult("Send"), "RouterToRouterLink", avg_number_broadcast_rr_links_traveled);
        avg_broadcast_flit->addSubResult(rs_link->getEventResult("Send"), "RouterToSiteLink", avg_number_broadcast_rs_links_traveled);
        if(router->hasEventResult("WriteBuffer"))
        {
            avg_broadcast_flit->addSubResult(router->getEventResult("WriteBuffer"), "MeshRouter", number_routers);
        }
        if(router->hasEventResult("ReadBuffer"))
        {
            avg_broadcast_flit->addSubResult(router->getEventResult("ReadBuffer"), "MeshRouter", number_routers);
        }
        avg_broadcast_flit->addSubResult(router->getEventResult("TraverseCrossbar->Multicast1"), "MeshRouter", avg_number_broadcast_router_crossbar_traveled);

        return;
    }

    void ElectricalMesh::updateModel()
    {
        // Get properties
        double site_pitch = getProperty("SitePitch").toDouble();
        double clock_freq = getParameter("Frequency");

        ASSERT(site_pitch > 0, "[Error] " + getInstanceName() + 
                " -> Site pitch must be > 0!");
        ASSERT(clock_freq > 0, "[Error] " + getInstanceName() +
                " -> Clock frequency must be > 0!");

        unsigned int number_sites_per_router = getParameter("NumberSitesPerRouter");
        // Get margin on link delays, since there are registers before and after the link
        double delay_ck_to_q = getTechModel()->getStdCellLib()->getStdCellCache()->get("DFFQ_X1->Delay->CK_to_Q");
        double delay_setup = getTechModel()->getStdCellLib()->getStdCellCache()->get("DFFQ_X1->Delay->CK_to_Q");
        double link_delay_margin = (delay_ck_to_q + delay_setup) * 1.5;
        
        double rr_link_length = site_pitch * sqrt(number_sites_per_router);
        double rr_link_delay = std::max(1e-99, 1.0 / clock_freq - link_delay_margin);
        double rs_link_length = site_pitch * (sqrt(number_sites_per_router) - 1.0);
        double rs_link_delay = std::max(1e-99, 1.0 / clock_freq - link_delay_margin );
        double router_delay = 1.0 / clock_freq;

        Model* rr_link = getSubInstance("RouterToRouterLink");
        rr_link->setProperty("WireLength", rr_link_length);
        rr_link->setProperty("Delay", rr_link_delay);
        rr_link->setProperty("IsKeepParity", "TRUE");
        rr_link->update();

        Model* rs_link = getSubInstance("RouterToSiteLink");
        rs_link->setProperty("WireLength", rs_link_length);
        rs_link->setProperty("Delay", rs_link_delay);
        rs_link->setProperty("IsKeepParity", "TRUE");
        rs_link->update();

        ElectricalModel* router = (ElectricalModel*)getSubInstance("MeshRouter");
        router->update();

        ElectricalTimingTree router_timing_tree("MeshRouter", router);
        router_timing_tree.performTimingOpt(router->getNet("CK"), router_delay);
        return;
    }

    void ElectricalMesh::propagateTransitionInfo()
    {
        // Get parameters
        unsigned int router_number_input_ports = getGenProperties()->get("Router->NumberInputPorts");

        ElectricalModel* rr_link = (ElectricalModel*)getSubInstance("RouterToRouterLink");
        assignPortTransitionInfo(rr_link, "In", TransitionInfo(0.25, 0.25, 0.25));
        rr_link->use();

        ElectricalModel* rs_link = (ElectricalModel*)getSubInstance("RouterToSiteLink");
        assignPortTransitionInfo(rs_link, "In", TransitionInfo(0.25, 0.25, 0.25));
        rs_link->use();

        ElectricalModel* router = (ElectricalModel*)getSubInstance("MeshRouter");
        for(unsigned int i = 0; i < router_number_input_ports; ++i)
        {
            assignPortTransitionInfo(router, "FlitIn" + (String)i, TransitionInfo(0.25, 0.25, 0.25));
        }
        assignPortTransitionInfo(router, "CK", TransitionInfo(0.0, 1.0, 0.0));
        router->getGenProperties()->set("UseModelEvent", "");
        router->use();

        return;
    }
} // namespace DSENT

