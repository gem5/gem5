/*****************************************************************************
 *                                McPAT
 *                      SOFTWARE LICENSE AGREEMENT
 *            Copyright 2012 Hewlett-Packard Development Company, L.P.
 *            Copyright (c) 2010-2013 Advanced Micro Devices, Inc.
 *                          All Rights Reserved
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
 ***************************************************************************/

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <string>

#include "basic_circuit.h"
#include "common.h"
#include "const.h"
#include "io.h"
#include "noc.h"
#include "parameter.h"

OnChipNetwork::OnChipNetwork(XMLNode* _xml_data, int ithNoC_,
                             InputParameter* interface_ip_)
    : McPATComponent(_xml_data), router(NULL), link_bus(NULL), ithNoC(ithNoC_),
      interface_ip(*interface_ip_), link_bus_exist(false),
      router_exist(false) {
    name = "On-Chip Network";
    set_param_stats();
    local_result = init_interface(&interface_ip, name);
    scktRatio = g_tp.sckt_co_eff;

    // TODO: Routers and links should be children of the NOC component
    if (noc_params.type) {
        init_router();
    } else {
        init_link_bus();
    }
}

void OnChipNetwork::init_router() {
    router = new Router(noc_params.flit_size,
                        noc_params.virtual_channel_per_port *
                        noc_params.input_buffer_entries_per_vc,
                        noc_params.virtual_channel_per_port,
                        &(g_tp.peri_global),
                        noc_params.input_ports, noc_params.output_ports,
                        noc_params.M_traffic_pattern);
    // TODO: Make a router class within McPAT that descends from McPATComponent
    // children.push_back(router);
    area.set_area(area.get_area() + router->area.get_area() *
                  noc_params.total_nodes);

    double long_channel_device_reduction = longer_channel_device_reduction(Uncore_device);
    router->power.readOp.longer_channel_leakage          = router->power.readOp.leakage * long_channel_device_reduction;
    router->buffer.power.readOp.longer_channel_leakage   = router->buffer.power.readOp.leakage * long_channel_device_reduction;
    router->crossbar.power.readOp.longer_channel_leakage = router->crossbar.power.readOp.leakage * long_channel_device_reduction;
    router->arbiter.power.readOp.longer_channel_leakage  = router->arbiter.power.readOp.leakage * long_channel_device_reduction;
    router_exist = true;
}

void OnChipNetwork::init_link_bus() {
    if (noc_params.type) {
        link_name = "Links";
    } else {
        link_name = "Bus";
    }

    interface_ip.throughput = noc_params.link_throughput /
        noc_params.clockRate;
    interface_ip.latency = noc_params.link_latency / noc_params.clockRate;

    link_len /= (noc_params.horizontal_nodes + noc_params.vertical_nodes) / 2;

    if (noc_params.total_nodes > 1) {
        //All links are shared by neighbors
        link_len /= 2;
    }
    link_bus = new Interconnect(xml_data, "Link", Uncore_device,
                                noc_params.link_base_width,
                                noc_params.link_base_height,
                                noc_params.flit_size, link_len, &interface_ip,
                                noc_params.link_start_wiring_level,
                                noc_params.clockRate, true/*pipelinable*/,
                                noc_params.route_over_perc);
    children.push_back(link_bus);

    link_bus_exist = true;
}

// TODO: This should use the McPATComponent::computeEnergy function to
// recursively calculate energy of routers and links and then add
void OnChipNetwork::computeEnergy() {
    double pppm_t[4]    = {1, 1, 1, 1};

    // Initialize stats for TDP
    tdp_stats.reset();
    tdp_stats.readAc.access = noc_stats.duty_cycle;
    if (router_exist) {
        // TODO: Define a regression to exercise routers
        // TODO: Clean this up: it is too invasive and breaks abstraction
        set_pppm(pppm_t, 1 * tdp_stats.readAc.access, 1, 1, 1);
        router->power = router->power * pppm_t;
        set_pppm(pppm_t, noc_params.total_nodes,
                noc_params.total_nodes,
                noc_params.total_nodes,
                noc_params.total_nodes);
    }
    if (link_bus_exist) {
        if (noc_params.type) {
            link_bus->int_params.active_ports = noc_params.min_ports - 1;
        } else {
            link_bus->int_params.active_ports = noc_params.min_ports;
        }
        link_bus->int_stats.duty_cycle =
            noc_params.M_traffic_pattern * noc_stats.duty_cycle;

        // TODO: Decide how to roll multiple routers into a single top-level
        // NOC module. I would prefer not to, but it might be a nice feature
        set_pppm(pppm_t, noc_params.total_nodes,
                 noc_params.total_nodes,
                 noc_params.total_nodes,
                 noc_params.total_nodes);
    }

    // Initialize stats for runtime energy and power
    rtp_stats.reset();
    rtp_stats.readAc.access = noc_stats.total_access;
    set_pppm(pppm_t, 1, 0 , 0, 0);
    if (router_exist) {
        // TODO: Move this to a McPATComponent parent class of Router
        router->buffer.rt_power.readOp.dynamic =
            (router->buffer.power.readOp.dynamic +
             router->buffer.power.writeOp.dynamic) * rtp_stats.readAc.access;
        router->crossbar.rt_power.readOp.dynamic =
            router->crossbar.power.readOp.dynamic * rtp_stats.readAc.access;
        router->arbiter.rt_power.readOp.dynamic =
            router->arbiter.power.readOp.dynamic * rtp_stats.readAc.access;

        router->rt_power = router->rt_power +
            (router->buffer.rt_power + router->crossbar.rt_power +
             router->arbiter.rt_power) * pppm_t +
            router->power * pppm_lkg;//TDP power must be calculated first!
    }
    if (link_bus_exist) {
        link_bus->int_stats.accesses = noc_stats.total_access;
    }

    // Recursively compute energy
    McPATComponent::computeEnergy();
}

void OnChipNetwork::set_param_stats() {
    // TODO: Remove this or move initialization elsewhere
    memset(&noc_params, 0, sizeof(OnChipNetworkParameters));

    int num_children = xml_data->nChildNode("param");
    int i;
    int mat_type;
    for (i = 0; i < num_children; i++) {
        XMLNode* paramNode = xml_data->getChildNodePtr("param", &i);
        XMLCSTR node_name = paramNode->getAttribute("name");
        XMLCSTR value = paramNode->getAttribute("value");

        if (!node_name)
            warnMissingParamName(paramNode->getAttribute("id"));

        ASSIGN_INT_IF("type", noc_params.type);
        ASSIGN_FP_IF("clockrate", noc_params.clockRate);
        ASSIGN_INT_IF("flit_bits", noc_params.flit_size);
        ASSIGN_FP_IF("link_len", link_len);
        ASSIGN_FP_IF("link_throughput", noc_params.link_throughput);
        ASSIGN_FP_IF("link_latency", noc_params.link_latency);
        ASSIGN_INT_IF("input_ports", noc_params.input_ports);
        ASSIGN_INT_IF("output_ports", noc_params.output_ports);
        ASSIGN_INT_IF("global_linked_ports", noc_params.global_linked_ports);
        ASSIGN_INT_IF("horizontal_nodes", noc_params.horizontal_nodes);
        ASSIGN_INT_IF("vertical_nodes", noc_params.vertical_nodes);
        ASSIGN_FP_IF("chip_coverage", noc_params.chip_coverage);
        ASSIGN_FP_IF("link_routing_over_percentage",
                     noc_params.route_over_perc);
        ASSIGN_INT_IF("has_global_link", noc_params.has_global_link);
        ASSIGN_INT_IF("virtual_channel_per_port",
                      noc_params.virtual_channel_per_port);
        ASSIGN_INT_IF("input_buffer_entries_per_vc",
                      noc_params.input_buffer_entries_per_vc);
        ASSIGN_FP_IF("M_traffic_pattern", noc_params.M_traffic_pattern);
        ASSIGN_FP_IF("link_base_width", noc_params.link_base_width);
        ASSIGN_FP_IF("link_base_height", noc_params.link_base_height);
        ASSIGN_INT_IF("link_start_wiring_level",
                      noc_params.link_start_wiring_level);
        ASSIGN_INT_IF("wire_mat_type", mat_type);
        ASSIGN_ENUM_IF("wire_type", interface_ip.wt, Wire_type);

        else {
            warnUnrecognizedParam(node_name);
        }
    }

    // Change from MHz to Hz
    noc_params.clockRate *= 1e6;

    interface_ip.wire_is_mat_type = mat_type;
    interface_ip.wire_os_mat_type = mat_type;

    num_children = xml_data->nChildNode("stat");
    for (i = 0; i < num_children; i++) {
        XMLNode* statNode = xml_data->getChildNodePtr("stat", &i);
        XMLCSTR node_name = statNode->getAttribute("name");
        XMLCSTR value = statNode->getAttribute("value");

        if (!node_name)
            warnMissingStatName(statNode->getAttribute("id"));

        ASSIGN_FP_IF("duty_cycle", noc_stats.duty_cycle);
        ASSIGN_FP_IF("total_accesses", noc_stats.total_access);

        else {
            warnUnrecognizedStat(node_name);
        }
    }

    clockRate = noc_params.clockRate;
    noc_params.min_ports =
        min(noc_params.input_ports, noc_params.output_ports);
    if (noc_params.type) {
        noc_params.global_linked_ports = (noc_params.input_ports - 1) +
            (noc_params.output_ports - 1);
    }
    noc_params.total_nodes =
        noc_params.horizontal_nodes * noc_params.vertical_nodes;

    assert(noc_params.chip_coverage <= 1);
    assert(noc_params.route_over_perc <= 1);
    assert(link_len > 0);
}

OnChipNetwork ::~OnChipNetwork() {

    if (router) {
        delete router;
        router = 0;
    }
    if (link_bus) {
        delete link_bus;
        link_bus = 0;
    }
}
