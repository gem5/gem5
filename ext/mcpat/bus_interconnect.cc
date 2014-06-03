/*****************************************************************************
 *                                McPAT
 *                      SOFTWARE LICENSE AGREEMENT
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
 * Author: Joel Hestness
 *
 ***************************************************************************/

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <string>

#include "basic_circuit.h"
#include "bus_interconnect.h"
#include "common.h"
#include "const.h"
#include "io.h"
#include "parameter.h"

BusInterconnect::BusInterconnect(XMLNode* _xml_data,
                                 InputParameter* interface_ip_)
    : McPATComponent(_xml_data), link_bus(NULL), interface_ip(*interface_ip_) {
    name = "Bus Interconnect";
    set_param_stats();
    local_result = init_interface(&interface_ip, name);
    scktRatio = g_tp.sckt_co_eff;

    interface_ip.throughput = bus_params.link_throughput / bus_params.clockRate;
    interface_ip.latency = bus_params.link_latency / bus_params.clockRate;

    link_len /= bus_params.total_nodes;
    if (bus_params.total_nodes > 1) {
        //All links are shared by neighbors
        link_len /= 2;
    }

    link_bus = new Interconnect(xml_data, "Link", Uncore_device,
                                bus_params.link_base_width,
                                bus_params.link_base_height,
                                bus_params.flit_size, link_len, &interface_ip,
                                bus_params.link_start_wiring_level,
                                bus_params.clockRate,
                                bus_params.pipelinable,
                                bus_params.route_over_perc);
    children.push_back(link_bus);
}

void BusInterconnect::computeEnergy() {
    // Initialize stats for TDP
    tdp_stats.reset();
    tdp_stats.readAc.access = bus_stats.duty_cycle;
    link_bus->int_params.active_ports = bus_params.min_ports - 1;
    link_bus->int_stats.duty_cycle =
        bus_params.M_traffic_pattern * bus_stats.duty_cycle;

    // Initialize stats for runtime energy and power
    rtp_stats.reset();
    rtp_stats.readAc.access = bus_stats.total_access;
    link_bus->int_stats.accesses = bus_stats.total_access;

    // Recursively compute energy
    McPATComponent::computeEnergy();
}

void BusInterconnect::set_param_stats() {
    memset(&bus_params, 0, sizeof(BusInterconnectParameters));

    int num_children = xml_data->nChildNode("param");
    int i;
    int mat_type;
    for (i = 0; i < num_children; i++) {
        XMLNode* paramNode = xml_data->getChildNodePtr("param", &i);
        XMLCSTR node_name = paramNode->getAttribute("name");
        XMLCSTR value = paramNode->getAttribute("value");

        if (!node_name)
            warnMissingParamName(paramNode->getAttribute("id"));

        ASSIGN_FP_IF("clockrate", bus_params.clockRate);
        ASSIGN_INT_IF("flit_bits", bus_params.flit_size);
        ASSIGN_FP_IF("link_throughput", bus_params.link_throughput);
        ASSIGN_FP_IF("link_latency", bus_params.link_latency);
        ASSIGN_INT_IF("total_nodes", bus_params.total_nodes);
        ASSIGN_INT_IF("input_ports", bus_params.input_ports);
        ASSIGN_INT_IF("output_ports", bus_params.output_ports);
        ASSIGN_INT_IF("global_linked_ports", bus_params.global_linked_ports);
        ASSIGN_FP_IF("chip_coverage", bus_params.chip_coverage);
        ASSIGN_INT_IF("pipelinable", bus_params.pipelinable);
        ASSIGN_FP_IF("link_routing_over_percentage",
                     bus_params.route_over_perc);
        ASSIGN_INT_IF("virtual_channel_per_port",
                      bus_params.virtual_channel_per_port);
        ASSIGN_FP_IF("M_traffic_pattern", bus_params.M_traffic_pattern);
        ASSIGN_FP_IF("link_len", link_len);
        ASSIGN_FP_IF("link_base_width", bus_params.link_base_width);
        ASSIGN_FP_IF("link_base_height", bus_params.link_base_height);
        ASSIGN_FP_IF("link_start_wiring_level",
                     bus_params.link_start_wiring_level);
        ASSIGN_INT_IF("wire_mat_type", mat_type);
        ASSIGN_ENUM_IF("wire_type", interface_ip.wt, Wire_type);

        else {
            warnUnrecognizedParam(node_name);
        }
    }

    // Change from MHz to Hz
    bus_params.clockRate *= 1e6;

    interface_ip.wire_is_mat_type = mat_type;
    interface_ip.wire_os_mat_type = mat_type;

    num_children = xml_data->nChildNode("stat");
    for (i = 0; i < num_children; i++) {
        XMLNode* statNode = xml_data->getChildNodePtr("stat", &i);
        XMLCSTR node_name = statNode->getAttribute("name");
        XMLCSTR value = statNode->getAttribute("value");

        if (!node_name)
            warnMissingStatName(statNode->getAttribute("id"));

        ASSIGN_FP_IF("duty_cycle", bus_stats.duty_cycle);
        ASSIGN_FP_IF("total_accesses", bus_stats.total_access);

        else {
            warnUnrecognizedStat(node_name);
        }
    }

    clockRate = bus_params.clockRate;
    bus_params.min_ports =
        min(bus_params.input_ports, bus_params.output_ports);

    assert(bus_params.chip_coverage <= 1);
    assert(bus_params.route_over_perc <= 1);
    assert(link_len > 0);
}

void
BusInterconnect::set_duty_cycle(double duty_cycle) {
    bus_stats.duty_cycle = duty_cycle;
}

void
BusInterconnect::set_number_of_accesses(double total_accesses) {
    bus_stats.total_access = total_accesses;
}

BusInterconnect::~BusInterconnect() {
    delete link_bus;
    link_bus = NULL;
}
