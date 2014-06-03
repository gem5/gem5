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

#ifndef BUS_INTERCONNECT_H_
#define BUS_INTERCONNECT_H_

#include "array.h"
#include "basic_components.h"
#include "interconnect.h"
#include "logic.h"
#include "parameter.h"

class BusInterconnectParameters {
public:
    double clockRate;
    int flit_size;
    int input_ports;
    int output_ports;
    int min_ports;
    int global_linked_ports;
    int virtual_channel_per_port;
    int input_buffer_entries_per_vc;
    int total_nodes;
    double link_throughput;
    double link_latency;
    double chip_coverage;
    bool pipelinable;
    double route_over_perc;
    bool has_global_link;
    bool type;
    double M_traffic_pattern;
    double link_base_width;
    double link_base_height;
    int link_start_wiring_level;
};

class BusInterconnectStatistics {
public:
    double duty_cycle;
    double total_access;
};

class BusInterconnect : public McPATComponent {
public:
    Interconnect* link_bus;

    int ithNoC;
    InputParameter interface_ip;
    double link_len;
    double scktRatio, chip_PR_overhead, macro_PR_overhead;
    BusInterconnectParameters bus_params;
    BusInterconnectStatistics bus_stats;
    uca_org_t local_result;
    statsDef stats_t;
    double M_traffic_pattern;

    BusInterconnect(XMLNode* _xml_data, InputParameter* interface_ip_);
    void set_param_stats();
    void set_duty_cycle(double duty_cycle);
    void set_number_of_accesses(double total_accesses);
    void computeEnergy();
    ~BusInterconnect();
};

#endif /* BUS_INTERCONNECT_H_ */
