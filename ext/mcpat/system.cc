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
 * Authors: Joel Hestness
 *          Yasuko Eckert
 *
 ***************************************************************************/

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>

#include "array.h"
#include "basic_circuit.h"
#include "common.h"
#include "const.h"
#include "parameter.h"
#include "system.h"
#include "version.h"

// TODO: Fix this constructor to default initialize all pointers to NULL
System::System(XMLNode* _xml_data)
        : McPATComponent(_xml_data) {
    int i;
    int currCore = 0;
    int currNOC = 0;
    name = "System";
    set_proc_param();

    // TODO: This loop can (and should) be called by every component in
    // the hierarchy.  Consider moving it to McPATComponent
    int numChildren = xml_data->nChildNode("component");
    for (i = 0; i < numChildren; i++ ) {
        // For each child node of the system,
        XMLNode* childXML = xml_data->getChildNodePtr("component", &i);
        XMLCSTR type = childXML->getAttribute("type");

        if (!type) {
            warnMissingComponentType(childXML->getAttribute("id"));

        } STRCMP(type, "Core") {
            // TODO: If homogeneous cores, and currCore > 0, just copy core 0
            children.push_back(new Core(childXML, currCore, &interface_ip));
            currCore++;
        } STRCMP(type, "CacheUnit") {
            children.push_back(new CacheUnit(childXML, &interface_ip));
        } STRCMP(type, "CacheController") {
            // TODO: Remove reliance on interface_ip - there should be a better
            // way to share global variables than passing, copying
            children.push_back(new CacheController(childXML, &interface_ip));
        } STRCMP(type, "MemoryController") {
            children.push_back(new MemoryController(childXML, &interface_ip));
        } STRCMP(type, "FlashController") {
            children.push_back(new FlashController(childXML, &interface_ip));
        } STRCMP(type, "NIUController") {
            children.push_back(new NIUController(childXML, &interface_ip));
        } STRCMP(type, "PCIeController") {
            children.push_back(new PCIeController(childXML, &interface_ip));
        } STRCMP(type, "Memory") {
            // TODO:
            warnIncompleteComponentType(type);
        } STRCMP(type, "OnChipNetwork") {
            // TODO: Many of the parameters to this constructor should be
            // handled in another way
            children.push_back(new OnChipNetwork(childXML, currNOC,
                                                 &interface_ip));
            currNOC++;
            warnIncompleteComponentType(type);
        } STRCMP(type, "BusInterconnect") {
            // TODO: Many of the parameters to this constructor should be
            // handled in another way
            children.push_back(new BusInterconnect(childXML, &interface_ip));
            warnIncompleteComponentType(type);

        // TODO: Add a directory data type that can handle the directories
        // as defined by certain McScript output
        } else {
            warnUnrecognizedComponent(type);
        }
    }
}

void System::displayDeviceType(int device_type_, uint32_t indent) {
    string indent_str(indent, ' ');
    cout << indent_str << "Device Type = ";

    switch ( device_type_ ) {
    case 0:
        cout << "ITRS high performance device type" << endl;
        break;
    case 1:
        cout << "ITRS low standby power device type" << endl;
        break;
    case 2:
        cout << "ITRS low operating power device type" << endl;
        break;
    case 3:
        cout << "LP-DRAM device type" << endl;
        break;
    case 4:
        cout << "COMM-DRAM device type" << endl;
        break;
    default:
        cout << indent_str << "Unknown!" << endl;
        exit(0);
    }
}

void System::displayInterconnectType(int interconnect_type_, uint32_t indent) {
    string indent_str(indent, ' ');
    cout << indent_str << "Interconnect metal projection = ";

    switch ( interconnect_type_ ) {
    case 0:
        cout << "aggressive interconnect technology projection" << endl;
        break;
    case 1:
        cout << "conservative interconnect technology projection" << endl;
        break;
    default:
        cout << indent_str << "Unknown!" << endl;
        exit(0);
    }
}

// TODO: Migrate this down to the McPATComponent::displayData function
void System::displayData(uint32_t indent, int plevel) {
    string indent_str(indent, ' ');
    string indent_str_next(indent + 2, ' ');
    if (plevel < 5) {
        cout << "\nMcPAT (version " << VER_MAJOR << "." << VER_MINOR
             << " of " << VER_UPDATE << ") results (current print level is "
             << plevel
             << ", please increase print level to see the details in "
             << "components) " << endl;
    } else {
        cout << "\nMcPAT (version " << VER_MAJOR << "." << VER_MINOR
             << " of " << VER_UPDATE << ") results  (current print level is 5)"
             << endl;
    }

    cout << "*****************************************************************"
         << "************************" << endl;
    cout << indent_str << "Technology " << core_tech_node << " nm" << endl;
    if (longer_channel_device)
        cout << indent_str << "Using Long Channel Devices When Appropriate" << endl;
    displayInterconnectType(interconnect_projection_type, indent);
    cout << indent_str << "Target Clock Rate (MHz) " << target_core_clockrate / 1e6 << endl;
    cout << endl;

    cout << "*****************************************************************"
         << "************************" << endl;

    McPATComponent::displayData(indent, plevel);
}

void System::set_proc_param() {
    // TODO: Consider creating a SystemParams class that tracks system-wide
    // parameters like these
    longer_channel_device = false;
    core_tech_node = -1;
    temperature = -1;
    interconnect_projection_type = -1;
    device_type = -1;
    physical_address_width = -1;

    int num_children = xml_data->nChildNode("param");
    int i;
    for (i = 0; i < num_children; i++) {
        XMLNode* paramNode = xml_data->getChildNodePtr("param", &i);
        XMLCSTR node_name = paramNode->getAttribute("name");
        XMLCSTR value = paramNode->getAttribute("value");

        if (!node_name)
            warnMissingParamName(paramNode->getAttribute("id"));

        ASSIGN_FP_IF("core_tech_node", core_tech_node);
        ASSIGN_INT_IF("target_core_clockrate", target_core_clockrate);
        ASSIGN_INT_IF("temperature", temperature);
        ASSIGN_INT_IF("device_type", device_type);
        ASSIGN_INT_IF("longer_channel_device", longer_channel_device);
        ASSIGN_INT_IF("interconnect_projection_type",
                      interconnect_projection_type);
        ASSIGN_INT_IF("machine_bits", data_path_width);
        ASSIGN_INT_IF("virtual_address_width", virtual_address_width);
        ASSIGN_INT_IF("physical_address_width", physical_address_width);
        ASSIGN_INT_IF("virtual_memory_page_size", virtual_memory_page_size);
        ASSIGN_INT_IF("wire_is_mat_type", interface_ip.wire_is_mat_type);
        ASSIGN_INT_IF("wire_os_mat_type", interface_ip.wire_os_mat_type);
        ASSIGN_INT_IF("delay_wt", interface_ip.delay_wt);
        ASSIGN_INT_IF("area_wt", interface_ip.area_wt);
        ASSIGN_INT_IF("dynamic_power_wt", interface_ip.dynamic_power_wt);
        ASSIGN_INT_IF("leakage_power_wt", interface_ip.leakage_power_wt);
        ASSIGN_INT_IF("cycle_time_wt", interface_ip.cycle_time_wt);
        ASSIGN_INT_IF("delay_dev", interface_ip.delay_dev);
        ASSIGN_INT_IF("area_dev", interface_ip.area_dev);
        ASSIGN_INT_IF("dynamic_power_dev", interface_ip.dynamic_power_dev);
        ASSIGN_INT_IF("leakage_power_dev", interface_ip.leakage_power_dev);
        ASSIGN_INT_IF("cycle_time_dev", interface_ip.cycle_time_dev);
        ASSIGN_INT_IF("ed", interface_ip.ed);
        ASSIGN_INT_IF("burst_len", interface_ip.burst_len);
        ASSIGN_INT_IF("int_prefetch_w", interface_ip.int_prefetch_w);
        ASSIGN_INT_IF("page_sz_bits", interface_ip.page_sz_bits);
        ASSIGN_ENUM_IF("rpters_in_htree", interface_ip.rpters_in_htree, bool);
        ASSIGN_INT_IF("ver_htree_wires_over_array",
                      interface_ip.ver_htree_wires_over_array);
        ASSIGN_INT_IF("broadcast_addr_din_over_ver_htrees",
                      interface_ip.broadcast_addr_din_over_ver_htrees);
        ASSIGN_INT_IF("nuca", interface_ip.nuca);
        ASSIGN_INT_IF("nuca_bank_count", interface_ip.nuca_bank_count);
        ASSIGN_ENUM_IF("force_cache_config",
                       interface_ip.force_cache_config, bool);
        ASSIGN_ENUM_IF("wt", interface_ip.wt, Wire_type);
        ASSIGN_INT_IF("force_wiretype", interface_ip.force_wiretype);
        ASSIGN_INT_IF("print_detail", interface_ip.print_detail);
        ASSIGN_ENUM_IF("add_ecc_b_", interface_ip.add_ecc_b_, bool);

        else {
            warnUnrecognizedParam(node_name);
        }
    }

    // Change from MHz to Hz
    target_core_clockrate *= 1e6;
    interconnect_projection_type =
        (interconnect_projection_type == 0) ? 0 : 1;

    num_children = xml_data->nChildNode("stat");
    for (i = 0; i < num_children; i++) {
        XMLNode* statNode = xml_data->getChildNodePtr("stat", &i);
        XMLCSTR node_name = statNode->getAttribute("name");
        XMLCSTR value = statNode->getAttribute("value");

        if (!node_name)
            warnMissingStatName(statNode->getAttribute("id"));

        ASSIGN_FP_IF("total_cycles", total_cycles);

        else {
            warnUnrecognizedStat(node_name);
        }
    }

    if (temperature < 0) {
        errorUnspecifiedParam("temperature");
    }

    if (core_tech_node < 0) {
        errorUnspecifiedParam("core_tech_node");
    }

    if (interconnect_projection_type < 0) {
        errorUnspecifiedParam("interconnect_projection_type");
    }

    if (device_type < 0) {
        errorUnspecifiedParam("device_type");
    }

    if (physical_address_width <= 0) {
        errorNonPositiveParam("physical_address_width");
    }

    if (data_path_width <= 0) {
        errorNonPositiveParam("machine_bits");
    }

    if (total_cycles <= 0) {
        fprintf(stderr, "WARNING: total_cycles <= 0 in system component, ",
                "power numbers will be funky...\n");
    }

    clockRate = target_core_clockrate;
    execution_time = total_cycles / (target_core_clockrate);

    /* Basic parameters*/
    interface_ip.data_arr_ram_cell_tech_type = device_type;
    interface_ip.data_arr_peri_global_tech_type = device_type;
    interface_ip.tag_arr_ram_cell_tech_type = device_type;
    interface_ip.tag_arr_peri_global_tech_type = device_type;

    interface_ip.ic_proj_type = interconnect_projection_type;
    interface_ip.temp = temperature;
    interface_ip.F_sz_nm = core_tech_node;
    interface_ip.F_sz_um = interface_ip.F_sz_nm / 1000;
    interface_ip.is_main_mem = false;

    // These are there just to make CACTI's error_checking() happy.
    // They are either not actually used or overwritten by each component.
    interface_ip.cache_sz = MIN_BUFFER_SIZE;
    interface_ip.nbanks = 1;
    interface_ip.out_w = 0;
    interface_ip.line_sz = 1;
    interface_ip.assoc = 1;
    interface_ip.num_rw_ports = 1;
    interface_ip.num_search_ports = 1;
    interface_ip.is_cache = true;
    interface_ip.pure_ram = false;
    interface_ip.pure_cam = false;


    //This section of code does not have real meaning; it is just to ensure
    //all data will have initial value to prevent errors.
    //They will be overridden  during each components initialization
    interface_ip.specific_tag = 1;
    interface_ip.tag_w = 64;
    interface_ip.access_mode = 2;

    interface_ip.obj_func_dyn_energy = 0;
    interface_ip.obj_func_dyn_power = 0;
    interface_ip.obj_func_leak_power = 0;
    interface_ip.obj_func_cycle_t = 1;
    interface_ip.num_rw_ports = 1;
    interface_ip.num_rd_ports = 0;
    interface_ip.num_wr_ports = 0;
    interface_ip.num_se_rd_ports = 0;
}

System::~System() {
    // TODO: Delete children... do this in McPATComponent
};
