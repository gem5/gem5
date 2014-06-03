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

#include <cassert>
#include <cmath>
#include <iostream>

#include "basic_components.h"
#include "cacheunit.h"
#include "common.h"

// Turn this to true to get debugging messages
bool McPATComponent::debug = false;

bool McPATComponent::opt_for_clk = true;
int McPATComponent::longer_channel_device = 0;
// Number of cycles per second, 2GHz = 2e9
double McPATComponent::target_core_clockrate = 2e9;
double McPATComponent::total_cycles = 0.0f;
double McPATComponent::execution_time = 0.0f;
int McPATComponent::physical_address_width = 0;
int McPATComponent::virtual_address_width = 0;
int McPATComponent::virtual_memory_page_size = 0;
int McPATComponent::data_path_width = 0;

void McPATOutput::reset() {
    storage = 0.0;
    area = 0.0;
    peak_dynamic_power = 0.0;
    subthreshold_leakage_power = 0.0;
    gate_leakage_power = 0.0;
    runtime_dynamic_energy = 0.0;
}

McPATOutput operator+(const McPATOutput &lhs, const McPATOutput &rhs) {
    McPATOutput to_return;
    to_return.storage = lhs.storage + rhs.storage;
    to_return.area = lhs.area + rhs.area;
    to_return.peak_dynamic_power = lhs.peak_dynamic_power +
        rhs.peak_dynamic_power;
    to_return.subthreshold_leakage_power = lhs.subthreshold_leakage_power +
        rhs.subthreshold_leakage_power;
    to_return.gate_leakage_power = lhs.gate_leakage_power +
        rhs.gate_leakage_power;
    to_return.runtime_dynamic_energy = lhs.runtime_dynamic_energy +
        rhs.runtime_dynamic_energy;
    return to_return;
}

void McPATOutput::operator+=(const McPATOutput &rhs) {
    storage += rhs.storage;
    area += rhs.area;
    peak_dynamic_power += rhs.peak_dynamic_power;
    subthreshold_leakage_power += rhs.subthreshold_leakage_power;
    gate_leakage_power += rhs.gate_leakage_power;
    runtime_dynamic_energy += rhs.runtime_dynamic_energy;
}

McPATComponent::McPATComponent()
        : xml_data(NULL), name("") {
}

McPATComponent::McPATComponent(XMLNode* _xml_data)
        : xml_data(_xml_data), name("") {
}

McPATComponent::McPATComponent(XMLNode* _xml_data,
                               InputParameter* _interface_ip)
        : xml_data(_xml_data), interface_ip(*_interface_ip), name("") {
}

McPATComponent::~McPATComponent() {
}

void McPATComponent::recursiveInstantiate() {
    if (debug) {
        fprintf(stderr, "WARNING: Called recursiveInstantiate from %s, with ",
                "'type' %s\n", name.c_str(), xml_data->getAttribute("type"));
    }
    int i;
    int numChildren = xml_data->nChildNode("component");
    for (i = 0; i < numChildren; i++ ) {
        // For each child node of the system,
        XMLNode* childXML = xml_data->getChildNodePtr("component", &i);
        XMLCSTR type = childXML->getAttribute("type");

        if (!type)
            warnMissingComponentType(childXML->getAttribute("id"));

        STRCMP(type, "Core")
            warnIncompleteComponentType(type);
        STRCMP(type, "CacheUnit")
            children.push_back(new CacheUnit(childXML, &interface_ip));
        STRCMP(type, "CacheController")
            warnIncompleteComponentType(type);
        STRCMP(type, "MemoryController")
            warnIncompleteComponentType(type);
        STRCMP(type, "Memory")
            warnIncompleteComponentType(type);
        STRCMP(type, "OnChipNetwork")
            warnIncompleteComponentType(type);
        STRCMP(type, "BusInterconnect")
            warnIncompleteComponentType(type);
        STRCMP(type, "Directory")
            warnIncompleteComponentType(type);

        else
            warnUnrecognizedComponent(type);
    }
}

void McPATComponent::computeArea() {
    if (debug) {
        fprintf(stderr, "WARNING: Called computeArea from %s, with 'type' ",
                "%s\n", name.c_str(), xml_data->getAttribute("type"));
    }

    // TODO: This calculation is incorrect and is overwritten by computeEnergy
    // Fix it up so that the values are available at the correct times
    int i;
    int numChildren = children.size();
    area.set_area(0.0);
    output_data.area = 0.0;
    for (i = 0; i < numChildren; i++) {
        children[i]->computeArea();
        output_data.area += area.get_area();
    }
}

void McPATComponent::computeEnergy() {
    if (debug) {
        fprintf(stderr, "WARNING: Called computeEnergy from %s, with 'type' ",
                "%s\n", name.c_str(), xml_data->getAttribute("type"));
    }

    power.reset();
    rt_power.reset();
    memset(&output_data, 0, sizeof(McPATOutput));
    int i;
    int numChildren = children.size();
    for (i = 0; i < numChildren; i++) {
        children[i]->computeEnergy();
        output_data += children[i]->output_data;
    }
}

void McPATComponent::displayData(uint32_t indent, int plevel) {
    if (debug) {
        fprintf(stderr, "WARNING: Called displayData from %s, with 'type' ",
                "%s\n", name.c_str(), xml_data->getAttribute("type"));
    }

    string indent_str(indent, ' ');
    string indent_str_next(indent + 2, ' ');

    double leakage_power = output_data.subthreshold_leakage_power +
        output_data.gate_leakage_power;
    double total_runtime_energy = output_data.runtime_dynamic_energy +
        leakage_power * execution_time;
    cout << indent_str << name << ":" << endl;
    cout << indent_str_next << "Area = " << output_data.area << " mm^2"
         << endl;
    cout << indent_str_next << "Peak Dynamic Power = "
         << output_data.peak_dynamic_power << " W" << endl;
    cout << indent_str_next << "Subthreshold Leakage Power = "
         << output_data.subthreshold_leakage_power << " W" << endl;
    cout << indent_str_next << "Gate Leakage Power = "
         << output_data.gate_leakage_power << " W" << endl;
    cout << indent_str_next << "Runtime Dynamic Power = "
         << (output_data.runtime_dynamic_energy / execution_time) << " W"
         << endl;
    cout << indent_str_next << "Runtime Dynamic Energy = "
         << output_data.runtime_dynamic_energy << " J" << endl;
    cout << indent_str_next << "Total Runtime Energy = "
         << total_runtime_energy << " J" << endl;
    cout << endl;

    // Recursively print children
    int i;
    int numChildren = children.size();
    for (i = 0; i < numChildren; i++) {
        children[i]->displayData(indent + 4, plevel);
    }
}

void McPATComponent::errorUnspecifiedParam(string param) {
    fprintf(stderr, "ERROR: Parameter must be specified in %s: %s\n",
            name.c_str(), param.c_str());
    exit(1);
}

void McPATComponent::errorNonPositiveParam(string param) {
    fprintf(stderr, "ERROR: Parameter must be positive in %s: %s\n",
            name.c_str(), param.c_str());
    exit(1);
}

void McPATComponent::warnUnrecognizedComponent(XMLCSTR component) {
    fprintf(stderr, "WARNING: Component type not recognized in %s: %s\n",
            name.c_str(), component);
}

void McPATComponent::warnUnrecognizedParam(XMLCSTR param) {
    fprintf(stderr, "WARNING: Parameter not recognized in %s: %s\n",
            name.c_str(), param);
}

void McPATComponent::warnUnrecognizedStat(XMLCSTR stat) {
    fprintf(stderr, "WARNING: Statistic not recognized in %s: %s\n",
            name.c_str(), stat);
}

void McPATComponent::warnIncompleteComponentType(XMLCSTR type) {
    fprintf(stderr, "  WARNING: %s handling not yet complete\n", type);
}

void McPATComponent::warnMissingComponentType(XMLCSTR id) {
    if (id) {
        fprintf(stderr,
                "WARNING: Ignoring a component due to the missing type: %s\n",
                id);
    } else {
        fprintf(stderr,
                "WARNING: Ignoring a component in %s due to the missing type\n",
                name.c_str());
    }
}

void McPATComponent::warnMissingParamName(XMLCSTR id) {
    if (id) {
        fprintf(stderr,
                "WARNING: Ignoring a parameter due to the missing name: %s\n",
                id);
    } else {
        fprintf(stderr,
                "WARNING: Ignoring a parameter in %s due to the missing name\n",
                name.c_str());
    }
}

void McPATComponent::warnMissingStatName(XMLCSTR id) {
    if (id) {
        fprintf(stderr,
                "WARNING: Ignoring a statistic due to the missing name: %s\n",
                id);
    } else {
        fprintf(stderr,
                "WARNING: Ignoring a statistic in %s due to the missing name\n",
                name.c_str());
    }
}

double longer_channel_device_reduction(
    enum Device_ty device_ty,
    enum Core_type core_ty) {

    double longer_channel_device_percentage_core;
    double longer_channel_device_percentage_uncore;
    double longer_channel_device_percentage_llc;

    double long_channel_device_reduction;

    longer_channel_device_percentage_llc    = 1.0;
    longer_channel_device_percentage_uncore = 0.82;
    if (core_ty == OOO) {
        //0.54 Xeon Tulsa //0.58 Nehelam
        longer_channel_device_percentage_core   = 0.56;
    } else {
        //0.8;//Niagara
        longer_channel_device_percentage_core   = 0.8;
    }

    if (device_ty == Core_device) {
        long_channel_device_reduction =
            (1 - longer_channel_device_percentage_core) +
            longer_channel_device_percentage_core *
            g_tp.peri_global.long_channel_leakage_reduction;
    } else if (device_ty == Uncore_device) {
        long_channel_device_reduction =
            (1 - longer_channel_device_percentage_uncore) +
            longer_channel_device_percentage_uncore *
            g_tp.peri_global.long_channel_leakage_reduction;
    } else if (device_ty == LLC_device) {
        long_channel_device_reduction =
            (1 - longer_channel_device_percentage_llc) +
            longer_channel_device_percentage_llc *
            g_tp.peri_global.long_channel_leakage_reduction;
    } else {
        cout << "ERROR: Unknown device category: " << device_ty << endl;
        exit(0);
    }

    return long_channel_device_reduction;
}

statsComponents operator+(const statsComponents & x, const statsComponents & y) {
    statsComponents z;

    z.access = x.access + y.access;
    z.hit    = x.hit + y.hit;
    z.miss   = x.miss  + y.miss;

    return z;
}

statsComponents operator*(const statsComponents & x, double const * const y) {
    statsComponents z;

    z.access = x.access * y[0];
    z.hit    = x.hit * y[1];
    z.miss   = x.miss * y[2];

    return z;
}

statsDef operator+(const statsDef & x, const statsDef & y) {
    statsDef z;

    z.readAc   = x.readAc  + y.readAc;
    z.writeAc  = x.writeAc + y.writeAc;
    z.searchAc  = x.searchAc + y.searchAc;
    return z;
}

statsDef operator*(const statsDef & x, double const * const y) {
    statsDef z;

    z.readAc   = x.readAc * y;
    z.writeAc  = x.writeAc * y;
    z.searchAc  = x.searchAc * y;
    return z;
}
