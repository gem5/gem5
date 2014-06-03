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
#include "iocontrollers.h"
#include "logic.h"

/*
SUN Niagara 2 I/O power analysis:
total signal bits: 711
Total FBDIMM bits: (14+10)*2*8= 384
PCIe bits:         (8 + 8)*2 = 32
10Gb NIC:          (4*2+4*2)*2 = 32
Debug I/Os:        168
Other I/Os:        711- 32-32 - 384 - 168 = 95

According to "Implementation of an 8-Core, 64-Thread, Power-Efficient SPARC Server on a Chip"
90% of I/Os are SerDers (the calucaltion is 384+64/(711-168)=83% about the same as the 90% reported in the paper)
--> around 80Pins are common I/Os.
Common I/Os consumes 71mW/Gb/s according to Cadence ChipEstimate @65nm
Niagara 2 I/O clock is 1/4 of core clock. --> 87pin (<--((711-168)*17%)) * 71mW/Gb/s *0.25*1.4Ghz = 2.17W

Total dynamic power of FBDIMM, NIC, PCIe = 84*0.132 + 84*0.049*0.132 = 11.14 - 2.17 = 8.98
Further, if assuming I/O logic power is about 50% of I/Os then Total energy of FBDIMM, NIC, PCIe = 11.14 - 2.17*1.5 = 7.89
 */

/*
 * A bug in Cadence ChipEstimator: After update the clock rate in the clock tab, a user
 * need to re-select the IP clock (the same clk) and then click Estimate. if not reselect
 * the new clock rate may not be propogate into the IPs.
 *
 */

NIUController::NIUController(XMLNode* _xml_data,InputParameter* interface_ip_)
    : McPATComponent(_xml_data, interface_ip_) {
    name = "NIU";
    set_niu_param();
}

void NIUController::computeArea() {
    double mac_area;
    double frontend_area;
    double SerDer_area;

    if (niup.type == 0) { //high performance NIU
        //Area estimation based on average of die photo from Niagara 2 and
        //Cadence ChipEstimate using 65nm.
        mac_area = (1.53 + 0.3) / 2 * (interface_ip.F_sz_um / 0.065) *
            (interface_ip.F_sz_um / 0.065);
        //Area estimation based on average of die photo from Niagara 2, ISSCC
        //"An 800mW 10Gb Ethernet Transceiver in 0.13μm CMOS"
        //and"A 1.2-V-Only 900-mW 10 Gb Ethernet Transceiver and XAUI Interface
        //With Robust VCO Tuning Technique" Frontend is PCS
        frontend_area = (9.8 + (6 + 18) * 65 / 130 * 65 / 130) / 3 *
            (interface_ip.F_sz_um / 0.065) * (interface_ip.F_sz_um / 0.065);
        //Area estimation based on average of die photo from Niagara 2 and
        //Cadence ChipEstimate hard IP @65nm.
        //SerDer is very hard to scale
        SerDer_area = (1.39 + 0.36) * (interface_ip.F_sz_um /
                                       0.065);//* (interface_ip.F_sz_um/0.065);
    } else {
        //Low power implementations are mostly from Cadence ChipEstimator;
        //Ignore the multiple IP effect
        // ---When there are multiple IP (same kind or not) selected, Cadence
        //ChipEstimator results are not a simple summation of all IPs.
        //Ignore this effect
        mac_area = 0.24 * (interface_ip.F_sz_um / 0.065) *
            (interface_ip.F_sz_um / 0.065);
        frontend_area = 0.1 * (interface_ip.F_sz_um / 0.065) *
            (interface_ip.F_sz_um / 0.065);//Frontend is the PCS layer
        SerDer_area = 0.35 * (interface_ip.F_sz_um / 0.065) *
            (interface_ip.F_sz_um/0.065);
        //Compare 130um implementation in "A 1.2-V-Only 900-mW 10 Gb Ethernet
        //Transceiver and XAUI Interface With Robust VCO Tuning Technique"
        //and the ChipEstimator XAUI PHY hard IP, confirm that even PHY can
        //scale perfectly with the technology
    }

    //total area
    output_data.area = (mac_area + frontend_area + SerDer_area) * 1e6;
 }

void NIUController::computeEnergy() {
    double mac_dyn;
    double frontend_dyn;
    double SerDer_dyn;
    double frontend_gates;
    double mac_gates;
    double SerDer_gates;
    double NMOS_sizing;
    double PMOS_sizing;
    double pmos_to_nmos_sizing_r = pmos_to_nmos_sz_ratio();

    if (niup.type == 0) { //high performance NIU
        //Power
        //Cadence ChipEstimate using 65nm (mac, front_end are all energy.
        //E=P*T = P/F = 1.37/1Ghz = 1.37e-9);
        //2.19W@1GHz fully active according to Cadence ChipEstimate @65nm
        mac_dyn = 2.19e-9 * g_tp.peri_global.Vdd / 1.1 * g_tp.peri_global.Vdd /
            1.1 * (interface_ip.F_sz_nm / 65.0);//niup.clockRate;
        //Cadence ChipEstimate using 65nm soft IP;
        frontend_dyn = 0.27e-9 * g_tp.peri_global.Vdd / 1.1 *
            g_tp.peri_global.Vdd / 1.1 * (interface_ip.F_sz_nm / 65.0);
        //according to "A 100mW 9.6Gb/s Transceiver in 90nm CMOS..." ISSCC 2006
        //SerDer_dyn is power not energy, scaling from 10mw/Gb/s @90nm
        SerDer_dyn = 0.01 * 10 * sqrt(interface_ip.F_sz_um / 0.09) *
            g_tp.peri_global.Vdd / 1.2 * g_tp.peri_global.Vdd / 1.2;

        //Cadence ChipEstimate using 65nm
        mac_gates = 111700;
        frontend_gates = 320000;
        SerDer_gates = 200000;
        NMOS_sizing = 5 * g_tp.min_w_nmos_;
        PMOS_sizing	= 5 * g_tp.min_w_nmos_ * pmos_to_nmos_sizing_r;
    } else {
        //Power
        //Cadence ChipEstimate using 65nm (mac, front_end are all energy.
        ///E=P*T = P/F = 1.37/1Ghz = 1.37e-9);
        //2.19W@1GHz fully active according to Cadence ChipEstimate @65nm
        mac_dyn = 1.257e-9 * g_tp.peri_global.Vdd / 1.1 * g_tp.peri_global.Vdd
            / 1.1 * (interface_ip.F_sz_nm / 65.0);//niup.clockRate;
        //Cadence ChipEstimate using 65nm soft IP;
        frontend_dyn = 0.6e-9 * g_tp.peri_global.Vdd / 1.1 *
            g_tp.peri_global.Vdd / 1.1 * (interface_ip.F_sz_nm / 65.0);
        //SerDer_dyn is power not energy, scaling from 216mw/10Gb/s @130nm
        SerDer_dyn = 0.0216 * 10 * (interface_ip.F_sz_um / 0.13) *
            g_tp.peri_global.Vdd / 1.2 * g_tp.peri_global.Vdd / 1.2;

        mac_gates = 111700;
        frontend_gates = 52000;
        SerDer_gates = 199260;
        NMOS_sizing = g_tp.min_w_nmos_;
        PMOS_sizing = g_tp.min_w_nmos_ * pmos_to_nmos_sizing_r;
    }

    //covert to energy per clock cycle of whole NIU
    SerDer_dyn /= niup.clockRate;

    power.readOp.dynamic = mac_dyn + frontend_dyn + SerDer_dyn;
    power.readOp.leakage = (mac_gates + frontend_gates + frontend_gates) *
        cmos_Isub_leakage(NMOS_sizing, PMOS_sizing, 2, nand) *
        g_tp.peri_global.Vdd;//unit W
    double long_channel_device_reduction =
        longer_channel_device_reduction(Uncore_device);
    power.readOp.longer_channel_leakage =
        power.readOp.leakage * long_channel_device_reduction;
    power.readOp.gate_leakage = (mac_gates + frontend_gates + frontend_gates) *
        cmos_Ig_leakage(NMOS_sizing, PMOS_sizing, 2, nand) *
        g_tp.peri_global.Vdd;//unit W

    // Output power
    output_data.subthreshold_leakage_power =
        longer_channel_device ? power.readOp.longer_channel_leakage :
        power.readOp.leakage;
    output_data.gate_leakage_power = power.readOp.gate_leakage;
    output_data.peak_dynamic_power = power.readOp.dynamic * nius.duty_cycle;
    output_data.runtime_dynamic_energy = power.readOp.dynamic * nius.perc_load;
}

void NIUController::set_niu_param() {
    int num_children = xml_data->nChildNode("param");
    int i;
    for (i = 0; i < num_children; i++) {
        XMLNode* paramNode = xml_data->getChildNodePtr("param", &i);
        XMLCSTR node_name = paramNode->getAttribute("name");
        XMLCSTR value = paramNode->getAttribute("value");

        if (!node_name)
            warnMissingParamName(paramNode->getAttribute("id"));

        ASSIGN_FP_IF("niu_clockRate", niup.clockRate);
        ASSIGN_INT_IF("num_units", niup.num_units);
        ASSIGN_INT_IF("type", niup.type);

        else {
            warnUnrecognizedParam(node_name);
        }
    }

    // Change from MHz to Hz
    niup.clockRate *= 1e6;

    num_children = xml_data->nChildNode("stat");
    for (i = 0; i < num_children; i++) {
        XMLNode* statNode = xml_data->getChildNodePtr("stat", &i);
        XMLCSTR node_name = statNode->getAttribute("name");
        XMLCSTR value = statNode->getAttribute("value");

        if (!node_name)
            warnMissingStatName(statNode->getAttribute("id"));

        ASSIGN_FP_IF("duty_cycle", nius.duty_cycle);
        ASSIGN_FP_IF("perc_load", nius.perc_load);

        else {
            warnUnrecognizedStat(node_name);
        }
    }
}

PCIeController::PCIeController(XMLNode* _xml_data,
                               InputParameter* interface_ip_)
    : McPATComponent(_xml_data, interface_ip_) {
    name = "PCIe";
    set_pcie_param();
}

void PCIeController::computeArea() {
    double ctrl_area;
    double SerDer_area;

    /* Assuming PCIe is bit-slice based architecture
     * This is the reason for /8 in both area and power calculation
     * to get per lane numbers
     */

    if (pciep.type == 0) { //high performance PCIe
        //Area estimation based on average of die photo from Niagara 2 and
        //Cadence ChipEstimate @ 65nm.
        ctrl_area = (5.2 + 0.5) / 2 * (interface_ip.F_sz_um / 0.065) *
            (interface_ip.F_sz_um / 0.065);
        //Area estimation based on average of die photo from Niagara 2 and
        //Cadence ChipEstimate hard IP @65nm.
        //SerDer is very hard to scale
        SerDer_area = (3.03 + 0.36) * (interface_ip.F_sz_um /
                                       0.065);//* (interface_ip.F_sz_um/0.065);
    } else {
        ctrl_area = 0.412 * (interface_ip.F_sz_um / 0.065) *
            (interface_ip.F_sz_um / 0.065);
        //Area estimation based on average of die photo from Niagara 2, and
        //Cadence ChipEstimate @ 65nm.
        SerDer_area = 0.36 * (interface_ip.F_sz_um / 0.065) *
            (interface_ip.F_sz_um / 0.065);
    }

    // Total area
    output_data.area = ((ctrl_area + (pciep.withPHY ? SerDer_area : 0)) / 8 *
                        pciep.num_channels) * 1e6;
}

void PCIeController::computeEnergy() {
    double ctrl_dyn;
    double SerDer_dyn;
    double ctrl_gates;
    double SerDer_gates = 0;
    double pmos_to_nmos_sizing_r = pmos_to_nmos_sz_ratio();
    double NMOS_sizing;
    double PMOS_sizing;

    /* Assuming PCIe is bit-slice based architecture
     * This is the reason for /8 in both area and power calculation
     * to get per lane numbers
     */

    if (pciep.type == 0) { //high performance PCIe
        //Power
        //Cadence ChipEstimate using 65nm the controller includes everything: the PHY, the data link and transaction layer
        ctrl_dyn = 3.75e-9 / 8 * g_tp.peri_global.Vdd / 1.1 *
            g_tp.peri_global.Vdd / 1.1 * (interface_ip.F_sz_nm / 65.0);
        //	  //Cadence ChipEstimate using 65nm soft IP;
        //	  frontend_dyn = 0.27e-9/8*g_tp.peri_global.Vdd/1.1*g_tp.peri_global.Vdd/1.1*(interface_ip.F_sz_nm/65.0);
        //SerDer_dyn is power not energy, scaling from 10mw/Gb/s @90nm
        //PCIe 2.0 max per lane speed is 4Gb/s
        SerDer_dyn = 0.01 * 4 * (interface_ip.F_sz_um /0.09) *
            g_tp.peri_global.Vdd / 1.2 * g_tp.peri_global.Vdd /1.2;

        //Cadence ChipEstimate using 65nm
        ctrl_gates = 900000 / 8 * pciep.num_channels;
        //	  frontend_gates   = 120000/8;
        //	  SerDer_gates     = 200000/8;
        NMOS_sizing = 5 * g_tp.min_w_nmos_;
        PMOS_sizing	= 5 * g_tp.min_w_nmos_ * pmos_to_nmos_sizing_r;
    } else {
        //Power
        //Cadence ChipEstimate using 65nm the controller includes everything: the PHY, the data link and transaction layer
        ctrl_dyn = 2.21e-9 / 8 * g_tp.peri_global.Vdd / 1.1 *
            g_tp.peri_global.Vdd / 1.1 * (interface_ip.F_sz_nm / 65.0);
        //	  //Cadence ChipEstimate using 65nm soft IP;
        //	  frontend_dyn = 0.27e-9/8*g_tp.peri_global.Vdd/1.1*g_tp.peri_global.Vdd/1.1*(interface_ip.F_sz_nm/65.0);
        //SerDer_dyn is power not energy, scaling from 10mw/Gb/s @90nm
        //PCIe 2.0 max per lane speed is 4Gb/s
        SerDer_dyn = 0.01 * 4 * (interface_ip.F_sz_um / 0.09) *
            g_tp.peri_global.Vdd / 1.2 * g_tp.peri_global.Vdd /1.2;

        //Cadence ChipEstimate using 65nm
        ctrl_gates = 200000 / 8 * pciep.num_channels;
        //	  frontend_gates   = 120000/8;
        SerDer_gates = 200000 / 8 * pciep.num_channels;
        NMOS_sizing = g_tp.min_w_nmos_;
        PMOS_sizing = g_tp.min_w_nmos_ * pmos_to_nmos_sizing_r;

    }

    //covert to energy per clock cycle
    SerDer_dyn /= pciep.clockRate;

    power.readOp.dynamic = (ctrl_dyn + (pciep.withPHY ? SerDer_dyn : 0)) *
        pciep.num_channels;
    power.readOp.leakage = (ctrl_gates + (pciep.withPHY ? SerDer_gates : 0)) *
        cmos_Isub_leakage(NMOS_sizing, PMOS_sizing, 2, nand) *
        g_tp.peri_global.Vdd;//unit W
    double long_channel_device_reduction =
        longer_channel_device_reduction(Uncore_device);
    power.readOp.longer_channel_leakage =
        power.readOp.leakage * long_channel_device_reduction;
    power.readOp.gate_leakage = (ctrl_gates +
                                 (pciep.withPHY ? SerDer_gates : 0)) *
        cmos_Ig_leakage(NMOS_sizing, PMOS_sizing, 2, nand) *
        g_tp.peri_global.Vdd;//unit W

    // Output power
    output_data.subthreshold_leakage_power =
        longer_channel_device ? power.readOp.longer_channel_leakage :
        power.readOp.leakage;
    output_data.gate_leakage_power = power.readOp.gate_leakage;
    output_data.peak_dynamic_power = power.readOp.dynamic * pcies.duty_cycle;
    output_data.runtime_dynamic_energy =
        power.readOp.dynamic * pcies.perc_load;
}

void PCIeController::set_pcie_param() {
    int num_children = xml_data->nChildNode("param");
    int i;
    for (i = 0; i < num_children; i++) {
        XMLNode* paramNode = xml_data->getChildNodePtr("param", &i);
        XMLCSTR node_name = paramNode->getAttribute("name");
        XMLCSTR value = paramNode->getAttribute("value");

        if (!node_name)
            warnMissingParamName(paramNode->getAttribute("id"));

        ASSIGN_FP_IF("pcie_clockRate", pciep.clockRate);
        ASSIGN_INT_IF("num_units", pciep.num_units);
        ASSIGN_INT_IF("num_channels", pciep.num_channels);
        ASSIGN_INT_IF("type", pciep.type);
        ASSIGN_ENUM_IF("withPHY", pciep.withPHY, bool);

        else {
            warnUnrecognizedParam(node_name);
        }
    }

    // Change from MHz to Hz
    pciep.clockRate *= 1e6;

    num_children = xml_data->nChildNode("stat");
    for (i = 0; i < num_children; i++) {
        XMLNode* statNode = xml_data->getChildNodePtr("stat", &i);
        XMLCSTR node_name = statNode->getAttribute("name");
        XMLCSTR value = statNode->getAttribute("value");

        if (!node_name)
            warnMissingStatName(statNode->getAttribute("id"));

        ASSIGN_FP_IF("duty_cycle", pcies.duty_cycle);
        ASSIGN_FP_IF("perc_load", pcies.perc_load);

        else {
            warnUnrecognizedStat(node_name);
        }
    }
}

FlashController::FlashController(XMLNode* _xml_data,
                                 InputParameter* interface_ip_)
    : McPATComponent(_xml_data, interface_ip_) {
    name = "Flash Controller";
    set_fc_param();
}

void FlashController::computeArea() {
    double ctrl_area;
    double SerDer_area;

    /* Assuming Flash is bit-slice based architecture
     * This is the reason for /8 in both area and power calculation
     * to get per lane numbers
     */

    if (fcp.type == 0) { //high performance flash controller
        cout << "Current McPAT does not support high performance flash "
             << "controller since even low power designs are enough for "
             << "maintain throughput" <<endl;
        exit(0);
    } else {
        ctrl_area = 0.243 * (interface_ip.F_sz_um / 0.065) *
            (interface_ip.F_sz_um / 0.065);
        //Area estimation based on Cadence ChipEstimate @ 65nm: NANDFLASH-CTRL
        //from CAST
        SerDer_area = 0.36 / 8 * (interface_ip.F_sz_um / 0.065) *
            (interface_ip.F_sz_um / 0.065);
    }

    double number_channel = 1 + (fcp.num_channels - 1) * 0.2;
    output_data.area = (ctrl_area + (fcp.withPHY ? SerDer_area : 0)) *
        1e6 * number_channel;
}

void FlashController::computeEnergy() {
    double ctrl_dyn;
    double SerDer_dyn;
    double ctrl_gates;
    double SerDer_gates;
    double pmos_to_nmos_sizing_r = pmos_to_nmos_sz_ratio();
    double NMOS_sizing;
    double PMOS_sizing;

    /* Assuming Flash is bit-slice based architecture
     * This is the reason for /8 in both area and power calculation
     * to get per lane numbers
     */

    if (fcp.type == 0) { //high performance flash controller
        cout << "Current McPAT does not support high performance flash "
             << "controller since even low power designs are enough for "
             << "maintain throughput" <<endl;
        exit(0);
        NMOS_sizing = 5 * g_tp.min_w_nmos_;
        PMOS_sizing = 5 * g_tp.min_w_nmos_ * pmos_to_nmos_sizing_r;
    } else {
        //based On PCIe PHY TSMC65GP from Cadence ChipEstimate @ 65nm, it
        //support 8x lanes with each lane speed up to 250MB/s (PCIe1.1x).
        //This is already saturate the 200MB/s of the flash controller core
        //above.
        ctrl_gates = 129267;
        SerDer_gates = 200000 / 8;
        NMOS_sizing = g_tp.min_w_nmos_;
        PMOS_sizing	= g_tp.min_w_nmos_ * pmos_to_nmos_sizing_r;

        //Power
        //Cadence ChipEstimate using 65nm the controller 125mW for every
        //200MB/s This is power not energy!
        ctrl_dyn = 0.125 * g_tp.peri_global.Vdd / 1.1 * g_tp.peri_global.Vdd /
            1.1 * (interface_ip.F_sz_nm / 65.0);
        //SerDer_dyn is power not energy, scaling from 10mw/Gb/s @90nm
        SerDer_dyn = 0.01 * 1.6 * (interface_ip.F_sz_um / 0.09) *
            g_tp.peri_global.Vdd / 1.2 * g_tp.peri_global.Vdd / 1.2;
        //max  Per controller speed is 1.6Gb/s (200MB/s)
    }

    double number_channel = 1 + (fcp.num_channels - 1) * 0.2;
    power.readOp.dynamic = (ctrl_dyn + (fcp.withPHY ? SerDer_dyn : 0)) *
        number_channel;
    power.readOp.leakage = ((ctrl_gates + (fcp.withPHY ? SerDer_gates : 0)) *
                            number_channel) *
        cmos_Isub_leakage(NMOS_sizing, PMOS_sizing, 2, nand) *
        g_tp.peri_global.Vdd;//unit W
    double long_channel_device_reduction =
        longer_channel_device_reduction(Uncore_device);
    power.readOp.longer_channel_leakage =
        power.readOp.leakage * long_channel_device_reduction;
    power.readOp.gate_leakage =
        ((ctrl_gates + (fcp.withPHY ? SerDer_gates : 0)) * number_channel) *
        cmos_Ig_leakage(NMOS_sizing, PMOS_sizing, 2, nand) *
        g_tp.peri_global.Vdd;//unit W

    // Output power
    output_data.subthreshold_leakage_power =
        longer_channel_device ? power.readOp.longer_channel_leakage :
        power.readOp.leakage;
    output_data.gate_leakage_power = power.readOp.gate_leakage;
    output_data.peak_dynamic_power = power.readOp.dynamic * fcs.duty_cycle;
    output_data.runtime_dynamic_energy = power.readOp.dynamic * fcs.perc_load;
}

void FlashController::set_fc_param()
{
    int num_children = xml_data->nChildNode("param");
    int i;
    for (i = 0; i < num_children; i++) {
        XMLNode* paramNode = xml_data->getChildNodePtr("param", &i);
        XMLCSTR node_name = paramNode->getAttribute("name");
        XMLCSTR value = paramNode->getAttribute("value");

        if (!node_name)
            warnMissingParamName(paramNode->getAttribute("id"));

        ASSIGN_INT_IF("num_channels", fcp.num_channels);
        ASSIGN_INT_IF("type", fcp.type);
        ASSIGN_ENUM_IF("withPHY", fcp.withPHY, bool);

        else {
            warnUnrecognizedParam(node_name);
        }
    }

    num_children = xml_data->nChildNode("stat");
    for (i = 0; i < num_children; i++) {
        XMLNode* statNode = xml_data->getChildNodePtr("stat", &i);
        XMLCSTR node_name = statNode->getAttribute("name");
        XMLCSTR value = statNode->getAttribute("value");

        if (!node_name)
            warnMissingStatName(statNode->getAttribute("id"));

        ASSIGN_FP_IF("duty_cycle", fcs.duty_cycle);
        ASSIGN_FP_IF("perc_load", fcs.perc_load);

        else {
            warnUnrecognizedStat(node_name);
        }
    }
}
