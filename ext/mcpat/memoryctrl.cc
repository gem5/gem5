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
#include "basic_components.h"
#include "common.h"
#include "const.h"
#include "io.h"
#include "logic.h"
#include "memoryctrl.h"
#include "parameter.h"

/* overview of MC models:
 * McPAT memory controllers are modeled according to large number of industrial data points.
 * The Basic memory controller architecture is base on the Synopsis designs
 * (DesignWare DDR2/DDR3-Lite memory controllers and DDR2/DDR3-Lite protocol controllers)
 * as in Cadence ChipEstimator Tool
 *
 * An MC has 3 parts as shown in this design. McPAT models both high performance MC
 * based on Niagara processor designs and curving and low power MC based on data points in
 * Cadence ChipEstimator Tool.
 *
 * The frontend is modeled analytically, the backend is modeled empirically according to
 * DDR2/DDR3-Lite protocol controllers in Cadence ChipEstimator Tool
 * The PHY is modeled based on
 * "A 100mW 9.6Gb/s Transceiver in 90nm CMOS for next-generation memory interfaces ," ISSCC 2006,
 * and A 14mW 6.25Gb/s Transceiver in 90nm CMOS for Serial Chip-to-Chip Communication," ISSCC 2007
 *
 * In Cadence ChipEstimator Tool there are two types of memory controllers: the full memory controllers
 * that includes the frontend as the DesignWare DDR2/DDR3-Lite memory controllers and the backend only
 * memory controllers as the DDR2/DDR3-Lite protocol controllers (except DesignWare DDR2/DDR3-Lite memory
 * controllers, all memory controller IP in Cadence ChipEstimator Tool are backend memory controllers such as
 * DDRC 1600A and DDRC 800A). Thus,to some extend the area and power difference between DesignWare
 * DDR2/DDR3-Lite memory controllers and DDR2/DDR3-Lite protocol controllers can be an estimation to the
 * frontend power and area, which is very close the analitically modeled results of the frontend for Niagara2@65nm
 *
 */

MCBackend::MCBackend(XMLNode* _xml_data, InputParameter* interface_ip_,
                     const MCParameters & mcp_, const MCStatistics & mcs_)
    : McPATComponent(_xml_data), l_ip(*interface_ip_), mcp(mcp_), mcs(mcs_) {
    name = "Transaction Engine";
    local_result = init_interface(&l_ip, name);

    // Set up stats for the power calculations
    tdp_stats.reset();
    tdp_stats.readAc.access = 0.5 * mcp.num_channels * mcp.clockRate;
    tdp_stats.writeAc.access = 0.5 * mcp.num_channels * mcp.clockRate;
    rtp_stats.reset();
    rtp_stats.readAc.access = mcs.reads;
    rtp_stats.writeAc.access = mcs.writes;
}

void MCBackend::computeArea() {
    // The area is in nm^2
    if (mcp.mc_type == MC) {
        if (mcp.type == 0) {
            output_data.area = (2.7927 * log(mcp.peak_transfer_rate * 2) -
                                19.862) / 2.0 * mcp.dataBusWidth / 128.0 *
                (l_ip.F_sz_um / 0.09) * mcp.num_channels;
        } else {
            output_data.area = 0.15 * mcp.dataBusWidth / 72.0 *
                (l_ip.F_sz_um / 0.065) * (l_ip.F_sz_um / 0.065) *
                mcp.num_channels;
        }
    } else {
        //skip old model
        cout << "Unknown memory controllers" << endl;
        exit(0);
        //area based on Cadence ChipEstimator for 8bit bus
        output_data.area = 0.243 * mcp.dataBusWidth / 8;
    }
}


void MCBackend::computeEnergy() {
    double C_MCB, mc_power;
    double backend_dyn;
    double backend_gates;
    double pmos_to_nmos_sizing_r = pmos_to_nmos_sz_ratio();
    double NMOS_sizing = g_tp.min_w_nmos_;
    double PMOS_sizing = g_tp.min_w_nmos_ * pmos_to_nmos_sizing_r;
    double area_um2 = output_data.area * 1e6;

    if (mcp.mc_type == MC) {
        if (mcp.type == 0) {
            //assuming the approximately same scaling factor as seen in processors.
            //C_MCB = 1.6/200/1e6/144/1.2/1.2*g_ip.F_sz_um/0.19;//Based on Niagara power numbers.The base power (W) is divided by device frequency and vdd and scale to target process.
            //mc_power = 0.0291*2;//29.1mW@200MHz @130nm From Power Analysis of SystemLevel OnChip Communication Architectures by Lahiri et
            mc_power = 4.32*0.1;//4.32W@1GhzMHz @65nm Cadence ChipEstimator 10% for backend
            C_MCB = mc_power/1e9/72/1.1/1.1*l_ip.F_sz_um/0.065;
            //per access energy in memory controller
            power.readOp.dynamic = C_MCB * g_tp.peri_global.Vdd *
                g_tp.peri_global.Vdd *
                (mcp.dataBusWidth/*+mcp.addressBusWidth*/);
            power.readOp.leakage = area_um2 / 2 *
                (g_tp.scaling_factor.core_tx_density) *
                cmos_Isub_leakage(NMOS_sizing, PMOS_sizing, 1, inv) *
                g_tp.peri_global.Vdd;//unit W
            power.readOp.gate_leakage = area_um2 / 2 *
                (g_tp.scaling_factor.core_tx_density) *
                cmos_Ig_leakage(NMOS_sizing, PMOS_sizing, 1, inv) *
                g_tp.peri_global.Vdd;//unit W
        } else {
            //Average on DDR2/3 protocol controller and DDRC 1600/800A in
            //Cadence ChipEstimate
            backend_dyn = 0.9e-9 / 800e6 * mcp.clockRate / 12800 *
                mcp.peak_transfer_rate* mcp.dataBusWidth / 72.0 *
                g_tp.peri_global.Vdd / 1.1 * g_tp.peri_global.Vdd / 1.1 *
                (l_ip.F_sz_nm/65.0);
            //Scaling to technology and DIMM feature. The base IP support
            //DDR3-1600(PC3 12800)
            //5000 is from Cadence ChipEstimator
            backend_gates = 50000 * mcp.dataBusWidth / 64.0;

            power.readOp.dynamic = backend_dyn;
            power.readOp.leakage = (backend_gates) *
                cmos_Isub_leakage(NMOS_sizing, PMOS_sizing, 2, nand) *
                g_tp.peri_global.Vdd;//unit W
            power.readOp.gate_leakage = (backend_gates) *
                cmos_Ig_leakage(NMOS_sizing, PMOS_sizing, 2, nand) *
                g_tp.peri_global.Vdd;//unit W
          }
    } else {
        //skip old model
        cout<<"Unknown memory controllers"<<endl;exit(0);
        //mc_power = 4.32*0.1;//4.32W@1GhzMHz @65nm Cadence ChipEstimator 10% for backend
        C_MCB = mc_power/1e9/72/1.1/1.1*l_ip.F_sz_um/0.065;
        power.readOp.leakage = area_um2 / 2 *
            (g_tp.scaling_factor.core_tx_density) *
            cmos_Isub_leakage(NMOS_sizing, PMOS_sizing, 1, inv) *
            g_tp.peri_global.Vdd;//unit W
        power.readOp.gate_leakage = area_um2 / 2 *
            (g_tp.scaling_factor.core_tx_density) *
            cmos_Ig_leakage(NMOS_sizing, PMOS_sizing, 1, inv) *
            g_tp.peri_global.Vdd;//unit W
        power.readOp.dynamic *= 1.2;
        power.readOp.leakage *= 1.2;
        power.readOp.gate_leakage *= 1.2;
        //flash controller has about 20% more backend power since BCH ECC in
        //flash is complex and power hungry
    }
  double long_channel_device_reduction =
      longer_channel_device_reduction(Uncore_device);
  power.readOp.longer_channel_leakage = power.readOp.leakage *
      long_channel_device_reduction;

  // Output leakage power calculations
  output_data.subthreshold_leakage_power =
      longer_channel_device ? power.readOp.longer_channel_leakage :
      power.readOp.leakage;
  output_data.gate_leakage_power = power.readOp.gate_leakage;

  // Peak dynamic power calculation
  output_data.peak_dynamic_power = power.readOp.dynamic *
      (tdp_stats.readAc.access + tdp_stats.writeAc.access);

  // Runtime dynamic energy calculation
  output_data.runtime_dynamic_energy =
      power.readOp.dynamic *
      (rtp_stats.readAc.access + rtp_stats.writeAc.access) *
      mcp.llcBlockSize * BITS_PER_BYTE / mcp.dataBusWidth +
      // Original McPAT code: Assume 10% of peak power is consumed by routine
      // job including memory refreshing and scrubbing
      power.readOp.dynamic * 0.1 * execution_time;
}

MCPHY::MCPHY(XMLNode* _xml_data, InputParameter* interface_ip_,
             const MCParameters & mcp_, const MCStatistics & mcs_)
    : McPATComponent(_xml_data), l_ip(*interface_ip_), mcp(mcp_), mcs(mcs_) {
    name = "Physical Interface (PHY)";
    local_result = init_interface(&l_ip, name);

    // Set up stats for the power calculations
    // TODO: Figure out why TDP stats aren't used
    tdp_stats.reset();
    tdp_stats.readAc.access = 0.5 * mcp.num_channels;
    tdp_stats.writeAc.access = 0.5 * mcp.num_channels;
    rtp_stats.reset();
    rtp_stats.readAc.access = mcs.reads;
    rtp_stats.writeAc.access = mcs.writes;
}

void MCPHY::computeArea() {
    if (mcp.mc_type == MC) {
        if (mcp.type == 0) {
            //Based on die photos from Niagara 1 and 2.
            //TODO merge this into undifferentiated core.PHY only achieves
            //square root of the ideal scaling.
            output_data.area = (6.4323 * log(mcp.peak_transfer_rate * 2) -
                                48.134) * mcp.dataBusWidth / 128.0 *
                (l_ip.F_sz_um / 0.09) * mcp.num_channels / 2;//TODO:/2
        } else {
            //Designware/synopsis 16bit DDR3 PHY is 1.3mm (WITH IOs) at 40nm
            //for upto DDR3 2133 (PC3 17066)
            double non_IO_percentage = 0.2;
            output_data.area = 1.3 * non_IO_percentage / 2133.0e6 *
                mcp.clockRate / 17066 * mcp.peak_transfer_rate *
                mcp.dataBusWidth / 16.0 * (l_ip.F_sz_um / 0.040)*
                (l_ip.F_sz_um / 0.040) * mcp.num_channels;//um^2
        }
    } else {
        //area based on Cadence ChipEstimator for 8bit bus
        output_data.area = 0.4e6 / 2 * mcp.dataBusWidth / 8 / 1e6;
    }
}

void MCPHY::computeEnergy() {
    //PHY uses internal data buswidth but the actuall off-chip datawidth is 64bits + ecc
    double pmos_to_nmos_sizing_r = pmos_to_nmos_sz_ratio();
    /*
     * according to "A 100mW 9.6Gb/s Transceiver in 90nm CMOS for next-generation memory interfaces ," ISSCC 2006;
     * From Cadence ChipEstimator for normal I/O around 0.4~0.8 mW/Gb/s
     */
    double power_per_gb_per_s, phy_dyn,phy_gates;
    double NMOS_sizing = g_tp.min_w_nmos_;
    double PMOS_sizing = g_tp.min_w_nmos_ * pmos_to_nmos_sizing_r;
    double area_um2 = output_data.area * 1e6;

    if (mcp.mc_type == MC) {
        if (mcp.type == 0) {
            power_per_gb_per_s = mcp.LVDS ? 0.01 : 0.04;
            //This is from curve fitting based on Niagara 1 and 2's PHY die photo.
            //This is power not energy, 10mw/Gb/s @90nm for each channel and scaling down
            //power.readOp.dynamic = 0.02*memAccesses*llcBlocksize*8;//change from Bytes to bits.
            power.readOp.dynamic = power_per_gb_per_s *
                sqrt(l_ip.F_sz_um / 0.09) * g_tp.peri_global.Vdd / 1.2 *
                g_tp.peri_global.Vdd / 1.2;
            power.readOp.leakage = area_um2 / 2 *
                (g_tp.scaling_factor.core_tx_density) *
                cmos_Isub_leakage(NMOS_sizing, PMOS_sizing, 1, inv) *
                g_tp.peri_global.Vdd;//unit W
            power.readOp.gate_leakage = area_um2 / 2 *
                (g_tp.scaling_factor.core_tx_density) *
                cmos_Ig_leakage(NMOS_sizing, PMOS_sizing, 1, inv) *
                g_tp.peri_global.Vdd;//unit W
        } else {
            phy_gates = 200000 * mcp.dataBusWidth / 64.0;
            power_per_gb_per_s = 0.01;
            //This is power not energy, 10mw/Gb/s @90nm for each channel and scaling down
            power.readOp.dynamic = power_per_gb_per_s * (l_ip.F_sz_um / 0.09) *
                g_tp.peri_global.Vdd / 1.2 * g_tp.peri_global.Vdd / 1.2;
            power.readOp.leakage = (mcp.withPHY ? phy_gates : 0) *
                cmos_Isub_leakage(NMOS_sizing, PMOS_sizing, 2, nand) *
                g_tp.peri_global.Vdd;//unit W
            power.readOp.gate_leakage = (mcp.withPHY ? phy_gates : 0) *
                cmos_Ig_leakage(NMOS_sizing, PMOS_sizing, 2, nand) *
                g_tp.peri_global.Vdd;//unit W
        }
    }

//  double phy_factor = (int)ceil(mcp.dataBusWidth/72.0);//Previous phy power numbers are based on 72 bit DIMM interface
//  power_t.readOp.dynamic *= phy_factor;
//  power_t.readOp.leakage *= phy_factor;
//  power_t.readOp.gate_leakage *= phy_factor;

    double long_channel_device_reduction =
        longer_channel_device_reduction(Uncore_device);
    power.readOp.longer_channel_leakage =
        power.readOp.leakage * long_channel_device_reduction;

    // Leakage power calculations
    output_data.subthreshold_leakage_power =
        longer_channel_device ? power.readOp.longer_channel_leakage :
        power.readOp.leakage;
    output_data.gate_leakage_power = power.readOp.gate_leakage;

    // Peak dynamic power calculation
    double data_transfer_unit = (mcp.mc_type == MC)? 72:16;/*DIMM data width*/
    output_data.peak_dynamic_power = power.readOp.dynamic *
        (mcp.peak_transfer_rate * BITS_PER_BYTE / 1e3) * mcp.dataBusWidth /
        data_transfer_unit * mcp.num_channels / mcp.clockRate;

    // Runtime dynamic energy calculation
    output_data.runtime_dynamic_energy =
        power.readOp.dynamic *
        (rtp_stats.readAc.access + rtp_stats.writeAc.access) *
        mcp.llcBlockSize * BITS_PER_BYTE / 1e9 +
        // Original McPAT code: Assume 10% of peak power is consumed by routine
        // job including memory refreshing and scrubbing
        power.readOp.dynamic * 0.1 * execution_time;
}

MCFrontEnd::MCFrontEnd(XMLNode* _xml_data, InputParameter* interface_ip_,
                       const MCParameters & mcp_, const MCStatistics & mcs_)
    : McPATComponent(_xml_data), frontendBuffer(NULL), readBuffer(NULL),
      writeBuffer(NULL), MC_arb(NULL), interface_ip(*interface_ip_),
    mcp(mcp_), mcs(mcs_) {
    int tag, data;
    bool is_default = true;//indication for default setup

    /* MC frontend engine channels share the same engines but logically partitioned
     * For all hardware inside MC. different channels do not share resources.
     * TODO: add docodeing/mux stage to steer memory requests to different channels.
     */

    name = "Front End";

    // Memory Request Reorder Buffer
    tag = mcp.addressbus_width + EXTRA_TAG_BITS + mcp.opcodeW;
    data = int(ceil((physical_address_width + mcp.opcodeW) / BITS_PER_BYTE));

    interface_ip.cache_sz = data * mcp.req_window_size_per_channel;
    interface_ip.line_sz = data;
    interface_ip.assoc = mcp.reorder_buffer_assoc;
    interface_ip.nbanks = mcp.reorder_buffer_nbanks;
    interface_ip.out_w = interface_ip.line_sz * BITS_PER_BYTE;
    interface_ip.specific_tag = tag > 0;
    interface_ip.tag_w = tag;
    interface_ip.access_mode = Normal;
    interface_ip.obj_func_dyn_energy = 0;
    interface_ip.obj_func_dyn_power = 0;
    interface_ip.obj_func_leak_power = 0;
    interface_ip.obj_func_cycle_t = 1;
    interface_ip.num_rw_ports = 0;
    interface_ip.num_rd_ports = mcp.num_channels;
    interface_ip.num_wr_ports = interface_ip.num_rd_ports;
    interface_ip.num_se_rd_ports = 0;
    interface_ip.num_search_ports = mcp.num_channels;
    interface_ip.is_cache = true;
    interface_ip.pure_cam = false;
    interface_ip.pure_ram = false;
    interface_ip.throughput = 1.0 / mcp.clockRate;
    interface_ip.latency = 1.0 / mcp.clockRate;
    frontendBuffer = new CacheArray(xml_data, &interface_ip, "Reorder Buffer",
                                    Uncore_device, mcp.clockRate);
    children.push_back(frontendBuffer);

    frontendBuffer->tdp_stats.reset();
    frontendBuffer->tdp_stats.readAc.access =
        frontendBuffer->l_ip.num_search_ports +
        frontendBuffer->l_ip.num_wr_ports;
    frontendBuffer->tdp_stats.writeAc.access =
        frontendBuffer->l_ip.num_search_ports;
    frontendBuffer->tdp_stats.searchAc.access =
        frontendBuffer->l_ip.num_wr_ports;
    frontendBuffer->rtp_stats.reset();
    // TODO: These stats assume that access power is calculated per buffer
    // bit, which requires the stats to take into account the number of
    // bits for each buffer slot. This should be revised...
    //For each channel, each memory word need to check the address data to
    //achieve best scheduling results.
    //and this need to be done on all physical DIMMs in each logical memory
    //DIMM *mcp.dataBusWidth/72
    frontendBuffer->rtp_stats.readAc.access = mcs.reads * mcp.llcBlockSize *
        BITS_PER_BYTE / mcp.dataBusWidth * mcp.dataBusWidth / 72;
    frontendBuffer->rtp_stats.writeAc.access = mcs.writes * mcp.llcBlockSize *
        BITS_PER_BYTE / mcp.dataBusWidth * mcp.dataBusWidth / 72;
    frontendBuffer->rtp_stats.searchAc.access =
        frontendBuffer->rtp_stats.readAc.access +
        frontendBuffer->rtp_stats.writeAc.access;

    // Read Buffers
    //Support key words first operation
    data = (int)ceil(mcp.dataBusWidth / BITS_PER_BYTE);

    interface_ip.cache_sz = data * mcp.IO_buffer_size_per_channel;
    interface_ip.line_sz = data;
    interface_ip.assoc = mcp.read_buffer_assoc;
    interface_ip.nbanks = mcp.read_buffer_nbanks;
    interface_ip.out_w = interface_ip.line_sz * BITS_PER_BYTE;
    interface_ip.specific_tag = mcp.read_buffer_tag_width > 0;
    interface_ip.tag_w = mcp.read_buffer_tag_width;
    interface_ip.access_mode = Sequential;
    interface_ip.obj_func_dyn_energy = 0;
    interface_ip.obj_func_dyn_power = 0;
    interface_ip.obj_func_leak_power = 0;
    interface_ip.obj_func_cycle_t = 1;
    interface_ip.num_rw_ports = 0;
    interface_ip.num_rd_ports = mcp.num_channels;
    interface_ip.num_wr_ports = interface_ip.num_rd_ports;
    interface_ip.num_se_rd_ports = 0;
    interface_ip.num_search_ports = 0;
    interface_ip.is_cache = false;
    interface_ip.pure_cam = false;
    interface_ip.pure_ram = true;
    interface_ip.throughput = 1.0 / mcp.clockRate;
    interface_ip.latency = 1.0 / mcp.clockRate;
    readBuffer = new CacheArray(xml_data, &interface_ip, "Read Buffer",
                                Uncore_device, mcp.clockRate);
    children.push_back(readBuffer);

    readBuffer->tdp_stats.reset();
    readBuffer->tdp_stats.readAc.access = readBuffer->l_ip.num_rd_ports *
        mcs.duty_cycle;
    readBuffer->tdp_stats.writeAc.access = readBuffer->l_ip.num_wr_ports *
        mcs.duty_cycle;
    readBuffer->rtp_stats.reset();
    readBuffer->rtp_stats.readAc.access = mcs.reads * mcp.llcBlockSize *
        BITS_PER_BYTE / mcp.dataBusWidth;
    readBuffer->rtp_stats.writeAc.access = mcs.reads * mcp.llcBlockSize *
        BITS_PER_BYTE / mcp.dataBusWidth;

    // Write Buffer
    //Support key words first operation
    data = (int)ceil(mcp.dataBusWidth / BITS_PER_BYTE);

    interface_ip.cache_sz = data * mcp.IO_buffer_size_per_channel;
    interface_ip.line_sz = data;
    interface_ip.assoc = mcp.write_buffer_assoc;
    interface_ip.nbanks = mcp.write_buffer_nbanks;
    interface_ip.out_w = interface_ip.line_sz * BITS_PER_BYTE;
    interface_ip.specific_tag = mcp.write_buffer_tag_width > 0;
    interface_ip.tag_w = mcp.write_buffer_tag_width;
    interface_ip.access_mode = Normal;
    interface_ip.obj_func_dyn_energy = 0;
    interface_ip.obj_func_dyn_power = 0;
    interface_ip.obj_func_leak_power = 0;
    interface_ip.obj_func_cycle_t = 1;
    interface_ip.num_rw_ports = 0;
    interface_ip.num_rd_ports = mcp.num_channels;
    interface_ip.num_wr_ports = interface_ip.num_rd_ports;
    interface_ip.num_se_rd_ports = 0;
    interface_ip.num_search_ports = 0;
    interface_ip.is_cache = false;
    interface_ip.pure_cam = false;
    interface_ip.pure_ram = true;
    interface_ip.throughput = 1.0 / mcp.clockRate;
    interface_ip.latency = 1.0 / mcp.clockRate;
    writeBuffer = new CacheArray(xml_data, &interface_ip, "Write Buffer",
                                 Uncore_device, mcp.clockRate);
    children.push_back(writeBuffer);

    writeBuffer->tdp_stats.reset();
    writeBuffer->tdp_stats.readAc.access = writeBuffer->l_ip.num_rd_ports *
        mcs.duty_cycle;
    writeBuffer->tdp_stats.writeAc.access = writeBuffer->l_ip.num_wr_ports *
        mcs.duty_cycle;
    writeBuffer->rtp_stats.reset();
    writeBuffer->rtp_stats.readAc.access = mcs.reads * mcp.llcBlockSize *
        BITS_PER_BYTE / mcp.dataBusWidth;
    writeBuffer->rtp_stats.writeAc.access = mcs.writes * mcp.llcBlockSize *
        BITS_PER_BYTE / mcp.dataBusWidth;

    // TODO: Set up selection logic as a leaf node in tree
    //selection and arbitration logic
    MC_arb =
        new selection_logic(xml_data, is_default,
                            mcp.req_window_size_per_channel, 1, &interface_ip,
                            "Arbitration Logic", (mcs.reads + mcs.writes),
                            mcp.clockRate, Uncore_device);
    // MC_arb is not included in the roll-up due to the uninitialized area
    //children.push_back(MC_arb);
}

MemoryController::MemoryController(XMLNode* _xml_data,
                                   InputParameter* interface_ip_)
    : McPATComponent(_xml_data), interface_ip(*interface_ip_) {
    name = "Memory Controller";
    set_mc_param();
    // TODO: Pass params and stats as pointers
    children.push_back(new MCFrontEnd(xml_data, &interface_ip, mcp, mcs));
    children.push_back(new MCBackend(xml_data, &interface_ip, mcp, mcs));

    if (mcp.type==0 || (mcp.type == 1 && mcp.withPHY)) {
        children.push_back(new MCPHY(xml_data, &interface_ip, mcp, mcs));
    }
}

void MemoryController::initialize_params() {
    memset(&mcp, 0, sizeof(MCParameters));
}

void MemoryController::set_mc_param() {
    initialize_params();

    int num_children = xml_data->nChildNode("param");
    int tech_type;
    int mat_type;
    int i;
    for (i = 0; i < num_children; i++) {
        XMLNode* paramNode = xml_data->getChildNodePtr("param", &i);
        XMLCSTR node_name = paramNode->getAttribute("name");
        XMLCSTR value = paramNode->getAttribute("value");

        if (!node_name)
            warnMissingParamName(paramNode->getAttribute("id"));

        ASSIGN_FP_IF("mc_clock", mcp.clockRate);
        ASSIGN_INT_IF("tech_type", tech_type);
        ASSIGN_ENUM_IF("mc_type", mcp.mc_type, MemoryCtrl_type);
        ASSIGN_FP_IF("num_mcs", mcp.num_mcs);
        ASSIGN_INT_IF("llc_line_length", mcp.llc_line_length);
        ASSIGN_INT_IF("databus_width", mcp.databus_width);
        ASSIGN_INT_IF("memory_channels_per_mc", mcp.num_channels);
        ASSIGN_INT_IF("req_window_size_per_channel",
                      mcp.req_window_size_per_channel);
        ASSIGN_INT_IF("IO_buffer_size_per_channel",
                      mcp.IO_buffer_size_per_channel);
        ASSIGN_INT_IF("addressbus_width", mcp.addressbus_width);
        ASSIGN_INT_IF("opcode_width", mcp.opcodeW);
        ASSIGN_INT_IF("type", mcp.type);
        ASSIGN_ENUM_IF("LVDS", mcp.LVDS, bool);
        ASSIGN_ENUM_IF("withPHY", mcp.withPHY, bool);
        ASSIGN_INT_IF("peak_transfer_rate", mcp.peak_transfer_rate);
        ASSIGN_INT_IF("number_ranks", mcp.number_ranks);
        ASSIGN_INT_IF("reorder_buffer_assoc", mcp.reorder_buffer_assoc);
        ASSIGN_INT_IF("reorder_buffer_nbanks", mcp.reorder_buffer_nbanks);
        ASSIGN_INT_IF("read_buffer_assoc", mcp.read_buffer_assoc);
        ASSIGN_INT_IF("read_buffer_nbanks", mcp.read_buffer_nbanks);
        ASSIGN_INT_IF("read_buffer_tag_width", mcp.read_buffer_tag_width);
        ASSIGN_INT_IF("write_buffer_assoc", mcp.write_buffer_assoc);
        ASSIGN_INT_IF("write_buffer_nbanks", mcp.write_buffer_nbanks);
        ASSIGN_INT_IF("write_buffer_tag_width", mcp.write_buffer_tag_width);
        ASSIGN_INT_IF("wire_mat_type", mat_type);
        ASSIGN_ENUM_IF("wire_type", interface_ip.wt, Wire_type);

        else {
            warnUnrecognizedParam(node_name);
        }
    }

    if (mcp.mc_type != MC) {
        cout << "Unknown memory controller type: Only DRAM controller is "
             << "supported for now" << endl;
                exit(0);
    }

    // Change from MHz to Hz
    mcp.clockRate *= 1e6;

    interface_ip.data_arr_ram_cell_tech_type    = tech_type;
    interface_ip.data_arr_peri_global_tech_type = tech_type;
    interface_ip.tag_arr_ram_cell_tech_type     = tech_type;
    interface_ip.tag_arr_peri_global_tech_type  = tech_type;
    interface_ip.wire_is_mat_type = mat_type;
    interface_ip.wire_os_mat_type = mat_type;

    num_children = xml_data->nChildNode("stat");
    for (i = 0; i < num_children; i++) {
        XMLNode* statNode = xml_data->getChildNodePtr("stat", &i);
        XMLCSTR node_name = statNode->getAttribute("name");
        XMLCSTR value = statNode->getAttribute("value");

        if (!node_name)
            warnMissingStatName(statNode->getAttribute("id"));

        ASSIGN_FP_IF("duty_cycle", mcs.duty_cycle);
        ASSIGN_FP_IF("perc_load", mcs.perc_load);
        ASSIGN_FP_IF("memory_reads", mcs.reads);
        ASSIGN_INT_IF("memory_writes", mcs.writes);

        else {
            warnUnrecognizedStat(node_name);
        }
    }

    // Add ECC overhead
    mcp.llcBlockSize = int(ceil(mcp.llc_line_length / BITS_PER_BYTE)) +
        mcp.llc_line_length;
    mcp.dataBusWidth = int(ceil(mcp.databus_width / BITS_PER_BYTE)) +
        mcp.databus_width;
}

MCFrontEnd ::~MCFrontEnd() {

    if (MC_arb) {
        delete MC_arb;
        MC_arb = NULL;
    }
    if (frontendBuffer) {
        delete frontendBuffer;
        frontendBuffer = NULL;
    }
    if (readBuffer) {
        delete readBuffer;
        readBuffer = NULL;
    }
    if (writeBuffer) {
        delete writeBuffer;
        writeBuffer = NULL;
    }
}

MemoryController::~MemoryController() {
    // TODO: use default constructor to delete children
}

