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

#include "model/std_cells/CellMacros.h"

#include <cmath>
#include <vector>

#include "model/std_cells/StdCell.h"
#include "model/timing_graph/ElectricalNet.h"
#include "model/timing_graph/ElectricalDriver.h"
#include "model/timing_graph/ElectricalLoad.h"

namespace DSENT
{
    //-------------------------------------------------------------------------
    // NOR2 Macro (TODO: Generalize to N-input macro once leakage calc is done)
    //-------------------------------------------------------------------------
    void CellMacros::addNor2(StdCell* cell_, const String& name_, 
        bool sizable_, bool a1_to_zn_path_, bool a2_to_zn_path_,
        const String& a1_net_, const String& a2_net_, const String& zn_net_)
    {        
        //Create electrical timing model for the nand
        // Construct loads and drivers
        cell_->createLoad(name_ + "_CgA1");
        cell_->createLoad(name_ + "_CgA2");
        cell_->createLoad(name_ + "_CdZN");
        cell_->createDriver(name_ + "_RonZN", sizable_);
        
        //Get references to loads and drivers
        ElectricalLoad* gate_a1_load = cell_->getLoad(name_ + "_CgA1");
        ElectricalLoad* gate_a2_load = cell_->getLoad(name_ + "_CgA2");        
        ElectricalLoad* drain_load = cell_->getLoad(name_ + "_CdZN");
        ElectricalDriver* zn_drive = cell_->getDriver(name_ + "_RonZN");        
        ElectricalNet* a1_net = cell_->getNet(a1_net_);
        ElectricalNet* a2_net = cell_->getNet(a2_net_);
        ElectricalNet* zn_net = cell_->getNet(zn_net_);        

        //Add loads and drivers to the specified nets
        a1_net->addDownstreamNode(gate_a1_load);
        a2_net->addDownstreamNode(gate_a2_load);
        zn_net->addDownstreamNode(drain_load);
        if (a1_to_zn_path_) gate_a1_load->addDownstreamNode(zn_drive);
        if (a2_to_zn_path_) gate_a2_load->addDownstreamNode(zn_drive);
        zn_drive->addDownstreamNode(zn_net);
        
        return;
    }

    void CellMacros::updateNor2(StdCell* cell_, const String& name_, double normalized_size_)
    {        
        ASSERT(normalized_size_ >= 0.0, "[Error] " + cell_->getInstanceName() + 
            " -> Cannot update a macro with a negative normalized size!");

        //Grab pointer to tech model
        const TechModel* tech = cell_->getTechModel();
        
        // Get technology parameters
        double vdd = tech->get("Vdd");
        double gate_cap = tech->get("Gate->CapPerWidth");
        double drain_cap = tech->get("Drain->CapPerWidth");
        double nmos_eff_res = tech->get("Nmos->EffResWidth");
        double pmos_eff_res = tech->get("Pmos->EffResWidth");
        double pmos_eff_res_stack_ratio = tech->get("Pmos->EffResStackRatio");
        double gate_pitch_contacted = tech->get("Gate->PitchContacted");
        double metal1_wire_min_width = tech->get("Wire->Metal1->MinWidth");
        
        //Calculate number of folds and gate pitches needed
        unsigned int folds = (normalized_size_ < 1.0) ? 1 : (unsigned int)ceil(normalized_size_);
        cell_->getGenProperties()->set(name_ + "_GatePitches", 2 * folds);

        //Calculate widths, making sure they are above the minimum width
        double nmos_width = std::max(calculateNmosWidth(cell_, 1, 2, 1) * normalized_size_ / folds, (double) tech->get("Gate->MinWidth"));
        double pmos_width = std::max(calculatePmosWidth(cell_, 1, 2, 2) * normalized_size_ / folds, (double) tech->get("Gate->MinWidth"));

        //Calculate leakage power for each given input state
        double leakage_power_00 = vdd * folds * 2 * tech->calculateNmosLeakageCurrent(1, nmos_width, 0x0);
        double leakage_power_01 = vdd * folds * tech->calculatePmosLeakageCurrent(2, pmos_width, ~0x1);
        double leakage_power_10 = vdd * folds * tech->calculatePmosLeakageCurrent(2, pmos_width, ~0x2);
        double leakage_power_11 = vdd * folds * tech->calculatePmosLeakageCurrent(2, pmos_width, ~0x3);                        
        cell_->getGenProperties()->set(name_ + "_LeakagePower_00", leakage_power_00);
        cell_->getGenProperties()->set(name_ + "_LeakagePower_01", leakage_power_01);
        cell_->getGenProperties()->set(name_ + "_LeakagePower_10", leakage_power_10);
        cell_->getGenProperties()->set(name_ + "_LeakagePower_11", leakage_power_11);
        
        //Calculate R_on and capacitances
        double pmos_stack2_balance = 1.0 + pmos_eff_res_stack_ratio;
        double c_g = (nmos_width + pmos_width) * gate_cap * folds;
        double c_d = (2 * pmos_width + 2 * nmos_width) * drain_cap * folds;
        double r_on = (nmos_eff_res / nmos_width + pmos_stack2_balance * pmos_eff_res / pmos_width) / (folds * 2.0);
        
        // Estimate the wire cap and add them all at the output
        double cell_height = cell_->getTotalHeight();
        double wire_width = metal1_wire_min_width;
        double wire_spacing = gate_pitch_contacted - metal1_wire_min_width;
        double wire_length = 2.0 * folds * cell_height;
        double wire_cap = tech->calculateWireCapacitance("Metal1", wire_width, wire_spacing, wire_length);

        // Construct equivalent load and drive strength  
        cell_->getLoad(name_ + "_CgA1")->setLoadCap(c_g);
        cell_->getLoad(name_ + "_CgA2")->setLoadCap(c_g);
        cell_->getLoad(name_ + "_CdZN")->setLoadCap(c_d + wire_cap);
        cell_->getDriver(name_ + "_RonZN")->setOutputRes(r_on);
        
        // Calculate flip energies
        double zn_flip_energy = 0.5 * (c_d + wire_cap) * vdd * vdd;
        double a1_flip_energy = 0.5 * c_g * vdd * vdd;
        double a2_flip_energy = 0.5 * c_g * vdd * vdd;
        cell_->getGenProperties()->set(name_ + "_ZN_Flip", zn_flip_energy);
        cell_->getGenProperties()->set(name_ + "_A1_Flip", a1_flip_energy);
        cell_->getGenProperties()->set(name_ + "_A2_Flip", a2_flip_energy);        
    }
    //-------------------------------------------------------------------------

    //-------------------------------------------------------------------------
    // NAND2 Macro (TODO: Generalize to N-input macro once leakage calc is done)
    //-------------------------------------------------------------------------
    //Adds a NAND2 to the standard cell, normalized to some size
    void CellMacros::addNand2(StdCell* cell_, const String& name_,
        bool sizable_, bool a1_to_zn_path_, bool a2_to_zn_path_,
        const String& a1_net_, const String& a2_net_, const String& zn_net_)
    {        
        //Create electrical timing model for the nor
        // Construct loads and drivers
        cell_->createLoad(name_ + "_CgA1");
        cell_->createLoad(name_ + "_CgA2");
        cell_->createLoad(name_ + "_CdZN");
        cell_->createDriver(name_ + "_RonZN", sizable_);    

        //Get references to loads and drivers
        ElectricalLoad* gate_a1_load = cell_->getLoad(name_ + "_CgA1");
        ElectricalLoad* gate_a2_load = cell_->getLoad(name_ + "_CgA2");        
        ElectricalLoad* drain_load = cell_->getLoad(name_ + "_CdZN");        
        ElectricalDriver* zn_drive = cell_->getDriver(name_ + "_RonZN");        
        ElectricalNet* a1_net = cell_->getNet(a1_net_);
        ElectricalNet* a2_net = cell_->getNet(a2_net_);
        ElectricalNet* zn_net = cell_->getNet(zn_net_);
                
        a1_net->addDownstreamNode(gate_a1_load);
        a2_net->addDownstreamNode(gate_a2_load);
        zn_net->addDownstreamNode(drain_load);
        if (a1_to_zn_path_) gate_a1_load->addDownstreamNode(zn_drive);
        if (a2_to_zn_path_) gate_a2_load->addDownstreamNode(zn_drive);
        zn_drive->addDownstreamNode(zn_net);
        
        return;
    }

    //Updates a NAND2 to to the standard cell, normalized to some size
    void CellMacros::updateNand2(StdCell* cell_, const String& name_, double normalized_size_)
    {
        ASSERT(normalized_size_ >= 0.0, "[Error] " + cell_->getInstanceName() + 
            " -> Cannot update a macro with a negative normalized size!");

        //Grab pointer to tech model
        const TechModel* tech = cell_->getTechModel();
        
        // Get technology parameters
        double vdd = tech->get("Vdd");
        double gate_cap = tech->get("Gate->CapPerWidth");
        double drain_cap = tech->get("Drain->CapPerWidth");
        double nmos_eff_res = tech->get("Nmos->EffResWidth");
        double pmos_eff_res = tech->get("Pmos->EffResWidth");
        double nmos_eff_res_stack_ratio = tech->get("Nmos->EffResStackRatio");
        double gate_pitch_contacted = tech->get("Gate->PitchContacted");
        double metal1_wire_min_width = tech->get("Wire->Metal1->MinWidth");

        //Calculate number of folds needed
        unsigned int folds = (normalized_size_ < 1.0) ? 1 : (unsigned int)ceil(normalized_size_);
        cell_->getGenProperties()->set(name_ + "_GatePitches", 2 * folds);

        //Calculate widths, making sure they are above the minimum width
        double nmos_width = std::max(calculateNmosWidth(cell_, 2, 1, 2) * normalized_size_ / folds, (double) tech->get("Gate->MinWidth"));
        double pmos_width = std::max(calculatePmosWidth(cell_, 2, 1, 1) * normalized_size_ / folds, (double) tech->get("Gate->MinWidth"));

        // Leakage power calculation
        double leakage_power_00 = vdd * folds * tech->calculateNmosLeakageCurrent(2, nmos_width, 0x0);
        double leakage_power_01 = vdd * folds * tech->calculateNmosLeakageCurrent(2, nmos_width, 0x1);
        double leakage_power_10 = vdd * folds * tech->calculateNmosLeakageCurrent(2, nmos_width, 0x2);
        double leakage_power_11 = vdd * folds * 2 * tech->calculatePmosLeakageCurrent(1, pmos_width, ~0x3);                        
        cell_->getGenProperties()->set(name_ + "_LeakagePower_00", leakage_power_00);
        cell_->getGenProperties()->set(name_ + "_LeakagePower_01", leakage_power_01);
        cell_->getGenProperties()->set(name_ + "_LeakagePower_10", leakage_power_10);
        cell_->getGenProperties()->set(name_ + "_LeakagePower_11", leakage_power_11);

        // Get input parameters
        double nmos_stack2_balance = 1.0 + nmos_eff_res_stack_ratio;
                
        //Calculate caps
        double c_g = (nmos_width + pmos_width) * gate_cap * folds;
        double c_d = (2 * pmos_width + 2 * nmos_width) * drain_cap * folds;
        double r_on = (nmos_stack2_balance * nmos_eff_res / nmos_width + pmos_eff_res / pmos_width) / (folds * 2.0);
                
        // Estimate the wire cap and add them all at the output
        double cell_height = cell_->getTotalHeight();
        double wire_width = metal1_wire_min_width;
        double wire_spacing = gate_pitch_contacted - metal1_wire_min_width;
        double wire_length = 2.0 * folds * cell_height;
        double wire_cap = tech->calculateWireCapacitance("Metal1", wire_width, wire_spacing, wire_length);

        // Construct equivalent load and drive strength  
        cell_->getLoad(name_ + "_CgA1")->setLoadCap(c_g);
        cell_->getLoad(name_ + "_CgA2")->setLoadCap(c_g);
        cell_->getLoad(name_ + "_CdZN")->setLoadCap(c_d + wire_cap);
        cell_->getDriver(name_ + "_RonZN")->setOutputRes(r_on);
        
        // Calculate flip energies
        double zn_flip_energy = 0.5 * (c_d + wire_cap) * vdd * vdd;
        double a1_flip_energy = 0.5 * c_g * vdd * vdd;
        double a2_flip_energy = 0.5 * c_g * vdd * vdd;
        cell_->getGenProperties()->set(name_ + "_ZN_Flip", zn_flip_energy);
        cell_->getGenProperties()->set(name_ + "_A1_Flip", a1_flip_energy);
        cell_->getGenProperties()->set(name_ + "_A2_Flip", a2_flip_energy);
    }
    //-------------------------------------------------------------------------

    //-------------------------------------------------------------------------
    // INV Macro
    //-------------------------------------------------------------------------
    //Adds an inverter to the model, normalized to some size
    void CellMacros::addInverter(StdCell* cell_, const String& name_, 
        bool sizable_, bool a_to_zn_path_,
        const String& a_net_, const String& zn_net_)
    {
        //Create electrical timing model for the inverter
        // Construct loads and drivers
        cell_->createLoad(name_ + "_CgA");
        cell_->createLoad(name_ + "_CdZN");
        cell_->createDriver(name_ + "_RonZN", sizable_);
        
        //Get references to loads and drivers
        ElectricalLoad* gate_load = cell_->getLoad(name_ + "_CgA");
        ElectricalLoad* drain_load = cell_->getLoad(name_ + "_CdZN");
        ElectricalDriver* out_drive = cell_->getDriver(name_ + "_RonZN");
        ElectricalNet* a_net = cell_->getNet(a_net_);
        ElectricalNet* zn_net = cell_->getNet(zn_net_);
        
        // Setup connectivity of loads and drivers
        a_net->addDownstreamNode(gate_load);
        if (a_to_zn_path_) gate_load->addDownstreamNode(out_drive);
        zn_net->addDownstreamNode(drain_load);
        out_drive->addDownstreamNode(zn_net);

        return;
    }
    
    //Updates the numbers of an inverter for some normalized size
    void CellMacros::updateInverter(StdCell* cell_, const String& name_, double normalized_size_)
    {
        ASSERT(normalized_size_ >= 0.0, "[Error] " + cell_->getInstanceName() + 
            " -> Cannot update a macro with a negative normalized size!");

        //Grab pointer to tech model
        const TechModel* tech = cell_->getTechModel();
        
        //Get values from technology library
        double vdd = tech->get("Vdd");
        double gate_cap = tech->get("Gate->CapPerWidth");
        double drain_cap = tech->get("Drain->CapPerWidth");
        double nmos_eff_res = tech->get("Nmos->EffResWidth");
        double pmos_eff_res = tech->get("Pmos->EffResWidth");
        double gate_pitch_contacted = tech->get("Gate->PitchContacted");
        double metal1_wire_min_width = tech->get("Wire->Metal1->MinWidth");

        //Calculate number of folds needed
        unsigned int folds = (normalized_size_ < 1.0) ? 1 : (unsigned int)ceil(normalized_size_);
        cell_->getGenProperties()->set(name_ + "_GatePitches", folds);

        //Calculate widths, making sure they are above the minimum width
        double nmos_width = std::max(calculateNmosWidth(cell_, 1, 1, 1) * normalized_size_ / folds, (double) tech->get("Gate->MinWidth"));
        double pmos_width = std::max(calculatePmosWidth(cell_, 1, 1, 1) * normalized_size_ / folds, (double) tech->get("Gate->MinWidth"));

        //Calculate leakage power for each given input state
        double leakage_power_0 = vdd * folds * tech->calculateNmosLeakageCurrent(1, nmos_width, 0x0);
        double leakage_power_1 = vdd * folds * tech->calculatePmosLeakageCurrent(1, pmos_width, ~0x1);
        cell_->getGenProperties()->set(name_ + "_LeakagePower_0", leakage_power_0);
        cell_->getGenProperties()->set(name_ + "_LeakagePower_1", leakage_power_1);

        //Calculate caps
        double c_g = (nmos_width + pmos_width) * gate_cap * folds;
        double c_d = (pmos_width + nmos_width) * drain_cap * folds;
        double r_on = (nmos_eff_res / nmos_width + pmos_eff_res / pmos_width) / (folds * 2.0);
        
        // Estimate the wire cap and add them all at the output
        double cell_height = cell_->getTotalHeight();
        double wire_width = metal1_wire_min_width;
        double wire_spacing = gate_pitch_contacted - metal1_wire_min_width;
        double wire_length = folds * cell_height;
        double wire_cap = tech->calculateWireCapacitance("Metal1", wire_width, wire_spacing, wire_length);

        // Construct equivalent load and drive strength  
        cell_->getLoad(name_ + "_CgA")->setLoadCap(c_g);
        cell_->getLoad(name_ + "_CdZN")->setLoadCap(c_d + wire_cap);
        cell_->getDriver(name_ + "_RonZN")->setOutputRes(r_on);
        
        // Calculate flip energy (output flip)
        // Calculate flip energies
        double zn_flip_energy = 0.5 * (c_d + wire_cap) * vdd * vdd;
        double a_flip_energy = 0.5 * c_g * vdd * vdd;
        cell_->getGenProperties()->set(name_ + "_ZN_Flip", zn_flip_energy);
        cell_->getGenProperties()->set(name_ + "_A_Flip", a_flip_energy);

        return;
    }    
    //-------------------------------------------------------------------------

    //-------------------------------------------------------------------------
    // INVZ Macro
    //-------------------------------------------------------------------------
    //Adds a tristated inverter to the model, normalized to some size
    void CellMacros::addTristate(StdCell* cell_, const String& name_, 
        bool sizable_, bool a_to_zn_path_, bool oe_to_zn_path_, bool oen_to_zn_path_,
        const String& a_net_, const String& oe_net_, const String& oen_net_, const String& zn_net_)
    {
        // Construct loads and drivers
        cell_->createLoad(name_ + "_CgA");
        cell_->createLoad(name_ + "_CgOE");
        cell_->createLoad(name_ + "_CgOEN");
        cell_->createLoad(name_ + "_CdZN");
        cell_->createDriver(name_ + "_RonZN", sizable_);
        
        // Get references to loads, nets and drivers
        ElectricalLoad* gate_a_load = cell_->getLoad(name_ + "_CgA");
        ElectricalLoad* gate_oe_load = cell_->getLoad(name_ + "_CgOE");
        ElectricalLoad* gate_oen_load = cell_->getLoad(name_ + "_CgOEN");
        ElectricalLoad* drain_load = cell_->getLoad(name_ + "_CdZN");
        ElectricalDriver* out_drive = cell_->getDriver(name_ + "_RonZN");
        ElectricalNet* a_net = cell_->getNet(a_net_);
        ElectricalNet* oe_net = cell_->getNet(oe_net_);
        ElectricalNet* oen_net = cell_->getNet(oen_net_);
        ElectricalNet* zn_net = cell_->getNet(zn_net_);
                
        // Setup connectivity of loads and drivers
        a_net->addDownstreamNode(gate_a_load);
        oe_net->addDownstreamNode(gate_oe_load);
        oen_net->addDownstreamNode(gate_oen_load);        
        if (a_to_zn_path_) gate_a_load->addDownstreamNode(out_drive);
        if (oe_to_zn_path_) gate_oe_load->addDownstreamNode(out_drive);
        if (oen_to_zn_path_) gate_oen_load->addDownstreamNode(out_drive);                
        zn_net->addDownstreamNode(drain_load);        
        out_drive->addDownstreamNode(zn_net);

        return;
    }
    
    //Updates the numbers of an inverter for some normalized size
    void CellMacros::updateTristate(StdCell* cell_, const String& name_, double normalized_size_)
    {
        ASSERT(normalized_size_ >= 0.0, "[Error] " + cell_->getInstanceName() + 
            " -> Cannot update a macro with a negative normalized size!");

        //Grab pointer to tech model
        const TechModel* tech = cell_->getTechModel();
        
        //Get values from technology library
        double vdd = tech->get("Vdd");
        double gate_cap = tech->get("Gate->CapPerWidth");
        double drain_cap = tech->get("Drain->CapPerWidth");
        double nmos_eff_res = tech->get("Nmos->EffResWidth");
        double pmos_eff_res = tech->get("Pmos->EffResWidth");
        double pmos_eff_res_stack_ratio = tech->get("Pmos->EffResStackRatio");
        double nmos_eff_res_stack_ratio = tech->get("Nmos->EffResStackRatio");
        double gate_pitch_contacted = tech->get("Gate->PitchContacted");
        double metal1_wire_min_width = tech->get("Wire->Metal1->MinWidth");

        //Calculate number of folds and gate pitches needed
        unsigned int folds = (normalized_size_ < 1.0) ? 1 : (unsigned int)ceil(normalized_size_);
        cell_->getGenProperties()->set(name_ + "_GatePitches", 2 * folds);

        //Calculate widths, making sure they are above the minimum width
        double nmos_width = std::max(calculateNmosWidth(cell_, 2, 2, 2) * normalized_size_ / folds, (double) tech->get("Gate->MinWidth"));
        double pmos_width = std::max(calculatePmosWidth(cell_, 2, 2, 2) * normalized_size_ / folds, (double) tech->get("Gate->MinWidth"));

        //Calculate leakage power for each given input state
        //if output_enable = 0, then it is possible that the PMOS may leak (if output = 0),
        //or the NMOS will leak (if output = 1)
        
        //OE OEN A _ ZN
        double leakage_power_010_0 = vdd * folds * tech->calculatePmosLeakageCurrent(2, pmos_width, ~0x2);
        double leakage_power_010_1 = vdd * folds * tech->calculateNmosLeakageCurrent(2, nmos_width, 0x0);
        double leakage_power_011_0 = vdd * folds * tech->calculatePmosLeakageCurrent(2, pmos_width, ~0x3);
        double leakage_power_011_1 = vdd * folds * tech->calculateNmosLeakageCurrent(2, nmos_width, 0x1);
        double leakage_power_100_1 = vdd * folds * tech->calculateNmosLeakageCurrent(2, nmos_width, 0x2);
        double leakage_power_101_0 = vdd * folds * tech->calculatePmosLeakageCurrent(2, pmos_width, ~0x1);        
        cell_->getGenProperties()->set(name_ + "_LeakagePower_010_0", leakage_power_010_0);
        cell_->getGenProperties()->set(name_ + "_LeakagePower_010_1", leakage_power_010_1);
        cell_->getGenProperties()->set(name_ + "_LeakagePower_011_0", leakage_power_011_0);
        cell_->getGenProperties()->set(name_ + "_LeakagePower_011_1", leakage_power_011_1);
        cell_->getGenProperties()->set(name_ + "_LeakagePower_100_1", leakage_power_100_1);
        cell_->getGenProperties()->set(name_ + "_LeakagePower_101_0", leakage_power_101_0);
               
        //Caculate stack balance
        double pmos_stack2_balance = 1.0 + pmos_eff_res_stack_ratio;
        double nmos_stack2_balance = 1.0 + nmos_eff_res_stack_ratio;

        //Calculate caps
        double c_g_a = (nmos_width + pmos_width) * gate_cap * folds;
        double c_g_oe = nmos_width * gate_cap * folds;
        double c_g_oen = pmos_width * gate_cap * folds;        
        double c_d = (2 * pmos_width + 2 * nmos_width) * drain_cap * folds;
        double r_on = (nmos_stack2_balance * nmos_eff_res / nmos_width + pmos_stack2_balance * pmos_eff_res / pmos_width) / (folds * 2.0);        
        
        // Estimate the wire cap and add them all at the output
        double cell_height = cell_->getTotalHeight();
        double wire_width = metal1_wire_min_width;
        double wire_spacing = gate_pitch_contacted - metal1_wire_min_width;
        double wire_length = 2.0 * folds * cell_height;
        double wire_cap = tech->calculateWireCapacitance("Metal1", wire_width, wire_spacing, wire_length);

        // Construct equivalent load and drive strength  
        cell_->getLoad(name_ + "_CgA")->setLoadCap(c_g_a);
        cell_->getLoad(name_ + "_CgOE")->setLoadCap(c_g_oe);
        cell_->getLoad(name_ + "_CgOEN")->setLoadCap(c_g_oen);
        cell_->getLoad(name_ + "_CdZN")->setLoadCap(c_d + wire_cap);
        cell_->getDriver(name_ + "_RonZN")->setOutputRes(r_on);
        
        // Calculate flip energy (output flip)
        double zn_flip_energy = 0.5 * (c_d + wire_cap) * vdd * vdd;
        double a_flip_energy = 0.5 * c_g_a * vdd * vdd;
        double oe_flip_energy = 0.5 * c_g_oe * vdd * vdd;
        double oen_flip_energy = 0.5 * c_g_oen * vdd * vdd;
        cell_->getGenProperties()->set(name_ + "_ZN_Flip", zn_flip_energy);
        cell_->getGenProperties()->set(name_ + "_A_Flip", a_flip_energy);
        cell_->getGenProperties()->set(name_ + "_OE_Flip", oe_flip_energy);
        cell_->getGenProperties()->set(name_ + "_OEN_Flip", oen_flip_energy);
        return;
    }    
    //-------------------------------------------------------------------------

    
    //-------------------------------------------------------------------------
    // Helper Functions
    //-------------------------------------------------------------------------
    //Returns the width of NMOS transistors, given the NMOS and PMOS stacking
    double CellMacros::calculateNmosWidth(const StdCell* cell_, unsigned int max_stacked_nmos_, unsigned int max_stacked_pmos_, unsigned int current_stacked_nmos_)
    {
        //Grab pointer to tech model
        const TechModel* tech = cell_->getTechModel();

        double nmos_eff_res_stack_ratio = tech->get("Nmos->EffResStackRatio");
        double pmos_eff_res_stack_ratio = tech->get("Pmos->EffResStackRatio");

        double nmos_stack_balance = 1.0 + nmos_eff_res_stack_ratio * (double) (max_stacked_nmos_ - 1);
        double pmos_stack_balance = 1.0 + pmos_eff_res_stack_ratio * (double) (max_stacked_pmos_ - 1);
        double current_nmos_stack_balance = 1.0 + nmos_eff_res_stack_ratio * (double) (current_stacked_nmos_ - 1);
        
        double pn_ratio = cell_->getPToNRatio();
        double active_height = cell_->getActiveHeight();
        
        //Calculate the width of the current device
        double nmos_width = active_height * current_nmos_stack_balance / (nmos_stack_balance + pn_ratio * pmos_stack_balance);

        return nmos_width;
    }
    
    //Returns the width of PMOS transistors, given the NMOS and PMOS stacking
    double CellMacros::calculatePmosWidth(const StdCell* cell_, unsigned int max_stacked_nmos_, unsigned int max_stacked_pmos_, unsigned int current_stacked_pmos_)
    {
        //Grab pointer to tech model
        const TechModel* tech = cell_->getTechModel();

        double nmos_eff_res_stack_ratio = tech->get("Nmos->EffResStackRatio");
        double pmos_eff_res_stack_ratio = tech->get("Pmos->EffResStackRatio");

        double nmos_stack_balance = 1.0 + nmos_eff_res_stack_ratio * (double) (max_stacked_nmos_ - 1);
        double pmos_stack_balance = 1.0 + pmos_eff_res_stack_ratio * (double) (max_stacked_pmos_ - 1);
        double current_pmos_stack_balance = 1.0 + pmos_eff_res_stack_ratio * (double) (current_stacked_pmos_ - 1);
        
        double pn_ratio = cell_->getPToNRatio();
        double active_height = cell_->getActiveHeight();
        
        //Calculate the width of the current device
        double pmos_width = active_height * current_pmos_stack_balance * pn_ratio / (nmos_stack_balance + pn_ratio * pmos_stack_balance);

        return pmos_width;    
    }
    //-------------------------------------------------------------------------

} // namespace DSENT

