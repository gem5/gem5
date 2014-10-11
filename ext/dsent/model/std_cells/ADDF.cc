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

#include "model/std_cells/ADDF.h"

#include <cmath>

#include "model/PortInfo.h"
#include "model/EventInfo.h"
#include "model/TransitionInfo.h"
#include "model/std_cells/StdCellLib.h"
#include "model/std_cells/CellMacros.h"
#include "model/timing_graph/ElectricalNet.h"
#include "model/timing_graph/ElectricalDriver.h"
#include "model/timing_graph/ElectricalLoad.h"
#include "model/timing_graph/ElectricalDelay.h"

namespace DSENT
{
    using std::ceil;
    using std::max;

    ADDF::ADDF(const String& instance_name_, const TechModel* tech_model_)
        : StdCell(instance_name_, tech_model_)
    {
        initParameters();
        initProperties();
    }

    ADDF::~ADDF()
    {}

    void ADDF::initProperties()
    {
        return;
    }

    void ADDF::constructModel()
    {
        // All constructModel should do is create Area/NDDPower/Energy Results as
        // well as instantiate any sub-instances using only the hard parameters
        
        createInputPort("A");
        createInputPort("B");
        createInputPort("CI");
        createOutputPort("S");
        createOutputPort("CO");
        
        createLoad("A_Cap");
        createLoad("B_Cap");
        createLoad("CI_Cap");
        createDelay("A_to_S_delay");
        createDelay("B_to_S_delay");
        createDelay("CI_to_S_delay");
        createDelay("A_to_CO_delay");
        createDelay("B_to_CO_delay");
        createDelay("CI_to_CO_delay");
        createDriver("S_Ron", true);
        createDriver("CO_Ron", true);
                
        ElectricalLoad* a_cap = getLoad("A_Cap");
        ElectricalLoad* b_cap = getLoad("B_Cap");
        ElectricalLoad* ci_cap = getLoad("CI_Cap");
        ElectricalDelay* a_to_s_delay = getDelay("A_to_S_delay");
        ElectricalDelay* b_to_s_delay = getDelay("B_to_S_delay");
        ElectricalDelay* ci_to_s_delay = getDelay("CI_to_S_delay");
        ElectricalDelay* a_to_co_delay = getDelay("A_to_CO_delay");
        ElectricalDelay* b_to_co_delay = getDelay("B_to_CO_delay");
        ElectricalDelay* ci_to_co_delay = getDelay("CI_to_CO_delay");
        ElectricalDriver* s_ron = getDriver("S_Ron");
        ElectricalDriver* co_ron = getDriver("CO_Ron");
        
        getNet("A")->addDownstreamNode(a_cap);
        getNet("B")->addDownstreamNode(b_cap);
        getNet("CI")->addDownstreamNode(ci_cap);
        a_cap->addDownstreamNode(a_to_s_delay);        
        b_cap->addDownstreamNode(b_to_s_delay);        
        ci_cap->addDownstreamNode(ci_to_s_delay);        
        a_cap->addDownstreamNode(a_to_co_delay);        
        b_cap->addDownstreamNode(b_to_co_delay);        
        ci_cap->addDownstreamNode(ci_to_co_delay);        
        
        a_to_s_delay->addDownstreamNode(s_ron);
        b_to_s_delay->addDownstreamNode(s_ron);
        ci_to_s_delay->addDownstreamNode(s_ron);
        a_to_co_delay->addDownstreamNode(co_ron);
        b_to_co_delay->addDownstreamNode(co_ron);
        ci_to_co_delay->addDownstreamNode(co_ron);
        
        s_ron->addDownstreamNode(getNet("S"));
        co_ron->addDownstreamNode(getNet("CO"));
        
        // Create Area result
        // Create NDD Power result
        createElectricalAtomicResults();
        // Create ADDF Event Energy Result
        createElectricalEventAtomicResult("ADDF");

        getEventInfo("Idle")->setStaticTransitionInfos();
        
        return;
    }
    
    void ADDF::updateModel()
    {
        // Get parameters
        double drive_strength = getDrivingStrength();
        Map<double>* cache = getTechModel()->getStdCellLib()->getStdCellCache();

        // Standard cell cache string
        String cell_name = "ADDF_X" + (String) drive_strength;
        
        // Get timing parameters
        getLoad("A_Cap")->setLoadCap(cache->get(cell_name + "->Cap->A"));
        getLoad("B_Cap")->setLoadCap(cache->get(cell_name + "->Cap->B"));
        getLoad("CI_Cap")->setLoadCap(cache->get(cell_name + "->Cap->CI"));

        getDelay("A_to_S_delay")->setDelay(cache->get(cell_name + "->Delay->A_to_S"));
        getDelay("B_to_S_delay")->setDelay(cache->get(cell_name + "->Delay->B_to_S"));
        getDelay("CI_to_S_delay")->setDelay(cache->get(cell_name + "->Delay->CI_to_S"));
        getDelay("A_to_CO_delay")->setDelay(cache->get(cell_name + "->Delay->A_to_CO"));
        getDelay("B_to_CO_delay")->setDelay(cache->get(cell_name + "->Delay->B_to_CO"));
        getDelay("CI_to_CO_delay")->setDelay(cache->get(cell_name + "->Delay->CI_to_CO"));
        
        getDriver("S_Ron")->setOutputRes(cache->get(cell_name + "->DriveRes->S"));
        getDriver("CO_Ron")->setOutputRes(cache->get(cell_name + "->DriveRes->CO"));        
        
        // Set the cell area
        getAreaResult("Active")->setValue(cache->get(cell_name + "->Area->Active"));
        getAreaResult("Metal1Wire")->setValue(cache->get(cell_name + "->Area->Metal1Wire"));
        
        return;
    }
    
    void ADDF::evaluateModel()
    {
        return;
    }

    void ADDF::useModel()
    {
        // Get parameters
        double drive_strength = getDrivingStrength();
        Map<double>* cache = getTechModel()->getStdCellLib()->getStdCellCache();

        // Standard cell cache string
        String cell_name = "ADDF_X" + (String) drive_strength;

        // Propagate the transition info and get the 0->1 transition count
        propagateTransitionInfo();
        double P_A = getInputPort("A")->getTransitionInfo().getProbability1();
        double P_B = getInputPort("B")->getTransitionInfo().getProbability1();
        double P_CI = getInputPort("CI")->getTransitionInfo().getProbability1();
        double A_num_trans_01 = getInputPort("A")->getTransitionInfo().getNumberTransitions01();
        double B_num_trans_01 = getInputPort("B")->getTransitionInfo().getNumberTransitions01();
        double CI_num_trans_01 = getInputPort("CI")->getTransitionInfo().getNumberTransitions01();
        double P_num_trans_01 = m_trans_P_.getNumberTransitions01();
        double G_num_trans_01 = m_trans_G_.getNumberTransitions01();
        double CP_num_trans_01 = m_trans_CP_.getNumberTransitions01();
        double S_num_trans_01 = getOutputPort("S")->getTransitionInfo().getNumberTransitions01();
        double CO_num_trans_01 = getOutputPort("CO")->getTransitionInfo().getNumberTransitions01();

        // Calculate leakage
        double leakage = 0;
        leakage += cache->get(cell_name + "->Leakage->!A!B!CI") * (1 - P_A) * (1 - P_B) * (1 - P_CI);
        leakage += cache->get(cell_name + "->Leakage->!A!BCI") * (1 - P_A) * (1 - P_B) * P_CI;
        leakage += cache->get(cell_name + "->Leakage->!AB!CI") * (1 - P_A) * P_B * (1 - P_CI);
        leakage += cache->get(cell_name + "->Leakage->!ABCI") * (1 - P_A) * P_B * P_CI;
        leakage += cache->get(cell_name + "->Leakage->A!B!CI") * P_A * (1 - P_B) * (1 - P_CI);
        leakage += cache->get(cell_name + "->Leakage->A!BCI") * P_A * (1 - P_B) * P_CI;
        leakage += cache->get(cell_name + "->Leakage->AB!CI") * P_A * P_B * (1 - P_CI);
        leakage += cache->get(cell_name + "->Leakage->ABCI") * P_A * P_B * P_CI;
        getNddPowerResult("Leakage")->setValue(leakage);

        // Get VDD
        double vdd = getTechModel()->get("Vdd");
    
        // Get capacitances
        double a_b_cap = cache->get(cell_name + "->Cap->A_b");
        double b_b_cap = cache->get(cell_name + "->Cap->B_b");
        double ci_b_cap = cache->get(cell_name + "->Cap->CI_b");
        double p_cap = cache->get(cell_name + "->Cap->P");
        double p_b_cap = cache->get(cell_name + "->Cap->P_b");
        double s_cap = cache->get(cell_name + "->Cap->S");        
        double cp_cap = cache->get(cell_name + "->Cap->CP");
        double g_cap = cache->get(cell_name + "->Cap->G");
        double co_cap = cache->get(cell_name + "->Cap->CO");
        double s_load_cap = getNet("S")->getTotalDownstreamCap();
        double co_load_cap = getNet("CO")->getTotalDownstreamCap();
        
        // Calculate ADDF Event energy
        double addf_event_energy = 0.0;
        addf_event_energy += a_b_cap * A_num_trans_01;
        addf_event_energy += b_b_cap * B_num_trans_01;
        addf_event_energy += ci_b_cap * CI_num_trans_01;
        addf_event_energy += (p_cap + p_b_cap) * P_num_trans_01; 
        addf_event_energy += (s_cap + s_load_cap) * S_num_trans_01;
        addf_event_energy += cp_cap * CP_num_trans_01;
        addf_event_energy += g_cap * G_num_trans_01;
        addf_event_energy += (co_cap + co_load_cap) * CO_num_trans_01;
        addf_event_energy *= vdd * vdd;
        getEventResult("ADDF")->setValue(addf_event_energy);

        return;
    }

    void ADDF::propagateTransitionInfo()
    {
        const TransitionInfo& trans_A = getInputPort("A")->getTransitionInfo();
        const TransitionInfo& trans_B = getInputPort("B")->getTransitionInfo();
        const TransitionInfo& trans_CI = getInputPort("CI")->getTransitionInfo();

        double max_freq_mult = max(max(trans_A.getFrequencyMultiplier(), trans_B.getFrequencyMultiplier()), trans_CI.getFrequencyMultiplier());
        const TransitionInfo& scaled_trans_A = trans_A.scaleFrequencyMultiplier(max_freq_mult);
        const TransitionInfo& scaled_trans_B = trans_B.scaleFrequencyMultiplier(max_freq_mult);
        const TransitionInfo& scaled_trans_CI = trans_CI.scaleFrequencyMultiplier(max_freq_mult);

        double A_prob_00 = scaled_trans_A.getNumberTransitions00() / max_freq_mult;
        double A_prob_01 = scaled_trans_A.getNumberTransitions01() / max_freq_mult;
        double A_prob_10 = A_prob_01;
        double A_prob_11 = scaled_trans_A.getNumberTransitions11() / max_freq_mult;
        double B_prob_00 = scaled_trans_B.getNumberTransitions00() / max_freq_mult;
        double B_prob_01 = scaled_trans_B.getNumberTransitions01() / max_freq_mult;
        double B_prob_10 = B_prob_01;
        double B_prob_11 = scaled_trans_B.getNumberTransitions11() / max_freq_mult;
        double CI_prob_00 = scaled_trans_CI.getNumberTransitions00() / max_freq_mult;
        double CI_prob_01 = scaled_trans_CI.getNumberTransitions01() / max_freq_mult;
        double CI_prob_10 = CI_prob_01;
        double CI_prob_11 = scaled_trans_CI.getNumberTransitions11() / max_freq_mult;

        // Set P transition info
        double P_prob_00 = A_prob_00 * B_prob_00 + 
                                A_prob_01 * B_prob_01 + 
                                A_prob_10 * B_prob_10 +
                                A_prob_11 * B_prob_11;
        double P_prob_01 = A_prob_00 * B_prob_01 +
                                A_prob_01 * B_prob_00 +
                                A_prob_10 * B_prob_11 + 
                                A_prob_11 * B_prob_10;
        double P_prob_10 = P_prob_01;
        double P_prob_11 = A_prob_00 * B_prob_11 +
                                A_prob_01 * B_prob_10 +
                                A_prob_10 * B_prob_01 +
                                A_prob_11 * B_prob_00;

        // Set G transition info
        double G_prob_00 = A_prob_11 * B_prob_11;
        double G_prob_01 = A_prob_11 * B_prob_10 +
                            A_prob_10 * (B_prob_11 + B_prob_10);
        double G_prob_10 = G_prob_01;
        double G_prob_11 = A_prob_00 +
                            A_prob_01 * (B_prob_00 + B_prob_10) +
                            A_prob_10 * (B_prob_00 + B_prob_01) +
                            A_prob_11 * B_prob_00;

        // Set CP transition info
        double CP_prob_00 = P_prob_11 * CI_prob_11;
        double CP_prob_01 = P_prob_11 * CI_prob_10 +
                            P_prob_10 * (CI_prob_11 + CI_prob_10);
        double CP_prob_10 = CP_prob_01;
        double CP_prob_11 = P_prob_00 +
                            P_prob_01 * (CI_prob_00 + CI_prob_10) +
                            P_prob_10 * (CI_prob_00 + CI_prob_01) +
                            P_prob_11 * CI_prob_00;

        // Set S transition info
        double S_prob_00 = P_prob_00 * CI_prob_00 + 
                                P_prob_01 * CI_prob_01 + 
                                P_prob_10 * CI_prob_10 +
                                P_prob_11 * CI_prob_11;
        double S_prob_01 = P_prob_00 * CI_prob_01 +
                                P_prob_01 * CI_prob_00 +
                                P_prob_10 * CI_prob_11 + 
                                P_prob_11 * CI_prob_10;
        double S_prob_11 = P_prob_00 * CI_prob_11 +
                                P_prob_01 * CI_prob_10 +
                                P_prob_10 * CI_prob_01 +
                                P_prob_11 * CI_prob_00;

        // Set CO transition info
        double CO_prob_00 = G_prob_11 * CP_prob_11;
        double CO_prob_01 = G_prob_11 * CP_prob_10 +
                            G_prob_10 * (CP_prob_11 + CP_prob_10);
        double CO_prob_11 = G_prob_00 +
                            G_prob_01 * (CP_prob_00 + CP_prob_10) +
                            G_prob_10 * (CP_prob_00 + CP_prob_01) +
                            G_prob_11 * CP_prob_00;

        m_trans_P_ = TransitionInfo(P_prob_00 * max_freq_mult, P_prob_01 * max_freq_mult, P_prob_11 * max_freq_mult);
        m_trans_G_ = TransitionInfo(G_prob_00 * max_freq_mult, G_prob_01 * max_freq_mult, G_prob_11 * max_freq_mult);
        m_trans_CP_ = TransitionInfo(CP_prob_00 * max_freq_mult, CP_prob_01 * max_freq_mult, CP_prob_11 * max_freq_mult);

        // Check that probabilities add up to 1.0 with some finite tolerance
        ASSERT(LibUtil::Math::isEqual((S_prob_00 + S_prob_01 + S_prob_01 + S_prob_11), 1.0), 
            "[Error] " + getInstanceName() +  "Output S transition probabilities must add up to 1 (" +
            (String) S_prob_00 + ", " + (String) S_prob_01 + ", " + (String) S_prob_11 + ")!");

        // Check that probabilities add up to 1.0 with some finite tolerance
        ASSERT(LibUtil::Math::isEqual((CO_prob_00 + CO_prob_01 + CO_prob_01 + CO_prob_11), 1.0), 
            "[Error] " + getInstanceName() +  "Output S transition probabilities must add up to 1 (" +
            (String) CO_prob_00 + ", " + (String) CO_prob_01 + ", " + (String) CO_prob_11 + ")!");

        // Turn probability of transitions per cycle into number of transitions per time unit
        TransitionInfo trans_S(S_prob_00 * max_freq_mult, S_prob_01 * max_freq_mult, S_prob_11 * max_freq_mult);
        getOutputPort("S")->setTransitionInfo(trans_S);
        TransitionInfo trans_CO(CO_prob_00 * max_freq_mult, CO_prob_01 * max_freq_mult, CO_prob_11 * max_freq_mult);
        getOutputPort("CO")->setTransitionInfo(trans_CO);
        return;
    }

    // Creates the standard cell, characterizes and abstracts away the details
    void ADDF::cacheStdCell(StdCellLib* cell_lib_, double drive_strength_)
    {
        // Get parameters        
        double gate_pitch = cell_lib_->getTechModel()->get("Gate->PitchContacted");
        Map<double>* cache = cell_lib_->getStdCellCache();

        // Standard cell cache string
        String cell_name = "ADDF_X" + (String) drive_strength_;

        Log::printLine("=== " + cell_name + " ===");
        
        // Now actually build the full standard cell model
        createInputPort("A");
        createInputPort("B");
        createInputPort("CI");
        createOutputPort("S");
        createOutputPort("CO");
        
        createNet("A_b");
        createNet("B_b");
        createNet("CI_b");
        createNet("P");
        createNet("P_b");
        createNet("G");             //actually G_b since it is NAND'ed
        createNet("CP");            //actually (CP)_b since it is NAND'ed
        
        // Adds macros
        CellMacros::addInverter(this, "INV1", false, true, "A", "A_b");
        CellMacros::addInverter(this, "INV2", false, true, "B", "B_b");
        CellMacros::addInverter(this, "INV3", false, true, "CI", "CI_b");
        CellMacros::addInverter(this, "INV4", false, true, "P", "P_b");
        CellMacros::addTristate(this, "INVZ1", false, true, true, true, "B", "A", "A_b", "P");
        CellMacros::addTristate(this, "INVZ2", false, true, true, true, "B_b", "A_b", "A", "P");
        CellMacros::addTristate(this, "INVZ3", true, true, true, true, "P", "CI", "CI_b", "S");
        CellMacros::addTristate(this, "INVZ4", true, true, true, true, "P_b", "CI_b", "CI", "S");
        CellMacros::addNand2(this, "NAND1", false, true, true, "CI", "P", "CP");
        CellMacros::addNand2(this, "NAND2", false, true, true, "A", "B", "G");
        CellMacros::addNand2(this, "NAND3", true, true, true, "CP", "G", "CO");
                
        // I have no idea how to size each of the parts haha
        CellMacros::updateInverter(this, "INV1", drive_strength_ * 0.250);
        CellMacros::updateInverter(this, "INV2", drive_strength_ * 0.250);
        CellMacros::updateInverter(this, "INV3", drive_strength_ * 0.250);
        CellMacros::updateInverter(this, "INV4", drive_strength_ * 0.500);
        CellMacros::updateTristate(this, "INVZ1", drive_strength_ * 0.250);
        CellMacros::updateTristate(this, "INVZ2", drive_strength_ * 0.250);
        CellMacros::updateTristate(this, "INVZ3", drive_strength_ * 0.500);
        CellMacros::updateTristate(this, "INVZ4", drive_strength_ * 0.500);
        CellMacros::updateNand2(this, "NAND1", drive_strength_ * 0.500);
        CellMacros::updateNand2(this, "NAND2", drive_strength_ * 0.500);
        CellMacros::updateNand2(this, "NAND3", drive_strength_ * 1.000);
                        
        // Cache area result
        double area = 0.0;
        area += gate_pitch * getTotalHeight() * 1;
        area += gate_pitch * getTotalHeight() * getGenProperties()->get("INV1_GatePitches").toDouble();
        area += gate_pitch * getTotalHeight() * getGenProperties()->get("INV2_GatePitches").toDouble();
        area += gate_pitch * getTotalHeight() * getGenProperties()->get("INV3_GatePitches").toDouble();
        area += gate_pitch * getTotalHeight() * getGenProperties()->get("INV4_GatePitches").toDouble();
        area += gate_pitch * getTotalHeight() * getGenProperties()->get("INVZ1_GatePitches").toDouble();
        area += gate_pitch * getTotalHeight() * getGenProperties()->get("INVZ2_GatePitches").toDouble();
        area += gate_pitch * getTotalHeight() * getGenProperties()->get("INVZ3_GatePitches").toDouble();
        area += gate_pitch * getTotalHeight() * getGenProperties()->get("INVZ4_GatePitches").toDouble();
        area += gate_pitch * getTotalHeight() * getGenProperties()->get("NAND1_GatePitches").toDouble();
        area += gate_pitch * getTotalHeight() * getGenProperties()->get("NAND2_GatePitches").toDouble();
        area += gate_pitch * getTotalHeight() * getGenProperties()->get("NAND3_GatePitches").toDouble();
        cache->set(cell_name + "->Area->Active", area);
        cache->set(cell_name + "->Area->Metal1Wire", area);
        Log::printLine(cell_name + "->Area->Active=" + (String) area);
        Log::printLine(cell_name + "->Area->Metal1Wire=" + (String) area);

        // --------------------------------------------------------------------
        // Leakage Model Calculation
        // --------------------------------------------------------------------
        // Cache leakage power results (for every single signal combination)
        double leakage_000 = 0;         //!A, !B, !CI
        double leakage_001 = 0;         //!A, !B, CI
        double leakage_010 = 0;         //!A, B, !CI
        double leakage_011 = 0;         //!A, B, CI
        double leakage_100 = 0;         //A, !B, !CI
        double leakage_101 = 0;         //A, !B, CI
        double leakage_110 = 0;         //A, B, !CI
        double leakage_111 = 0;         //A, B, CI

        //This is so painful...
        leakage_000 += getGenProperties()->get("INV1_LeakagePower_0").toDouble();
        leakage_000 += getGenProperties()->get("INV2_LeakagePower_0").toDouble();
        leakage_000 += getGenProperties()->get("INV3_LeakagePower_0").toDouble();
        leakage_000 += getGenProperties()->get("INV4_LeakagePower_0").toDouble();
        leakage_000 += getGenProperties()->get("INVZ1_LeakagePower_010_0").toDouble();
        leakage_000 += getGenProperties()->get("INVZ2_LeakagePower_101_0").toDouble();
        leakage_000 += getGenProperties()->get("INVZ3_LeakagePower_010_0").toDouble();
        leakage_000 += getGenProperties()->get("INVZ4_LeakagePower_101_0").toDouble();
        leakage_000 += getGenProperties()->get("NAND1_LeakagePower_00").toDouble();
        leakage_000 += getGenProperties()->get("NAND2_LeakagePower_00").toDouble();
        leakage_000 += getGenProperties()->get("NAND3_LeakagePower_11").toDouble();

        leakage_001 += getGenProperties()->get("INV1_LeakagePower_0").toDouble();
        leakage_001 += getGenProperties()->get("INV2_LeakagePower_0").toDouble();
        leakage_001 += getGenProperties()->get("INV3_LeakagePower_1").toDouble();
        leakage_001 += getGenProperties()->get("INV4_LeakagePower_0").toDouble();
        leakage_001 += getGenProperties()->get("INVZ1_LeakagePower_010_0").toDouble();
        leakage_001 += getGenProperties()->get("INVZ2_LeakagePower_101_0").toDouble();
        leakage_001 += getGenProperties()->get("INVZ3_LeakagePower_100_1").toDouble();
        leakage_001 += getGenProperties()->get("INVZ4_LeakagePower_011_1").toDouble();
        leakage_001 += getGenProperties()->get("NAND1_LeakagePower_10").toDouble();
        leakage_001 += getGenProperties()->get("NAND2_LeakagePower_00").toDouble();
        leakage_001 += getGenProperties()->get("NAND3_LeakagePower_11").toDouble();

        leakage_010 += getGenProperties()->get("INV1_LeakagePower_0").toDouble();
        leakage_010 += getGenProperties()->get("INV2_LeakagePower_1").toDouble();
        leakage_010 += getGenProperties()->get("INV3_LeakagePower_0").toDouble();
        leakage_010 += getGenProperties()->get("INV4_LeakagePower_1").toDouble();
        leakage_010 += getGenProperties()->get("INVZ1_LeakagePower_011_1").toDouble();
        leakage_010 += getGenProperties()->get("INVZ2_LeakagePower_100_1").toDouble();
        leakage_010 += getGenProperties()->get("INVZ3_LeakagePower_011_1").toDouble();
        leakage_010 += getGenProperties()->get("INVZ4_LeakagePower_100_1").toDouble();
        leakage_010 += getGenProperties()->get("NAND1_LeakagePower_01").toDouble();
        leakage_010 += getGenProperties()->get("NAND2_LeakagePower_01").toDouble();
        leakage_010 += getGenProperties()->get("NAND3_LeakagePower_11").toDouble();

        leakage_011 += getGenProperties()->get("INV1_LeakagePower_0").toDouble();
        leakage_011 += getGenProperties()->get("INV2_LeakagePower_1").toDouble();
        leakage_011 += getGenProperties()->get("INV3_LeakagePower_1").toDouble();
        leakage_011 += getGenProperties()->get("INV4_LeakagePower_1").toDouble();
        leakage_011 += getGenProperties()->get("INVZ1_LeakagePower_011_1").toDouble();
        leakage_011 += getGenProperties()->get("INVZ2_LeakagePower_100_1").toDouble();
        leakage_011 += getGenProperties()->get("INVZ3_LeakagePower_101_0").toDouble();
        leakage_011 += getGenProperties()->get("INVZ4_LeakagePower_010_0").toDouble();
        leakage_011 += getGenProperties()->get("NAND1_LeakagePower_11").toDouble();
        leakage_011 += getGenProperties()->get("NAND2_LeakagePower_01").toDouble();
        leakage_011 += getGenProperties()->get("NAND3_LeakagePower_01").toDouble();
        
        leakage_100 += getGenProperties()->get("INV1_LeakagePower_1").toDouble();
        leakage_100 += getGenProperties()->get("INV2_LeakagePower_0").toDouble();
        leakage_100 += getGenProperties()->get("INV3_LeakagePower_0").toDouble();
        leakage_100 += getGenProperties()->get("INV4_LeakagePower_1").toDouble();
        leakage_100 += getGenProperties()->get("INVZ1_LeakagePower_100_1").toDouble();
        leakage_100 += getGenProperties()->get("INVZ2_LeakagePower_011_1").toDouble();
        leakage_100 += getGenProperties()->get("INVZ3_LeakagePower_011_1").toDouble();
        leakage_100 += getGenProperties()->get("INVZ4_LeakagePower_100_1").toDouble();
        leakage_100 += getGenProperties()->get("NAND1_LeakagePower_01").toDouble();
        leakage_100 += getGenProperties()->get("NAND2_LeakagePower_10").toDouble();
        leakage_100 += getGenProperties()->get("NAND3_LeakagePower_11").toDouble();
        
        leakage_101 += getGenProperties()->get("INV1_LeakagePower_1").toDouble();
        leakage_101 += getGenProperties()->get("INV2_LeakagePower_0").toDouble();
        leakage_101 += getGenProperties()->get("INV3_LeakagePower_1").toDouble();
        leakage_101 += getGenProperties()->get("INV4_LeakagePower_1").toDouble();
        leakage_101 += getGenProperties()->get("INVZ1_LeakagePower_100_1").toDouble();
        leakage_101 += getGenProperties()->get("INVZ2_LeakagePower_011_1").toDouble();
        leakage_101 += getGenProperties()->get("INVZ3_LeakagePower_101_0").toDouble();
        leakage_101 += getGenProperties()->get("INVZ4_LeakagePower_010_0").toDouble();
        leakage_101 += getGenProperties()->get("NAND1_LeakagePower_11").toDouble();
        leakage_101 += getGenProperties()->get("NAND2_LeakagePower_10").toDouble();
        leakage_101 += getGenProperties()->get("NAND3_LeakagePower_01").toDouble();

        leakage_110 += getGenProperties()->get("INV1_LeakagePower_1").toDouble();
        leakage_110 += getGenProperties()->get("INV2_LeakagePower_1").toDouble();
        leakage_110 += getGenProperties()->get("INV3_LeakagePower_0").toDouble();
        leakage_110 += getGenProperties()->get("INV4_LeakagePower_0").toDouble();
        leakage_110 += getGenProperties()->get("INVZ1_LeakagePower_101_0").toDouble();
        leakage_110 += getGenProperties()->get("INVZ2_LeakagePower_010_0").toDouble();
        leakage_110 += getGenProperties()->get("INVZ3_LeakagePower_010_0").toDouble();
        leakage_110 += getGenProperties()->get("INVZ4_LeakagePower_101_0").toDouble();
        leakage_110 += getGenProperties()->get("NAND1_LeakagePower_00").toDouble();
        leakage_110 += getGenProperties()->get("NAND2_LeakagePower_11").toDouble();
        leakage_110 += getGenProperties()->get("NAND3_LeakagePower_10").toDouble();

        leakage_111 += getGenProperties()->get("INV1_LeakagePower_1").toDouble();
        leakage_111 += getGenProperties()->get("INV2_LeakagePower_1").toDouble();
        leakage_111 += getGenProperties()->get("INV3_LeakagePower_1").toDouble();
        leakage_111 += getGenProperties()->get("INV4_LeakagePower_0").toDouble();
        leakage_111 += getGenProperties()->get("INVZ1_LeakagePower_101_0").toDouble();
        leakage_111 += getGenProperties()->get("INVZ2_LeakagePower_010_0").toDouble();
        leakage_111 += getGenProperties()->get("INVZ3_LeakagePower_100_1").toDouble();
        leakage_111 += getGenProperties()->get("INVZ4_LeakagePower_011_1").toDouble();
        leakage_111 += getGenProperties()->get("NAND1_LeakagePower_10").toDouble();
        leakage_111 += getGenProperties()->get("NAND2_LeakagePower_11").toDouble();
        leakage_111 += getGenProperties()->get("NAND3_LeakagePower_10").toDouble();
        
        cache->set(cell_name + "->Leakage->!A!B!CI", leakage_000);
        cache->set(cell_name + "->Leakage->!A!BCI", leakage_001);
        cache->set(cell_name + "->Leakage->!AB!CI", leakage_010);
        cache->set(cell_name + "->Leakage->!ABCI", leakage_011);
        cache->set(cell_name + "->Leakage->A!B!CI", leakage_100);
        cache->set(cell_name + "->Leakage->A!BCI", leakage_101);
        cache->set(cell_name + "->Leakage->AB!CI", leakage_110);
        cache->set(cell_name + "->Leakage->ABCI", leakage_111);
        Log::printLine(cell_name + "->Leakage->!A!B!CI=" + (String) leakage_000);
        Log::printLine(cell_name + "->Leakage->!A!BCI=" + (String) leakage_001);
        Log::printLine(cell_name + "->Leakage->!AB!CI=" + (String) leakage_010);
        Log::printLine(cell_name + "->Leakage->!ABCI=" + (String) leakage_011);
        Log::printLine(cell_name + "->Leakage->A!B!CI=" + (String) leakage_100);
        Log::printLine(cell_name + "->Leakage->A!BCI=" + (String) leakage_101);
        Log::printLine(cell_name + "->Leakage->AB!CI=" + (String) leakage_110);
        Log::printLine(cell_name + "->Leakage->ABCI=" + (String) leakage_111);
        // --------------------------------------------------------------------

        /*
        // Cache event energy results
        double event_a_flip = 0.0;
        event_a_flip += getGenProperties()->get("INV1_A_Flip").toDouble() + getGenProperties()->get("INV1_ZN_Flip").toDouble();
        event_a_flip += getGenProperties()->get("INVZ1_OE_Flip").toDouble() + getGenProperties()->get("INVZ1_OEN_Flip").toDouble();
        event_a_flip += getGenProperties()->get("INVZ2_OE_Flip").toDouble() + getGenProperties()->get("INVZ2_OEN_Flip").toDouble();
        event_a_flip += getGenProperties()->get("NAND2_A1_Flip").toDouble();
        cache->set(cell_name + "->Event_A_Flip", event_a_flip);
        Log::printLine(cell_name + "->Event_A_Flip=" + (String) event_a_flip);
        
        double event_b_flip = 0.0;
        event_b_flip += getGenProperties()->get("INV2_A_Flip").toDouble() + getGenProperties()->get("INV2_ZN_Flip").toDouble();
        event_b_flip += getGenProperties()->get("INVZ1_A_Flip").toDouble();
        event_b_flip += getGenProperties()->get("INVZ2_A_Flip").toDouble();
        event_b_flip += getGenProperties()->get("NAND2_A1_Flip").toDouble();
        cache->set(cell_name + "->Event_B_Flip", event_b_flip);
        Log::printLine(cell_name + "->Event_B_Flip=" + (String) event_b_flip);

        double event_ci_flip = 0.0;
        event_ci_flip += getGenProperties()->get("INV3_A_Flip").toDouble() + getGenProperties()->get("INV3_ZN_Flip").toDouble();
        event_ci_flip += getGenProperties()->get("INVZ3_OE_Flip").toDouble() + getGenProperties()->get("INVZ3_OEN_Flip").toDouble();
        event_ci_flip += getGenProperties()->get("INVZ4_OE_Flip").toDouble() + getGenProperties()->get("INVZ4_OEN_Flip").toDouble();
        event_ci_flip += getGenProperties()->get("NAND1_A1_Flip").toDouble();
        cache->set(cell_name + "->Event_CI_Flip", event_ci_flip);
        Log::printLine(cell_name + "->Event_CI_Flip=" + (String) event_ci_flip);
        
        double event_p_flip = 0.0;
        event_p_flip += getGenProperties()->get("INV4_A_Flip").toDouble() + getGenProperties()->get("INV4_ZN_Flip").toDouble();
        event_p_flip += getGenProperties()->get("INVZ1_ZN_Flip").toDouble();
        event_p_flip += getGenProperties()->get("INVZ2_ZN_Flip").toDouble();
        event_p_flip += getGenProperties()->get("NAND1_A2_Flip").toDouble();
        cache->set(cell_name + "->Event_P_Flip", event_p_flip);
        Log::printLine(cell_name + "->Event_P_Flip=" + (String) event_p_flip);

        double event_s_flip = 0.0;
        event_s_flip += getGenProperties()->get("INVZ3_ZN_Flip").toDouble();
        event_s_flip += getGenProperties()->get("INVZ4_ZN_Flip").toDouble();
        cache->set(cell_name + "->Event_S_Flip", event_s_flip);
        Log::printLine(cell_name + "->Event_S_Flip=" + (String) event_s_flip);

        double event_cp_flip = 0.0;
        event_cp_flip += getGenProperties()->get("NAND1_ZN_Flip").toDouble();
        event_cp_flip += getGenProperties()->get("NAND3_A2_Flip").toDouble();
        cache->set(cell_name + "->Event_CP_Flip", event_cp_flip);
        Log::printLine(cell_name + "->Event_CP_Flip=" + (String) event_cp_flip);

        double event_g_flip = 0.0;
        event_g_flip += getGenProperties()->get("NAND2_ZN_Flip").toDouble();
        event_g_flip += getGenProperties()->get("NAND3_A2_Flip").toDouble();
        cache->set(cell_name + "->Event_G_Flip", event_g_flip);
        Log::printLine(cell_name + "->Event_G_Flip=" + (String) event_g_flip);

        double event_co_flip = 0.0;
        event_co_flip += getGenProperties()->get("NAND3_ZN_Flip").toDouble();
        cache->set(cell_name + "->Event_CO_Flip", event_co_flip);
        Log::printLine(cell_name + "->Event_CO_Flip=" + (String) event_co_flip);
        */
        // --------------------------------------------------------------------
        // Get Node Capacitances
        // --------------------------------------------------------------------
        double a_cap = getNet("A")->getTotalDownstreamCap();
        double b_cap = getNet("B")->getTotalDownstreamCap();
        double ci_cap = getNet("CI")->getTotalDownstreamCap();
        double a_b_cap = getNet("A_b")->getTotalDownstreamCap();
        double b_b_cap = getNet("B_b")->getTotalDownstreamCap();
        double ci_b_cap = getNet("CI_b")->getTotalDownstreamCap();
        double p_cap = getNet("P")->getTotalDownstreamCap();
        double p_b_cap = getNet("P_b")->getTotalDownstreamCap();
        double s_cap = getNet("S")->getTotalDownstreamCap();
        double cp_cap = getNet("CP")->getTotalDownstreamCap();
        double g_cap = getNet("G")->getTotalDownstreamCap();
        double co_cap = getNet("CO")->getTotalDownstreamCap();
        
        cache->set(cell_name + "->Cap->A", a_cap);
        cache->set(cell_name + "->Cap->B", b_cap);        
        cache->set(cell_name + "->Cap->CI", ci_cap);        
        cache->set(cell_name + "->Cap->A_b", a_b_cap);
        cache->set(cell_name + "->Cap->B_b", b_b_cap);        
        cache->set(cell_name + "->Cap->CI_b", ci_b_cap);        
        cache->set(cell_name + "->Cap->P", p_cap);
        cache->set(cell_name + "->Cap->P_b", p_b_cap);        
        cache->set(cell_name + "->Cap->S", s_cap);        
        cache->set(cell_name + "->Cap->CP", cp_cap);
        cache->set(cell_name + "->Cap->G", g_cap);        
        cache->set(cell_name + "->Cap->CO", co_cap);        

        Log::printLine(cell_name + "->Cap->A=" + (String) a_cap);
        Log::printLine(cell_name + "->Cap->B=" + (String) b_cap);
        Log::printLine(cell_name + "->Cap->CI=" + (String) ci_cap);
        Log::printLine(cell_name + "->Cap->A_b=" + (String) a_b_cap);
        Log::printLine(cell_name + "->Cap->B_b=" + (String) b_b_cap);
        Log::printLine(cell_name + "->Cap->CI_b=" + (String) ci_b_cap);
        Log::printLine(cell_name + "->Cap->P=" + (String) p_cap);
        Log::printLine(cell_name + "->Cap->P_b=" + (String) p_b_cap);
        Log::printLine(cell_name + "->Cap->S=" + (String) s_cap);
        Log::printLine(cell_name + "->Cap->CP=" + (String) cp_cap);
        Log::printLine(cell_name + "->Cap->G=" + (String) g_cap);
        Log::printLine(cell_name + "->Cap->CO=" + (String) co_cap);
        // --------------------------------------------------------------------

        // --------------------------------------------------------------------
        // Build Internal Delay Model
        // --------------------------------------------------------------------
        // Build abstracted timing model
        double s_ron = (getDriver("INVZ3_RonZN")->getOutputRes() + getDriver("INVZ4_RonZN")->getOutputRes()) / 2;
        double co_ron = getDriver("NAND3_RonZN")->getOutputRes();

        double a_to_s_delay = 0.0;
        a_to_s_delay += getDriver("INV1_RonZN")->calculateDelay();
        a_to_s_delay += max(getDriver("INVZ1_RonZN")->calculateDelay(), getDriver("INVZ2_RonZN")->calculateDelay());
        a_to_s_delay += max(getDriver("INVZ3_RonZN")->calculateDelay(), getDriver("INV4_RonZN")->calculateDelay() + getDriver("INVZ4_RonZN")->calculateDelay());
        
        double b_to_s_delay = 0.0;
        b_to_s_delay += max(getDriver("INVZ1_RonZN")->calculateDelay(), getDriver("INV2_RonZN")->calculateDelay() + getDriver("INVZ2_RonZN")->calculateDelay());
        b_to_s_delay += max(getDriver("INVZ3_RonZN")->calculateDelay(), getDriver("INV4_RonZN")->calculateDelay() + getDriver("INVZ4_RonZN")->calculateDelay());
        
        double ci_to_s_delay = 0.0;
        ci_to_s_delay += getDriver("INV3_RonZN")->calculateDelay();
        ci_to_s_delay += max(getDriver("INVZ3_RonZN")->calculateDelay(), getDriver("INVZ4_RonZN")->calculateDelay());
                                
        double a_to_co_delay = 0.0;
        a_to_co_delay += max(getDriver("NAND2_RonZN")->calculateDelay(),              //Generate path
                            getDriver("INV1_RonZN")->calculateDelay() +             //Carry propagate path
                            max(getDriver("INVZ1_RonZN")->calculateDelay(), getDriver("INVZ2_RonZN")->calculateDelay()) +
                            getDriver("NAND1_RonZN")->calculateDelay());
        a_to_co_delay += getDriver("NAND3_RonZN")->calculateDelay();

        double b_to_co_delay = 0.0;
        b_to_co_delay += max(getDriver("NAND2_RonZN")->calculateDelay(),              //Generate path
                            max(getDriver("INVZ1_RonZN")->calculateDelay(),         //Carry propagate path
                                getDriver("INV2_RonZN")->calculateDelay() + getDriver("INVZ2_RonZN")->calculateDelay()) +
                                getDriver("NAND1_RonZN")->calculateDelay());
        b_to_co_delay += getDriver("NAND3_RonZN")->calculateDelay();
                                
        double ci_to_co_delay = 0.0;
        ci_to_co_delay += getDriver("NAND1_RonZN")->calculateDelay();
        ci_to_co_delay += getDriver("NAND3_RonZN")->calculateDelay();
                                                                
        cache->set(cell_name + "->DriveRes->S", s_ron);    
        cache->set(cell_name + "->DriveRes->CO", co_ron);    
        
        cache->set(cell_name + "->Delay->A_to_S", a_to_s_delay);
        cache->set(cell_name + "->Delay->B_to_S", b_to_s_delay);
        cache->set(cell_name + "->Delay->CI_to_S", ci_to_s_delay);
        cache->set(cell_name + "->Delay->A_to_CO", a_to_co_delay);
        cache->set(cell_name + "->Delay->B_to_CO", b_to_co_delay);
        cache->set(cell_name + "->Delay->CI_to_CO", ci_to_co_delay);
        
        Log::printLine(cell_name + "->DriveRes->S=" + (String) s_ron);    
        Log::printLine(cell_name + "->DriveRes->CO=" + (String) co_ron);            
        Log::printLine(cell_name + "->Delay->A_to_S=" + (String) a_to_s_delay);
        Log::printLine(cell_name + "->Delay->B_to_S=" + (String) b_to_s_delay);
        Log::printLine(cell_name + "->Delay->CI_to_S=" + (String) ci_to_s_delay);
        Log::printLine(cell_name + "->Delay->A_to_CO=" + (String) a_to_co_delay);
        Log::printLine(cell_name + "->Delay->B_to_CO=" + (String) b_to_co_delay);
        Log::printLine(cell_name + "->Delay->CI_to_CO=" + (String) ci_to_co_delay);
        // --------------------------------------------------------------------
                
        return;

    }    
    
} // namespace DSENT

