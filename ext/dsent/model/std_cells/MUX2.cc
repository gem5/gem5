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

#include "model/std_cells/MUX2.h"

#include <cmath>

#include "model/PortInfo.h"
#include "model/TransitionInfo.h"
#include "model/EventInfo.h"
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

    MUX2::MUX2(const String& instance_name_, const TechModel* tech_model_)
        : StdCell(instance_name_, tech_model_)
    {
        initProperties();
    }

    MUX2::~MUX2()
    {}

    void MUX2::initProperties()
    {
        return;
    }

    void MUX2::constructModel()
    {
        // All constructModel should do is create Area/NDDPower/Energy Results as
        // well as instantiate any sub-instances using only the hard parameters
        
        createInputPort("A");
        createInputPort("B");
        createInputPort("S0");
        createOutputPort("Y");
        
        createLoad("A_Cap");
        createLoad("B_Cap");
        createLoad("S0_Cap");
        createDelay("A_to_Y_delay");
        createDelay("B_to_Y_delay");
        createDelay("S0_to_Y_delay");
        createDriver("Y_Ron", true);
                
        ElectricalLoad* a_cap = getLoad("A_Cap");
        ElectricalLoad* b_cap = getLoad("B_Cap");
        ElectricalLoad* s0_cap = getLoad("S0_Cap");
        ElectricalDelay* a_to_y_delay = getDelay("A_to_Y_delay");
        ElectricalDelay* b_to_y_delay = getDelay("B_to_Y_delay");
        ElectricalDelay* s0_to_y_delay = getDelay("S0_to_Y_delay");
        ElectricalDriver* y_ron = getDriver("Y_Ron");
        
        getNet("A")->addDownstreamNode(a_cap);
        getNet("B")->addDownstreamNode(b_cap);
        getNet("S0")->addDownstreamNode(s0_cap);
        a_cap->addDownstreamNode(a_to_y_delay);        
        b_cap->addDownstreamNode(b_to_y_delay);        
        s0_cap->addDownstreamNode(s0_to_y_delay);        
        a_to_y_delay->addDownstreamNode(y_ron);
        b_to_y_delay->addDownstreamNode(y_ron);
        s0_to_y_delay->addDownstreamNode(y_ron);
        y_ron->addDownstreamNode(getNet("Y"));

        // Create Area result
        createElectricalAtomicResults();
        getEventInfo("Idle")->setStaticTransitionInfos();
        // Create MUX2 Event Energy Result
        createElectricalEventAtomicResult("MUX2");

        
        return;
    }
    
    void MUX2::updateModel()
    {
        // Get parameters
        double drive_strength = getDrivingStrength();
        Map<double>* cache = getTechModel()->getStdCellLib()->getStdCellCache();

        // Standard cell cache string
        String cell_name = "MUX2_X" + (String) drive_strength;
        
        // Get timing parameters
        getLoad("A_Cap")->setLoadCap(cache->get(cell_name + "->Cap->A"));
        getLoad("B_Cap")->setLoadCap(cache->get(cell_name + "->Cap->B"));
        getLoad("S0_Cap")->setLoadCap(cache->get(cell_name + "->Cap->S0"));

        getDelay("A_to_Y_delay")->setDelay(cache->get(cell_name + "->Delay->A_to_Y"));
        getDelay("B_to_Y_delay")->setDelay(cache->get(cell_name + "->Delay->B_to_Y"));
        getDelay("S0_to_Y_delay")->setDelay(cache->get(cell_name + "->Delay->S0_to_Y"));

        getDriver("Y_Ron")->setOutputRes(cache->get(cell_name + "->DriveRes->Y"));
                
        // Set the cell area
        getAreaResult("Active")->setValue(cache->get(cell_name + "->ActiveArea"));
        getAreaResult("Metal1Wire")->setValue(cache->get(cell_name + "->ActiveArea"));
        
        return;
    }

    void MUX2::evaluateModel()
    {
        return;
    }

    void MUX2::useModel()
    {
        // Get parameters
        double drive_strength = getDrivingStrength();
        Map<double>* cache = getTechModel()->getStdCellLib()->getStdCellCache();

        // Standard cell cache string
        String cell_name = "MUX2_X" + (String) drive_strength;

        // Propagate the transition and get the 0->1 transition count
        propagateTransitionInfo();
        double P_A = getInputPort("A")->getTransitionInfo().getProbability1();
        double P_B = getInputPort("B")->getTransitionInfo().getProbability1();
        double P_S0 = getInputPort("S0")->getTransitionInfo().getProbability1();
        double S0_num_trans_01 = getInputPort("S0")->getTransitionInfo().getNumberTransitions01();
        double Y_num_trans_01 = getOutputPort("Y")->getTransitionInfo().getNumberTransitions01();

        // Calculate leakage
        double leakage = 0;
        leakage += cache->get(cell_name + "->Leakage->!A!B!S0") * (1 - P_A) * (1 - P_B) * (1 - P_S0);
        leakage += cache->get(cell_name + "->Leakage->!A!BS0") * (1 - P_A) * (1 - P_B) * P_S0;
        leakage += cache->get(cell_name + "->Leakage->!AB!S0") * (1 - P_A) * P_B * (1 - P_S0);
        leakage += cache->get(cell_name + "->Leakage->!ABS0") * (1 - P_A) * P_B * P_S0;
        leakage += cache->get(cell_name + "->Leakage->A!B!S0") * P_A * (1 - P_B) * (1 - P_S0);
        leakage += cache->get(cell_name + "->Leakage->A!BS0") * P_A * (1 - P_B) * P_S0;
        leakage += cache->get(cell_name + "->Leakage->AB!S0") * P_A * P_B * (1 - P_S0);
        leakage += cache->get(cell_name + "->Leakage->ABS0") * P_A * P_B * P_S0;
        getNddPowerResult("Leakage")->setValue(leakage);
        
        // Get VDD
        double vdd = getTechModel()->get("Vdd");

        // Get capacitances
        double s0_b_cap = cache->get(cell_name + "->Cap->S0_b");
        double y_bar_cap = cache->get(cell_name + "->Cap->Y_b"); 
        double y_cap = cache->get(cell_name + "->Cap->Y");
        double y_load_cap = getNet("Y")->getTotalDownstreamCap();
        // Create mux2 event energy
        double mux2_event_energy = 0.0;
        mux2_event_energy += (s0_b_cap) * S0_num_trans_01;
        mux2_event_energy += (y_bar_cap + y_cap + y_load_cap) * Y_num_trans_01;
        mux2_event_energy *= vdd * vdd;
        getEventResult("MUX2")->setValue(mux2_event_energy);

        return;
    }

    void MUX2::propagateTransitionInfo()
    {
        // Get input signal transition info
        const TransitionInfo& trans_A = getInputPort("A")->getTransitionInfo();
        const TransitionInfo& trans_B = getInputPort("B")->getTransitionInfo();
        const TransitionInfo& trans_S0 = getInputPort("S0")->getTransitionInfo();

        // Scale all transition information to the highest freq multiplier
        double max_freq_mult = max(max(trans_A.getFrequencyMultiplier(), trans_B.getFrequencyMultiplier()), trans_S0.getFrequencyMultiplier());
        const TransitionInfo& scaled_trans_A = trans_A.scaleFrequencyMultiplier(max_freq_mult);
        const TransitionInfo& scaled_trans_B = trans_B.scaleFrequencyMultiplier(max_freq_mult);
        const TransitionInfo& scaled_trans_S0 = trans_S0.scaleFrequencyMultiplier(max_freq_mult);
        
        // Compute the probability of each transition on a given cycle
        double A_prob_00 = scaled_trans_A.getNumberTransitions00() / max_freq_mult;
        double A_prob_01 = scaled_trans_A.getNumberTransitions01() / max_freq_mult;
        double A_prob_10 = A_prob_01;
        double A_prob_11 = scaled_trans_A.getNumberTransitions11() / max_freq_mult;
        double B_prob_00 = scaled_trans_B.getNumberTransitions00() / max_freq_mult;
        double B_prob_01 = scaled_trans_B.getNumberTransitions01() / max_freq_mult;
        double B_prob_10 = B_prob_01;
        double B_prob_11 = scaled_trans_B.getNumberTransitions11() / max_freq_mult;
        double S0_prob_00 = scaled_trans_S0.getNumberTransitions00() / max_freq_mult;
        double S0_prob_01 = scaled_trans_S0.getNumberTransitions01() / max_freq_mult;
        double S0_prob_10 = S0_prob_01;
        double S0_prob_11 = scaled_trans_S0.getNumberTransitions11() / max_freq_mult;

        // Compute output probabilities
        double Y_prob_00 = S0_prob_00 * A_prob_00 + 
                            S0_prob_01 * (A_prob_00 + A_prob_01) * (B_prob_00 + B_prob_10) + 
                            S0_prob_10 * (A_prob_00 + A_prob_10) * (B_prob_00 + B_prob_01) +
                            S0_prob_11 * B_prob_00;
        double Y_prob_01 = S0_prob_00 * A_prob_01 + 
                            S0_prob_01 * (A_prob_00 + A_prob_01) * (B_prob_01 + B_prob_11) + 
                            S0_prob_10 * (A_prob_01 + A_prob_11) * (B_prob_00 + B_prob_01) +
                            S0_prob_11 * B_prob_01;
        double Y_prob_11 = S0_prob_00 * A_prob_11 + 
                            S0_prob_01 * (A_prob_10 + A_prob_11) * (B_prob_01 + B_prob_11) + 
                            S0_prob_10 * (A_prob_01 + A_prob_11) * (B_prob_10 + B_prob_11) +
                            S0_prob_11 * B_prob_11;
                            
        // Check that probabilities add up to 1.0 with some finite tolerance
        ASSERT(LibUtil::Math::isEqual((Y_prob_00 + Y_prob_01 + Y_prob_01 + Y_prob_11), 1.0), 
            "[Error] " + getInstanceName() +  "Output transition probabilities must add up to 1 (" +
            (String) Y_prob_00 + ", " + (String) Y_prob_01 + ", " + (String) Y_prob_11 + ")!");
                            
        // Turn probability of transitions per cycle into number of transitions per time unit
        TransitionInfo trans_Y(Y_prob_00 * max_freq_mult, Y_prob_01 * max_freq_mult, Y_prob_11 * max_freq_mult);
        getOutputPort("Y")->setTransitionInfo(trans_Y);

        return;
    }

    // Creates the standard cell, characterizes and abstracts away the details
    void MUX2::cacheStdCell(StdCellLib* cell_lib_, double drive_strength_)
    {
        // Get parameters        
        double gate_pitch = cell_lib_->getTechModel()->get("Gate->PitchContacted");
        Map<double>* cache = cell_lib_->getStdCellCache();

        // Standard cell cache string
        String cell_name = "MUX2_X" + (String) drive_strength_;

        Log::printLine("=== " + cell_name + " ===");
        
        // Now actually build the full standard cell model
        createInputPort("A");
        createInputPort("B");
        createInputPort("S0");
        createOutputPort("Y");
        
        createNet("S0_b");
        createNet("Y_b");

        // Adds macros
        CellMacros::addInverter(this, "INV1", false, true, "S0", "S0_b");
        CellMacros::addInverter(this, "INV2", false, true, "Y_b", "Y");
        CellMacros::addTristate(this, "INVZ1", true, true, true, true, "A", "S0_b", "S0", "Y_b");
        CellMacros::addTristate(this, "INVZ2", true, true, true, true, "B", "S0", "S0_b", "Y_b");
                
        // I have no idea how to size each of the parts haha
        CellMacros::updateInverter(this, "INV1", drive_strength_ * 0.250);
        CellMacros::updateInverter(this, "INV2", drive_strength_ * 1.000);
        CellMacros::updateTristate(this, "INVZ1", drive_strength_ * 0.500);
        CellMacros::updateTristate(this, "INVZ2", drive_strength_ * 0.500);
                        
        // Cache area result
        double area = 0.0;
        area += gate_pitch * getTotalHeight() * 1;
        area += gate_pitch * getTotalHeight() * getGenProperties()->get("INV1_GatePitches").toDouble();
        area += gate_pitch * getTotalHeight() * getGenProperties()->get("INV2_GatePitches").toDouble();
        area += gate_pitch * getTotalHeight() * getGenProperties()->get("INVZ1_GatePitches").toDouble();
        area += gate_pitch * getTotalHeight() * getGenProperties()->get("INVZ2_GatePitches").toDouble();
        cache->set(cell_name + "->ActiveArea", area);
        Log::printLine(cell_name + "->ActiveArea=" + (String) area);

        // --------------------------------------------------------------------
        // Cache Leakage Power (for every single signal combination)
        // --------------------------------------------------------------------
        double leakage_000 = 0;          //!A, !B, !S0
        double leakage_001 = 0;          //!A, !B, S0
        double leakage_010 = 0;          //!A, B, !S0
        double leakage_011 = 0;          //!A, B, S0
        double leakage_100 = 0;          //A, !B, !S0
        double leakage_101 = 0;          //A, !B, S0
        double leakage_110 = 0;          //A, B, !S0
        double leakage_111 = 0;          //A, B, S0

        //This is so painful...
        leakage_000 += getGenProperties()->get("INV1_LeakagePower_0").toDouble();
        leakage_000 += getGenProperties()->get("INV2_LeakagePower_1").toDouble();
        leakage_000 += getGenProperties()->get("INVZ1_LeakagePower_100_1").toDouble();
        leakage_000 += getGenProperties()->get("INVZ2_LeakagePower_010_1").toDouble();

        leakage_001 += getGenProperties()->get("INV1_LeakagePower_1").toDouble();
        leakage_001 += getGenProperties()->get("INV2_LeakagePower_1").toDouble();
        leakage_001 += getGenProperties()->get("INVZ1_LeakagePower_010_1").toDouble();
        leakage_001 += getGenProperties()->get("INVZ2_LeakagePower_100_1").toDouble();

        leakage_010 += getGenProperties()->get("INV1_LeakagePower_0").toDouble();
        leakage_010 += getGenProperties()->get("INV2_LeakagePower_1").toDouble();
        leakage_010 += getGenProperties()->get("INVZ1_LeakagePower_100_1").toDouble();
        leakage_010 += getGenProperties()->get("INVZ2_LeakagePower_011_1").toDouble();

        leakage_011 += getGenProperties()->get("INV1_LeakagePower_1").toDouble();
        leakage_011 += getGenProperties()->get("INV2_LeakagePower_0").toDouble();
        leakage_011 += getGenProperties()->get("INVZ1_LeakagePower_010_0").toDouble();
        leakage_011 += getGenProperties()->get("INVZ2_LeakagePower_101_0").toDouble();

        leakage_100 += getGenProperties()->get("INV1_LeakagePower_0").toDouble();
        leakage_100 += getGenProperties()->get("INV2_LeakagePower_0").toDouble();
        leakage_100 += getGenProperties()->get("INVZ1_LeakagePower_101_0").toDouble();
        leakage_100 += getGenProperties()->get("INVZ2_LeakagePower_010_0").toDouble();

        leakage_101 += getGenProperties()->get("INV1_LeakagePower_1").toDouble();
        leakage_101 += getGenProperties()->get("INV2_LeakagePower_0").toDouble();
        leakage_101 += getGenProperties()->get("INVZ1_LeakagePower_011_1").toDouble();
        leakage_101 += getGenProperties()->get("INVZ2_LeakagePower_100_1").toDouble();

        leakage_110 += getGenProperties()->get("INV1_LeakagePower_1").toDouble();
        leakage_110 += getGenProperties()->get("INV2_LeakagePower_1").toDouble();
        leakage_110 += getGenProperties()->get("INVZ1_LeakagePower_101_0").toDouble();
        leakage_110 += getGenProperties()->get("INVZ2_LeakagePower_011_0").toDouble();

        leakage_111 += getGenProperties()->get("INV1_LeakagePower_1").toDouble();
        leakage_111 += getGenProperties()->get("INV2_LeakagePower_1").toDouble();
        leakage_111 += getGenProperties()->get("INVZ1_LeakagePower_011_0").toDouble();
        leakage_111 += getGenProperties()->get("INVZ2_LeakagePower_101_0").toDouble();
        
        cache->set(cell_name + "->Leakage->!A!B!S0", leakage_000);
        cache->set(cell_name + "->Leakage->!A!BS0", leakage_001);
        cache->set(cell_name + "->Leakage->!AB!S0", leakage_010);
        cache->set(cell_name + "->Leakage->!ABS0", leakage_011);
        cache->set(cell_name + "->Leakage->A!B!S0", leakage_100);
        cache->set(cell_name + "->Leakage->A!BS0", leakage_101);
        cache->set(cell_name + "->Leakage->AB!S0", leakage_110);
        cache->set(cell_name + "->Leakage->ABS0", leakage_111);
        Log::printLine(cell_name + "->Leakage->!A!B!S0=" + (String) leakage_000);
        Log::printLine(cell_name + "->Leakage->!A!BS0=" + (String) leakage_001);
        Log::printLine(cell_name + "->Leakage->!AB!S0=" + (String) leakage_010);
        Log::printLine(cell_name + "->Leakage->!ABS0=" + (String) leakage_011);
        Log::printLine(cell_name + "->Leakage->A!B!S0=" + (String) leakage_100);
        Log::printLine(cell_name + "->Leakage->A!BS0=" + (String) leakage_101);
        Log::printLine(cell_name + "->Leakage->AB!S0=" + (String) leakage_110);
        Log::printLine(cell_name + "->Leakage->ABS0=" + (String) leakage_111);

        // Cache event energy results
        /*
        double event_a_flip = 0.0;
        event_a_flip += getGenProperties()->get("INVZ1_A_Flip").toDouble();
        cache->set(cell_name + "->Event_A_Flip", event_a_flip);
        Log::printLine(cell_name + "->Event_A_Flip=" + (String) event_a_flip);
        
        double event_b_flip = 0.0;
        event_b_flip += getGenProperties()->get("INVZ1_A_Flip").toDouble();
        cache->set(cell_name + "->Event_B_Flip", event_b_flip);
        Log::printLine(cell_name + "->Event_B_Flip=" + (String) event_b_flip);

        double event_s0_flip = 0.0;
        event_s0_flip += getGenProperties()->get("INV1_A_Flip").toDouble();
        event_s0_flip += getGenProperties()->get("INV1_ZN_Flip").toDouble();
        event_s0_flip += getGenProperties()->get("INVZ1_OE_Flip").toDouble() + getGenProperties()->get("INVZ1_OEN_Flip").toDouble();
        event_s0_flip += getGenProperties()->get("INVZ2_OE_Flip").toDouble() + getGenProperties()->get("INVZ2_OEN_Flip").toDouble();
        cache->set(cell_name + "->Event_S0_Flip", event_s0_flip);
        Log::printLine(cell_name + "->Event_S0_Flip=" + (String) event_s0_flip);
                
        double event_y_flip = 0.0;
        event_y_flip += getGenProperties()->get("INVZ1_ZN_Flip").toDouble();
        event_y_flip += getGenProperties()->get("INVZ2_ZN_Flip").toDouble();
        event_y_flip += getGenProperties()->get("INV2_A_Flip").toDouble();
        event_y_flip += getGenProperties()->get("INV2_ZN_Flip").toDouble();
        cache->set(cell_name + "->Event_Y_Flip", event_y_flip);
        Log::printLine(cell_name + "->Event_Y_Flip=" + (String) event_y_flip);

        double a_cap = getLoad("INVZ1_CgA")->getLoadCap();
        double b_cap = getLoad("INVZ2_CgA")->getLoadCap();        
        double s0_cap = getLoad("INV1_CgA")->getLoadCap() + getLoad("INVZ1_CgOEN")->getLoadCap() + getLoad("INVZ2_CgOE")->getLoadCap();
        double y_ron = getDriver("INV2_RonZN")->getOutputRes();
        */
        // --------------------------------------------------------------------
        
        // --------------------------------------------------------------------
        // Get Node capacitances
        // --------------------------------------------------------------------
        double a_cap = getNet("A")->getTotalDownstreamCap(); 
        double b_cap = getNet("B")->getTotalDownstreamCap(); 
        double s0_cap = getNet("S0")->getTotalDownstreamCap();
        double s0_b_cap = getNet("S0_b")->getTotalDownstreamCap();
        double y_b_cap = getNet("Y_b")->getTotalDownstreamCap();
        double y_cap = getNet("Y")->getTotalDownstreamCap();
        
        cache->set(cell_name + "->Cap->A", a_cap);
        cache->set(cell_name + "->Cap->B", b_cap);        
        cache->set(cell_name + "->Cap->S0", s0_cap);
        cache->set(cell_name + "->Cap->S0_b", s0_b_cap);        
        cache->set(cell_name + "->Cap->Y_b", y_b_cap);
        cache->set(cell_name + "->Cap->Y", y_cap);

        Log::printLine(cell_name + "->Cap->A=" + (String) a_cap);
        Log::printLine(cell_name + "->Cap->B=" + (String) b_cap);
        Log::printLine(cell_name + "->Cap->S0=" + (String) s0_cap);
        Log::printLine(cell_name + "->Cap->S0_b=" + (String) s0_b_cap);
        Log::printLine(cell_name + "->Cap->Y_b=" + (String) y_b_cap);
        Log::printLine(cell_name + "->Cap->Y=" + (String) y_cap);
        // --------------------------------------------------------------------
        
        // --------------------------------------------------------------------
        // Build Internal Delay Model
        // --------------------------------------------------------------------
        // Build abstracted timing model
        double y_ron = getDriver("INV2_RonZN")->getOutputRes();

        double a_to_y_delay = 0.0;
        a_to_y_delay += getDriver("INVZ1_RonZN")->calculateDelay();
        a_to_y_delay += getDriver("INV2_RonZN")->calculateDelay();
        
        double b_to_y_delay = 0.0;
        b_to_y_delay += getDriver("INVZ1_RonZN")->calculateDelay();
        b_to_y_delay += getDriver("INV2_RonZN")->calculateDelay();

        double s0_to_y_delay = 0.0;
        s0_to_y_delay += getDriver("INV1_RonZN")->calculateDelay();
        s0_to_y_delay += max(getDriver("INVZ1_RonZN")->calculateDelay(), getDriver("INVZ1_RonZN")->calculateDelay());        
        s0_to_y_delay += getDriver("INV2_RonZN")->calculateDelay();
        
        cache->set(cell_name + "->DriveRes->Y", y_ron);
        cache->set(cell_name + "->Delay->A_to_Y", a_to_y_delay);
        cache->set(cell_name + "->Delay->B_to_Y", b_to_y_delay);
        cache->set(cell_name + "->Delay->S0_to_Y", s0_to_y_delay);
        
        Log::printLine(cell_name + "->DriveRes->Y=" + (String) y_ron);    
        Log::printLine(cell_name + "->Delay->A_to_Y=" + (String) a_to_y_delay);
        Log::printLine(cell_name + "->Delay->B_to_Y=" + (String) b_to_y_delay);
        Log::printLine(cell_name + "->Delay->S0_to_Y=" + (String) s0_to_y_delay);
        // --------------------------------------------------------------------

        return;
    }        
    
} // namespace DSENT

