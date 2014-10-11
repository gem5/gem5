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

#include "model/std_cells/XOR2.h"

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

    XOR2::XOR2(const String& instance_name_, const TechModel* tech_model_)
        : StdCell(instance_name_, tech_model_)
    {
        initProperties();
    }

    XOR2::~XOR2()
    {}

    void XOR2::initProperties()
    {
        return;
    }

    void XOR2::constructModel()
    {
        // All constructModel should do is create Area/NDDPower/Energy Results as
        // well as instantiate any sub-instances using only the hard parameters
        
        createInputPort("A");
        createInputPort("B");
        createOutputPort("Y");
        
        createLoad("A_Cap");
        createLoad("B_Cap");
        createDelay("A_to_Y_delay");
        createDelay("B_to_Y_delay");
        createDriver("Y_Ron", true);
                
        ElectricalLoad* a_cap = getLoad("A_Cap");
        ElectricalLoad* b_cap = getLoad("B_Cap");
        ElectricalDelay* a_to_y_delay = getDelay("A_to_Y_delay");
        ElectricalDelay* b_to_y_delay = getDelay("B_to_Y_delay");
        ElectricalDriver* y_ron = getDriver("Y_Ron");
        
        getNet("A")->addDownstreamNode(a_cap);
        getNet("B")->addDownstreamNode(b_cap);
        a_cap->addDownstreamNode(a_to_y_delay);        
        b_cap->addDownstreamNode(b_to_y_delay);        
        a_to_y_delay->addDownstreamNode(y_ron);
        b_to_y_delay->addDownstreamNode(y_ron);
        y_ron->addDownstreamNode(getNet("Y"));

        // Create Area result
        // Create NDD Power result
        createElectricalAtomicResults();
        // Create XOR2 Event Energy Result
        createElectricalEventAtomicResult("XOR2");

        getEventInfo("Idle")->setStaticTransitionInfos();
        
        return;
    }
    
    void XOR2::updateModel()
    {
        // Get parameters
        double drive_strength = getDrivingStrength();
        Map<double>* cache = getTechModel()->getStdCellLib()->getStdCellCache();

        // Standard cell cache string
        String cell_name = "XOR2_X" + (String) drive_strength;
        
        // Get timing parameters
        getLoad("A_Cap")->setLoadCap(cache->get(cell_name + "->Cap->A"));
        getLoad("B_Cap")->setLoadCap(cache->get(cell_name + "->Cap->B"));

        getDelay("A_to_Y_delay")->setDelay(cache->get(cell_name + "->Delay->A_to_Y"));
        getDelay("B_to_Y_delay")->setDelay(cache->get(cell_name + "->Delay->B_to_Y"));

        getDriver("Y_Ron")->setOutputRes(cache->get(cell_name + "->DriveRes->Y"));
                
        // Set the cell area
        getAreaResult("Active")->setValue(cache->get(cell_name + "->ActiveArea"));
        getAreaResult("Metal1Wire")->setValue(cache->get(cell_name + "->ActiveArea"));
        
        return;
    }

    void XOR2::evaluateModel()
    {
        return;
    }
    
    void XOR2::useModel()
    {
        // Get parameters
        double drive_strength = getDrivingStrength();
        Map<double>* cache = getTechModel()->getStdCellLib()->getStdCellCache();

        // Standard cell cache string
        String cell_name = "XOR2_X" + (String) drive_strength;
    
        // Propagate the transition info and get the 0->1 transtion count
        propagateTransitionInfo();
        double P_A = getInputPort("A")->getTransitionInfo().getProbability1();
        double P_B = getInputPort("B")->getTransitionInfo().getProbability1();
        double A_num_trans_01 = getInputPort("A")->getTransitionInfo().getNumberTransitions01();
        double B_num_trans_01 = getInputPort("B")->getTransitionInfo().getNumberTransitions01();
        double Y_num_trans_01 = getOutputPort("Y")->getTransitionInfo().getNumberTransitions01();

        // Calculate leakage
        double leakage = 0;
        leakage += cache->get(cell_name + "->Leakage->!A!B") * (1 - P_A) * (1 - P_B);
        leakage += cache->get(cell_name + "->Leakage->!AB") * (1 - P_A) * P_B;
        leakage += cache->get(cell_name + "->Leakage->A!B") * P_A * (1 - P_B);
        leakage += cache->get(cell_name + "->Leakage->AB") * P_A * P_B;
        getNddPowerResult("Leakage")->setValue(leakage);

        // Get VDD
        double vdd = getTechModel()->get("Vdd");
        
        // Get capacitances
        double a_b_cap = cache->get(cell_name + "->Cap->A_b");
        double b_b_cap = cache->get(cell_name + "->Cap->B_b");
        double y_cap = cache->get(cell_name + "->Cap->Y");
        double y_load_cap = getNet("Y")->getTotalDownstreamCap();                
        
        // Calculate XOR Event energy
        double xor2_event_result = 0.0;
        xor2_event_result += a_b_cap * A_num_trans_01;
        xor2_event_result += b_b_cap * B_num_trans_01;        
        xor2_event_result += (y_cap + y_load_cap) * Y_num_trans_01;
        xor2_event_result *= vdd * vdd;
        getEventResult("XOR2")->setValue(xor2_event_result);

        return;
    }

    void XOR2::propagateTransitionInfo()
    {
        // Get input signal transition info
        const TransitionInfo& trans_A = getInputPort("A")->getTransitionInfo();
        const TransitionInfo& trans_B = getInputPort("B")->getTransitionInfo();

        double max_freq_mult = max(trans_A.getFrequencyMultiplier(), trans_B.getFrequencyMultiplier());
        const TransitionInfo& scaled_trans_A = trans_A.scaleFrequencyMultiplier(max_freq_mult);
        const TransitionInfo& scaled_trans_B = trans_B.scaleFrequencyMultiplier(max_freq_mult);

        
        double A_prob_00 = scaled_trans_A.getNumberTransitions00() / max_freq_mult;
        double A_prob_01 = scaled_trans_A.getNumberTransitions01() / max_freq_mult;
        double A_prob_10 = A_prob_01;
        double A_prob_11 = scaled_trans_A.getNumberTransitions11() / max_freq_mult;
        double B_prob_00 = scaled_trans_B.getNumberTransitions00() / max_freq_mult;
        double B_prob_01 = scaled_trans_B.getNumberTransitions01() / max_freq_mult;
        double B_prob_10 = B_prob_01;
        double B_prob_11 = scaled_trans_B.getNumberTransitions11() / max_freq_mult;

        // Set output transition info
        double Y_prob_00 = A_prob_00 * B_prob_00 + 
                                A_prob_01 * B_prob_01 + 
                                A_prob_10 * B_prob_10 +
                                A_prob_11 * B_prob_11;
        double Y_prob_01 = A_prob_00 * B_prob_01 +
                                A_prob_01 * B_prob_00 +
                                A_prob_10 * B_prob_11 + 
                                A_prob_11 * B_prob_10;
        double Y_prob_11 = A_prob_00 * B_prob_11 +
                                A_prob_01 * B_prob_10 +
                                A_prob_10 * B_prob_01 +
                                A_prob_11 * B_prob_00;
                                
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
    void XOR2::cacheStdCell(StdCellLib* cell_lib_, double drive_strength_)
    {
        // Get parameters        
        double gate_pitch = cell_lib_->getTechModel()->get("Gate->PitchContacted");
        Map<double>* cache = cell_lib_->getStdCellCache();

        // Standard cell cache string
        String cell_name = "XOR2_X" + (String) drive_strength_;

        Log::printLine("=== " + cell_name + " ===");
        
        // Now actually build the full standard cell model
        createInputPort("A");
        createInputPort("B");
        createOutputPort("Y");
        
        createNet("A_b");
        createNet("B_b");

        // Adds macros
        CellMacros::addInverter(this, "INV1", false, true, "A", "A_b");
        CellMacros::addInverter(this, "INV2", false, true, "B", "B_b");
        CellMacros::addTristate(this, "INVZ1", true, true, true, true, "B", "A", "A_b", "Y");
        CellMacros::addTristate(this, "INVZ2", true, true, true, true, "B_b", "A_b", "A", "Y");
                
        // I have no idea how to size each of the parts haha
        CellMacros::updateInverter(this, "INV1", drive_strength_ * 0.500);
        CellMacros::updateInverter(this, "INV2", drive_strength_ * 0.500);
        CellMacros::updateTristate(this, "INVZ1", drive_strength_ * 1.000);
        CellMacros::updateTristate(this, "INVZ2", drive_strength_ * 1.000);
                        
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
        // Leakage Model Calculation
        // --------------------------------------------------------------------
        // Cache leakage power results (for every single signal combination)
        double leakage_00 = 0;          //!A, !B
        double leakage_01 = 0;          //!A, B
        double leakage_10 = 0;          //A, !B
        double leakage_11 = 0;          //A, B

        //This is so painful...
        leakage_00 += getGenProperties()->get("INV1_LeakagePower_0").toDouble();
        leakage_00 += getGenProperties()->get("INV2_LeakagePower_0").toDouble();
        leakage_00 += getGenProperties()->get("INVZ1_LeakagePower_010_0").toDouble();
        leakage_00 += getGenProperties()->get("INVZ2_LeakagePower_101_0").toDouble();

        leakage_01 += getGenProperties()->get("INV1_LeakagePower_0").toDouble();
        leakage_01 += getGenProperties()->get("INV2_LeakagePower_1").toDouble();
        leakage_01 += getGenProperties()->get("INVZ1_LeakagePower_011_1").toDouble();
        leakage_01 += getGenProperties()->get("INVZ2_LeakagePower_100_1").toDouble();

        leakage_10 += getGenProperties()->get("INV1_LeakagePower_1").toDouble();
        leakage_10 += getGenProperties()->get("INV2_LeakagePower_0").toDouble();
        leakage_10 += getGenProperties()->get("INVZ1_LeakagePower_100_1").toDouble();
        leakage_10 += getGenProperties()->get("INVZ2_LeakagePower_011_1").toDouble();

        leakage_11 += getGenProperties()->get("INV1_LeakagePower_1").toDouble();
        leakage_11 += getGenProperties()->get("INV2_LeakagePower_1").toDouble();
        leakage_11 += getGenProperties()->get("INVZ1_LeakagePower_101_0").toDouble();
        leakage_11 += getGenProperties()->get("INVZ2_LeakagePower_010_0").toDouble();

        cache->set(cell_name + "->Leakage->!A!B", leakage_00);
        cache->set(cell_name + "->Leakage->!AB", leakage_01);
        cache->set(cell_name + "->Leakage->A!B", leakage_10);
        cache->set(cell_name + "->Leakage->AB", leakage_11);
        Log::printLine(cell_name + "->Leakage->!A!B=" + (String) leakage_00);
        Log::printLine(cell_name + "->Leakage->!AB=" + (String) leakage_01);
        Log::printLine(cell_name + "->Leakage->A!B=" + (String) leakage_10);
        Log::printLine(cell_name + "->Leakage->AB=" + (String) leakage_11);
        // --------------------------------------------------------------------

        // Cache event energy results
        /*
        double event_a_flip = 0.0;
        event_a_flip += getGenProperties()->get("INV1_A_Flip").toDouble() + getGenProperties()->get("INV1_ZN_Flip").toDouble();
        event_a_flip += getGenProperties()->get("INVZ1_OE_Flip").toDouble() + getGenProperties()->get("INVZ1_OEN_Flip").toDouble();
        event_a_flip += getGenProperties()->get("INVZ2_OE_Flip").toDouble() + getGenProperties()->get("INVZ2_OEN_Flip").toDouble();
        cache->set(cell_name + "->Event_A_Flip", event_a_flip);
        Log::printLine(cell_name + "->Event_A_Flip=" + (String) event_a_flip);
        
        double event_b_flip = 0.0;
        event_b_flip += getGenProperties()->get("INV2_A_Flip").toDouble() + getGenProperties()->get("INV2_ZN_Flip").toDouble();
        event_b_flip += getGenProperties()->get("INVZ1_A_Flip").toDouble();
        event_b_flip += getGenProperties()->get("INVZ2_A_Flip").toDouble();
        cache->set(cell_name + "->Event_B_Flip", event_b_flip);
        Log::printLine(cell_name + "->Event_B_Flip=" + (String) event_b_flip);

        double event_y_flip = 0.0;
        event_y_flip += getGenProperties()->get("INVZ1_ZN_Flip").toDouble();
        event_y_flip += getGenProperties()->get("INVZ2_ZN_Flip").toDouble();
        cache->set(cell_name + "->Event_Y_Flip", event_y_flip);
        Log::printLine(cell_name + "->Event_Y_Flip=" + (String) event_y_flip);
        */
        
        // --------------------------------------------------------------------
        // Get Node Capacitances
        // --------------------------------------------------------------------
        // Build abstracted timing model
        double a_cap = getNet("A")->getTotalDownstreamCap();
        double b_cap = getNet("B")->getTotalDownstreamCap();
        double a_b_cap = getNet("A_b")->getTotalDownstreamCap();
        double b_b_cap = getNet("B_b")->getTotalDownstreamCap();
        double y_cap = getNet("Y")->getTotalDownstreamCap();

        cache->set(cell_name + "->Cap->A", a_cap);
        cache->set(cell_name + "->Cap->B", b_cap);
        cache->set(cell_name + "->Cap->A_b", a_b_cap);
        cache->set(cell_name + "->Cap->B_b", b_b_cap);
        cache->set(cell_name + "->Cap->Y", y_cap);
        Log::printLine(cell_name + "->Cap->A=" + (String) a_cap);
        Log::printLine(cell_name + "->Cap->B=" + (String) b_cap);
        Log::printLine(cell_name + "->Cap->A=" + (String) a_b_cap);
        Log::printLine(cell_name + "->Cap->B=" + (String) b_b_cap);
        Log::printLine(cell_name + "->Cap->Y=" + (String) y_cap);
        // --------------------------------------------------------------------

        // --------------------------------------------------------------------
        // Build Internal Delay Model
        // --------------------------------------------------------------------
        double y_ron = (getDriver("INVZ1_RonZN")->getOutputRes() + getDriver("INVZ2_RonZN")->getOutputRes()) / 2;

        double a_to_y_delay = 0.0;
        a_to_y_delay += getDriver("INV1_RonZN")->calculateDelay();
        a_to_y_delay += max(getDriver("INVZ1_RonZN")->calculateDelay(), getDriver("INVZ2_RonZN")->calculateDelay());
        
        double b_to_y_delay = 0.0;
        b_to_y_delay += max(getDriver("INVZ1_RonZN")->calculateDelay(), getDriver("INV2_RonZN")->calculateDelay() + getDriver("INVZ2_RonZN")->calculateDelay());
        
        cache->set(cell_name + "->DriveRes->Y", y_ron);    
        cache->set(cell_name + "->Delay->A_to_Y", a_to_y_delay);
        cache->set(cell_name + "->Delay->B_to_Y", b_to_y_delay);        
        Log::printLine(cell_name + "->DriveRes->Y=" + (String) y_ron);    
        Log::printLine(cell_name + "->Delay->A_to_Y=" + (String) a_to_y_delay);
        Log::printLine(cell_name + "->Delay->B_to_Y=" + (String) b_to_y_delay);
        // --------------------------------------------------------------------

        return;
    }        
    
} // namespace DSENT

