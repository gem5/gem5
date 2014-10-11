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

#include "model/std_cells/LATQ.h"

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
    using std::min;

    LATQ::LATQ(const String& instance_name_, const TechModel* tech_model_)
        : StdCell(instance_name_, tech_model_)
    {
        initProperties();
    }

    LATQ::~LATQ()
    {}

    void LATQ::initProperties()
    {
        return;
    }

    void LATQ::constructModel()
    {
        // All constructModel should do is create Area/NDDPower/Energy Results as
        // well as instantiate any sub-instances using only the hard parameters
        
        createInputPort("D");
        createInputPort("G");
        createOutputPort("Q");
        
        createLoad("D_Cap");
        createLoad("G_Cap");
        createDelay("D_to_Q_delay");
        createDelay("G_to_Q_delay");
        createDriver("Q_Ron", true);

        ElectricalLoad* d_cap = getLoad("D_Cap");
        ElectricalLoad* g_cap = getLoad("G_Cap");
        ElectricalDelay* d_to_q_delay = getDelay("D_to_Q_delay");
        ElectricalDelay* g_to_q_delay = getDelay("G_to_Q_delay");
        ElectricalDriver* q_ron = getDriver("Q_Ron");
        
        getNet("D")->addDownstreamNode(d_cap);
        getNet("G")->addDownstreamNode(g_cap);
        d_cap->addDownstreamNode(d_to_q_delay);        
        g_cap->addDownstreamNode(g_to_q_delay);
        g_to_q_delay->addDownstreamNode(q_ron);
        q_ron->addDownstreamNode(getNet("Q"));
        
        // Create Area result
        // Create NDD Power result
        createElectricalAtomicResults();
        // Create G Event Energy Result
        createElectricalEventAtomicResult("G");
        // Create DFF Event Energy Result
        createElectricalEventAtomicResult("LATD");
        createElectricalEventAtomicResult("LATQ");
        // Create Idle event for leakage
        // G pin is assumed to be on all the time
        //createElectricalEventAtomicResult("Idle");
        getEventInfo("Idle")->setStaticTransitionInfos();
        return;
    }
    
    void LATQ::updateModel()
    {
        // Get parameters        
        double drive_strength = getDrivingStrength();
        Map<double>* cache = getTechModel()->getStdCellLib()->getStdCellCache();

        // Standard cell cache string
        String cell_name = "LATQ_X" + (String) drive_strength;
        
        // Get timing parameters
        getLoad("D_Cap")->setLoadCap(cache->get(cell_name + "->Cap->D"));
        getLoad("G_Cap")->setLoadCap(cache->get(cell_name + "->Cap->G"));
        getDriver("Q_Ron")->setOutputRes(cache->get(cell_name + "->DriveRes->Q"));
        getDelay("G_to_Q_delay")->setDelay(cache->get(cell_name + "->Delay->G_to_Q"));
        getDelay("D_to_Q_delay")->setDelay(cache->get(cell_name + "->Delay->D_to_Q"));
        
        // Set the cell area
        getAreaResult("Active")->setValue(cache->get(cell_name + "->Area->Active"));
        getAreaResult("Metal1Wire")->setValue(cache->get(cell_name + "->Area->Metal1Wire"));
        
        return;
    }
    
    void LATQ::evaluateModel()
    {
        return;
    }

    void LATQ::useModel()
    {
        // Get parameters        
        double drive_strength = getDrivingStrength();
        Map<double>* cache = getTechModel()->getStdCellLib()->getStdCellCache();

        // Standard cell cache string
        String cell_name = "LATQ_X" + (String) drive_strength;

        // Propagate the transition info and get P_D, P_M, and P_Q
        propagateTransitionInfo();
        double P_D = getInputPort("D")->getTransitionInfo().getProbability1();
        double P_G = getInputPort("G")->getTransitionInfo().getProbability1();
        double P_Q = getOutputPort("Q")->getTransitionInfo().getProbability1();
        double G_num_trans_01 = getInputPort("G")->getTransitionInfo().getNumberTransitions01();
        double D_num_trans_01 = getInputPort("D")->getTransitionInfo().getNumberTransitions01();
        double Q_num_trans_01 = getOutputPort("Q")->getTransitionInfo().getNumberTransitions01();        
        
        // Calculate leakage
        double leakage = 0;
        leakage += cache->get(cell_name + "->Leakage->!D!G!Q") * (1 - P_D) * (1 - P_G) * (1 - P_Q);
        leakage += cache->get(cell_name + "->Leakage->!D!GQ") * (1 - P_D) * (1 - P_G) * P_Q;
        leakage += cache->get(cell_name + "->Leakage->!DG!Q") * (1 - P_D) * P_G * (1 - P_Q);
        leakage += cache->get(cell_name + "->Leakage->D!G!Q") * P_D * (1 - P_G) * (1 - P_Q);
        leakage += cache->get(cell_name + "->Leakage->D!GQ") * P_D * (1 - P_G) * P_Q;
        leakage += cache->get(cell_name + "->Leakage->DGQ") * P_D * P_G * P_Q;
        getNddPowerResult("Leakage")->setValue(leakage);
        
        // Get VDD
        double vdd = getTechModel()->get("Vdd");

        // Get capacitances
        double g_b_cap = cache->get(cell_name + "->Cap->G_b");
        double d_b_cap = cache->get(cell_name + "->Cap->D_b");
        double q_i_cap = cache->get(cell_name + "->Cap->Q_i");
        double q_b_cap = cache->get(cell_name + "->Cap->Q_b");
        double q_cap = cache->get(cell_name + "->Cap->Q");
        double q_load_cap = getNet("Q")->getTotalDownstreamCap();

        // Calculate G Event energy
        double g_event_energy = 0.0;
        g_event_energy += (g_b_cap) * G_num_trans_01;
        g_event_energy *= vdd * vdd;
        getEventResult("G")->setValue(g_event_energy);
        // Calculate LATD Event energy
        double latd_event_energy = 0.0;
        latd_event_energy += (d_b_cap) * D_num_trans_01;
        latd_event_energy *= vdd * vdd;
        getEventResult("LATD")->setValue(latd_event_energy);
        // Calculate LATQ Event energy
        double latq_event_energy = 0.0;
        latq_event_energy += (q_i_cap + q_b_cap + q_cap + q_load_cap) * Q_num_trans_01;
        latq_event_energy *= vdd * vdd;
        getEventResult("LATQ")->setValue(latq_event_energy);
        
        return;
    }
    
    void LATQ::propagateTransitionInfo()
    {
        const TransitionInfo& trans_G = getInputPort("G")->getTransitionInfo();
        const TransitionInfo& trans_D = getInputPort("D")->getTransitionInfo();

        double G_num_trans_01 = trans_G.getNumberTransitions01();
        double G_num_trans_10 = G_num_trans_01;
        double G_num_trans_00 = trans_G.getNumberTransitions00();
        double D_freq_mult = trans_D.getFrequencyMultiplier();
        
        // If the latch is sampling just as fast or faster than input data signal
        // Then it can capture all transitions (though it should be normalized to clock)
        if((G_num_trans_10 + G_num_trans_00) >= D_freq_mult)
        {
            const TransitionInfo& trans_Q = trans_D.scaleFrequencyMultiplier(G_num_trans_10 + G_num_trans_00);
            getOutputPort("Q")->setTransitionInfo(trans_Q); 
        }
        // If the latch is sampling slower than the input data signal, then input
        // will look like they transition more
        else
        {
            // Calculate scale ratio
            double scale_ratio = (G_num_trans_10 + G_num_trans_00) / D_freq_mult;
            // 00 and 11 transitions become fewer
            double D_scaled_diff = 0.5 * (1 - scale_ratio) * (trans_D.getNumberTransitions00() + trans_D.getNumberTransitions11());
            double D_scaled_num_trans_00 = trans_D.getNumberTransitions00() * scale_ratio;
            double D_scaled_num_trans_11 = trans_D.getNumberTransitions11() * scale_ratio;
            // 01 and 10 transitions become more frequent
            double D_scaled_num_trans_10 = trans_D.getNumberTransitions01() + D_scaled_diff;
        
            // Create final transition info, remembering to apply scaling ratio to normalize to G
            const TransitionInfo trans_Q(   D_scaled_num_trans_00 * scale_ratio,
                                            D_scaled_num_trans_10 * scale_ratio,
                                            D_scaled_num_trans_11 * scale_ratio);
            getOutputPort("Q")->setTransitionInfo(trans_Q);
        }

        return;
    }    
    
    // Creates the standard cell, characterizes and abstracts away the details
    void LATQ::cacheStdCell(StdCellLib* cell_lib_, double drive_strength_)
    {
        // Get parameters        
        double gate_pitch = cell_lib_->getTechModel()->get("Gate->PitchContacted");
        Map<double>* cache = cell_lib_->getStdCellCache();

        // Standard cell cache string
        String cell_name = "LATQ_X" + (String) drive_strength_;

        Log::printLine("=== " + cell_name + " ===");

        
        // Now actually build the full standard cell model
        createInputPort("D");
        createInputPort("G");
        createOutputPort("Q");
        
        createNet("D_b");
        createNet("Q_i");
        createNet("Q_b");
        createNet("G_b");
        
        // Adds macros
        CellMacros::addInverter(this, "INV1", false, true, "D", "D_b");
        CellMacros::addInverter(this, "INV2", false, true, "Q_i", "Q_b");
        CellMacros::addInverter(this, "INV3", false, true, "Q_b", "Q");
        CellMacros::addInverter(this, "INV4", false, true, "G", "G_b");
        CellMacros::addTristate(this, "INVZ1", false, true, false, false, "D_b", "G", "G_b", "Q_i");        //trace timing through A->ZN path only
        CellMacros::addTristate(this, "INVZ2", false, false, false, false, "Q_b", "G_b", "G", "Q_i");       //don't trace timing through the feedback path

        // Update macros
        CellMacros::updateInverter(this, "INV1", drive_strength_ * 0.125);
        CellMacros::updateInverter(this, "INV2", drive_strength_ * 0.5);
        CellMacros::updateInverter(this, "INV3", drive_strength_ * 1.0);
        CellMacros::updateInverter(this, "INV4", drive_strength_ * 0.125);
        CellMacros::updateTristate(this, "INVZ1", drive_strength_ * 0.5);
        CellMacros::updateTristate(this, "INVZ2", drive_strength_ * 0.0625);
                
        // Cache area result
        double area = 0.0;
        area += gate_pitch * getTotalHeight() * 1;
        area += gate_pitch * getTotalHeight() * getGenProperties()->get("INV1_GatePitches").toDouble();
        area += gate_pitch * getTotalHeight() * getGenProperties()->get("INV2_GatePitches").toDouble();
        area += gate_pitch * getTotalHeight() * getGenProperties()->get("INV3_GatePitches").toDouble();
        area += gate_pitch * getTotalHeight() * getGenProperties()->get("INV4_GatePitches").toDouble();
        area += gate_pitch * getTotalHeight() * getGenProperties()->get("INVZ1_GatePitches").toDouble();
        area += gate_pitch * getTotalHeight() * getGenProperties()->get("INVZ2_GatePitches").toDouble();
        cache->set(cell_name + "->Area->Active", area);
        cache->set(cell_name + "->Area->Metal1Wire", area);     //Cover-block m1 area
        Log::printLine(cell_name + "->Area->Active=" + (String) area);
        Log::printLine(cell_name + "->Area->Metal1Wire=" + (String) area);

        // --------------------------------------------------------------------
        // Leakage Model Calculation
        // --------------------------------------------------------------------
        // Cache leakage power results (for every single signal combination)
        double leakage_000 = 0;         //!D, !G, !Q
        double leakage_001 = 0;         //!D, !G, Q
        double leakage_010 = 0;         //!D, G, !Q
        double leakage_100 = 0;         //D, !G, !Q
        double leakage_101 = 0;         //D, !G, Q
        double leakage_111 = 0;         //D, G, Q

        //This is so painful...
        leakage_000 += getGenProperties()->get("INV1_LeakagePower_0").toDouble();
        leakage_000 += getGenProperties()->get("INV2_LeakagePower_0").toDouble();
        leakage_000 += getGenProperties()->get("INV3_LeakagePower_1").toDouble();
        leakage_000 += getGenProperties()->get("INV4_LeakagePower_0").toDouble();
        leakage_000 += getGenProperties()->get("INVZ1_LeakagePower_011_0").toDouble();
        leakage_000 += getGenProperties()->get("INVZ2_LeakagePower_101_0").toDouble();

        leakage_001 += getGenProperties()->get("INV1_LeakagePower_0").toDouble();
        leakage_001 += getGenProperties()->get("INV2_LeakagePower_0").toDouble();
        leakage_001 += getGenProperties()->get("INV3_LeakagePower_0").toDouble();
        leakage_001 += getGenProperties()->get("INV4_LeakagePower_0").toDouble();
        leakage_001 += getGenProperties()->get("INVZ1_LeakagePower_011_1").toDouble();
        leakage_001 += getGenProperties()->get("INVZ2_LeakagePower_100_1").toDouble();

        leakage_010 += getGenProperties()->get("INV1_LeakagePower_0").toDouble();
        leakage_010 += getGenProperties()->get("INV2_LeakagePower_0").toDouble();
        leakage_010 += getGenProperties()->get("INV3_LeakagePower_1").toDouble();
        leakage_010 += getGenProperties()->get("INV4_LeakagePower_1").toDouble();
        leakage_010 += getGenProperties()->get("INVZ1_LeakagePower_101_0").toDouble();
        leakage_010 += getGenProperties()->get("INVZ2_LeakagePower_011_0").toDouble();

        leakage_100 += getGenProperties()->get("INV1_LeakagePower_1").toDouble();
        leakage_100 += getGenProperties()->get("INV2_LeakagePower_1").toDouble();
        leakage_100 += getGenProperties()->get("INV3_LeakagePower_1").toDouble();
        leakage_100 += getGenProperties()->get("INV4_LeakagePower_0").toDouble();
        leakage_100 += getGenProperties()->get("INVZ1_LeakagePower_010_0").toDouble();
        leakage_100 += getGenProperties()->get("INVZ2_LeakagePower_101_0").toDouble();
        
        leakage_101 += getGenProperties()->get("INV1_LeakagePower_1").toDouble();
        leakage_101 += getGenProperties()->get("INV2_LeakagePower_1").toDouble();
        leakage_101 += getGenProperties()->get("INV3_LeakagePower_0").toDouble();
        leakage_101 += getGenProperties()->get("INV4_LeakagePower_0").toDouble();
        leakage_101 += getGenProperties()->get("INVZ1_LeakagePower_010_1").toDouble();
        leakage_101 += getGenProperties()->get("INVZ2_LeakagePower_100_1").toDouble();

        leakage_111 += getGenProperties()->get("INV1_LeakagePower_1").toDouble();
        leakage_111 += getGenProperties()->get("INV2_LeakagePower_1").toDouble();
        leakage_111 += getGenProperties()->get("INV3_LeakagePower_0").toDouble();
        leakage_111 += getGenProperties()->get("INV4_LeakagePower_1").toDouble();
        leakage_111 += getGenProperties()->get("INVZ1_LeakagePower_100_1").toDouble();
        leakage_111 += getGenProperties()->get("INVZ2_LeakagePower_010_1").toDouble();
        
        cache->set(cell_name + "->Leakage->!D!G!Q", leakage_000);
        cache->set(cell_name + "->Leakage->!D!GQ", leakage_001);
        cache->set(cell_name + "->Leakage->!DG!Q", leakage_010);
        cache->set(cell_name + "->Leakage->D!G!Q", leakage_100);
        cache->set(cell_name + "->Leakage->D!GQ", leakage_101);
        cache->set(cell_name + "->Leakage->DGQ", leakage_111);
        Log::printLine(cell_name + "->Leakage->!D!G!Q=" + (String) leakage_000);
        Log::printLine(cell_name + "->Leakage->!D!GQ=" + (String) leakage_001);
        Log::printLine(cell_name + "->Leakage->!DG!Q=" + (String) leakage_010);
        Log::printLine(cell_name + "->Leakage->D!G!Q=" + (String) leakage_100);
        Log::printLine(cell_name + "->Leakage->D!GQ=" + (String) leakage_101);
        Log::printLine(cell_name + "->Leakage->DGQ=" + (String) leakage_111);
        // --------------------------------------------------------------------
        
        // --------------------------------------------------------------------
        // Get Node Capacitances
        // --------------------------------------------------------------------
        double d_cap = getNet("D")->getTotalDownstreamCap();
        double d_b_cap = getNet("D_b")->getTotalDownstreamCap();
        double q_i_cap = getNet("Q_i")->getTotalDownstreamCap();
        double q_b_cap = getNet("Q_b")->getTotalDownstreamCap();
        double q_cap = getNet("Q")->getTotalDownstreamCap();
        double g_cap = getNet("G")->getTotalDownstreamCap();
        double g_b_cap = getNet("G_b")->getTotalDownstreamCap();

        cache->set(cell_name + "->Cap->D", d_cap);
        cache->set(cell_name + "->Cap->D_b", d_b_cap);
        cache->set(cell_name + "->Cap->Q_i", q_i_cap);
        cache->set(cell_name + "->Cap->Q_b", q_b_cap);
        cache->set(cell_name + "->Cap->Q", q_cap);
        cache->set(cell_name + "->Cap->G", g_cap);
        cache->set(cell_name + "->Cap->G_b", g_b_cap);
        
        Log::printLine(cell_name + "->Cap->D=" + (String) d_cap);
        Log::printLine(cell_name + "->Cap->D_b=" + (String) d_b_cap);        
        Log::printLine(cell_name + "->Cap->Q_i=" + (String) q_i_cap);
        Log::printLine(cell_name + "->Cap->Q_b=" + (String) q_b_cap);        
        Log::printLine(cell_name + "->Cap->Q=" + (String) q_cap);
        Log::printLine(cell_name + "->Cap->G=" + (String) g_cap);        
        Log::printLine(cell_name + "->Cap->G_b=" + (String) g_b_cap);        
        // --------------------------------------------------------------------

        // --------------------------------------------------------------------
        // Build Internal Delay Model
        // --------------------------------------------------------------------
        double q_ron = getDriver("INV3_RonZN")->getOutputRes();

        double d_to_q_delay = getDriver("INV1_RonZN")->calculateDelay() + 
                                getDriver("INVZ1_RonZN")->calculateDelay() +
                                getDriver("INV2_RonZN")->calculateDelay() + 
                                getDriver("INV3_RonZN")->calculateDelay();
        double g_to_q_delay = getDriver("INV4_RonZN")->calculateDelay() +
                                getDriver("INVZ1_RonZN")->calculateDelay() +
                                getDriver("INV2_RonZN")->calculateDelay() +
                                getDriver("INV3_RonZN")->calculateDelay();        

        cache->set(cell_name + "->DriveRes->Q", q_ron);        
        cache->set(cell_name + "->Delay->D_to_Q", d_to_q_delay);
        cache->set(cell_name + "->Delay->G_to_Q", g_to_q_delay);
        Log::printLine(cell_name + "->DriveRes->Q=" + (String) q_ron);        
        Log::printLine(cell_name + "->Delay->D_to_Q=" + (String) d_to_q_delay);
        Log::printLine(cell_name + "->Delay->G_to_Q=" + (String) g_to_q_delay);                

        return;
        // --------------------------------------------------------------------

    }    
    
} // namespace DSENT

