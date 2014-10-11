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

#include "model/std_cells/DFFQ.h"

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
    using std::min;

    DFFQ::DFFQ(const String& instance_name_, const TechModel* tech_model_)
        : StdCell(instance_name_, tech_model_)
    {
        initProperties();
    }

    DFFQ::~DFFQ()
    {}

    void DFFQ::initProperties()
    {
        return;
    }

    void DFFQ::constructModel()
    {
        // All constructModel should do is create Area/NDDPower/Energy Results as
        // well as instantiate any sub-instances using only the hard parameters
        
        createInputPort("D");
        createInputPort("CK");
        createOutputPort("Q");
        
        createLoad("D_Cap");
        createLoad("CK_Cap");
        createDelay("D_Setup_delay");
        createDelay("CK_to_Q_delay");
        createDriver("Q_Ron", true);

        ElectricalLoad* d_cap = getLoad("D_Cap");
        ElectricalLoad* ck_cap = getLoad("CK_Cap");
        ElectricalDelay* d_setup_delay = getDelay("D_Setup_delay");
        ElectricalDelay* ck_to_q_delay = getDelay("CK_to_Q_delay");
        ElectricalDriver* q_ron = getDriver("Q_Ron");
        
        getNet("D")->addDownstreamNode(d_cap);
        getNet("CK")->addDownstreamNode(ck_cap);
        d_cap->addDownstreamNode(d_setup_delay);        
        ck_cap->addDownstreamNode(ck_to_q_delay);
        ck_to_q_delay->addDownstreamNode(q_ron);
        q_ron->addDownstreamNode(getNet("Q"));
        
        // Create Area result
        // Create NDD Power result
        createElectricalAtomicResults();
        // Create CK Event Energy Result
        createElectricalEventAtomicResult("CK");
        getEventInfo("CK")->setTransitionInfo("CK", TransitionInfo(0.0, 1.0, 0.0));
        // Create DFF Event Energy Result
        createElectricalEventAtomicResult("DFFD");
        getEventInfo("DFFD")->setTransitionInfo("CK", TransitionInfo(0.0, 1.0, 0.0));
        createElectricalEventAtomicResult("DFFQ");
        getEventInfo("DFFQ")->setTransitionInfo("CK", TransitionInfo(0.0, 1.0, 0.0));

        // Update Idle event for leakage
        // CK pin is assumed to be on all the time
        EventInfo* idle_event_info = getEventInfo("Idle");
        idle_event_info->setTransitionInfo("CK", TransitionInfo(0.0, 1.0, 0.0));
        idle_event_info->setTransitionInfo("D", TransitionInfo(0.5, 0.0, 0.5));
        
        return;
    }
    
    void DFFQ::updateModel()
    {
        // Get parameters        
        double drive_strength = getDrivingStrength();
        Map<double>* cache = getTechModel()->getStdCellLib()->getStdCellCache();

        // Standard cell cache string
        String cell_name = "DFFQ_X" + (String) drive_strength;
        
        // Get timing parameters
        getLoad("D_Cap")->setLoadCap(cache->get(cell_name + "->Cap->D"));
        getLoad("CK_Cap")->setLoadCap(cache->get(cell_name + "->Cap->CK"));
        getDriver("Q_Ron")->setOutputRes(cache->get(cell_name + "->DriveRes->Q"));
        getDelay("CK_to_Q_delay")->setDelay(cache->get(cell_name + "->Delay->CK_to_Q"));
        getDelay("D_Setup_delay")->setDelay(cache->get(cell_name + "->Delay->D_Setup"));
        
        // Set the cell area
        getAreaResult("Active")->setValue(cache->get(cell_name + "->Area->Active"));
        getAreaResult("Metal1Wire")->setValue(cache->get(cell_name + "->Area->Metal1Wire"));
        
        return;
    }
    
    void DFFQ::evaluateModel()
    {
        return;
    }

    void DFFQ::useModel()
    {
        // Get parameters        
        double drive_strength = getDrivingStrength();
        Map<double>* cache = getTechModel()->getStdCellLib()->getStdCellCache();

        // Standard cell cache string
        String cell_name = "DFFQ_X" + (String) drive_strength;

        // Propagate the transition info and get P_D, P_M, and P_Q
        propagateTransitionInfo();
        double P_D = getInputPort("D")->getTransitionInfo().getProbability1();
        double P_CK = getInputPort("CK")->getTransitionInfo().getProbability1();
        double P_Q = getOutputPort("Q")->getTransitionInfo().getProbability1();
        double CK_num_trans_01 = getInputPort("CK")->getTransitionInfo().getNumberTransitions01();
        double D_num_trans_01 = getInputPort("D")->getTransitionInfo().getNumberTransitions01();
        double M_num_trans_01 = m_trans_M_.getNumberTransitions01();
        double Q_num_trans_01 = getOutputPort("Q")->getTransitionInfo().getNumberTransitions01();

        // Calculate leakage
        double leakage = 0;
        leakage += cache->get(cell_name + "->Leakage->!D!CK!Q") * (1 - P_D) * (1 - P_CK) * (1 - P_Q);
        leakage += cache->get(cell_name + "->Leakage->!D!CKQ") * (1 - P_D) * (1 - P_CK) * P_Q;
        leakage += cache->get(cell_name + "->Leakage->!DCK!Q") * (1 - P_D) * P_CK * (1 - P_Q);
        leakage += cache->get(cell_name + "->Leakage->!DCKQ") * (1 - P_D) * P_CK * P_Q;
        leakage += cache->get(cell_name + "->Leakage->D!CK!Q") * P_D * (1 - P_CK) * (1 - P_Q);
        leakage += cache->get(cell_name + "->Leakage->D!CKQ") * P_D * (1 - P_CK) * P_Q;
        leakage += cache->get(cell_name + "->Leakage->DCK!Q") * P_D * P_CK * (1 - P_Q);
        leakage += cache->get(cell_name + "->Leakage->DCKQ") * P_D * P_CK * P_Q;
        getNddPowerResult("Leakage")->setValue(leakage);
        
        // Get VDD
        double vdd = getTechModel()->get("Vdd");

        // Get capacitances
        double ck_b_cap = cache->get(cell_name + "->Cap->CK_b");
        double ck_i_cap = cache->get(cell_name + "->Cap->CK_i");
        double d_b_cap = cache->get(cell_name + "->Cap->D_b");
        double m_b_cap = cache->get(cell_name + "->Cap->M_b");
        double m_cap = cache->get(cell_name + "->Cap->M");
        double m_i_cap = cache->get(cell_name + "->Cap->M_i");
        double q_b_cap = cache->get(cell_name + "->Cap->Q_b");
        double q_cap = cache->get(cell_name + "->Cap->Q");
        double q_load_cap = getNet("Q")->getTotalDownstreamCap();
        
        // Calculate CK Event energy
        double ck_event_energy = 0.0;
        ck_event_energy += (ck_b_cap + ck_i_cap) * CK_num_trans_01;
        ck_event_energy *= vdd * vdd;
        getEventResult("CK")->setValue(ck_event_energy);
        // Calculate DFFD Event energy
        double dffd_event_energy = 0.0;
        dffd_event_energy += (d_b_cap) * D_num_trans_01;
        dffd_event_energy += (m_b_cap + m_cap) * M_num_trans_01;
        dffd_event_energy *= vdd * vdd;
        getEventResult("DFFD")->setValue(dffd_event_energy);
        // Calculate DFFQ Event energy
        double dffq_event_energy = 0.0;
        dffq_event_energy += (m_i_cap + q_b_cap + q_cap + q_load_cap) * Q_num_trans_01;
        dffq_event_energy *= vdd * vdd;
        getEventResult("DFFQ")->setValue(dffq_event_energy);

        return;
    }

    void DFFQ::propagateTransitionInfo()
    {
        const TransitionInfo& trans_CK = getInputPort("CK")->getTransitionInfo();
        const TransitionInfo& trans_D = getInputPort("D")->getTransitionInfo();

        double CK_num_trans_01 = trans_CK.getNumberTransitions01();
        double CK_num_trans_10 = CK_num_trans_01;
        double CK_num_trans_00 = trans_CK.getNumberTransitions00();
        double D_freq_mult = trans_D.getFrequencyMultiplier();
                
        // If thre is no activity on the clock or D, assume M node is randomly distributed among 0 and 1
        if(LibUtil::Math::isEqual(CK_num_trans_10 + CK_num_trans_00, 0.0) || LibUtil::Math::isEqual(D_freq_mult, 0.0))
        {
            m_trans_M_ = TransitionInfo(0.5, 0.0, 0.5);
        }
        // If the master latch is sampling just as fast or faster than input data signal
        // Then it can capture all transitions (though it should be normalized to clock)
        else if((CK_num_trans_10 + CK_num_trans_00) >= D_freq_mult)
        {
            m_trans_M_ = trans_D.scaleFrequencyMultiplier(CK_num_trans_10 + CK_num_trans_00);
        }
        // If the master latch is sampling slower than the input data signal, then input
        // will look like they transition more
        else
        {
            // Calculate scale ratio
            double scale_ratio = (CK_num_trans_10 + CK_num_trans_00) / D_freq_mult;
            // 00 and 11 transitions become fewer
            double D_scaled_diff = 0.5 * (1 - scale_ratio) * (trans_D.getNumberTransitions00() + trans_D.getNumberTransitions11());
            double D_scaled_num_trans_00 = trans_D.getNumberTransitions00() * scale_ratio;
            double D_scaled_num_trans_11 = trans_D.getNumberTransitions11() * scale_ratio;
            // 01 and 10 transitions become more frequent
            double D_scaled_num_trans_10 = trans_D.getNumberTransitions01() + D_scaled_diff;
        
            // Create final transition info, remembering to apply scaling ratio to normalize to CK
            m_trans_M_ = TransitionInfo(D_scaled_num_trans_00 * scale_ratio,
                                        D_scaled_num_trans_10 * scale_ratio,
                                        D_scaled_num_trans_11 * scale_ratio);
        }

        // If the clock activity is 0 or if D activity is 0, then we assume that the output is randomly distributed among 0 and 1
        if(LibUtil::Math::isEqual(CK_num_trans_01, 0.0) || LibUtil::Math::isEqual(D_freq_mult, 0.0))
        {
            getOutputPort("Q")->setTransitionInfo(TransitionInfo(0.5, 0.0, 0.5));
        }
        // If the DFF's CK is running at a higher frequency than D, Q is just D with a
        // scaled up frequency multiplier
        else if(CK_num_trans_01 >= D_freq_mult)
        {
            const TransitionInfo& trans_Q = trans_D.scaleFrequencyMultiplier(CK_num_trans_01);  
            getOutputPort("Q")->setTransitionInfo(trans_Q); 
        }
        // If the DFF is sampling slower than the input data signal, then inputs
        // will look like they transition more
        else
        {
            // Calculate scale ratio
            double scale_ratio = CK_num_trans_01 / D_freq_mult;
            // 00 and 11 transitions become fewer
            double D_scaled_diff = 0.5 * (1 - scale_ratio) * (trans_D.getNumberTransitions00() + trans_D.getNumberTransitions11());
            double D_scaled_num_trans_00 = trans_D.getNumberTransitions00() * scale_ratio;
            double D_scaled_num_trans_11 = trans_D.getNumberTransitions11() * scale_ratio;
            // 01 and 10 transitions become more frequent
            double D_scaled_num_trans_10 = trans_D.getNumberTransitions01() + D_scaled_diff;
            const TransitionInfo trans_Q(   D_scaled_num_trans_00 * scale_ratio,
                    D_scaled_num_trans_10 * scale_ratio,
                    D_scaled_num_trans_11 * scale_ratio);
            getOutputPort("Q")->setTransitionInfo(trans_Q);
        }        
        return;
    }

    // Creates the standard cell, characterizes and abstracts away the details
    void DFFQ::cacheStdCell(StdCellLib* cell_lib_, double drive_strength_)
    {
        // Get parameters        
        double gate_pitch = cell_lib_->getTechModel()->get("Gate->PitchContacted");
        Map<double>* cache = cell_lib_->getStdCellCache();

        // Standard cell cache string
        String cell_name = "DFFQ_X" + (String) drive_strength_;

        Log::printLine("=== " + cell_name + " ===");


        // Now actually build the full standard cell model
        createInputPort("D");
        createInputPort("CK");
        createOutputPort("Q");

        createNet("D_b");
        createNet("M_b");
        createNet("M");
        createNet("M_i");
        createNet("Q_b");
        createNet("CK_b");
        createNet("CK_i");

        // Adds macros
        CellMacros::addInverter(this, "INV1", false, true, "D", "D_b");
        CellMacros::addInverter(this, "INV2", false, true, "M_b", "M");
        CellMacros::addInverter(this, "INV3", false, true, "M_i", "Q_b");
        CellMacros::addInverter(this, "INV4", true, true, "Q_b", "Q");
        CellMacros::addInverter(this, "INV5", false, true, "CK", "CK_b");
        CellMacros::addInverter(this, "INV6", false, true, "CK_b", "CK_i");
        CellMacros::addTristate(this, "INVZ1", false, true, false, false, "D_b", "CK_b", "CK_i", "M_b");        //trace timing through A->ZN path only
        CellMacros::addTristate(this, "INVZ2", false, false, false, false, "M", "CK_i", "CK_b", "M_b");         //don't trace timing through the feedback path
        CellMacros::addTristate(this, "INVZ3", false, false, true, true, "M", "CK_i", "CK_b", "M_i");           //trace timing from OE->ZN and OEN->ZN paths only
        CellMacros::addTristate(this, "INVZ4", false, false, false, false, "Q_b", "CK_b", "CK_i", "M_i");       //don't trace timing through the feedback path

        // Update macros
        CellMacros::updateInverter(this, "INV1", drive_strength_ * 0.125);
        CellMacros::updateInverter(this, "INV2", drive_strength_ * 0.5);
        CellMacros::updateInverter(this, "INV3", drive_strength_ * 0.5);
        CellMacros::updateInverter(this, "INV4", drive_strength_ * 1.0);
        CellMacros::updateInverter(this, "INV5", drive_strength_ * 0.125);
        CellMacros::updateInverter(this, "INV6", drive_strength_ * 0.125);
        CellMacros::updateTristate(this, "INVZ1", drive_strength_ * 0.5);
        CellMacros::updateTristate(this, "INVZ2", drive_strength_ * 0.0625);
        CellMacros::updateTristate(this, "INVZ3", drive_strength_ * 0.5);
        CellMacros::updateTristate(this, "INVZ4", drive_strength_ * 0.0625);

        // Cache area result
        double area = 0.0;
        area += gate_pitch * getTotalHeight() * 1;
        area += gate_pitch * getTotalHeight() * getGenProperties()->get("INV1_GatePitches").toDouble();
        area += gate_pitch * getTotalHeight() * getGenProperties()->get("INV2_GatePitches").toDouble();
        area += gate_pitch * getTotalHeight() * getGenProperties()->get("INV3_GatePitches").toDouble();
        area += gate_pitch * getTotalHeight() * getGenProperties()->get("INV4_GatePitches").toDouble();
        area += gate_pitch * getTotalHeight() * getGenProperties()->get("INV5_GatePitches").toDouble();
        area += gate_pitch * getTotalHeight() * getGenProperties()->get("INV6_GatePitches").toDouble();
        area += gate_pitch * getTotalHeight() * getGenProperties()->get("INVZ1_GatePitches").toDouble();
        area += gate_pitch * getTotalHeight() * getGenProperties()->get("INVZ2_GatePitches").toDouble();
        area += gate_pitch * getTotalHeight() * getGenProperties()->get("INVZ3_GatePitches").toDouble();
        area += gate_pitch * getTotalHeight() * getGenProperties()->get("INVZ4_GatePitches").toDouble();
        cache->set(cell_name + "->Area->Active", area);
        cache->set(cell_name + "->Area->Metal1Wire", area);
        Log::printLine(cell_name + "->Area->Active=" + (String) area);
        Log::printLine(cell_name + "->Area->Metal1Wire=" + (String) area);

        // --------------------------------------------------------------------
        // Leakage Model Calculation
        // --------------------------------------------------------------------
        // Cache leakage power results (for every single signal combination)
        double leakage_000 = 0;         //!D, !CK, !Q
        double leakage_001 = 0;         //!D, !CK, Q
        double leakage_010 = 0;         //!D, CK, !Q
        double leakage_011 = 0;         //!D, CK, Q
        double leakage_100 = 0;         //D, !CK, !Q
        double leakage_101 = 0;         //D, !CK, Q
        double leakage_110 = 0;         //D, CK, !Q
        double leakage_111 = 0;         //D, CK, Q

        //This is so painful...
        leakage_000 += getGenProperties()->get("INV1_LeakagePower_0").toDouble();
        leakage_000 += getGenProperties()->get("INV2_LeakagePower_0").toDouble();
        leakage_000 += getGenProperties()->get("INV3_LeakagePower_0").toDouble();
        leakage_000 += getGenProperties()->get("INV4_LeakagePower_1").toDouble();
        leakage_000 += getGenProperties()->get("INV5_LeakagePower_0").toDouble();
        leakage_000 += getGenProperties()->get("INV6_LeakagePower_1").toDouble();
        leakage_000 += getGenProperties()->get("INVZ1_LeakagePower_101_0").toDouble();
        leakage_000 += getGenProperties()->get("INVZ2_LeakagePower_011_0").toDouble();
        leakage_000 += getGenProperties()->get("INVZ3_LeakagePower_011_0").toDouble();
        leakage_000 += getGenProperties()->get("INVZ4_LeakagePower_101_0").toDouble();

        leakage_001 += getGenProperties()->get("INV1_LeakagePower_0").toDouble();
        leakage_001 += getGenProperties()->get("INV2_LeakagePower_0").toDouble();
        leakage_001 += getGenProperties()->get("INV3_LeakagePower_1").toDouble();
        leakage_001 += getGenProperties()->get("INV4_LeakagePower_0").toDouble();
        leakage_001 += getGenProperties()->get("INV5_LeakagePower_0").toDouble();
        leakage_001 += getGenProperties()->get("INV6_LeakagePower_1").toDouble();
        leakage_001 += getGenProperties()->get("INVZ1_LeakagePower_101_0").toDouble();
        leakage_001 += getGenProperties()->get("INVZ2_LeakagePower_011_0").toDouble();
        leakage_001 += getGenProperties()->get("INVZ3_LeakagePower_011_1").toDouble();
        leakage_001 += getGenProperties()->get("INVZ4_LeakagePower_100_1").toDouble();

        leakage_010 += getGenProperties()->get("INV1_LeakagePower_0").toDouble();
        leakage_010 += getGenProperties()->get("INV2_LeakagePower_0").toDouble();
        leakage_010 += getGenProperties()->get("INV3_LeakagePower_0").toDouble();
        leakage_010 += getGenProperties()->get("INV4_LeakagePower_1").toDouble();
        leakage_010 += getGenProperties()->get("INV5_LeakagePower_1").toDouble();
        leakage_010 += getGenProperties()->get("INV6_LeakagePower_0").toDouble();
        leakage_010 += getGenProperties()->get("INVZ1_LeakagePower_011_0").toDouble();
        leakage_010 += getGenProperties()->get("INVZ2_LeakagePower_101_0").toDouble();
        leakage_010 += getGenProperties()->get("INVZ3_LeakagePower_101_0").toDouble();
        leakage_010 += getGenProperties()->get("INVZ4_LeakagePower_011_0").toDouble();

        leakage_011 += getGenProperties()->get("INV1_LeakagePower_0").toDouble();
        leakage_011 += getGenProperties()->get("INV2_LeakagePower_1").toDouble();
        leakage_011 += getGenProperties()->get("INV3_LeakagePower_1").toDouble();
        leakage_011 += getGenProperties()->get("INV4_LeakagePower_0").toDouble();
        leakage_011 += getGenProperties()->get("INV5_LeakagePower_1").toDouble();
        leakage_011 += getGenProperties()->get("INV6_LeakagePower_0").toDouble();
        leakage_011 += getGenProperties()->get("INVZ1_LeakagePower_011_1").toDouble();
        leakage_011 += getGenProperties()->get("INVZ2_LeakagePower_100_1").toDouble();
        leakage_011 += getGenProperties()->get("INVZ3_LeakagePower_100_1").toDouble();
        leakage_011 += getGenProperties()->get("INVZ4_LeakagePower_010_1").toDouble();

        leakage_100 += getGenProperties()->get("INV1_LeakagePower_1").toDouble();
        leakage_100 += getGenProperties()->get("INV2_LeakagePower_1").toDouble();
        leakage_100 += getGenProperties()->get("INV3_LeakagePower_0").toDouble();
        leakage_100 += getGenProperties()->get("INV4_LeakagePower_1").toDouble();
        leakage_100 += getGenProperties()->get("INV5_LeakagePower_0").toDouble();
        leakage_100 += getGenProperties()->get("INV6_LeakagePower_1").toDouble();
        leakage_100 += getGenProperties()->get("INVZ1_LeakagePower_100_1").toDouble();
        leakage_100 += getGenProperties()->get("INVZ2_LeakagePower_010_1").toDouble();
        leakage_100 += getGenProperties()->get("INVZ3_LeakagePower_010_0").toDouble();
        leakage_100 += getGenProperties()->get("INVZ4_LeakagePower_101_0").toDouble();

        leakage_101 += getGenProperties()->get("INV1_LeakagePower_1").toDouble();
        leakage_101 += getGenProperties()->get("INV2_LeakagePower_1").toDouble();
        leakage_101 += getGenProperties()->get("INV3_LeakagePower_1").toDouble();
        leakage_101 += getGenProperties()->get("INV4_LeakagePower_0").toDouble();
        leakage_101 += getGenProperties()->get("INV5_LeakagePower_0").toDouble();
        leakage_101 += getGenProperties()->get("INV6_LeakagePower_1").toDouble();
        leakage_101 += getGenProperties()->get("INVZ1_LeakagePower_100_1").toDouble();
        leakage_101 += getGenProperties()->get("INVZ2_LeakagePower_010_1").toDouble();
        leakage_101 += getGenProperties()->get("INVZ3_LeakagePower_010_1").toDouble();
        leakage_101 += getGenProperties()->get("INVZ4_LeakagePower_100_1").toDouble();

        leakage_110 += getGenProperties()->get("INV1_LeakagePower_1").toDouble();
        leakage_110 += getGenProperties()->get("INV2_LeakagePower_0").toDouble();
        leakage_110 += getGenProperties()->get("INV3_LeakagePower_0").toDouble();
        leakage_110 += getGenProperties()->get("INV4_LeakagePower_1").toDouble();
        leakage_110 += getGenProperties()->get("INV5_LeakagePower_1").toDouble();
        leakage_110 += getGenProperties()->get("INV6_LeakagePower_0").toDouble();
        leakage_110 += getGenProperties()->get("INVZ1_LeakagePower_010_0").toDouble();
        leakage_110 += getGenProperties()->get("INVZ2_LeakagePower_101_0").toDouble();
        leakage_110 += getGenProperties()->get("INVZ3_LeakagePower_101_0").toDouble();
        leakage_110 += getGenProperties()->get("INVZ4_LeakagePower_011_0").toDouble();

        leakage_111 += getGenProperties()->get("INV1_LeakagePower_1").toDouble();
        leakage_111 += getGenProperties()->get("INV2_LeakagePower_1").toDouble();
        leakage_111 += getGenProperties()->get("INV3_LeakagePower_1").toDouble();
        leakage_111 += getGenProperties()->get("INV4_LeakagePower_0").toDouble();
        leakage_111 += getGenProperties()->get("INV5_LeakagePower_1").toDouble();
        leakage_111 += getGenProperties()->get("INV6_LeakagePower_0").toDouble();
        leakage_111 += getGenProperties()->get("INVZ1_LeakagePower_010_1").toDouble();
        leakage_111 += getGenProperties()->get("INVZ2_LeakagePower_100_1").toDouble();
        leakage_111 += getGenProperties()->get("INVZ3_LeakagePower_100_1").toDouble();
        leakage_111 += getGenProperties()->get("INVZ4_LeakagePower_010_1").toDouble();

        cache->set(cell_name + "->Leakage->!D!CK!Q", leakage_000);
        cache->set(cell_name + "->Leakage->!D!CKQ", leakage_001);
        cache->set(cell_name + "->Leakage->!DCK!Q", leakage_010);
        cache->set(cell_name + "->Leakage->!DCKQ", leakage_011);
        cache->set(cell_name + "->Leakage->D!CK!Q", leakage_100);
        cache->set(cell_name + "->Leakage->D!CKQ", leakage_101);
        cache->set(cell_name + "->Leakage->DCK!Q", leakage_110);
        cache->set(cell_name + "->Leakage->DCKQ", leakage_111);
        Log::printLine(cell_name + "->Leakage->!D!CK!Q=" + (String) leakage_000);
        Log::printLine(cell_name + "->Leakage->!D!CKQ=" + (String) leakage_001);
        Log::printLine(cell_name + "->Leakage->!DCK!Q=" + (String) leakage_010);
        Log::printLine(cell_name + "->Leakage->!DCKQ=" + (String) leakage_011);
        Log::printLine(cell_name + "->Leakage->D!CK!Q=" + (String) leakage_100);
        Log::printLine(cell_name + "->Leakage->D!CKQ=" + (String) leakage_101);
        Log::printLine(cell_name + "->Leakage->DCK!Q=" + (String) leakage_110);
        Log::printLine(cell_name + "->Leakage->DCKQ=" + (String) leakage_111);
        // --------------------------------------------------------------------

        /*
        // Cache event energy results
        double event_ck_flip = 0.0;
        event_ck_flip += getGenProperties()->get("INV5_A_Flip").toDouble() + getGenProperties()->get("INV5_ZN_Flip").toDouble();
        event_ck_flip += getGenProperties()->get("INV6_A_Flip").toDouble() + getGenProperties()->get("INV6_ZN_Flip").toDouble();
        event_ck_flip += getGenProperties()->get("INVZ1_OE_Flip").toDouble() + getGenProperties()->get("INVZ1_OEN_Flip").toDouble();
        event_ck_flip += getGenProperties()->get("INVZ2_OE_Flip").toDouble() + getGenProperties()->get("INVZ2_OEN_Flip").toDouble();
        event_ck_flip += getGenProperties()->get("INVZ3_OE_Flip").toDouble() + getGenProperties()->get("INVZ3_OEN_Flip").toDouble();
        event_ck_flip += getGenProperties()->get("INVZ4_OE_Flip").toDouble() + getGenProperties()->get("INVZ4_OEN_Flip").toDouble();
        cache->set(cell_name + "->Event_CK_Flip", event_ck_flip);
        Log::printLine(cell_name + "->Event_CK_Flip=" + (String) event_ck_flip);

        // Update D flip results
        double event_d_flip = 0.0;
        event_d_flip += getGenProperties()->get("INV1_A_Flip").toDouble() + getGenProperties()->get("INV1_ZN_Flip").toDouble();
        event_d_flip += getGenProperties()->get("INVZ1_A_Flip").toDouble();
        cache->set(cell_name + "->Event_D_Flip", event_d_flip);
        Log::printLine(cell_name + "->Event_D_Flip=" + (String) event_d_flip);
        // Update M flip results
        double event_m_flip = 0.0;
        event_m_flip += getGenProperties()->get("INVZ1_ZN_Flip").toDouble();
        event_m_flip += getGenProperties()->get("INV2_A_Flip").toDouble() + getGenProperties()->get("INV2_ZN_Flip").toDouble();
        event_m_flip += getGenProperties()->get("INVZ2_A_Flip").toDouble() + getGenProperties()->get("INVZ2_ZN_Flip").toDouble();
        event_m_flip += getGenProperties()->get("INVZ3_A_Flip").toDouble();        
        cache->set(cell_name + "->Event_M_Flip", event_m_flip);
        Log::printLine(cell_name + "->Event_M_Flip=" + (String) event_m_flip);
        // Update Q flip results
        double event_q_flip = 0.0;
        event_q_flip += getGenProperties()->get("INVZ3_ZN_Flip").toDouble();
        event_q_flip += getGenProperties()->get("INV3_A_Flip").toDouble() + getGenProperties()->get("INV3_ZN_Flip").toDouble();
        event_q_flip += getGenProperties()->get("INVZ4_A_Flip").toDouble() + getGenProperties()->get("INVZ4_ZN_Flip").toDouble();
        event_q_flip += getGenProperties()->get("INV4_A_Flip").toDouble() + getGenProperties()->get("INV4_ZN_Flip").toDouble();
        cache->set(cell_name + "->Event_Q_Flip", event_q_flip);
        Log::printLine(cell_name + "->Event_Q_Flip=" + (String) event_q_flip);
         */

        // --------------------------------------------------------------------
        // Get Node Capacitances
        // --------------------------------------------------------------------
        double d_cap = getNet("D")->getTotalDownstreamCap();
        double d_b_cap = getNet("D_b")->getTotalDownstreamCap();
        double m_b_cap = getNet("M_b")->getTotalDownstreamCap();
        double m_cap = getNet("M")->getTotalDownstreamCap();
        double m_i_cap = getNet("M_i")->getTotalDownstreamCap();
        double q_b_cap = getNet("Q_b")->getTotalDownstreamCap();
        double q_cap = getNet("Q")->getTotalDownstreamCap();
        double ck_cap = getNet("CK")->getTotalDownstreamCap();
        double ck_b_cap = getNet("CK_b")->getTotalDownstreamCap();
        double ck_i_cap = getNet("CK_i")->getTotalDownstreamCap();

        cache->set(cell_name + "->Cap->D", d_cap);
        cache->set(cell_name + "->Cap->D_b", d_b_cap);
        cache->set(cell_name + "->Cap->M_b", m_b_cap);
        cache->set(cell_name + "->Cap->M", m_cap);
        cache->set(cell_name + "->Cap->M_i", m_i_cap);
        cache->set(cell_name + "->Cap->Q_b", q_b_cap);
        cache->set(cell_name + "->Cap->Q", q_cap);
        cache->set(cell_name + "->Cap->CK", ck_cap);
        cache->set(cell_name + "->Cap->CK_b", ck_b_cap);
        cache->set(cell_name + "->Cap->CK_i", ck_i_cap);

        Log::printLine(cell_name + "->Cap->D=" + (String) d_cap);
        Log::printLine(cell_name + "->Cap->D_b=" + (String) d_b_cap);        
        Log::printLine(cell_name + "->Cap->M_b=" + (String) m_b_cap);
        Log::printLine(cell_name + "->Cap->M=" + (String) m_cap);        
        Log::printLine(cell_name + "->Cap->M_i=" + (String) m_i_cap);
        Log::printLine(cell_name + "->Cap->Q_b=" + (String) q_b_cap);        
        Log::printLine(cell_name + "->Cap->Q=" + (String) q_cap);
        Log::printLine(cell_name + "->Cap->CK=" + (String) ck_cap);        
        Log::printLine(cell_name + "->Cap->CK_b=" + (String) ck_b_cap);        
        Log::printLine(cell_name + "->Cap->CK_i=" + (String) ck_i_cap);        
        // --------------------------------------------------------------------

        // --------------------------------------------------------------------
        // Build Internal Delay Model
        // --------------------------------------------------------------------
        double q_ron = getDriver("INV4_RonZN")->getOutputRes();

        double d_setup_delay = getDriver("INV1_RonZN")->calculateDelay() + 
            getDriver("INVZ1_RonZN")->calculateDelay() +
            getDriver("INV2_RonZN")->calculateDelay();        
        double ck_to_q_delay = getDriver("INV5_RonZN")->calculateDelay() +
            getDriver("INV6_RonZN")->calculateDelay() +
            getDriver("INVZ3_RonZN")->calculateDelay() +
            getDriver("INV3_RonZN")->calculateDelay() +
            getDriver("INV4_RonZN")->calculateDelay();        

        cache->set(cell_name + "->DriveRes->Q", q_ron);        
        cache->set(cell_name + "->Delay->D_Setup", d_setup_delay);
        cache->set(cell_name + "->Delay->CK_to_Q", ck_to_q_delay);
        Log::printLine(cell_name + "->DriveRes->Q=" + (String) q_ron);        
        Log::printLine(cell_name + "->Delay->D_Setup=" + (String) d_setup_delay);
        Log::printLine(cell_name + "->Delay->CK_to_Q=" + (String) ck_to_q_delay);                

        return;
        // --------------------------------------------------------------------
    }
    
} // namespace DSENT

