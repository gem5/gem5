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

#include "model/std_cells/INV.h"

#include <cmath>

#include "model/PortInfo.h"
#include "model/TransitionInfo.h"
#include "model/EventInfo.h"
#include "model/std_cells/CellMacros.h"
#include "model/std_cells/StdCellLib.h"
#include "model/timing_graph/ElectricalNet.h"
#include "model/timing_graph/ElectricalDriver.h"
#include "model/timing_graph/ElectricalLoad.h"
#include "model/timing_graph/ElectricalDelay.h"

namespace DSENT
{
    using std::ceil;
    using std::max;

    INV::INV(const String& instance_name_, const TechModel* tech_model_)
        : StdCell(instance_name_, tech_model_)
    {
        initProperties();
    }

    INV::~INV()
    {}

    void INV::initProperties()
    {
        return;
    }

    void INV::constructModel()
    {
        // All constructModel should do is create Area/NDDPower/Energy Results as
        // well as instantiate any sub-instances using only the hard parameters
        
        // Build Electrical Connectivity
        createInputPort("A");
        createOutputPort("Y");
        
        createLoad("A_Cap");
        createDelay("A_to_Y_delay");
        createDriver("Y_Ron", true);

        ElectricalLoad* a_cap = getLoad("A_Cap");
        ElectricalDelay* a_to_y_delay = getDelay("A_to_Y_delay");
        ElectricalDriver* y_ron = getDriver("Y_Ron");
        
        getNet("A")->addDownstreamNode(a_cap);
        a_cap->addDownstreamNode(a_to_y_delay);
        a_to_y_delay->addDownstreamNode(y_ron);
        y_ron->addDownstreamNode(getNet("Y"));

        // Create Area result
        // Create NDD Power result
        createElectricalAtomicResults();
        // Create INV Event Energy Result
        createElectricalEventAtomicResult("INV");

        getEventInfo("Idle")->setStaticTransitionInfos();
        
        return;
    }
    
    void INV::updateModel()
    {
        // All updateModel should do is calculate numbers for the Area/NDDPower/Energy
        // Results as anything else that needs to be done using either soft or hard parameters
        
        // Get parameters        
        double drive_strength = getDrivingStrength();
        Map<double>* cache = getTechModel()->getStdCellLib()->getStdCellCache();

        // Standard cell cache string
        String cell_name = "INV_X" + (String) drive_strength;
        
        // Get timing parameters
        getLoad("A_Cap")->setLoadCap(cache->get(cell_name + "->Cap->A"));
        getDriver("Y_Ron")->setOutputRes(cache->get(cell_name + "->DriveRes->Y"));
        getDelay("A_to_Y_delay")->setDelay(cache->get(cell_name + "->Delay->A_to_Y"));
        
        // Set the cell area
        getAreaResult("Active")->setValue(cache->get(cell_name + "->Area->Active"));
        getAreaResult("Metal1Wire")->setValue(cache->get(cell_name + "->Area->Metal1Wire"));

        return;
    }
    
    void INV::evaluateModel()
    {
        return;        
    }

    void INV::useModel()
    {
        // Get parameters 
        double drive_strength = getDrivingStrength();
        Map<double>* cache = getTechModel()->getStdCellLib()->getStdCellCache();

        // Standard cell cache string
        String cell_name = "INV_X" + (String) drive_strength;

        // Propagate the transition info and get the 0->1 transtion count
        propagateTransitionInfo();
        double P_A = getInputPort("A")->getTransitionInfo().getProbability1();
        double Y_num_trans_01 = getOutputPort("Y")->getTransitionInfo().getNumberTransitions01();

        // Calculate leakage
        double leakage = 0;
        leakage += cache->get(cell_name + "->Leakage->!A") * (1 - P_A);
        leakage += cache->get(cell_name + "->Leakage->A") * P_A;                
        getNddPowerResult("Leakage")->setValue(leakage);

        // Get VDD
        double vdd = getTechModel()->get("Vdd");

        // Get capacitances
        //double a_cap = cache->get(cell_name + "->Cap->A");
        double y_cap = cache->get(cell_name + "->Cap->Y");
        double y_load_cap = getNet("Y")->getTotalDownstreamCap();

        // Calculate INV Event energy
        double energy_per_trans_01 = (y_cap + y_load_cap) * vdd * vdd;
        getEventResult("INV")->setValue(energy_per_trans_01 * Y_num_trans_01);
        return;
    }

    void INV::propagateTransitionInfo()
    {
        // Get input transition info
        const TransitionInfo& trans_A = getInputPort("A")->getTransitionInfo();

        // Set output transition info
        double Y_num_trans_00 = trans_A.getNumberTransitions11();
        double Y_num_trans_01 = trans_A.getNumberTransitions01();
        double Y_num_trans_11 = trans_A.getNumberTransitions00();
        
        TransitionInfo trans_Y(Y_num_trans_00, Y_num_trans_01, Y_num_trans_11);
        getOutputPort("Y")->setTransitionInfo(trans_Y);
        return;
    }
    
    // Creates the standard cell, characterizes and abstracts away the details
    void INV::cacheStdCell(StdCellLib* cell_lib_, double drive_strength_)
    {
        // Standard cell cache string
        String cell_name = "INV_X" + (String) drive_strength_;

        Log::printLine("=== " + cell_name + " ===");

        // Get parameters        
        double gate_pitch = cell_lib_->getTechModel()->get("Gate->PitchContacted");
        Map<double>* cache = cell_lib_->getStdCellCache();
        
        // Now actually build the full standard cell model
        // Create the two input ports
        createInputPort("A");
        createOutputPort("Y");
        
        // Adds macros
        CellMacros::addInverter(this, "INV", true, true, "A", "Y");    
        CellMacros::updateInverter(this, "INV", drive_strength_);

        // Cache area result
        double area = gate_pitch * getTotalHeight() * (1 + getGenProperties()->get("INV_GatePitches").toDouble());
        cache->set(cell_name + "->Area->Active", area);
        cache->set(cell_name + "->Area->Metal1Wire", area);
        Log::printLine(cell_name + "->Area->Active=" + (String) area);
        Log::printLine(cell_name + "->Area->Metal1Wire=" + (String) area);

        // --------------------------------------------------------------------
        // Leakage Model Calculation
        // --------------------------------------------------------------------
        double leakage_a0 = getGenProperties()->get("INV_LeakagePower_0").toDouble();
        double leakage_a1 = getGenProperties()->get("INV_LeakagePower_1").toDouble();        
        cache->set(cell_name + "->Leakage->!A", leakage_a0);
        cache->set(cell_name + "->Leakage->A", leakage_a1);
        Log::printLine(cell_name + "->Leakage->!A=" + (String) leakage_a0);
        Log::printLine(cell_name + "->Leakage->A=" + (String) leakage_a1);
        // --------------------------------------------------------------------
        
        /*
        // Cache event energy results
        double event_a_flip = getGenProperties()->get("INV_A_Flip").toDouble() + getGenProperties()->get("INV_ZN_Flip").toDouble();
        cache->set(cell_name + "->Event_A_Flip", event_a_flip);
        Log::printLine(cell_name + "->Event_A_Flip=" + (String) event_a_flip);
        */
        
        // --------------------------------------------------------------------
        // Get Node Capacitances
        // --------------------------------------------------------------------
        double a_cap = getNet("A")->getTotalDownstreamCap();
        double y_cap = getNet("Y")->getTotalDownstreamCap();

        cache->set(cell_name + "->Cap->A", a_cap);
        cache->set(cell_name + "->Cap->Y", y_cap);
        Log::printLine(cell_name + "->Cap->A=" + (String) a_cap);
        Log::printLine(cell_name + "->Cap->Y=" + (String) y_cap);
        // --------------------------------------------------------------------

        // --------------------------------------------------------------------
        // Build Internal Delay Model
        // --------------------------------------------------------------------
        double y_ron = getDriver("INV_RonZN")->getOutputRes();
        double a_to_y_delay = getDriver("INV_RonZN")->calculateDelay();        
        cache->set(cell_name + "->DriveRes->Y", y_ron);
        cache->set(cell_name + "->Delay->A_to_Y", a_to_y_delay);
        Log::printLine(cell_name + "->DriveRes->Y=" + (String) y_ron);
        Log::printLine(cell_name + "->Delay->A_to_Y=" + (String) a_to_y_delay);
        // --------------------------------------------------------------------
                
        return;

    }
    
} // namespace DSENT

