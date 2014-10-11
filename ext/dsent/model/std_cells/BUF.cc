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

#include "model/std_cells/BUF.h"

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
    using std::max;

    BUF::BUF(const String& instance_name_, const TechModel* tech_model_)
        : StdCell(instance_name_, tech_model_)
    {
        initProperties();
    }

    BUF::~BUF()
    {}

    void BUF::initProperties()
    {
        return;
    }

    void BUF::constructModel()
    {
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
        // Create OR Event Energy Result
        createElectricalEventAtomicResult("BUF");

        getEventInfo("Idle")->setStaticTransitionInfos();
        
        return;
    }

    void BUF::updateModel()
    {
        // Get parameters
        double drive_strength = getDrivingStrength();
        Map<double>* cache = getTechModel()->getStdCellLib()->getStdCellCache();

        // Standard cell cache string
        const String& cell_name = "BUF_X" + (String) drive_strength;

        // Get timing parameters
        getLoad("A_Cap")->setLoadCap(cache->get(cell_name + "->Cap->A"));      
        getDelay("A_to_Y_delay")->setDelay(cache->get(cell_name + "->Delay->A_to_Y"));      
        getDriver("Y_Ron")->setOutputRes(cache->get(cell_name + "->DriveRes->Y"));

        // Set the cell area
        getAreaResult("Active")->setValue(cache->get(cell_name + "->ActiveArea"));
        getAreaResult("Metal1Wire")->setValue(cache->get(cell_name + "->ActiveArea"));
        
        return;
    }

    void BUF::evaluateModel()
    {
        return;
    }

    void BUF::useModel()
    {
        // Get parameters
        double drive_strength = getDrivingStrength();
        Map<double>* cache = getTechModel()->getStdCellLib()->getStdCellCache();

        // Stadard cell cache string
        const String& cell_name = "BUF_X" + (String) drive_strength;

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
        double y_b_cap = cache->get(cell_name + "->Cap->Y_b");
        double y_cap = cache->get(cell_name + "->Cap->Y");
        double y_load_cap = getNet("Y")->getTotalDownstreamCap();                
        
        // Calculate BUFEvent energy
        double energy_per_trans_01 = (y_b_cap + y_cap + y_load_cap) * vdd * vdd;
        getEventResult("BUF")->setValue(energy_per_trans_01 * Y_num_trans_01);
                
        return;
    }

    void BUF::propagateTransitionInfo()
    {
        // Get input signal transition info
        const TransitionInfo& trans_A = getInputPort("A")->getTransitionInfo();
        
        getOutputPort("Y")->setTransitionInfo(trans_A);
        return;
    }

    // Creates the standard cell, characterizes and abstracts away the details
    void BUF::cacheStdCell(StdCellLib* cell_lib_, double drive_strength_)
    {
        // Get parameters
        double gate_pitch = cell_lib_->getTechModel()->get("Gate->PitchContacted");
        Map<double>* cache = cell_lib_->getStdCellCache();

        // Stadard cell cache string
        const String& cell_name = "BUF_X" + (String) drive_strength_;

        Log::printLine("=== " + cell_name + " ===");

        // Now actually build the full standard cell model
        createInputPort("A");
        createOutputPort("Y");

        createNet("Y_b");

        // Adds macros
        CellMacros::addInverter(this, "INV0", false, true, "A", "Y_b");
        CellMacros::addInverter(this, "INV1", false, true, "Y_b", "Y");

        // Update macros
        CellMacros::updateInverter(this, "INV0", drive_strength_ * 0.367);
        CellMacros::updateInverter(this, "INV1", drive_strength_ * 1.0);

        // Cache area result
        double area = 0.0;
        area += gate_pitch * getTotalHeight() * 1;
        area += gate_pitch * getTotalHeight() * getGenProperties()->get("INV0_GatePitches").toDouble();
        area += gate_pitch * getTotalHeight() * getGenProperties()->get("INV1_GatePitches").toDouble();
        cache->set(cell_name + "->ActiveArea", area);
        Log::printLine(cell_name + "->ActiveArea=" + (String)area);
        
        // --------------------------------------------------------------------
        // Leakage Model Calculation
        // --------------------------------------------------------------------
        // Cache leakage power results (for every single signal combination)
        double leakage_0 = 0.0; // !A
        double leakage_1 = 0.0; // A

        leakage_0 += getGenProperties()->get("INV0_LeakagePower_0").toDouble();
        leakage_0 += getGenProperties()->get("INV1_LeakagePower_1").toDouble();

        leakage_1 += getGenProperties()->get("INV0_LeakagePower_1").toDouble();
        leakage_1 += getGenProperties()->get("INV1_LeakagePower_0").toDouble();

        cache->set(cell_name + "->Leakage->!A", leakage_0);
        cache->set(cell_name + "->Leakage->A", leakage_1);
        Log::printLine(cell_name + "->Leakage->!A=" + (String) leakage_0);
        Log::printLine(cell_name + "->Leakage->A=" + (String) leakage_1);
        // --------------------------------------------------------------------

        // --------------------------------------------------------------------
        // Get Node Capacitances
        // --------------------------------------------------------------------
        double a_cap = getNet("A")->getTotalDownstreamCap();
        double y_b_cap = getNet("Y_b")->getTotalDownstreamCap();
        double y_cap = getNet("Y")->getTotalDownstreamCap();

        cache->set(cell_name + "->Cap->A", a_cap);
        cache->set(cell_name + "->Cap->Y_b", y_b_cap);
        cache->set(cell_name + "->Cap->Y", y_cap);
        Log::printLine(cell_name + "->Cap->A_Cap=" + (String) a_cap);
        Log::printLine(cell_name + "->Cap->Y_b_Cap=" + (String) y_b_cap);
        Log::printLine(cell_name + "->Cap->Y_Cap=" + (String) y_cap);
        // --------------------------------------------------------------------

        // --------------------------------------------------------------------
        // Build Internal Delay Model
        // --------------------------------------------------------------------
        double y_ron = getDriver("INV1_RonZN")->getOutputRes();
        double a_to_y_delay = getDriver("INV0_RonZN")->calculateDelay() + 
                              getDriver("INV1_RonZN")->calculateDelay();

        cache->set(cell_name + "->DriveRes->Y", y_ron);
        cache->set(cell_name + "->Delay->A_to_Y", a_to_y_delay);
        Log::printLine(cell_name + "->DriveRes->Y=" + (String) y_ron);
        Log::printLine(cell_name + "->Delay->A_to_Y=" + (String) a_to_y_delay);
        // --------------------------------------------------------------------

        return;
    }
} // namespace DSENT

