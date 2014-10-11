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

#include "model/optical/RingModulator.h"

#include <cmath>

#include "util/Constants.h"
#include "model/PortInfo.h"
#include "model/TransitionInfo.h"
#include "model/EventInfo.h"
#include "model/std_cells/StdCell.h"
#include "model/std_cells/StdCellLib.h"
#include "model/optical_graph/OpticalWaveguide.h"
#include "model/optical_graph/OpticalModulator.h"
#include "model/optical_graph/OpticalFilter.h"
#include "model/optical_graph/OpticalTransmitter.h"
#include "model/timing_graph/ElectricalNet.h"
#include "model/timing_graph/ElectricalLoad.h"
#include "model/timing_graph/ElectricalTimingTree.h"

namespace DSENT
{
    using std::max;
    using std::min;
    
    // TODO: Don't like the way this is written right now. Probably fix in a future version

    RingModulator::RingModulator(const String& instance_name_, const TechModel* tech_model_)
        : OpticalModel(instance_name_, tech_model_)
    {
        initParameters();
        initProperties();
    }

    RingModulator::~RingModulator()
    {}

    void RingModulator::initParameters()
    {
        addParameterName("DataRate");
        addParameterName("InStart");
        addParameterName("InEnd");
        addParameterName("ModStart");
        addParameterName("ModEnd");
        addParameterName("OptimizeLoss", "TRUE");
        return;
    }

    void RingModulator::initProperties()
    {
        addPropertyName("ExtinctionRatio", 6);  //default properties
        addPropertyName("InsertionLoss", 2);    //default properties
        return;
    }

    void RingModulator::constructModel()
    {
        // Create electrical results
        createElectricalAtomicResults();
        // Create Area result
        addAreaResult(new AtomicResult("Photonic"));
        // Create Modulate result
        createElectricalEventAtomicResult("Modulate");

        // Get parameters
        WavelengthGroup in_wavelengths = makeWavelengthGroup(getParameter("InStart"), getParameter("InEnd"));
        WavelengthGroup mod_wavelengths = makeWavelengthGroup(getParameter("ModStart"), getParameter("ModEnd"));
        int number_wavelengths = mod_wavelengths.second - mod_wavelengths.first + 1;
        bool optimize_loss = getParameter("OptimizeLoss");

        getGenProperties()->set("NumberWavelengths", number_wavelengths);
        
        // Create optical ports
        createOpticalInputPort(         "In",   in_wavelengths);
        createOpticalOutputPort(        "Out",  in_wavelengths);
        // Create the filter and modulator
        createFilter(                   "RingFilter",       in_wavelengths, true, mod_wavelengths);
        createModulator(                "RingModulator",    mod_wavelengths, optimize_loss, this);
        createWaveguide(                "RingTemp",         mod_wavelengths);
        OpticalFilter* ring_filter = getFilter("RingFilter");
        OpticalModulator* ring_modulator = getModulator("RingModulator");        
        // Connect the filter and modulator
        getWaveguide("In")->addDownstreamNode(ring_filter);
        ring_filter->addDownstreamNode(getWaveguide("Out"));
        ring_filter->setDropPort(ring_modulator);
        ring_modulator->addDownstreamNode(getWaveguide("Out"));
        
        // Create electrical ports
        createInputPort(                "In", makeNetIndex(0, number_wavelengths-1));
        // Create driver
        createNet("PredriverIn");
        // VFI from In to PredriverIn
        assignVirtualFanin("PredriverIn", "In");        
        // Create input load (due to predrivers)
        createLoad("PredriverCap");
        getNet("PredriverIn")->addDownstreamNode(getLoad("PredriverCap"));
        
        // Precompute some values
        precomputeTech();
        
        return;
    }
    
    void RingModulator::updateModel()
    {
        // Get properties
        double ER_dB = getProperty("ExtinctionRatio").toDouble();
        double IL_dB = getProperty("InsertionLoss").toDouble();
            
        // Get Gen properties
        int number_wavelengths = getGenProperties()->get("NumberWavelengths");

        // Get tech model parameters
        double ring_area = getTechModel()->get("Ring->Area").toDouble();
        double thru_loss = getTechModel()->get("Ring->ThroughLoss").toDouble();
        
        // Design the modulator and the modulator driver
        bool success = designModulator(IL_dB, ER_dB);        
        getGenProperties()->set("Success", success);     

        // If not successful, make the modulate energy extremely large
        if (!success) getEventResult("Modulate")->setValue(1e99);
        
        // Update losses
        // Connect the filter and modulator
        OpticalFilter* ring_filter = getFilter("RingFilter");
        ring_filter->setLoss(thru_loss * number_wavelengths);
        ring_filter->setDropLoss(thru_loss * number_wavelengths);     // Assume worst-case through loss for a dropped wavelength
        // Update area
        getAreaResult("Photonic")->setValue(ring_area * (number_wavelengths));        
    }
    
    void RingModulator::useModel()
    {
        // Propagate the transition info and get the 0->1 transtion count
        propagateTransitionInfo();
        double P_In = getInputPort("In")->getTransitionInfo().getProbability1();
        double P_num_trans_01 = getInputPort("In")->getTransitionInfo().getNumberTransitions01();
        
        // Get Gen properties
        int number_wavelengths = getGenProperties()->get("NumberWavelengths");
        
        // If I can't build it...then it is infinitely expensive!
        bool success = getGenProperties()->get("Success");
        double driver_size = 1e99;
        double total_predriver_size = 1e99;
        if (success)
        {
            driver_size = getGenProperties()->get("DriverSize");
            total_predriver_size = getGenProperties()->get("TotalPredriverSize");
        }

        // Get parameters corresponding to a unit-inverter
        double unit_leak_0 = getTechModel()->getStdCellLib()->getStdCellCache()->get("INV_X1->Leakage->!A");
        double unit_leak_1 = getTechModel()->getStdCellLib()->getStdCellCache()->get("INV_X1->Leakage->A");        
        
        // Approximate leakage
        double total_leakage = number_wavelengths * 0.5 * ((driver_size + total_predriver_size) * P_In * unit_leak_1 + 
                                                            (driver_size + total_predriver_size) * (1 - P_In) * unit_leak_0);
        
        getNddPowerResult("Leakage")->setValue(total_leakage);
        getEventResult("Modulate")->setValue(calcModulatorEnergy() * P_num_trans_01);
        
        return;
    }
    
    void RingModulator::propagateTransitionInfo()
    {
        // Very simple...whatever comes in electrically is encoded optically
        getOpticalOutputPort("Out")->setTransitionInfo(getInputPort("In")->getTransitionInfo());

        return;
    }
        
    void RingModulator::precomputeTech()
    {
        // Get parameters
        double data_rate = getParameter("DataRate");
                
        // Constants shortcuts
        double pi = Constants::pi;
        double c = Constants::c;
        double k = Constants::k;
        double e0 = Constants::e0;
        double es = Constants::es;
        double q = Constants::q;   
        double T = getTechModel()->get("Temperature");
        
        // Get modulator parameters
        double lambda = getTechModel()->get("Ring->Lambda").toDouble();
        double n_f = getTechModel()->get("Modulator->Ring->FCPDEffect").toDouble();
        double NA = getTechModel()->get("Modulator->Ring->NA").toDouble();
        double ND = getTechModel()->get("Modulator->Ring->ND").toDouble();
        double ni = getTechModel()->get("Modulator->Ring->ni").toDouble();
        double L_j = getTechModel()->get("Modulator->Ring->JunctionRatio").toDouble();
        double H = getTechModel()->get("Modulator->Ring->Height").toDouble();
        double W = getTechModel()->get("Modulator->Ring->Width").toDouble();
        double g_c = getTechModel()->get("Modulator->Ring->ConfinementFactor").toDouble();
        // Get ring parameters
        double R = getTechModel()->get("Ring->Radius").toDouble();
        double n_g = getTechModel()->get("Ring->GroupIndex").toDouble();
        double Q_max = getTechModel()->get("Ring->MaxQualityFactor").toDouble();

        // Setup calculations
        double f0 = c / lambda;
        double BW = data_rate;                      // Modulator bandwidth
        double Q_f = std::min(f0 / BW, Q_max);      // Quality factor
        double L_tot = 2 * pi * R;                  // Optical length of the ring

        double V_bi = k * T / q * log(NA * ND / (ni * ni));                     // Junction Built-in voltage
        double x_d0 = sqrt(2 * e0 * es / q * V_bi * (NA + ND) / (NA * ND));     // Junction nominal depletion width
        double C_j0 = e0 * es * L_tot * L_j * W / x_d0;                         // Junction nominal cap        
        double Q_0 = q * n_g * (L_tot * H * W) / (2 * n_f * Q_f * g_c);         // Charge in depletion region

        // Store into precomputed values
        m_precompute_V_bi_ = V_bi;
        m_precompute_x_d0_ = x_d0;
        m_precompute_C_j0_ = C_j0;
        m_precompute_Q_0_ = Q_0;

        return;        
    }
    
    bool RingModulator::designModulator(double IL_dB_, double ER_dB_)
    {
        // Get parameters
        double vdd = getTechModel()->get("Vdd");
        double data_rate = getParameter("DataRate");
        unsigned int max_predriver_stages = 20;         //TODO: Make this not hardcoded
        // Get modulator parameters
        double boost_ratio = getTechModel()->get("Modulator->Ring->SupplyBoostRatio");
        double Tn = getTechModel()->get("Modulator->Ring->Tn").toDouble();;
        double H = getTechModel()->get("Modulator->Ring->Height").toDouble();

        // Get Gen properties
        int number_wavelengths = getGenProperties()->get("NumberWavelengths");
                
        // Checking ASSERTions (input properties that don't make any sense)
        ASSERT(ER_dB_ > 0, "[Error] " + getInstanceName() + " -> Extinction ratio must be > 0!");
        ASSERT(IL_dB_ > 0, "[Error] " + getInstanceName() + " -> Insertion loss must be > 0!");        
        
        // Setup calculations
        double ER = pow(10, ER_dB_ / 10);            // Extinction ratio
        double T1 = pow(10, -IL_dB_ / 10);           // Transmisivity on
        double T0 = T1 / ER;                        // Transmisivity off

        // Get precomputed values
        double V_bi = m_precompute_V_bi_;
        double x_d0 = m_precompute_x_d0_;
        double C_j0 = m_precompute_C_j0_;
        double Q_0 = m_precompute_Q_0_;
        
        // Charge
        double int_c = -2 * V_bi * C_j0;
        // Calculate shift using lorentzian
        double gamma = sqrt((1 - Tn)/(1 - T1) - 1) - sqrt((1 - Tn)/(1 - T0) - 1);   // gamma = delta_f / delta_f_FWHM        
        double Q = gamma * Q_0;                                                     // Charge required to hit given Tf
        // Voltage required
        double V_a = V_bi * (pow( (Q - int_c)/(2 * V_bi * C_j0), 2) - 1);
        // Calculate driver vdd
        double hvdd = V_a * boost_ratio;
        // Depletion region required
        double x_d = x_d0 * sqrt((V_bi + V_a) / V_bi);

        // Calculate C_eff
        double c_eff = Q / V_a;
        
        // Feasibility checks
        // Not feasible if the transmisivity when transmitting an optical 1 is greater than 1.0...
        if (T1 >= 1) return false;
        // Not feasible if the transmisivity when transmitting an optical 0 is smaller than the notch of the ring
        if (T0 <= Tn) return false;
        // Not feasible if the extinction ratio is greater than the notch of the ring
        if (ER >= 1 / Tn) return false;
        // Not feasible if the required depletion width is greater than the height of the junction
        if (x_d >= H) return false;

        // Analytically calculate driver sizes
        // Get parameters corresponding to a unit-inverter
        double unit_c_g = getTechModel()->getStdCellLib()->getStdCellCache()->get("INV_X1->Cap->A");
        double unit_c_d = getTechModel()->getStdCellLib()->getStdCellCache()->get("INV_X1->Cap->Y");
        double unit_r_o = getTechModel()->getStdCellLib()->getStdCellCache()->get("INV_X1->DriveRes->Y");
        double unit_area_active = getTechModel()->getStdCellLib()->getStdCellCache()->get("INV_X1->Area->Active");
        double unit_area_metal1 = getTechModel()->getStdCellLib()->getStdCellCache()->get("INV_X1->Area->Metal1Wire");
        
        // Get device resistance/cap
        double device_par_res = getTechModel()->get("Modulator->Ring->ParasiticRes");
        double device_par_cap = getTechModel()->get("Modulator->Ring->ParasiticCap");
        
        // Use timing tree to size modulator drivers
        // Coefficient of R*C to give a 0->V_a transition
        double transition_scale = log(hvdd / (hvdd - V_a));             
        double transition_required = 1 / (4 * data_rate);      // I am not sure what the factor of 4 is for...
        
        // Calculate inverter intrinsic transition time
        double transition_intrinsic = transition_scale * unit_c_d * unit_r_o;
        // Calculate minimum possible device transition time
        double min_transition_intrinsic = transition_intrinsic + transition_scale * device_par_res * c_eff;
        // If the minimum possible transition time is already bigger
        // than the required transition, then this particular driver is not possible...
        if (min_transition_intrinsic > transition_required)
            return false;

        // Calculate driver size
        double driver_size = max(1.0, transition_scale * unit_r_o * (c_eff + device_par_cap) / (transition_required - min_transition_intrinsic));
        // Keep track of the total multiplier of unit inverters (for area, leakage calculations)
        double total_unit_inverters = driver_size * max(1.0, hvdd / vdd);
        // Calculate load cap for predriver stages
        double current_load_cap = driver_size * unit_c_g;
        // Number of predriver stages
        unsigned int predriver_stages = 0;
        // Add predriver stages until the input cap is less than the unit INV_X1 gate cap or
        // if the signal is still inverted (need an odd number of predriver stages)
        while (current_load_cap > unit_c_g || (predriver_stages == 0) || ((predriver_stages & 0x1) == 0))
        {
            // Calculate the size of the current predriver stage
            double current_predriver_size = max(1.0, unit_r_o * current_load_cap / (transition_required - transition_intrinsic));
            // Calculate load cap for the next predriver stage
            current_load_cap = current_predriver_size * unit_c_g;
            // Add cap to total predriver total cap
            total_unit_inverters += current_predriver_size;
            // Consider this a failure if the number of predriver stages exceed some maximum
            if (predriver_stages > max_predriver_stages)
                return false;

            ++predriver_stages;
        }
        // Set the input load capacitance
        getLoad("PredriverCap")->setLoadCap(current_load_cap);

        // Set generated properties
        getGenProperties()->set("DriverSize", driver_size);
        getGenProperties()->set("FirstPredriverSize", current_load_cap);
        getGenProperties()->set("TotalPredriverSize", total_unit_inverters - driver_size);
        getGenProperties()->set("Hvdd", hvdd);
        getGenProperties()->set("Ceff", c_eff);
        
        // Calculate leakage, area, energy consumption
        double area_active = total_unit_inverters * unit_area_active;
        double area_metal1 = total_unit_inverters * unit_area_metal1;
                
        // Set results
        getAreaResult("Active")->setValue(area_active * number_wavelengths);
        getAreaResult("Metal1Wire")->setValue(area_metal1 * number_wavelengths);

        // Only if everything was successful do we set the modulator specification
        getModulator("RingModulator")->setLosses(IL_dB_, ER_dB_);
        return true;        
    }
    
    double RingModulator::calcModulatorEnergy() const
    {
        // Get tech parameters
        double vdd = getTechModel()->get("Vdd");
        double device_par_cap = getTechModel()->get("Modulator->Ring->ParasiticCap");

        // Get Gen properties
        int number_wavelengths = getGenProperties()->get("NumberWavelengths");
        
        bool success = getGenProperties()->get("Success");
        if (success)
        {
            double driver_size = getGenProperties()->get("DriverSize");
            double total_predriver_size = getGenProperties()->get("TotalPredriverSize");
            double first_predriver_size = getGenProperties()->get("FirstPredriverSize");
            double c_eff = getGenProperties()->get("Ceff");
            double hvdd = getGenProperties()->get("Hvdd");
            
            // Get parameters corresponding to a unit-inverter
            double unit_c_g = getTechModel()->getStdCellLib()->getStdCellCache()->get("INV_X1->Cap->A");
            double unit_c_d = getTechModel()->getStdCellLib()->getStdCellCache()->get("INV_X1->Cap->Y");
            
            // Approximate leakage
            double energy_predriver = number_wavelengths * vdd * vdd * ((unit_c_d * total_predriver_size + 
                                            unit_c_g * (total_predriver_size + driver_size - first_predriver_size)));
            double energy_driver = number_wavelengths * hvdd * std::max(hvdd, vdd) * (driver_size * unit_c_d + c_eff + device_par_cap);
            
            return (energy_predriver + energy_driver);
        }
        else
            return 1e99;    // An infinitely expensive modulator
    }
    
    bool RingModulator::setTransmitterSpec(double IL_dB_, double ER_dB_)
    {
        setProperty("InsertionLoss", IL_dB_);
        setProperty("ExtinctionRatio", ER_dB_);
        update();
        evaluate();
        
        return getGenProperties()->get("Success");
    }
    
    double RingModulator::getPower(double util_) const
    {
        // Get parameters
        double data_rate = getParameter("DataRate");
		// Check arguments
        ASSERT((util_ <= 1.0) && (util_ >= 0.0), "[Error] " + getInstanceName() + " -> Modulator utilization must be between 0.0 and 1.0!");

        return calcModulatorEnergy() * 0.25 * util_ * data_rate;
    }
    
} // namespace DSENT

