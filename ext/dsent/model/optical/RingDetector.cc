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

#include "model/optical/RingDetector.h"

#include <cmath>

#include "util/Constants.h"
#include "model/PortInfo.h"
#include "model/TransitionInfo.h"
#include "model/EventInfo.h"
#include "model/std_cells/StdCell.h"
#include "model/std_cells/StdCellLib.h"
#include "model/optical_graph/OpticalWaveguide.h"
#include "model/optical_graph/OpticalDetector.h"
#include "model/optical_graph/OpticalFilter.h"
#include "model/timing_graph/ElectricalDriver.h"
#include "model/timing_graph/ElectricalNet.h"

namespace DSENT
{
    // TODOs for this model
    // Add the other receiver topologies from [Georgas, CICC 2011]
    // Split integ_time_ratio = SA integ time ratio
    // Right now perfect clock gating is assumed...may not be what we want
        
    // Constants
    const String RingDetector::INTEGRATINGSENSEAMP = "INTSA";

    RingDetector::RingDetector(const String& instance_name_, const TechModel* tech_model_)
        : OpticalModel(instance_name_, tech_model_), OpticalReceiver()
    {
        initParameters();
        initProperties();
    }

    RingDetector::~RingDetector()
    {}
    
    void RingDetector::initParameters()
    {
        addParameterName("DataRate");
        addParameterName("InStart");
        addParameterName("InEnd");
        addParameterName("DetStart");
        addParameterName("DetEnd");
        addParameterName("DropAll");
        addParameterName("Topology");
        return;
    }

    void RingDetector::initProperties()
    {
        return;
    }
    
    void RingDetector::constructModel()
    {
        // Get parameters
        WavelengthGroup in_wavelengths = makeWavelengthGroup(getParameter("InStart"), getParameter("InEnd"));
        WavelengthGroup det_wavelengths = makeWavelengthGroup(getParameter("DetStart"), getParameter("DetEnd"));
        int number_wavelengths = det_wavelengths.second - det_wavelengths.first + 1;
        bool drop_all = getParameter("DropAll");
        const String& topology = getParameter("Topology");
    
        // Set some generated properties
        getGenProperties()->set("NumberWavelengths", number_wavelengths);
    
        // Create device area result
        addAreaResult(new AtomicResult("Photonic"));
        // Create electrical results
        createElectricalAtomicResults();
        if (topology == INTEGRATINGSENSEAMP) addEventResult(new AtomicResult("Receive"));
        else ASSERT(false, "[Error] " + getInstanceName() + " -> Unknown receiver topology (" + topology + ")!");
    
        // Create optical ports
        createOpticalInputPort(         "In",   in_wavelengths);
        createOpticalOutputPort(        "Out",  in_wavelengths);
        // Create the filter and modulator
        createFilter(                   "RingFilter",   in_wavelengths, drop_all, det_wavelengths);
        createDetector(                 "RingDetector", det_wavelengths, this);
        OpticalFilter* ring_filter = getFilter("RingFilter");
        OpticalDetector* ring_detector = getDetector("RingDetector");        
        // Connect the filter and modulator
        getWaveguide("In")->addDownstreamNode(ring_filter);
        ring_filter->addDownstreamNode(getWaveguide("Out"));
        ring_filter->setDropPort(ring_detector);
        
        // Create electrical ports
        createOutputPort("Out", makeNetIndex(0, number_wavelengths-1));
        // Create net
        createNet("OutVFO");
        // Create output driver
        createDriver("OutDriver", false);
        // Connect driver
        getDriver("OutDriver")->addDownstreamNode(getNet("OutVFO"));        
        // Connect output
        assignVirtualFanout("Out", "OutVFO");

        // Precompute some technology values
        precomputeTech();
        
        return;
    }
    
    void RingDetector::updateModel()
    {
        // Get some generated properties
        unsigned int number_wavelengths = getGenProperties()->get("NumberWavelengths");

        // Get tech model numbers
        double ring_area = getTechModel()->get("Ring->Area");
        double thru_loss = getTechModel()->get("Ring->ThroughLoss");
        double drop_loss = getTechModel()->get("Ring->DropLoss");
        double pd_loss = getTechModel()->get("Photodetector->Loss");
        double pd_responsivity = getTechModel()->get("Photodetector->Responsivity");
        
        // Design the receiver
        designReceiver();
        
        // Update losses
        // Connect the filter and modulator
        OpticalFilter* ring_filter = getFilter("RingFilter");
        OpticalDetector* ring_detector = getDetector("RingDetector");        
        ring_filter->setLoss(thru_loss * number_wavelengths);
        ring_filter->setDropLoss(drop_loss + thru_loss * number_wavelengths);
        ring_detector->setLoss(pd_loss);
        ring_detector->setResponsivity(pd_responsivity);
        // Update device area
        getAreaResult("Photonic")->setValue(ring_area * (number_wavelengths));

        return;
    }
    
    void RingDetector::useModel()
    {
        // Get parameters
        const String& topology = getParameter("Topology");

        // Get some generated properties
        unsigned int number_wavelengths = getGenProperties()->get("NumberWavelengths");

        // Get optical input transition info
        const TransitionInfo& in_trans = getOpticalInputPort("In")->getTransitionInfo();
        
        // Get tech models
        double vdd = getTechModel()->get("Vdd");
        // Get caps
        double unit_gate_cap = getTechModel()->get("Gate->MinWidth").toDouble() * getTechModel()->get("Gate->CapPerWidth").toDouble();
        double unit_drain_cap = getTechModel()->get("Gate->MinWidth").toDouble() * getTechModel()->get("Drain->CapPerWidth").toDouble();
        double inv_x1_gate_cap = getTechModel()->getStdCellLib()->getStdCellCache()->get("INV_X1->Cap->A");
        double inv_x1_drain_cap = getTechModel()->getStdCellLib()->getStdCellCache()->get("INV_X1->Cap->Y");
        
        // Construct a simple sense-amp model
        if(topology == INTEGRATINGSENSEAMP)
        {
            // Use ratios from the receiver published in [Georgas, ESSCIRC 2011]
            // Note:
            // The numbers in the paper (43fJ/b, 50 fJ/b in the cited work) is done with the clock buffer (there are 4 receivers),
            // capacitive DAC, and extra output flops used in the physical layout, as the compared receiver is extremely conservative
            // We simplified this model to not have the capacitive DAC, the clock buffer (since this is an individual receiver), or
            // the extra output flops (since receiver structure is already a posedge flop functionally).
            // Look for an upcoming paper [Georgas, JSSC 2012] (when it is published) for the power breakdown pie-chart for the receiver.
            // This model only models the latch (sampler) and the dynamic to static (RS latch) part of the design, which is all you really
            // need in the receiver.
            
            // Gate caps
            double c_gate_sampler = unit_gate_cap * (4 * 2.0 + 2 * 1.0 + 2 * 3.0 + 2 * 5.0) + unit_gate_cap * (2 * 6.0 + 2 * 1.0) + inv_x1_gate_cap;
            double c_gate_rslatch = unit_gate_cap * (4 * 1.0) + inv_x1_gate_cap;
            // Drain caps
            double c_drain_sampler = unit_drain_cap * (2 * 2.0 + 2 * 1.0 + 3 * 5.0 + 1 * 3.0) + inv_x1_drain_cap;
            double c_drain_rslatch = unit_drain_cap * (2 * 6.0) + inv_x1_drain_cap;
            // Sum up cap switched for the sampler
            double c_sampler = c_gate_sampler + c_drain_sampler;
            double c_rslatch = c_gate_rslatch + c_drain_rslatch;
            // Average cap switched 
            // Sampler is differential, one side will always switch (R or S in the latch) regardless of probability
            double avg_cap = c_sampler + c_rslatch * in_trans.getProbability0() * in_trans.getProbability1();

            // Get parameters corresponding to a unit-inverter
            double unit_leak_0 = getTechModel()->getStdCellLib()->getStdCellCache()->get("INV_X1->Leakage->!A");
            double unit_leak_1 = getTechModel()->getStdCellLib()->getStdCellCache()->get("INV_X1->Leakage->A");        
            
            // Approximate leakage (curve fit with design)
            double total_leakage = 0.5 * (unit_leak_0 + unit_leak_1) * 7.43;            

            // Create results
            getEventResult("Receive")->setValue(vdd * vdd * avg_cap * number_wavelengths);
            getNddPowerResult("Leakage")->setValue(total_leakage * number_wavelengths);

        }
        else ASSERT(false, "[Error] " + getInstanceName() + " -> Unknown receiver topology (" + topology + ")!");        
        
        return;
    }
    
    void RingDetector::propagateTransitionInfo()
    {
        // Propagate probabilities from optical input to electrical output port
        getOutputPort("Out")->setTransitionInfo(getOpticalInputPort("In")->getTransitionInfo());        
            
        return;        
    }
    
    void RingDetector::precomputeTech()
    {
        // Get parameters
        const double data_rate = getParameter("DataRate");
        const String& topology = getParameter("Topology");

        // Get tech model numbers
        double pd_cap = getTechModel()->get("Photodetector->Cap");
        double parasitic_cap = getTechModel()->get("Photodetector->ParasiticCap");
        double apd = getTechModel()->get("Photodetector->AvalancheGain");
        double vdd = getTechModel()->get("Vdd");

        // Constants shortcuts
        double pi = Constants::pi;
        double k = Constants::k;
        double q = Constants::q;   
        double T = getTechModel()->get("Temperature");
                
        if(topology == INTEGRATINGSENSEAMP)
        {
            // Get more tech parameters
            double integ_time_ratio = getTechModel()->get("Receiver->Int->IntegrationTimeRatio");
            double BER = getTechModel()->get("SenseAmp->BER");
            double CMRR = getTechModel()->get("SenseAmp->CMRR");
            double offset_comp_bits = getTechModel()->get("SenseAmp->OffsetCompensationBits");
            double offset = getTechModel()->get("SenseAmp->OffsetRatio").toDouble() * vdd;
            double supply_noise_rand = getTechModel()->get("SenseAmp->SupplyNoiseRandRatio").toDouble() * vdd;
            double supply_noise_det = getTechModel()->get("SenseAmp->SupplyNoiseDetRatio").toDouble() * vdd;
            double noise_margin = getTechModel()->get("SenseAmp->NoiseMargin");
            double jitter_ratio = getTechModel()->get("SenseAmp->JitterRatio");
            
            // Approximate tao using FO4
            double unit_drain_cap = getTechModel()->get("Gate->MinWidth").toDouble() * getTechModel()->get("Drain->CapPerWidth").toDouble();
            double c_g = getTechModel()->getStdCellLib()->getStdCellCache()->get("INV_X1->Cap->A");
            double c_d = getTechModel()->getStdCellLib()->getStdCellCache()->get("INV_X1->Cap->Y");
            double r_o = getTechModel()->getStdCellLib()->getStdCellCache()->get("INV_X1->DriveRes->Y");
            // Calculate sense amp tau from sense amp output loading
            double tau = r_o * (c_g + c_d);
            // Set output inverter drive strength
            getDriver("OutDriver")->setOutputRes(r_o);

            // Calculate sense amp input cap based on schematic
            double sense_amp_cap_in = unit_drain_cap * (2.0 + 3.0 + 5.0 + 1.0);

            // Residual offset
            double v_residual = 3 * offset / pow(2, offset_comp_bits);
            // Noise
            double v_noise = supply_noise_rand * supply_noise_rand / (CMRR * CMRR);
            // Sense amp voltage build-up minimum
            double v_sense = vdd * exp(-(1 - integ_time_ratio) / (data_rate * tau)) + noise_margin + v_residual + supply_noise_det / CMRR;
            // Sigmas corresponding to BER
            double sigma = calcInvNormCdf(BER);
            
            //K_int is the time the bit is valid for evaluation

            // Total input cap load
            double input_node_cap = sense_amp_cap_in + pd_cap + parasitic_cap;
            double z_int = integ_time_ratio / (data_rate * input_node_cap); //should use K_int
            
            // Store precalculated values
            m_quad_a_ = 1 - (sigma * sigma * jitter_ratio * jitter_ratio);
            m_quad_b1_ = - 2 * pi / 2 * sigma * sigma * q * 0.7 * data_rate;
            m_quad_b2_ = -2 * v_sense / (z_int * apd);
            m_quad_c_ = 1 / (z_int * z_int) * (v_sense * v_sense - sigma * sigma * (k * T / input_node_cap + v_noise));
        }
        else ASSERT(false, "[Error] " + getInstanceName() + " -> Unknown receiver topology (" + topology + ")!");

        return;
    }

    void RingDetector::designReceiver()
    {
        // Get some generated properties
        unsigned int number_wavelengths = getGenProperties()->get("NumberWavelengths");

        // Get relevant properties/parameters
        const String& topology = getParameter("Topology");
                
        // Construct a simple sense-amp model
        if(topology == INTEGRATINGSENSEAMP)
        {
            // No really good way to estimate the area...can assume each receiver is the size of 40 inverters, which is
            // about the right size for just the sense amp in the layout
            double unit_area_active = getTechModel()->getStdCellLib()->getStdCellCache()->get("INV_X1->Area->Active");
            double unit_area_metal1 = getTechModel()->getStdCellLib()->getStdCellCache()->get("INV_X1->Area->Metal1Wire");          
            getAreaResult("Active")->setValue(unit_area_active * 40 * number_wavelengths);
            getAreaResult("Metal1Wire")->setValue(unit_area_metal1 * 40 * number_wavelengths);
        }
        else ASSERT(false, "[Error] " + getInstanceName() + " -> Unknown receiver topology (" + topology + ")!");
        
        return;
    }
    
    double RingDetector::getSensitivity(double ER_dB_) const
    {
        // Get parameters
        const String& topology = getParameter("Topology");
        // Turn extinction ratio into a ratio from dB scale
        double ER = pow(10, ER_dB_ / 10);
                
        // Initialize sensitivity
        double sensitivity = 1e99;
        // Construct a simple sense-amp model
        if(topology == INTEGRATINGSENSEAMP)
        {
            // Scale photodetector shot noise using ER, add rest of noise source
            double b = m_quad_b1_ * (1 + ER) / (2 * (ER - 1)) + m_quad_b2_;            
        
            // Find sensitivity (-b + sqrt(b^2-4ac)) / 2a
            sensitivity = ((-b + sqrt(b * b - 4 * m_quad_a_ * m_quad_c_)) / (2 * m_quad_a_));
        }
        else ASSERT(false, "[Error] " + getInstanceName() + " -> Unknown receiver topology (" + topology + ")!");        
        
        return sensitivity;
    }
    
    double RingDetector::calcInvNormCdf(double num_)
    {
        // 53 bit precision for double FP
        unsigned int num_iterations = 20;
        // Upperbound the step
        double step = 20;
        double out = step;                
        // Iteratively guess and check calculation
        for (unsigned int i = 0; i < num_iterations; ++i)
        {
            double current = 0.5 * erfc(out / sqrt(2));
            if (current > num_) out += step;
            else out -= step;            
            step = step * 0.5;
        }
        
        return out;
    }
    
} // namespace DSENT

