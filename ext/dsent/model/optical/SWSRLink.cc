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

#include "model/optical/SWSRLink.h"

#include "model/ModelGen.h"
#include "model/PortInfo.h"
#include "model/TransitionInfo.h"
#include "model/EventInfo.h"
#include "model/optical_graph/OpticalGraph.h"
#include "model/optical_graph/OpticalWaveguide.h"
#include "model/optical/RingModulator.h"
#include "model/optical/RingFilter.h"
#include "model/optical/RingDetector.h"
#include "model/optical/LaserSource.h"
#include "model/optical/ThrottledLaserSource.h"

namespace DSENT
{
    SWSRLink::SWSRLink(const String& instance_name_, const TechModel* tech_model_)
        : OpticalModel(instance_name_, tech_model_)
    {
        initParameters();
        initProperties();
    }

    SWSRLink::~SWSRLink()
    {}

    void SWSRLink::initParameters()
    {
        addParameterName("NumberBits");     
        addParameterName("CoreDataRate");
        addParameterName("LinkDataRate");

        addParameterName("LaserType");
        addParameterName("RingTuningMethod");
        addParameterName("OptimizeLoss", "TRUE");

        return;
    }

    void SWSRLink::initProperties()
    {
        addPropertyName("Length");
		addPropertyName("OptUtil", 0.5);        // default to 50% utilization (a new word 50% of the time)
        addPropertyName("ExtinctionRatio", 6);  // default properties
        addPropertyName("InsertionLoss", 2);    // default properties
        return;
    }

    void SWSRLink::constructModel()
    {
        // Get parameters
        unsigned int number_bits = getParameter("NumberBits");
        double core_data_rate = getParameter("CoreDataRate");
        double link_data_rate = getParameter("LinkDataRate");
        
        // Get directly propagated parameters
        const String& ring_tuning_method = getParameter("RingTuningMethod");
        
        // Calculate number of wavelengths needed
        unsigned int number_wavelengths = (unsigned int)((double) number_bits * core_data_rate / link_data_rate);
        
        // Set some generated properties
        getGenProperties()->set("NumberWavelengths", number_wavelengths);
        
        // Create electrical ports
        createInputPort("LinkCK");
        createInputPort("In", makeNetIndex(0, number_bits-1));
        createOutputPort("Out", makeNetIndex(0, number_bits-1));

        // Create Waveguides
        // Temporarily assume its all on one waveguide
        createWaveguide("LaserToMod", makeWavelengthGroup(0, number_wavelengths-1));
        createWaveguide("ModToDetector", makeWavelengthGroup(0, number_wavelengths-1));        
        
        // Add area results
        addAreaResult(new Result("Photonic"));
        createElectricalResults();
        // Setup idle event
        getEventInfo("Idle")->setStaticTransitionInfos();
        // Create a waveguide area result
        addAreaResult(new AtomicResult("Waveguide"));
        getAreaResult("Photonic")->addSubResult(getAreaResult("Waveguide"), "Waveguide", 1.0);
        // Add results
        addNddPowerResult(new Result("Laser"));
        addNddPowerResult(new Result("RingTuning"));
        // Add event result
        createElectricalEventResult("Send");
        
        // Create Tx, Rx backends
        // Create Tx electrical backend
        ElectricalModel* tx_backend = (ElectricalModel*) ModelGen::createModel("OpticalLinkBackendTx", "OpticalLinkBackendTx", getTechModel());
        tx_backend->setParameter("InBits", number_bits);
        tx_backend->setParameter("CoreDataRate", core_data_rate);
        tx_backend->setParameter("LinkDataRate", link_data_rate);
        tx_backend->setParameter("RingTuningMethod", ring_tuning_method);
        tx_backend->setParameter("BitDuplicate", "TRUE");
        tx_backend->construct();

        // Create Rx electrical backend
        ElectricalModel* rx_backend = (ElectricalModel*) ModelGen::createModel("OpticalLinkBackendRx", "OpticalLinkBackendRx", getTechModel());
        rx_backend->setParameter("OutBits", number_bits);
        rx_backend->setParameter("CoreDataRate", core_data_rate);
        rx_backend->setParameter("LinkDataRate", link_data_rate);
        rx_backend->setParameter("RingTuningMethod", ring_tuning_method);
        rx_backend->setParameter("BitDuplicate", "TRUE");
        rx_backend->construct();
        
        // Connect ports
        createNet("TxBackendToTx", makeNetIndex(0, number_wavelengths-1));
        createNet("RxToRxBackend", makeNetIndex(0, number_wavelengths-1));
        portConnect(tx_backend, "In", "In");
        portConnect(tx_backend, "Out", "TxBackendToTx");
        portConnect(tx_backend, "LinkCK", "LinkCK");
        portConnect(rx_backend, "In", "RxToRxBackend");
        portConnect(rx_backend, "Out", "Out");
        portConnect(rx_backend, "LinkCK", "LinkCK");

        // Add instances
        addSubInstances(tx_backend, 1.0);
        addSubInstances(rx_backend, 1.0);

        // Add electrical results
        addElectricalSubResults(tx_backend, 1.0);
        addElectricalSubResults(rx_backend, 1.0);
        
        // Add tuning power result
        getNddPowerResult("RingTuning")->addSubResult(tx_backend->getNddPowerResult("RingTuning"), "OpticalLinkBackendTx", 1.0);
        getNddPowerResult("RingTuning")->addSubResult(rx_backend->getNddPowerResult("RingTuning"), "OpticalLinkBackendRx", 1.0);
        
        // Add event results
        getEventInfo("Send")->setTransitionInfo("LinkCK", TransitionInfo(0.0, (double) link_data_rate / (core_data_rate * 2.0), 0.0));

        getEventResult("Send")->addSubResult(tx_backend->getEventResult("ProcessBits"), "OpticalLinkBackendTx", 1.0);
        getEventResult("Send")->addSubResult(rx_backend->getEventResult("ProcessBits"), "OpticalLinkBackendRx", 1.0);
        
        buildLaser();
        buildModulator();
        buildDetector();
        
        return;
    }
    
    void SWSRLink::updateModel()
    {
        // Get parameters
        double link_data_rate = getParameter("LinkDataRate");
        
        // Get properties
        double length = getProperty("Length");
        const String& extinction_ratio = getProperty("ExtinctionRatio");
        const String& insertion_loss = getProperty("InsertionLoss");
		const double opt_util = getProperty("OptUtil");
		
        // Calculate loss for waveguide
        double waveguide_loss = getTechModel()->get("Waveguide->LossPerMeter").toDouble() * length;
        // Set loss of the waveguide
        getWaveguide("ModToDetector")->setLoss(waveguide_loss);
        // Calculate waveguide area
        double waveguide_area = length * getTechModel()->get("Waveguide->Pitch").toDouble();
        getAreaResult("Waveguide")->setValue(waveguide_area);
    
        // Update the laser
        Model* laser = getSubInstance("Laser");
        laser->setProperty("LaserEventTime", 1.0 / link_data_rate);
		laser->setProperty("OptUtil", opt_util);
        laser->update();
        
        // Update the modulator
        Model* modulator = getSubInstance("Modulator");
        modulator->setProperty("ExtinctionRatio", extinction_ratio);
        modulator->setProperty("InsertionLoss", insertion_loss);
        modulator->update();        
        
        Model* detector = getSubInstance("Detector");
        detector->update();
        
        Model* tx_backend = getSubInstance("OpticalLinkBackendTx");
        tx_backend->update();
                   
        Model* rx_backend = getSubInstance("OpticalLinkBackendRx");
        rx_backend->update();

        return;
    }
    
    void SWSRLink::propagateTransitionInfo()
    {
        // Get parameters
        const String& laser_type = getParameter("LaserType");

        // Propagate transition info to tx backend
        OpticalModel* tx_backend = (OpticalModel*) getSubInstance("OpticalLinkBackendTx");
        propagatePortTransitionInfo(tx_backend, "In", "In");
        propagatePortTransitionInfo(tx_backend, "LinkCK", "LinkCK");
        tx_backend->use();
        
        // Set transition info for the modulator
        OpticalModel* modulator = (OpticalModel*) getSubInstance("Modulator");
        propagatePortTransitionInfo(modulator, "In", tx_backend, "Out");
        modulator->use();
        
        // Modulator out transition info
        const TransitionInfo& mod_out_transitions = modulator->getOpticalOutputPort("Out")->getTransitionInfo();
        
        // Set transition info for the receiver
        OpticalModel* detector = (OpticalModel*) getSubInstance("Detector");
        detector->getOpticalInputPort("In")->setTransitionInfo(mod_out_transitions);
        detector->use();            
        
        // Propagate transition info to tx backend
        OpticalModel* rx_backend = (OpticalModel*) getSubInstance("OpticalLinkBackendRx");
        propagatePortTransitionInfo(rx_backend, "In", detector, "Out");
        propagatePortTransitionInfo(rx_backend, "LinkCK", "LinkCK");
        rx_backend->use();

        // Propagate output transition info to output
        propagatePortTransitionInfo("Out", rx_backend, "Out");

        // Set enable signals for the laser, if applicable
        if (laser_type == "Throttled")
        {
            // Figure out how many cycles the laser needs to be on
            double cycles = getInputPort("In")->getTransitionInfo().getFrequencyMultiplier();
        
            OpticalModel* laser = (OpticalModel*) getSubInstance("Laser");
            laser->getInputPort("LaserEnable")->setTransitionInfo(TransitionInfo(0.0, 1.0, cycles - 1.0));
            laser->use();
        }
        
        
        return;
    }

    void SWSRLink::buildLaser()
    {
        // Get parameters
        unsigned int number_wavelengths = getGenProperties()->get("NumberWavelengths");
        const String& laser_type = getParameter("LaserType");

        // Create laser
        OpticalModel* laser = NULL;
        if (laser_type == "Throttled") laser = new ThrottledLaserSource("Laser", getTechModel());
        else if (laser_type == "Standard") laser = new LaserSource("Laser", getTechModel());        
        else ASSERT(false, "[Error] " + getInstanceName() + " -> Unknown laser type '" + laser_type + "'!");
        
        laser->setParameter("OutStart", 0);
        laser->setParameter("OutEnd", number_wavelengths-1);
        laser->setParameter("MaxDetectors", 1);
        laser->setParameter("MinDetectors", 1);
        laser->construct();        

        addSubInstances(laser, 1.0);
        getAreaResult("Photonic")->addSubResult(laser->getAreaResult("Photonic"), "Laser", 1.0);
        // Connect laser output port
        opticalPortConnect(laser, "Out", "LaserToMod");    

        // Without laser gating, laser is pure NDD power
        if (laser_type == "Standard") getNddPowerResult("Laser")->addSubResult(laser->getNddPowerResult("Laser"), "Laser", 1.0);
        // With laser power gating, laser is an event
        else getEventResult("Send")->addSubResult(laser->getEventResult("Laser1"), "Laser", 1.0);
        
        return;
    }
    
    void SWSRLink::buildModulator()
    {
        // Get parameters
        double link_data_rate = getParameter("LinkDataRate");
        const String& optimize_loss = getParameter("OptimizeLoss");
        unsigned int number_wavelengths = getGenProperties()->get("NumberWavelengths");

        // Create modulator
        RingModulator* modulator = new RingModulator("Modulator", getTechModel());
        modulator->setParameter("DataRate", link_data_rate);
        modulator->setParameter("InStart", 0);
        modulator->setParameter("InEnd", number_wavelengths-1);
        modulator->setParameter("ModStart", 0);
        modulator->setParameter("ModEnd", number_wavelengths-1);
        modulator->setParameter("OptimizeLoss", optimize_loss);
        modulator->construct();
        addSubInstances(modulator, 1.0);
        getAreaResult("Photonic")->addSubResult(modulator->getAreaResult("Photonic"), "Modulator", 1.0);
        addElectricalSubResults(modulator, 1.0);

        // Connect electrical port
        portConnect(modulator, "In", "TxBackendToTx");
        // Connect modulator input, output port
        opticalPortConnect(modulator, "In", "LaserToMod");
        opticalPortConnect(modulator, "Out", "ModToDetector");
        
        // Add modulator energy event for send events
        getEventResult("Send")->addSubResult(modulator->getEventResult("Modulate"), "Modulator", 1.0);
        return;
    }
    
    void SWSRLink::buildDetector()
    {
        // Get parameters
        double link_data_rate = getParameter("LinkDataRate");
        unsigned int number_wavelengths = getGenProperties()->get("NumberWavelengths");

        // Create resonant ring detector
        RingDetector* detector = new RingDetector("Detector", getTechModel());
        detector->setParameter("DataRate", link_data_rate);
        detector->setParameter("InStart", 0);
        detector->setParameter("InEnd", number_wavelengths-1);
        detector->setParameter("DetStart", 0);
        detector->setParameter("DetEnd", number_wavelengths-1);
        detector->setParameter("DropAll", "TRUE");
        detector->setParameter("Topology", RingDetector::INTEGRATINGSENSEAMP);
        detector->construct();
        addSubInstances(detector, 1.0);
        getAreaResult("Photonic")->addSubResult(detector->getAreaResult("Photonic"), "Detector", 1.0);
        addElectricalSubResults(detector, 1.0);

        // connect to electrical port
        portConnect(detector, "Out", "RxToRxBackend");
        // connect optical input, output port           
        opticalPortConnect(detector, "In", "ModToDetector");
        
        // Add receiver energy
        getEventResult("Send")->addSubResult(detector->getEventResult("Receive"), "Detector", 1.0);
        
        return;
    }
    
} // namespace DSENT

