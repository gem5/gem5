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

#include "model/optical/LaserSource.h"

#include "model/optical_graph/OpticalWaveguide.h"
#include "model/optical_graph/OpticalWavelength.h"
#include "model/optical_graph/OpticalLaser.h"
#include "model/optical_graph/OpticalGraph.h"

namespace DSENT
{
    LaserSource::LaserSource(const String& instance_name_, const TechModel* tech_model_)
        : OpticalModel(instance_name_, tech_model_)
    {
        initParameters();
        initProperties();
    }

    LaserSource::~LaserSource()
    {}

    void LaserSource::initParameters()
    {
        addParameterName("OutStart");
        addParameterName("OutEnd");
        addParameterName("MaxDetectors");
        return;
    }

    void LaserSource::initProperties()
    {
		addPropertyName("OptUtil", 1.0);
        return;
    }

    void LaserSource::constructModel()
    {
        // Create Area result
        Result* area_result = new AtomicResult("Photonic");
        addAreaResult(area_result);
        // Create NDD power result
        Result* power_result = new AtomicResult("Laser");
        addNddPowerResult(power_result);
    
        // Get parameters
        WavelengthGroup laser_wavelengths = makeWavelengthGroup(getParameter("OutStart"), getParameter("OutEnd"));

        // Create optical ports
        createOpticalOutputPort(    "Out",      laser_wavelengths);
        // Create the filter
        createLaser(                "Laser",    laser_wavelengths);
        OpticalLaser* laser = getLaser("Laser");
        // Connect the laser to the output
        laser->addDownstreamNode(getWaveguide("Out"));
    }
    
    void LaserSource::updateModel()
    {
        // Get properties
        double laser_efficiency = getTechModel()->get("Laser->CW->Efficiency").toDouble();
        double laser_area = getTechModel()->get("Laser->CW->Area").toDouble();
        double laser_diode_loss = getTechModel()->get("Laser->CW->LaserDiodeLoss");

        // Get parameters
        WavelengthGroup laser_wavelengths = makeWavelengthGroup(getParameter("OutStart"), getParameter("OutEnd"));
        unsigned int number_wavelengths = laser_wavelengths.second - laser_wavelengths.first + 1;
        // Update losses
        OpticalLaser* laser = getLaser("Laser");
        laser->setLoss(laser_diode_loss);
        laser->setEfficiency(laser_efficiency);
        // Update area
        getAreaResult("Photonic")->setValue(laser_area * number_wavelengths);
    }
    
    void LaserSource::evaluateModel()
    {    
        // Get parameters
        unsigned int max_detectors = getParameter("MaxDetectors").toUInt();
        WavelengthGroup laser_wavelengths = makeWavelengthGroup(getParameter("OutStart"), getParameter("OutEnd"));

		// Get properties
		double opt_util = getProperty("OptUtil");
                
        // Create optical graph object
        OpticalGraph* optical_graph = new OpticalGraph("LaserTrace", this);        
        // Ask optical graph object to perform power optimization
        bool success = optical_graph->performPowerOpt(getLaser("Laser"), laser_wavelengths, max_detectors, opt_util);
        if (!success)
        {
            Log::printLine(std::cerr, "[Warning] " + getInstanceName() +
                " -> Wavelengths contains data paths with no possible modulator configurations!");
        }        
        // Trace the wavelengths the laser is outputting to find the output
        // power needed by the laser        
        OpticalWavelength* wavelength = optical_graph->traceWavelength(laser_wavelengths, getLaser("Laser"));                
        // Calculate the power needed by the wavelength
        double laser_power = wavelength->getLaserPower(max_detectors);
        
        // Calculate NDD power
        getNddPowerResult("Laser")->setValue(laser_power);
        
        delete wavelength;
        delete optical_graph;
    }
    
} // namespace DSENT

