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

#ifndef __DSENT_MODEL_OPTICALGRAPH_OPTICALWAVELENGTH_H__
#define __DSENT_MODEL_OPTICALGRAPH_OPTICALWAVELENGTH_H__

#include "model/OpticalModel.h"
#include "util/CommonType.h"

namespace DSENT
{
    // Optical datapath structure storing a detector table consisting of a
    // detector, the loss to that detector, and the modulator driving
    // the wavelength for that detector
    struct OpticalDataPath
    {
        OpticalLaser* laser;
        OpticalModulator* modulator;
        vector<OpticalDetector*> detectors;
        vector<double> losses;
        
        OpticalDataPath(OpticalLaser* laser_, OpticalModulator* modulator_, OpticalDetector* detector_, double loss_)
            : laser(laser_), modulator(modulator_), detectors(1, detector_), losses(1, loss_) {}
    };

    class OpticalWavelength
    {
        // A data structure of a wavelength (or a group of wavelengths). This
        // keeps track of all lasers sources, modulators, and detectors that
        // the wavelength hits.
        public:
            OpticalWavelength(const String& instance_name_, const WavelengthGroup& wavelengths_);
            ~OpticalWavelength();

        public:
            // Get tree name
            const String& getInstanceName() const;
            // Get wavelength groups
            WavelengthGroup getWavelengths() const;
            // Add a datapath for this wavelength
            void addDataPath(OpticalLaser* laser_, OpticalModulator* modulator_, OpticalDetector* detector_, double loss_);
            const vector<OpticalDataPath>* getDataPaths() const;
            // Calculate required wavelength power to reach some number of detectors
            // If number_detectors < the number of total detectors this wavelength hits then
            // it simply returns the laser power required to reach the worst-case detectors
            double getLaserPower(unsigned int number_detectors_) const;
        
        private:
            // Name of the wavelength
            const String m_instance_name_;
            // Keeps track of the wavelengths
            const WavelengthGroup m_wavelengths_;
            // Keeps track of a table of laser, detector, modulator mappings
            vector<OpticalDataPath>* m_data_paths_;            
    };
    
} // namespace DSENT

#endif // __DSENT_MODEL_OPTICALGRAPH_OPTICALWAVEGUIDE_H__

