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


#include "model/optical_graph/OpticalWavelength.h"
#include "model/optical_graph/OpticalNode.h"
#include "model/optical_graph/OpticalLaser.h"
#include "model/optical_graph/OpticalModulator.h"
#include "model/optical_graph/OpticalFilter.h"
#include "model/optical_graph/OpticalDetector.h"
#include "model/optical_graph/OpticalWavelength.h"
#include <list>
#include <cmath>

namespace DSENT
{
    using std::list;
    using std::min;

    OpticalWavelength::OpticalWavelength(const String& instance_name_, const WavelengthGroup& wavelengths_)
        : m_instance_name_(instance_name_), m_wavelengths_(wavelengths_)
    {
        m_data_paths_ = new vector<OpticalDataPath>;
    }

    OpticalWavelength::~OpticalWavelength()
    {
        delete m_data_paths_;
    }

    const String& OpticalWavelength::getInstanceName() const
    {
        return m_instance_name_;
    }
    
    void OpticalWavelength::addDataPath(OpticalLaser* laser_, OpticalModulator* modulator_, OpticalDetector* detector_, double loss_)
    {
        // Expected wavelengths check
        ASSERT(laser_->isExpected(getWavelengths()), "[Error] " + getInstanceName() +
            " -> " + laser_->getInstanceName() + " is not expecting the set wavelengths!");        
        ASSERT(modulator_->isExpected(getWavelengths()), "[Error] " + getInstanceName() +
            " -> " + modulator_->getInstanceName() + " is not expecting the set wavelengths!");       
        ASSERT(detector_->isExpected(getWavelengths()), "[Error] " + getInstanceName() +
            " -> " + detector_->getInstanceName() + " is not expecting the set wavelengths!");       
        
        // Check to see if the modulator and laser already have a data path entry
        bool entry_exists = false;
        for (unsigned int i = 0; i < m_data_paths_->size(); ++i)
        {
            OpticalDataPath& current = m_data_paths_->at(i);
            bool current_laser = current.laser == laser_;
            bool current_modulator = current.modulator == modulator_;            

            ASSERT((current_modulator && current_laser) || !current_modulator, "[Error] " +
                getInstanceName() + " -> Modulator is the same, but laser is different?");
            
            // If it is already in the table
            if (current_modulator)
            {
                entry_exists = true;                
                current.detectors.push_back(detector_);
                current.losses.push_back(loss_);
            }
        }
        
        // If it wasn't found, add the entry
        if (!entry_exists)
            m_data_paths_->push_back(OpticalDataPath(laser_, modulator_, detector_, loss_));
        return;
    }
    
    const vector<OpticalDataPath>* OpticalWavelength::getDataPaths() const
    {
        return (const vector<OpticalDataPath>*) m_data_paths_;
    }
    
    WavelengthGroup OpticalWavelength::getWavelengths() const
    {
        return m_wavelengths_;
    }
    
    double OpticalWavelength::getLaserPower(unsigned int number_detectors_) const
    {
        ASSERT(number_detectors_ > 0, "[Error] " + getInstanceName() +
            " -> Number of detectors must be non-zero!");
        // Find the number of actual wavelengths
        int number_wavelengths = getWavelengths().second - getWavelengths().first + 1;
        // Laser power sum
        double laser_power_sum = 0;        
        // Loop through all data paths
        for (unsigned int i = 0; i < getDataPaths()->size(); ++i)
        {
            // Get the current data_path
            const OpticalDataPath& current_path = getDataPaths()->at(i);        
            // Create data structure holding the worstcase detectors
            list<double>* detectors = new list<double>();
            // Get the extinction ratio of the modulator
            double ER_dB = current_path.modulator->getExtinctionRatio();
            // Get the insertion loss of the modulator
            double IR_dB = current_path.modulator->getInsertionLoss();
            // Walk through all detectors in a data path
            for (unsigned int j = 0; j < current_path.detectors.size(); ++j)
            {
                // Convert sensitivity, extinction ratio, and path loss to a required laser power
                double current_laser_power = current_path.detectors[j]->getSensitivity(ER_dB) *
                    std::pow(10.0, (current_path.losses[j] + IR_dB) / 10.0) * 
                    1.0 / (1.0 - pow(10, -ER_dB / 10));
                    
                // Add the laser power
                detectors->push_back(current_laser_power);
            }
            // Cap the number of detectors
            number_detectors_ = std::min(number_detectors_, (unsigned int) current_path.detectors.size());        
            // Sort the detectors list in ascending order, only necessary if the number
            // of detectors is < total number of detectors
            if (number_detectors_ < detectors->size())
                detectors->sort();
            // Sum up the laser power from the worst-case detectors 
            list<double>::reverse_iterator iter = detectors->rbegin();
            for (unsigned int j = 0; j < number_detectors_; ++j)
            {
                laser_power_sum += (*iter) / current_path.laser->getEfficiency();
                ++iter;
            }                    
            delete detectors;
        }
        return number_wavelengths * laser_power_sum;        
    }
            
} // namespace DSENT


