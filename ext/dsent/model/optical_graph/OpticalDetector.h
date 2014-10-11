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

#ifndef __DSENT_MODEL_OPTICALGRAPH_OPTICALDETECTOR_H__
#define __DSENT_MODEL_OPTICALGRAPH_OPTICALDETECTOR_H__

#include "model/optical_graph/OpticalNode.h"
#include "util/CommonType.h"

namespace DSENT
{
    class OpticalReceiver;

    class OpticalDetector : public OpticalNode
    {
        public:
            OpticalDetector(const String& instance_name_, OpticalModel* model_, const WavelengthGroup& wavelengths_, OpticalReceiver* receiver_);
            ~OpticalDetector();

        public:
            // Set the responsitivity of the photodetector
            void setResponsivity(double responsivity_);
            
            // Get the detector sensitivity given an extinction ratio (in Watts)
            double getSensitivity(double ER_dB_) const;
            
            // Ask the receiver for its power (ONLY use for power optimization, as this
            // assumes an activity of 1.0)
            double getPower() const;
        
        private:
            // Disable copy constructor
            OpticalDetector(const OpticalDetector& node_);            
        
        private:
            // The required laser power
            double m_sensitivity_;
            // The receiver connected to this detector
            OpticalReceiver* m_receiver_;
            // Responsivity of the photodetector
            double m_responsivity_;
        
    };
    
} // namespace DSENT

#endif // __DSENT_MODEL_OPTICALGRAPH_OPTICALDETECTOR_H__

