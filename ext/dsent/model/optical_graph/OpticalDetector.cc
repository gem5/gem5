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


#include "model/optical_graph/OpticalDetector.h"
#include "model/optical_graph/OpticalReceiver.h"

namespace DSENT
{
    OpticalDetector::OpticalDetector(const String& instance_name_, OpticalModel* model_, const WavelengthGroup& wavelengths_, OpticalReceiver* receiver_)
        : OpticalNode(OpticalNode::DETECTOR, instance_name_, model_, wavelengths_), m_receiver_(receiver_), m_responsivity_(0)
    {
        m_sensitivity_ = 0.0;
    }

    OpticalDetector::~OpticalDetector()
    {

    }
    
    void OpticalDetector::setResponsivity(double responsivity_)
    {
        m_responsivity_ = responsivity_;
        return;
    }
    
    double OpticalDetector::getSensitivity(double ER_dB_) const
    {
        // Get responsivity (in Amps) of the receiver, divide by responsivity to get sensitivity in Watts
        return m_receiver_->getSensitivity(ER_dB_) / m_responsivity_;
    }
                
} // namespace DSENT


