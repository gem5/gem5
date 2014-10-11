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


#include "model/optical_graph/OpticalLaser.h"

namespace DSENT
{
    OpticalLaser::OpticalLaser(const String& instance_name_, OpticalModel* model_, const WavelengthGroup& wavelengths_)
        : OpticalNode(OpticalNode::LASER, instance_name_, model_, wavelengths_), m_efficiency_(0)
    {
        
    }

    void OpticalLaser::setEfficiency(double efficiency_)
    {
        m_efficiency_ = efficiency_;
        return;
    }
    
    double OpticalLaser::getEfficiency() const
    {
        return m_efficiency_;
    }
    
    OpticalLaser::~OpticalLaser()
    {

    }

            
} // namespace DSENT


