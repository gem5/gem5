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


#include "model/optical_graph/OpticalFilter.h"

namespace DSENT
{
    OpticalFilter::OpticalFilter(const String& instance_name_, OpticalModel* model_, const WavelengthGroup& wavelengths_, bool drop_all_, const WavelengthGroup& drop_wavelengths_)
        : OpticalNode(OpticalNode::FILTER, instance_name_, model_, wavelengths_), m_drop_all_(drop_all_), m_drop_wavelengths_(drop_wavelengths_)
    {
        m_drop_loss_ = 0.0;
        m_drop_port_ = NULL;
    }

    OpticalFilter::~OpticalFilter()
    {

    }
    
    bool OpticalFilter::getDropAll() const
    {
        return m_drop_all_;
    }

    WavelengthGroup OpticalFilter::getDropWavelengths() const
    {
        return m_drop_wavelengths_;
    }

    void OpticalFilter::setDropLoss(double drop_loss_)
    {
        m_drop_loss_ = drop_loss_;
        return;
    }
    
    double OpticalFilter::getDropLoss() const
    {
        return m_drop_loss_;
    }

    void OpticalFilter::setDropPort(OpticalNode* drop_port_)
    {
        m_drop_port_ = drop_port_;
    }
    
    OpticalNode* OpticalFilter::getDropPort()
    {
        return m_drop_port_;
    }
    
    bool OpticalFilter::isDropped(const WavelengthGroup& wavelengths_) const
    {
        // Check that the lower limits are within bounds
        bool lower_match = (wavelengths_.first >= getDropWavelengths().first);
        // Check that the upper limits are within bounds
        bool upper_match = (wavelengths_.second <= getDropWavelengths().second);
        // Assert that there are no misalignments
        ASSERT(lower_match == upper_match, "[Error] " + getInstanceName() +
            " -> Wavelength group misalignment!" + 
            " InWavelength" + toString(wavelengths_) + 
            ", DropWavelength" + toString(getDropWavelengths()));
        // Both upper and lower bounds must match
        return (upper_match && lower_match);
    }
} // namespace DSENT


