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


#include "model/timing_graph/ElectricalDriver.h"
#include "model/timing_graph/ElectricalNet.h"
#include "model/ElectricalModel.h"

namespace DSENT
{
    ElectricalDriver::ElectricalDriver(const String& instance_name_, ElectricalModel* model_, bool sizable_)
        : ElectricalTimingNode(instance_name_, model_), m_output_res_(0.0), m_sizable_(sizable_)
    {

    }

    ElectricalDriver::~ElectricalDriver()
    {

    }
    
    void ElectricalDriver::setOutputRes(double output_res_)
    {
        m_output_res_ = output_res_;
        return;
    }

    double ElectricalDriver::getOutputRes() const
    {        
        return m_output_res_;
    }

    double ElectricalDriver::calculateDelay() const
    {
        return 0.693 * m_output_res_ * getTotalDownstreamCap();
    }    

    double ElectricalDriver::calculateTransition() const
    {
        return 1.386 * getMaxUpstreamRes() * getTotalDownstreamCap();
    }

    double ElectricalDriver::getMaxUpstreamRes() const
    {
        return m_output_res_;
    }

    bool ElectricalDriver::isSizable() const
    {
        return m_sizable_;
    }
    
    bool ElectricalDriver::hasMaxDrivingStrength() const
    {
        if (!isSizable())
        {
            return true;
        }
        return (getModel() == NULL) || (getModel()->hasMaxDrivingStrength());
    }

    bool ElectricalDriver::hasMinDrivingStrength() const
    {
        if (!isSizable())
        {
            return true;
        }
        return (getModel() == NULL) || (getModel()->hasMinDrivingStrength());
    }
    
    void ElectricalDriver::increaseDrivingStrength()
    {
        ASSERT(isSizable(), "[Error] " + getInstanceName() + 
            " -> Attempted to size up unsizable driver!");
        if(!hasMaxDrivingStrength())
        {
            getModel()->increaseDrivingStrength();
        }
        return;
    }

    void ElectricalDriver::decreaseDrivingStrength()
    {
        ASSERT(isSizable(), "[Error] " + getInstanceName() + 
            " -> Attempted to size down unsizable driver!");
        if(!hasMinDrivingStrength())
        {
            getModel()->decreaseDrivingStrength();
        }
        return;
    }
    
    bool ElectricalDriver::isDriver() const
    {
        return true;
    }    
} // namespace DSENT

