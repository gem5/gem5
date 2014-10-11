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


#include "model/timing_graph/ElectricalDelay.h"
#include "model/timing_graph/ElectricalLoad.h"

namespace DSENT
{
    //-------------------------------------------------------------------------
    // Electrical Delay
    //-------------------------------------------------------------------------

    ElectricalDelay::ElectricalDelay(const String& instance_name_, ElectricalModel* model_)
        : ElectricalTimingNode(instance_name_, model_), m_delay_(0.0)
    {

    }

    ElectricalDelay::~ElectricalDelay()
    {

    }

    void ElectricalDelay::setDelay(double delay_)
    {
        m_delay_ = delay_;
        return;
    }

    double ElectricalDelay::getDelay() const
    {
        return m_delay_;
    }
    
    double ElectricalDelay::calculateDelay() const
    {
        return m_delay_;
    }

    double ElectricalDelay::calculateTransition() const
    {
        return 1.386 * getMaxUpstreamRes() * getTotalDownstreamCap();
    }

    double ElectricalDelay::getMaxUpstreamRes() const
    {
        return ElectricalTimingNode::getMaxUpstreamRes();
    }
    
    double ElectricalDelay::getTotalDownstreamCap() const
    {
        return ElectricalTimingNode::getTotalDownstreamCap();
    }
    
} // namespace DSENT


