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


#include "model/timing_graph/ElectricalNet.h"
#include "model/timing_graph/ElectricalLoad.h"

namespace DSENT
{
    //-------------------------------------------------------------------------
    // Electrical Net
    //-------------------------------------------------------------------------

    ElectricalNet::ElectricalNet(const String& instance_name_, ElectricalModel* model_)
        : ElectricalTimingNode(instance_name_, model_), m_distributed_res_(0), m_distributed_cap_(0)
    {

    }

    ElectricalNet::~ElectricalNet()
    {

    }

    double ElectricalNet::calculateDelay() const
    {
        // Remember that this is a pi model, delay is distributed cap * distributed_res / 2 +
        // distributed res * (other downstream caps)
        return 0.693 * (getTotalDownstreamCap() - m_distributed_cap_ / 2) * m_distributed_res_;
    }

    double ElectricalNet::calculateTransition() const
    {
        return 1.386 * getMaxUpstreamRes() * (m_distributed_cap_ * 0.2 + ElectricalTimingNode::getTotalDownstreamCap());
    }

    double ElectricalNet::getMaxUpstreamRes() const
    {
        return m_distributed_res_ + ElectricalTimingNode::getMaxUpstreamRes();
    }
    
    double ElectricalNet::getTotalDownstreamCap() const
    {
        return m_distributed_cap_ + ElectricalTimingNode::getTotalDownstreamCap();
    }
    
    void ElectricalNet::setDistributedCap(double distributed_cap_)
    {
        m_distributed_cap_ = distributed_cap_;
        return;
    }

    void ElectricalNet::setDistributedRes(double distributed_res_)
    {
        m_distributed_res_ = distributed_res_;
        return;
    }

    double ElectricalNet::getDistributedCap() const
    {
        return m_distributed_cap_;
    }

    double ElectricalNet::getDistributedRes() const
    {
        return m_distributed_res_;
    }
    
    bool ElectricalNet::isNet() const
    {
        return true;
    }    
    
} // namespace DSENT


