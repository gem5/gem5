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

#ifndef __DSENT_MODEL_ELECTRICAL_NET_H__
#define __DSENT_MODEL_ELECTRICAL_NET_H__

#include "util/CommonType.h"
#include "model/timing_graph/ElectricalTimingNode.h"

namespace DSENT
{
    class ElectricalLoad;

    class ElectricalNet : public ElectricalTimingNode
    {
        public:
            ElectricalNet(const String& instance_name_, ElectricalModel* model_);
            virtual ~ElectricalNet();

        public:
            // Set distributed res/cap
            void setDistributedRes(double distributed_res_);
            void setDistributedCap(double distributed_cap_);
            // Get distributed res/cap
            double getDistributedRes() const;
            double getDistributedCap() const;
            // Calculate wiring delay (or net delay)
            double calculateDelay() const;
            // Calculate transition
            double calculateTransition() const;
            // get maximum of upstream drive resistance
            double getMaxUpstreamRes() const;
            // get total amount of downstream load capacitance 
            double getTotalDownstreamCap() const;

            virtual bool isNet() const;
            
        private:
            // Disable copy constructor
            ElectricalNet(const ElectricalNet& net_);

        private:
            // Name of this instance
            String m_instance_name_;
            // Distributed capacitance and resistance of the net
            double m_distributed_res_;            
            double m_distributed_cap_;
    };
    
} // namespace DSENT

#endif // __DSENT_MODEL_ELECTRICAL_NET_H__

