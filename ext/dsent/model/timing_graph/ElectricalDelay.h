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

#ifndef __DSENT_MODEL_ELECTRICAL_DELAY_H__
#define __DSENT_MODEL_ELECTRICAL_DELAY_H__

#include "util/CommonType.h"
#include "model/timing_graph/ElectricalTimingNode.h"

namespace DSENT
{
    class ElectricalLoad;

    class ElectricalDelay : public ElectricalTimingNode
    {
        public:
            ElectricalDelay(const String& instance_name_, ElectricalModel* model_);
            virtual ~ElectricalDelay();

        public:
            // Specify an ideal delay
            void setDelay(double delay_);
            // Get the ideal delay
            double getDelay() const;
            // Calculate delay
            double calculateDelay() const;
            // Calculate transition
            double calculateTransition() const;
            // get maximum of upstream drive resistance
            double getMaxUpstreamRes() const;
            // get total amount of downstream load capacitance 
            double getTotalDownstreamCap() const;
            
        private:
            // Disable copy constructor
            ElectricalDelay(const ElectricalDelay& net_);

        private:
            // The amount of ideal delay
            double m_delay_;
    };
    
} // namespace DSENT

#endif // __DSENT_MODEL_ELECTRICAL_DELAY_H__

