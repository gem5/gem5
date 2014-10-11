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

#ifndef __DSENT_MODEL_ELECTRICAL_LOAD_H__
#define __DSENT_MODEL_ELECTRICAL_LOAD_H__

#include "util/CommonType.h"
#include "model/timing_graph/ElectricalTimingNode.h"

namespace DSENT
{
    class ElectricalModel;
    class ElectricalDriver;
    
    class ElectricalLoad : public ElectricalTimingNode
    {
        public:
            ElectricalLoad(const String& instance_name_, ElectricalModel* model_);
            virtual ~ElectricalLoad();

        public:
            // Set the input capacitance of this input port
            void setLoadCap(double load_cap_);
            // Get the load capacitance
            double getLoadCap() const;
            // Get total load capacitance
            double getTotalDownstreamCap() const;
            // Calculate delay due to total load capacitance
            double calculateDelay() const;
            // Calculate transition
            double calculateTransition() const;

            bool isLoad() const;
            
        private:
            // Disable copy constructor
            ElectricalLoad(const ElectricalLoad& load_);

        private:
            // Load capacitance
            double m_load_cap_;
    };
} // namespace DSENT

#endif // __DSENT_MODEL_ELECTRICAL_LOAD_H__

