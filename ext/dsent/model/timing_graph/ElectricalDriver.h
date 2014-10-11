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

#ifndef __DSENT_MODEL_ELECTRICAL_DRIVER_H__
#define __DSENT_MODEL_ELECTRICAL_DRIVER_H__

#include "util/CommonType.h"
#include "model/timing_graph/ElectricalTimingNode.h"

namespace DSENT
{
    class ElectricalModel;

    class ElectricalDriver : public ElectricalTimingNode
    {
        public:
            ElectricalDriver(const String& instance_name_, ElectricalModel* model_, bool sizable_);
            virtual ~ElectricalDriver();

        public:
            // Set the output resistance of this driver
            void setOutputRes(double output_res_);            
            // Get the output resistance of this driver
            double getOutputRes() const;
            // Calculate delay due to total load capacitance
            double calculateDelay() const;
            // Calculate transition
            double calculateTransition() const;
            // get maximum of upstream drive resistance
            double getMaxUpstreamRes() const;
            
            // Get whether the driver is sizable
            bool isSizable() const;
            // Return true if the instance has minimum driving strength
            bool hasMinDrivingStrength() const;
            // Return true if the instance has maximum driving strength
            bool hasMaxDrivingStrength() const;
            // Increase driving strength index by 1
            void increaseDrivingStrength();
            // Decrease driving strength index by 1
            void decreaseDrivingStrength();
                        
            bool isDriver() const;                        
                        
        private:
            // Disable copy constructor
            ElectricalDriver(const ElectricalDriver& port_);

        private:
            // Name of this instance
            String m_instance_name_;
            // Output resistance
            double m_output_res_;
            // Sizable flag
            bool m_sizable_;
    };
} // namespace DSENT

#endif // __DSENT_MODEL_ELECTRICAL_DRIVER_H__

