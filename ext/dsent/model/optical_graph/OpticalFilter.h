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

#ifndef __DSENT_MODEL_OPTICALGRAPH_OPTICALFILTER_H__
#define __DSENT_MODEL_OPTICALGRAPH_OPTICALFILTER_H__

#include "model/optical_graph/OpticalNode.h"
#include "util/CommonType.h"

namespace DSENT
{
    class OpticalFilter : public OpticalNode
    {
        public:
            OpticalFilter(const String& instance_name_, OpticalModel* model_, const WavelengthGroup& wavelengths_, bool drop_all_, const WavelengthGroup& drop_wavelengths_);
            ~OpticalFilter();

        public:
            // Get the drop all flag
            bool getDropAll() const;
            // Get drop wavelengths
            WavelengthGroup getDropWavelengths() const;
            // Set and get the drop loss
            void setDropLoss(double drop_loss_);
            double getDropLoss() const;
            // Set and get drop port
            void setDropPort(OpticalNode* drop_port_);
            OpticalNode* getDropPort();
            // Checks to see if a set of wavelengths will be dropped
            bool isDropped(const WavelengthGroup& wavelengths_) const;
            
        private:
            // Disable copy constructor
            OpticalFilter(const OpticalFilter& node_);

        private:
            // Whether to drop all the optical signal for the drop wavelengths
            // i.e. so that the drop wavelengths are not traced anymore
            const bool m_drop_all_;
            // The loss incurred from in to drop port
            double m_drop_loss_;
            // The wavelengths that are dropped
            const WavelengthGroup m_drop_wavelengths_;
            // The node at the drop port
            OpticalNode* m_drop_port_;
    };
    
} // namespace DSENT

#endif // __DSENT_MODEL_OPTICALGRAPH_OPTICALFILTER_H__

