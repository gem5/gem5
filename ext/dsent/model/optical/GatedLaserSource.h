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

#ifndef __DSENT_MODEL_OPTICAL_GATEDLASERSOURCE_H__
#define __DSENT_MODEL_OPTICAL_GATEDLASERSOURCE_H__

#include "util/CommonType.h"
#include "model/OpticalModel.h"

namespace DSENT
{
    // A laser source that outputs some number of wavelengths. This laser
    // full on/off power gating, thus all power are event-based energies
    class GatedLaserSource : public OpticalModel
    {
        public:
            GatedLaserSource(const String& instance_name_, const TechModel* tech_model_);
            virtual ~GatedLaserSource();

        public:
            // Set a list of properties' name needed to construct model
            void initParameters();
            // Set a list of properties' name needed to construct model
            void initProperties();

        protected:
            // Build the model
            void constructModel();
            void updateModel();
            void evaluateModel();
            
    }; // class GatedLaserSource
} // namespace DSENT

#endif // __DSENT_MODEL_OPTICAL_GATEDLASERSOURCE_H__

