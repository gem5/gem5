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

#ifndef __DSENT_MODEL_OPTICAL_SWMRLINK_H__
#define __DSENT_MODEL_OPTICAL_SWMRLINK_H__

#include "util/CommonType.h"
#include "model/OpticalModel.h"

namespace DSENT
{
    class SWMRLink : public OpticalModel
    {
        // A SWMR Link consists of a laser, a modulator (the writer) and a variable
        // number of readers
        public:
            SWMRLink(const String& instance_name_, const TechModel* tech_model_);
            virtual ~SWMRLink();

        public:
            // Set a list of properties' name needed to construct model
            void initParameters();
            // Set a list of properties' name needed to construct model
            void initProperties();

        protected:
            // Build the model
            virtual void constructModel();
            virtual void updateModel();
            virtual void propagateTransitionInfo();
            
        private:
            void buildLaser();
            void buildModulator();
            void buildDetectors();

    }; // class SWMRLink
} // namespace DSENT

#endif // __DSENT_MODEL_OPTICAL_SWMRLINK_H__

