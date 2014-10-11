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

#ifndef __DSENT_MODEL_OPTICAL_RINGDETECTOR_H__
#define __DSENT_MODEL_OPTICAL_RINGDETECTOR_H__

#include "util/CommonType.h"
#include "model/OpticalModel.h"
#include "model/optical_graph/OpticalReceiver.h"

namespace DSENT
{
    class RingDetector : public OpticalModel, public OpticalReceiver
    {
        public:
            // Receiver topolgy strings
            static const String INTEGRATINGSENSEAMP;
    
        public:
            RingDetector(const String& instance_name_, const TechModel* tech_model_);
            virtual ~RingDetector();

        public:
            // Set a list of properties' name needed to construct model
            void initParameters();
            // Set a list of properties' name needed to construct model
            void initProperties();
            // Returns the sensitivity of the receiver given an extinction ratio
            double getSensitivity(double ER_dB_) const;
            
        private:
            // Precompute values based on tech parameters
            void precomputeTech();
            // Design the receiver helper function
            void designReceiver();
            // Calculates inverse normal cdf
            double calcInvNormCdf(double num_);

        protected:
            // Build the model
            virtual void constructModel();
            virtual void updateModel();
            virtual void useModel();
            virtual void propagateTransitionInfo();
            
        private:
            // Precomputed numbers
            double m_quad_a_;
            double m_quad_b1_;
            double m_quad_b2_;
            double m_quad_c_;

    }; // class RingDetector
} // namespace DSENT

#endif // __DSENT_MODEL_OPTICAL_RINGDETECTOR_H__

