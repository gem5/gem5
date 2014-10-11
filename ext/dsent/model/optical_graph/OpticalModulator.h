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

#ifndef __DSENT_MODEL_OPTICALGRAPH_OPTICALMODULATOR_H__
#define __DSENT_MODEL_OPTICALGRAPH_OPTICALMODULATOR_H__

#include "model/optical_graph/OpticalNode.h"
#include "util/CommonType.h"

namespace DSENT
{
    class OpticalTransmitter;
    
    class OpticalModulator : public OpticalNode
    {
        public:
            OpticalModulator(const String& instance_name_, OpticalModel* model_, const WavelengthGroup& wavelengths_, bool opt_loss_, OpticalTransmitter* transmitter_);
            ~OpticalModulator();

        public:
            // Set losses
            void setLosses(double IL_dB_, double ER_dB_);        
            // Tell the modulator to set a new insertion loss and extinction ratio
            bool setModulatorSpec(double IL_dB_, double ER_dB_);
            // Get modulator insertion loss
            double getInsertionLoss() const;
            // Get modulator extinction ratio
            double getExtinctionRatio() const;
            // Ask whether the model is able to optimize for insertion loss
            // and extinction ratios
            bool canOptimizeLoss() const;
            // Ask the modulator for its power at a given utilization
            double getPower(double util_) const;
        
        private:
            // Disable copy constructor
            OpticalModulator(const OpticalModulator& node_);
        
        private:
            // Optical sender of the modulator
            OpticalTransmitter* m_transmitter_;
            // Insertion loss of the modulator
            double m_insertion_loss_;
            // Extinction ratio of the modulator
            double m_extinction_ratio_;
            // Whether the modulator can be optimized
            bool m_opt_loss_;
        
    };
    
} // namespace DSENT

#endif // __DSENT_MODEL_OPTICALGRAPH_OPTICALMODULATOR_H__

