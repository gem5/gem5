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


#include "model/optical_graph/OpticalModulator.h"
#include "model/optical_graph/OpticalTransmitter.h"

namespace DSENT
{
    OpticalModulator::OpticalModulator(const String& instance_name_, OpticalModel* model_, const WavelengthGroup& wavelengths_, bool opt_loss_, OpticalTransmitter* transmitter_)
        : OpticalNode(OpticalNode::MODULATOR, instance_name_, model_, wavelengths_), m_transmitter_(transmitter_), m_insertion_loss_(0), m_extinction_ratio_(0), m_opt_loss_(opt_loss_)
    {

    }

    OpticalModulator::~OpticalModulator()
    {

    }

    bool OpticalModulator::canOptimizeLoss() const
    {
        return m_opt_loss_;
    }
    
    void OpticalModulator::setLosses(double IL_dB_, double ER_dB_)
    {
        m_insertion_loss_ = IL_dB_;
        m_extinction_ratio_ = ER_dB_;
        
        return;
    }

    bool OpticalModulator::setModulatorSpec(double IL_dB_, double ER_dB_)
    {
        // Ask the transmitter to design to those specs, returns success or fail
        return m_transmitter_->setTransmitterSpec(IL_dB_, ER_dB_);
    }
    
    double OpticalModulator::getPower(double util_) const
    {
        return m_transmitter_->getPower(util_);
    }
        
    double OpticalModulator::getInsertionLoss() const
    {
        return m_insertion_loss_;
    }

    double OpticalModulator::getExtinctionRatio() const
    {
        return m_extinction_ratio_;
    }
            
} // namespace DSENT


