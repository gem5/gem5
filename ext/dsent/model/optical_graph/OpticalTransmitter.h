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

#ifndef __DSENT_MODEL_OPTICALGRAPH_OPTICALTRANSMITTER_H__
#define __DSENT_MODEL_OPTICALGRAPH_OPTICALTRANSMITTER_H__

#include "model/OpticalModel.h"
#include "util/CommonType.h"

namespace DSENT
{
    // The job of a optical sender interface is to provide a function to
    // allow the insertion loss and extinction ratio to be changed
    class OpticalTransmitter
    {
        public:
            OpticalTransmitter(){};
            virtual ~OpticalTransmitter(){};

        public:
            // Set the transmitter specifications, returns whether it is possible
            // to build a modulator that met those specs
            virtual bool setTransmitterSpec(double IL_dB_, double ER_dB_) = 0;
            // Returns power of the transmitter at a given utilization
            virtual double getPower(double util_) const = 0;
    }; // class OpticalTransmitter
    
} // namespace DSENT

#endif // __DSENT_MODEL_OPTICALGRAPH_OPTICALTRANSMITTER_H__

