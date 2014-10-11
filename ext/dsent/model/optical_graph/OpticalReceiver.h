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

#ifndef __DSENT_MODEL_OPTICALGRAPH_OPTICALRECEIVER_H__
#define __DSENT_MODEL_OPTICALGRAPH_OPTICALRECEIVER_H__

#include "model/OpticalModel.h"
#include "util/CommonType.h"

namespace DSENT
{
    // The job of an optical receiver interface is to provide a function to
    // return a sensitivity (in Amps)
    class OpticalReceiver
    {
        public:
            OpticalReceiver(){};
            virtual ~OpticalReceiver(){};

        public:
            // Returns the sensitivity of the receiver given an extinction ratio
            virtual double getSensitivity(double ER_dB_) const = 0;
    };
    
} // namespace DSENT

#endif // __DSENT_MODEL_OPTICALGRAPH_OPTICALRECEIVER_H__

