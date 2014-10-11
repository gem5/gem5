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

#ifndef __DSENT_MODEL_TIMING_GRAPH_ELECTRICAL_TIMING_OPTIMIZER_H__
#define __DSENT_MODEL_TIMING_GRAPH_ELECTRICAL_TIMING_OPTIMIZER_H__

#include "util/CommonType.h"
#include "model/ElectricalModel.h"

namespace DSENT
{
    // This model is only used to optimize the timing
    class ElectricalTimingOptimizer : public ElectricalModel
    {
        public:
            ElectricalTimingOptimizer(const String& instance_name_, const TechModel* tech_model_);
            virtual ~ElectricalTimingOptimizer();

        public:
            void setModel(ElectricalModel* model_);
            ElectricalModel* getModel();

        protected:
            // Build the optimizer
            virtual void constructModel();

        private:
            ElectricalModel* m_model_;
    }; // class ElectricalTimingOptimizer
} // namespace

#endif // __DSENT_MODEL_TIMING_GRAPH_ELECTRICAL_TIMING_OPTIMIZER_H__

