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

#ifndef __DSENT_MODEL_STD_CELLS_DFFQ_H__
#define __DSENT_MODEL_STD_CELLS_DFFQ_H__

#include "util/CommonType.h"
#include "model/std_cells/StdCell.h"
#include "model/TransitionInfo.h"

namespace DSENT
{
    class DFFQ : public StdCell
    {
        // A DQ flip-flop
        public:
            DFFQ(const String& instance_name_, const TechModel* tech_model_);
            virtual ~DFFQ();

        public:
            // Set a list of properties' name needed to construct model
            void initProperties();
            // Cache the standard cell
            void cacheStdCell(StdCellLib* cell_lib_, double drive_strength_);
            
        private:
            TransitionInfo m_trans_M_;

        protected:
            // Build the model            
            virtual void constructModel();
            virtual void updateModel();
            virtual void evaluateModel();
            virtual void useModel();
            virtual void propagateTransitionInfo();

    }; // class DFFQ
} // namespace DSENT

#endif // __DSENT_MODEL_STD_CELLS_INV_H__

