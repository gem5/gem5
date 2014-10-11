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

#ifndef __DSENT_MODEL_ELECTRICAL_BROADCAST_HTREE_H__
#define __DSENT_MODEL_ELECTRICAL_BROADCAST_HTREE_H__

#include "util/CommonType.h"
#include "model/ElectricalModel.h"

#include <vector>

namespace DSENT
{
    using std::vector;

    class StdCell;
    class ElectricalLoad;
    class ElectricalTimingTree;

    class BroadcastHTree : public ElectricalModel
    {
        public:
            BroadcastHTree(const String& instance_name_, const TechModel* tech_model_);
            virtual ~BroadcastHTree();

        public:
            // Set a list of properties' name needed to construct model
            void initParameters();
            // Set a list of properties' name needed to construct model
            void initProperties();

            // Clone and return a new instance
            virtual BroadcastHTree* clone() const;

        protected:
            // Build the model
            virtual void constructModel();
            virtual void updateModel();
            virtual void useModel();
            virtual void propagateTransitionInfo();

        private:
            vector<StdCell*> m_repeaters_;
            vector<ElectricalLoad*> m_repeater_loads_;
            vector<ElectricalTimingTree*> m_timing_trees_;
            vector<unsigned int> m_number_segments_;

            vector<StdCell*> m_leaf_drivers_;
            ElectricalLoad* m_leaf_load_;
            StdCell* m_leaf_head_driver_;
            ElectricalLoad* m_leaf_head_load_;

    }; // class BroadcastHTree
} // namespace DSENT

#endif // __DSENT_MODEL_ELECTRICAL_BROADCAST_HTREE_H__

