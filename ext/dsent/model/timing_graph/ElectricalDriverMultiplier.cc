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


#include "model/timing_graph/ElectricalDriverMultiplier.h"
#include "model/timing_graph/ElectricalLoad.h"

namespace DSENT
{
    //-------------------------------------------------------------------------
    // Electrical Net
    //-------------------------------------------------------------------------

    ElectricalDriverMultiplier::ElectricalDriverMultiplier(const String& instance_name_, ElectricalModel* model_)
        : ElectricalTimingNode(instance_name_, model_)
    {

    }

    ElectricalDriverMultiplier::~ElectricalDriverMultiplier()
    {

    }

    double ElectricalDriverMultiplier::calculateDriveRes( double input_drive_res_) const
    {
        return input_drive_res_;
    }

    double ElectricalDriverMultiplier::calculateDelay() const
    {
        // This is just a model helper element, it does not contribute delay
        return 0;
    }

    double ElectricalDriverMultiplier::calculateTransition() const
    {
        return getMaxUpstreamRes() * getTotalDownstreamCap();
    }
    
    double ElectricalDriverMultiplier::getTotalDownstreamCap() const
    {
        // Finds the max of the load caps (as opposed to summing)
        double max_cap = 0;
        vector<ElectricalTimingNode*>* downstream_nodes = ElectricalTimingNode::getDownstreamNodes();
        for (unsigned int i = 0; i < downstream_nodes->size(); ++i)
        {
            max_cap = std::max(max_cap, downstream_nodes->at(i)->getTotalDownstreamCap());
        }
        
        return max_cap;
    }    
    
} // namespace DSENT


