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

#ifndef __DSENT_MODEL_OPTICALGRAPH_OPTICALGRAPH_H__
#define __DSENT_MODEL_OPTICALGRAPH_OPTICALGRAPH_H__

#include <vector>

#include "util/CommonType.h"
#include "model/optical_graph/OpticalNode.h"

namespace DSENT
{
    class OpticalNode;
    class OpticalWavelength;
            
    class OpticalGraph
    {    
        public:
            // The visited number for the next timing run. This needs to be
            // global because several timing trees may be created to evaluate
            // a single timing path, causing problems
            static int msTreeNum;
        
        public:
            // Construct timing tree that watches over model_
            OpticalGraph(const String& instance_name_, OpticalModel* model_);
            ~OpticalGraph();
            
        public:
            // Get graph name
            const String& getInstanceName() const;
            // Perform datapath power optimization by balancing insertion loss and extinction
            // ratio with modulator/receiver and laser power, returns false if there are no
            // designs that are possible
            bool performPowerOpt(OpticalNode* node_, const WavelengthGroup& wavelengths_, unsigned int number_detectors_, double util_);
            // Recursively trace a wavelength starting from an OpticalLaser
            // source finding all lasers, modulators and detectors that a
            // wavelength group hits.
            OpticalWavelength* traceWavelength(const WavelengthGroup& wavelengths_, OpticalNode* node_);
            OpticalWavelength* traceWavelength(OpticalWavelength* wavelength_, OpticalNode* node_, OpticalLaser* laser_, OpticalModulator* modulator_, double loss_);
            // Return the model
            OpticalModel* getModel();

        private:

            // Disable the use of copy constructor
            OpticalGraph(const OpticalGraph& graph_);
        
        public:
            // Set the sequence number of the optical graph
            static void setTreeNum(int tree_num_);
            static int getTreeNum();
            
        private:
            // Name of the optical graph
            const String m_instance_name_;
            // A pointer to the model that contains this node
            OpticalModel* m_model_;

    }; // class OpticalGraph
} // namespace DSENT

#endif // __DSENT_MODEL_OPTICALGRAPH_OPTICALGRAPH_H__

