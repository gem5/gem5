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


#include "model/optical_graph/OpticalGraph.h"

#include "model/OpticalModel.h"
#include "model/optical_graph/OpticalNode.h"
#include "model/optical_graph/OpticalLaser.h"
#include "model/optical_graph/OpticalModulator.h"
#include "model/optical_graph/OpticalFilter.h"
#include "model/optical_graph/OpticalDetector.h"
#include "model/optical_graph/OpticalWavelength.h"

namespace DSENT
{
    // Initialize the next visited number to be one above the initial number
    // used by OpticalNode
    int OpticalGraph::msTreeNum = OpticalNode::OPTICAL_NODE_INIT_VISITED_NUM + 1;

    OpticalGraph::OpticalGraph(const String& instance_name_, OpticalModel* model_)
        : m_instance_name_(instance_name_), m_model_(model_)
    {

    }

    OpticalGraph::~OpticalGraph()
    {

    }

    const String& OpticalGraph::getInstanceName() const
    {
        return m_instance_name_;
    }
        
    //-------------------------------------------------------------------------
    // Perform Datapath power optimization
    //-------------------------------------------------------------------------
    bool OpticalGraph::performPowerOpt(OpticalNode* node_, const WavelengthGroup& wavelengths_, unsigned int number_detectors_, double util_)
    {
        // Total number of iterations
        unsigned int number_iterations = 1250;
        // Maximum IL + ER
        double IL_ER_max = 10;  
        // Figure out the step size used in the sweep
        double step = (double) (IL_ER_max / sqrt(2 * number_iterations));

        // Assume it is possible
        bool possible = true;

        // Begin optical data path power optimization
        Log::printLine(getInstanceName() + " -> Beginning optical data path power optimization");
        
        // Trace the specified wavelengths
        OpticalWavelength* wavelength = traceWavelength(wavelengths_, node_);                

        // For each data path found in the wavelength
        const vector<OpticalDataPath>* data_paths = wavelength->getDataPaths();
        for (unsigned int i = 0; i < data_paths->size(); ++i)
        {
            const OpticalDataPath& data_path = data_paths->at(i);
            // Default to worst possible modulator
            double best_power = 1e99;
            double best_IL = IL_ER_max - step;
            double best_ER = step;
            
            // Perform power optimization for this data path
            Log::printLine(getInstanceName() + " -> Optimize data path - Laser = " + data_path.laser->getInstanceName()
                + ", Modulator = " + data_path.modulator->getInstanceName());

            if (data_path.modulator->canOptimizeLoss())
            {
                // Iterate over IL and ER to find optimal set of IL and ER
                for (double IL = step; IL < IL_ER_max; IL += step)
                {
                    for (double ER = step; ER <= (IL_ER_max - IL); ER += step)
                    {
                        // Ask the modulator to try this new ER and IL
                        bool success = data_path.modulator->setModulatorSpec(IL, ER);
                        // If the modulator was successful
                        if (success)
                        {
                            double laser_power = wavelength->getLaserPower(number_detectors_);
                            double modulator_power = data_path.modulator->getPower(util_);                         
                            double total_power = laser_power + modulator_power;
                            // If this is the new lowest power point
                            if (total_power < best_power)
                            {    
                                best_power = total_power;
                                best_IL = IL;
                                best_ER = ER;								
                            }
                        }
                    }
                }

                // Set IL and ER to the best ones we found
                bool success = data_path.modulator->setModulatorSpec(best_IL, best_ER);
                // If the best one we found was still not possible...
                possible = possible && success;
                
                // Print best IL and ER
                Log::printLine(getInstanceName() + " -> Best IL=" + (String) best_IL + ", Best ER=" + (String) best_ER + 
                    ", Best Laser/Mod Power=" + (String) best_power);
            }
            else
            {
                // Perform power optimization for this data path
                Log::printLine(getInstanceName() + " -> Data path not set to allow optimization");                
            }
        }

        // End optical data path power optimization
        Log::printLine(getInstanceName() + " -> End optical data path power optimization");

        delete wavelength;
        return possible;
    }


    //-------------------------------------------------------------------------
    // Trace wavelength(s), returning a wavelength data structure
    //-------------------------------------------------------------------------
    OpticalWavelength* OpticalGraph::traceWavelength(const WavelengthGroup& wavelengths_, OpticalNode* node_)
    {
        setTreeNum(getTreeNum() + 1);
        OpticalWavelength* wavelength = new OpticalWavelength("TraceWavelength", wavelengths_); 
        return traceWavelength(wavelength, node_, NULL, NULL, 0.0);
    }

    OpticalWavelength* OpticalGraph::traceWavelength(OpticalWavelength* wavelength_, OpticalNode* node_, OpticalLaser* laser_, OpticalModulator* modulator_, double loss_)
    {        
        // If the node has already been visited, don't do anything!
        if (node_->getVisitedNum() != getTreeNum())
        {
            // Set the new parity for this node
            node_->setVisitedNum(getTreeNum());
            
            // Calculate the loss of the current path
            double current_loss = loss_ + node_->getLoss();
            // Check if the current node is a laser, modulator or detector
            if(node_->getType() == OpticalNode::LASER)
            {
                // Set the laser lighting up the wavelength
                ASSERT(laser_ == NULL, "[Error] " + getInstanceName() + " -> Multiple " +
                    "Lasers lighting up the wavelength!");
                laser_ = (OpticalLaser*) node_;
            }
            else if (node_->getType() == OpticalNode::MODULATOR)
            {
                // Check that the path already lit up by a laser and there are no
                // modulators already driving data
                ASSERT(laser_ != NULL, "[Error] " + getInstanceName() + " -> Wavelength reaches a " +
                    "modulator (" + node_->getInstanceName() + ") prior to being lit up by a laser!");
                ASSERT(modulator_ == NULL, "[Error] " + getInstanceName() + " -> Two modulators are driving" +
                    " the same optical data path (" + node_->getInstanceName() + ")!");
                modulator_ = (OpticalModulator*) node_;
            }
            else if (node_->getType() == OpticalNode::DETECTOR)
            {
                // Check that the path is both lit up by a laser and there is
                // a modulator driving data
                ASSERT(laser_ != NULL, "[Error] " + getInstanceName() + " -> Wavelength reaches a " +
                    "detector (" + node_->getInstanceName() + ") prior to being lit up by a laser!");
                ASSERT(modulator_ != NULL, "[Error] " + getInstanceName() + " -> Wavelength reaches a " +
                    "detector (" + node_->getInstanceName() + ") prior to being driven by a modulator!");
                // Add a detector to the wavelength
                wavelength_->addDataPath(laser_, modulator_, (OpticalDetector*) node_, current_loss);
            }

            // Traverse downstream nodes to calculate the delay through each downstream path
            vector<OpticalNode*>* d_nodes = node_->getDownstreamNodes();
            bool trace_downstream = (node_->getType() != OpticalNode::DETECTOR);
            // Do special things when traversing filters
            if (node_->getType() == OpticalNode::FILTER)
            {
                OpticalFilter* filter_node = (OpticalFilter*) node_;
                if (filter_node->isDropped(wavelength_->getWavelengths()))
                    traceWavelength(wavelength_, filter_node->getDropPort(), laser_, modulator_, loss_ + filter_node->getDropLoss());

                // If the filter is not modeled as a complete drop, continue tracing downstream
                trace_downstream = !filter_node->getDropAll();
            }
            
            if (trace_downstream)
            {
                // Trace downstream nodes
                for (unsigned int i = 0; i < d_nodes->size(); ++i)
                    traceWavelength(wavelength_, d_nodes->at(i), laser_, modulator_, current_loss);
            }
        }
        return wavelength_;
    }

    //-------------------------------------------------------------------------
    OpticalGraph::OpticalGraph(const OpticalGraph& /* graph_ */)
    {
        // Disabled
    }
    
    OpticalModel* OpticalGraph::getModel()
    {
        return m_model_;
    }

    void OpticalGraph::setTreeNum(int tree_num_)
    {
        msTreeNum = tree_num_;
        return;
    }

    int OpticalGraph::getTreeNum()
    {
        return msTreeNum;
    }
    
} // namespace DSENT

