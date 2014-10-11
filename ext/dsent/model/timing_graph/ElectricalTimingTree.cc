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


#include "model/timing_graph/ElectricalTimingTree.h"

#include "model/ElectricalModel.h"
#include "model/timing_graph/ElectricalTimingNode.h"
#include "model/timing_graph/ElectricalDriver.h"
#include "model/timing_graph/ElectricalNet.h"

namespace DSENT
{
    // Initialize the next visited number to be one above the initial number
    // used by ElectricalTimingNode
    int ElectricalTimingTree::msTreeNum = ElectricalTimingNode::TIMING_NODE_INIT_VISITED_NUM + 1;

    ElectricalTimingTree::ElectricalTimingTree(const String& instance_name_, ElectricalModel* model_)
        : m_instance_name_(instance_name_), m_model_(model_)
    {
        //setTreeNum(1);
    }

    ElectricalTimingTree::~ElectricalTimingTree()
    {

    }

    const String& ElectricalTimingTree::getInstanceName() const
    {
        return m_instance_name_;
    }
        
    bool ElectricalTimingTree::performTimingOpt(ElectricalTimingNode* node_, double required_delay_)
    {
        // Extract the critical path from all timing paths branching out from the starting node
        double delay = performCritPathExtract(node_);
        double min_delay = delay;
                
        unsigned int iteration = 0;
        unsigned int crit_path_iteration = 0;
        unsigned int max_iterations = 8000;                     //TODO: make this not hard-coded
        unsigned int max_iterations_single_crit_path = 400;     //TODO: make this not hard-coded

        Log::printLine(getInstanceName() + " -> Beginning Incremental Timing Optimization");
        
        // Size up the nodes if timing is not met
        while(required_delay_ < delay)
        {
            Log::printLine(getInstanceName() + " -> Timing Optimization Iteration " + (String) iteration + 
                    ": Required delay = " + (String) required_delay_ + ", Delay = " +
                    (String) delay + ", Slack = " + (String) (required_delay_ - delay));

            ElectricalTimingNode* node_for_timing_opt = NULL;
            // Go into the less expensive critical path delay calculation
            // While the timing is not met for this critical path
            while (required_delay_ < delay)
            {
                // Find the node to optimize timing for, it would return a node to size up
                node_for_timing_opt = findNodeForTimingOpt(node_);
                // Give up if there are no appropriate nodes to size up or 
                // max number of iterations has been reached
                // Size up the chosen node if there is an appropriate node to size up
                if(node_for_timing_opt == NULL || iteration > max_iterations || crit_path_iteration > max_iterations_single_crit_path)
                    break;
                else
                    node_for_timing_opt->increaseDrivingStrength();

                // Re-evaluate the delay of the critical path
                delay = calculateCritPathDelay(node_);
                iteration++;
                crit_path_iteration++;
                Log::printLine(getInstanceName() + " -> Critical Path Slack: " + (String) (required_delay_ - delay));
            }            
            // Give up if there are no appropriate nodes to size up or
            // max number of iterations has been reached
            if (node_for_timing_opt == NULL || iteration > max_iterations || crit_path_iteration > max_iterations_single_crit_path)
                break;
                
            crit_path_iteration = 0;
            // Once the critical path timing is met, extract a new critical path from
            // all timing paths branching out from the starting node
            delay = performCritPathExtract(node_);
            min_delay = (min_delay > delay) ? delay : min_delay;
        }
        Log::printLine(getInstanceName() + " -> Timing Optimization Ended after Iteration: " + (String) iteration + 
                ": Required delay = " + (String) required_delay_ + ", Delay = " +
                (String) delay + ", Slack = " + (String) (required_delay_ - delay));            
                
        min_delay = (min_delay > delay) ? delay : min_delay;
        
        // Check if the timing meets the required delay
        if(required_delay_ < delay)
        {
            // Timing not met. Return false and print out a warning message
            const String& warning_msg = "[Warning] " + getInstanceName() + " -> Timing not met: Required delay = " + 
                (String) required_delay_ + ", Minimum Delay = " + (String) min_delay + ", Slack = " +
                (String) (required_delay_ - delay);
            Log::printLine(std::cerr, warning_msg);
            return false;
        }
        return true;
    }
    //-------------------------------------------------------------------------
    // Extract Crit Path Delay (and marks the crit path)
    //-------------------------------------------------------------------------
    double ElectricalTimingTree::performCritPathExtract(ElectricalTimingNode* node_)
    {
        setTreeNum(getTreeNum() + 1);
        return extractCritPathDelay(node_);
    }

    double ElectricalTimingTree::extractCritPathDelay(ElectricalTimingNode* node_)
    {
        //TODO: Replace with a stack data structure instead of recursion to prevent
        //stack overflow problems with long chains of logic (4000+ nodes) and/or better
        //performance. Nvm, stack data structure version seems to run much slower

        // If the node has already been visited, return the delay!
        if (node_->getVisitedNum() == getTreeNum())
            return node_->getDelayLeft();        
        // If the node has been marked as a false path, return 0.0
        else if (node_->getFalsePath())
            return 0.0;

        // Set the new parity for this node
        node_->setVisitedNum(getTreeNum());
        node_->setDelayLeft(0.0);

        double max_delay = 0;
        double current_delay = 0;

        // Traverse downstream nodes to calculate the delay through each downstream path
        vector<ElectricalTimingNode*>* d_nodes = node_->getDownstreamNodes();        
        for (unsigned int i = 0; i < d_nodes->size(); ++i)
        {
            current_delay = extractCritPathDelay(d_nodes->at(i));
            // Update the critical path
            if (current_delay > max_delay)
            {
                node_->setCritPath(i);
                max_delay = current_delay;
            }
        }        
        // Calculate the delay left from this node
        double delay_left = node_->calculateDelay() + max_delay;
        node_->setDelayLeft(delay_left);

        return delay_left;

    }

    double ElectricalTimingTree::calculateCritPathDelay(ElectricalTimingNode* node_) const
    {
        // Simplest case where theres nothing to optimize
        if (node_ == NULL)
            return 0.0;

        double delay = 0.0;
        int crit_path = 0;

        // Traverse the critical path and sum up delays
        while (crit_path >= 0)
        {
            delay += node_->calculateDelay();
            //Move on to the next node in the critical path
            crit_path = node_->getCritPath();
            if (crit_path >= 0)
                node_ = node_->getDownstreamNodes()->at(crit_path);
        }
        return delay;
    }
    //-------------------------------------------------------------------------

    //-------------------------------------------------------------------------
    // Find Worst Slew Helpers
    //-------------------------------------------------------------------------
    ElectricalTimingNode* ElectricalTimingTree::findNodeForTimingOpt(ElectricalTimingNode* node_) const
    {
        // Simplest case where theres nothing to optimize
        if (node_ == NULL)
            return NULL;

        double max_transition_ratio = -10.0;
        double current_transition_ratio = 0.0;
        double previous_transition = 1e3 * node_->getTotalDownstreamCap();
        double current_transition = 0.0;
        ElectricalTimingNode* worst = NULL;        
        int crit_path = 0;

        // Find the node with the highest max_transition_ratio to return
        while (crit_path >= 0)
        {
            current_transition = node_->calculateDelay();

            //If the node is not yet at max size, it is a potential choice for size up
            if (!node_->hasMaxDrivingStrength())
            {            
                current_transition_ratio = current_transition / previous_transition;                

                if (current_transition_ratio > max_transition_ratio)
                {
                    worst = node_;
                    max_transition_ratio = current_transition_ratio;
                }
            }

            if (node_->isDriver())
                previous_transition = 0.0;            
            previous_transition += current_transition;

            //Move on to the next node in the critical path
            crit_path = node_->getCritPath();

            if (crit_path >= 0)
                node_ = node_->getDownstreamNodes()->at(crit_path);
        }

        return worst;
    }
    //-------------------------------------------------------------------------

    double ElectricalTimingTree::calculateNodeTransition(ElectricalTimingNode* node_) const
    {
        return node_->calculateTransition();
    }

    ElectricalTimingTree::ElectricalTimingTree(const ElectricalTimingTree& /* graph_ */)
    {
        // Disabled
    }

    ElectricalModel* ElectricalTimingTree::getModel()
    {
        return m_model_;
    }

    void ElectricalTimingTree::setTreeNum(int tree_num_)
    {
        msTreeNum = tree_num_;
        return;
    }

    int ElectricalTimingTree::getTreeNum()
    {
        return msTreeNum;
    }

} // namespace DSENT

