#ifndef __DSENT_MODEL_ELECTRICAL_TIMING_TREE_H__
#define __DSENT_MODEL_ELECTRICAL_TIMING_TREE_H__

#include <vector>

#include "util/CommonType.h"
#include "model/timing_graph/ElectricalTimingNode.h"

namespace DSENT
{
    using std::vector;
    
    class ElectricalDriver;

    class ElectricalTimingTree
    {
        public:
            // The visited number for the next timing run. This needs to be
            // global because several timing trees may be created to evaluate
            // a single timing path, causing problems
            static int msTreeNum;
        
        public:
            // Construct timing tree that watches over model_
            ElectricalTimingTree(const String& instance_name_, ElectricalModel* model_);
            ~ElectricalTimingTree();
            
        public:
            // Get tree name
            const String& getInstanceName() const;

            // A wrapper for extractCritPathDelay
            // Update the tree num before do extract critical path delay recursively
            double performCritPathExtract(ElectricalTimingNode* node_);
            // Calculate the delay of the marked critical path from a starting node
            double calculateCritPathDelay(ElectricalTimingNode* node_) const;            
            // Calculate the transition at a node
            double calculateNodeTransition(ElectricalTimingNode* node_) const;
            // Returns the optimal node to optimize timing (by sizing up) in the critical
            // path to reduce critical path delay
            ElectricalTimingNode* findNodeForTimingOpt(ElectricalTimingNode* node_) const;
            // Perform incremental timing optimization to guarantee that all timing paths from a
            // starting node meets a required delay
            // Return false if the timing optimization fails to meet the required delay
            bool performTimingOpt(ElectricalTimingNode* node_, double required_delay_);
            
            // Return the model
            ElectricalModel* getModel();

        private:
            // Disable the use of copy constructor
            ElectricalTimingTree(const ElectricalTimingTree& graph_);
        
            // Recursively calculate delay from a starting node, finding and marking the 
            // critical path along the way and returns the delay of the critical path
            double extractCritPathDelay(ElectricalTimingNode* node_);            

        public:
            // Set the sequence number of the timing tree
            static void setTreeNum(int tree_num_);
            static int getTreeNum();
            
        private:
            // Name of the timing tree
            const String m_instance_name_;
            // A pointer to the model that contains this node
            ElectricalModel* m_model_;

    }; // class ElectricalTimingTree
} // namespace DSENT

#endif // __DSENT_MODEL_ELECTRICAL_TIMING_TREE_H__

