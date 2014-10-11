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

