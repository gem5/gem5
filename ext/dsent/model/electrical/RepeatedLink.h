#ifndef __DSENT_MODEL_ELECTRICAL_REPEATED_LINK_H__
#define __DSENT_MODEL_ELECTRICAL_REPEATED_LINK_H__

#include "util/CommonType.h"
#include "model/ElectricalModel.h"

namespace DSENT
{
    class StdCell;
    class ElectricalLoad;
    class ElectricalTimingTree;

    class RepeatedLink : public ElectricalModel
    {
        public:
            RepeatedLink(const String& instance_name_, const TechModel* tech_model_);
            virtual ~RepeatedLink();

        public:
            // Set a list of properties' name needed to construct model
            void initParameters();
            // Set a list of properties' name needed to construct model
            void initProperties();

            // Clone and return a new instance
            virtual RepeatedLink* clone() const;

        protected:
            // Build the model
            virtual void constructModel();
            virtual void updateModel();
            virtual void useModel();
            virtual void propagateTransitionInfo();

        private:
            // Use a repeater and a load to mimic a segment of the repeated link
            StdCell* m_repeater_;
            ElectricalLoad* m_repeater_load_;
            ElectricalTimingTree* m_timing_tree_;
    }; // class RepeatedLink
} // namespace DSENT

#endif // __DSENT_MODEL_ELECTRICAL_REPEATED_LINK_H__

