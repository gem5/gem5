#ifndef __DSENT_MODEL_ELECTRICAL_ROUTER_ROUTER_SWITCH_ALLOCATOR_H__
#define __DSENT_MODEL_ELECTRICAL_ROUTER_ROUTER_SWITCH_ALLOCATOR_H__

#include "util/CommonType.h"
#include "model/ElectricalModel.h"

namespace DSENT
{
    class RouterSwitchAllocator : public ElectricalModel
    {
        public:
            RouterSwitchAllocator(const String& instance_name_, const TechModel* tech_model_);
            virtual ~RouterSwitchAllocator();

        public:
            // Set a list of properties' name needed to construct model
            void initParameters();
            // Set a list of properties' name needed to construct model
            void initProperties();

            // Clone and return a new instance
            virtual RouterSwitchAllocator* clone() const;

        protected:
            // Build the model
            virtual void constructModel();
            virtual void propagateTransitionInfo();

    }; // RouterSwitchAllocator
} // namespace DSENT

#endif // __DSENT_MODEL_ELECTRICAL_ROUTER_ROUTER_SWITCH_ALLOCATOR_H__

