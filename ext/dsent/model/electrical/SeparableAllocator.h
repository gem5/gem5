#ifndef __DSENT_MODEL_ELECTRICAL_SEPARABLE_ALLOCATOR_H__
#define __DSENT_MODEL_ELECTRICAL_SEPARABLE_ALLOCATOR_H__

#include "util/CommonType.h"
#include "model/ElectricalModel.h"

namespace DSENT
{
    class SeparableAllocator : public ElectricalModel
    {
        public:
            SeparableAllocator(const String& instance_name_, const TechModel* tech_model_);
            virtual ~SeparableAllocator();

        public:
            // Set a list of properties' name needed to construct model
            void initParameters();
            // Set a list of properties' name needed to construct model
            void initProperties();

            // Clone and return a new instance
            virtual SeparableAllocator* clone() const;

        protected:
            // Build the model
            virtual void constructModel();
            virtual void updateModel();

    }; // class SeparableAllocator
} // namespace DSENT

#endif // __DSENT_MODEL_ELECTRICAL_SEPARABLE_ALLOCATOR_H__

