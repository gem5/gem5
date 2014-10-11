#ifndef __DSENT_MODEL_ELECTRICAL_RIPPLE_ADDER_H__
#define __DSENT_MODEL_ELECTRICAL_RIPPLE_ADDER_H__

#include "util/CommonType.h"
#include "model/ElectricalModel.h"

namespace DSENT
{
    class RippleAdder : public ElectricalModel
    {
        public:
            RippleAdder(const String& instance_name_, const TechModel* tech_model_);
            virtual ~RippleAdder();

        public:
            // Set a list of properties' name needed to construct model
            void initParameters();
            // Set a list of properties' name needed to construct model
            void initProperties();

        protected:
            // Build the model
            virtual void constructModel();
            virtual void propagateTransitionInfo();

    }; // class RippleAdder
} // namespace DSENT

#endif // __DSENT_MODEL_ELECTRICAL_TESTMODEL_H__

