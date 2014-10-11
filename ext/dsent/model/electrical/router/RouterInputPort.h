#ifndef __DSENT_MODEL_ELECTRICAL_ROUTER_ROUTER_INPUT_PORT_H__
#define __DSENT_MODEL_ELECTRICAL_ROUTER_ROUTER_INPUT_PORT_H__

#include "util/CommonType.h"
#include "model/ElectricalModel.h"

namespace DSENT
{
    class RouterInputPort : public ElectricalModel
    {
        public:
            RouterInputPort(const String& instance_name_, const TechModel* tech_model_);
            virtual ~RouterInputPort();

        public:
            // Set a list of properties' name needed to construct model
            void initParameters();
            // Set a list of properties' name needed to construct model
            void initProperties();

            // Clone and return a new instance
            virtual RouterInputPort* clone() const;

        protected:
            // Build the model
            virtual void constructModel();
            virtual void propagateTransitionInfo();

    }; // class RouterInputPort
} // namespace DSENT

#endif // __DSENT_MODEL_ELECTRICAL_ROUTER_ROUTER_INPUT_PORT_H__

