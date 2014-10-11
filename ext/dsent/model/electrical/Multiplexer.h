#ifndef __DSENT_MODEL_ELECTRICAL_MULTIPLEXER_H__
#define __DSENT_MODEL_ELECTRICAL_MULTIPLEXER_H__

#include "util/CommonType.h"
#include "model/ElectricalModel.h"

namespace DSENT
{
    // A model of an N-to-1 multiplexer
    class Multiplexer : public ElectricalModel
    {
        public:
            Multiplexer(const String& instance_name_, const TechModel* tech_model_);
            virtual ~Multiplexer();

        public:
            // Set a list of properties' name needed to construct model
            void initParameters();
            // Set a list of properties' name needed to construct model
            void initProperties();

            // Clone and return a new instance
            virtual Multiplexer* clone() const;

        protected:
            // Build the model
            virtual void constructModel();
            virtual void propagateTransitionInfo();

    }; // class Multiplexer
} // namespace DSENT

#endif // __DSENT_MODEL_ELECTRICAL_MULTIPLEXER_H__

