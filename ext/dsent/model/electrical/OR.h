#ifndef __DSENT_MODEL_ELECTRICAL_OR_H__
#define __DSENT_MODEL_ELECTRICAL_OR_H__

#include "util/CommonType.h"
#include "model/ElectricalModel.h"

namespace DSENT
{
    /**
     * \brief A class that implements a n-input OR gate
     */
    class OR : public ElectricalModel
    {
        public:
            OR(const String& instance_name_, const TechModel* tech_model_);
            virtual ~OR();

        public:
            // Set a list of properties' name needed to construct model
            void initParameters();
            // Set a list of properties' name needed to construct model
            void initProperties();

            // Clone and return a new instance
            virtual OR* clone() const;

        protected:
            // Build the model
            virtual void constructModel();
            virtual void propagateTransitionInfo();

    }; // class OR
} // namespace DSENT

#endif // __DSENT_MODEL_ELECTRICAL_OR_H__

