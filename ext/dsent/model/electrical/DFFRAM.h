#ifndef __DSENT_MODEL_ELECTRICAL_DFFRAM_H__
#define __DSENT_MODEL_ELECTRICAL_DFFRAM_H__

#include "util/CommonType.h"
#include "model/ElectricalModel.h"

namespace DSENT
{
    class DFFRAM : public ElectricalModel
    {
        public:
            DFFRAM(const String& instance_name_, const TechModel* tech_model_);
            virtual ~DFFRAM();

        public:
            // Set a list of properties' name needed to construct model
            void initParameters();
            // Set a list of properties' name needed to construct model
            void initProperties();

            // Clone and return a new instance
            virtual DFFRAM* clone() const;

        protected:
            // Build the model
            virtual void constructModel();
            virtual void propagateTransitionInfo();

    }; // class DFFRAM
} // namespace DSENT

#endif // __DSENT_MODEL_ELECTRICAL_BUFFER_H__

