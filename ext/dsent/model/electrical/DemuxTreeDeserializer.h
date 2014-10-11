#ifndef __DSENT_MODEL_ELECTRICAL_MULTIPHASEDESERIALIZER_H__
#define __DSENT_MODEL_ELECTRICAL_MULTIPHASEDESERIALIZER_H__

#include "util/CommonType.h"
#include "model/ElectricalModel.h"

namespace DSENT
{
    class DemuxTreeDeserializer : public ElectricalModel
    {
        public:
            DemuxTreeDeserializer(const String& instance_name_, const TechModel* tech_model_);
            virtual ~DemuxTreeDeserializer();

        public:
            // Set a list of properties' name needed to construct model
            void initParameters();
            // Set a list of properties' name needed to construct model
            void initProperties();

            // Clone and return a new instance
            virtual DemuxTreeDeserializer* clone() const;

        protected:
            // Build the model
            virtual void constructModel();
            virtual void propagateTransitionInfo();

    }; // class DemuxTreeDeserializer
} // namespace DSENT

#endif // __DSENT_MODEL_ELECTRICAL_MULTIPHASEDESERIALIZER_H__

