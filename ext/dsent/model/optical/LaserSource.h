#ifndef __DSENT_MODEL_OPTICAL_LASERSOURCE_H__
#define __DSENT_MODEL_OPTICAL_LASERSOURCE_H__

#include "util/CommonType.h"
#include "model/OpticalModel.h"

namespace DSENT
{
    // A laser source that outputs some number of wavelengths. This laser cannot
    // be gated on/off at will and thus constitutes an NDD Power consumer
    class LaserSource : public OpticalModel
    {
        public:
            LaserSource(const String& instance_name_, const TechModel* tech_model_);
            virtual ~LaserSource();

        public:
            // Set a list of properties' name needed to construct model
            void initParameters();
            // Set a list of properties' name needed to construct model
            void initProperties();

        protected:
            // Build the model
            void constructModel();
            void updateModel();
            void evaluateModel();
            
    }; // class LaserSource
} // namespace DSENT

#endif // __DSENT_MODEL_OPTICAL_LASERSOURCE_H__

