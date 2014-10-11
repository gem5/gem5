#ifndef __DSENT_MODEL_OPTICAL_GATEDLASERSOURCE_H__
#define __DSENT_MODEL_OPTICAL_GATEDLASERSOURCE_H__

#include "util/CommonType.h"
#include "model/OpticalModel.h"

namespace DSENT
{
    // A laser source that outputs some number of wavelengths. This laser
    // full on/off power gating, thus all power are event-based energies
    class GatedLaserSource : public OpticalModel
    {
        public:
            GatedLaserSource(const String& instance_name_, const TechModel* tech_model_);
            virtual ~GatedLaserSource();

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
            
    }; // class GatedLaserSource
} // namespace DSENT

#endif // __DSENT_MODEL_OPTICAL_GATEDLASERSOURCE_H__

